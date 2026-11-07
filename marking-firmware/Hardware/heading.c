#include "stm32f10x.h"
#include <math.h>
#include "mpu6050.h"
#include "heading.h"
#include "calib.h"
#include "StepMotor_New.h"

/* timer_counter 在 TIM1 更新中断里每 1ms 加一（system.c 定义）。
 * 用它而不是自己再开一个定时器：多一个定时器就多一处要维护的初值。 */
extern volatile uint32_t timer_counter;

#define HDG_PI              3.14159265f

/* Z 轴角速度的符号。
 *
 * MPU6050 是右手系：模块平放时 Z 轴朝上，绕 Z 轴的正方向是从上方看
 * 逆时针。而本工程约定「车头向右偏为正」，那是顺时针 —— 两者相反。
 *
 * 不处理的话后果很隐蔽：车头往右偏 2 度，闭环把它当成"往左偏 2 度"，
 * 于是继续往右打，下一次偏更多，再往右打…… 车会画出一个越来越急的
 * 右转圆圈，而代码每一步"看起来"都对。
 *
 * 现场核对办法：手扶着车原地向右转 30 度，上位机上的航向应该显示 +30。
 * 显示 -30 就说明装反了，但这个宏本身不用改 —— 传感器可能被倒装
 * （Z 轴朝下），那种情况符号会自然反过来，改宏反而错上加错。
 * 只有在"确认装法正确、读数符号仍然相反"时才需要动它。 */
#ifndef HDG_Z_SIGN
#define HDG_Z_SIGN          (-1.0f)
#endif
#define HDG_TRIM_PERIOD_MS  100     /* 纠偏动作的最小间隔。10ms 采一次样，
                                     * 但 10ms 改一次目标脉冲没有意义 ——
                                     * 脉冲还没走完，改了也看不出差别，
                                     * 只会让四路目标的更新过于密集。 */

static float        hdg_abs_deg;        /* 上电以来的累计航向，右正 */
static float        hdg_ref_deg;        /* 段起点航向 */
static float        hdg_bias_dps;       /* 零偏估计，dps */
static float        hdg_rate_dps;       /* 最近一次角速度，已去零偏 */
static HeadingState hdg_state = HDG_OFF;
static uint8_t      hdg_enabled;
/* 用户/上层的意图，和"此刻实际生效"分开记。
 * 掉线重连会重新走一遍标定，标定完不能自作主张把闭环打开 ——
 * 用户可能刚刚用 CMD_GYRO 0 关掉它（比如发现传感器坏了，想退回纯视觉）。
 * 首次上电默认是"想开"：装了陀螺仪就是为了让它管直线。 */
static uint8_t      hdg_want_enabled = 1;

static uint32_t hdg_last_sample_ms;
static uint32_t hdg_last_trim_ms;
static uint32_t hdg_last_retry_ms;

/* 轮距，运行时可改。初值取 calib.h 的标定值。 */
static float hdg_track_width = CALIB_TRACK_WIDTH_MM;

/* 标定过程 */
static uint8_t  cal_n;                  /* 已采样本数，最多 100，uint8 够 */
static int32_t  cal_sum;
static int16_t  cal_min;
static int16_t  cal_max;

/* 主循环被卡住、丢了积分的次数。不报警，但状态帧带出去，
 * 现场看到它在涨，就知道有别的地方在阻塞主循环。 */
static uint16_t hdg_dropped;

/* 清空标定缓冲，进入采集状态。掉线重连、标定失败重试、手动重标都走这里。 */
static void heading_cal_start(void)
{
    cal_n   = 0;
    cal_sum = 0;
    cal_min = 32767;
    cal_max = -32768;
    hdg_state = HDG_CALIBRATING;
}

/* --------------------------------------------------------------------------
 * 初始化
 * -------------------------------------------------------------------------- */
uint8_t Heading_Init(void)
{
    uint8_t ret;

    hdg_abs_deg = 0.0f;
    hdg_ref_deg = 0.0f;
    hdg_bias_dps = 0.0f;
    hdg_rate_dps = 0.0f;
    hdg_enabled = 0;
    hdg_dropped = 0;

    hdg_last_sample_ms = timer_counter;
    hdg_last_trim_ms   = 0;
    hdg_last_retry_ms  = timer_counter;

    ret = MPU6050_Init();
    if (ret != MPU_OK) {
        hdg_state = HDG_OFF;
        return ret;
    }

    heading_cal_start();

    return MPU_OK;
}

/* --------------------------------------------------------------------------
 * 零偏标定
 *
 * 判断"车是不是在动"用 (max - min)，不用方差：
 *   - 方差要存全部样本或者用 sum of squares，100 个 32767 量级的平方和
 *     会溢出 32 位，得退化到 long long（ARMCC 上是软件实现，慢且占地）
 *   - 极差对"有一颗样本被干扰"敏感，但标定本来就要求车静止，
 *     真被干扰了重来一次就行，代价很低
 *   - 更重要的：极差能区分"整体偏了"和"在晃"。零偏是整体偏（所有样本
 *     一起偏），晃动才是极差大。所以极差正是我们要判断的那个量
 * -------------------------------------------------------------------------- */
static void heading_cal_sample(int16_t raw)
{
    /* ±32767 只在传感器饱和时出现，不是真实读数，不参与标定 */
    if (raw > 32700 || raw < -32700) {
        return;
    }

    cal_sum += raw;
    if (raw < cal_min) cal_min = raw;
    if (raw > cal_max) cal_max = raw;
    cal_n++;

    if (cal_n < HDG_CAL_SAMPLES) {
        return;
    }

    /* 必须用 32 位比较：cal_max - cal_min 最大能到 65400，先转 int16
     * 会绕成负数，于是"车在以 300dps 疯转"被判成"静止"，
     * 一个完全错误的零偏就这么被写进去了。 */
    if (((int32_t)cal_max - (int32_t)cal_min) > HDG_CAL_SPREAD_LSB) {
        /* 车在动（或者被搬着）。这一次不算，回 OFF 等 2 秒重来。
         * 不拿"采到一半"的数据凑数：混了运动的零偏会让之后每一段
         * 直线都往同一个方向偏。 */
        hdg_state = HDG_OFF;
        return;
    }

    /* 必须乘 HDG_Z_SIGN：cal_sum 累加的是传感器的原始读数，而运行时
     * 的 dps 是"原始读数 × HDG_Z_SIGN"（见 heading_update）。两边不统一
     * 的话，静止时 rate = -bias - bias = -2·bias —— 零偏 1dps 就是每秒
     * 2 度的漂移，5.3 米一段走 24 秒差 48 度。同时静止判据
     * fabsf(dps - bias) 恒等于 2|bias|，超过 0.6dps 就再也进不了"静止"
     * 分支，零偏连自愈的路都没有（MPU6050 未校准 ZRO 常在 1~3dps）。 */
    hdg_bias_dps = ((float)cal_sum / (float)HDG_CAL_SAMPLES) / MPU_GYRO_LSB_PER_DPS
                 * HDG_Z_SIGN;
    hdg_abs_deg = 0.0f;
    hdg_ref_deg = 0.0f;

    /* 标定完成后按"之前的意图"决定开不开：走到这一步说明传感器是好的、
     * 车是静的、零偏是准的，默认（want = 1）就该打开。
     * 但之前被明确关掉过的，这里不能偷偷再打开。 */
    hdg_enabled = hdg_want_enabled;
    hdg_state = hdg_enabled ? HDG_CLOSED : HDG_READY;
}

/* --------------------------------------------------------------------------
 * 采样 + 积分
 * -------------------------------------------------------------------------- */
static void heading_update(void)
{
    int16_t raw = 0;
    float   dps;
    uint8_t still;

    if (MPU6050_ReadGyroZ(&raw) != MPU_OK) {
        /* 读失败：这一次不积分，也不更新零偏。
         * MPU6050_ReadGyro 内部已经把 online 置 0 并做过一次总线恢复，
         * 重连由 Heading_Poll 在那之后接手 —— 这里不重复做。 */
        hdg_state = HDG_FAULT;
        return;
    }

    /* 换算成 dps，同时统一到本工程的符号约定（右正） */
    dps = ((float)raw / MPU_GYRO_LSB_PER_DPS) * HDG_Z_SIGN;

    /* 静止判定：电机没在跑 + 角速度和零偏估计接近。
     * 两个条件缺一不可 ——
     *   只看角速度：车慢慢转的时候（画线修正就是慢转）会被误判成静止，
     *   把真实的转动吸收进零偏里，角度就停了。
     *   只看电机：电机停着但人推车，会把人的转动吃成零偏，
     *   之后零偏就错了，整条线都歪。 */
    still = (uint8_t)(!StepMotor_IsAnyRunning()
                   && fabsf(dps - hdg_bias_dps) < HDG_STILL_DPS);

    if (still) {
        /* 静止时不积分，只让零偏慢慢跟上温漂。
         * 单步限幅 0.01dps：温漂是分钟级的，1dps/s 的跟踪速度远远够用；
         * 而一次 I2C 干扰造成的假尖峰会被这个限幅挡在外面，不会把零偏拉走。 */
        float step = (dps - hdg_bias_dps) * HDG_BIAS_ALPHA;
        if (step >  0.01f) step =  0.01f;
        if (step < -0.01f) step = -0.01f;
        hdg_bias_dps += step;
        hdg_rate_dps = 0.0f;
        return;
    }

    hdg_rate_dps = dps - hdg_bias_dps;
}

/* 积分单独一个函数，只因为 dt 必须用"两次采样之间的真实间隔"。
 * 如果按固定的 10ms 去乘，主循环抖一下角度就系统性偏一点，
 * 一次走十秒下来能差好几度 —— 而这正是我们要修的那个症状。 */
static void heading_integrate(uint32_t dt_ms)
{
    if (hdg_state != HDG_READY && hdg_state != HDG_CLOSED) {
        return;
    }
    if (dt_ms == 0) {
        return;
    }
    if (dt_ms > HDG_DT_MAX_MS) {
        hdg_dropped++;
        return;
    }

    hdg_abs_deg += hdg_rate_dps * ((float)dt_ms * 0.001f);
}

/* --------------------------------------------------------------------------
 * 主循环入口
 * -------------------------------------------------------------------------- */
void Heading_Poll(void)
{
    uint32_t now = timer_counter;
    uint32_t dt_ms;
    int16_t  raw;

    /* --- 1. 传感器不在线：车停下时试着救回来 --- */
    if (!MPU6050_IsOnline()) {
        if ((uint32_t)(now - hdg_last_retry_ms) < HDG_RETRY_MS) {
            return;
        }
        hdg_last_retry_ms = now;

        /* 只在车停着的时候重连。重连要拉 9 个时钟做总线恢复、再重写
         * 6 个配置寄存器，运动中做会打乱采样节拍；而且运动中掉线
         * 本来就该先停车（状态机会因为超时回 WAIT）。 */
        if (StepMotor_IsAnyRunning()) {
            hdg_state = HDG_FAULT;
            return;
        }

        if (MPU6050_Recover() != MPU_OK) {
            hdg_state = HDG_FAULT;
            return;
        }

        /* 救回来了，零偏必须重标：掉线期间温度和环境可能已经变了，
         * 而断开那段时间转了多少度也无从补起 —— 只能以现在为零点。 */
        heading_cal_start();
        return;
    }

    /* --- 2. 在线但没标定成功：上电时车恰好在动，等它停下自动重来 --- */
    if (hdg_state == HDG_OFF) {
        if (!StepMotor_IsAnyRunning()
            && (uint32_t)(now - hdg_last_retry_ms) >= HDG_RETRY_MS) {
            hdg_last_retry_ms = now;
            heading_cal_start();
        }
        return;
    }

    /* --- 3. 采样节流 --- */
    dt_ms = (uint32_t)(now - hdg_last_sample_ms);
    if (dt_ms < HDG_SAMPLE_MS) {
        return;
    }
    hdg_last_sample_ms = now;

    /* --- 标定中 --- */
    if (hdg_state == HDG_CALIBRATING) {
        if (MPU6050_ReadGyroZ(&raw) != MPU_OK) {
            hdg_state = HDG_FAULT;
            return;
        }
        heading_cal_sample(raw);
        return;
    }

    /* --- 正常运行 --- */
    heading_update();
    heading_integrate(dt_ms);
}

/* --------------------------------------------------------------------------
 * 查询 / 开关
 * -------------------------------------------------------------------------- */
void Heading_MarkReference(void)
{
    hdg_ref_deg = hdg_abs_deg;
}

float Heading_GetDeg(void)
{
    if (hdg_state != HDG_READY && hdg_state != HDG_CLOSED) {
        return 0.0f;
    }
    return hdg_abs_deg - hdg_ref_deg;
}

float Heading_GetAbsoluteDeg(void)
{
    if (hdg_state != HDG_READY && hdg_state != HDG_CLOSED) {
        return 0.0f;
    }
    return hdg_abs_deg;
}

float Heading_RateDps(void)
{
    return hdg_rate_dps;
}

HeadingState Heading_GetState(void)
{
    return hdg_state;
}

uint8_t Heading_IsReady(void)
{
    return (uint8_t)(hdg_state == HDG_READY || hdg_state == HDG_CLOSED);
}

uint8_t Heading_IsEnabled(void)
{
    return hdg_enabled;
}

uint16_t Heading_DroppedSamples(void)
{
    return hdg_dropped;
}

uint8_t Heading_SetEnabled(uint8_t en)
{
    if (en) {
        /* 记住意图，即使现在还不能生效 —— 传感器接上、标定完之后要自动恢复 */
        hdg_want_enabled = 1;

        /* 没标定就开，等于拿一个可能是任意值的零偏去纠偏。
         * 车会朝着一个固定方向一直偏，比不开还糟。 */
        if (hdg_state != HDG_READY && hdg_state != HDG_CLOSED) {
            hdg_enabled = 0;
            return 0;
        }
        hdg_enabled = 1;
        hdg_state = HDG_CLOSED;
        /* 开的时候把当前航向当起点，不然会把之前累积的偏差一次性纠回来 */
        hdg_ref_deg = hdg_abs_deg;
        return 1;
    }

    hdg_want_enabled = 0;
    hdg_enabled = 0;
    if (hdg_state == HDG_CLOSED) {
        hdg_state = HDG_READY;
    }
    return 0;
}

uint8_t Heading_Recalibrate(void)
{
    if (!MPU6050_IsOnline()) {
        return 2;       /* 传感器都不在线，标零没有意义 */
    }
    if (StepMotor_IsAnyRunning()) {
        return 1;       /* 车在动，采到的"零偏"里混着真实的转动 */
    }

    heading_cal_start();

    return 0;
}

/* --------------------------------------------------------------------------
 * 控制律
 * -------------------------------------------------------------------------- */
float Heading_TrimDeltaMm(void)
{
    float err;
    float delta;
    uint32_t now;

    if (hdg_state != HDG_CLOSED || !hdg_enabled) {
        return 0.0f;
    }

    now = timer_counter;
    if ((uint32_t)(now - hdg_last_trim_ms) < HDG_TRIM_PERIOD_MS) {
        return 0.0f;
    }

    err = hdg_abs_deg - hdg_ref_deg;    /* 正 = 车头偏右 */

    if (fabsf(err) < HDG_DEADBAND_DEG) {
        return 0.0f;
    }

    /* 几何补偿：车要转 -err 度，左右轮要走出的行程差是
     *     |Δd| = |err| (弧度) × 轮距
     * 符号：err > 0（偏右）要往左打，也就是右侧轮多走、左侧轮少走，
     * 而 Motion_Trim 的约定是"正 = 车头向右"，所以取负。
     *
     * 不额外乘增益的原因：这是一个"由几何唯一确定"的量，不是凭经验调的
     * 参数。1.0 就是"恰好转回来"。乘 2 会冲过头，然后反向再冲，
     * 车头就开始左右摆 —— 表现上像是"陀螺仪不准"，其实是控制律错了。 */
    delta = -err * (HDG_PI / 180.0f) * hdg_track_width * HDG_KP;

    if (delta >  HDG_TRIM_ONCE_MAX_MM) delta =  HDG_TRIM_ONCE_MAX_MM;
    if (delta < -HDG_TRIM_ONCE_MAX_MM) delta = -HDG_TRIM_ONCE_MAX_MM;

    hdg_last_trim_ms = now;

    return delta;
}

float Heading_EstimateTrackWidth(float diff_mm, float turned_deg)
{
    float rad;
    float width;

    /* 转得太少时反算出来的轮距没有意义：角度测量误差被放大成倍。
     * 1 度的时候，±0.2 度的读取误差就是 ±20% 的轮距误差。 */
    if (turned_deg < 1.0f || turned_deg > 180.0f) {
        return 0.0f;
    }
    if (diff_mm <= 0.0f) {
        return 0.0f;
    }

    rad = turned_deg * (HDG_PI / 180.0f);
    width = diff_mm / rad;

    /* 落在这个区间之外，说明测量本身有问题（打滑、只走了一个轮子的一半、
     * 角度读错符号），不要往 calib 里写一个荒唐值。 */
    if (width < 150.0f || width > 1500.0f) {
        return 0.0f;
    }

    return width;
}

uint8_t Heading_SetTrackWidth(float mm)
{
    if (mm < 150.0f || mm > 1500.0f) {
        return 0;       /* 离谱值不写进去，宁可继续用旧的 */
    }
    hdg_track_width = mm;
    return 1;
}

float Heading_GetTrackWidth(void)
{
    return hdg_track_width;
}
