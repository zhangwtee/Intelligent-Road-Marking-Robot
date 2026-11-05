#include "motion.h"
#include "calib.h"
#include "StepMotor_New.h"

#define MOTION_PI      3.14159265f
#define MOTION_PI_180  (3.14159265f / 180.0f)

/* 当前底盘是否具备横移能力，来源是 chassis.h（由 Target 的 Define 决定）。
 * 这里只留一个别名，方便老代码继续引用 MOTION_HAS_MECANUM。 */
#define MOTION_HAS_MECANUM   CHASSIS_HAS_MECANUM

/* 单次纠偏的限幅（毫米，指整车中心再多走/少走多少）。
 * 100mm 一段的节拍下，一次纠 20mm 相当于把左右轮差开到 20mm，
 * 折算成航向约 2~4 度 —— 已经是很猛的修正了。再大就是原地画弧。
 * 限幅的意义不在于"温和"，在于防止上位机算飞了（比如线跳到了画面外）
 * 时把一段 5.3 米的直线改成两个大圈。 */
#ifndef MOTION_TRIM_MAX_MM
#define MOTION_TRIM_MAX_MM   CHASSIS_TRIM_MAX_MM
#endif

/* 纠偏方向的符号。+1 = delta 为正时车头向右。
 * 相机左右装反、或者某一侧电机接线反了，只改这个宏，别去翻协议层。 */
#ifndef MOTION_TRIM_SIGN
#define MOTION_TRIM_SIGN     1
#endif

/* ---------- 预计算系数 ----------
 * F103 没有 FPU，float 乘除都是软件模拟。运行时的除法一次要几十微秒，
 * 放在状态机里会造成脉冲启停延迟。所以这里全部在 Init 时算完，
 * 之后只剩乘法。 */
static float   s_pulse_per_mm  = 0.0f;
static float   s_pulse_per_deg = 0.0f;
static float   s_k[4]          = {1.0f, 1.0f, 1.0f, 1.0f};
static uint8_t s_ready         = 0;

/* 本段累计纠偏（脉冲，带符号）。每次开新段清零。
 * 只在 Motion_Trim 和开段函数里读写，都是主循环上下文，不需要临界区。 */
static int32_t s_trim_accum    = 0;

void Motion_Init(void)
{
    /* 轮径补偿系数由车型决定。
     *
     * 麦轮：四轮各自独立的 K1~K4。四个轮子都能横滑，走快了走慢了不会
     *       互相顶，逐轮标定是有意义的。
     * 差速：左组只用 K1、右组只用 K2（见 chassis.h）。普通轮不能横滑，
     *       组内只要有一点行程差，多出来的那部分只能靠某个轮子打滑消化，
     *       而哪个先打滑不确定 —— 车身就会莫名其妙扭一下。宁可整组
     *       统一差 2%，也不要组内差 0.5%。 */
    s_k[0] = CHASSIS_K_LEFT_FRONT;
    s_k[1] = CHASSIS_K_RIGHT_FRONT;
    s_k[2] = CHASSIS_K_LEFT_REAR;
    s_k[3] = CHASSIS_K_RIGHT_REAR;

    /* 每毫米需要多少脉冲 = 每圈脉冲 / 轮子周长 */
    s_pulse_per_mm = (float)CALIB_STEPS_PER_REV / (MOTION_PI * CALIB_WHEEL_DIA_MM);

    /* 原地转 1 度时，每个轮子走过的弧长 = 半径 x 弧度，
     * 再乘每毫米脉冲数。 */
    s_pulse_per_deg = MOTION_PI_180 * CALIB_TURN_RADIUS_MM * s_pulse_per_mm;

    s_ready = 1;
}

uint8_t Motion_IsReady(void)
{
    return s_ready;
}

/* ---------- 纯换算 ---------- */

uint32_t Motion_MmToPulse(float mm)
{
    if (mm <= 0.0f) {
        return 0;
    }
    return (uint32_t)(mm * s_pulse_per_mm + 0.5f);
}

uint32_t Motion_DegToPulse(float deg)
{
    if (deg <= 0.0f) {
        return 0;
    }
    return (uint32_t)(deg * s_pulse_per_deg + 0.5f);
}

float Motion_PulseToMm(uint32_t pulse)
{
    if (s_pulse_per_mm <= 0.0f) {
        return 0.0f;
    }
    return (float)pulse / s_pulse_per_mm;
}

float Motion_PulseToDeg(uint32_t pulse)
{
    if (s_pulse_per_deg <= 0.0f) {
        return 0.0f;
    }
    return (float)pulse / s_pulse_per_deg;
}

/* ---------- 四轮驱动（内部） ----------
 * pulse 用 uint32_t 直接传给 SetPulse，绕开 GoForward 的 uint16_t 上限，
 * 这样超过 12.9 米的行程也不会被截断。
 * 至少要给 1 个脉冲：SetPulse(0) 在中断里会被当成"已停止"，
 * 但驱动器的使能状态依赖首脉冲，0 会让它保持上一次的方向语义。 */
static void DriveAll(uint8_t forward, uint32_t p1, uint32_t p2,
                     uint32_t p3, uint32_t p4)
{
    if (p1 == 0) { p1 = 1; }
    if (p2 == 0) { p2 = 1; }
    if (p3 == 0) { p3 = 1; }
    if (p4 == 0) { p4 = 1; }

    /* 开新段 = 重新对齐。上一段攒下的左右差在这一刻归零，累计纠偏量也
     * 跟着归零 —— 这正是"每段重设而不是全程一次"能把误差锁在段内的原因。 */
    s_trim_accum = 0;

    /* 上一段可能是原地转，差速车在转弯时把速度压到了 CHASSIS_TURN_ARR。
     * 走直线必须回到用户档位，否则车会一直慢下去而界面上完全看不出来。
     * 放在这里而不是转弯结束时恢复：转弯结束那一刻正在转，恢复会当场
     * 加速；等下一个直线段的开头再恢复，时间点才是对的。 */
    StepMotor_RestoreSpeed();

    StepMotor_SetDirection(MOTOR1, forward ? M1_FWD : M1_REV);
    StepMotor_SetDirection(MOTOR2, forward ? M2_FWD : M2_REV);
    StepMotor_SetDirection(MOTOR3, forward ? M3_FWD : M3_REV);
    StepMotor_SetDirection(MOTOR4, forward ? M4_FWD : M4_REV);

    StepMotor_SetPulse(MOTOR1, p1);
    StepMotor_SetPulse(MOTOR2, p2);
    StepMotor_SetPulse(MOTOR3, p3);
    StepMotor_SetPulse(MOTOR4, p4);
}

/* ---------- 运动指令 ---------- */

void Motion_Forward(float mm)
{
    uint32_t base;

    if (!s_ready || mm <= 0.0f) {
        return;
    }

    base = Motion_MmToPulse(mm);

    DriveAll(1,
             (uint32_t)((float)base * s_k[0] + 0.5f),
             (uint32_t)((float)base * s_k[1] + 0.5f),
             (uint32_t)((float)base * s_k[2] + 0.5f),
             (uint32_t)((float)base * s_k[3] + 0.5f));
}

void Motion_Backward(float mm)
{
    uint32_t base;

    if (!s_ready || mm <= 0.0f) {
        return;
    }

    base = Motion_MmToPulse(mm);

    /* 后退就是四个轮子的方向全部取反，其余和前进完全一样 */
    DriveAll(0,
             (uint32_t)((float)base * s_k[0] + 0.5f),
             (uint32_t)((float)base * s_k[1] + 0.5f),
             (uint32_t)((float)base * s_k[2] + 0.5f),
             (uint32_t)((float)base * s_k[3] + 0.5f));
}

uint8_t Motion_HasMecanum(void)
{
    return MOTION_HAS_MECANUM;
}

void Motion_ForwardFlat(float mm)
{
    if (!s_ready || mm <= 0.0f) {
        return;
    }

    /* 关斜坡 -> 启动 -> 立刻恢复，不把状态留给下一条指令。
     * 与旧 StepMotor_GoForwardFlat 的做法一致：ramp_enable 只需要在
     * SetPulse 调用的瞬间为 0。 */
    StepMotor_SetRampEnabled(0);
    Motion_Forward(mm);
    StepMotor_SetRampEnabled(1);
}

void Motion_TurnRight(float deg)
{
    uint32_t p;

    if (!s_ready || deg <= 0.0f) {
        return;
    }

    /* 转角补偿：普通轮原地转是"拧"过去的，拖胎比麦轮严重，实测转不够。
     * 系数放在 chassis.h（麦轮 1.000，差速 1.050），必须现场量了再改 ——
     * 这个数错了的直接表现是"车位不是直角，差几度"。 */
    deg *= CHASSIS_TURN_SCALE;

    p = Motion_DegToPulse(deg);
    if (p == 0) {
        p = 1;
    }

    /* 原地右转：四个 DIR 同电平。
     * 本机 DIR 为左右镜像接线，四个 DIR_CW 等于"左侧后退 + 右侧前进"。
     * 注意：这里不能用 M1_FWD..M4_FWD —— 那组宏编码的是前进语义，
     * 原地转需要的是纯粹的"四个同电平"。 */
    StepMotor_SetDirection(MOTOR1, DIR_CW);
    StepMotor_SetDirection(MOTOR2, DIR_CW);
    StepMotor_SetDirection(MOTOR3, DIR_CW);
    StepMotor_SetDirection(MOTOR4, DIR_CW);

    /* 差速车减速转（CHASSIS_TURN_ARR > 0），麦轮不限速（= 0）。
     *
     * 这是"转的时候不顺畅"的机械侧对策：普通橡胶轮横向不能滑移，四轮
     * 做原地转时是在地面上拧，速度越快越容易"攒一下、弹一下"。降速不能
     * 消除拖胎，只能让每次跳动变小、变密，看起来是连续的。
     *
     * 限速是"本段"的：下一次前进时由 Motion_Forward / DriveLR 恢复，
     * 所以这里不保存也不恢复原档位 —— 转弯之后必定还有一个走直线的段，
     * 那一段开头的恢复动作就是天然的收尾。 */
    if (CHASSIS_TURN_ARR > 0) {
        StepMotor_SetSpeedTarget(CHASSIS_TURN_ARR);
    }

    /* 转弯同样不做软起软停。
     * 理由和 DriveLR 一样：斜坡改的是全局 ARR，转弯时四路一起变，而
     * 四路的目标脉冲是相等的 —— 起坡不影响左右关系，但会让"转 90 度"
     * 的实际角速度前后不一致（起步慢、中段快），画车位时角速度不均
     * 会让喷头在拐角处堆积。匀速拧过去更可控。 */
    StepMotor_SetRampEnabled(0);

    StepMotor_SetPulse(MOTOR1, p);
    StepMotor_SetPulse(MOTOR2, p);
    StepMotor_SetPulse(MOTOR3, p);
    StepMotor_SetPulse(MOTOR4, p);

    StepMotor_SetRampEnabled(1);
}

/* ---------- 视觉纠偏 ---------- */

uint8_t Motion_Trim(float delta_mm)
{
    int32_t half;
    uint8_t moved = 0;

    if (!s_ready) {
        return 0;
    }

    if (delta_mm > MOTION_TRIM_MAX_MM) {
        delta_mm = MOTION_TRIM_MAX_MM;
    }
    if (delta_mm < -MOTION_TRIM_MAX_MM) {
        delta_mm = -MOTION_TRIM_MAX_MM;
    }

    /* 差速的一半摊到每一侧。先取绝对值再定符号，免得负数截断和 +0.5f
     * 的舍入互相打架（-3.7 -> -3 还是 -4，两种都有道理，但差一个脉冲
     * 在左右两侧就是两次不同的方向修正）。 */
    if (delta_mm >= 0.0f) {
        half = (int32_t)(delta_mm * 0.5f * s_pulse_per_mm + 0.5f);
    } else {
        half = -(int32_t)((-delta_mm) * 0.5f * s_pulse_per_mm + 0.5f);
    }

    if (half == 0) {
        /* 落在死区里：不用调，但要把"车到底在不在跑"如实报上去。
         * 这里若一律返回 0，上层就无法区分"死区不需要动"和"四路都停了"，
         * 只能把每一次小修正都当成失败。 */
        return StepMotor_IsAnyRunning();
    }

    half *= MOTION_TRIM_SIGN;

    /* 累计上限。见头文件里的 MOTION_TRIM_ACCUM_MAX_MM ——
     * 一段里一直往同一个方向纠，内侧轮会越停越早，累计到一米的时候
     * 车已经在原地转了 90 度，而每一步单独看都只是"小小的修正"。
     * 撞上限时返回 2，让上层知道"这段该结束了"，而不是继续硬纠。 */
    {
        int32_t limit = (int32_t)(MOTION_TRIM_ACCUM_MAX_MM * s_pulse_per_mm);
        int32_t next  = s_trim_accum + half;

        if (next > limit || next < -limit) {
            return 2;
        }

        s_trim_accum = next;
    }

    /* 向右修正 = 左边多走、右边少走。
     * 注意这不是旋转：四个轮子都还在往前走，只是目标脉冲数一边加一边减，
     * 车走出来的是一段很缓的弧。每 100mm 纠一次，宏观上看就是直线。 */
    if (StepMotor_AdjustTarget(MOTOR1, +half) == 0) { moved = 1; }   /* 左前多走 */
    if (StepMotor_AdjustTarget(MOTOR3, +half) == 0) { moved = 1; }   /* 左后多走 */
    if (StepMotor_AdjustTarget(MOTOR2, -half) == 0) { moved = 1; }   /* 右前少走 */
    if (StepMotor_AdjustTarget(MOTOR4, -half) == 0) { moved = 1; }   /* 右后少走 */

    return moved;
}

void Motion_DriveLR(float left_mm, float right_mm, uint8_t forward)
{
    uint32_t pl;
    uint32_t pr;

    if (!s_ready) {
        return;
    }

    pl = Motion_MmToPulse(left_mm);
    pr = Motion_MmToPulse(right_mm);

    /* SetPulse(0) 会停机。这里给 1 保持"这一路有目标"，和 DriveAll 一致。 */
    if (pl == 0) { pl = 1; }
    if (pr == 0) { pr = 1; }

    /* 新的一段开始：累计纠偏归零。这是"每 5 米重新对齐一次"的落点 ——
     * 不归零的话，跨段累计迟早撞上 MOTION_TRIM_ACCUM_MAX_MM，
     * 然后整条线后半段都纠不动了，而日志里只有一串 ERR_RANGE。 */
    s_trim_accum = 0;

    /* 同 DriveAll：直线段开头一定回到用户档位。 */
    StepMotor_RestoreSpeed();

    StepMotor_SetDirection(MOTOR1, forward ? M1_FWD : M1_REV);
    StepMotor_SetDirection(MOTOR3, forward ? M3_FWD : M3_REV);
    StepMotor_SetDirection(MOTOR2, forward ? M2_FWD : M2_REV);
    StepMotor_SetDirection(MOTOR4, forward ? M4_FWD : M4_REV);

    /* 关斜坡再启动，理由和 Motion_ForwardFlat 一样，但这里更强：
     * DriveLR 会被视觉伺服按 200mm 一段连续调用，段与段之间只有几微秒的
     * 间隙。每段都起一次坡的话，速度在 555Hz 和 3.3kHz 之间来回跳，
     * 画出来的线忽粗忽细，而且分段越多越明显 —— 纠偏反而把线画坏了。
     *
     * 斜坡是给"人按一下走一段"的单次指令准备的，不是给伺服用的。 */
    StepMotor_SetRampEnabled(0);

    StepMotor_SetPulse(MOTOR1, (uint32_t)((float)pl * s_k[0] + 0.5f));
    StepMotor_SetPulse(MOTOR3, (uint32_t)((float)pl * s_k[2] + 0.5f));
    StepMotor_SetPulse(MOTOR2, (uint32_t)((float)pr * s_k[1] + 0.5f));
    StepMotor_SetPulse(MOTOR4, (uint32_t)((float)pr * s_k[3] + 0.5f));

    StepMotor_SetRampEnabled(1);
}

#if MOTION_HAS_MECANUM

uint8_t Motion_MoveLeft(float mm)
{
    uint32_t p;

    if (!s_ready || mm <= 0.0f) {
        return 1;
    }

    p = Motion_MmToPulse(mm);
    if (p == 0) {
        p = 1;
    }

    /* 麦轮横移：左前/左后 与 右前/右后 两组反向，组内同向。
     * 与现有 StepMotor_MOVE_LEFT 的方向组合保持一致。 */
    StepMotor_SetRampEnabled(0);
    StepMotor_SetDirection(MOTOR1, DIR_CCW);
    StepMotor_SetDirection(MOTOR2, DIR_CCW);
    StepMotor_SetDirection(MOTOR3, DIR_CW);
    StepMotor_SetDirection(MOTOR4, DIR_CW);
    StepMotor_SetPulse(MOTOR1, p);
    StepMotor_SetPulse(MOTOR2, p);
    StepMotor_SetPulse(MOTOR3, p);
    StepMotor_SetPulse(MOTOR4, p);
    StepMotor_SetRampEnabled(1);
    return 0;
}

uint8_t Motion_MoveRight(float mm)
{
    uint32_t p;

    if (!s_ready || mm <= 0.0f) {
        return 1;
    }

    p = Motion_MmToPulse(mm);
    if (p == 0) {
        p = 1;
    }

    StepMotor_SetRampEnabled(0);
    StepMotor_SetDirection(MOTOR1, DIR_CW);
    StepMotor_SetDirection(MOTOR2, DIR_CW);
    StepMotor_SetDirection(MOTOR3, DIR_CCW);
    StepMotor_SetDirection(MOTOR4, DIR_CCW);
    StepMotor_SetPulse(MOTOR1, p);
    StepMotor_SetPulse(MOTOR2, p);
    StepMotor_SetPulse(MOTOR3, p);
    StepMotor_SetPulse(MOTOR4, p);
    StepMotor_SetRampEnabled(1);
    return 0;
}

#endif /* MOTION_HAS_MECANUM —— 差速车到此为止，下面没有横移这两个函数 */

uint8_t Motion_IsAnyRunning(void)
{
    return StepMotor_IsAnyRunning();
}
