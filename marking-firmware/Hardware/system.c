#include "stm32f10x.h"
#include "system.h"
#include "proto.h"      /* 帧协议：CRC8 / 组帧 / 验帧 */
#include "motion.h"     /* 毫米、度 -> 脉冲 */
#include "chassis.h"    /* 底盘车型：麦轮 / 四轮差速，由 Target 的 Define 决定 */
#include "heading.h"    /* 陀螺仪航向估计与直线闭环 */
#include "mpu6050.h"    /* 只为了拿错误计数给状态帧用 */
#include "cfg.h"        /* 速度档位的掉电保存 */
/* 画车位的尺寸和转角。
 *
 * 改造前这里是三个脉冲数（26500 / 12500 / 3000）—— 它们等于多少米、多少度，
 * 只有拿 calib 参数反解才知道，而 calib 参数本身又没写在同一个文件里。
 * 现在写毫米和度：换轮子、改细分只动 calib.h，这三个数不用跟着改。
 *
 * 默认值就是标准车位 5.3m × 2.5m，和改造前的三个脉冲数严格等价
 * （26500 脉冲 / 4.998 脉冲每毫米 = 5302mm）。CMD_PAINT 可以在现场覆盖。 */
#define PAINT_LEN_DEFAULT_MM   5300.0f
#define PAINT_WID_DEFAULT_MM   2500.0f
#define PAINT_TURN_DEG         90.0f

// 整机自检前进距离（约 0.4m）
#define SELFCHECK_MOVE_MM      400.0f

/* 当前这一轮画车位用的尺寸。CMD_PAINT 会改它，画完不复位 ——
 * 下一条 'P' 沿用上次的尺寸，符合"设一次画一排"的现场用法。 */
static float paint_len_mm = PAINT_LEN_DEFAULT_MM;
static float paint_wid_mm = PAINT_WID_DEFAULT_MM;

/* 二次喷涂长度分档：'1'~'5' 设定 0.2 / 0.4 / 0.6 / 0.8 / 1.0 m，
 * 'C' 按当前档位喷。按标定值 4.998 脉冲/毫米算，
 * 第 5 档 5000 脉冲 ≈ 1.0m，默认 speed_arr=300（TIM3 1MHz → 3.3kHz）下约 1.5s。
 *
 * 为什么值得为这个加五条命令：
 * 原来 'C' 写死补 1 米 —— 相机说"补"车补一米，相机说"不补"车不动。
 * 视觉的输出是一个布尔值，动作是一个常数，中间没有任何"看到了多少"
 * 流向"走多远"。这样的视觉，换成每 2 秒自己发一次的定时器，行为完全一样
 * —— 这就是"它像个蓝牙遥控壳子"的准确来源。
 * 有了档位，缺口比例第一次真正参与决策：缺口大补得多，缺口小补得少。
 *
 * 上电默认第 5 档（1.0m），和改造前的固定值一致，上位机不改也能跑。 */
/* 补喷长度只有一个真值：毫米。
 * 改造前是 kResprayPulse[5] 一张表，长度只能是 0.2 的整数倍。
 * 帧命令 PROTO_CMD_RESPRAY 直接给毫米，300mm 就是 300mm，不受档位约束。
 * respray_level 退化成「显示用的档位」，由毫米反算，不再参与执行。 */
static uint16_t respray_mm = 1000;
static uint8_t  respray_level = 4;

/* ---- 看护时限 ----
 * 上电静默：timer_counter 从上电起 1ms 递增，头 3 秒里除急停外什么都不执行。
 *   对着 README 里那条真实风险：上电瞬间串口电平不稳、蓝牙模块启动吐字节，
 *   被当成 'P' 车就自己开始画车位。3 秒够蓝牙模块完成初始化（HC-05 约 1~2s）。 */
#define BOOT_SILENCE_MS     3000

/* 画车位总时限：正常画一个车位 20~40s（4 段 5.3m + 4 次转向）。
 * 超过 120s 还没跑完，说明电机没停（驱动器报警、机械卡死）或者状态机没推进
 * —— 这时候泵还开着，漆会在原地积成一滩。 */
#define PAINT_TIMEOUT_MS    120000

/* 补喷总时限：一段最长 1.0m，正常 1.5s 内结束。10s 是 6 倍余量。 */
#define RESPRAY_TIMEOUT_MS  10000

/* 续画的最小剩余行程。
 *
 * 剩得太少就别记断点了：续一段 3mm 的线，泵开一下关一下，只会在断点
 * 那儿滴一个点，而这一条边本来就是"已经走完了"。 */
#define PAINT_RESUME_MIN_MM 50.0f

/* 帧命令 CMD_MOVE 的总时限：最长 5 米，最慢档 0.13m/s 约 38 秒。 */
#define MOVE_TIMEOUT_MS     60000

// 遥控看门狗超时（ms）：U/R/Y/Z/A/X 是手机蓝牙的点动指令，必须重复发送续命。
// 超时不刷新 = 蓝牙断了、或人已经松手 -> 自动停车。上位机走有线，不受影响。
// 手机 APP 按住按钮会重复发送的话，调到 300 最跟手；
// 按一次才发一条的 APP，得留 2000 才来得及补点。
#define JOG_TIMEOUT_MS  2000

// 画线子状态枚举
typedef enum {
    PAINT_IDLE = 0,
    PAINT_EDGE1,
    PAINT_TURN1,
    PAINT_EDGE2,
    PAINT_TURN2,
    PAINT_EDGE3,
    PAINT_TURN3,
    PAINT_EDGE4,
    PAINT_DONE
} PaintStep;
static PaintStep paint_step = PAINT_IDLE;
static uint8_t selfcheck_step = 0;

/* 现在这条边在不在走（EDGE1~4）。TURN 那几步不算 —— 那几步四个轮子
 * 是同速反向转，本来就没有"左右该走一样"这件事可纠。 */
static uint8_t PaintEdgeRunning(void)
{
    return (uint8_t)(paint_step == PAINT_EDGE1 || paint_step == PAINT_EDGE2
                     || paint_step == PAINT_EDGE3 || paint_step == PAINT_EDGE4);
}

/* STATE_MOVE 的参数：走哪个方向、走多少毫米。
 * 由帧命令 PROTO_CMD_MOVE 填，进入状态时读一次。 */
static uint8_t  move_dir = PROTO_MOVE_FWD;
static uint16_t move_mm  = 0;     /* 左轮距离；横移时是两轮共同距离 */
static uint16_t move_right_mm = 0;/* 右轮距离；等于 move_mm 时是普通直行 */

/* ---- 陀螺仪航向闭环 ----
 *
 * 这一段是"车走不了直线"的正面回答。
 *
 * 视觉纠偏（CMD_TRIM）和陀螺仪闭环解决的不是同一件事：
 *   CMD_TRIM   纠「横向位置」—— 车离标线偏了 30mm，是位置误差，
 *              靠相机看得见，但相机受光照影响，而且有 200ms 级的延迟。
 *   陀螺仪闭环 纠「航向角度」—— 车头朝左偏了 2 度，是姿态误差，
 *              10ms 一次，不受光照影响，但有零偏漂移。
 * 两者互补，但**不能同时往同一个差速上叠**（见 proto.h 的 PROTO_ERR_BUSY）。
 * 所以陀螺仪闭环开着的时候，CMD_TRIM 直接回 BUSY，让上位机停手。 */
static uint8_t gyro_measure;              /* 1 = 正在标轮距，段内不纠偏 */
static uint8_t gyro_measure_prev_en;      /* 标定前闭环是不是开着的 */
static float   gyro_measure_start_deg;    /* 标定开始时的累计航向 */
static uint8_t move_trim_exhausted;       /* 本段纠偏累计到顶，后面别再调 */

/* ---- 画车位时的航向纠偏（可选，由 CMD_PAINT 的 DATA[4] bit0 打开） ----
 *
 * 四条边走的是 Motion_ForwardFlat（匀速、无斜坡），以前完全没有航向兜底：
 * 一条 5.3 米的边，左右轮差 1% 末端就偏 5cm，而且四条边往同一个方向偏，
 * 闭合时缺口翻倍。
 *
 * 为什么做成"每次画车位现选"而不是一个常开开关：纠偏要改左右目标脉冲，
 * 而 ForwardFlat 的全部意义是匀速（漆量均匀）。为了"直"接受多少速度不匀，
 * 取决于这一段画什么 —— 交给按按钮的人决定，比烧死在固件里好。
 *
 * 只纠正边，不纠转弯：转完之后 MarkReference 把那一刻记成"直"，所以
 * 边永远是直的，而角度的误差由 CHASSIS_TURN_SCALE 决定 —— 那两个数
 * 一个管线直不直，一个管角正不正，别混在一起调。 */
static uint8_t paint_heading;          /* 本次画车位四条边走不走航向闭环 */
static uint8_t paint_heading_forced;   /* 闭环是这次画车位打开的，退出要还回去 */
static uint8_t paint_trim_exhausted;   /* 这条边纠偏累计到顶，别再调 */

/* ---- 急停断点 ----
 *
 * T 停在某条边中途时记一份，之后 CMD_PAINT 带 RESUME 位就从这儿接着走。
 *
 * 存"还剩多少毫米"而不是"已经走了多少"：续画时 SetPulse 会把 CNT 清零
 * 重新计数（见 StepMotor_SetPulse 的说明），只有剩余量能直接喂给
 * Motion_ForwardFlat。
 *
 * 生命周期：T 记下 -> RESUME 用掉/失败清掉 -> W（停止复位）也清掉。
 * W 清掉是有意的：W 的语义是"这条活不干了"，留着一个更早的断点，
 * 会让"续画"按钮在几天后仍然亮着。 */
static uint8_t   paint_resume_valid;
static PaintStep paint_resume_step;
static float     paint_resume_mm;
static uint8_t   paint_resume_heading;   /* 断点那条边当时有没有在纠偏 */

/* 轮距自标定时单侧走的距离。
 * 300mm 的取法：若轮距 500mm，车会转 300/500 = 0.6 rad ≈ 34 度。
 * 角度够大，读数的相对误差就小；又不会大到让车转出测试场地。
 * 太小（比如 50mm 只有 5 度）时，角度测量里那 0.2 度的误差
 * 会变成 4% 的轮距误差，标了等于没标。 */
#define GYRO_MEASURE_DIFF_MM    300.0f

/* 续段（视觉伺服专用）
 * ------------------------------------------------------------------
 * 视觉纠偏是"每 200mm 重新对齐一次"：段结束四路 CNT 归零、目标重设，
 * 上一段攒下的左右差就此清空 —— 这是它不会像"整条 5.3m 只纠一次"
 * 那样一路累积到车身横过来的原因。
 *
 * 但段与段之间不能等：等状态回到 WAIT、等上位机看到 STATUS 再发下一段，
 * 中间是 200ms 的空档，而这 200ms 里泵还在出漆，车不动 —— 每一个接头
 * 就是一坨漆。所以下一段的参数先存着，这一段一结束立刻接上。
 *
 * 只缓冲一段：缓冲更多没有意义，上位机就算连着发十段，中间任何一段
 * 出错都会让后面全部作废，不如让"发得太快"直接在 ACK 里暴露出来。 */
static uint8_t  next_move_valid = 0;
static uint8_t  next_move_dir   = PROTO_MOVE_FWD;
static uint16_t next_move_left  = 0;
static uint16_t next_move_right = 0;
// 全局变量用于记录定时器中断次数
volatile uint32_t timer_counter = 0;
static uint16_t led_tick = 0;   // 心跳灯计数（1ms 一次，500 次翻转 = 500ms）

// last_state 初值用非法状态，保证上电后首轮会执行 STATE_WAIT 的进入动作
static SystemCtrl sys_ctrl = {STATE_WAIT, (SystemState)0, 0};

/* 急停前把"这条边还剩多少"记下来。
 *
 * 只在边上记。停在转弯（PAINT_TURN*）上不记 —— 那不是"少喷一段"，是
 * "少转了 90 度"：续一段直线接上去，框是歪的，而且歪得很像转角系数
 * 没标定，人会去调错地方。宁可让续画直接失败（上位机那个按钮是灰的），
 * 也不要给一个"看起来成功、实际接不上"的结果。
 *
 * 换算取 MOTOR1 作代表：这条边四路的目标都是 M1 的脉冲乘轮径系数得来的，
 * 相差在 1% 以内；而 Motion_ForwardFlat 续画时四路仍然等长，用哪一路的
 * 剩余都是同一个数。
 */
static void PaintCaptureResume(void)
{
    int32_t remain;

    /* 不在"画车位"里：这次急停和画车位无关，已有的断点别动。
     *
     * 反例就是连按两下急停 —— 人急的时候会连点。第一下停在某条边
     * 中途，断点记好、状态变 TING；第二下进来时已经不在 STATE_PAINT
     * 了，以前开头那句无条件清空会把刚记好的断点抹掉，面板上的
     * "续画"跟着变灰，而人只看到"按了两下急停，续画没了"。
     *
     * 反过来，在画车位里被打断却记不下断点（停在转弯上、剩不到
     * 50mm），那是真没有可续的，必须清成 0 —— 留着上一次的残值会
     * 让"续画"从一条已经画完的边上再走一段。 */
    if (sys_ctrl.state != STATE_PAINT) {
        return;
    }

    paint_resume_valid = 0;

    if (!PaintEdgeRunning()) {
        return;
    }
    if (!StepMotor_IsAnyRunning()) {
        return;              /* 这条边已经走完，只是状态机还没轮到 */
    }

    remain = StepMotor_GetRemainOf(MOTOR1);
    if (remain <= 0) {
        return;              /* 纠偏把目标压过头了，等于已经走完 */
    }

    if (Motion_PulseToMm((uint32_t)remain) < PAINT_RESUME_MIN_MM) {
        return;
    }

    paint_resume_step    = paint_step;
    paint_resume_mm      = Motion_PulseToMm((uint32_t)remain);
    paint_resume_heading = paint_heading;
    paint_resume_valid   = 1;
}

/* ---- 速度档位的掉电保存 ----
 *
 * 只在"车停着、状态空闲、改完已经过了 2 秒"的时候写 flash：
 *   1) 擦一页要几十毫秒，这段时间取指被挂起，正在发脉冲的那一路会少几步
 *      —— 开环步进丢的步补不回来。而改速本来只许在空闲/急停/遥控点动里做，
 *      遥控点动那几种状态电机是转着的，所以不能只看状态，还得看电机。
 *   2) 2 秒是给"拖滑条调几档"留的合并窗口。flash 擦写寿命一万次上下，
 *      每拖一下就写一次，几百次调试就磨掉一成的余量。
 *
 * 变速的检测放在这里比对 speed_base，而不是塞进 '+'/'-'/CMD_SPEED 三个分支：
 * 以后再加一条改速度的路，忘了在这里登记的后果是"改了但不保存" —— 这类
 * 静默失效在断电之后才被发现，查起来最远。 */
#define SPEED_SAVE_DELAY_MS   2000

/* 写失败之后的退避间隔。擦一页几十毫秒，失败还每圈重试的话，主循环
 * 会从几千 Hz 掉到几十 Hz —— 串口收不进字节、状态帧跟着断，而失败原因
 * （页被写保护之类）不会因为重试得密就好起来。 */
#define SPEED_SAVE_RETRY_MS  10000

static uint16_t speed_seen;
static uint8_t  speed_seen_valid;
static uint32_t speed_changed_ms;
static uint8_t  speed_save_pending;
static uint32_t speed_try_ms;         /* 上一次真去写 flash 的时刻 */
static uint8_t  speed_tried;          /* 写过没有；0 时 speed_try_ms 不作数 */

static void SpeedSaveService(void)
{
    uint16_t arr = StepMotor_GetSpeedBase();

    if (!speed_seen_valid) {
        /* 第一圈只对齐基线，不当作"用户改了速"。
         *
         * 但新板子第一次上电时 flash 里什么都没有，界面上那个"未保存"
         * 标记会永远是橙的 —— 而那个标记的意思是"此刻断电会丢"，实际上
         * 默认值就在代码里，断电也不丢。所以这里把默认值存一次，让
         * "存过的值 == 正在跑的值"这个等式从一开始就成立。
         * 存过一次之后，之后每次上电都能读到，不会再走到这个分支。 */
        speed_seen         = arr;
        speed_seen_valid   = 1;
        speed_save_pending = (uint8_t)(arr != Cfg_GetSavedSpeed());
        speed_changed_ms   = timer_counter;
        return;
    }

    if (arr != speed_seen) {
        speed_seen         = arr;
        speed_save_pending = (uint8_t)(arr != Cfg_GetSavedSpeed());
        speed_changed_ms   = timer_counter;
        /* 换了新值就从零开始计退避：退避针对的是"同一个值写不进去"，
         * 拿上一次失败的等待去拖新值没有道理。 */
        speed_tried        = 0;
    }

    if (!speed_save_pending) {
        return;
    }
    if ((uint32_t)(timer_counter - speed_changed_ms) < SPEED_SAVE_DELAY_MS) {
        return;
    }
    if (sys_ctrl.state != STATE_WAIT && sys_ctrl.state != STATE_TING) {
        return;                 /* 还在干别的，等停下再说 */
    }
    if (StepMotor_IsAnyRunning()) {
        return;                 /* 遥控点动里也在转，一样不能写 */
    }

    /* 失败要退避，不能下一圈接着试。擦一页几十毫秒是 CPU 全程停住的时间，
     * 而失败原因（页被写保护、片子是打磨过的）不会自己好 —— 每圈重试等于
     * 把主循环钉死在几十 Hz，串口和状态帧一起断，症状还看不出是 flash 引起
     * 的。pending 保留着，界面上的"已保存"标记不亮，人至少能看出来没存上。 */
    if (speed_tried
        && (uint32_t)(timer_counter - speed_try_ms) < SPEED_SAVE_RETRY_MS) {
        return;
    }
    speed_tried  = 1;
    speed_try_ms = timer_counter;

    if (Cfg_SaveSpeed(arr)) {
        speed_save_pending = 0;
        speed_tried        = 0;     /* 成功之后重新计：下次改速要能立刻存 */
    }
}

// TIM1初始化函数
void TIM1_Init(void) {
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* 1. 使能TIM1时钟 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

    /* 2. 配置定时器参数 */
    TIM_TimeBaseStructure.TIM_Prescaler = 7200 - 1;     // 预分频值（72MHz / 7200 = 10KHz）
    TIM_TimeBaseStructure.TIM_Period = 10 - 1;       // 自动重装载值（10KHz / 10000 = 1秒中断一次）
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;    // 高级定时器特有参数
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

    /* 3. 使能定时器更新中断 */
    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);

    /* 4. 配置NVIC中断优先级 */
    NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* 5. 启动定时器 */
    TIM_Cmd(TIM1, ENABLE);
}

// GPIO初始化
void GPIO15_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    /* 推挽输出 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}

// TIM1更新中断服务函数
void TIM1_UP_IRQHandler(void) {
    if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update); // 清除中断标志
        timer_counter++;
        if (++led_tick >= 500) {   // 500ms 翻转一次
            led_tick = 0;
            GPIO_WriteBit(GPIOB, GPIO_Pin_15, 
                         (BitAction)(1 - GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_15)));
        }
    }
}

void Hardware_Init(void)  //外设初始化
{
	Servo_Init();		  //舵机初始化
	Servo2_Init();
	// 上电先摆到确定的合拢位：DPWM 初始 TIM_Pulse=0（占空比 0%），不写这两句
	// 舵机在收到第一条 J/D 之前完全没信号，舵臂会被外力带着乱动
	Servo_SetAngle(40.0f);
	Servo2_SetAngle(90.0f);
	Serial2_Init();
	Pump_Init();		  //水泵初始化
	Buzzer_Init();			  //蜂鸣器初始化
	OLED_Init();
	Serial1_Init();
	GPIO15_Init();
	TIM1_Init();
	StepMotor_Init();   
StepMotor_SetPulse(MOTOR1, 1);           
StepMotor_SetPulse(MOTOR2, 1);
StepMotor_SetPulse(MOTOR3, 1);
StepMotor_SetPulse(MOTOR4, 1);

    /* 换算系数在这里算一次（一次浮点除法），之后全是乘法。
     * F103 没有 FPU，运行时的浮点除法一次要几十微秒，放在状态机里
     * 会让脉冲启停明显滞后。 */
    Motion_Init();

    /* 速度档位从 flash 恢复。放在 Motion_Init 之后、Heading_Init 之前：
     * 这里只改 speed_base（用户档位），真正写进定时器要等下一次
     * SetPulse，所以上电不会因此自己动一下，也不会打乱下面陀螺仪的
     * 上电等待。
     *
     * 读不到有效值就保持 StepMotor_Init 里的 SPEED_DEFAULT_ARR —— 一块
     * 新板子第一次上电走的就是这条。 */
    {
        uint16_t saved_speed;

        if (Cfg_LoadSpeed(&saved_speed)) {
            StepMotor_SetUserSpeed(saved_speed);
        }
    }

    /* 陀螺仪放在最后初始化：MPU6050_Init 里有 200 多毫秒的上电等待
     * （手册 tPOWERUP + 软复位恢复时间），放在前面会把串口、舵机、电机
     * 的上电时序一起往后推，上电瞬间那几路电平是悬空的。
     * 返回码不管 —— Heading_Poll 会在车停着的时候自己重连，
     * 传感器没插也不影响其它任何功能。 */
    Heading_Init();
}

/* 指令来源。
 *   USART1 (PA9/PA10) = 蓝牙手机     -> SOURCE_BT
 *   USART2 (PA2/PA3)  = 树莓派上位机 -> SOURCE_PI
 * 以前这两处判断写的是 from == 1，而 1 到底对应哪个口，只能靠 USART.c 里的
 * 接线注释去对。搞反了不会报错，只是 '+'/'-'（蓝牙专用调速）的门开在了
 * 错误的一侧 —— 变成上位机反而能提速、手机反而不能。换成命名常量之后，
 * 看门控逻辑不需要再背接线表。 */
typedef enum {
    SOURCE_BT = 1,      /* USART1 */
    SOURCE_PI = 2       /* USART2 */
} CmdSource;

/* 向指令的来源端口回一个字节：谁发的回给谁，两边日志不会互相串。 */
static void ReplyTo(uint8_t from, uint8_t byte) {
    if (from == SOURCE_BT) {
        Serial1_SendByte(byte);
    } else {
        Serial2_SendByte(byte);
    }
}

/* 现在能不能改速度。
 *
 * 段内（MOVE / PAINT / RESPRAY / SELFCHECK）不许改，两个理由：
 *   1) DriveSyncArr 在段首按"脉冲数 x ARR = 常数"把左右两组的 ARR 定了。
 *      中途改基准会把四路拉成同一个 ARR，比例当场失效，剩下的路只有
 *      一侧在推 —— 车斜着走完这一段，而每一步单独看都没错。
 *   2) 画线时速度决定单位长度的漆量，改速就是改线宽。
 * 遥控点动（U/R/Y/Z/A/X）不限：那几路是无限行程，本来就没有左右比例，
 * 而且"边走边调速对位"正是点动的用法。 */
static uint8_t SpeedChangeAllowed(void)
{
    switch (sys_ctrl.state) {
    case STATE_WAIT:
    case STATE_TING:
    case STATE_APPROACH:
    case STATE_RETREAT:
    case STATE_YOU:
    case STATE_ZUO:
    case STATE_MOVE_LEFT:
    case STATE_MOVE_RIGHT:
        return 1;
    default:
        return 0;
    }
}

/* 第 3 行显示当前档位。改速的三个入口（'+' / '-' / CMD_SPEED）共用，
 * 免得有一处忘了刷新，OLED 上的数和车上跑的就不一样了。 */
static void ShowSpeedOnOled(void)
{
    OLED_ShowString(3, 1, "Speed ");
    OLED_ShowNum(3, 7, StepMotor_GetSpeedBase(), 4);
    OLED_ShowString(3, 11, "      ");
}

/* ==========================================================================
 * 帧协议接入
 *
 * 解析在【主循环】里做，不在中断里做。中断只负责把字节塞进环形缓冲
 * （见 USART.c），主循环用 Peek/Drop 做滚动解析。
 * 对比割草机固件：它是在中断里直接往 11 字节缓冲填、用 __disable_irq()
 * 包住整个填帧过程。那样能用，但中断里做校验和状态判断，延迟不可控，
 * 而且一旦哪里算错就是死机。这里分开之后，中断延迟是常数。
 * ========================================================================== */

/* 每路一份解析状态。下标按 CmdSource，所以数组开 3 个（0 不用）。 */
typedef struct {
    uint8_t  resync;        /* 坏帧之后、重新找到合法帧之前，裸字节不执行 */
    uint8_t  have_last;     /* last_seq / last_cmd 是否有效 */
    uint8_t  last_seq;
    uint8_t  last_cmd;
    uint8_t  last_result;
    uint32_t last_ms;       /* 上一条帧命令的执行时刻，用于去重时间窗 */
    uint32_t last_rx_ms;    /* 最近一次消费字节的时刻，用于静默期判定 */
    uint32_t wait_ms;       /* 队首卡着一个不完整帧的起始时刻，0 = 没在等 */
    uint16_t dropped;       /* 丢掉的字节数（CRC 错 + 静默期） */
} PortStat;

static PortStat port_stat[3];

/* 坏帧之后静默多久没新字节就认为"这段数据流结束了"。
 * 9600 下一个字节 1.04ms，10 字节的帧尾 10ms 内一定到齐。
 * 取 20ms：足够跨过一帧的残余，又短于人手再按一次按钮的间隔。 */
#define PROTO_RESYNC_IDLE_MS    20

/* 队首卡着一个凑不齐的帧，等多久就放弃。
 * 一个孤立的 0x7B（噪声）会永久堵住这一路：后面每条命令进来都变成
 * 「帧的第 2、3…个字节」，永远等不齐 10 字节，整条链路静默失效。
 * 300ms 的取法：OLED 刷一屏要 80~100ms，这期间主循环是停的，超时值
 * 必须比它长，否则一次刷屏就会把正在到达的正常帧判死；
 * 又要比人手重发的间隔短得多，卡住之后能自己恢复。 */
#define PROTO_PARTIAL_MS        300

/* 重发去重的时间窗。同 SEQ 同 CMD 在这个窗口内再来一次 = 对面没收到 ACK
 * 在重发：ACK 要再回（它就是因为没收到才重发的），但命令不能再执行 ——
 * 我们的命令没有一个是幂等的，'P' 执行两次就画两个车位。
 * 窗口不能太大：串口助手上手动发同一帧（SEQ 都是 0）间隔超过 1 秒就该真执行。 */
#define PROTO_DEDUP_MS          1000

/* 上行状态帧周期。200ms = 5Hz，9600 下一帧 10 字节约 10.4ms，
 * 占空比 5%。再快没有意义，2 秒的补喷节拍用不到。 */
#define PROTO_STATUS_PERIOD_MS  200

static uint8_t PortPeek(uint8_t from, uint16_t offset, uint8_t *out)
{
    if (from == SOURCE_BT) {
        return Serial1_Peek(offset, out);
    }
    return Serial2_Peek(offset, out);
}

static uint16_t PortCount(uint8_t from)
{
    if (from == SOURCE_BT) {
        return Serial1_RxCount();
    }
    return Serial2_RxCount();
}

static void PortDrop(uint8_t from, uint16_t n)
{
    if (from == SOURCE_BT) {
        Serial1_Drop(n);
    } else {
        Serial2_Drop(n);
    }
}

/* 整帧发出去。9600 下 10 字节约 10.4ms 的阻塞 —— 状态帧 5Hz 的话占空比 5%，
 * 期间舵机插值和脉冲完成判断会晚 10ms。舵机是 1.5~2 秒的平滑插值，
 * 脉冲停止在 TIM3 中断里，都不受影响，所以这点阻塞可以接受。
 * 如果哪天要提到 50Hz，这里必须先改成 DMA 或中断发送。 */
static void SendFrame(uint8_t from, uint8_t cmd, uint8_t seq, const uint8_t *data)
{
    uint8_t buf[PROTO_FRAME_LEN];
    uint8_t i;

    Proto_Build(buf, cmd, seq, data);

    for (i = 0; i < PROTO_FRAME_LEN; i++) {
        ReplyTo(from, buf[i]);
    }
}

/* ACK：SEQ 原样回显，上位机靠它把"发出去的"和"被接受的"配对。
 * 这是这套协议里唯一能证明"命令真的到达并且被执行"的东西。 */
static void ProtoSendAck(uint8_t from, uint8_t cmd, uint8_t seq, uint8_t result)
{
    uint8_t d[PROTO_DATA_LEN];

    d[0] = cmd;
    d[1] = result;
    d[2] = 0;
    d[3] = 0;
    d[4] = 0;

    SendFrame(from, PROTO_CMD_ACK, seq, d);
}

/* 毫米 -> 显示档位（0..4）。只用于 OLED 和旧命令，
 * 执行长度一律用 respray_mm，不再经过档位。 */
static uint8_t ResprayLevelOfMm(uint16_t mm)
{
    uint8_t lv = (uint8_t)((mm + 99) / 200);

    if (lv < 1) {
        lv = 1;
    }
    if (lv > 5) {
        lv = 5;
    }

    return (uint8_t)(lv - 1);
}

static void HandleFrame(const uint8_t *f, uint8_t from)
{
    const uint8_t *d = &f[3];
    uint8_t  cmd    = f[1];
    uint8_t  seq    = f[2];
    uint8_t  result = PROTO_OK;
    uint8_t  level;
    uint16_t mm;
    uint16_t right_mm;
    uint16_t len_mm, wid_mm;
    uint16_t arr;
    int16_t  trim_mm;

    /* 上电静默同样管帧。帧头和 CRC 只能证明"这串字节没被打乱"，
     * 证明不了"它是给现在的" —— 上电瞬间对面可能正在启动。 */
    if (timer_counter < BOOT_SILENCE_MS) {
        ProtoSendAck(from, cmd, seq, PROTO_ERR_STATE);
        return;
    }

    /* 重发去重。见 PROTO_DEDUP_MS 的说明。 */
    if (port_stat[from].have_last
        && port_stat[from].last_seq == seq
        && port_stat[from].last_cmd == cmd
        && (uint32_t)(timer_counter - port_stat[from].last_ms) < PROTO_DEDUP_MS) {
        ProtoSendAck(from, cmd, seq, port_stat[from].last_result);
        return;
    }

    switch (cmd) {

    case PROTO_CMD_RESPRAY:
        /* 这是"视觉算出来要补多少"真正落地的地方：一个 uint16 毫米值。
         * 改造前要发两条命令（设档 + 开喷），字节之间被打断就静默失败。 */
        mm = (uint16_t)(((uint16_t)d[0] << 8) | d[1]);

        if (mm < PROTO_RESPRAY_MIN_MM || mm > PROTO_RESPRAY_MAX_MM) {
            result = PROTO_ERR_RANGE;
        } else if (sys_ctrl.state != STATE_WAIT) {
            result = PROTO_ERR_STATE;    /* 画着车位时不许插队，和 'C' 同一条规矩 */
        } else {
            respray_mm = mm;
            respray_level = ResprayLevelOfMm(mm);
            sys_ctrl.state = STATE_RESPRAY;
        }
        break;

    case PROTO_CMD_MOVE:
        mm       = (uint16_t)(((uint16_t)d[1] << 8) | d[2]);
        right_mm = (uint16_t)(((uint16_t)d[3] << 8) | d[4]);

        /* 右轮填 0 = 左右同距。老格式只填了 DATA[1:3]，这里补上，
         * 串口助手手敲帧时不用关心第四个字段。 */
        if (right_mm == 0) {
            right_mm = mm;
        }

        if (d[0] > PROTO_MOVE_RIGHT || mm == 0 || mm > PROTO_MOVE_MAX_MM
            || right_mm > PROTO_MOVE_MAX_MM) {
            result = PROTO_ERR_RANGE;
#if CHASSIS_HAS_MECANUM
        } else if ((d[0] == PROTO_MOVE_LEFT || d[0] == PROTO_MOVE_RIGHT)
                   && !Motion_HasMecanum()) {
            /* 麦轮版理论上永远进不来（CHASSIS_HAS_MECANUM 恒为 1），
             * 留着是为了将来出现"麦轮机构但没装横移轮"的半成品车时，
             * 还能靠运行时那句话回绝，而不是直接撞进 Motion_MoveLeft。 */
            result = PROTO_ERR_HW;
#else
        } else if (d[0] == PROTO_MOVE_LEFT || d[0] == PROTO_MOVE_RIGHT) {
            /* 差速底盘：横移在编译期就不存在。
             *
             * 这条分支编出来只有一个常量赋值，没有函数调用 ——
             * 反汇编里看不到 Motion_MoveLeft，链接器也不会把它带进来。
             * 上位机收到 ERR_HW 就知道"这台车不能横移"，而不是
             * "收下了但没动"（后者是现场最难查的一类故障）。 */
            result = PROTO_ERR_HW;
#endif
        } else if (sys_ctrl.state == STATE_MOVE) {
            /* 车正在这一段上走：把下一段存起来，段一结束立刻接上。
             * 不回绝 —— 回绝的话上位机只能等段尾，中间那 200ms 空档
             * 就是漆线上的一处疙瘩。 */
            next_move_valid = 1;
            next_move_dir   = d[0];
            next_move_left  = mm;
            next_move_right = right_mm;
        } else if (sys_ctrl.state != STATE_WAIT) {
            result = PROTO_ERR_STATE;
        } else {
            move_dir      = d[0];
            move_mm       = mm;
            move_right_mm = right_mm;
            sys_ctrl.state = STATE_MOVE;
        }
        break;

    case PROTO_CMD_TRIM:
        /* 段内增量纠偏：不改"这段走多远"，只改"左右两边各自还差多少"。
         *
         * 和 CMD_MOVE 左右轮独立的关系：MOVE 是每段重新对齐（推荐，误差不
         * 累积），TRIM 是段内微调（连续、响应快，适合陀螺仪闭环这种高频源）。
         * 两条都能用，但别在同一段里同时用 —— 一个在重设目标，一个在加减
         * 目标，谁后到谁覆盖，行为取决于时序，没法调试。 */
        trim_mm = (int16_t)(((uint16_t)d[0] << 8) | (uint16_t)d[1]);

        if (trim_mm > PROTO_TRIM_MAX_MM || trim_mm < -PROTO_TRIM_MAX_MM) {
            result = PROTO_ERR_RANGE;
        } else if (sys_ctrl.state != STATE_MOVE) {
            /* 只有车在走的时候纠偏才有落点。静止时调目标，改的是下一段，
             * 而下一段会重新设定 —— 这一下就白丢了，还必须明确告诉上位机。 */
            result = PROTO_ERR_STATE;
        } else if (Heading_IsEnabled()) {
            /* 下位机的航向闭环已经接管差速。这不是故障，是分工 ——
             * 但必须明确回绝，否则上位机会以为"纠了但没效果"，
             * 然后加大增益，两个控制器就开始互相打架。 */
            result = PROTO_ERR_BUSY;
        } else {
            /* Motion_Trim 三种返回：1 = 调了，0 = 车没在走，2 = 累计到顶。
             * 0 和 2 都不能回 OK —— 前者是"车不动了"，后者是"这段纠到头了，
             * 该重新对齐"。上位机看到 2 应该发一条新的 MOVE 段。 */
            uint8_t trim_ret = Motion_Trim((float)trim_mm);

            if (trim_ret == 0) {
                result = PROTO_ERR_STATE;
            } else if (trim_ret == 2) {
                result = PROTO_ERR_RANGE;
            }
        }
        break;

    case PROTO_CMD_PUMP:
        /* 泵以前只能靠 'K' / 'G' 两个裸字节开关，而裸字节在帧化之后
         * 只能由蓝牙那条路走。画线时要"边走边喷"，泵必须能跟 MOVE 一起下。 */
        if (d[0] > 1) {
            result = PROTO_ERR_RANGE;
        } else if (sys_ctrl.state != STATE_WAIT && sys_ctrl.state != STATE_MOVE) {
            result = PROTO_ERR_STATE;
        } else if (d[0]) {
            Pump_On();
        } else {
            Pump_Off();
        }
        break;

    case PROTO_CMD_GYRO:
        /* 陀螺仪航向闭环的开关和现场标定。四个子命令都在这里落地。 */
        switch (d[0]) {

        case PROTO_GYRO_OFF:
            Heading_SetEnabled(0);
            break;

        case PROTO_GYRO_ON:
            /* 没标零就开，等于拿一个可能是任意值的零偏去纠偏 ——
             * 回绝，而不是"先打开再说"。上位机拿到 ERR_STATE 就知道
             * 得先修传感器，而不是去调增益。 */
            if (!Heading_SetEnabled(1)) {
                result = PROTO_ERR_STATE;
            }
            break;

        case PROTO_GYRO_RECAL:
        {
            uint8_t r = Heading_Recalibrate();
            if (r == 1) {
                result = PROTO_ERR_STATE;    /* 车在动，采到的"零偏"里混着转动 */
            } else if (r == 2) {
                result = PROTO_ERR_HW;       /* 传感器不在线 */
            }
            break;
        }

        case PROTO_GYRO_MEASURE:
            /* 轮距自标定：单侧走 300mm，用实际转过的角度反算左右轮中心距。
             *
             * 为什么必须先关闭环：不关的话，车一转，航向闭环就开始
             * 往反方向纠，测出来的角度是"300mm 的差速"和"闭环的修正"
             * 叠加的结果 —— 反算出来的轮距会偏大或偏小，而且看不出来。
             *
             * 为什么必须在 WAIT：这条命令会直接驱动电机，和 CMD_MOVE 一样
             * 是"会动起来"的命令，不能在画车位、补喷中途插进去。 */
            if (sys_ctrl.state != STATE_WAIT) {
                result = PROTO_ERR_STATE;
            } else if (!Heading_IsReady()) {
                result = PROTO_ERR_HW;
            } else {
                gyro_measure_prev_en = Heading_IsEnabled();
                Heading_SetEnabled(0);

                gyro_measure = 1;
                gyro_measure_start_deg = Heading_GetAbsoluteDeg();

                move_dir      = PROTO_MOVE_FWD;
                move_mm       = (uint16_t)GYRO_MEASURE_DIFF_MM;
                move_right_mm = 0;          /* 右轮不走 = 单侧差速 */
                sys_ctrl.state = STATE_MOVE;
            }
            break;

        default:
            result = PROTO_ERR_RANGE;
            break;
        }
        break;

    case PROTO_CMD_SPEED:
        /* 绝对设速。为什么是 ARR 而不是档位号，见 proto.h 里这段的说明。 */
        arr = (uint16_t)(((uint16_t)d[0] << 8) | d[1]);

        if (arr < PROTO_SPEED_MIN_ARR || arr > PROTO_SPEED_MAX_ARR) {
            result = PROTO_ERR_RANGE;
        } else if (!SpeedChangeAllowed()) {
            result = PROTO_ERR_STATE;
        } else {
            StepMotor_SetUserSpeed(arr);
            ShowSpeedOnOled();
        }
        break;

    case PROTO_CMD_PAINT:
        len_mm = (uint16_t)(((uint16_t)d[0] << 8) | d[1]);
        wid_mm = (uint16_t)(((uint16_t)d[2] << 8) | d[3]);

        if (len_mm == 0) { len_mm = (uint16_t)PAINT_LEN_DEFAULT_MM; }
        if (wid_mm == 0) { wid_mm = (uint16_t)PAINT_WID_DEFAULT_MM; }

        if (d[4] & PROTO_PAINT_FLAG_RESUME) {
            /* 续画。尺寸不看：断点里已经记着这条边还剩多少，这个帧里的
             * 长宽只是上位机顺手带上的当前设置。
             *
             * 允许从 TING 直接续 —— 急停之后本来就停在那儿，要求先按一次
             * "停止复位"才能续，等于把最容易忘的一步塞进了流程里。
             * 普通画车位仍然只认 WAIT：那是"从零开一条新线"，
             * 急停后误触一下就动起来，和 'C' 的风险是一回事。 */
            if (!paint_resume_valid) {
                result = PROTO_ERR_STATE;          /* 没有断点，别当成新的一条画 */
            } else if (sys_ctrl.state != STATE_WAIT && sys_ctrl.state != STATE_TING) {
                result = PROTO_ERR_STATE;
            } else if (paint_resume_heading && !Heading_IsReady()) {
                result = PROTO_ERR_HW;             /* 断点带纠偏，但陀螺仪已经掉线 */
            } else {
                sys_ctrl.state = STATE_PAINT;
            }
            break;
        }

        if (len_mm < PROTO_PAINT_MIN_MM || len_mm > PROTO_PAINT_MAX_MM
            || wid_mm < PROTO_PAINT_MIN_MM || wid_mm > PROTO_PAINT_MAX_MM) {
            result = PROTO_ERR_RANGE;
        } else if ((d[4] & PROTO_PAINT_FLAG_HEADING) && !Heading_IsReady()) {
            /* 要了纠偏但陀螺仪没标定/没接上：回绝，而不是"照画但不纠"。
             * 后者画出来是歪的，而日志里一切正常 —— 上位机会以为这一版
             * 闭环已经在工作，然后去怀疑 CHASSIS_TURN_SCALE。 */
            result = PROTO_ERR_HW;
        } else if (sys_ctrl.state != STATE_WAIT) {
            result = PROTO_ERR_STATE;
        } else {
            paint_len_mm       = (float)len_mm;
            paint_wid_mm       = (float)wid_mm;
            paint_heading      = (d[4] & PROTO_PAINT_FLAG_HEADING) ? 1 : 0;
            paint_resume_valid = 0;   /* 新画一条，旧断点作废 */
            sys_ctrl.state     = STATE_PAINT;
        }
        break;

    case PROTO_CMD_LEVEL:
        level = d[0];

        if (level < 1 || level > 5) {
            result = PROTO_ERR_RANGE;
        } else if (sys_ctrl.state != STATE_WAIT && sys_ctrl.state != STATE_TING) {
            result = PROTO_ERR_STATE;
        } else {
            respray_level = (uint8_t)(level - 1);
            respray_mm = (uint16_t)(level * 200);
        }
        break;

    case PROTO_CMD_PING:
        /* 只回 ACK，什么都不动。用来测链路是否单向往通。 */
        break;

    default:
        result = PROTO_ERR_TYPE;
        break;
    }

    port_stat[from].have_last   = 1;
    port_stat[from].last_seq    = seq;
    port_stat[from].last_cmd    = cmd;
    port_stat[from].last_result = result;
    port_stat[from].last_ms     = timer_counter;

    ProtoSendAck(from, cmd, seq, result);
}

/**
  * 函    数：往树莓派发一帧状态
  * 说    明：改造前下位机是【零状态上行】，只回显收到的字节。上位机那个
  *           "spray x5" 计数器只能证明"我发了五次"，不能证明"它动了五次"。
  *           有了这一帧，上位机第一次能知道：现在什么状态、画到第几段、
  *           电机在不在跑、这一段已经走了多少脉冲。
  *
  *           PULSE 是 16 位，65535 脉冲 ≈ 13.1 米，够覆盖单段最长行程
  *           （5.3 米 = 26500 脉冲）。它是"这一段"的进度，不是累计里程。
  *
  *           只发给上位机，不发蓝牙 —— 手机端是串口助手，收到二进制帧
  *           只会显示成一堆乱码。
  */
void SendStatusFrame(void)
{
    static uint32_t last_ms = 0;

    uint8_t  d[PROTO_DATA_LEN];
    uint8_t  flags = 0;
    uint32_t pulse;

    if ((uint32_t)(timer_counter - last_ms) < PROTO_STATUS_PERIOD_MS) {
        return;
    }
    last_ms = timer_counter;

    pulse = StepMotor_GetPulse();

    if (StepMotor_IsAnyRunning())          flags |= 0x01;   /* 电机在输出脉冲 */
    if (timer_counter < BOOT_SILENCE_MS)   flags |= 0x02;   /* 还在上电静默 */
    if (port_stat[SOURCE_PI].dropped > 0)  flags |= 0x04;   /* 丢过字节，链路可疑 */
    /* 0x08 = 陀螺仪可用（在线且已标零）。上位机据此决定要不要发 CMD_TRIM：
     * 不可用的时候视觉纠偏是唯一的直线手段，可用的时候归下位机管。 */
    if (Heading_IsReady())                 flags |= 0x08;
    if (Heading_IsEnabled())               flags |= 0x10;   /* 航向闭环正在跑 */
    /* 0x20 = 陀螺仪掉过线。跑着跑着冒出来，说明接线松了或者干扰变大，
     * 角度精度已经开始下降 —— 赶在车走歪之前就能看见。 */
    if (MPU6050_ErrorCount() > 0)          flags |= 0x20;
    /* 0x40 = 画车位有断点可续（急停停在某条边中途）。
     * 上位机据此决定"续画"按钮能不能按 —— 按钮常亮、按下去回 ERR_STATE
     * 是最糟的交互：人以为在续画，其实一个字都没动。 */
    if (paint_resume_valid)                flags |= 0x40;

    d[0] = (uint8_t)sys_ctrl.state;
    d[1] = (uint8_t)((sys_ctrl.state == STATE_PAINT) ? paint_step : selfcheck_step);
    d[2] = flags;
    d[3] = (uint8_t)(pulse & 0xFF);
    d[4] = (uint8_t)((pulse >> 8) & 0xFF);

    SendFrame(SOURCE_PI, PROTO_CMD_STATUS, 0, d);
}

/**
  * 函    数：往树莓派发一帧航向
  * 说    明：和状态帧同频（5Hz），但内容独立。
  *
  *           为什么不能塞进状态帧：那 5 个字节已经排满了（状态/段号/标志/
  *           两个字节的脉冲进度），而航向本身就要 2 个字节。挤掉脉冲进度
  *           等于拿"走到哪了"换"偏了多少"，两者现场都要看。
  *
  *           航向用 0.01 度为单位（int16 覆盖 ±327 度）：画线时一段的
  *           航向偏差是几度到几十度，0.01 度的分辨率远超需要，
  *           但多出来的位宽不花钱 —— 定长帧的 5 个字节本来就在那。
  *
  *           角速度用 0.1 dps（int16 覆盖 ±3276 dps，量程只有 ±500）：
  *           它的唯一用途是在上位机上画一条曲线看"车是不是在抖"，
  *           精度够用就行。
  *
  *           未就绪时整帧不发，而不是发一帧 0：发 0 的话上位机会画出一条
  *           贴着零线的完美直线，看起来像"走得非常直"。
  */
static void SendHeadingFrame(void)
{
    static uint32_t last_ms = 0;

    uint8_t d[PROTO_DATA_LEN];
    float   deg;
    float   dps;
    int16_t deg100;
    int16_t dps10;
    uint8_t st;
    uint16_t err;

    if ((uint32_t)(timer_counter - last_ms) < PROTO_STATUS_PERIOD_MS) {
        return;
    }
    last_ms = timer_counter;

    if (!Heading_IsReady()) {
        return;
    }

    deg = Heading_GetDeg();
    if (deg >  327.0f) deg =  327.0f;
    if (deg < -327.0f) deg = -327.0f;
    deg100 = (int16_t)(deg * 100.0f);

    dps = Heading_RateDps();
    if (dps >  3276.0f) dps =  3276.0f;
    if (dps < -3276.0f) dps = -3276.0f;
    dps10 = (int16_t)(dps * 10.0f);

    err = MPU6050_ErrorCount();
    if (err > 15) err = 15;

    st = (uint8_t)(((uint8_t)Heading_GetState() & 0x0F) << 4);
    st = (uint8_t)(st | (uint8_t)err);

    d[0] = (uint8_t)((uint16_t)deg100 >> 8);
    d[1] = (uint8_t)((uint16_t)deg100 & 0xFF);
    d[2] = (uint8_t)((uint16_t)dps10 >> 8);
    d[3] = (uint8_t)((uint16_t)dps10 & 0xFF);
    d[4] = st;

    SendFrame(SOURCE_PI, PROTO_CMD_HEADING, 0, d);
}

/* 速度帧：把固件当前档位报给上位机。
 *
 * 发送时机：值一变立刻发（滑条拖一下马上能看到回执），没变 1Hz 兜底。
 * 1Hz 不是 5Hz —— 这一帧除了"现在多快"没有别的用途，9600 下省下来的
 * 带宽是 ACK 和错误码的，而那两样才是排障时真正要看的。 */
static uint16_t speed_info_arr = 0xFFFF;    /* 0xFFFF 保证第一帧一定发出去 */
static uint32_t speed_info_ms;

static void SendSpeedInfoFrame(void)
{
    uint8_t  d[PROTO_DATA_LEN] = {0, 0, 0, 0, 0};
    uint16_t arr = StepMotor_GetSpeedBase();

    if (arr == speed_info_arr
        && (uint32_t)(timer_counter - speed_info_ms) < PROTO_SPEED_INFO_PERIOD_MS) {
        return;
    }
    speed_info_arr = arr;
    speed_info_ms  = timer_counter;

    d[0] = (uint8_t)((arr >> 8) & 0xFF);
    d[1] = (uint8_t)(arr & 0xFF);
    if (Cfg_GetSavedSpeed() == arr) {
        /* 打到闪存里了才置位：上位机据此显示"断电还在"还是"过一会儿才存"。
         * 反过来，若拿"发过 CMD_SPEED"当已保存，掉电丢值的现场会没有任何线索。 */
        d[2] |= PROTO_SPEED_FLAG_SAVED;
    }

    SendFrame(SOURCE_PI, PROTO_CMD_SPEED_INFO, 0, d);
}

static void HandleCmd(uint8_t cmd, uint8_t from) {
    /* 上电静默：头 3 秒只放行急停。
     * 'T' 必须例外 —— 急停在任何时刻都得有效，包括上电那一瞬间。 */
    if (timer_counter < BOOT_SILENCE_MS && cmd != 'T') {
        ReplyTo(from, '?');
        return;
    }

    /* 回显证明"这个字节到了 MCU 串口"。它证明不了命令被执行，但能把故障
     * 分成两半 —— 没回显是链路问题，有回显但车不动是逻辑问题。
     * 这是单字节协议里唯一的一点可观测性，别删。 */
    ReplyTo(from, cmd);
    // 手机点动指令（U/R/Y/Z/A/X）刷新遥控看门狗的时间戳，见 System_StateMachine。
    if (cmd=='U'||cmd=='R'||cmd=='Y'||cmd=='Z'||cmd=='A'||cmd=='X') {
        sys_ctrl.state_timestamp = timer_counter;
    }

    switch(cmd) {
			
		case 'W'://等待指令  串口'W'
			    StepMotor_StopAll();
					OLED_ShowString(3, 1, "Wait            ");
        /* W = 这条活不干了，断点跟着作废。留着的话，"续画"按钮会在
         * 几天后仍然亮着，按下去接到一个早就不存在的位置。 */
        paint_resume_valid = 0;
        sys_ctrl.state = STATE_WAIT;
				break;
		case 'K'://开水泵
        sys_ctrl.state = STATE_KAI;
				break;
		case 'G'://水泵关
				sys_ctrl.state = STATE_GUAN;
				break;
		case 'D'://整机自检夹臂开
				sys_ctrl.state = STATE_SELFCHECK;
				break;
		case 'J'://舵机关
				sys_ctrl.state = STATE_GSERVO;
				break;
		case 'F':	// 蜂鸣提示	'F'
        sys_ctrl.state = STATE_RETREAT_BEEP;
        break;
		/*--------------------------------------------------------------------------------------------------*/
    case 'U':		//前进'U'
        sys_ctrl.state = STATE_APPROACH;
        break;
    case 'R':			//后撤	'R'
        sys_ctrl.state = STATE_RETREAT;
        break;
		case 'Y'://右转
        sys_ctrl.state = STATE_YOU;
				break;
		case 'Z'://左转
        sys_ctrl.state = STATE_ZUO;
				break;
		case 'A': //左移
				sys_ctrl.state = STATE_MOVE_LEFT; 
				break;
    case 'X': //右移
				sys_ctrl.state = STATE_MOVE_RIGHT; 
				break;
		case 'T'://强制停止
        /* 先记断点再切状态：切状态那一下会把电机停掉（见状态机的退出块），
         * 之后 StepMotor_GetRemainOf 读到的就全是"空闲"。 */
        PaintCaptureResume();
        sys_ctrl.state = STATE_TING;
				break;
		case '+':   // 提速一档（单字节通道：蓝牙和上位机都能用）
			if (!SpeedChangeAllowed()) {
			    /* 正在画线/走段：回绝，别让它以为生效了 */
			    ReplyTo(from, '?');
			} else {
			    StepMotor_SpeedUp();
			    ShowSpeedOnOled();
			}
			break;
		case '-':   // 降速一档（同 '+')
			if (!SpeedChangeAllowed()) {
			    ReplyTo(from, '?');
			} else {
			    StepMotor_SpeedDown();
			    ShowSpeedOnOled();
			}
			break;
		case '1': case '2': case '3': case '4': case '5':
		    /* 设定补喷长度。只在 WAIT / TING 里改 —— 跑着的时候改，正在喷的
		     * 那一段长度和 OLED 上显示的档位就对不上了。
		     * 对应关系：'1'->0.2m ... '5'->1.0m。 */
		    if (sys_ctrl.state == STATE_WAIT || sys_ctrl.state == STATE_TING) {
		        respray_level = (uint8_t)(cmd - '1');
		        respray_mm = (uint16_t)((respray_level + 1) * 200);
		        OLED_ShowString(3, 1, "Spray Lvl ");
		        OLED_ShowNum(3, 11, respray_level + 1, 1);
		        OLED_ShowString(3, 12, "    ");
		    } else {
		        ReplyTo(from, '?');     /* 跑着的时候不许改，明确回绝 */
		    }
		    break;

		case 'C':// 二次喷涂
		    // 只在 WAIT 接，一条判断管三件事：
		    //  1) 已经在喷就别重进 —— 重进会把这一段从头开始，而上位机是周期重发的
		    //  2) 别打断正在跑的活（画车位/自检/夹臂都会把状态从 WAIT 带走）
		    //  3) 急停后状态停在 TING，C 进不来 —— 自己会喷的东西必须停得住
		    if (sys_ctrl.state == STATE_WAIT) {
		        sys_ctrl.state = STATE_RESPRAY;
		    }
		    break;
		case 'P'://画车位
    /* 蓝牙兜底路径：不传尺寸，一律用固件默认值。
     *
     * 这里以前直接进 STATE_PAINT，长度用的 paint_len_mm —— 而那是上一次
     * 帧命令写进去的。于是"手机发 P 画标准车位"会画出上一次在界面上设
     * 的尺寸，而且哪儿都不报错：文档说默认 5.3x2.5，实际画的是 5.0x2.3。 */
    if (sys_ctrl.state == STATE_WAIT) {
        paint_len_mm       = PAINT_LEN_DEFAULT_MM;
        paint_wid_mm       = PAINT_WID_DEFAULT_MM;
        paint_heading      = 0;   /* 单字节通道不带纠偏 */
        paint_resume_valid = 0;   /* 更不是续画 */
        sys_ctrl.state     = STATE_PAINT;
    } else {
        /* 正在画 / 走段 / 补喷时发 P：不打断，明确回绝。
         * 和 'C' 用的是同一条判断 —— 这两个按键都会让车自己动起来。 */
        ReplyTo(from, '?');
    }
    break;

		default:
		    /* 不认识的字节：改回一个 '?'，而不是静默丢弃。
		     * 这条直接对着两个真实故障：
		     *   1) 蓝牙模块断开/重连时会吐 "+DISC"、"+CONNECT" 这类 AT 响应，
		     *      里面的 'C'、'D'、'T' 恰好都是合法命令（补喷/自检/急停），
		     *      以前照执行。现在虽挡不住"乱码刚好等于合法字符"，但至少
		     *      不认识的字节有了明确回执，调试时能看见"这个字节我收到了，
		     *      但我不认"。
		     *   2) 波特率漂移、线上噪声产生的假字节，同样落进这里。
		     * 要真正挡住第一种，得靠帧头 + CRC 或者蓝牙口令前缀，
		     * 那是协议改造那一步的事。 */
		    ReplyTo(from, '?');
		    break;
        }
}

/* 按 move_dir / move_mm / move_right_mm 起动一段。
 * 进入 STATE_MOVE 和「续段」两处都调它，保证两条路径行为一致 ——
 * 以前这类"起动逻辑抄两遍"的地方，改一处忘一处是常态。
 * 放在 HandleFrame 之前，因为 HandleFrame 只填参数，不直接驱动。 */
static void StartMove(void)
{
#if CHASSIS_HAS_MECANUM
    if (move_dir == PROTO_MOVE_LEFT) {
        Motion_MoveLeft((float)move_mm);
    } else if (move_dir == PROTO_MOVE_RIGHT) {
        Motion_MoveRight((float)move_mm);
    } else
#endif
    {
        /* 前后都用左右独立接口：右轮 == 左轮时就是直行，不等时是差速。
         * 统一走一条路，免得"直行"和"纠偏"在代码里是两个实现、两套行为。
         *
         * 一律不起坡：CMD_MOVE 是机器指令，不是人按着走。视觉伺服会按
         * 200mm 一段连着发几百条，条条起坡的话速度在 555Hz~3.3kHz 之间
         * 跳，漆厚跟着跳 —— 纠偏反而把线画坏了。 */
        Motion_DriveLR((float)move_mm, (float)move_right_mm,
                       (move_dir == PROTO_MOVE_BACK) ? 0 : 1);
    }
}

/* 每轮主循环、每路最多处理几个【裸字节】（帧单独算，一轮最多一帧）。
 * 不能一次把缓冲里积压的 63 个字节全处理完：万一收到一串噪声或手机连发的
 * 长串，一轮之内状态会被改几十次，而中间那些状态的"进入动作"一个都不会
 * 执行（进入块只在状态变了的那一轮跑），表现出来就是"乱跳"。
 * 分批的另一个好处是每批之后都回状态机推进一次，电机完成判断、舵机插值、
 * 超时检测不会被长时间饿着。
 * 4 个/轮：主循环一圈几十微秒，远快于 9600 下 1.04ms/字节的到达速度。 */
#define CMD_PER_CYCLE   4

/**
  * 函    数：从一路串口里解析消息
  * 参    数：from 端口（SOURCE_BT / SOURCE_PI）
  * 说    明：这是整个协议的核心。规则只有三条：
  *
  *   1) 首字节不是帧头 -> 当成一条单字节命令交出去，丢 1 个字节。
  *      "重同步"和"兼容老的单字节协议"是同一个动作，不是两件事。
  *   2) 首字节是帧头但不够 10 字节 -> 什么都不做，下一轮再来。
  *      绝不能把半帧丢掉 —— 串口是一个字节一个字节送过来的。
  *   3) 够了 10 字节但 CRC 不对 -> 只丢 1 个字节，重新找帧头。
  *      丢整帧会把紧跟其后的真帧一起吞掉，然后永久失步；而失步是静默的，
  *      看起来像"协议不兼容"，是这类 bug 里最难查的一种。
  *
  *   第 3 条之后还有一步：进入静默期，后面的裸字节一律丢弃不执行。
  *   否则坏帧的 payload 会被逐字节当成命令执行（payload 里出现 0x50
  *   就是"开始画车位"），CRC 就白加了。静默期在数据流断开 20ms 后结束。
  */
static void ServicePort(uint8_t from)
{
    uint8_t frame[PROTO_FRAME_LEN];
    uint8_t b;
    uint8_t i;
    uint8_t budget = CMD_PER_CYCLE;   /* 裸字节额度 */
    uint8_t frames = 0;               /* 本轮已处理的帧数 */

    /* 一段数据流结束，退出静默期。人再按一次按钮的间隔远大于 20ms，
     * 所以不会把后面的正常命令吞掉。 */
    if (port_stat[from].resync
        && (uint32_t)(timer_counter - port_stat[from].last_rx_ms) > PROTO_RESYNC_IDLE_MS) {
        port_stat[from].resync = 0;
    }

    while (budget > 0 && frames < 1) {

        if (!PortPeek(from, 0, &b)) {
            return;                       /* 没数据了，回去推进状态机 */
        }

        if (b != PROTO_SOF) {
            PortDrop(from, 1);
            port_stat[from].last_rx_ms = timer_counter;
            port_stat[from].wait_ms = 0;
            budget--;

            if (port_stat[from].resync) {
                port_stat[from].dropped++;    /* 坏帧的残余：丢，不执行 */
            } else {
                HandleCmd(b, from);
            }
            continue;
        }

        /* 半帧：等。串口是一个字节一个字节送过来的，绝不能在这里把
         * 半帧丢掉。但不能无限等 —— 见 PROTO_PARTIAL_MS。 */
        if (PortCount(from) < PROTO_FRAME_LEN) {
            uint16_t avail = PortCount(from);

            if (port_stat[from].wait_ms == 0) {
                port_stat[from].wait_ms = timer_counter;
                return;
            }

            if ((uint32_t)(timer_counter - port_stat[from].wait_ms) < PROTO_PARTIAL_MS) {
                return;
            }

            /* 超时。整段丢掉，重新找帧头。 */
            PortDrop(from, avail);
            port_stat[from].dropped += avail;
            port_stat[from].wait_ms = 0;
            port_stat[from].last_rx_ms = timer_counter;

            /* 这里【不】进静默。能卡到超时，说明后面已经没有连续的字节
             * 正在到达了，没有残余需要防；此刻进来的下一条命令是新的一次
             * 发送，照常执行。进静默反而会把那条命令吞掉。 */
            budget--;
            continue;
        }

        /* 帧凑齐了，等待状态清零 */
        port_stat[from].wait_ms = 0;

        for (i = 0; i < PROTO_FRAME_LEN; i++) {
            PortPeek(from, i, &frame[i]);
        }

        if (!Proto_Check(frame)) {
            PortDrop(from, 1);
            port_stat[from].dropped++;
            port_stat[from].resync = 1;
            port_stat[from].last_rx_ms = timer_counter;
            budget--;
            continue;
        }

        PortDrop(from, PROTO_FRAME_LEN);
        port_stat[from].last_rx_ms = timer_counter;
        port_stat[from].resync = 0;
        frames++;
        HandleFrame(frame, from);
    }
}

void ProcessCommand(void)
{
    /* 两路都要服务。原来是 if / else if，一圈只取一个字节，
     * 蓝牙有事的时候上位机的字节就一直排在后面 —— 现在两路各走一遍，
     * 互不饿死。 */
    ServicePort(SOURCE_BT);
    ServicePort(SOURCE_PI);
}

uint32_t GetTick(void) {
    return timer_counter;  
}
void System_StateMachine(void) {
    /* 陀螺仪采样放在最前面：它内部按 10ms 节流，调用频率高一点没关系。
     * 放在状态机之后的话，某一轮如果状态机里做了长活（OLED 刷新、
     * 舵机插值），采样就会被推迟，而 dt 是实测的 —— 角度不会因此算错，
     * 但采样间隔越大，转弯时的角度就越像"折线"。 */
    Heading_Poll();

    ProcessCommand();
    SendStatusFrame();     /* 5Hz 上行状态，只发给树莓派 */
    SendHeadingFrame();    /* 5Hz 上行航向，未就绪时不发 */
    SendSpeedInfoFrame();  /* 变速立刻发，否则 1Hz 兜底 */

    /* 速度的掉电保存：改完 2 秒、车停着、状态空闲时才真正写 flash。
     * 放在状态机之前 —— 状态机里那一轮可能刚把车开起来，先让这里
     * 看一眼"停没停"，比在状态机里到处埋点可靠。 */
    SpeedSaveService();
    uint32_t current_time = GetTick();
	 // 检测状态是否改变
    if (sys_ctrl.state != sys_ctrl.last_state) {
        // 状态退出处理：停止所有电机（无论之前是什么状态）
        // 注意：如果新状态本身需要电机运动，稍后会重新启动
        
        /* 排队的下一段必须跟着当前段一起作废：留着的话，下次再进
         * STATE_MOVE（可能是完全另一条线）会先跑上一轮残留的那 200mm。 */
        next_move_valid = 0;

        /* 轮距标定只在那一段里有效。放在"离开 MOVE"清，而不是"进入 MOVE"清：
         * CMD_GYRO 的 MEASURE 子命令正是先置标志、再把状态切到 MOVE，
         * 进去就清等于永远开始不了。而急停/超时离开 MOVE 时，
         * last_state 还是 MOVE，标志在这里被清掉，不会带到下一段去。 */
        if (sys_ctrl.last_state == STATE_MOVE) {
            gyro_measure = 0;
        }

        /* 画车位结束（画完 / 急停 / 超时都是这条路）：把闭环还回原样。
         * 不还的话，界面上勾选框写着"没接管"，实际还在纠下一条 MOVE，
         * 于是视觉纠偏被回 BUSY —— 人会以为是视觉坏了。 */
        if (sys_ctrl.last_state == STATE_PAINT) {
            if (paint_heading_forced) {
                Heading_SetEnabled(0);
                paint_heading_forced = 0;
            }
            paint_heading = 0;
        }

        Pump_Off();   // 泵只由下面每个状态的进入动作决定，别让它跟着上个状态活下来
        Buzzer_Off(); // 同理。原来 Buzzer_Off() 只有 RETREAT_BEEP 的未变分支在调，
                      // 所以按了 F 再在 2 秒内按别的键，蜂鸣器就一直响 ——
                      // 连急停都停不掉它
        StepMotor_StopAll();
        Servo_StopAllMotions();  // 舵机同理：打断就停在当前角度保持住。
                                 // 不清 is_active 的话它会一直挂着 1，见 Servo.c

        // 记录新状态进入时间
        sys_ctrl.state_timestamp = current_time;

        // 更新 last_state
        sys_ctrl.last_state = sys_ctrl.state;
    switch(sys_ctrl.state) {
    case STATE_WAIT:   
				StepMotor_StopAll();
				Pump_Off();   // 停车必须关泵
		   OLED_ShowString(3, 1, "Wait            ");
					break;
    case STATE_APPROACH:
				OLED_ShowString(3, 1, "Approach        ");
				StepMotor_APPROACH();
					break;
		case STATE_RETREAT:    // 后退
        OLED_Clear();
        OLED_ShowString(3, 1, "Retreat Done    ");
				StepMotor_RETREAT();
//        if(current_time - sys_ctrl.state_timestamp > 30000) {
//            sys_ctrl.state = STATE_RETREAT_BEEP;
//            sys_ctrl.state_timestamp = current_time;
//        }
        break;
		
	   case STATE_YOU:        // 右转（原地顺时针）
        OLED_ShowString(3, 1, "YOU             ");
				StepMotor_You();
        break;
		 

    case STATE_ZUO:        // 左转（原地逆时针）
        OLED_ShowString(3, 1, "ZUO             ");
				StepMotor_Zuo();
        break;

	
		case STATE_MOVE_LEFT:   
				StepMotor_MOVE_LEFT();
        break;
		
		case STATE_MOVE_RIGHT:   
				StepMotor_MOVE_RIGHT();
        break;
		case STATE_RESPRAY:// 二次喷涂：泵开直行一段
		{
		        Pump_On();
		        Motion_ForwardFlat((float)respray_mm);   /* 毫米是真值，档位只用于显示 */
		        OLED_ShowString(3, 1, "Respray         ");
		        OLED_ShowNum(3, 9, respray_mm, 4);
		        OLED_ShowString(3, 13, "mm ");
		    break;
		}
		case STATE_MOVE:   /* 帧命令 CMD_MOVE：按毫米走一段 */
		{
		    OLED_ShowString(3, 1, "Move            ");

		    /* 段起点对齐：这一段的"直"以此刻的车头方向为准。
		     *
		     * 只在这里做，续段那条路（下面 next_move_valid 分支）不做 ——
		     * 续的是同一条线，重新对齐等于把这一段已经修掉的偏差又当成 0，
		     * 闭环就白跑了。
		     *
		     * 标定轮距时跳过：那一段是故意走弯的，把弯的终点记成"直"，
		     * 后面每一段都会带着 34 度的偏差开始。 */
		    if (!gyro_measure) {
		        Heading_MarkReference();
		    }
		    move_trim_exhausted = 0;

		    StartMove();
		    break;
		}
		case STATE_PAINT:
{
            /* 是不是从断点接着画。断点只在这一拍有效，取完就作废 ——
             * 忘了清的话，下一条"画车位"会莫名其妙只画半条边。 */
            uint8_t resume = paint_resume_valid;

            paint_trim_exhausted = 0;
            paint_heading_forced  = 0;
            paint_resume_valid    = 0;

            if (resume) {
                /* 边号和纠偏开关按断点那一刻还原。
                 * 后者必须显式还原：急停切出 STATE_PAINT 时，退出块已经
                 * 把 paint_heading 清成 0 了，不还回去就等于"续画但不再纠"，
                 * 而界面上那个勾选框还勾着。 */
                paint_step    = paint_resume_step;
                paint_heading = paint_resume_heading;
            }

            /* 要了纠偏就把闭环打开，并记住"是不是这次打开的" —— 退出时
             * 只还回自己打开的那一次，用户本来就开着的不能碰。 */
            if (paint_heading) {
                if (!Heading_IsEnabled()) {
                    Heading_SetEnabled(1);
                    paint_heading_forced = 1;
                }
                Heading_MarkReference();
            }

            Pump_On();

            if (resume) {
                /* 只走这条边剩下的那段。SetPulse 会把 CNT 清零，所以
                 * "续"在这里等于"从当前位置再走 remain 毫米"。 */
                Motion_ForwardFlat(paint_resume_mm);
                paint_resume_mm = 0.0f;
            } else {
                paint_step = PAINT_EDGE1;
                Motion_ForwardFlat(paint_len_mm);
            }

            /* OLED 上必须显示真实边号：续画从 3/4 开始，屏幕上就该是 3/4，
             * 显示 1/4 会让人以为车又从第一条边重新走了。 */
            switch (paint_step) {
                case PAINT_EDGE2: OLED_ShowString(3, 1, "Paint:2/4       "); break;
                case PAINT_EDGE3: OLED_ShowString(3, 1, "Paint:3/4       "); break;
                case PAINT_EDGE4: OLED_ShowString(3, 1, "Paint:4/4       "); break;
                default:          OLED_ShowString(3, 1, "Paint:1/4       "); break;
            }

    break;
}
		
	  case STATE_TING:       // 强制停止
        Pump_Off();
				StepMotor_StopAll();
		    OLED_ShowString(3, 1, "STOPPED         "); 
        break;
		case STATE_KAI://开水泵
					Pump_On();
					OLED_ShowString(3, 1, "KAI             ");
					break;
		case STATE_GUAN://关水泵
					Pump_Off();
					OLED_ShowString(3, 1, "GUAN            ");
					break;
		case STATE_SELFCHECK://整机自检
					selfcheck_step = 0;
					OLED_ShowString(3, 1, "SelfCheck       ");
					break;
			case STATE_GSERVO://关闭舵机
				
						Servo_StartSmoothMotion(0, Servo_GetAngle(0), 40.0f, 2000); // 舵机1
						Servo_StartSmoothMotion(1, Servo_GetAngle(1), 90.0f, 2000);  // 舵机2
		
        // 等待运动完成

						OLED_ShowString(3, 1, "GSERVO          ");
						break;
			
////关闭舵机（非阻塞启动）
//						Servo_StartSmoothMotion(0, Servo_GetAngle(0), 40.0f, 2000);
//						Servo_StartSmoothMotion(1, Servo_GetAngle(1), 90.0f, 2000);
//						OLED_ShowString(3, 1, "GSERVO          ");
//						break;
			
			case STATE_RETREAT_BEEP://蜂鸣器，结束后到等待状态
						Buzzer_On();
						OLED_Clear();
						OLED_ShowString(3, 1, "FMQ             ");
//        if(current_time - sys_ctrl.state_timestamp > 2000) {
//            Buzzer_Off();
//            sys_ctrl.state = STATE_WAIT;
//        }
        break;
				  default:
                break;
    }
}else {
        // 状态未改变，执行周期性任务（如超时判断）
        switch (sys_ctrl.state) {
            case STATE_GSERVO:
                Servo_UpdateAllMotions();
                if (!Servo_IsMoving(0) && !Servo_IsMoving(1)) {
                    sys_ctrl.state = STATE_WAIT;
                }
                break;
            case STATE_SELFCHECK:
                switch (selfcheck_step) {
                    case 0:   // 打开舵臂
                        Servo_StartSmoothMotion(0, Servo_GetAngle(0), 80.0f, 1500);
                        Servo_StartSmoothMotion(1, Servo_GetAngle(1), 50.0f, 1500);
                        selfcheck_step = 1;
                        break;
                    case 1:   // 等舵臂到位
                        Servo_UpdateAllMotions();
                        if (!Servo_IsMoving(0) && !Servo_IsMoving(1)) {
                            Motion_Forward(SELFCHECK_MOVE_MM);
                            selfcheck_step = 2;
                            OLED_ShowString(3, 1, "Check Move      ");
                        }
                        break;
                    case 2:   // 等前进走完
                        if (!StepMotor_IsAnyRunning()) {
                            Servo_StartSmoothMotion(0, Servo_GetAngle(0), 40.0f, 1500);
                            Servo_StartSmoothMotion(1, Servo_GetAngle(1), 90.0f, 1500);
                            selfcheck_step = 3;
                            OLED_ShowString(3, 1, "Check Close     ");
                        }
                        break;
                    case 3:   // 等舵臂关回
                        Servo_UpdateAllMotions();
                        if (!Servo_IsMoving(0) && !Servo_IsMoving(1)) {
                            OLED_ShowString(3, 1, "Check Done      ");
                            sys_ctrl.state = STATE_WAIT;
                        }
                        break;
                    default:
                        sys_ctrl.state = STATE_WAIT;
                        break;
                }
                break;
            case STATE_RETREAT_BEEP:
                if (current_time - sys_ctrl.state_timestamp > 2000) {
                    Buzzer_Off();
                    sys_ctrl.state = STATE_WAIT;
                }
                break;

            case STATE_MOVE:
                /* 和别的运动状态一样，靠「电机真的停了」推进，不是靠延时。
                 *
                 * 停了先看有没有排队的下一段：有就接上，状态不离开
                 * STATE_MOVE —— 离开一次就走一遍退出块（Pump_Off +
                 * StopAll），泵停一下再开，画出来的线就是一段段的。 */
                if (!StepMotor_IsAnyRunning()) {

                    /* --- 轮距标定跑完了：用转过的角度反算 --- */
                    if (gyro_measure) {
                        float turned = Heading_GetAbsoluteDeg() - gyro_measure_start_deg;
                        float width;

                        gyro_measure = 0;

                        /* 只取大小：往哪边转由单侧差速的方向决定，
                         * 轮距是个几何尺寸，和转向正负无关。 */
                        if (turned < 0.0f) {
                            turned = -turned;
                        }

                        width = Heading_EstimateTrackWidth(GYRO_MEASURE_DIFF_MM, turned);

                        if (width > 0.0f && Heading_SetTrackWidth(width)) {
                            /* 显示到第 4 行。这个数要抄进 calib.h 才算固化，
                             * 掉电就没了 —— 见 Heading_SetTrackWidth 的说明。 */
                            OLED_ShowString(4, 1, "W=");
                            OLED_ShowNum(4, 3, (uint32_t)(width + 0.5f), 4);
                            OLED_ShowString(4, 7, "mm  ");
                        } else {
                            /* 转的角度太小或者算出来离谱：多半是打滑，
                             * 或者左轮根本没走（轮子卡住）。不采用。 */
                            OLED_ShowString(4, 1, "W-MEAS FAILED   ");
                        }

                        if (gyro_measure_prev_en) {
                            Heading_SetEnabled(1);
                        }
                        sys_ctrl.state = STATE_WAIT;
                        break;
                    }

                    if (next_move_valid) {
                        move_dir        = next_move_dir;
                        move_mm         = next_move_left;
                        move_right_mm   = next_move_right;
                        next_move_valid = 0;
                        /* 超时是按"整条线"算的，续一段就重新计时，
                         * 否则连续伺服跑到 60 秒会被判卡死。 */
                        sys_ctrl.state_timestamp = current_time;
                        StartMove();
                    } else {
                        sys_ctrl.state = STATE_WAIT;
                    }
                } else if (current_time - sys_ctrl.state_timestamp > MOVE_TIMEOUT_MS) {
                    /* 5 米最慢档要 38 秒，60 秒还没走完说明卡住了。
                     * 没有这条，一次卡死就是状态永远出不去。 */
                    gyro_measure = 0;      /* 标定中途超时，这一次作废 */
                    sys_ctrl.state = STATE_WAIT;
                } else if (!gyro_measure) {
                    /* --- 车在走：跑一次航向闭环 ---
                     *
                     * Heading_TrimDeltaMm 内部已经做了三件事：判断是否就绪、
                     * 10ms 采样之外的 100ms 动作节流、死区和单次限幅。
                     * 这里拿到的 0 就是"这次不用动"，非 0 就交给 Motion_Trim。
                     *
                     * 为什么不在进入块里就纠：车刚起步时脉冲还没发几个，
                     * 改目标只会让斜坡和差速互相干扰；而且那一刻的航向
                     * 本身就是刚才对齐的 0，纠也纠不出东西来。 */
                    float delta = Heading_TrimDeltaMm();

                    if (delta != 0.0f && !move_trim_exhausted) {
                        uint8_t r = Motion_Trim(delta);

                        if (r == 2) {
                            /* 段内累计纠偏到顶（MOTION_TRIM_ACCUM_MAX_MM）。
                             * 再纠下去内侧轮会提前停太多，车开始原地打转，
                             * 而每一步单独看都只是小修正。
                             * 本段剩下的路让它走完，下一段重新对齐。 */
                            move_trim_exhausted = 1;
                        }
                    }
                }
                break;
            case STATE_RESPRAY:
                if (!StepMotor_IsAnyRunning()) {
                    Pump_Off();
                    sys_ctrl.state = STATE_WAIT;
                } else if (current_time - sys_ctrl.state_timestamp > RESPRAY_TIMEOUT_MS) {
                    /* 电机没停但时间超了：驱动器报警、机械卡死、或者脉冲根本
                     * 没发出去。泵还在转，漆会在一处积成一滩。
                     * 强制回 WAIT —— 退出块会统一 Pump_Off + StopAll。 */
                    sys_ctrl.state = STATE_WAIT;
                }
                break;
            case STATE_PAINT:
            {
                /* 总时限看护：画一个车位正常 20~40s，超过 120s 说明卡住了。
                 * 没有这条，一次机械卡死就是泵一直开、车不动、地上一滩漆。 */
                if (current_time - sys_ctrl.state_timestamp > PAINT_TIMEOUT_MS) {
                    sys_ctrl.state = STATE_WAIT;
                    break;
                }

                /* 边走边纠。Heading_TrimDeltaMm 内部有节流（100ms）、死区
                 * （0.4 度）和单次限幅，拿到 0 就是"这一拍不用动"。
                 *
                 * 返回 2 = 这条边的累计纠偏到顶（MOTION_TRIM_ACCUM_MAX_MM）。
                 * 再纠下去内侧轮会越停越早，车开始原地打转，而每一步单独
                 * 看都只是小修正。剩下的路让它走完，下一条边重新对齐。 */
                if (paint_heading && !paint_trim_exhausted && PaintEdgeRunning()) {
                    float delta = Heading_TrimDeltaMm();

                    if (delta != 0.0f && Motion_Trim(delta) == 2) {
                        paint_trim_exhausted = 1;
                    }
                }

                switch (paint_step) {
                    case PAINT_EDGE1:
                        if (!StepMotor_IsAnyRunning()) {
                            Pump_Off();
                            Motion_TurnRight(PAINT_TURN_DEG);
                            paint_step = PAINT_TURN1;
                            OLED_ShowString(3, 1, "Turn1           ");
                        }
                        break;
                    case PAINT_TURN1:
                        if (!StepMotor_IsAnyRunning()) {
                            Pump_On();
                            /* 新的一条边从这一刻起算"直"。转弯本身带来的
                             * 角度偏差在这里归零 —— 边是直的，角的误差归
                             * CHASSIS_TURN_SCALE 管。 */
                            if (paint_heading) {
                                Heading_MarkReference();
                                paint_trim_exhausted = 0;
                            }
                            Motion_ForwardFlat(paint_wid_mm);
                            paint_step = PAINT_EDGE2;
                            OLED_ShowString(3, 1, "Paint:2/4       ");
                        }
                        break;
                    case PAINT_EDGE2:
                        if (!StepMotor_IsAnyRunning()) {
                            Pump_Off();
                            Motion_TurnRight(PAINT_TURN_DEG);
                            paint_step = PAINT_TURN2;
                            OLED_ShowString(3, 1, "Turn2           ");
                        }
                        break;
                    case PAINT_TURN2:
                        if (!StepMotor_IsAnyRunning()) {
                            Pump_On();
                            /* 新的一条边从这一刻起算"直"。转弯本身带来的
                             * 角度偏差在这里归零 —— 边是直的，角的误差归
                             * CHASSIS_TURN_SCALE 管。 */
                            if (paint_heading) {
                                Heading_MarkReference();
                                paint_trim_exhausted = 0;
                            }
                            Motion_ForwardFlat(paint_len_mm);
                            paint_step = PAINT_EDGE3;
                            OLED_ShowString(3, 1, "Paint:3/4       ");
                        }
                        break;
                    case PAINT_EDGE3:
                        if (!StepMotor_IsAnyRunning()) {
                            Pump_Off();
                            Motion_TurnRight(PAINT_TURN_DEG);
                            paint_step = PAINT_TURN3;
                            OLED_ShowString(3, 1, "Turn3           ");
                        }
                        break;
                    case PAINT_TURN3:
                        if (!StepMotor_IsAnyRunning()) {
                            Pump_On();
                            /* 新的一条边从这一刻起算"直"。转弯本身带来的
                             * 角度偏差在这里归零 —— 边是直的，角的误差归
                             * CHASSIS_TURN_SCALE 管。 */
                            if (paint_heading) {
                                Heading_MarkReference();
                                paint_trim_exhausted = 0;
                            }
                            Motion_ForwardFlat(paint_wid_mm);
                            paint_step = PAINT_EDGE4;
                            OLED_ShowString(3, 1, "Paint:4/4       ");
                        }
                        break;
                    case PAINT_EDGE4:
                        if (!StepMotor_IsAnyRunning()) {
                            Pump_Off();
                            paint_step = PAINT_DONE;
                            sys_ctrl.state = STATE_WAIT;
                            OLED_ShowString(3, 1, "Paint Done      ");
                        }
                        break;
                    default:
                        break;
               }
                break;
            }
            // 遥控看门狗：点动指令靠重复发送续命，超时说明蓝牙断了或人松手了。
            // 先柔和减速（StopJog 幂等），减速跑完再回等待 —— 直接切状态的话，
            // 状态退出时的 StopAll 会把它硬停掉，白减速一场。
            case STATE_APPROACH:  case STATE_RETREAT:
            case STATE_YOU:       case STATE_ZUO:
            case STATE_MOVE_LEFT: case STATE_MOVE_RIGHT:
                if (current_time - sys_ctrl.state_timestamp > JOG_TIMEOUT_MS) {
                    StepMotor_StopJog();
                }
                if (!StepMotor_IsAnyRunning()) {
                    sys_ctrl.state = STATE_WAIT;
                }
                break;
            default:
                break;
        }
    }
}
