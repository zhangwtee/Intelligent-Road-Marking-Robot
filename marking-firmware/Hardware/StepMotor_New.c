#include "StepMotor_New.h"

/* ==========================================================================
 * 四路脉冲输出：每个电机独占一个定时器、一个通道
 * ==========================================================================
 *
 * 之前四路全在 TIM3 的 CH1~CH4 上。一个定时器的四个通道共用一个 ARR，
 * 也就是四路只能同频 —— 差速只能表现为"内侧轮直接停"，车是"顿一下、
 * 扭一下"过去的。ZET6 有 TIM2/3/5/8 四个可用通用/高级定时器，每路一个，
 * 各自有自己的 ARR、自己的计数、自己的斜坡。
 *
 * 引脚分配（只改了三根线，其余一个没动）：
 *
 *   M1 左前  TIM3_CH1  PA6    （不变）
 *   M2 右前  TIM2_CH1  PA0    （原 PA7）
 *   M3 左后  TIM5_CH2  PA1    （原 PB0）
 *   M4 右后  TIM8_CH1  PC6    （原 PB1）
 *
 * 为什么 M3 用 CH2 而不是 CH1：
 *   在 F103 上 TIM5_CH1 和 TIM2_CH1 是同一个脚 PA0，而 PA0 已经给了 M2。
 *   PA1 对应的是 TIM5_CH2，所以 M3 只能走通道 2。四路的通道号因此不统一，
 *   下面用 ch 字段记着，所有寄存器操作都按它分派。
 *
 * TIM4 不能动：CH1/CH2 是 PB6/PB7 的两个舵机，50Hz；电机是 kHz 级。
 * 两者差三个数量级，不可能共用一个 ARR。
 *
 * TIM8 是高级定时器，除了常规配置还要单独开主输出（MOE），
 * 否则 CCR 全对、CCER 全使能，引脚上一个脉冲都没有。见 StepMotor_Init。
 * ========================================================================== */

#define NUM_MOTORS  4

/* ---------- 速度档位（仅蓝牙 '+' / '-' 可调） ----------
 * ARR 越小越快：定时器计数时钟 1MHz，ARR=300 → 3.33kHz，ARR=1000 → 1kHz
 */
#define SPEED_FAST_ARR     150   /* 最快档（ARR 下限，约 6.7kHz） */
#define SPEED_SLOW_ARR    1500   /* 最慢档（ARR 上限，约 667Hz） */
#define SPEED_STEP_ARR     100   /* 每按一次增减的步长 */
#define SPEED_DEFAULT_ARR  300   /* 上电默认档（与旧版巡航速度一致） */

typedef struct {
    TIM_TypeDef      *tim;          /* 本路的定时器 */
    uint8_t           ch;           /* 本路用的是哪个通道 1~4 */
    uint16_t          ccen;         /* 对应的 TIM_CCER_CCxE */
    volatile uint32_t set;          /* 本段目标脉冲数，0 = 空闲 */
    volatile uint32_t cnt;          /* 本段已发脉冲数 */
    volatile uint16_t arr;          /* 本路目标 ARR（不含斜坡） */
    volatile uint16_t arr_now;      /* 本路当前已写入的 ARR（含斜坡） */
    volatile uint8_t  ramp_on;      /* 本路是否正在走斜坡 */
    volatile uint8_t  ramp_state;   /* 加速 / 匀速 / 减速 */
    volatile uint16_t ramp_period;  /* 斜坡节拍计数（脉冲数） */
    volatile uint32_t ramp_decel;   /* 本路减速段长度（脉冲数） */
} MotorCh_t;

/* TIM5 走 CH2，其余走 CH1 */
static MotorCh_t s_m[NUM_MOTORS] = {
    { TIM3, 1, TIM_CCER_CC1E, 0, 0, SPEED_DEFAULT_ARR, SPEED_DEFAULT_ARR, 0, 0, 0, 0 },
    { TIM2, 1, TIM_CCER_CC1E, 0, 0, SPEED_DEFAULT_ARR, SPEED_DEFAULT_ARR, 0, 0, 0, 0 },
    { TIM5, 2, TIM_CCER_CC2E, 0, 0, SPEED_DEFAULT_ARR, SPEED_DEFAULT_ARR, 0, 0, 0, 0 },
    { TIM8, 1, TIM_CCER_CC1E, 0, 0, SPEED_DEFAULT_ARR, SPEED_DEFAULT_ARR, 0, 0, 0, 0 },
};

static volatile uint16_t speed_arr  = SPEED_DEFAULT_ARR;   /* 当前生效档位 */
static volatile uint16_t speed_base = SPEED_DEFAULT_ARR;   /* 用户档位 */

/* ---------- 软起软停 ----------
 * 每路各有一套斜坡状态（存在 s_m 里），互不干扰。
 *
 * 为什么不能像以前那样四路共用一套：以前四路同一个 ARR，一套斜坡就够；
 * 现在左右两组的 ARR 可以不同（差速转弯就是这么做的），同一个"每次减 100"
 * 对 ARR=300 的那一路是 33% 的提速，对 ARR=900 的那一路只有 11% ——
 * 共用一套要么快的一路丢步，要么慢的一路根本起不来。
 */
#define RAMP_SOFT_ARR         1800   /* 起停两端的 ARR（越大越柔，1.8kHz 起步） */
#define RAMP_SOFT_STEP_ARR     100   /* 每次调整的 ARR 步长 */
#define RAMP_SOFT_STEP_PULSE   100   /* 每多少个脉冲调整一次 */

#define RAMP_ACCEL    0
#define RAMP_CRUISE   1
#define RAMP_DECEL    2

static volatile uint8_t ramp_enable = 1;   /* 0 = 本次运动不起坡（画线用） */

/* ---------- 换向建立时间 ----------
 *
 * SetDirection 只做一次 GPIO_WriteBit，紧接着 SetPulse 就开通道发脉冲，
 * 中间隔的是几条指令（72MHz 下约 100ns）。而步进驱动器输入侧是光耦，
 * 方向信号从"变电平"到"被内部逻辑认下来"典型要 5~10us（手册 DIR setup
 * time）。这期间到来的脉冲会被按【旧方向】吃掉。
 *
 * 表现：原地转 90 度时四个 DIR 同时翻转，四个驱动器各丢 1~3 个脉冲，
 * 而且丢的数量不一致 —— 车身先顿一下、再扭一下，然后才转起来。
 * 开环步进丢了步没人补，这 3 个脉冲就是 3 个脉冲的永久误差。
 *
 * 只对"方向真的变了"的情况等待。同向时写 GPIO 也照写（防止外部改过引脚），
 * 但不等待 —— 否则每续一段都要白等 10us，视觉伺服 200mm 一段，一秒五段，
 * 一天下来是无意义的累积。
 */
static uint8_t s_last_dir[NUM_MOTORS] = {0xFF, 0xFF, 0xFF, 0xFF};  /* 0xFF = 还没设过 */
static volatile uint8_t s_dir_changed = 0;

static void StepMotor_DirSettleDelay(void)
{
    /* 72MHz 下大约 10~12us。用 volatile 保证优化等级变化时循环不被删掉。 */
    volatile uint8_t i;
    for (i = 0; i < 90; i++) {
        __NOP();
    }
}

/* ---------- 写本路的 ARR 与 CCR ----------
 *
 * ARR 和 CCR 都开了预装载，写入落在影子寄存器里，下一次更新事件才生效。
 * 这样改频的瞬间不会切出一个宽度不完整的脉冲 —— 开环步进丢的步补不回来。
 *
 * CCR 必须跟着 ARR 一起改：只改 ARR 的话占空比会跟着变，而 CCR 一旦
 * 超过 ARR，这一路就完全不出脉冲，等于电机停转。两者绑定写。
 *
 * arr_now 自己记一份，不去读 ARR 寄存器 —— 预装载下读回来是"当前生效值"，
 * 而刚写进去的新值要等下一次更新事件才反映出来。斜坡每 100 个脉冲推进一档，
 * 那时候早就生效了，但读寄存器会让人误以为"没写进去"。
 */
static void Motor_ApplyArr(uint8_t i, uint16_t arr)
{
    MotorCh_t *m = &s_m[i];
    uint16_t v;

    if (arr < 2) {
        arr = 2;
    }
    v = (uint16_t)(arr - 1);

    m->tim->ARR = v;
    switch (m->ch) {
        case 1: m->tim->CCR1 = (uint16_t)(v / 2); break;
        case 2: m->tim->CCR2 = (uint16_t)(v / 2); break;
        case 3: m->tim->CCR3 = (uint16_t)(v / 2); break;
        default: m->tim->CCR4 = (uint16_t)(v / 2); break;
    }
    m->arr_now = arr;
}

static uint16_t Motor_ClampArr(uint16_t arr)
{
    if (arr < SPEED_FAST_ARR) {
        return SPEED_FAST_ARR;
    }
    if (arr > SPEED_SLOW_ARR) {
        return SPEED_SLOW_ARR;
    }
    return arr;
}

/* ---------- 档位：当前生效值的直接设置 / 恢复 ---------- */

/* 把四路统一设成 arr（夹在最快到最慢之间），不动用户档位。
 *
 * 用途只有两个：转弯时临时限速（差速车），以及转完之后恢复。
 * 业务代码不要拿它当"调速按钮" —— 那是 SpeedUp / SpeedDown 的事，
 * 它们改的是用户档位。
 *
 * ramp_on 时不立刻写寄存器：那一拍写下去，斜坡里「当前 ARR > 目标」
 * 当场变假，从慢速直接跳到目标频率。开环步进丢的步补不回来。
 * 让它自己一档档走过去。
 */
void StepMotor_SetSpeedTarget(uint16_t arr)
{
    uint8_t i;

    arr = Motor_ClampArr(arr);
    speed_arr = arr;

    for (i = 0; i < NUM_MOTORS; i++) {
        s_m[i].arr = arr;
        if (!s_m[i].ramp_on) {
            Motor_ApplyArr(i, arr);
        }
    }
}

/* 左右两组分开设。差速转弯用：内侧 ARR 调大，让两组同时到达终点。
 *
 * speed_arr 跟着左组走 —— GetSpeedTarget() 只用来显示"当前大概多快"，
 * 左右不同速时没有唯一答案，取左组是个约定，别拿它做判断。
 */
void StepMotor_SetSpeedLR(uint16_t left_arr, uint16_t right_arr)
{
    uint8_t i;

    left_arr  = Motor_ClampArr(left_arr);
    right_arr = Motor_ClampArr(right_arr);

    speed_arr = left_arr;

    s_m[0].arr = left_arr;    /* M1 左前 */
    s_m[2].arr = left_arr;    /* M3 左后 */
    s_m[1].arr = right_arr;   /* M2 右前 */
    s_m[3].arr = right_arr;   /* M4 右后 */

    for (i = 0; i < NUM_MOTORS; i++) {
        if (!s_m[i].ramp_on) {
            Motor_ApplyArr(i, s_m[i].arr);
        }
    }
}

/* 单路设速度。四轮各自补偿轮径时用，差速模式下不调。 */
void StepMotor_SetSpeedOf(uint8_t motor, uint16_t arr)
{
    if (motor < MOTOR1 || motor > MOTOR4) {
        return;
    }
    arr = Motor_ClampArr(arr);
    s_m[motor - 1].arr = arr;
    if (!s_m[motor - 1].ramp_on) {
        Motor_ApplyArr(motor - 1, arr);
    }
}

uint16_t StepMotor_GetSpeedOf(uint8_t motor)
{
    if (motor < MOTOR1 || motor > MOTOR4) {
        return 0;
    }
    return s_m[motor - 1].arr;
}

/* 回到用户档位。转弯结束后由 motion 层调用。
 *
 * 没必要改的时候直接返回：motion 层每开一段都会调一次（那是唯一能保证
 * "转过弯之后一定恢复"的位置），如果每次都重写一遍 ARR 和 CCR，
 * 相当于在一个匀速段的开头硬生生插一次频率重设，白白扰动一次。
 */
void StepMotor_RestoreSpeed(void)
{
    uint8_t i;
    uint8_t need = (speed_arr != speed_base) ? 1 : 0;

    for (i = 0; i < NUM_MOTORS; i++) {
        if (s_m[i].arr != speed_base) {
            need = 1;
            break;
        }
    }
    if (!need) {
        return;
    }
    StepMotor_SetSpeedTarget(speed_base);
}

uint16_t StepMotor_GetSpeedTarget(void)
{
    return speed_arr;
}

/* 用户档位。和 GetSpeedTarget 的区别：
 *   GetSpeedTarget 是"现在实际在跑多快"，转弯时会被压到 CHASSIS_TURN_ARR；
 *   GetSpeedBase   是"用户用 +/- 选的那个档"，转弯不影响它。
 * motion 层算左右同步比例必须以 base 为基准，否则转一次弯之后
 * 下一段直线的基准就跟着变慢了，而且一次比一次慢。 */
uint16_t StepMotor_GetSpeedBase(void)
{
    return speed_base;
}

/* ---------- 开关斜坡：包在运动调用外面用，用完立刻恢复 ---------- */
void StepMotor_SetRampEnabled(uint8_t en)
{
    ramp_enable = en;
}

/* ---------- 斜坡初始化：某一路从空闲启动时调用 ---------- */
static void Motor_RampStart(uint8_t i, uint32_t pulse)
{
    MotorCh_t *m = &s_m[i];
    uint32_t steps;

    if (!ramp_enable || m->arr >= RAMP_SOFT_ARR) {   /* 斜坡被关掉，或本来就很慢 */
        m->ramp_on = 0;
        Motor_ApplyArr(i, m->arr);
        return;
    }

    steps         = (uint32_t)(RAMP_SOFT_ARR - m->arr) / RAMP_SOFT_STEP_ARR;
    m->ramp_decel = steps * RAMP_SOFT_STEP_PULSE;
    if (m->ramp_decel * 2 > pulse) {            /* 行程短，走成三角形 */
        m->ramp_decel = pulse / 2;
    }
    m->ramp_period = 0;
    m->ramp_state  = RAMP_ACCEL;
    m->ramp_on     = 1;
    Motor_ApplyArr(i, RAMP_SOFT_ARR);           /* 从最慢起步 */
}

/* ---------- 蓝牙调速：每按一次走一档 ----------
 * ARR 越小越快，所以"加速"是减小 ARR。
 *
 * 运动中也只改目标档位，不直接写寄存器：直接写等于把斜坡打断，
 * 从起始慢速一个周期内跳到目标频率，开环步进这一下丢的步补不回来。
 */
void StepMotor_SpeedUp(void)
{
    if (speed_base > SPEED_FAST_ARR + SPEED_STEP_ARR) {
        speed_base -= SPEED_STEP_ARR;
    } else {
        speed_base = SPEED_FAST_ARR;
    }
    StepMotor_SetSpeedTarget(speed_base);
}

void StepMotor_SpeedDown(void)
{
    if (speed_base < SPEED_SLOW_ARR - SPEED_STEP_ARR) {
        speed_base += SPEED_STEP_ARR;
    } else {
        speed_base = SPEED_SLOW_ARR;
    }
    StepMotor_SetSpeedTarget(speed_base);
}

/* ---------- 初始化 ---------- */

static void Motor_InitOne(uint8_t i)
{
    TIM_TimeBaseInitTypeDef tb;
    TIM_OCInitTypeDef       oc;
    TIM_TypeDef            *t = s_m[i].tim;
    uint16_t                arr = s_m[i].arr;

    /* 1. 时基：PSC 72-1 → 计数频率 1MHz，ARR 就是"每微秒多少拍"的倒数 */
    tb.TIM_ClockDivision     = TIM_CKD_DIV1;
    tb.TIM_CounterMode       = TIM_CounterMode_Up;
    tb.TIM_Period            = (uint16_t)(arr - 1);
    tb.TIM_Prescaler         = 72 - 1;
    tb.TIM_RepetitionCounter = 0;          /* 只对 TIM1/TIM8 有意义 */
    TIM_TimeBaseInit(t, &tb);

    /* 2. 输出比较：PWM1，高电平有效，50% */
    TIM_OCStructInit(&oc);
    oc.TIM_OCMode      = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_OCPolarity  = TIM_OCPolarity_High;
    oc.TIM_Pulse       = (uint16_t)((arr - 1) / 2);

    /* 3. 按通道号分派。M3 在 TIM5 上走的是 CH2。 */
    switch (s_m[i].ch) {
        case 1:
            TIM_OC1Init(t, &oc);
            TIM_OC1PreloadConfig(t, TIM_OCPreload_Enable);
            break;
        case 2:
            TIM_OC2Init(t, &oc);
            TIM_OC2PreloadConfig(t, TIM_OCPreload_Enable);
            break;
        case 3:
            TIM_OC3Init(t, &oc);
            TIM_OC3PreloadConfig(t, TIM_OCPreload_Enable);
            break;
        default:
            TIM_OC4Init(t, &oc);
            TIM_OC4PreloadConfig(t, TIM_OCPreload_Enable);
            break;
    }

    /* 4. ARR 预装载：改频落在下一次更新事件，不切出残缺脉冲 */
    TIM_ARRPreloadConfig(t, ENABLE);

    /* 5. 上电先关通道，避免电机自走。
     *    驱动器需要的首脉冲，由 Hardware_Init 里的 StepMotor_SetPulse(x, 1) 显式给出 */
    t->CCER &= ~s_m[i].ccen;

    /* 6. 清标志、开更新中断 */
    TIM_ClearFlag(t, TIM_FLAG_Update);
    TIM_ITConfig(t, TIM_IT_Update, ENABLE);

    /* 7. 启动计数 */
    TIM_Cmd(t, ENABLE);
}

void StepMotor_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;

    /* 1. 时钟。TIM2/3/5 在 APB1，TIM8 在 APB2。 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 |
                           RCC_APB1Periph_TIM3 |
                           RCC_APB1Periph_TIM5, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8 |
                           RCC_APB2Periph_GPIOA |
                           RCC_APB2Periph_GPIOB |
                           RCC_APB2Periph_GPIOC, ENABLE);

    /* 2. 方向引脚：推挽输出（PA8 / PA12 / PA11 / PB12，与旧版一致） */
    GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_Pin   = M1_DIR_PIN | M2_DIR_PIN | M3_DIR_PIN;
    GPIO_Init(M1_DIR_PORT, &GPIO_InitStruct);      /* 三个都在 GPIOA */
    GPIO_InitStruct.GPIO_Pin   = M4_DIR_PIN;
    GPIO_Init(M4_DIR_PORT, &GPIO_InitStruct);

    /* 3. 脉冲引脚：复用推挽。M1/M2/M3 都在 GPIOA，M4 在 GPIOC。 */
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStruct.GPIO_Pin  = M1_PUL_PIN | M2_PUL_PIN | M3_PUL_PIN;
    GPIO_Init(M1_PUL_PORT, &GPIO_InitStruct);      /* 三个都在 GPIOA */
    GPIO_InitStruct.GPIO_Pin  = M4_PUL_PIN;
    GPIO_Init(M4_PUL_PORT, &GPIO_InitStruct);      /* PC6，只有 RCT6/ZET6 才有 */

    /* 4. 四路定时器 */
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        Motor_InitOne(i);
    }

    /* 5. TIM8 是高级定时器，必须单独开主输出（BDTR 的 MOE）。
     *    不开的话 CCR 和 CCER 全对，PA6/PA0/PA1 三路正常，PC6 一动不动 ——
     *    这种"三路转、一路不转"最难查，看起来像引脚虚焊。 */
    TIM_CtrlPWMOutputs(TIM8, ENABLE);

    /* 6. NVIC。四路同优先级，不互相抢占 —— 每路中断里只处理自己那一路，
     *    几十条指令就出来了，没必要分高低。 */
    NVIC_InitStruct.NVIC_IRQChannel                   = TIM2_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority        = 1;
    NVIC_InitStruct.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Init(&NVIC_InitStruct);

    NVIC_InitStruct.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_Init(&NVIC_InitStruct);

    NVIC_InitStruct.NVIC_IRQChannel = TIM5_IRQn;
    NVIC_Init(&NVIC_InitStruct);

    NVIC_InitStruct.NVIC_IRQChannel = TIM8_UP_IRQn;
    NVIC_Init(&NVIC_InitStruct);
}

/* ---------- 设置电机方向 ---------- */
void StepMotor_SetDirection(uint8_t motor, uint8_t dir)
{
    GPIO_TypeDef* port;
    uint16_t pin;

    switch (motor) {
        case MOTOR1: port = M1_DIR_PORT; pin = M1_DIR_PIN; break;
        case MOTOR2: port = M2_DIR_PORT; pin = M2_DIR_PIN; break;
        case MOTOR3: port = M3_DIR_PORT; pin = M3_DIR_PIN; break;
        case MOTOR4: port = M4_DIR_PORT; pin = M4_DIR_PIN; break;
        default: return;
    }

    /* 记录方向变化。真正翻转了才置标志，同向写入不置。 */
    if (s_last_dir[motor - 1] != dir) {
        s_last_dir[motor - 1] = dir;
        s_dir_changed = 1;
    }

    GPIO_WriteBit(port, pin, dir ? Bit_SET : Bit_RESET);
}

/* ---------- 设置指定电机输出指定数量的脉冲 ---------- */
void StepMotor_SetPulse(uint8_t motor, uint32_t pulse)
{
    uint8_t    i;
    MotorCh_t *m;
    uint8_t    was_idle;

    if (motor < MOTOR1 || motor > MOTOR4) {
        return;
    }
    if (pulse == 0) {
        StepMotor_Stop(motor);
        return;
    }

    i = (uint8_t)(motor - 1);
    m = &s_m[i];

    /* 换向之后先等方向电平稳定，再开通道。
     *
     * 放在 SetPulse 而不是 SetDirection 里，是因为四个电机是"先全部设
     * 方向、再逐个开通道"：只要第一个 SetPulse 等这一次，四个 DIR 就都
     * 已经稳定了（它们的 WriteBit 都在几十纳秒内完成）。
     *
     * 必须在 __disable_irq 之前 —— 这段是纯忙等，占了中断的话，
     * 正在走的那几路会在这 10us 里少发几个脉冲。 */
    if (s_dir_changed) {
        StepMotor_DirSettleDelay();
        s_dir_changed = 0;
    }

    /* SET / CNT / 通道使能必须一次写完。
     *
     * 中断是「CNT++ >= SET 就关通道并把 SET 清零」，而 CNT 里上一段的
     * 残余值不会因为换段而变小：中断只要挤在 SET 和 CNT 这两句之间进来，
     * 就会拿旧的大 CNT 跟新 SET 一比、当场判"这段跑完了"，把 SET 清 0。
     * 然后这里在 SET==0 的情况下把通道使能出去 —— 中断的守卫是 `SET > 0`，
     * 从此再没有谁会关它，那一路就一直在转，而 IsAnyRunning() 报"空闲"。
     *
     * 起坡也一起圈进来：它的 ramp_on 同样被中断读写。 */
    __disable_irq();

    was_idle = (m->set == 0) ? 1 : 0;

    m->set = pulse;
    m->cnt = 0;

    /* 本路刚才是空的，才起坡；续段不动斜坡 */
    if (was_idle) {
        Motor_RampStart(i, pulse);
    }

    m->tim->CCER |= m->ccen;

    __enable_irq();
}

/* ---------- 查询当前运动的进度 ----------
 * 用 MOTOR1 作代表：四路同时启动，脉冲数只在补偿时差一两个。
 * CNT 在 SetPulse 里被清零，所以它是"这一段"的进度，不是累计里程。
 */
uint32_t StepMotor_GetPulse(void)
{
    return s_m[0].cnt;
}

uint32_t StepMotor_GetTarget(void)
{
    return s_m[0].set;
}

/* 取某一路的进度。转弯时左右脉冲数不同，要分开看就得用这个。 */
uint32_t StepMotor_GetPulseOf(uint8_t motor)
{
    if (motor < MOTOR1 || motor > MOTOR4) {
        return 0;
    }
    return s_m[motor - 1].cnt;
}

/* 本段剩余脉冲 = SET - CNT。
 *
 * 可以为负：Motion_Trim 把目标往下调的时候，如果已经走过头，下一拍中断
 * 就会收尾。这不是错误，调用方自己决定负值怎么处理（motion 层碰到
 * 非正数就跳过同步）。
 *
 * SET 与 CNT 在这里分两次读，中间可能被中断改一次 —— 差一个脉冲对
 * "按比例分速度"毫无影响，不值得为它关中断。
 */
int32_t StepMotor_GetRemainOf(uint8_t motor)
{
    MotorCh_t *m;

    if (motor < MOTOR1 || motor > MOTOR4) {
        return 0;
    }
    m = &s_m[motor - 1];

    if (m->set == JOG_PULSE_INFINITE || m->set > 0x7FFFFFFFu) {
        return 0x7FFFFFFF;
    }
    return (int32_t)m->set - (int32_t)m->cnt;
}

/* ---------- 运动中调整本段目标脉冲 ----------
 * 要点只有两条：
 *   1) 不清 CNT —— 清了就是停机重来；
 *   2) SET 不能压到 CNT 上 —— 中断的守卫是 CNT >= SET 就关通道，
 *      那是"纠偏"变成"急停"。
 */
uint8_t StepMotor_AdjustTarget(uint8_t motor, int32_t delta)
{
    MotorCh_t *m;
    uint32_t   set;
    uint32_t   cnt;
    int32_t    next;

    if (motor < MOTOR1 || motor > MOTOR4) {
        return 1;
    }
    m = &s_m[motor - 1];

    __disable_irq();

    set = m->set;
    cnt = m->cnt;

    /* 空闲；或者正在被遥控点动（无限行程），两种都不该被纠偏改目标 */
    if (set == 0 || set == JOG_PULSE_INFINITE) {
        __enable_irq();
        return 1;
    }

    next = (int32_t)set + delta;

    /* 至少留 1 个脉冲。CNT 是"已经发出的"，SET 必须比它大。 */
    if (next <= (int32_t)cnt + 1) {
        next = (int32_t)cnt + 1;
    }

    /* 不能顶到 JOG_PULSE_INFINITE，否则这一路会被当成"无限行程"，
     * StopJog 之外再也没有东西能让它停。 */
    if ((uint32_t)next >= JOG_PULSE_INFINITE) {
        next = (int32_t)(JOG_PULSE_INFINITE - 1);
    }

    m->set = (uint32_t)next;

    __enable_irq();
    return 0;
}

/* ---------- 停止单个电机 ---------- */
void StepMotor_Stop(uint8_t motor)
{
    MotorCh_t *m;

    if (motor < MOTOR1 || motor > MOTOR4) {
        return;
    }
    m = &s_m[motor - 1];

    __disable_irq();
    m->tim->CCER &= ~m->ccen;
    m->set        = 0;
    m->cnt        = 0;
    m->ramp_on    = 0;      /* 被主动停掉，斜坡作废 */
    __enable_irq();
}

/* ---------- 停止所有电机 ---------- */
void StepMotor_StopAll(void)
{
    for (uint8_t i = 1; i <= NUM_MOTORS; i++) {
        StepMotor_Stop(i);
    }
}

/* ---------- 遥控点动的柔和停车：把"无限"改成"再走一段减速距离" ----------
 * 直接 StopAll 会猛地一顿，这里复用已有的斜坡状态机把速度压下来。
 * 幂等：已经是有限值的不动，所以看门狗超时后每毫秒调一次也没关系。
 */
void StepMotor_StopJog(void)
{
    uint8_t i;
    uint8_t any_ramp = 0;

    for (i = 0; i < NUM_MOTORS; i++) {
        if (s_m[i].ramp_on) {
            any_ramp = 1;
            break;
        }
    }

    if (!any_ramp) {          /* 没在起坡（本来就是慢档），别拖，直接停 */
        StepMotor_StopAll();
        return;
    }

    for (i = 0; i < NUM_MOTORS; i++) {
        if (s_m[i].set == JOG_PULSE_INFINITE) {
            s_m[i].set = s_m[i].cnt + s_m[i].ramp_decel;
        }
    }
}

/* ---------- 单路更新中断的公共部分 ----------
 *
 * 以前一个 TIM3 中断里循环四路；现在每路一个中断，进来只处理自己，
 * 也不碰别人的变量。最坏情况四路各 6.7kHz，总 26.8k 次/秒，
 * 每次几十条指令，占用不到 3% CPU。
 */
static void Motor_ISR(uint8_t i)
{
    MotorCh_t   *m = &s_m[i];
    TIM_TypeDef *t = m->tim;

    /* 直接读寄存器比 TIM_GetITStatus() 少一次函数调用 —— 这个中断最高
     * 每秒进 6700 次，省下的每一条指令都是实的。 */
    if ((t->SR & TIM_SR_UIF) == 0) {
        return;
    }
    t->SR = (uint16_t)~TIM_SR_UIF;      /* 写 0 清标志，写 1 无效果 */

    /* 1. 脉冲计数 */
    if (m->set > 0) {
        m->cnt++;
        if (m->cnt >= m->set) {
            t->CCER   &= ~m->ccen;
            m->set     = 0;
            m->ramp_on = 0;
        }
    }

    /* 2. 软起软停：一档档推进 ARR，消掉起停瞬间的速度跳变 */
    if (m->ramp_on) {
        uint32_t remaining = m->set - m->cnt;

        if (++m->ramp_period >= RAMP_SOFT_STEP_PULSE) {
            m->ramp_period = 0;

            if (m->ramp_state == RAMP_ACCEL) {
                if (remaining <= m->ramp_decel) {
                    m->ramp_state = RAMP_DECEL;       /* 行程不够，直接收尾 */
                } else if (m->arr_now > m->arr) {
                    uint16_t a = (uint16_t)(m->arr_now - RAMP_SOFT_STEP_ARR);
                    if (a < m->arr) {
                        a = m->arr;
                    }
                    Motor_ApplyArr(i, a);
                } else {
                    m->ramp_state = RAMP_CRUISE;
                }
            } else if (m->ramp_state == RAMP_CRUISE) {
                if (remaining <= m->ramp_decel) {
                    m->ramp_state = RAMP_DECEL;
                }
            } else {                                  /* RAMP_DECEL */
                if (m->arr_now < RAMP_SOFT_ARR) {
                    uint16_t a = (uint16_t)(m->arr_now + RAMP_SOFT_STEP_ARR);
                    if (a > RAMP_SOFT_ARR) {
                        a = RAMP_SOFT_ARR;
                    }
                    Motor_ApplyArr(i, a);
                }
            }
        }
    }
}

void TIM3_IRQHandler(void)      { Motor_ISR(0); }   /* M1 左前 */
void TIM2_IRQHandler(void)      { Motor_ISR(1); }   /* M2 右前 */
void TIM5_IRQHandler(void)      { Motor_ISR(2); }   /* M3 左后 */
void TIM8_UP_IRQHandler(void)   { Motor_ISR(3); }   /* M4 右后 */

/* ---------- 运动函数（设置运动类型） ---------- */

/* 前进（直线） */
void StepMotor_APPROACH(void)
{
    StepMotor_SetDirection(MOTOR1, M1_FWD);
    StepMotor_SetDirection(MOTOR2, M2_FWD);
    StepMotor_SetDirection(MOTOR3, M3_FWD);
    StepMotor_SetDirection(MOTOR4, M4_FWD);
    StepMotor_SetPulse(MOTOR1, JOG_PULSE_INFINITE);
    StepMotor_SetPulse(MOTOR2, JOG_PULSE_INFINITE);
    StepMotor_SetPulse(MOTOR3, JOG_PULSE_INFINITE);
    StepMotor_SetPulse(MOTOR4, JOG_PULSE_INFINITE);
}

/* 后退（直线） */
void StepMotor_RETREAT(void)
{
    StepMotor_SetDirection(MOTOR1, M1_REV);
    StepMotor_SetDirection(MOTOR2, M2_REV);
    StepMotor_SetDirection(MOTOR3, M3_REV);
    StepMotor_SetDirection(MOTOR4, M4_REV);
    StepMotor_SetPulse(MOTOR1, JOG_PULSE_INFINITE);
    StepMotor_SetPulse(MOTOR2, JOG_PULSE_INFINITE);
    StepMotor_SetPulse(MOTOR3, JOG_PULSE_INFINITE);
    StepMotor_SetPulse(MOTOR4, JOG_PULSE_INFINITE);
}

/* 右转（原地旋转） */
void StepMotor_You(void)
{
    StepMotor_SetDirection(MOTOR1, DIR_CW);
    StepMotor_SetDirection(MOTOR2, DIR_CW);
    StepMotor_SetDirection(MOTOR3, DIR_CW);
    StepMotor_SetDirection(MOTOR4, DIR_CW);
    StepMotor_SetPulse(MOTOR1, JOG_PULSE_INFINITE);
    StepMotor_SetPulse(MOTOR2, JOG_PULSE_INFINITE);
    StepMotor_SetPulse(MOTOR3, JOG_PULSE_INFINITE);
    StepMotor_SetPulse(MOTOR4, JOG_PULSE_INFINITE);
}

/* 左转（原地旋转） */
void StepMotor_Zuo(void)
{
    StepMotor_SetDirection(MOTOR1, DIR_CCW);
    StepMotor_SetDirection(MOTOR2, DIR_CCW);
    StepMotor_SetDirection(MOTOR3, DIR_CCW);
    StepMotor_SetDirection(MOTOR4, DIR_CCW);
    StepMotor_SetPulse(MOTOR1, JOG_PULSE_INFINITE);
    StepMotor_SetPulse(MOTOR2, JOG_PULSE_INFINITE);
    StepMotor_SetPulse(MOTOR3, JOG_PULSE_INFINITE);
    StepMotor_SetPulse(MOTOR4, JOG_PULSE_INFINITE);
}

/* 左移（麦轮平移） */
void StepMotor_MOVE_LEFT(void)
{
    StepMotor_SetDirection(MOTOR1, DIR_CCW);
    StepMotor_SetDirection(MOTOR2, DIR_CCW);
    StepMotor_SetDirection(MOTOR3, DIR_CW);
    StepMotor_SetDirection(MOTOR4, DIR_CW);
    StepMotor_SetPulse(MOTOR1, JOG_PULSE_INFINITE);
    StepMotor_SetPulse(MOTOR2, JOG_PULSE_INFINITE);
    StepMotor_SetPulse(MOTOR3, JOG_PULSE_INFINITE);
    StepMotor_SetPulse(MOTOR4, JOG_PULSE_INFINITE);
}

/* 右移（麦轮平移） */
void StepMotor_MOVE_RIGHT(void)
{
    StepMotor_SetDirection(MOTOR1, DIR_CW);
    StepMotor_SetDirection(MOTOR2, DIR_CW);
    StepMotor_SetDirection(MOTOR3, DIR_CCW);
    StepMotor_SetDirection(MOTOR4, DIR_CCW);
    StepMotor_SetPulse(MOTOR1, JOG_PULSE_INFINITE);
    StepMotor_SetPulse(MOTOR2, JOG_PULSE_INFINITE);
    StepMotor_SetPulse(MOTOR3, JOG_PULSE_INFINITE);
    StepMotor_SetPulse(MOTOR4, JOG_PULSE_INFINITE);
}

/* 直线前进指定脉冲数（四个电机同步） */
void StepMotor_GoForward(uint16_t pulse)
{
    StepMotor_SetDirection(MOTOR1, M1_FWD);
    StepMotor_SetDirection(MOTOR2, M2_FWD);
    StepMotor_SetDirection(MOTOR3, M3_FWD);
    StepMotor_SetDirection(MOTOR4, M4_FWD);
    StepMotor_SetPulse(MOTOR1, pulse);
    StepMotor_SetPulse(MOTOR2, pulse);
    StepMotor_SetPulse(MOTOR3, pulse);
    StepMotor_SetPulse(MOTOR4, pulse);
}

/* 画线专用：匀速走直线，不做软起软停。
 * 起坡段车速只有巡航的 1/1.75，而泵的出漆量恒定 —— 走得慢就喷得厚，
 * 会让每条线的头尾 0.3m 明显比中段厚，所以画线这几段必须匀速。 */
void StepMotor_GoForwardFlat(uint16_t pulse)
{
    StepMotor_SetRampEnabled(0);
    StepMotor_GoForward(pulse);
    StepMotor_SetRampEnabled(1);
}

/* 原地右转指定脉冲数（四个电机同向） */
void StepMotor_TurnRight(uint16_t pulse)
{
    StepMotor_SetDirection(MOTOR1, DIR_CW);
    StepMotor_SetDirection(MOTOR2, DIR_CW);
    StepMotor_SetDirection(MOTOR3, DIR_CW);
    StepMotor_SetDirection(MOTOR4, DIR_CW);
    StepMotor_SetPulse(MOTOR1, pulse);
    StepMotor_SetPulse(MOTOR2, pulse);
    StepMotor_SetPulse(MOTOR3, pulse);
    StepMotor_SetPulse(MOTOR4, pulse);
}

/* 检查是否有任何电机仍在发送脉冲 */
uint8_t StepMotor_IsAnyRunning(void)
{
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        if (s_m[i].set != 0) {
            return 1;
        }
    }
    return 0;
}
