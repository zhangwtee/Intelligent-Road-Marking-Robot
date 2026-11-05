#include "StepMotor_New.h"

// ---------- 脉冲计数变量 ----------
static volatile uint32_t Motor_PUL_SET[4] = {0, 0, 0, 0};
static volatile uint32_t Motor_PUL_CNT[4] = {0, 0, 0, 0};

// ---------- 速度档位（仅蓝牙 '+' / '-' 可调） ----------
// ARR 越小越快：TIM3 计数时钟 1MHz，ARR=300 → 3.3kHz，ARR=1000 → 1kHz
#define SPEED_FAST_ARR     150   // 最快档（ARR 下限，约 6.7kHz）
#define SPEED_SLOW_ARR    1500   // 最慢档（ARR 上限，约 667Hz）
#define SPEED_STEP_ARR     100   // 每按一次增减的步长
#define SPEED_DEFAULT_ARR  300   // 上电默认档（与旧版巡航速度一致）

static volatile uint16_t speed_arr = SPEED_DEFAULT_ARR;   // 当前生效档位

/* 用户档位。蓝牙 '+'/'-' 改的是它，转弯限速不改它。
 *
 * 为什么要分成两个：转弯时差速车要把速度临时压到 CHASSIS_TURN_ARR，
 * 转完必须回到用户设的那个档。如果只有一个变量，转弯直接覆盖它，
 * 转完之后车就永远停在慢档上，而界面上"档位"显示的还是原来那个 ——
 * 用户会以为电机衰减了。 */
static volatile uint16_t speed_base = SPEED_DEFAULT_ARR;

// ---------- 软起软停（固定斜坡，不随行程变） ----------
// 起步从 RAMP_SOFT_ARR 一档档加到 speed_arr，收尾再一档档退回。
// 巡航速度永远等于档位速度；行程不够就自动走成三角形（加到一半直接转减速）。
#define RAMP_SOFT_ARR         1800   // 起停两端的 ARR（越大越柔，1.8kHz 起步）
#define RAMP_SOFT_STEP_ARR     100   // 每次调整的 ARR 步长
#define RAMP_SOFT_STEP_PULSE   100   // 每多少个脉冲调整一次

#define RAMP_ACCEL    0
#define RAMP_CRUISE   1
#define RAMP_DECEL    2

static volatile uint16_t current_arr = 1000;        // 当前 ARR，由 SetFrequency 维护
static volatile uint8_t  ramp_on     = 0;           // 本次运动是否在做斜坡
static volatile uint8_t  ramp_state  = RAMP_ACCEL;  // 加速 / 匀速 / 减速
static volatile uint16_t ramp_period = 0;           // 斜坡节拍计数
static volatile uint32_t ramp_decel  = 0;           // 减速段长度（脉冲数）
static volatile uint8_t  ramp_enable = 1;           // 0 = 本次运动不起坡（画线用）

// ---------- 换向建立时间 ----------
//
// 这是"转的时候不顺畅"的第一个原因，和车型无关，两套固件都受益。
//
// SetDirection 只做一次 GPIO_WriteBit，紧接着 SetPulse 就开通道发脉冲，
// 中间隔的是几条指令（72MHz 下约 100ns）。而步进驱动器输入侧是光耦，
// 方向信号从"变电平"到"被内部逻辑认下来"典型要 5~10us（手册 DIR setup
// time）。这期间到来的脉冲会被按【旧方向】吃掉。
//
// 表现：原地转 90 度时四个 DIR 同时翻转，四个驱动器各丢 1~3 个脉冲，
// 而且丢的数量不一致 —— 车身先顿一下、再扭一下，然后才转起来。
// 开环步进丢了步没人补，这 3 个脉冲就是 3 个脉冲的永久误差。
//
// 只对"方向真的变了"的情况等待。同向时写 GPIO 也照写（防止外部改过引脚），
// 但不等待 —— 否则每续一段都要白等 10us，视觉伺服 200mm 一段，一秒五段，
// 一天下来是无意义的累积。 */
static uint8_t s_last_dir[4] = {0xFF, 0xFF, 0xFF, 0xFF};   // 0xFF = 还没设过
static volatile uint8_t s_dir_changed = 0;

static void StepMotor_DirSettleDelay(void)
{
    /* 72MHz 下大约 10~12us。__NOP 是一个周期，循环体本身（比较/自增）
     * 还有几个周期，所以 90 次足够覆盖 720 个周期。
     *
     * 不需要精确：驱动器要求的是"至少"5us，多几微秒没有任何代价 ——
     * 这点时间在 3.3kHz（300us 周期）的脉冲序列面前可以忽略。
     * 用 volatile 保证优化等级变化时循环不被删掉。 */
    volatile uint8_t i;
    for (i = 0; i < 90; i++) {
        __NOP();
    }
}

// ---------- 辅助函数：获取对应通道的使能位 ----------
static uint16_t GetCCER_EnableBit(uint8_t motor)
{
    switch (motor) {
        case MOTOR1: return TIM_CCER_CC1E;
        case MOTOR2: return TIM_CCER_CC2E;
        case MOTOR3: return TIM_CCER_CC3E;
        case MOTOR4: return TIM_CCER_CC4E;
        default: return 0;
    }
}

// ---------- 动态修改PWM频率（保持50%占空比） ----------
void StepMotor_SetFrequency(uint16_t arr)
{
    if (arr < 2) arr = 2;                      // 最小值保护
    TIM3->ARR = arr - 1;                       // 更新自动重载值
    // 更新四个通道的比较值，保持占空比50%
    TIM3->CCR1 = (arr - 1) / 2;
    TIM3->CCR2 = (arr - 1) / 2;
    TIM3->CCR3 = (arr - 1) / 2;
    TIM3->CCR4 = (arr - 1) / 2;
    current_arr = arr;
}

// ---------- 档位：当前生效值的直接设置 / 恢复 ----------

/* 直接把"当前生效档位"设成 arr（夹在最快到最慢之间），不动用户档位。
 *
 * 用途只有两个：转弯时临时限速（差速车），以及转完之后恢复。
 * 业务代码不要拿它当"调速按钮" —— 那是 SpeedUp / SpeedDown 的事，
 * 它们改的是用户档位，掉电前一直有效。
 *
 * ramp_on 时不立刻写 ARR：那一拍写下去，斜坡里「current_arr > speed_arr」
 * 当场变假，直接从 555Hz 跳到目标频率。开环步进丢的步补不回来。
 * 让它自己一档档走过去（每 RAMP_SOFT_STEP_PULSE 个脉冲一步）。 */
void StepMotor_SetSpeedTarget(uint16_t arr)
{
    if (arr < SPEED_FAST_ARR) {
        arr = SPEED_FAST_ARR;
    }
    if (arr > SPEED_SLOW_ARR) {
        arr = SPEED_SLOW_ARR;
    }

    speed_arr = arr;

    if (!ramp_on) {
        StepMotor_SetFrequency(arr);
    }
}

/* 回到用户档位。转弯结束后由 motion 层调用。
 *
 * 没被限过速时直接返回：motion 层每开一段都会调一次（那是唯一能保证
 * "转过弯之后一定恢复"的位置），如果每次都重写一遍 ARR 和四个 CCR，
 * 相当于在一个匀速段的开头硬生生插一次频率重设，白白扰动一次。 */
void StepMotor_RestoreSpeed(void)
{
    if (speed_arr == speed_base) {
        return;
    }
    StepMotor_SetSpeedTarget(speed_base);
}

uint16_t StepMotor_GetSpeedTarget(void)
{
    return speed_arr;
}

// ---------- 开关斜坡：包在运动调用外面用，用完立刻恢复，不留跨指令的状态 ----------
void StepMotor_SetRampEnabled(uint8_t en)
{
    ramp_enable = en;
}

// ---------- 斜坡初始化：每次运动启动时由第一路电机调用一次 ----------
static void StepMotor_RampStart(uint32_t pulse)
{
    uint32_t steps;

    if (!ramp_enable || speed_arr >= RAMP_SOFT_ARR) {   // 斜坡被关掉，或档位本来就慢
        ramp_on = 0;
        StepMotor_SetFrequency(speed_arr);
        return;
    }

    steps      = (uint32_t)(RAMP_SOFT_ARR - speed_arr) / RAMP_SOFT_STEP_ARR;
    ramp_decel = steps * RAMP_SOFT_STEP_PULSE;
    if (ramp_decel * 2 > pulse) {            // 行程短，走成三角形
        ramp_decel = pulse / 2;
    }
    ramp_period = 0;
    ramp_state  = RAMP_ACCEL;
    ramp_on     = 1;
    StepMotor_SetFrequency(RAMP_SOFT_ARR);   // 从最慢起步
}

// ---------- 蓝牙调速：每按一次走一档，正在运动时立即生效 ----------
// ARR 越小越快，所以"加速"是减小 ARR
void StepMotor_SpeedUp(void)
{
    if (speed_base > SPEED_FAST_ARR + SPEED_STEP_ARR) {
        speed_base -= SPEED_STEP_ARR;
    } else {
        speed_base = SPEED_FAST_ARR;
    }
    // 只在没起坡的时候直接写 ARR。运动中写下去等于把 current_arr 一步设成
    // 目标值，斜坡里那句「current_arr > speed_arr」当场变成假，下一拍就切
    // CRUISE —— 从 1800(=555Hz) 一个周期内跳到 150(=5kHz)，九倍速的突变。
    // 开环步进没有编码器，这一下丢的步再也补不回来。
    // 只改档位当目标，中断里那套斜坡会自己一档档走过去
    // （每 RAMP_SOFT_STEP_PULSE 个脉冲走 RAMP_SOFT_STEP_ARR）。
    StepMotor_SetSpeedTarget(speed_base);
}

void StepMotor_SpeedDown(void)
{
    if (speed_base < SPEED_SLOW_ARR - SPEED_STEP_ARR) {
        speed_base += SPEED_STEP_ARR;
    } else {
        speed_base = SPEED_SLOW_ARR;
    }
    // 只在没起坡的时候直接写 ARR。运动中写下去等于把 current_arr 一步设成
    // 目标值，斜坡里那句「current_arr > speed_arr」当场变成假，下一拍就切
    // CRUISE —— 从 1800(=555Hz) 一个周期内跳到 150(=5kHz)，九倍速的突变。
    // 开环步进没有编码器，这一下丢的步再也补不回来。
    // 只改 speed_arr 当目标，中断里那套斜坡会自己一档档走过去
    // （每 RAMP_SOFT_STEP_PULSE 个脉冲走 RAMP_SOFT_STEP_ARR）。
    // 降速这边只跳一档，本身不危险，但同理：直接写会把正在走的斜坡
    // 打断成 CRUISE，加速段白跑。走斜坡更平滑。
    StepMotor_SetSpeedTarget(speed_base);
}

// ---------- 初始化TIM3，产生四路PWM（频率可调，占空比50%） ----------
void StepMotor_Init(void)
{
    // 1. 时钟使能
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);

    // 2. 配置方向引脚为推挽输出
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_Pin = M1_DIR_PIN | M2_DIR_PIN | M3_DIR_PIN;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Pin = M4_DIR_PIN;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    // 3. 配置脉冲引脚为复用推挽输出
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
    // PA6 (CH1), PA7 (CH2)
    GPIO_InitStruct.GPIO_Pin = M1_PUL_PIN | M2_PUL_PIN;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
    // PB0 (CH3), PB1 (CH4)
    GPIO_InitStruct.GPIO_Pin = M3_PUL_PIN | M4_PUL_PIN;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    // 4. 配置TIM3时基单元
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
    TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStruct.TIM_Period = 1000 - 1;       // ARR，初始1kHz
    TIM_TimeBaseStruct.TIM_Prescaler = 72 - 1;      // PSC，计数频率1MHz
    TIM_TimeBaseStruct.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStruct);

    // 5. 配置PWM输出（四个通道，占空比50%）
    TIM_OCInitTypeDef TIM_OCStruct;
    TIM_OCStructInit(&TIM_OCStruct);
    TIM_OCStruct.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCStruct.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCStruct.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCStruct.TIM_Pulse = 500;   // CCR = 500，占空比50%
    TIM_OC1Init(TIM3, &TIM_OCStruct);
    TIM_OC2Init(TIM3, &TIM_OCStruct);
    TIM_OC3Init(TIM3, &TIM_OCStruct);
    TIM_OC4Init(TIM3, &TIM_OCStruct);

    // 5.1 使能预装载：ARR/CCR 在下一次更新事件才生效，避免改频瞬间产生残缺脉冲而丢步
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM3, ENABLE);

    // 5.2 初始化时先关闭四路 PWM 输出，避免上电电机自走。
    //     驱动器需要的首脉冲，由 Hardware_Init 里的 StepMotor_SetPulse(MOTORx, 1) 显式给出
    TIM3->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E);

    // 6. 清除更新标志，配置中断
    TIM_ClearFlag(TIM3, TIM_FLAG_Update);
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);

    // 7. NVIC配置
    NVIC_InitTypeDef NVIC_InitStruct;
    NVIC_InitStruct.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
    NVIC_Init(&NVIC_InitStruct);

    // 8. 使能TIM3
    TIM_Cmd(TIM3, ENABLE);
}

// ---------- 设置电机方向 ----------
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
    /* 记录方向变化。真正翻转了才置标志，同向写入不置 ——
     * 见上面 StepMotor_DirSettleDelay 的说明。 */
    if (s_last_dir[motor - 1] != dir) {
        s_last_dir[motor - 1] = dir;
        s_dir_changed = 1;
    }

    GPIO_WriteBit(port, pin, dir ? Bit_SET : Bit_RESET);
}

// ---------- 设置指定电机输出指定数量的脉冲 ----------
void StepMotor_SetPulse(uint8_t motor, uint32_t pulse)
{
    if (pulse == 0) {
        StepMotor_Stop(motor);
        return;
    }

    uint16_t enableBit = GetCCER_EnableBit(motor);

    /* 换向之后先等方向电平稳定，再开通道。
     *
     * 放在 SetPulse 而不是 SetDirection 里，是因为四个电机是"先全部设
     * 方向、再逐个开通道"：只要第一个 SetPulse 等这一次，四个 DIR 就都
     * 已经稳定了（它们的 WriteBit 都在几十纳秒内完成）。在 SetDirection
     * 里等的话，一次转弯要等四遍，而且前三个的等待完全多余。
     *
     * 必须在 __disable_irq 之前 —— 这段是纯忙等，占了中断的话，
     * 正在走的那几路会在这 10us 里少发几个脉冲。 */
    if (s_dir_changed) {
        StepMotor_DirSettleDelay();
        s_dir_changed = 0;
    }

    // SET / CNT / 通道使能必须一次写完，不能只锁最后那句 CCER 写。
    // TIM3 中断是「CNT++ >= SET 就关通道并把 SET 清零」，而 CNT 里上一段的
    // 残余值不会因为换段而变小：中断只要挤在 SET 和 CNT 这两句之间进来，
    // 就会拿旧的大 CNT 跟新 SET 一比、当场判"这段跑完了"，把 SET 清 0。
    // 然后这里在 SET==0 的情况下把通道使能出去 —— 中断的守卫是 `SET > 0`，
    // 从此再没有谁会关它，那一路就一直在转，而 IsAnyRunning() 报的是"空闲"。
    // 起坡也一起圈进来：它的 ramp_on 同样被中断读写，放外面会被当成
    // remaining==0 清掉。这一段都是寄存器写，占中断的时间可以忽略。
    __disable_irq();

    uint8_t was_idle = (Motor_PUL_SET[0] | Motor_PUL_SET[1] |
                        Motor_PUL_SET[2] | Motor_PUL_SET[3]) == 0;

    Motor_PUL_SET[motor-1] = pulse;
    Motor_PUL_CNT[motor-1] = 0;

    // 第一路起坡；后面几路保持频率不动，免得把斜坡打断
    if (was_idle) {
        StepMotor_RampStart(pulse);
    }

    TIM3->CCER |= enableBit;
    __enable_irq();
}

// ---------- 查询当前运动的进度 ----------
// 用 MOTOR1 作代表：四路总是同时启动、脉冲数只在每轮补偿时才有一两个的差别。
// CNT 在 SetPulse 里被清零，所以它是"这一段"的进度，不是累计里程。
uint32_t StepMotor_GetPulse(void)
{
    return Motor_PUL_CNT[0];
}

uint32_t StepMotor_GetTarget(void)
{
    return Motor_PUL_SET[0];
}

/* ---------- 运动中调整本段目标脉冲 ----------
 * 见头文件里的说明。要点只有两条：
 *   1) 不清 CNT —— 清了就是停机重来；
 *   2) SET 不能压到 CNT 上 —— 中断的守卫是 CNT >= SET 就关通道，
 *      那是"纠偏"变成"急停"。
 * 整个读改写用 __disable_irq() 包住，和 SetPulse / Stop 保持同一套规矩。 */
uint8_t StepMotor_AdjustTarget(uint8_t motor, int32_t delta)
{
    uint32_t set;
    uint32_t cnt;
    int32_t  next;

    if (motor < MOTOR1 || motor > MOTOR4) {
        return 1;
    }

    __disable_irq();

    set = Motor_PUL_SET[motor-1];
    cnt = Motor_PUL_CNT[motor-1];

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

    Motor_PUL_SET[motor-1] = (uint32_t)next;

    __enable_irq();
    return 0;
}

// ---------- 停止单个电机 ----------
void StepMotor_Stop(uint8_t motor)
{
    uint16_t enableBit = GetCCER_EnableBit(motor);
    __disable_irq();   // 同上，保证停机动作不会被中断抢掉
    TIM3->CCER &= ~enableBit;
    __enable_irq();
    Motor_PUL_SET[motor-1] = 0;
    Motor_PUL_CNT[motor-1] = 0;
    ramp_on = 0;                     // 被主动停掉，斜坡作废

}

// ---------- 停止所有电机 ----------
void StepMotor_StopAll(void)
{
    for (uint8_t i = 1; i <= 4; i++) {
        StepMotor_Stop(i);
    }
}

// ---------- 遥控点动的柔和停车：把"无限"改成"再走一段减速距离" ----------
// 直接 StopAll 会猛地一顿，这里复用已有的斜坡状态机把速度压下来。
// 幂等：已经是有限值的不动，所以看门狗超时后每毫秒调一次也没关系。
void StepMotor_StopJog(void)
{
    if (!ramp_on) {          // 没在起坡（本来就是慢档），别拖，直接停
        StepMotor_StopAll();
        return;
    }
    for (uint8_t i = 0; i < 4; i++) {
        if (Motor_PUL_SET[i] == JOG_PULSE_INFINITE) {
            Motor_PUL_SET[i] = Motor_PUL_CNT[i] + ramp_decel;
        }
    }
}

// ---------- TIM3更新中断处理函数 ----------
void TIM3_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) {
        // 1. 处理脉冲计数
        for (uint8_t i = 0; i < 4; i++) {
            if (Motor_PUL_SET[i] > 0) {
                Motor_PUL_CNT[i]++;
                if (Motor_PUL_CNT[i] >= Motor_PUL_SET[i]) {
                    uint16_t enableBit = GetCCER_EnableBit(i+1);
                    TIM3->CCER &= ~enableBit;
                    Motor_PUL_SET[i] = 0;
                }
            }
        }

        // 2. 软起软停：一档档推进 ARR，消掉起停瞬间的速度跳变
        if (ramp_on) {
            uint32_t remaining = 0;
            for (uint8_t i = 0; i < 4; i++) {
                if (Motor_PUL_SET[i] > 0) {
                    remaining = Motor_PUL_SET[i] - Motor_PUL_CNT[i];
                    break;
                }
            }
            if (remaining == 0) {
                ramp_on = 0;
            } else if (++ramp_period >= RAMP_SOFT_STEP_PULSE) {
                ramp_period = 0;
                if (ramp_state == RAMP_ACCEL) {
                    if (remaining <= ramp_decel) {
                        ramp_state = RAMP_DECEL;          // 行程不够，直接收尾
                    } else if (current_arr > speed_arr) {
                        uint16_t a = current_arr - RAMP_SOFT_STEP_ARR;
                        if (a < speed_arr) a = speed_arr;
                        StepMotor_SetFrequency(a);
                    } else {
                        ramp_state = RAMP_CRUISE;
                    }
                } else if (ramp_state == RAMP_CRUISE) {
                    if (remaining <= ramp_decel) ramp_state = RAMP_DECEL;
                } else {   // RAMP_DECEL
                    if (current_arr < RAMP_SOFT_ARR) {
                        uint16_t a = current_arr + RAMP_SOFT_STEP_ARR;
                        if (a > RAMP_SOFT_ARR) a = RAMP_SOFT_ARR;
                        StepMotor_SetFrequency(a);
                    }
                }
            }
        }

        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
    }
}

// ---------- 运动函数（设置运动类型） ----------

// 前进（直线）
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

// 后退（直线）
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

// 右转（原地旋转）
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

// 左转（原地旋转）
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

// 左移（直线运动，平移）
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

// 右移（直线运动，平移）
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
// 直线前进指定脉冲数（四个电机同步）
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

// 画线专用：匀速走直线，不做软起软停。
// 起坡段车速只有巡航的 1/1.75，而泵的出漆量恒定 —— 走得慢就喷得厚，
// 会让每条线的头尾 0.3m 明显比中段厚，所以画线这几段必须匀速。
void StepMotor_GoForwardFlat(uint16_t pulse)
{
    StepMotor_SetRampEnabled(0);
    StepMotor_GoForward(pulse);
    StepMotor_SetRampEnabled(1);
}

// 原地右转指定脉冲数（四个电机同向）
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


// 检查是否有任何电机仍在发送脉冲
uint8_t StepMotor_IsAnyRunning(void)
{
    for (uint8_t i = 0; i < 4; i++) {
        if (Motor_PUL_SET[i] != 0) {
            return 1;
        }
    }
    return 0;
}
