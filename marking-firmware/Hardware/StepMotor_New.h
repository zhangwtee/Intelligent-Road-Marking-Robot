#ifndef __STEP_MOTOR_NEW_H
#define __STEP_MOTOR_NEW_H

#include "stm32f10x.h"

/*左前轮：左旋轮
右前轮：右旋轮
左后轮：右旋轮
右后轮：左旋轮*/
#define MOTOR1  1
#define MOTOR2  2
#define MOTOR3  3
#define MOTOR4  4

// 连续运动（手机遥控点动）：一直走到收到停止指令，不设脉冲上限。
// 不是真"无限" —— 4294967295 个脉冲在最高档也要 15 天才数完，实际等于无限。
// 兜底靠 system.c 里的遥控看门狗超时停车（蓝牙断线 / 人松手）。
#define JOG_PULSE_INFINITE  0xFFFFFFFFu

//// 方向定义基础方向宏
#define DIR_CW      0   // 正转（顺时针）
#define DIR_CCW     1   // 反转（逆时针）

// 根据实测：前轮向前用当前值，后轮向前需要取反
#define M1_FWD   DIR_CCW   // 左前
#define M2_FWD   DIR_CW    // 右前
#define M3_FWD   DIR_CW    // 左后（修正）
#define M4_FWD   DIR_CCW   // 右后（修正）

// 后退方向 = 前进方向取反
#define M1_REV   DIR_CW
#define M2_REV   DIR_CCW
#define M3_REV   DIR_CCW
#define M4_REV   DIR_CW
/* 脉冲输出：每路一个定时器一个通道（ZET6 之后）
 *
 * 以前四路全挂在 TIM3 的 CH1~CH4 上，共用一个 ARR，所以只能同频。
 * 换到 ZET6 之后分给四个定时器，各自有自己的 ARR —— 差速转弯时内侧
 * 可以真正降速，而不是"停在那等外侧走完"。
 *
 *   M1 左前  TIM3_CH1  PA6    （不变）
 *   M2 右前  TIM2_CH1  PA0    （原 PA7）
 *   M3 左后  TIM5_CH2  PA1    （原 PB0）
 *   M4 右后  TIM8_CH1  PC6    （原 PB1）
 *
 * M3 为什么是 CH2：F103 上 TIM5_CH1 和 TIM2_CH1 都是 PA0，已经被 M2 占了，
 * PA1 对应的是 TIM5_CH2。所以四路的通道号不统一，驱动里按路记着分派。
 *
 * 换线之后，PA7 / PB0 / PB1 三个脚空出来了，可以留给别的用途。 */

// 电机1：DIR=PA8, PUL=PA6 (TIM3_CH1)
#define M1_DIR_PORT     GPIOA
#define M1_DIR_PIN      GPIO_Pin_8
#define M1_PUL_PORT     GPIOA
#define M1_PUL_PIN      GPIO_Pin_6

// 电机2：DIR=PA12, PUL=PA0 (TIM2_CH1)
#define M2_DIR_PORT     GPIOA
#define M2_DIR_PIN      GPIO_Pin_12
#define M2_PUL_PORT     GPIOA
#define M2_PUL_PIN      GPIO_Pin_0

// 电机3：DIR=PA11, PUL=PA1 (TIM5_CH2)
#define M3_DIR_PORT     GPIOA
#define M3_DIR_PIN      GPIO_Pin_11
#define M3_PUL_PORT     GPIOA
#define M3_PUL_PIN      GPIO_Pin_1

// 电机4：DIR=PB12, PUL=PC6 (TIM8_CH1)
#define M4_DIR_PORT     GPIOB
#define M4_DIR_PIN      GPIO_Pin_12
#define M4_PUL_PORT     GPIOC
#define M4_PUL_PIN      GPIO_Pin_6


// 新增函数
uint8_t StepMotor_IsAnyRunning(void);           // 检查是否有电机在输出脉冲
void StepMotor_GoForward(uint16_t pulse);       // 指定脉冲数前进
void StepMotor_TurnRight(uint16_t pulse);       // 指定脉冲数右转

void StepMotor_Init(void);                         // 初始化所有电机（TIM3）
void StepMotor_SetDirection(uint8_t motor, uint8_t dir); // 设置电机方向
void StepMotor_SetPulse(uint8_t motor, uint32_t pulse);  // 设置电机脉冲数（启动输出）
void StepMotor_Stop(uint8_t motor);//电机停止转动
void StepMotor_StopAll(void);//全部停止
void StepMotor_StopJog(void);   // 遥控点动的柔和停车（减速跑完再停）
void StepMotor_SetRampEnabled(uint8_t en);      // 0 = 本次运动不起坡
void StepMotor_GoForwardFlat(uint16_t pulse);   // 匀速前进（画线用，不做软起软停）
void StepMotor_SpeedUp(void);   // '+'：提速一档（改用户档位）
void StepMotor_SpeedDown(void); // '-'：降速一档（改用户档位）

/* 直接设用户档位。帧命令 CMD_SPEED 走这里：上位机给的是绝对 ARR，
 * 不是"加一档"，因为现场要的是"回到 300"而不是"按七下减号"。
 *
 * 越界值夹到 150~1500，不报错 —— 判越界在协议层做（proto.h 的
 * PROTO_SPEED_MIN/MAX_ARR），这里只是兜底。
 *
 * 调用前提：没在一段"左右不同速"的运动里。段内调用会把四路拉成同一个
 * ARR，DriveSyncArr 按脉冲比例定好的左右比例当场失效。system.c 的
 * SpeedChangeAllowed 负责这个前提。 */
void     StepMotor_SetUserSpeed(uint16_t arr);

/* 直接设置"当前生效档位"，不动用户档位。
 *
 * 唯一的正式用途是临时限速：差速底盘原地转时，普通轮胎横向不能滑移，
 * 转快了会"攒一下、弹一下"，所以要压到 CHASSIS_TURN_ARR。
 * 转完之后由 motion 层调 StepMotor_RestoreSpeed() 回到用户档位。
 *
 * 为什么要有"用户档位"这个概念：只有一个 speed_arr 的话，转弯把它覆盖掉，
 * 之后车就永远停在慢档上，而界面上显示的档位还是原来那个 ——
 * 用户会以为电机坏了。 */
void     StepMotor_SetSpeedTarget(uint16_t arr);
void     StepMotor_RestoreSpeed(void);
uint16_t StepMotor_GetSpeedTarget(void);

/* 用户档位（speed_base），即"没被转弯临时限速覆盖的那个"。
 *
 * 差速算同步 ARR 时必须以它为基准：上一段如果是原地转，四路 ARR 已经被
 * 压到 CHASSIS_TURN_ARR，拿 GetSpeedTarget() 当基准的话，下一段直线的
 * 速度会被转弯的限速污染，越转越慢。 */
uint16_t StepMotor_GetSpeedBase(void);

/* ---------- 分路设速度：四路独立之后才有的能力 ----------
 *
 * SetSpeedLR   左右两组分开设。差速转弯用它：内侧 ARR 调大（变慢），
 *              两组按同一段时间算，车走的是连续圆弧而不是"先停后走"。
 *              左组 = M1 + M3，右组 = M2 + M4。
 *
 * SetSpeedOf   单路设。四轮各自补轮径时用（差速模式下同侧两轮必须同速，
 *              这个只在麦轮或特殊标定时才调）。
 *
 * 两者都不动用户档位（speed_base），SetSpeedTarget / RestoreSpeed 会覆盖它们。
 * 注意 SetSpeedLR 会把"当前档位"（speed_arr）记成左组的值：左右不同速时
 * GetSpeedTarget() 没有唯一答案，它只用于界面上显示"大概多快"。 */
void     StepMotor_SetSpeedLR(uint16_t left_arr, uint16_t right_arr);
void     StepMotor_SetSpeedOf(uint8_t motor, uint16_t arr);
uint16_t StepMotor_GetSpeedOf(uint8_t motor);

/* 当前这段运动已经发出的脉冲数 / 本段的目标脉冲数。
 * 给上行状态帧用：上位机第一次能知道"它走到哪了"，而不是只知道"我发了"。
 * 各路自己的中断里只增自己的 CNT，读它们是单个 32 位对齐读，在 Cortex-M3 上原子。
 * GetPulse / GetTarget 取 MOTOR1 作代表；转弯时左右脉冲数不同，要分开看就用 Of 版本。 */
uint32_t StepMotor_GetPulse(void);
uint32_t StepMotor_GetTarget(void);
uint32_t StepMotor_GetPulseOf(uint8_t motor);

/* 本段还剩多少脉冲（可能为负 —— 纠偏把目标减到"已经走过头"时，
 * 中断会尽快收尾，不是错误）。
 *
 * 给 motion 层算左右同步比例用：段首按总脉冲算好的 ARR 比例，在段中间
 * 被 Motion_Trim 改过目标之后就过期了，必须按"剩余"重算，否则一次纠偏
 * 就把同步打回"一侧先停"。
 *
 * 遥控点动（无限行程）返回 0x7FFFFFFF，调用方按"忽律比例"处理。 */
int32_t  StepMotor_GetRemainOf(uint8_t motor);

/* 这一路的目标还能往回减多少脉冲（0 = 减不动了）。
 *
 * 纠偏的差速靠"一侧多走、另一侧少走"实现，而少走的那一侧目标不能压到
 * CNT+1 以下 —— 再往下就是让那个轮子当场停下，不是纠偏。这条上限和
 * StepMotor_AdjustTarget 里的夹位是同一条规则，所以放在这个模块里：
 * motion 层只取两侧的较小值，不再自己重算一遍。
 *
 * 空闲、遥控点动、参数非法都返回 0（本来就不该被纠偏改目标）。 */
int32_t  StepMotor_TrimRoom(uint8_t motor);
/* ---------- 运动中调整本段目标脉冲（不清进度） ----------
 *
 * 这是视觉纠偏能成立的前提，也是它和 StepMotor_SetPulse 的唯一区别。
 *
 * SetPulse 每次调用都把 CNT 清零：拿它做"续段"，正在跑的那一路会先被判
 * 完成、通道关掉、斜坡重启。画出来的线在每一个续段点都是一次停顿，
 * 停顿时泵还在出漆 —— 线上就是一串疙瘩。
 *
 * 这里只改 SET，CNT 原样保留。中断里 `CNT++ >= SET` 的守卫不会命中，
 * 通道一直开着，脉冲均匀连续，转向靠的是左右轮目标差，不是启停差。
 *
 * delta  正 = 多走，负 = 少走（单位：脉冲）
 * 返回   0 = 该路正在运动，已调整；1 = 该路空闲（或参数非法），没动手。
 *        上位机据此知道"纠偏有没有真的落下去"，而不是发完就当成功。
 */
uint8_t StepMotor_AdjustTarget(uint8_t motor, int32_t delta);

void StepMotor_APPROACH(void);
void StepMotor_RETREAT(void);
void StepMotor_You(void);
void StepMotor_Zuo(void);
void StepMotor_MOVE_LEFT(void);
void StepMotor_MOVE_RIGHT(void);
#endif
