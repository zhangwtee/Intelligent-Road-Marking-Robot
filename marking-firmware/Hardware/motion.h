#ifndef __MOTION_H
#define __MOTION_H

#include "stm32f10x.h"

/* 底盘车型（麦轮 / 四轮差速）由 chassis.h 统一决定，见那里的说明。
 * 这一层只关心 CHASSIS_HAS_MECANUM 这类能力宏，不直接判 MEC_CAR ——
 * 以后加第三种底盘时，改动只在 chassis.h 里。 */
#include "chassis.h"

/*
 * 运动学抽象层
 * ============================================================================
 * 这一层存在的唯一目的：让上层永远不用写"脉冲数"。
 *
 * 改造前：
 *     StepMotor_GoForwardFlat(26500);     // 这是什么？多长时间？多远？
 *     StepMotor_TurnRight(3000);          // 这是几度？
 *
 * 改造后：
 *     Motion_ForwardFlat(5300.0f);        // 5.3 米
 *     Motion_TurnRight(90.0f);            // 90 度
 *
 * 换轮子、改细分、改驱动器闭环模式，只改 calib.h 三个数，这一层以上的
 * 调用代码一行不动。
 * ============================================================================
 */

/* 把标定参数装进模块并预计算换算系数。
 * 必须在第一次调用任何 Motion_xxx 之前调用一次。
 * 内部只做一次浮点除法，之后全是乘法（F103 无 FPU，浮点除法很贵）。 */
void Motion_Init(void);

/* ---------- 纯换算（不驱动电机，可用于校验） ---------- */
uint32_t Motion_MmToPulse(float mm);
uint32_t Motion_DegToPulse(float deg);
float    Motion_PulseToMm(uint32_t pulse);
float    Motion_PulseToDeg(uint32_t pulse);

/* ---------- 运动指令 ---------- */
void Motion_Forward(float mm);        /* 带软起软停，用于普通移动 */
void Motion_Backward(float mm);       /* 带软起软停，用于普通后退 */
void Motion_ForwardFlat(float mm);    /* 匀速无斜坡，用于喷漆段（漆量随速度变） */
void Motion_TurnRight(float deg);     /* 原地右转，四轮同向（左反右正） */

/* ---------- 视觉纠偏 ---------- */

/* 运动中做一次差速修正：delta_mm > 0 = 让车头向右，< 0 = 向左。
 *
 * 和 Motion_Forward 的根本区别：它不改"这段走多远"，只改"左右两边各自
 * 还差多少"。所以它必须在某一段正在走的时候调用 —— 车是靠左右轮目标差
 * 转向的，不是靠停下来再起步。停一次，泵就在原地出一摊漆。
 *
 * 内部带限幅（MOTION_TRIM_MAX_MM = CHASSIS_TRIM_MAX_MM）和死区：|delta| 太小直接返回，不用
 * 让四个轮子做无意义的脉冲增减。
 *
 * 返回 1 = 至少有一路真的被调整了（说明车正在走）；
 *      0 = 四路全空闲，什么也没发生；
 *      2 = 这一段的累计纠偏已经到顶，这次没执行。
 *
 * 调用方必须区分这三种：0 是"车没在走"（该回 ERR_STATE），2 是"该重新
 * 对齐了"（该回 ERR_RANGE，上位机收到之后应该发一条新的 MOVE 段而不是
 * 继续硬纠）。都返回 0 的话，上层只能看到"没生效"，不知道为什么。
 */
uint8_t Motion_Trim(float delta_mm);

/* 单段内累计纠偏量（毫米）的硬上限。
 *
 * 为什么必须有：四个轮子共用一个 ARR，差速只能表现为"内侧轮提前停"。
 * 一段 5 米里如果一直往同一个方向纠，内侧轮会越停越早 —— 累计到 1 米时，
 * 车已经在原地转了 90 度，而每一步单独看都只是"小小的修正"。
 *
 * 200mm 的取法：正常纠偏是双向的（左偏了往右纠，右偏了往左纠），累计量
 * 在零附近来回，远到不了这个数；能到这个数说明这一段本身就该结束了，
 * 该重新对齐（发新 MOVE 段）而不是继续硬纠。
 *
 * 这个上限只保护"段内"，跨段会清零 —— 见 Motion_DriveLR 里的说明。
 */
/* 数值按车型分：麦轮 200mm，差速车 80mm（见 chassis.h 的说明）。
 * 差速车每做一次左右差速，内侧轮都要"硬停"一下，累计多了车身会积累扭动。 */
#ifndef MOTION_TRIM_ACCUM_MAX_MM
#define MOTION_TRIM_ACCUM_MAX_MM   CHASSIS_TRIM_ACCUM_MAX_MM
#endif

/* 左右轮分别走不同距离，一次设定（不是运动中的增量）。
 * 只在调试和开环验证时用：正常画线不该走这条路 —— 它走完就停。
 * forward = 0 时左右方向同时取反。 */
void Motion_DriveLR(float left_mm, float right_mm, uint8_t forward);

#if CHASSIS_HAS_MECANUM
/* 横移：只有麦轮底盘有这个接口。
 *
 * 差速车上这个函数根本不存在 —— 不是"存在但返回失败"。这是两套固件
 * 真正的区别：system.c 那边的调用点在差速版里被 #if 掉了，编出来的固件
 * 里没有这条路径，也就不存在"忘了检查返回值"这种 bug。
 * 返回 0 = 已执行，1 = 还没 Init 或行程非法。 */
uint8_t Motion_MoveLeft(float mm);
uint8_t Motion_MoveRight(float mm);
#endif

/* 当前底盘是否具备横移能力（编译期常量，恒等于 CHASSIS_HAS_MECANUM）。
 *
 * 保留这个函数是为了让"不知道自己在哪个车型上"的代码（比如自检流程）
 * 还能问一句。业务代码不要用它做 if 分支 —— 那种分支在编译期就能消掉，
 * 留着它只是把编译期问题推到了运行时。 */
uint8_t Motion_HasMecanum(void);

/* ---------- 状态查询 ---------- */
uint8_t Motion_IsAnyRunning(void);
uint8_t Motion_IsReady(void);         /* 0 = 还没 Motion_Init */

#endif
