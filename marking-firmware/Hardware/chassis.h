#ifndef __CHASSIS_H
#define __CHASSIS_H

/* ============================================================================
 * 底盘车型配置中心
 * ============================================================================
 *
 * 一套源码，两套固件：
 *     Mec_Car   Target，Define 里带 MEC_CAR    -> 麦轮底盘（可横移）
 *     4WD_Car   Target，Define 里带 _4WD_CAR   -> 四轮差速（普通橡胶轮，不能横移）
 *
 * 车型不是在代码里改的，是改 Keil 工程 Target 下拉框。这和割草机 V650 那套
 * 做法一样（那个工程有 Akm_Car / Diff_Car / Mec_Car / 4WD_Car / Omni_Car
 * 五个 Target，文件列表完全相同，只有 Define 差一个宏）。
 *
 * 为什么不用运行时判断（比如 robot.type = 6）：
 *   1. 麦轮的横移代码在差速车上根本不该存在。运行时判断只能"拒绝执行"，
 *      代码还是编进去了，占 Flash、占栈、还得在每个调用点记得检查返回值。
 *   2. 编译期裁掉，调用点就不存在了。忘检查返回值这件事从根上消失了。
 *   3. 两套固件可以同时产出、同时归档。
 *
 * 规则：车型差异只允许在这个文件里出现一次。motion.c / system.c 里只写
 *       #if CHASSIS_HAS_MECANUM，不写 #if defined(MEC_CAR)。这样以后加
 *       第三种底盘（比如两轮差速）时，改动范围只有这个文件。
 * ==========================================================================*/

#if defined(MEC_CAR) && defined(_4WD_CAR)
#error "MEC_CAR 与 _4WD_CAR 同时定义。Target 的 Define 里只能留一个。"
#endif

#if !defined(MEC_CAR) && !defined(_4WD_CAR)
#error "没有选车型。Target 的 C/C++ -> Define 里必须加 MEC_CAR 或 _4WD_CAR。"
#endif

#include "calib.h"

/* 两种车型都是四个步进电机，左右各两个。
 * 数量相同不代表代码相同 —— 差速车左右两组必须严格同脉冲，
 * 麦轮四轮可以各不相同（横移就是靠这个）。 */
#define CHASSIS_MOTOR_COUNT     4

/* ---------- 左右两组按脉冲比例同步（0 = 关，1 = 开） ----------
 *
 * 开：左右两组的脉冲数不同时（转弯、纠偏、左右补轮径），给脉冲少的那组
 *     按比例放大 ARR，让两侧在同一时刻到达。走出来的是一段连续圆弧。
 *
 * 关：两组同频，脉冲少的那一侧先停，剩下那段路只有外侧轮在推 ——
 *     车头会在段末额外拐一下，这就是改造前"拐角处线跑偏"的成因。
 *
 * 为什么可以放心开：四路现在已经各自有独立时基（TIM3/TIM2/TIM5/TIM8），
 * 改一组的 ARR 不影响另一组。以前四路共用一个 TIM3 的 ARR，物理上做不到。
 *
 * 留着这个开关是为了现场对比：万一某个机械状态下同步反而更别扭，
 * 改成 0 重编一次就退回原来的行为，不用回滚代码。 */
#define CHASSIS_SYNC_ARR        1

/* 同步死区（千分比）。两侧脉冲差不到这个比例就当作一样，统一用用户档位。
 *
 * 为什么需要：ARR 是整数，差 1~2 对同步没有任何实际作用，却会让每开一段
 * 都改写一次寄存器。视觉伺服是 200mm 一段连续调用，段段都改的话，
 * 频率在段边界上会有一拍不稳。8‰ 在 500mm 轮距上约等于 4mm 的左右差。 */
#define CHASSIS_SYNC_DEADBAND_PM 8

#if defined(MEC_CAR)

/* ------------------------------------------------------------ 麦轮（MEC） */
#define CHASSIS_NAME            "MEC"
#define CHASSIS_HAS_MECANUM     1

/* 转弯速度：0 = 不额外限速，用当前档位。
 * 麦轮原地转时四个轮子都在横滑，摩擦小、不吃力，不需要降速。 */
#define CHASSIS_TURN_ARR        0

/* 转角补偿。麦轮原地转有轻微横滑，实测 90 度指令大约转出 88~89 度。
 * 1.0 = 不做补偿，把误差统一交给 CALIB_TURN_RADIUS_MM 吸收。 */
#define CHASSIS_TURN_SCALE      1.000f

/* 单次纠偏上限和段内累计上限。麦轮能横滑，差速不会让轮子互顶，
 * 可以纠得狠一点、久一点。 */
#define CHASSIS_TRIM_MAX_MM       20.0f
#define CHASSIS_TRIM_ACCUM_MAX_MM 200.0f

/* 轮径补偿系数：四个轮子各管各的。
 * 麦轮四轮独立打滑，逐轮标定是有意义的。 */
#define CHASSIS_K_LEFT_FRONT    CALIB_WHEEL_K1
#define CHASSIS_K_RIGHT_FRONT   CALIB_WHEEL_K2
#define CHASSIS_K_LEFT_REAR     CALIB_WHEEL_K3
#define CHASSIS_K_RIGHT_REAR    CALIB_WHEEL_K4

#else   /* _4WD_CAR */

/* -------------------------------------------------- 四轮差速（普通橡胶轮） */
#define CHASSIS_NAME            "4WD"
#define CHASSIS_HAS_MECANUM     0

/* 转弯速度上限（ARR，越大越慢）。0 = 不限。
 *
 * 为什么差速车要单独降速：普通橡胶轮横向不能滑移，四个轮子做原地转时
 * 是在地面上"拧"，轮胎要发生横向变形才能转过去。速度越快，单位时间要
 * 拧掉的横向位移越多，轮胎就越是"攒一下、弹一下"地跳 —— 这就是
 * "转的时候不顺畅"的机械侧原因，代码改不掉，只能靠降速让它柔一点。
 *
 * 900 ≈ 1.1kHz ≈ 220mm/s。原地转半径 382mm，角速度约 33 度/秒，
 * 转 90 度约 2.7 秒。慢，但画车位本来就不赶时间。 */
#define CHASSIS_TURN_ARR        900

/* 转角补偿。普通轮原地转的拖胎比麦轮严重，实测转不够。
 * 1.05 = 多给 5% 脉冲。现场量法：转 90 度，用量角器看实际转了多少，
 * 实际 85 度就改成 90/85 = 1.059。 */
#define CHASSIS_TURN_SCALE      1.050f

/* 纠偏上限比麦轮小。
 *
 * 差速车做左右差速时，多出来的那点行程只能靠轮胎横向变形消化 ——
 * 普通橡胶轮不能横滑。脉冲数会以为"都走到了"，但车身里攒的扭动不会消失，
 * 而且会盖过纠偏本身想修的那点角度。麦轮能横滑，这个积累效应小得多。
 *
 * 80mm 不是"更快修好"，是"别修太久才去重新对一次线"：到 80mm 就该发
 * 新的一段 MOVE 重新对齐，而不是在这一段里继续硬纠。 */
#define CHASSIS_TRIM_MAX_MM       20.0f
#define CHASSIS_TRIM_ACCUM_MAX_MM  80.0f

/* 轮径补偿系数：左右各一个，组内强制相同。
 *
 * 这是两套固件在数值上最实质的差别。普通轮左右两侧速度不等时，
 * 多出来的行程必须靠打滑消化，而哪个轮子先打滑是不确定的 ——
 * 表现就是车身莫名其妙扭一下。所以差速车不能用四个独立系数，
 * 只能左组一个、右组一个：宁可整体差 2%，也不要组内差 0.5%。
 *
 * 左组取 K1（左前）和 K3（左后）中更接近 1 的那个，是现场选择：
 * 两个都偏离 1 说明有轮子装错了，先查机械再谈补偿。 */
#define CHASSIS_K_LEFT_FRONT    CALIB_WHEEL_K1
#define CHASSIS_K_RIGHT_FRONT   CALIB_WHEEL_K2
#define CHASSIS_K_LEFT_REAR     CALIB_WHEEL_K1
#define CHASSIS_K_RIGHT_REAR    CALIB_WHEEL_K2

#endif

#endif /* __CHASSIS_H */
