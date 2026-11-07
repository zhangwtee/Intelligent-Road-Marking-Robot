#ifndef __BATTERY_H
#define __BATTERY_H

#include "stm32f10x.h"

/* ==========================================================================
 * 电池电压采样（ADC1_IN10 = PC0）
 *
 * 为什么是 PC0 而不是 PA7：
 *   PA7 是 C8 版 M2 的脉冲脚。换四路独立时基之后它空出来了，但那根线在
 *   大多数车上还接着步进驱动器 —— 接上去读回来的是脉冲方波，不是电池电压。
 *   PC0 全仓没有任何人用（GPIOC 只占了 PC6 给 M4_PUL），干净。
 *
 * 为什么要分压：
 *   ADC 引脚耐压 3.3V，电池是 12V/24V 级别，直连会把脚打死。R_TOP 接电池
 *   正极、R_BOT 接地，ADC 取中间节点。
 *
 *   R_TOP = 100k、R_BOT = 10k → 分压比 11:1
 *     12V → 1.09V    24V → 2.18V    36V → 3.27V（再高就顶到量程了）
 *     分辨率 3300/4095*11 ≈ 8.9mV
 *
 *   换电阻只改下面两个宏。R_BOT 别超过 20k：源阻抗（R_TOP//R_BOT）越大，
 *   采样保持电容越充不满，读数偏低，而且随温度、湿度漂。
 *
 * 分压电阻和电池之间那颗 100nF 也不要省：步进驱动器的开关噪声会直接从
 * 电源线耦合进来，单次读数能差上百毫伏，靠软件多次平均去不掉低频的那部分。
 * ========================================================================== */

#define BATT_ADC_CHANNEL    ADC_Channel_10
#define BATT_GPIO_PORT      GPIOC
#define BATT_GPIO_PIN       GPIO_Pin_0
#define BATT_GPIO_CLK       RCC_APB2Periph_GPIOC

#define BATT_DIV_TOP_OHM    100000u
#define BATT_DIV_BOT_OHM    10000u

/* 低电压门限。这几个数是按「12V 铅酸」取的：
 *   静置满电 12.8V、剩一半 12.1V、该收工 11.5V、负载下塌到 10.8V 就危险。
 * 换 3 串锂电（满 12.6、空 9.9）或 24V 系统都要重新取，别照抄。 */
#define BATT_LOW_MV         11500u
#define BATT_CRIT_MV        10800u

/* 采样周期。电池电压是慢变量，500ms 已经比任何负载变化都快了。 */
#define BATT_SAMPLE_PERIOD_MS   500u

/* ---- 怎么判"这个脚上根本没接电池" ----
 *
 * 分压电阻没焊的时候 PC0 是悬空的，ADC 读回来不是 0，是到处乱跳的一串
 * 数。按"读到的就是电压"去显示，界面上会出现一个 8.3V 或者 21.6V，看着
 * 像一块快没电的电池 —— 而人端着万用表去量，电池是满的。
 *
 * 三个条件任一成立就判"没接"（见 battery.c 的 Battery_Poll）：
 *   1) 换算出来低于 BATT_MIN_MV：真电池不会低于 3V，低于它就是没接或者
 *      线掉了，这时显示 0.0V 会把人引去充电；
 *   2) 高于 BATT_MAX_MV：分压比配错、或者引脚被别的信号串进来了。
 *      33V 是给 24V 系统留的余量（浮充 28.8V），再高就不是这块电池。
 *   3) 最近 8 次采样的极差超过 BATT_UNSTABLE_MV：悬空脚就是这样跳的。
 *      阈值取 2V 而不是几百毫伏，是因为电机启停时真电池也会掉一截压，
 *      那个要放过 —— 它落在合理范围内，不是"没接"。
 *
 * 8 次 × 500ms = 4 秒。上电头 4 秒不足 8 个样本，只按前两条判。 */
#define BATT_MIN_MV         3000u
#define BATT_MAX_MV         33000u
#define BATT_UNSTABLE_MV    2000u
#define BATT_HISTORY        8u

/* 低压提示音的节奏：每 30 秒「嘀」两声。长鸣不行 —— 状态一切换就会
 * Buzzer_Off，两声短促的反而听得见，也不至于吵得人把它当成故障。 */
#define BATT_ALARM_PERIOD_MS    30000u
#define BATT_CRIT_ALARM_PERIOD_MS 8000u   /* 低于 BATT_CRIT_MV 之后响得更勤 */
#define BATT_CHIRP_ON_MS        80u
#define BATT_CHIRP_GAP_MS       120u

void     Battery_Init(void);
void     Battery_Poll(void);
uint16_t Battery_GetMv(void);
uint8_t  Battery_IsValid(void);
uint8_t  Battery_IsLow(void);
/* 1 = 引脚上没有电池（分压没接 / 线掉了）。和 IsValid() == 0 不是一回事：
 * 那个是"这一轮没读到"，这个是"读到了，但读到的不是电池"。 */
uint8_t  Battery_IsAbsent(void);
uint8_t  Battery_IsCritical(void);

#endif
