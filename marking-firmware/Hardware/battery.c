#include "battery.h"

extern volatile uint32_t timer_counter;

/* 一次采 8 个样取平均。单个样本的量化噪声是 ±1LSB ≈ ±9mV，
 * 8 个平均之后落到 ±3mV，比我们关心的"12.4 还是 12.1"细得多。 */
#define BATT_OVERSAMPLE     8u

/* 线没接时 ADC 读到的是 0 或者几十个字的底噪。低于这个数就当没测到 ——
 * 报 0.0V 会让人去充电，而真正的问题是分压线掉了。 */
#define BATT_RAW_MIN        16u

/* 等 EOC 的循环上限。给的余量很大，只用来兜底 ADC 真的不响应的情况。 */
#define BATT_EOC_TIMEOUT    4000u

static uint16_t batt_mv;
static uint8_t  batt_valid;
static uint32_t batt_ms;

/* ADC 单次转换，阻塞到 EOC。
 *
 * 阻塞时间 = (239.5 + 12.5) / 12MHz ≈ 21微秒，8 次 168微秒，500ms 才一次。
 * 放在主循环里不影响脉冲输出：四路脉冲在自己的定时器中断里发，ADC 转换
 * 期间中断照常响应，只是主循环这一圈慢了一点。
 *
 * 采样时间取最长的 239.5 周期，是因为分压源阻抗有 9k 左右。手册给的
 * 表格里，源阻抗超过 10k 就要加长采样时间，短了读数会系统性偏低。 */
static int16_t Batt_ReadOnce(void)
{
    uint32_t wait;

    /* 先清掉上一次的 EOC。上一次如果是超时退出的，它那次转换迟到的结果
     * 会留在这个标志上 —— 不清的话，下面一启动就看到它，读到的是上一次
     * 那个没等到结果的样本。 */
    ADC_ClearFlag(ADC1, ADC_FLAG_EOC);

    ADC_RegularChannelConfig(ADC1, BATT_ADC_CHANNEL, 1, ADC_SampleTime_239Cycles5);
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);

    /* 等 EOC，带超时。ADC 不响应的时候，宁可这一次读数作废，也不能把
     * 主循环卡死在这儿 —— 那等于整台车死机，而电池电压只是顺手看看的量。 */
    wait = BATT_EOC_TIMEOUT;
    while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET) {
        if (--wait == 0u) {
            return -1;
        }
    }
    return (int16_t)ADC_GetConversionValue(ADC1);   /* 读 DR 顺带清 EOC */
}

void Battery_Init(void)
{
    GPIO_InitTypeDef gpio;
    ADC_InitTypeDef  adc;

    RCC_APB2PeriphClockCmd(BATT_GPIO_CLK | RCC_APB2Periph_ADC1, ENABLE);

    /* ADCCLK 上限 14MHz，PCLK2 是 72MHz，只能 6 分频到 12MHz。 */
    RCC_ADCCLKConfig(RCC_PCLK2_Div6);

    gpio.GPIO_Pin   = BATT_GPIO_PIN;
    gpio.GPIO_Mode  = GPIO_Mode_AIN;     /* 模拟输入，数字输入缓冲断开 */
    gpio.GPIO_Speed = GPIO_Speed_50MHz;  /* 模拟输入用不到，填上只为结构体不留野值 */
    GPIO_Init(BATT_GPIO_PORT, &gpio);

    ADC_DeInit(ADC1);
    adc.ADC_Mode               = ADC_Mode_Independent;
    adc.ADC_ScanConvMode       = DISABLE;
    adc.ADC_ContinuousConvMode = DISABLE;
    adc.ADC_ExternalTrigConv   = ADC_ExternalTrigConv_None;
    adc.ADC_DataAlign          = ADC_DataAlign_Right;
    adc.ADC_NbrOfChannel       = 1;
    ADC_Init(ADC1, &adc);
    ADC_Cmd(ADC1, ENABLE);

    /* 数据手册要求 ADC 上电后等 tSTAB 再开始校准。这里空转，不靠
     * 前面那几条指令的耗时去凑。 */
    {
        volatile uint32_t i;
        for (i = 0; i < 200u; i++) {
        }
    }

    /* 校准不能省。不做的话零点和增益都停在出厂默认，读数能偏几十毫伏，
     * 而且每次上电偏的方向一样 —— 看着像分压电阻不准，实际白换一次电阻。 */
    ADC_ResetCalibration(ADC1);
    while (ADC_GetResetCalibrationStatus(ADC1) == SET) {
    }
    ADC_StartCalibration(ADC1);
    while (ADC_GetCalibrationStatus(ADC1) == SET) {
    }
}

void Battery_Poll(void)
{
    uint32_t sum = 0;
    uint32_t mv;
    uint16_t raw;
    uint8_t  i;

    if ((uint32_t)(timer_counter - batt_ms) < BATT_SAMPLE_PERIOD_MS) {
        return;
    }
    batt_ms = timer_counter;

    for (i = 0; i < BATT_OVERSAMPLE; i++) {
        int16_t one = Batt_ReadOnce();

        /* 有一次没读到就整轮作废。要是拿 0 顶上，8 次里错 1 次就能把
         * 平均值拉下一大截，直接跨过低电压门限 —— 车上开始叫，而电池
         * 是好的。这种误报比没有读数更坏。 */
        if (one < 0) {
            batt_mv    = 0;
            batt_valid = 0;
            return;
        }
        sum += (uint32_t)one;
    }
    raw = (uint16_t)(sum / BATT_OVERSAMPLE);

    if (raw < BATT_RAW_MIN) {
        batt_mv    = 0;
        batt_valid = 0;
        return;
    }

    /* raw/4095 × 3300mV = 引脚电压，再乘 (TOP+BOT)/BOT 还原电池电压。
     * 全整数：4095 × 3300 = 13.5M，再乘 11 是 148M，32 位放得下。 */
    mv = (uint32_t)raw * 3300u / 4095u;
    mv = mv * (BATT_DIV_TOP_OHM + BATT_DIV_BOT_OHM) / BATT_DIV_BOT_OHM;
    if (mv > 65535u) {
        mv = 65535u;
    }

    batt_mv    = (uint16_t)mv;
    batt_valid = 1;
}

uint16_t Battery_GetMv(void)
{
    return batt_mv;
}

uint8_t Battery_IsValid(void)
{
    return batt_valid;
}

uint8_t Battery_IsLow(void)
{
    return (uint8_t)(batt_valid && batt_mv < BATT_LOW_MV);
}

uint8_t Battery_IsCritical(void)
{
    return (uint8_t)(batt_valid && batt_mv < BATT_CRIT_MV);
}
