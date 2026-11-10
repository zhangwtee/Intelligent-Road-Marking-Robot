#include "stm32f10x.h"                  // Device header
#include <stdio.h>
#include <stdarg.h>
#include "uart_ring.h"

/* --------------------------------------------------------------------------
 * 两路串口：接收和发送各一个环
 *
 * 接收：中断塞字节、主循环取。原来是【一个字节变量 + 一个标志位】，中断写、
 * 主循环读，同时只能记住一个字节；OLED 刷一屏（软件 I2C，80~100ms）、舵机
 * 插值时主循环会停住，这期间到的字节，后一个直接把前一个覆盖掉。
 *
 * 发送：主循环塞字节、TXE 中断取。以前是阻塞发 —— 一帧 10 字节在 9600 下要
 * 10.4ms，状态帧 5Hz + 航向帧 5Hz，一秒有一百多毫秒在等 TXE，收字节和状态机
 * 推进全被推后。现在这一圈只花几十微秒，线上还是 10.4ms，但不等它。
 *
 * 两条路都是单生产者 / 单消费者，所以不需要临界区；具体见 uart_ring.h 顶部。
 * -------------------------------------------------------------------------- */
static UartRing rx1;    /* USART1 接收（蓝牙） */
static UartRing tx1;    /* USART1 发送：主循环放，TXE 中断取 */
static UartRing rx2;    /* USART2 接收（树莓派） */
static UartRing tx2;    /* USART2 发送，同上 */

void Serial1_Init(void)
{
    /* 先清缓冲，再开中断 —— 反过来的话，开中断到清缓冲之间收到的字节会被抹掉 */
    UartRing_Init(&rx1);
    UartRing_Init(&tx1);

    /*开启时钟*/
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);    // USART1时钟在APB2上
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);     // GPIOA时钟

    /*GPIO初始化*/
    GPIO_InitTypeDef GPIO_InitStructure;

    // PA9作为USART1_TX（复用推挽输出）
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;                 // PA9
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // PA10作为USART1_RX（浮空输入）
    // 上拉输入，不是浮空。对端（蓝牙模块 / 树莓派）没上电或者没插的时候，
    // 悬空的 RX 脚被电机和驱动器的干扰一耦合就能合成一个起始位，HandleCmd
    // 拿到一个任意字节照执行 —— 'U' 是前进，'P' 是开始画车位。
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;             // 上拉输入
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;                // PA10
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /*USART初始化*/
    USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_Init(USART1, &USART_InitStructure);                 // USART1

    /*中断配置*/
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);            // USART1接收中断

    /*NVIC配置*/
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;         // USART1中断通道
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_Init(&NVIC_InitStructure);

    /*USART使能*/
    USART_Cmd(USART1, ENABLE);                                // USART1
}

/* 把一个字节放进发送环，并保证 TXE 中断开着。
 *
 * 环满时等中断腾位置（9600 下一字节约 1ms），不直接写 DR —— 主循环和
 * 中断同时往 DR 写会丢字节，而丢一个字节就要靠对端 300ms 静默期重新同步。
 * Put 和"开 TXE 中断"放在同一个临界区里：分开做的话，中断可能刚好在
 * 中间把环抽空、顺手把 TXE 关掉，那个字节就永远留在环里了。
 *
 * 这条链第一次上电就该确认：接上上位机或串口助手，状态帧 5Hz 在滚就是通的；
 * 一帧都没有，先查 PA9 / PA2 的 TX 线和波特率，别去翻协议层。 */
static void Uart_TxPut(USART_TypeDef *u, UartRing *r, uint8_t b)
{
    for (;;) {
        __disable_irq();
        if (UartRing_Put(r, b)) {
            USART_ITConfig(u, USART_IT_TXE, ENABLE);
            __enable_irq();
            return;
        }
        __enable_irq();
        USART_ITConfig(u, USART_IT_TXE, ENABLE);   /* 环满：确保中断在抽 */
    }
}

void Serial1_SendByte(uint8_t Byte)
{
    Uart_TxPut(USART1, &tx1, Byte);
}

uint32_t Serial1_Pow(uint32_t X, uint32_t Y)
{
    uint32_t Result = 1;
    while (Y --)
    {
        Result *= X;
    }
    return Result;
}

void Serial1_SendNumber(uint32_t Number, uint8_t Length)
{
    uint8_t i;
    for (i = 0; i < Length; i ++)
    {
        Serial1_SendByte(Number / Serial1_Pow(10, Length - i - 1) % 10 + '0');
    }
}

/**
  * 函    数：取一个已收到的字节
  * 参    数：out 用来带回数据
  * 返 回 值：1 = 取到了，0 = 没有新字节
  * 说    明：数据源从"一个变量 + 一个标志位"换成了环形缓冲。
  *           原来必须在临界区里"读数据 + 清标志"打包，否则同一个字节会被
  *           执行两次（'P' 画两次车位）；现在 Get 只挪一个读指针，
  *           中断只挪一个写指针，两者互不覆盖，临界区自然就不需要了。
  *           接口名不变，system.c 一行都不用改就能跑。
  */
uint8_t Serial1_TakeRxData(uint8_t *out)
{
    return UartRing_Get(&rx1, out);
}

/* 积压字节数，调试时看主循环有没有被卡住 */
uint16_t Serial1_RxCount(void)
{
    return UartRing_Count(&rx1);
}

/* 因缓冲满而丢弃的字节数。正常运行时应该恒为 0 */
uint16_t Serial1_RxOverflow(void)
{
    return UartRing_Overflow(&rx1);
}

/* 看第 offset 个待处理字节，不消费 —— 帧协议解析用，见 proto.c */
uint8_t Serial1_Peek(uint16_t offset, uint8_t *out)
{
    return UartRing_Peek(&rx1, offset, out);
}

/* 丢弃 n 个字节。只有主循环改 tail，所以和 Get 一样不需要临界区 */
void Serial1_Drop(uint16_t n)
{
    UartRing_Drop(&rx1, n);
}

void USART1_IRQHandler(void)  // 改为USART1中断处理函数
{
    if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET)  // USART1
    {
        // 无条件读一次 DR：同时清 RXNE 和 ORE（溢出）。
        // 不清 ORE 的话，一旦发生过一次溢出，RXNE 会被 ORE 顶住反复进中断，
        // 主循环被饿死。所以这里必须"先读、再判断用不用"。
        uint8_t b = (uint8_t)USART_ReceiveData(USART1);
        UartRing_Put(&rx1, b);
    }

    /* 发送：环里有就继续写 DR，取空了就把 TXE 中断关掉（否则 DR 一空
     * 就一直进中断）。写 DR 会自动清 TXE，不需要手动清标志。 */
    if (USART_GetITStatus(USART1, USART_IT_TXE) == SET)
    {
        uint8_t out;
        if (UartRing_Get(&tx1, &out)) {
            USART_SendData(USART1, out);
        } else {
            USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
        }
    }
}


void Serial2_Init(void)
{
    /* 先清缓冲，再开中断 */
    UartRing_Init(&rx2);
    UartRing_Init(&tx2);

    /*开启时钟*/
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    /*GPIO初始化*/
    GPIO_InitTypeDef GPIO_InitStructure;

    // PA2作为USART2_TX（复用推挽输出）
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;                 // PA2
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // PA3作为USART2_RX（浮空输入）
    // 上拉输入，不是浮空。对端（蓝牙模块 / 树莓派）没上电或者没插的时候，
    // 悬空的 RX 脚被电机和驱动器的干扰一耦合就能合成一个起始位，HandleCmd
    // 拿到一个任意字节照执行 —— 'U' 是前进，'P' 是开始画车位。
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;             // 上拉输入
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;                 // PA3
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /*USART初始化*/
    USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_Init(USART2, &USART_InitStructure);                 // USART2

    /*中断配置*/
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);            //USART2

    /*NVIC配置*/
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;         // USART2中断通道
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_Init(&NVIC_InitStructure);

    /*USART使能*/
    USART_Cmd(USART2, ENABLE);                                // USART2
}

/**
  * 函    数：串口发送一个字节
  * 参    数：Byte 要发送的一个字节
  * 返 回 值：无
  */
void Serial2_SendByte(uint8_t Byte)
{
    Uart_TxPut(USART2, &tx2, Byte);
}

/**
  * 函    数：次方函数（内部使用）
  * 返 回 值：返回值等于X的Y次方
  */
uint32_t Serial2_Pow(uint32_t X, uint32_t Y)
{
	uint32_t Result = 1;	//设置结果初值为1
	while (Y --)			//执行Y次
	{
		Result *= X;		//将X累乘到结果
	}
	return Result;
}

/**
  * 函    数：串口发送数字
  * 参    数：Number 要发送的数字，范围：0~4294967295
  * 参    数：Length 要发送数字的长度，范围：0~10
  * 返 回 值：无
  */
void Serial2_SendNumber(uint32_t Number, uint8_t Length)
{
	uint8_t i;
	for (i = 0; i < Length; i ++)		//根据数字长度遍历数字的每一位
	{
		Serial2_SendByte(Number / Serial2_Pow(10, Length - i - 1) % 10 + '0');	//依次调用Serial_SendByte发送每位数字
	}
}

/**
  * 函    数：取一个已收到的字节
  * 参    数：out 用来带回数据
  * 返 回 值：1 = 取到了，0 = 没有新字节
  * 说    明：与 Serial1_TakeRxData 同构，见上面的说明。
  */
uint8_t Serial2_TakeRxData(uint8_t *out)
{
    return UartRing_Get(&rx2, out);
}

uint16_t Serial2_RxCount(void)
{
    return UartRing_Count(&rx2);
}

uint16_t Serial2_RxOverflow(void)
{
    return UartRing_Overflow(&rx2);
}

uint8_t Serial2_Peek(uint16_t offset, uint8_t *out)
{
    return UartRing_Peek(&rx2, offset, out);
}

void Serial2_Drop(uint16_t n)
{
    UartRing_Drop(&rx2, n);
}

/**
  * 函    数：USART2中断函数
  * 参    数：无
  * 返 回 值：无
  * 注意事项：此函数为中断函数，无需调用，中断触发后自动执行
  *           函数名为预留的指定名称，可以从启动文件复制
  *           请确保函数名正确，不能有任何差异，否则中断函数将不能进入
  */
void USART2_IRQHandler(void)
{
	if (USART_GetITStatus(USART2, USART_IT_RXNE) == SET)		//判断是否是USART2的接收事件触发的中断
	{
		// 无条件读一次 DR：同时清 RXNE 和 ORE，理由同 USART1。
		uint8_t b = (uint8_t)USART_ReceiveData(USART2);
		UartRing_Put(&rx2, b);
	}

    /* 发送：同 USART1，取空了就关 TXE 中断 */
    if (USART_GetITStatus(USART2, USART_IT_TXE) == SET)
    {
        uint8_t out;
        if (UartRing_Get(&tx2, &out)) {
            USART_SendData(USART2, out);
        } else {
            USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
        }
    }
}
