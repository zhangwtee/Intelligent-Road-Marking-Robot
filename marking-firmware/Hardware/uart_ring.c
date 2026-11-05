#include "uart_ring.h"

/* 掩码取模。要求 UART_RING_SIZE 是 2 的幂。 */
#define RING_MASK   ((uint16_t)(UART_RING_SIZE - 1))

void UartRing_Init(UartRing *r)
{
    r->head = 0;
    r->tail = 0;
    r->overflow = 0;
}

/**
  * 函    数：中断里存一个字节
  * 参    数：r 环形缓冲；byte 收到的字节
  * 返 回 值：1 = 已存入；0 = 缓冲满，已丢弃
  * 说    明：只有中断会改 head，所以这里不需要任何保护。
  *           缓冲满时丢【新】字节而不是覆盖最旧的 —— 旧命令先到先服务，
  *           而且"丢最旧"会让刚到的急停 T 被后面的噪声挤掉，方向反了。
  */
uint8_t UartRing_Put(UartRing *r, uint8_t byte)
{
    uint16_t next = (uint16_t)((r->head + 1) & RING_MASK);

    if (next == r->tail) {          /* 满：留一格空位用来区分空和满 */
        r->overflow++;              /* 计数留着，调试时能看出主循环被卡过 */
        return 0;
    }

    r->buf[r->head] = byte;
    r->head = next;                 /* 先写数据再挪指针，主循环不会读到半成品 */
    return 1;
}

/**
  * 函    数：主循环里取一个字节
  * 参    数：r 环形缓冲；out 带回数据
  * 返 回 值：1 = 取到；0 = 空
  * 说    明：只有主循环会改 tail。
  */
uint8_t UartRing_Get(UartRing *r, uint8_t *out)
{
    if (r->head == r->tail) {
        return 0;
    }

    *out = r->buf[r->tail];
    r->tail = (uint16_t)((r->tail + 1) & RING_MASK);
    return 1;
}

/**
  * 函    数：查看第 offset 个待处理字节，不消费
  * 参    数：r 环形缓冲；offset 相对队首的偏移；out 带回数据
  * 返 回 值：1 = 有这个字节；0 = 不够
  * 说    明：head 可能在循环里被中断改掉，但那只让"可用字节变多"，
  *           不会让已经读到的字节失效 —— 所以不需要临界区。
  */
uint8_t UartRing_Peek(UartRing *r, uint16_t offset, uint8_t *out)
{
    uint16_t head  = r->head;
    uint16_t avail = (uint16_t)((head - r->tail) & RING_MASK);
    uint16_t idx;

    if (offset >= avail) {
        return 0;
    }

    idx = (uint16_t)((r->tail + offset) & RING_MASK);
    *out = r->buf[idx];
    return 1;
}

/**
  * 函    数：丢弃 n 个待处理字节
  * 参    数：r 环形缓冲；n 丢弃数量
  * 返 回 值：无
  * 说    明：n 超过实际积压量时按实际量丢，不会把 tail 甩到 head 前面
  *           —— 那会让 Count 算出一个巨大值，解析器去读根本不存在的字节。
  */
void UartRing_Drop(UartRing *r, uint16_t n)
{
    uint16_t head  = r->head;
    uint16_t avail = (uint16_t)((head - r->tail) & RING_MASK);

    if (n > avail) {
        n = avail;
    }

    r->tail = (uint16_t)((r->tail + n) & RING_MASK);
}

uint16_t UartRing_Count(UartRing *r)
{
    /* head 可能在读它的这一瞬间被中断改掉。读一次存下来再用，
     * 得到的是"读的那一刻"的准确值，够用了 —— 这个函数只用于调试显示。 */
    uint16_t head = r->head;
    uint16_t tail = r->tail;

    return (uint16_t)((head - tail) & RING_MASK);
}

uint16_t UartRing_Overflow(UartRing *r)
{
    return r->overflow;
}
