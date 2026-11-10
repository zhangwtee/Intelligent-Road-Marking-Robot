#ifndef __USART_H
#define __USART_H

#include <stdio.h>

/* Peek / Drop 是给帧协议解析用的：解析器要先看队首是不是帧头，再决定
 * 丢 1 个字节还是丢一整帧。Get 只能"取出来"，取出来再塞回去会打乱顺序。 */

/* 两路串口各有用途，别只按引脚记名字：
 *   USART1 (PA9/PA10) = 蓝牙手机，SOURCE_BT
 *   USART2 (PA2/PA3)  = 树莓派上位机，SOURCE_PI
 * 这个对应关系一旦搞反，'+'/'-'（蓝牙专用调速）的门就开在了错误的一侧。 */

/* USART1 —— 蓝牙 (PA9 / PA10) */
uint8_t  Serial1_TakeRxData(uint8_t *out);   // 从环形缓冲取一个字节
void     Serial1_Init(void);
/* 发送：字节进发送环，由 TXE 中断挪出去 —— 这里不会等 10ms */
void     Serial1_SendByte(uint8_t Byte);
void     Serial1_SendNumber(uint32_t Number, uint8_t Length);
uint16_t Serial1_RxCount(void);              // 积压字节数（调试用）
uint16_t Serial1_RxOverflow(void);           // 因满丢弃的字节数（正常恒为 0）
uint8_t  Serial1_Peek(uint16_t offset, uint8_t *out);  // 看第 offset 个字节，不消费
void     Serial1_Drop(uint16_t n);                     // 丢弃 n 个字节

/* USART2 —— 树莓派 (PA2 / PA3) */
uint8_t  Serial2_TakeRxData(uint8_t *out);
void     Serial2_Init(void);
void     Serial2_SendByte(uint8_t Byte);   // 同上，非阻塞
void     Serial2_SendNumber(uint32_t Number, uint8_t Length);
uint16_t Serial2_RxCount(void);
uint16_t Serial2_RxOverflow(void);
uint8_t  Serial2_Peek(uint16_t offset, uint8_t *out);
void     Serial2_Drop(uint16_t n);

#endif
