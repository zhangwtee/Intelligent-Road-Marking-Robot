#include "proto.h"

/* ==========================================================================
 * 帧协议的纯函数部分
 *
 * 这里只做「算 CRC / 验帧 / 组帧」三件事，不碰串口、不碰状态机。
 * 这样它既可以被 system.c 调用，也可以在 PC 上用 gcc 单独编译做对账测试
 * —— 协议这种两边各写一遍的东西，能单独跑才有意义。
 * ========================================================================== */

/**
  * 函    数：CRC-8（poly 0x07，初值 0，不反射，不异或输出）
  * 参    数：buf 数据首地址；len 字节数
  * 返 回 值：CRC 值
  * 说    明：不用查表。查表省的是几十微秒，代价是一张 256 字节的表要跟
  *           上位机那份手工对齐 —— 位运算版本两边逐位相同，出错的空间更小。
  *           帧长只有 10 字节，速度完全不是问题。
  *           标准校验值：Proto_Crc8("123456789", 9) == 0xF4。
  */
uint8_t Proto_Crc8(const uint8_t *buf, uint8_t len)
{
    uint8_t crc = 0;
    uint8_t i;
    uint8_t bit;

    for (i = 0; i < len; i++) {
        crc ^= buf[i];
        for (bit = 0; bit < 8; bit++) {
            if (crc & 0x80) {
                crc = (uint8_t)((crc << 1) ^ 0x07);
            } else {
                crc = (uint8_t)(crc << 1);
            }
        }
    }

    return crc;
}

/**
  * 函    数：整帧校验
  * 参    数：frame 指向帧头
  * 返 回 值：1 = 合法；0 = 不是帧
  * 说    明：三个条件都要过：帧头对、帧尾对、CRC 对。
  *           只看帧头是绝对不行的 —— 0x7B 在噪声里出现的概率是 1/256，
  *           一秒钟的噪声就能凑出好几个。
  */
uint8_t Proto_Check(const uint8_t *frame)
{
    if (frame[0] != PROTO_SOF) {
        return 0;
    }
    if (frame[PROTO_FRAME_LEN - 1] != PROTO_EOF) {
        return 0;
    }
    if (Proto_Crc8(frame, PROTO_FRAME_LEN - 2) != frame[PROTO_FRAME_LEN - 2]) {
        return 0;
    }
    return 1;
}

/**
  * 函    数：组一帧
  * 参    数：out 输出缓冲（至少 PROTO_FRAME_LEN 字节）
  *           cmd 命令号；seq 序号；data 参数（可为 0）
  * 返 回 值：无
  * 说    明：data 不足 5 字节的部分补 0，不读越界。
  */
void Proto_Build(uint8_t *out, uint8_t cmd, uint8_t seq, const uint8_t *data)
{
    uint8_t i;

    out[0] = PROTO_SOF;
    out[1] = cmd;
    out[2] = seq;

    for (i = 0; i < PROTO_DATA_LEN; i++) {
        out[3 + i] = (data != 0) ? data[i] : 0;
    }

    out[PROTO_FRAME_LEN - 2] = Proto_Crc8(out, PROTO_FRAME_LEN - 2);
    out[PROTO_FRAME_LEN - 1] = PROTO_EOF;
}
