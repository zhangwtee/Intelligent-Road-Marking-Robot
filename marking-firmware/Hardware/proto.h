#ifndef __PROTO_H
#define __PROTO_H

#include "stm32f10x.h"

/* ==========================================================================
 * 帧协议（上下位机共用）
 *
 * 改造前是「一个字节一条命令」，两个问题都是安全问题：
 *   1) 线上任何噪声都是合法指令。9600 下一个毛刺落在 'P' 上，车就开始
 *      自动画 5.3 米并喷漆。
 *   2) 带不了参数。补喷长度只能拆成「先发 '3' 设档、再发 'C' 开喷」，
 *      两条之间被打断就变成「档位变了、车没动」，而且完全静默。
 *
 * 帧结构（定长 10 字节）：
 *      0    0x7B  '{'   帧头
 *      1    CMD         命令/类型
 *      2    SEQ         序号，ACK 原样回显，也是重发去重的依据
 *      3-7  DATA[5]     参数，多字节一律大端
 *      8    CRC8        覆盖第 0~7 字节（poly 0x07，初值 0，不反射）
 *      9    0x7D  '}'   帧尾
 *
 * 为什么定长不是变长：变长要先验 LEN 再等长度，LEN 被破坏时解析器会安静地
 * 等一个永远不来的长度。本项目参数最多 2 字节，变长换不到任何好处。
 *
 * 为什么 0x7B / 0x7D 可以共存：所有单字节命令都是可打印 ASCII 的字母数字，
 * '{' 和 '}' 不在命令表里，天然不冲突 —— 这是「老的单字节路径」和「帧」
 * 能共用一条串口的前提。
 *
 * 上位机那一份在 marking-host/core/frame.py，两边的常量由
 * protocol_test.py::test_frame_constants_match_firmware 对账。
 *
 * 航向闭环的帧（CMD_GYRO / CMD_HEADING）是后来加的，但仍然是定长 10 字节
 * 的那一套 —— 加命令不加结构，是这套协议能一路扩下来的原因。
 * ========================================================================== */

#define PROTO_SOF           0x7B
#define PROTO_EOF           0x7D
#define PROTO_FRAME_LEN     10
#define PROTO_DATA_LEN      5

/* ---- 下行命令（上位机 -> STM32） ---- */
#define PROTO_CMD_RESPRAY   0x01    /* DATA[0:2] = 补喷长度 mm 大端 */
#define PROTO_CMD_MOVE      0x02    /* DATA[0]     = 方向
                                     * DATA[1:3]   = 左轮 mm 大端
                                     * DATA[3:5]   = 右轮 mm 大端
                                     *   右轮填 0 = 左右同距（老格式，
                                     *   串口助手手敲帧时只填左轮就行）。
                                     * 左右不等 = 差速，视觉纠偏用它。 */
#define PROTO_CMD_LEVEL     0x03    /* DATA[0] = 档位 1~5，兼容 '1'~'5' */
#define PROTO_CMD_TRIM      0x04    /* DATA[0:1] = int16 毫米，>0 向右修正
                                     * 只在 STATE_MOVE 里有效。这是视觉闭环
                                     * 唯一的执行口：它不改"走多远"，只改
                                     * "左右两边各自还差多少"。 */
#define PROTO_CMD_PUMP      0x05    /* DATA[0] = 0 关 / 1 开，只在 WAIT/MOVE */
#define PROTO_CMD_PAINT     0x06    /* DATA[0:1] 长 mm，DATA[2:3] 宽 mm，0=默认
                                     * DATA[4]   = 标志，bit0 = 四条边用航向闭环 */
#define PROTO_CMD_GYRO      0x07    /* DATA[0] = 子命令（见下），陀螺仪航向闭环 */
#define PROTO_CMD_SPEED     0x08    /* DATA[0:1] = 速度参数 ARR 大端，越小越快。
                                     * 见 PROTO_SPEED_MIN/MAX_ARR。
                                     *
                                     * 为什么给 ARR 而不是"第几档"：ARR 就是固件
                                     * 真正写进定时器的那两个数之一，中间再加一层
                                     * 档位表，就多一处"两边档位表不一样"的机会。
                                     * 补喷长度当年就是从档位改成毫米的。
                                     *
                                     * 只允许在空闲 / 急停 / 遥控点动时改。段内改
                                     * 会把段首按脉冲比例定好的左右 ARR 拉平，
                                     * 剩下的路只有一侧在推，车斜着走完这一段。 */
#define PROTO_CMD_PING      0x09    /* 无参数，只回 ACK */

/* ---- CMD_GYRO 的子命令（DATA[0]） ----
 *
 * 为什么陀螺仪闭环要能被关掉：传感器是会坏的。零偏飘了、接线松了、
 * 换了个山寨模块，闭环给出的就不再是"修正"而是"持续的偏"，而且它
 * 比视觉纠偏优先级高（见 PROTO_ERR_BUSY）。没有这个开关，现场唯一的
 * 办法是拔线，而那会让整车一起失去陀螺仪的其他用途。 */
#define PROTO_GYRO_OFF      0x00    /* 关闭环，退回纯视觉/开环 */
#define PROTO_GYRO_ON       0x01    /* 开闭环（未标定时返回 ERR_STATE） */
#define PROTO_GYRO_RECAL    0x02    /* 重新标零，车必须停着 */
#define PROTO_GYRO_MEASURE  0x03    /* 轮距自标定：单侧走 300mm 再反算轮距 */

/* ---- 上行（STM32 -> 上位机） ---- */
#define PROTO_CMD_ACK       0x81    /* DATA[0]=原命令 DATA[1]=结果码 */
#define PROTO_CMD_STATUS    0x82    /* DATA[0]=状态 DATA[1]=段号
                                     * DATA[2]=标志 DATA[3:5]=脉冲数 */

/* 航向帧，和状态帧一样 5Hz 发。
 *
 * 单独一帧而不是塞进 STATUS：STATUS 那 5 个字节已经排满了，
 * 而航向是 int16，硬塞会把脉冲进度挤掉。多一种帧类型比压缩字段安全 ——
 * 上位机按 CMD 分发，不认识的直接忽略，老版本上位机也不会被破坏。
 *
 * 上报的是「相对段起点」的航向，不是上电累计值：
 * 上位机画曲线看的是"这一段偏了多少"，累计值每段都要自己减，反而容易错。 */
#define PROTO_CMD_HEADING   0x83    /* DATA[0:1] = int16 航向，0.01 度，右正
                                     * DATA[2:3] = int16 角速度，0.1 dps
                                     * DATA[4]   = 高位状态，低 4 位错误计数 */

/* ---- ACK 结果码 ---- */
#define PROTO_OK            0x00
#define PROTO_ERR_STATE     0x01    /* 当前状态不接受（比如正在画车位时补喷） */
#define PROTO_ERR_RANGE     0x02    /* 参数越界 */
#define PROTO_ERR_TYPE      0x03    /* 不认识的命令类型 */
#define PROTO_ERR_HW        0x04    /* 硬件不支持（普通轮没有横移） */
#define PROTO_ERR_BUSY      0x05    /* 执行器被下位机自己的闭环占着。
                                     * 目前只有一种情况：陀螺仪航向闭环开着时
                                     * 收到 CMD_TRIM。
                                     *
                                     * 为什么不是"两个一起用"：差速只有一个。
                                     * 上位机按视觉横向偏移算差速，下位机按航向
                                     * 算差速，两个都往同一个 Motion_Trim 上叠，
                                     * 效果是两个积分器互相打架 —— 车会周期性
                                     * 地左一下右一下，而且双方都觉得自己没错，
                                     * 日志里一条错误都没有。分开管才是对的：
                                     * 有陀螺仪时航向归下位机，视觉只管开关泵。 */

/* ---- MOVE 方向 ---- */
#define PROTO_MOVE_FWD      0x00
#define PROTO_MOVE_BACK     0x01
#define PROTO_MOVE_LEFT     0x02
#define PROTO_MOVE_RIGHT    0x03

/* ---- 参数范围 ---- */
#define PROTO_RESPRAY_MIN_MM    50
#define PROTO_RESPRAY_MAX_MM    3000
#define PROTO_MOVE_MAX_MM       5000
#define PROTO_TRIM_MAX_MM       20      /* 必须和 motion.c 的 MOTION_TRIM_MAX_MM
                                         * 一致，protocol_test 会对账这两个数 */
#define PROTO_PAINT_MIN_MM      1000
#define PROTO_PAINT_MAX_MM      8000

/* ---- CMD_PAINT 的 DATA[4] 标志 ---- */
#define PROTO_PAINT_FLAG_HEADING 0x01   /* 四条边走陀螺仪航向闭环。
                                         * 陀螺仪没标定时整条命令回 ERR_HW ——
                                         * "照画但不纠"是歪的且没人报错。 */

/* ---- 速度：脉冲定时器的 ARR（计数时钟 1MHz，ARR 即微秒数）
 *
 * 这两个值和 StepMotor_New.c 的 SPEED_FAST_ARR / SPEED_SLOW_ARR 必须一致，
 * 越界的帧在这里就回绝，而不是让底层默默夹紧 —— 夹紧的话上位机界面上
 * 显示的数和车上真正跑的数就不一样了。 */
#define PROTO_SPEED_MIN_ARR     150     /* 最快：6.7kHz，约 1.3m/s */
#define PROTO_SPEED_MAX_ARR     1500    /* 最慢：667Hz，约 0.13m/s */

/* CRC-8/ATM：poly 0x07，初值 0，不反射，不异或输出。
 * 标准自检值 Proto_Crc8("123456789", 9) == 0xF4，上位机用同一个值锚定。 */
uint8_t Proto_Crc8(const uint8_t *buf, uint8_t len);

/* 整帧校验：帧头、帧尾、CRC 三者都对才返回 1。
 * 要求 frame 至少有 PROTO_FRAME_LEN 个字节可读。 */
uint8_t Proto_Check(const uint8_t *frame);

/* 组一帧到 out（PROTO_FRAME_LEN 字节）。data 可为 0，表示参数全 0。 */
void Proto_Build(uint8_t *out, uint8_t cmd, uint8_t seq, const uint8_t *data);

#endif
