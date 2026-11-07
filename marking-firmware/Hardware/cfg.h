#ifndef __CFG_H
#define __CFG_H

#include "stm32f10x.h"

/* ==========================================================================
 * 参数掉电保存：速度档位 + 画车位断点
 *
 * 为什么值得占一页 flash：速度决定漆量和拐角冲击，是现场最需要当场调的
 * 一个量。以前改完只活在 RAM 里，一断电回到 300 —— 而这台机器的断电是
 * 常态（干完活就关），于是每次开机都要重新拖一遍滑条，或者干脆不改。
 *
 * 断点也存这里：急停那一刻记下"这条边还剩多少毫米"，断电重启之后还能
 * 接着走。注意它记的是剩余距离，不是位置 —— 车要是被人推走了，续出来的
 * 框就是歪的。这一点和不断电时按续画按钮完全一样，靠人看着车判断。
 *
 * 位置：512K 的最后一页（2KB）。代码只占二十几 KB，离这里很远；地址写死在
 * cfg.c，IROM 的范围由 protocol_test 对一次账，将来 IROM 改了会被测出来。
 *
 * 写一次要擦一页（F103 的最小擦除单位），擦除期间取指被挂起几十毫秒 ——
 * 所以调用方必须在车停着的时候写，见 system.c 的 SpeedSaveService 和
 * PaintResumeService。
 * ========================================================================== */

/* 画车位断点。字段统一用毫米整数：1mm 的分辨率对"还剩多远"足够，
 * 定长也好对账。 */
typedef struct {
    uint8_t  valid;         /* 1 = 这条记录有效 */
    uint8_t  step;          /* PaintStep 的当前值（EDGE1 ~ EDGE4） */
    uint8_t  heading;       /* 断点那条边当时有没有在纠偏 */
    uint16_t left_mm;       /* 左侧还剩多少毫米 */
    uint16_t right_mm;      /* 右侧还剩多少毫米 */
    uint16_t len_mm;        /* 这次画车位的长宽：续完这条边之后还要用 */
    uint16_t wid_mm;
} CfgResume;

/* 初始化时读回有效值。返回 1 表示 arr 已经填好，0 表示没有存过/存坏了。 */
uint8_t Cfg_LoadSpeed(uint16_t *arr);

/* 写进去并读回校验。返回 1 = 真的写成功了。
 * flash 写失败不会报错，只能写完再读一遍 —— 所以这个函数一定要返回值。 */
uint8_t Cfg_SaveSpeed(uint16_t arr);

/* flash 里现在存的是多少。没有有效值返回 0。
 * ARR 的合法范围是 150~1500，0 撞不上，调用方可以直接拿它当"没有存过"。 */
uint16_t Cfg_GetSavedSpeed(void);

/* 读回断点。返回 1 = 有有效断点；0 = 没有，或者存的值根本不像个断点。 */
uint8_t Cfg_LoadResume(CfgResume *rec);

/* 写断点（rec->valid = 1）或抹掉它（rec->valid = 0）。
 * 返回 1 = 真的写成功了。失败之后 flash 里可能什么都不剩，
 * 调用方要能接受"这次断点丢了"。 */
uint8_t Cfg_SaveResume(const CfgResume *rec);

/* 抹掉断点。没有断点时直接返回 1，不为一页擦除付代价。 */
uint8_t Cfg_ClearResume(void);

#endif
