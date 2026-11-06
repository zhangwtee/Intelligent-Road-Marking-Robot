#include "cfg.h"
#include "proto.h"          /* 只为了速度的合法范围 */
#include "stm32f10x_flash.h"

/* F103 大容量型号的页大小固定 2KB。地址取 512K 的最后 2KB，
 * 和 Project.uvprojx 里 IROM(0x08000000, 0x80000) 对齐。 */
#define CFG_PAGE_SIZE   2048u
#define CFG_IMAGE_END   (0x08000000u + 512u * 1024u)
#define CFG_PAGE_ADDR   (CFG_IMAGE_END - CFG_PAGE_SIZE)

/* 页首两个字：magic + (值 | 取反 << 16)。
 *
 * 取反那一半防的是"擦完页、只写进了低半字就掉电"：这种情况下高半字读回来
 * 是 0xFFFF，和取反对不上，直接判无效 —— 不会拿一个半截值去跑电机。
 * 单纯校验 magic 不够：magic 是最先写进去的，它成立的时候值可能还没落地。 */
#define CFG_MAGIC       0x53504431u     /* "SPD1" */

static uint16_t cfg_saved;      /* flash 里当前有效的值，0 = 没有 */

static uint32_t Cfg_ReadWord(uint32_t addr)
{
    return *(volatile uint32_t *)addr;
}

uint16_t Cfg_GetSavedSpeed(void)
{
    return cfg_saved;
}

uint8_t Cfg_LoadSpeed(uint16_t *arr)
{
    uint32_t w0, w1;
    uint16_t value, inv;

    cfg_saved = 0;

    w0 = Cfg_ReadWord(CFG_PAGE_ADDR);
    w1 = Cfg_ReadWord(CFG_PAGE_ADDR + 4);

    if (w0 != CFG_MAGIC) {
        return 0;
    }

    value = (uint16_t)(w1 & 0xFFFFu);
    inv   = (uint16_t)(w1 >> 16);

    if ((uint16_t)(~value) != inv) {
        return 0;
    }
    if (value < PROTO_SPEED_MIN_ARR || value > PROTO_SPEED_MAX_ARR) {
        return 0;
    }

    cfg_saved = value;
    if (arr != 0) {
        *arr = value;
    }
    return 1;
}

uint8_t Cfg_SaveSpeed(uint16_t arr)
{
    uint16_t check;
    uint32_t payload;

    if (arr < PROTO_SPEED_MIN_ARR || arr > PROTO_SPEED_MAX_ARR) {
        return 0;
    }
    if (arr == cfg_saved) {
        return 1;               /* 值没变，不为一页擦除付那份代价 */
    }

    payload = (uint32_t)arr | ((uint32_t)(uint16_t)(~arr) << 16);

    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);

    if (FLASH_ErasePage(CFG_PAGE_ADDR) == FLASH_COMPLETE
        && FLASH_ProgramWord(CFG_PAGE_ADDR, CFG_MAGIC) == FLASH_COMPLETE
        && FLASH_ProgramWord(CFG_PAGE_ADDR + 4, payload) == FLASH_COMPLETE) {
        FLASH_Lock();
        /* 写没写进去只有读回来才知道 */
        if (Cfg_LoadSpeed(&check) && check == arr) {
            return 1;
        }
        return 0;
    }

    FLASH_Lock();
    cfg_saved = 0;              /* 页已经擦了，之前的值不再存在 */
    return 0;
}
