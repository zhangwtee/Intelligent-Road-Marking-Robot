#include "cfg.h"
#include "proto.h"          /* 只为了速度、画线尺寸的合法范围 */
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
 * 单纯校验 magic 不够：magic 是最先写进去的，它成立的时候值可能还没落地。
 */
#define CFG_MAGIC       0x53504431u     /* "SPD1" */

/* 断点记录从第 3 个字开始，自带一个 magic。
 *
 * 为什么不复用页首那个 magic：页首那个的含义是"速度有效"。老固件写过的
 * 页里没有断点这一段，读回来是全 0xFF —— 这里 magic 对不上就判无断点，
 * 速度照常读，升级不用擦页。
 *
 *   word2  RESUME_MAGIC
 *   word3  step | heading<<8 | left_mm<<16
 *   word4  right_mm | len_mm<<16
 *   word5  wid_mm | checksum<<16
 *
 * 校验用一句异或折叠，不是逐字取反：这里要塞五个字段，逐字取反要六个字。
 * 它防的是"擦到一半掉电""写了几个字就没往下写"，那种情况下对不上的概率
 * 已经足够高。
 */
#define RESUME_MAGIC    0x50525331u     /* "PRS1" */

static uint16_t  cfg_saved;         /* flash 里当前有效的速度值，0 = 没有 */
static CfgResume cfg_resume;        /* flash 里当前有效的断点 */
static uint8_t   cfg_loaded;        /* 上面两个是否已经从 flash 读出来过 */

static uint32_t Cfg_ReadWord(uint32_t addr)
{
    return *(volatile uint32_t *)addr;
}

static uint16_t CfgChecksum(uint32_t w3, uint32_t w4, uint16_t wid)
{
    uint32_t sum = w3 ^ w4 ^ (uint32_t)wid;

    return (uint16_t)(sum ^ (sum >> 16));
}

static uint8_t CfgResumeLooksSane(const CfgResume *rec)
{
    /* PaintStep 是 IDLE、EDGE1、TURN1、EDGE2 …… 交替排下来的，边永远
     * 落在奇数位。断点只在边上记（转弯上记下来的接不上，见 system.c 的
     * PaintCaptureResume），所以偶数一律作废 —— 认了一个 TURN 当断点，
     * 续画会顺着转弯那一步直接往前开，框就歪了，而且歪得很像标定没做好。 */
    if (rec->step < 1 || rec->step > 7 || (rec->step & 1u) == 0u) {
        return 0;
    }
    if (rec->left_mm == 0 || rec->right_mm == 0) {
        return 0;               /* 剩 0 毫米就不是断点，是已经走完了 */
    }
    if (rec->len_mm < PROTO_PAINT_MIN_MM || rec->len_mm > PROTO_PAINT_MAX_MM
        || rec->wid_mm < PROTO_PAINT_MIN_MM || rec->wid_mm > PROTO_PAINT_MAX_MM) {
        return 0;               /* 长宽要拿去走后面三条边，不能是个野值 */
    }
    if (rec->left_mm > PROTO_PAINT_MAX_MM || rec->right_mm > PROTO_PAINT_MAX_MM) {
        return 0;
    }
    return 1;
}

/* 把整页读一遍，填 cfg_saved / cfg_resume。上电和写完校验都用它。 */
static void CfgReadPage(void)
{
    uint32_t w0, w1, w2, w3, w4, w5;
    uint16_t value, inv;

    cfg_saved        = 0;
    cfg_resume.valid = 0;

    w0 = Cfg_ReadWord(CFG_PAGE_ADDR);
    w1 = Cfg_ReadWord(CFG_PAGE_ADDR + 4);
    w2 = Cfg_ReadWord(CFG_PAGE_ADDR + 8);
    w3 = Cfg_ReadWord(CFG_PAGE_ADDR + 12);
    w4 = Cfg_ReadWord(CFG_PAGE_ADDR + 16);
    w5 = Cfg_ReadWord(CFG_PAGE_ADDR + 20);

    if (w0 == CFG_MAGIC) {
        value = (uint16_t)(w1 & 0xFFFFu);
        inv   = (uint16_t)(w1 >> 16);
        if ((uint16_t)(~value) == inv
            && value >= PROTO_SPEED_MIN_ARR && value <= PROTO_SPEED_MAX_ARR) {
            cfg_saved = value;
        }
    }

    if (w2 == RESUME_MAGIC) {
        uint16_t wid = (uint16_t)(w5 & 0xFFFFu);
        uint16_t sum = (uint16_t)(w5 >> 16);

        if (sum == CfgChecksum(w3, w4, wid)) {
            CfgResume rec;

            rec.step     = (uint8_t)(w3 & 0xFFu);
            rec.heading  = (uint8_t)((w3 >> 8) & 0xFFu);
            rec.left_mm  = (uint16_t)(w3 >> 16);
            rec.right_mm = (uint16_t)(w4 & 0xFFFFu);
            rec.len_mm   = (uint16_t)(w4 >> 16);
            rec.wid_mm   = wid;

            if (CfgResumeLooksSane(&rec)) {
                rec.valid    = 1;
                cfg_resume   = rec;
            }
        }
    }

    cfg_loaded = 1;
}

/* 整页重写。speed = 0 表示"没有有效速度"（写 0xFFFF 占位，读回时范围
 * 检查会判无效）。rec 为 0 或 rec->valid = 0 表示不写断点那一段。
 *
 * 返回 1 只代表"flash 操作没报错"，写完有没有落地由调用方读回比对 ——
 * 和原来的 Cfg_SaveSpeed 一个套路。 */
static uint8_t CfgWritePage(uint16_t speed, const CfgResume *rec)
{
    uint16_t sp;
    uint32_t payload;

    if (!cfg_loaded) {
        /* 没读过就写，会把一个还活着的断点擦掉。 */
        CfgReadPage();
    }

    /* 调用方给的可能是"还没读出来"的 0（刚上电就存断点这条路）。
     * 这时用页里刚读出来的那份 —— 否则存一次断点，速度就变回默认值了。 */
    if (speed < PROTO_SPEED_MIN_ARR || speed > PROTO_SPEED_MAX_ARR) {
        speed = cfg_saved;
    }

    sp = (speed >= PROTO_SPEED_MIN_ARR && speed <= PROTO_SPEED_MAX_ARR)
         ? speed : 0xFFFFu;
    payload = (uint32_t)sp | ((uint32_t)(uint16_t)(~sp) << 16);

    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);

    if (FLASH_ErasePage(CFG_PAGE_ADDR) == FLASH_COMPLETE
        && FLASH_ProgramWord(CFG_PAGE_ADDR, CFG_MAGIC) == FLASH_COMPLETE
        && FLASH_ProgramWord(CFG_PAGE_ADDR + 4, payload) == FLASH_COMPLETE) {

        if (rec != 0 && rec->valid) {
            uint16_t wid = rec->wid_mm;
            uint32_t w3  = (uint32_t)rec->step
                         | ((uint32_t)rec->heading << 8)
                         | ((uint32_t)rec->left_mm << 16);
            uint32_t w4  = (uint32_t)rec->right_mm
                         | ((uint32_t)rec->len_mm << 16);
            uint32_t w5  = (uint32_t)wid
                         | ((uint32_t)CfgChecksum(w3, w4, wid) << 16);

            if (FLASH_ProgramWord(CFG_PAGE_ADDR + 8,  RESUME_MAGIC) != FLASH_COMPLETE
                || FLASH_ProgramWord(CFG_PAGE_ADDR + 12, w3) != FLASH_COMPLETE
                || FLASH_ProgramWord(CFG_PAGE_ADDR + 16, w4) != FLASH_COMPLETE
                || FLASH_ProgramWord(CFG_PAGE_ADDR + 20, w5) != FLASH_COMPLETE) {
                FLASH_Lock();
                CfgReadPage();
                return 0;
            }
        }

        FLASH_Lock();
        return 1;
    }

    FLASH_Lock();
    CfgReadPage();              /* 页已经擦了，之前的值不再存在 */
    return 0;
}

uint16_t Cfg_GetSavedSpeed(void)
{
    return cfg_saved;
}

uint8_t Cfg_LoadSpeed(uint16_t *arr)
{
    CfgReadPage();

    if (cfg_saved == 0) {
        return 0;
    }
    if (arr != 0) {
        *arr = cfg_saved;
    }
    return 1;
}

uint8_t Cfg_SaveSpeed(uint16_t arr)
{
    uint16_t check;

    if (arr < PROTO_SPEED_MIN_ARR || arr > PROTO_SPEED_MAX_ARR) {
        return 0;
    }
    if (arr == cfg_saved) {
        return 1;               /* 值没变，不为一页擦除付那份代价 */
    }

    if (!CfgWritePage(arr, &cfg_resume)) {
        return 0;
    }

    /* 写没写进去只有读回来才知道 */
    if (Cfg_LoadSpeed(&check) && check == arr) {
        return 1;
    }
    return 0;
}

uint8_t Cfg_LoadResume(CfgResume *rec)
{
    CfgReadPage();

    if (!cfg_resume.valid) {
        return 0;
    }
    if (rec != 0) {
        *rec = cfg_resume;
    }
    return 1;
}

uint8_t Cfg_SaveResume(const CfgResume *rec)
{
    CfgResume want, check;

    if (rec == 0 || !rec->valid) {
        return Cfg_ClearResume();
    }

    want = *rec;
    want.valid = 1;
    if (!CfgResumeLooksSane(&want)) {
        return 0;               /* 这条记录要拿去驱动电机，野值宁可不写 */
    }

    if (!CfgWritePage(cfg_saved, &want)) {
        return 0;
    }

    if (!Cfg_LoadResume(&check)) {
        return 0;
    }
    if (check.step != want.step || check.heading != want.heading
        || check.left_mm != want.left_mm || check.right_mm != want.right_mm
        || check.len_mm != want.len_mm || check.wid_mm != want.wid_mm) {
        return 0;
    }
    return 1;
}

uint8_t Cfg_ClearResume(void)
{
    CfgResume none;

    CfgReadPage();
    if (!cfg_resume.valid) {
        return 1;               /* 本来就没有，不擦这一页 */
    }

    none.valid = 0;
    if (!CfgWritePage(cfg_saved, &none)) {
        return 0;
    }

    /* CfgWritePage 成功时不回读，这里得自己把 RAM 里的那份清掉 ——
     * 它才是"flash 里现在有什么"的副本，放着不管，下一次判断会以为
     * 断点还在，于是把这次成功的擦除报成失败。 */
    cfg_resume.valid = 0;
    return 1;
}
