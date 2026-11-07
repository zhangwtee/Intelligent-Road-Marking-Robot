#include "cfg.h"
#include "proto.h"          /* 只为速度、画线尺寸的合法范围 */
#include "stm32f10x_flash.h"

/* F103 大容量的页大小固定 2KB。基址取 512K 镜像的末尾，和 Project.uvprojx
 * 里 IROM(0x08000000, 0x80000) 对齐。 */
#define CFG_PAGE_SIZE   2048u
#define CFG_IMAGE_END   (0x08000000u + 512u * 1024u)

/* 四页：速度和断点各两页，写满一页才擦另一页。 */
#define CFG_SPEED_PAGE0   (CFG_IMAGE_END - 4u * CFG_PAGE_SIZE)
#define CFG_SPEED_PAGE1   (CFG_IMAGE_END - 3u * CFG_PAGE_SIZE)
#define CFG_RESUME_PAGE0  (CFG_IMAGE_END - 2u * CFG_PAGE_SIZE)
#define CFG_RESUME_PAGE1  (CFG_IMAGE_END - 1u * CFG_PAGE_SIZE)

/* 记录格式。两种记录都从页首开始一格一格往后排，写到页尾才换页：
 *
 *   速度（12 字节 / 条）              断点（20 字节 / 条）
 *   +0   magic 'SPD2'                 +0   magic 'PRS2'
 *   +4   seq | ~seq<<16               +4   seq | ~seq<<16
 *   +8   value | ~value<<16           +8   step | heading<<8 | left_mm<<16
 *                                     +12  right_mm | len_mm<<16
 *                                     +16  wid_mm | checksum<<16
 *
 * 为什么不"擦一页写一条"：F103 擦一页期间取指是停的，而画车位要在每条边
 * 的开头记一次断点，一圈光断点就是好几次擦除 —— 一万次的擦写寿命，几圈
 * 就把一页磨掉了。改成页内追加之后，写一条只编程 3~5 个字，一页能装一百
 * 多条，很久才轮到一次擦除。
 *
 * 序号和值都带一份取反，挡的是"只写了低半字就掉电"那种半截记录：全 0xFF
 * 的空白格在取反这一关就过不去，读的时候直接跳过。 */
#define CFG_MAGIC_SPEED   0x53504432u   /* "SPD2" */
#define CFG_MAGIC_RESUME  0x50525332u   /* "PRS2" */
#define CFG_WORD_BLANK    0xFFFFFFFFu

#define CFG_SPEED_WORDS   3u
#define CFG_SPEED_SLOT    (4u * CFG_SPEED_WORDS)    /* 12 字节 */
#define CFG_RESUME_WORDS  5u
#define CFG_RESUME_SLOT   (4u * CFG_RESUME_WORDS)   /* 20 字节 */

static uint16_t  cfg_saved;         /* 当前有效的速度值，0 = 没有 */
static CfgResume cfg_resume;        /* 当前有效的断点记录（valid 见结构体注释） */
static uint8_t   cfg_loaded;

/* 两页里见过的最大序号。新记录一律 +1，读的时候挑最大的那条。 */
static uint16_t  cfg_speed_seq;
static uint16_t  cfg_resume_seq;

/* 上一次写到哪一页。写满就换另一页。 */
static uint32_t  cfg_speed_page;
static uint32_t  cfg_resume_page;

/* 两页里有没有任何一条记录（含墓碑）。没有的话不必为"抹断点"写一次。 */
static uint8_t   cfg_speed_have;
static uint8_t   cfg_resume_have;

static uint32_t Cfg_ReadWord(uint32_t addr)
{
    return *(volatile uint32_t *)addr;
}

/* 折成 16 位的异或校验。覆盖断点记录里除了 wid 以外所有会决定"续到哪、
 * 续多长"的字段；wid 自己单独进函数。 */
static uint16_t CfgChecksum(uint32_t wa, uint32_t wb, uint16_t wid)
{
    uint32_t sum = wa ^ wb ^ (uint32_t)wid;

    return (uint16_t)(sum ^ (sum >> 16));
}

static uint8_t CfgResumeLooksSane(const CfgResume *rec)
{
    /* PaintStep 是 IDLE、EDGE1、TURN1、EDGE2 …… 交替排下来的，只有边在
     * 奇数位。断点只在边上记；转弯上记下来的接不上 —— 那不是"少喷一段"，
     * 是"少转了 90 度"。所以这里再挡一道，页里可能有半截的记录。 */
    if (rec->step < 1 || rec->step > 7 || (rec->step & 1u) == 0u) {
        return 0;
    }
    if (rec->left_mm == 0 || rec->right_mm == 0) {
        return 0;               /* 剩 0 毫米就不是断点，是已经走完了 */
    }
    if (rec->len_mm < PROTO_PAINT_MIN_MM || rec->len_mm > PROTO_PAINT_MAX_MM
        || rec->wid_mm < PROTO_PAINT_MIN_MM || rec->wid_mm > PROTO_PAINT_MAX_MM) {
        return 0;               /* 续画要用它走后面几条边，不能是个野值 */
    }
    if (rec->left_mm > PROTO_PAINT_MAX_MM || rec->right_mm > PROTO_PAINT_MAX_MM) {
        return 0;
    }
    return 1;
}

/* 擦一页。只在写满换页时调用。 */
static uint8_t CfgErasePage(uint32_t addr)
{
    FLASH_Status st;

    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
    st = FLASH_ErasePage(addr);
    FLASH_Lock();

    return (uint8_t)(st == FLASH_COMPLETE);
}

/* 往一个空格里写 n 个字，写完读回核对。不擦页 —— 调用方保证这块是空的。 */
static uint8_t CfgProgramWords(uint32_t addr, const uint32_t *words, uint8_t n)
{
    uint8_t i;

    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);

    for (i = 0; i < n; i++) {
        if (FLASH_ProgramWord(addr + 4u * (uint32_t)i, words[i]) != FLASH_COMPLETE) {
            FLASH_Lock();
            return 0;
        }
    }
    FLASH_Lock();

    for (i = 0; i < n; i++) {
        if (Cfg_ReadWord(addr + 4u * (uint32_t)i) != words[i]) {
            return 0;
        }
    }
    return 1;
}

/* 追加一条记录。当前页写满就擦另一页、从它开头接着写。
 *
 * 擦除发生的时候，另一页里至少躺着这一条之前的所有记录 —— 也就是说
 * 读的时候永远有东西可读。这是"写坏一次只丢这一次"的来源。 */
static uint8_t CfgAppend(uint32_t *page, uint8_t slot, uint8_t words_n,
                         const uint32_t *words)
{
    uint32_t addr      = *page;
    uint32_t page_end  = *page + CFG_PAGE_SIZE;
    uint32_t span      = 4u * (uint32_t)words_n;

    while (addr + span <= page_end && Cfg_ReadWord(addr) != CFG_WORD_BLANK) {
        addr += slot;
    }

    if (addr + span > page_end) {
        /* 这页满了。换另一页：擦掉它，从开头写。 */
        uint32_t other = (*page == CFG_SPEED_PAGE0 || *page == CFG_RESUME_PAGE0)
                       ? *page + CFG_PAGE_SIZE : *page - CFG_PAGE_SIZE;

        if (!CfgErasePage(other)) {
            return 0;
        }
        *page = other;
        addr  = other;
    }

    return CfgProgramWords(addr, words, words_n);
}

/* 扫一页速度记录，挑序号最大且校验通过的那条。 */
static uint8_t CfgScanSpeed(uint32_t page, uint16_t *seq, uint16_t *val)
{
    uint32_t addr;
    uint16_t best_seq = 0;
    uint16_t best_val = 0;
    uint8_t  got      = 0;

    for (addr = page; addr + CFG_SPEED_SLOT <= page + CFG_PAGE_SIZE;
         addr += CFG_SPEED_SLOT) {
        uint32_t w1, w2;
        uint16_t s, ns, v, nv;

        if (Cfg_ReadWord(addr) != CFG_MAGIC_SPEED) {
            continue;
        }
        w1 = Cfg_ReadWord(addr + 4u);
        w2 = Cfg_ReadWord(addr + 8u);

        s  = (uint16_t)(w1 & 0xFFFFu);
        ns = (uint16_t)(w1 >> 16);
        v  = (uint16_t)(w2 & 0xFFFFu);
        nv = (uint16_t)(w2 >> 16);

        if ((uint16_t)(~s) != ns || (uint16_t)(~v) != nv) {
            continue;
        }
        if (v < PROTO_SPEED_MIN_ARR || v > PROTO_SPEED_MAX_ARR) {
            continue;
        }
        if (!got || (int16_t)(s - best_seq) > 0) {
            best_seq = s;
            best_val = v;
            got      = 1;
        }
    }

    if (!got) {
        return 0;
    }
    *seq = best_seq;
    *val = best_val;
    return 1;
}

/* 扫一页断点记录。step == 0 是墓碑：这一页明确记着"没有断点"。
 *
 * 光有"没找到记录"是不够的 —— 另一页里还躺着上一份断点，两边一比序号，
 * 墓碑必须参与竞争，否则那份就复活了。 */
static uint8_t CfgScanResume(uint32_t page, uint16_t *seq, CfgResume *out)
{
    uint32_t addr;
    uint16_t best_seq = 0;
    CfgResume best;
    uint8_t  got      = 0;

    best.valid = 0;

    for (addr = page; addr + CFG_RESUME_SLOT <= page + CFG_PAGE_SIZE;
         addr += CFG_RESUME_SLOT) {
        uint32_t w1, w2, w3, w4;
        uint16_t s, ns, sum, wid;
        CfgResume rec;

        if (Cfg_ReadWord(addr) != CFG_MAGIC_RESUME) {
            continue;
        }
        w1 = Cfg_ReadWord(addr + 4u);
        w2 = Cfg_ReadWord(addr + 8u);
        w3 = Cfg_ReadWord(addr + 12u);
        w4 = Cfg_ReadWord(addr + 16u);

        s  = (uint16_t)(w1 & 0xFFFFu);
        ns = (uint16_t)(w1 >> 16);
        if ((uint16_t)(~s) != ns) {
            continue;
        }

        wid = (uint16_t)(w4 & 0xFFFFu);
        sum = (uint16_t)(w4 >> 16);
        if (sum != CfgChecksum(w2, w3, wid)) {
            continue;
        }

        rec.valid    = 0;
        rec.step     = (uint8_t)(w2 & 0xFFu);
        rec.heading  = (uint8_t)((w2 >> 8) & 0xFFu);
        rec.left_mm  = (uint16_t)(w2 >> 16);
        rec.right_mm = (uint16_t)(w3 & 0xFFFFu);
        rec.len_mm   = (uint16_t)(w3 >> 16);
        rec.wid_mm   = wid;

        if (rec.step != 0u) {
            if (!CfgResumeLooksSane(&rec)) {
                continue;       /* 半截的、被别的东西写过的，跳过 */
            }
            rec.valid = 1;
        }

        if (!got || (int16_t)(s - best_seq) > 0) {
            best_seq = s;
            best     = rec;
            got      = 1;
        }
    }

    if (!got) {
        return 0;
    }
    *seq = best_seq;
    *out = best;
    return 1;
}

/* 两页都扫一遍，取各自最新的一条，再比序号。 */
static void CfgReadAll(void)
{
    uint16_t seq;
    uint16_t val;
    CfgResume rec;

    cfg_saved        = 0;
    cfg_speed_seq    = 0;
    cfg_speed_have   = 0;
    cfg_speed_page   = CFG_SPEED_PAGE0;

    cfg_resume_have  = 0;
    cfg_resume_seq   = 0;
    cfg_resume_page  = CFG_RESUME_PAGE0;
    cfg_resume.valid = 0;

    if (CfgScanSpeed(CFG_SPEED_PAGE0, &seq, &val)) {
        cfg_saved      = val;
        cfg_speed_seq  = seq;
        cfg_speed_page = CFG_SPEED_PAGE0;
        cfg_speed_have = 1;
    }
    if (CfgScanSpeed(CFG_SPEED_PAGE1, &seq, &val)) {
        if (!cfg_speed_have || (int16_t)(seq - cfg_speed_seq) > 0) {
            cfg_saved      = val;
            cfg_speed_seq  = seq;
            cfg_speed_page = CFG_SPEED_PAGE1;
            cfg_speed_have = 1;
        }
    }

    if (CfgScanResume(CFG_RESUME_PAGE0, &seq, &rec)) {
        cfg_resume      = rec;
        cfg_resume_seq  = seq;
        cfg_resume_page = CFG_RESUME_PAGE0;
        cfg_resume_have = 1;
    }
    if (CfgScanResume(CFG_RESUME_PAGE1, &seq, &rec)) {
        /* 序号比较用有符号差，16 位回绕之后仍然是对的。 */
        if (!cfg_resume_have || (int16_t)(seq - cfg_resume_seq) > 0) {
            cfg_resume      = rec;
            cfg_resume_seq  = seq;
            cfg_resume_page = CFG_RESUME_PAGE1;
            cfg_resume_have = 1;
        }
    }

    cfg_loaded = 1;
}

uint16_t Cfg_GetSavedSpeed(void)
{
    if (!cfg_loaded) {
        CfgReadAll();
    }
    return cfg_saved;
}

uint8_t Cfg_LoadSpeed(uint16_t *arr)
{
    CfgReadAll();

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
    uint32_t w[CFG_SPEED_WORDS];
    uint16_t seq;
    uint16_t sp;

    if (!cfg_loaded) {
        CfgReadAll();
    }
    if (arr < PROTO_SPEED_MIN_ARR || arr > PROTO_SPEED_MAX_ARR) {
        return 0;
    }
    if (cfg_speed_have && arr == cfg_saved) {
        return 1;               /* 值没变，不必再占一格 */
    }

    seq = (uint16_t)(cfg_speed_seq + 1u);
    sp  = arr;

    w[0] = CFG_MAGIC_SPEED;
    w[1] = (uint32_t)seq | ((uint32_t)(uint16_t)(~seq) << 16);
    w[2] = (uint32_t)sp  | ((uint32_t)(uint16_t)(~sp)  << 16);

    if (!CfgAppend(&cfg_speed_page, CFG_SPEED_SLOT, CFG_SPEED_WORDS, w)) {
        CfgReadAll();           /* 按 flash 里实际剩下的重来 */
        return 0;
    }

    cfg_saved      = sp;
    cfg_speed_seq  = seq;
    cfg_speed_have = 1;
    return 1;
}

/* 追加一份断点记录，rec == 0 时写墓碑。 */
static uint8_t CfgWriteResume(const CfgResume *rec)
{
    uint32_t w[CFG_RESUME_WORDS];
    uint32_t wa = 0;
    uint32_t wb = 0;
    uint16_t seq;
    uint16_t wid = 0;

    if (!cfg_loaded) {
        CfgReadAll();
    }

    seq = (uint16_t)(cfg_resume_seq + 1u);

    if (rec != 0) {
        wid = rec->wid_mm;
        wa  = (uint32_t)rec->step
            | ((uint32_t)rec->heading << 8)
            | ((uint32_t)rec->left_mm << 16);
        wb  = (uint32_t)rec->right_mm | ((uint32_t)rec->len_mm << 16);
    }

    w[0] = CFG_MAGIC_RESUME;
    w[1] = (uint32_t)seq | ((uint32_t)(uint16_t)(~seq) << 16);
    w[2] = wa;
    w[3] = wb;
    w[4] = (uint32_t)wid | ((uint32_t)CfgChecksum(wa, wb, wid) << 16);

    if (!CfgAppend(&cfg_resume_page, CFG_RESUME_SLOT, CFG_RESUME_WORDS, w)) {
        CfgReadAll();
        return 0;
    }

    cfg_resume_seq  = seq;
    cfg_resume_have = 1;

    /* RAM 里那份"flash 里现在是什么"的副本跟着改。抄漏这一段，下一次
     * 判断会以为断点还在，把一次成功的擦除报成失败 —— 上层于是每 10 秒
     * 重试一次，而每次只是追加一条墓碑，现象是"擦不掉"，实际早就没了。 */
    if (rec != 0) {
        cfg_resume       = *rec;
        cfg_resume.valid = 1;
    } else {
        cfg_resume.valid = 0;
    }
    return 1;
}

uint8_t Cfg_LoadResume(CfgResume *rec)
{
    CfgReadAll();

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
    CfgResume want;

    if (rec == 0 || !rec->valid) {
        return Cfg_ClearResume();
    }

    want = *rec;
    want.valid = 1;
    if (!CfgResumeLooksSane(&want)) {
        return 0;               /* 这条记录拿去续画也接不上，干脆不写 */
    }
    return CfgWriteResume(&want);
}

uint8_t Cfg_ClearResume(void)
{
    if (!cfg_loaded) {
        CfgReadAll();
    }
    if (!cfg_resume_have) {
        return 1;               /* 两页都是空的，不用写 */
    }
    if (!cfg_resume.valid) {
        return 1;               /* 最新那份本来就是"没有断点" */
    }
    return CfgWriteResume(0);
}
