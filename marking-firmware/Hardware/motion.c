#include "motion.h"
#include "calib.h"
#include "StepMotor_New.h"

#define MOTION_PI      3.14159265f
#define MOTION_PI_180  (3.14159265f / 180.0f)

/* 当前底盘是否具备横移能力，来源是 chassis.h（由 Target 的 Define 决定）。
 * 这里只留一个别名，方便老代码继续引用 MOTION_HAS_MECANUM。 */
#define MOTION_HAS_MECANUM   CHASSIS_HAS_MECANUM

/* 纠偏限幅只有一个：MOTION_TRIM_ACCUM_MAX_MM，即"这一段累计能纠多少"，
 * 定义在 motion.h（按车型分 80 / 200mm）。
 *
 * 以前这里还有一个 MOTION_TRIM_MAX_MM = 20mm，是"每次纠一点"那个年代
 * 的每步限幅。入参改成累计量之后它变成"这一段总共只能纠 20mm"，
 * 折合两度多，把闭环整个卡死，所以删掉了。 */

/* 纠偏方向的符号。+1 = delta 为正时车头向右。
 * 相机左右装反、或者某一侧电机接线反了，只改这个宏，别去翻协议层。 */
#ifndef MOTION_TRIM_SIGN
#define MOTION_TRIM_SIGN     1
#endif

/* 段内重新同步的最小剩余行程（脉冲）。约 40mm。
 *
 * 低于这个数就别重算 ARR 了：剩余脉冲太小的时候，"按比例分速度"算出来
 * 的比值完全被量化噪声主导（剩 12 个对 7 个就是 1.7 倍，而真实比例未必），
 * 而那点行程改不改都听不出来。 */
#ifndef MOTION_SYNC_MIN_REMAIN
#define MOTION_SYNC_MIN_REMAIN   200
#endif

/* ---------- 预计算系数 ----------
 * F103 没有 FPU，float 乘除都是软件模拟。运行时的除法一次要几十微秒，
 * 放在状态机里会造成脉冲启停延迟。所以这里全部在 Init 时算完，
 * 之后只剩乘法。 */
static float   s_pulse_per_mm  = 0.0f;
static float   s_pulse_per_deg = 0.0f;
static float   s_k[4]          = {1.0f, 1.0f, 1.0f, 1.0f};
static uint8_t s_ready         = 0;

/* 本段已经落地的累计纠偏（脉冲，带符号）。每次开新段清零。
 * Motion_Trim 拿它和"上层期望的累计量"作差，只把差值加到剩余目标上 ——
 * 上层反复发同一个期望值时不会重复累加。
 * 只在 Motion_Trim 和开段函数里读写，都是主循环上下文，不需要临界区。 */
static int32_t s_trim_accum    = 0;

void Motion_Init(void)
{
    /* 轮径补偿系数由车型决定。
     *
     * 麦轮：四轮各自独立的 K1~K4。四个轮子都能横滑，走快了走慢了不会
     *       互相顶，逐轮标定是有意义的。
     * 差速：左组只用 K1、右组只用 K2（见 chassis.h）。普通轮不能横滑，
     *       组内只要有一点行程差，多出来的那部分只能靠某个轮子打滑消化，
     *       而哪个先打滑不确定 —— 车身就会莫名其妙扭一下。宁可整组
     *       统一差 2%，也不要组内差 0.5%。 */
    s_k[0] = CHASSIS_K_LEFT_FRONT;
    s_k[1] = CHASSIS_K_RIGHT_FRONT;
    s_k[2] = CHASSIS_K_LEFT_REAR;
    s_k[3] = CHASSIS_K_RIGHT_REAR;

    /* 每毫米需要多少脉冲 = 每圈脉冲 / 轮子周长 */
    s_pulse_per_mm = (float)CALIB_STEPS_PER_REV / (MOTION_PI * CALIB_WHEEL_DIA_MM);

    /* 原地转 1 度时，每个轮子走过的弧长 = 半径 x 弧度，
     * 再乘每毫米脉冲数。 */
    s_pulse_per_deg = MOTION_PI_180 * CALIB_TURN_RADIUS_MM * s_pulse_per_mm;

    s_ready = 1;
}

uint8_t Motion_IsReady(void)
{
    return s_ready;
}

/* ---------- 纯换算 ---------- */

uint32_t Motion_MmToPulse(float mm)
{
    if (mm <= 0.0f) {
        return 0;
    }
    return (uint32_t)(mm * s_pulse_per_mm + 0.5f);
}

uint32_t Motion_DegToPulse(float deg)
{
    if (deg <= 0.0f) {
        return 0;
    }
    return (uint32_t)(deg * s_pulse_per_deg + 0.5f);
}

float Motion_PulseToMm(uint32_t pulse)
{
    if (s_pulse_per_mm <= 0.0f) {
        return 0.0f;
    }
    return (float)pulse / s_pulse_per_mm;
}

float Motion_PulseToDeg(uint32_t pulse)
{
    if (s_pulse_per_deg <= 0.0f) {
        return 0.0f;
    }
    return (float)pulse / s_pulse_per_deg;
}

/* base x num / den，四舍五入，饱和到 0xFFFF。
 *
 * 为什么中间用 64 位：左右比例极端时（一边 40 万脉冲、另一边几十个），
 * base x num 会超出 32 位。这里的结果其实只用来当 ARR，最终会被
 * StepMotor 层夹到 SPEED_SLOW_ARR（1500），所以宁可粗糙也不能回绕 ——
 * 回绕出来的 ARR 会让那一侧瞬间全速，而这一拍正在走。
 *
 * den 为 0 的情况在调用点已经排除，这里再挡一次，省得以后改调用点踩坑。 */
#if CHASSIS_SYNC_ARR
static uint32_t RatioArr(uint32_t base, uint32_t num, uint32_t den)
{
    uint64_t v;

    if (den == 0) {
        return base;
    }

    v = ((uint64_t)base * num + den / 2) / den;
    if (v > 0xFFFFu) {
        v = 0xFFFFu;
    }
    return (uint32_t)v;
}
#endif /* CHASSIS_SYNC_ARR —— 关掉同步时连这个函数都不编，否则会有
        * "static function declared but never used" 的警告 */

/* ---------- 左右同步：按脉冲比例算两组 ARR ----------
 *
 * 两组同频时，脉冲多的一侧要跑更久。那一侧跑完之前，另一侧已经停了 ——
 * 剩下的路程只有单侧在推，车头方向在这一段里持续偏，段与段接起来就是
 * 一条弯的线（拐角处线跑偏、原地转有"拖一下"的感觉）。
 *
 * 以前四路脉冲共用 TIM3 的一个 ARR，物理上做不到"两边不同频"；
 * 现在每路一个时基，只要让两边同时到达就行：
 *
 *     t = 脉冲数 x ARR / 时钟        （ARR 参数即一个周期的计数个数，
 *                                     写进寄存器的是 ARR-1，见 Motor_ApplyArr）
 *     令 t_left == t_right：
 *     p_left x ARR_left == p_right x ARR_right
 *
 * 所以 ARR 与脉冲数成反比 —— 脉冲少的一侧 ARR 按同样比例放大，
 * 两边就走完在同一时刻，而且**段的时长完全不变**（仍是脉冲多的一侧
 * 在用户档位下走完的时间）。
 *
 * 两个边界：
 *   1. 比例算出来的 ARR 会被底层夹到 [SPEED_FAST_ARR, SPEED_SLOW_ARR]。
 *      超出时同步不完整（仍比不同步好），但不会失控。
 *   2. 用户档位本身如果接近最慢档（1500），稍有一点比例差就会顶到夹位。
 *      现场调速度时别把档位一直按到最慢，转弯时没有余量可让。
 */
static void DriveSyncArr(uint32_t pl, uint32_t pr)
{
#if CHASSIS_SYNC_ARR
    uint32_t base;
    uint32_t hi;
    uint32_t diff;
    uint32_t permille;
    uint32_t al;
    uint32_t ar;

    if (pl == 0 || pr == 0) {
        StepMotor_RestoreSpeed();
        return;
    }

    /* 基准必须是用户档位，不能用 GetSpeedTarget：上一段可能是原地转，
     * 那个值已经被压到 CHASSIS_TURN_ARR 了，拿它当基准会越转越慢。 */
    base = StepMotor_GetSpeedBase();

    hi       = (pl > pr) ? pl : pr;
    diff     = (pl > pr) ? (pl - pr) : (pr - pl);
    permille = (uint32_t)(((uint64_t)diff * 1000u) / hi);

    /* 差得很少就当一样。ARR 是整数，为 0.x% 去改写寄存器没有意义；
     * 而 DriveLR 被视觉伺服每 200mm 调一次，段段都写会有一拍不稳。 */
    if (permille <= CHASSIS_SYNC_DEADBAND_PM) {
        StepMotor_RestoreSpeed();      /* 内部会判断"本来就在 base"直接返回 */
        return;
    }

    /* ARR 与脉冲数反比：脉冲多的一侧保持用户档位，少的一侧按同样倍数放大。
     * base x pl / pr 一步算到位（等价于 base + base x diff / pr，但没有
     * 中间那次加法，也就少一次溢出机会）。 */
    if (pl > pr) {
        al = base;
        ar = RatioArr(base, pl, pr);
    } else {
        ar = base;
        al = RatioArr(base, pr, pl);
    }

    StepMotor_SetSpeedLR((uint16_t)al, (uint16_t)ar);
#else
    /* 关掉同步：两组同频，脉冲少的一侧先停。这是改造前的行为，
     * 留在这里只为了现场对比，不要当成"更简单所以更好"。 */
    (void)pl;
    (void)pr;
    StepMotor_RestoreSpeed();
#endif
}

/* ---------- 四轮驱动（内部） ----------
 * pulse 用 uint32_t 直接传给 SetPulse，绕开 GoForward 的 uint16_t 上限，
 * 这样超过 12.9 米的行程也不会被截断。
 * 至少要给 1 个脉冲：SetPulse(0) 在中断里会被当成"已停止"，
 * 但驱动器的使能状态依赖首脉冲，0 会让它保持上一次的方向语义。 */
static void DriveAll(uint8_t forward, uint32_t p1, uint32_t p2,
                     uint32_t p3, uint32_t p4)
{
    if (p1 == 0) { p1 = 1; }
    if (p2 == 0) { p2 = 1; }
    if (p3 == 0) { p3 = 1; }
    if (p4 == 0) { p4 = 1; }

    /* 开新段 = 重新对齐。上一段攒下的左右差在这一刻归零，累计纠偏量也
     * 跟着归零 —— 这正是"每段重设而不是全程一次"能把误差锁在段内的原因。 */
    s_trim_accum = 0;

    StepMotor_SetDirection(MOTOR1, forward ? M1_FWD : M1_REV);
    StepMotor_SetDirection(MOTOR2, forward ? M2_FWD : M2_REV);
    StepMotor_SetDirection(MOTOR3, forward ? M3_FWD : M3_REV);
    StepMotor_SetDirection(MOTOR4, forward ? M4_FWD : M4_REV);

    /* 两组 ARR：多数情况下左右脉冲相同（直线），这里等同原来的
     * StepMotor_RestoreSpeed —— 它既负责"转弯之后回到用户档位"，也负责
     * 在左右真不一样时把两侧拉成同时到达。
     *
     * 两个作用合到一处是因为它们的前提相同：都要求"以用户档位为基准"。
     * 分开写的话，恢复档位和同步比例会各写一次寄存器，中间那一下
     * 就是一次无意义的频率扰动。
     *
     * 组代表取 M1 / M2：差速车的 k 系数本来就把左右两组的组内差抹平了
     * （见 chassis.h），麦轮四路虽然各不相同，但相差不到 1%，而且麦轮
     * 能横滑吸收，不值得为它把 SetSpeedLR 扩成四路接口。 */
    DriveSyncArr(p1, p2);

    StepMotor_SetPulse(MOTOR1, p1);
    StepMotor_SetPulse(MOTOR2, p2);
    StepMotor_SetPulse(MOTOR3, p3);
    StepMotor_SetPulse(MOTOR4, p4);
}

/* ---------- 运动指令 ---------- */

void Motion_Forward(float mm)
{
    uint32_t base;

    if (!s_ready || mm <= 0.0f) {
        return;
    }

    base = Motion_MmToPulse(mm);

    DriveAll(1,
             (uint32_t)((float)base * s_k[0] + 0.5f),
             (uint32_t)((float)base * s_k[1] + 0.5f),
             (uint32_t)((float)base * s_k[2] + 0.5f),
             (uint32_t)((float)base * s_k[3] + 0.5f));
}

void Motion_Backward(float mm)
{
    uint32_t base;

    if (!s_ready || mm <= 0.0f) {
        return;
    }

    base = Motion_MmToPulse(mm);

    /* 后退就是四个轮子的方向全部取反，其余和前进完全一样 */
    DriveAll(0,
             (uint32_t)((float)base * s_k[0] + 0.5f),
             (uint32_t)((float)base * s_k[1] + 0.5f),
             (uint32_t)((float)base * s_k[2] + 0.5f),
             (uint32_t)((float)base * s_k[3] + 0.5f));
}

uint8_t Motion_HasMecanum(void)
{
    return MOTION_HAS_MECANUM;
}

void Motion_ForwardFlat(float mm)
{
    if (!s_ready || mm <= 0.0f) {
        return;
    }

    /* 关斜坡 -> 启动 -> 立刻恢复，不把状态留给下一条指令。
     * 与旧 StepMotor_GoForwardFlat 的做法一致：ramp_enable 只需要在
     * SetPulse 调用的瞬间为 0。 */
    StepMotor_SetRampEnabled(0);
    Motion_Forward(mm);
    StepMotor_SetRampEnabled(1);
}

void Motion_ForwardFlatLR(float left_mm, float right_mm)
{
    uint32_t pl;
    uint32_t pr;

    if (!s_ready) {
        return;
    }
    if (left_mm <= 0.0f && right_mm <= 0.0f) {
        return;
    }

    pl = Motion_MmToPulse(left_mm);
    pr = Motion_MmToPulse(right_mm);

    /* 直接按脉冲喂 DriveAll，不走 DriveLR。
     *
     * DriveLR 会按轮径系数 k 再乘一次。而这两个数是拿 StepMotor 的剩余
     * 脉冲反算回来的，本身已经含过 k 了 —— 再乘一次，这一截就会走成 k 倍。
     *
     * DriveAll 就是 Motion_Forward 走的那条路：复位累计纠偏、按左右脉冲
     * 定两组 ARR、四路一起启动。除了左右可以不同，和 Motion_ForwardFlat
     * 完全一致。单侧给 0 时 DriveAll 会抬到 1 个脉冲，不会停掉那一路。 */
    StepMotor_SetRampEnabled(0);
    DriveAll(1, pl, pr, pl, pr);
    StepMotor_SetRampEnabled(1);
}

void Motion_TurnRight(float deg)
{
    uint32_t p;

    if (!s_ready || deg <= 0.0f) {
        return;
    }

    /* 转角补偿：普通轮原地转是"拧"过去的，拖胎比麦轮严重，实测转不够。
     * 系数放在 chassis.h（麦轮 1.000，差速 1.050），必须现场量了再改 ——
     * 这个数错了的直接表现是"车位不是直角，差几度"。 */
    deg *= CHASSIS_TURN_SCALE;

    p = Motion_DegToPulse(deg);
    if (p == 0) {
        p = 1;
    }

    /* 原地右转：左侧后退、右侧前进，四个轮子同脉冲。
     *
     * 这里必须从 M*_FWD / M*_REV 派生，不能写死一个 DIR 电平。四个轮子装在
     * 车两侧、镜像接线，"物理向前"对应的电平本来就左右相反（见
     * StepMotor_New.h 的方向表：M1_FWD 和 M2_FWD 就不是同一个值）。写死
     * 一个电平等于假设四个驱动器极性一致 —— 那样左后、右后两个轮子会反着
     * 转，车不是转弯，是四个轮子两两对拧。 */
    StepMotor_SetDirection(MOTOR1, M1_REV);   /* 左前：后退 */
    StepMotor_SetDirection(MOTOR2, M2_FWD);   /* 右前：前进 */
    StepMotor_SetDirection(MOTOR3, M3_REV);   /* 左后：后退 */
    StepMotor_SetDirection(MOTOR4, M4_FWD);   /* 右后：前进 */

    /* 差速车减速转（CHASSIS_TURN_ARR > 0），麦轮不限速（= 0）。
     *
     * 这是"转的时候不顺畅"的机械侧对策：普通橡胶轮横向不能滑移，四轮
     * 做原地转时是在地面上拧，速度越快越容易"攒一下、弹一下"。降速不能
     * 消除拖胎，只能让每次跳动变小、变密，看起来是连续的。
     *
     * 限速是"本段"的：下一次前进时由 Motion_Forward / DriveLR 恢复，
     * 调完这里不用再存原档位 —— 转弯之后必定还有一个走直线的段，那一段
     * 开头的恢复动作就是天然的收尾。 */
    if (CHASSIS_TURN_ARR > 0) {
        StepMotor_SetSpeedTarget(CHASSIS_TURN_ARR);
    } else {
        /* 麦轮不限速。但上一段可能是 DriveLR 设成左右不同 ARR 的（差速纠偏
         * 改完目标会按剩余脉冲重算左右 ARR），不恢复的话转弯这四个轮子同
         * 脉冲不同频：先到的一侧停住，另一侧继续拧。 */
        StepMotor_RestoreSpeed();
    }

    /* 转弯同样不做软起软停。
     * 理由和 DriveLR 一样：斜坡每 100 个脉冲改一档 ARR，四路一起变，而
     * 四路的目标脉冲是相等的（绕车体中心转，四轮弧长相同）—— 起坡不影响
     * 左右关系，但会让"转 90 度"的实际角速度前后不一致（起步慢、中段快），
     * 画车位时角速度不均会让喷头在拐角处堆积。匀速拧过去更可控。 */
    StepMotor_SetRampEnabled(0);

    StepMotor_SetPulse(MOTOR1, p);
    StepMotor_SetPulse(MOTOR2, p);
    StepMotor_SetPulse(MOTOR3, p);
    StepMotor_SetPulse(MOTOR4, p);

    StepMotor_SetRampEnabled(1);
}

/* ---------- 视觉纠偏 ---------- */

uint8_t Motion_Trim(float delta_mm)
{
    int32_t want;       /* 这一段累计该纠到多少（半差，脉冲） */
    int32_t step;       /* 这一次真正要落地的差 */
    uint8_t moved = 0;
    uint8_t capped = 0;   /* 期望值撞上段内累计上限，已经被削平 */

    if (!s_ready) {
        return 0;
    }

    /* 单次期望值不超过"这一段累计能纠多少"本身。
     *
     * 这里以前钳的是 MOTION_TRIM_MAX_MM（20mm）—— 那是按增量用时的每步
     * 限幅。改成累计量语义之后，钳的是"这一段总共能纠多少"：20mm 差速在
     * 500mm 轮距上只折合两度多，偏差再大也压不下去，而且永远撞不到上面
     * 那个 80mm / 200mm 的累计上限，返回 2 成了死代码，上层再也不知道
     * 这一段该重新对齐了。
     *
     * 真正的"别一下改太多"由两件事保证：调用方的速率限制（视觉那条有
     * MAX_STEP_MM，陀螺仪那条按几何量给），以及这里的累计上限。 */
    if (delta_mm > MOTION_TRIM_ACCUM_MAX_MM) {
        delta_mm = MOTION_TRIM_ACCUM_MAX_MM;
    }
    if (delta_mm < -MOTION_TRIM_ACCUM_MAX_MM) {
        delta_mm = -MOTION_TRIM_ACCUM_MAX_MM;
    }

    /* 差速的一半摊到每一侧。先取绝对值再定符号，免得负数截断和 +0.5f
     * 的舍入互相打架（-3.7 -> -3 还是 -4，两种都有道理，但差一个脉冲
     * 在左右两侧就是两次不同的方向修正）。 */
    if (delta_mm >= 0.0f) {
        want = (int32_t)(delta_mm * 0.5f * s_pulse_per_mm + 0.5f);
    } else {
        want = -(int32_t)((-delta_mm) * 0.5f * s_pulse_per_mm + 0.5f);
    }

    want *= MOTION_TRIM_SIGN;

    /* 段内累计上限。见头文件里的 MOTION_TRIM_ACCUM_MAX_MM ——
     * 一段里一直往同一个方向纠，内侧轮会越停越早，累计到一米的时候
     * 车已经在原地转了 90 度，而每一步单独看都只是"小小的修正"。
     *
     * want 是"半差"，所以上限也得取一半（全差 = 2 x want）。以前这里漏了
     * 0.5f，判据变成"全差上限"，而入参在上一步已经被钳到 ACCUM_MAX ——
     * 于是这个分支永远不触发，累计到顶只是被静默夹住，上层收不到信号。
     *
     * 撞上限时把期望值削平、照常落地，然后回 2 通知上层"这段该结束了"。
     * 只回报不落地是不行的：差速车的 80mm 正好折合九度，那一下等于一个
     * 脉冲都不纠，剩下的五米就这么歪着走完。
     * 边界用 > 而不是 >=：正好等于上限不算超。 */
    {
        int32_t limit = (int32_t)(MOTION_TRIM_ACCUM_MAX_MM * 0.5f * s_pulse_per_mm);

        if (want > limit) {
            want = limit;
            capped = 1;
        } else if (want < -limit) {
            want = -limit;
            capped = 1;
        }
    }

    /* 削平之后才按差值落地 —— step 必须用"钳过的 want"来算，
     * 否则到顶的那一拍会把超出的部分照样发出去。
     *
     * 目标没变就别动。这一步既是死区，也是"绝对量"的实现：
     * 上层连续发同一个期望值时，只有第一次会真的改目标。
     * 返回 StepMotor_IsAnyRunning 而不是 0：0 是"四路都停了"的意思，
     * 上层拿它回 ERR_STATE。目标没变不等于车没在走。 */
    step = want - s_trim_accum;

    if (step == 0) {
        return StepMotor_IsAnyRunning();
    }

    /* 先按"这一侧还剩多少脉冲"把这次能落地的量削到真实值。
     *
     * 夹位规则在 StepMotor_TrimRoom 里，和 AdjustTarget 共用同一条 ——
     * motion 层只负责取两侧的较小值，不再自己写一遍"SET 必须大于 CNT"。
     * 被夹住的时候实际落地的比 step 少，这里仍按 want 记账的话，固件会
     * 以为纠了、其实没纠，后面每一拍都在补一个不存在的缺口。 */
    {
        int32_t room  = (step > 0) ? StepMotor_TrimRoom(MOTOR2)
                                   : StepMotor_TrimRoom(MOTOR1);
        int32_t other = (step > 0) ? StepMotor_TrimRoom(MOTOR4)
                                   : StepMotor_TrimRoom(MOTOR3);

        if (other < room) {
            room = other;
        }
        if (room <= 0) {
            /* 这一侧已经到头，没有地方再减了。这不是错误：5 米的段、累计
             * 上限 80mm，被减的那一侧最多早到 40mm，也就是段末最后几毫米。
             * 返回"这一拍不用动"，而不是 2（2 的意思是"该重新对齐了"，那是
             * 累计上限那条路的事）。 */
            return StepMotor_IsAnyRunning();
        }

        if (step > room)  { step = room; }
        if (step < -room) { step = -room; }
    }

    /* 向右修正 = 左边多走、右边少走。
     * 注意这不是旋转：四个轮子都还在往前走，只是目标脉冲数一边加一边减，
     * 车走出来的是一段很缓的弧。每 100mm 纠一次，宏观上看就是直线。 */
    if (StepMotor_AdjustTarget(MOTOR1, +step) == 0) { moved = 1; }   /* 左前多走 */
    if (StepMotor_AdjustTarget(MOTOR3, +step) == 0) { moved = 1; }   /* 左后多走 */
    if (StepMotor_AdjustTarget(MOTOR2, -step) == 0) { moved = 1; }   /* 右前少走 */
    if (StepMotor_AdjustTarget(MOTOR4, -step) == 0) { moved = 1; }   /* 右后少走 */

    /* 四路都没落地（车已经停了）就别记这个目标，下一拍还要再来一次。
     * 记上的话，车重新起步时这段纠偏会被悄悄吃掉。 */
    if (moved) {
        s_trim_accum += step;   /* 只记真正落地的量 */
    }

    /* 目标改过之后，"两侧同时到达"就失效了 —— 按剩余脉冲重算一次 ARR。
     *
     * 不重算的话：段首在 DriveAll / DriveLR 里同步好的比例，被第一次
     * Trim 打回"一侧先停"，而且越来越偏。5 米的直线段每 100mm 纠一次，
     * 一路纠下来等于段首那次同步白做了。
     *
     * 这一步改的是 ARR，不是脉冲数——纠偏该转多少度只由左右行程差决定，
     * 与是否同时到达无关。所以重算 ARR 不会改变纠偏量，只改变走出来的
     * 形状（折线变圆弧）。 */
    {
        int32_t r1 = StepMotor_GetRemainOf(MOTOR1);
        int32_t r2 = StepMotor_GetRemainOf(MOTOR2);

        if (r1 >= MOTION_SYNC_MIN_REMAIN && r2 >= MOTION_SYNC_MIN_REMAIN) {
            DriveSyncArr((uint32_t)r1, (uint32_t)r2);
        }
    }

    return moved ? (uint8_t)(capped ? 2 : 1) : 0;
}

void Motion_DriveLR(float left_mm, float right_mm, uint8_t forward)
{
    uint32_t pl;
    uint32_t pr;
    uint32_t p1;
    uint32_t p2;
    uint32_t p3;
    uint32_t p4;

    if (!s_ready) {
        return;
    }

    pl = Motion_MmToPulse(left_mm);
    pr = Motion_MmToPulse(right_mm);

    /* SetPulse(0) 会停机。这里给 1 保持"这一路有目标"，和 DriveAll 一致。 */
    if (pl == 0) { pl = 1; }
    if (pr == 0) { pr = 1; }

    /* 轮径补偿先乘进来，后面算同步比例要用"真正发出去的脉冲数"。
     * 用左右指令值算的话，k 系数带来的那点差会被漏掉 —— 虽然现在只差
     * 百分之几，但同步的全部意义就是把差抹平，从源头就用错数没必要。 */
    p1 = (uint32_t)((float)pl * s_k[0] + 0.5f);
    p3 = (uint32_t)((float)pl * s_k[2] + 0.5f);
    p2 = (uint32_t)((float)pr * s_k[1] + 0.5f);
    p4 = (uint32_t)((float)pr * s_k[3] + 0.5f);

    if (p1 == 0) { p1 = 1; }
    if (p3 == 0) { p3 = 1; }
    if (p2 == 0) { p2 = 1; }
    if (p4 == 0) { p4 = 1; }

    /* 新的一段开始：累计纠偏归零。这是"每 5 米重新对齐一次"的落点 ——
     * 不归零的话，跨段累计迟早撞上 MOTION_TRIM_ACCUM_MAX_MM，
     * 然后整条线后半段都纠不动了，而日志里只有一串 ERR_RANGE。 */
    s_trim_accum = 0;

    StepMotor_SetDirection(MOTOR1, forward ? M1_FWD : M1_REV);
    StepMotor_SetDirection(MOTOR3, forward ? M3_FWD : M3_REV);
    StepMotor_SetDirection(MOTOR2, forward ? M2_FWD : M2_REV);
    StepMotor_SetDirection(MOTOR4, forward ? M4_FWD : M4_REV);

    /* 关斜坡再启动，理由和 Motion_ForwardFlat 一样，但这里更强：
     * DriveLR 会被视觉伺服按 200mm 一段连续调用，段与段之间只有几微秒的
     * 间隙。每段都起一次坡的话，速度在 555Hz 和 3.3kHz 之间来回跳，
     * 画出来的线忽粗忽细，而且分段越多越明显 —— 纠偏反而把线画坏了。
     *
     * 斜坡是给"人按一下走一段"的单次指令准备的，不是给伺服用的。
     *
     * 先关斜坡、再定 ARR，这个顺序不能反：StepMotor_SetSpeedLR 在
     * 某一路的 ramp_on 为真时只记下目标值不写寄存器（那是为了让斜坡
     * 自己一档档走过去）。虽然段与段首尾相接时 ramp_on 恒为 0，
     * 但没有理由让正确性依赖那个前提。 */
    StepMotor_SetRampEnabled(0);

    /* 左右不同距离时两组分开定 ARR，让两侧同时到达 —— 这是 DriveLR
     * 和 DriveAll 唯一实质的差别。差速转弯（一边 1000mm 一边 500mm）
     * 以前是内侧走一半就停、外侧单独推着车头拐，走出来是折线；
     * 现在是全程按 1:2 的速度比走一条圆弧，两者行程差一样，转向角一样，
     * 但路径形状和车头的角速度都平顺了。
     *
     * 注意组代表取 p1 / p2，也就是 M1 / M2。差速车的 s_k 保证同组两个
     * 轮子的 k 相同，所以 p1 == p3、p2 == p4，取哪个都一样。 */
    DriveSyncArr(p1, p2);

    StepMotor_SetPulse(MOTOR1, p1);
    StepMotor_SetPulse(MOTOR3, p3);
    StepMotor_SetPulse(MOTOR2, p2);
    StepMotor_SetPulse(MOTOR4, p4);

    StepMotor_SetRampEnabled(1);
}

#if MOTION_HAS_MECANUM

uint8_t Motion_MoveLeft(float mm)
{
    uint32_t p;

    if (!s_ready || mm <= 0.0f) {
        return 1;
    }

    p = Motion_MmToPulse(mm);
    if (p == 0) {
        p = 1;
    }

    /* 麦轮横移：左前/左后 与 右前/右后 两组反向，组内同向。
     * 与现有 StepMotor_MOVE_LEFT 的方向组合保持一致。 */
    StepMotor_SetRampEnabled(0);
    StepMotor_SetDirection(MOTOR1, DIR_CCW);
    StepMotor_SetDirection(MOTOR2, DIR_CCW);
    StepMotor_SetDirection(MOTOR3, DIR_CW);
    StepMotor_SetDirection(MOTOR4, DIR_CW);
    StepMotor_SetPulse(MOTOR1, p);
    StepMotor_SetPulse(MOTOR2, p);
    StepMotor_SetPulse(MOTOR3, p);
    StepMotor_SetPulse(MOTOR4, p);
    StepMotor_SetRampEnabled(1);
    return 0;
}

uint8_t Motion_MoveRight(float mm)
{
    uint32_t p;

    if (!s_ready || mm <= 0.0f) {
        return 1;
    }

    p = Motion_MmToPulse(mm);
    if (p == 0) {
        p = 1;
    }

    StepMotor_SetRampEnabled(0);
    StepMotor_SetDirection(MOTOR1, DIR_CW);
    StepMotor_SetDirection(MOTOR2, DIR_CW);
    StepMotor_SetDirection(MOTOR3, DIR_CCW);
    StepMotor_SetDirection(MOTOR4, DIR_CCW);
    StepMotor_SetPulse(MOTOR1, p);
    StepMotor_SetPulse(MOTOR2, p);
    StepMotor_SetPulse(MOTOR3, p);
    StepMotor_SetPulse(MOTOR4, p);
    StepMotor_SetRampEnabled(1);
    return 0;
}

#endif /* MOTION_HAS_MECANUM —— 差速车到此为止，下面没有横移这两个函数 */

uint8_t Motion_IsAnyRunning(void)
{
    return StepMotor_IsAnyRunning();
}
