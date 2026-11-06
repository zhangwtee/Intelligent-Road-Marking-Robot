"""左右同步的检查：差速动作是不是真的走成了一条圆弧。

四路各有独立时基之后，"内侧先停"这件事就不再有理由存在了。但把同步写进
motion 层有两个很容易犯的错，两个都编译得过、也跑得动：

    比例写反        -> 脉冲多的一侧被调慢，车不是走弧线而是往反方向拐
    基准用错        -> 拿 GetSpeedTarget（转弯时被压到 CHASSIS_TURN_ARR）
                       当基准，每转一次弯，下一段直线就比上一段更慢

还有一条不算错、但会让改动白做：段中间 Motion_Trim 改过脉冲目标之后
不重算 ARR，那么段首同步好的比例在第一次纠偏之后立刻失效。

数学部分用一份 Python 镜像实现来验证。镜像和 C 里的公式必须一一对应，
所以下面每个常量都从源码里读，不在这里写死。
"""
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
FW = ROOT.parent / "marking-firmware"
MOTION_C = FW / "Hardware" / "motion.c"
MOTION_H = FW / "Hardware" / "motion.h"
CHASSIS_H = FW / "Hardware" / "chassis.h"
SM_C = FW / "Hardware" / "StepMotor_New.c"
SM_H = FW / "Hardware" / "StepMotor_New.h"

TESTS = []


def test(fn):
    TESTS.append(fn)
    return fn


def read_gbk(path):
    return path.read_bytes().decode("gbk", errors="replace")


def func_body(text, name):
    """取出 C 函数体（按花括号配对，避免嵌套提前截断）。"""
    m = re.search(rf"\b{name}\s*\([^)]*\)\s*\{{", text)
    assert m, f"找不到函数 {name}"
    i = m.end() - 1
    depth = 0
    for j in range(i, len(text)):
        if text[j] == "{":
            depth += 1
        elif text[j] == "}":
            depth -= 1
            if depth == 0:
                return text[i:j + 1]
    raise AssertionError(f"{name} 的花括号不配对")


# ---------- 镜像实现：与 C 里的 DriveSyncArr 一一对应 ----------
def const(name):
    text = read_gbk(SM_C)
    m = re.search(rf"#define\s+{name}\s+(\d+)", text)
    assert m, f"StepMotor_New.c 里找不到 {name}"
    return int(m.group(1))


def sync_arr(pl, pr, base, deadband_pm):
    """返回 (left_arr, right_arr)。与 motion.c 的 DriveSyncArr 同构。"""
    if pl == 0 or pr == 0:
        return base, base

    hi = max(pl, pr)
    diff = abs(pl - pr)
    permille = diff * 1000 // hi
    if permille <= deadband_pm:
        return base, base

    # 与 C 里的 RatioArr 一致：base * 多的一侧 / 少的一侧，四舍五入
    if pl > pr:
        al, ar = base, (base * pl + pr // 2) // pr
    else:
        al, ar = (base * pr + pl // 2) // pl, base

    ar = min(ar, 0xFFFF)
    al = min(al, 0xFFFF)

    lo, hi_arr = const("SPEED_FAST_ARR"), const("SPEED_SLOW_ARR")
    return min(max(al, lo), hi_arr), min(max(ar, lo), hi_arr)


def duration(pulse, arr):
    """走完这段需要的"时间单位"（实际时间 = 这个数 / 时钟频率）。"""
    return pulse * arr


# ---------- 结构检查 ----------
@test
def test_sync_switch_in_common_section():
    """同步开关必须在公共段，两种车型都要有，不能只在差速里定义。"""
    text = read_gbk(CHASSIS_H)
    # 注意不能用 "#if defined(MEC_CAR)" 直接 split：文件开头那句
    # "#if defined(MEC_CAR) && defined(_4WD_CAR)" 也含这个子串。
    parts = re.split(r"^#if defined\(MEC_CAR\)\s*$", text, flags=re.M)
    assert len(parts) == 2, "找不到车型分支的起点"
    head, rest = parts
    body_4wd = re.split(r"^#else\s+/\* _4WD_CAR \*/\s*$", rest, flags=re.M)[1]
    body_mec = re.split(r"^#else\s+/\* _4WD_CAR \*/\s*$", rest, flags=re.M)[0]

    assert re.search(r"#define\s+CHASSIS_SYNC_ARR\s+[01]\b", head), \
        "公共段里没有 CHASSIS_SYNC_ARR"
    assert re.search(r"#define\s+CHASSIS_SYNC_DEADBAND_PM\s+\d+", head), \
        "公共段里没有 CHASSIS_SYNC_DEADBAND_PM"

    # 两个车型分支里都不许再重复定义（会覆盖公共段的值）
    for name, seg in (("MEC", body_mec), ("4WD", body_4wd)):
        assert "#define CHASSIS_SYNC_ARR" not in seg, \
            f"{name} 分支里重复定义了 CHASSIS_SYNC_ARR"


@test
def test_speed_base_api():
    """GetSpeedBase 必须真的返回 speed_base，不能顺手返回 speed_arr。"""
    assert "StepMotor_GetSpeedBase" in read_gbk(SM_H), "头文件缺 StepMotor_GetSpeedBase"
    body = func_body(read_gbk(SM_C), "StepMotor_GetSpeedBase")
    assert "speed_base" in body, "GetSpeedBase 返回的不是 speed_base"
    assert "speed_arr" not in body, "GetSpeedBase 里混进了 speed_arr"


@test
def test_remain_api():
    """剩余脉冲接口要有，且无限行程不能返回一个会被当成"还剩很多"的小数。"""
    assert "StepMotor_GetRemainOf" in read_gbk(SM_H), "头文件缺 StepMotor_GetRemainOf"
    body = func_body(read_gbk(SM_C), "StepMotor_GetRemainOf")
    assert "JOG_PULSE_INFINITE" in body, "GetRemainOf 没处理无限行程"
    assert "m->set" in body and "m->cnt" in body, "GetRemainOf 不是 set - cnt"


@test
def test_sync_used_by_all_three_paths():
    """DriveAll / DriveLR / Trim 三条路都要走同步，少一条就退回硬停。"""
    text = read_gbk(MOTION_C)
    assert re.search(r"static\s+void\s+DriveSyncArr\s*\(", text), "没有 DriveSyncArr"
    for fn in ("DriveAll", "Motion_DriveLR", "Motion_Trim"):
        body = func_body(text, fn)
        assert "DriveSyncArr(" in body, f"{fn} 没有调用 DriveSyncArr"


@test
def test_drivelr_no_leftover_restore():
    """DriveLR 里不该再有"先恢复档位再设脉冲"的旧路径。

    留着它不会错，但会在同一段里写两次寄存器：先 RestoreSpeed 拉回用户档位，
    紧接着 SetSpeedLR 又改成比例值，中间那一下是一次无意义的频率扰动。
    """
    body = func_body(read_gbk(MOTION_C), "Motion_DriveLR")
    assert "StepMotor_RestoreSpeed" not in body, \
        "DriveLR 里还留着 RestoreSpeed，会和 SetSpeedLR 打架"


@test
def test_sync_off_fallback():
    """关掉同步时必须回退到"恢复用户档位 + 同频"，而不是什么都不做。

    什么都不做的话，转过弯的限速会一直留着，车越走越慢。
    """
    body = func_body(read_gbk(MOTION_C), "DriveSyncArr")
    assert "#else" in body, "DriveSyncArr 没有关闭同步的分支"
    tail = body.split("#else")[1]
    assert "StepMotor_RestoreSpeed" in tail, "关闭同步的分支里没有恢复档位"


@test
def test_trim_recomputes_by_remain():
    """Trim 之后要按"剩余"重算，不能按本段总量。"""
    body = func_body(read_gbk(MOTION_C), "Motion_Trim")
    assert "GetRemainOf" in body, "Trim 没有按剩余脉冲重算"
    assert re.search(r"MOTION_SYNC_MIN_REMAIN", body), \
        "Trim 里没有剩余行程门限（剩几十个脉冲时重算没有意义）"


# ---------- 数学检查 ----------
@test
def test_equal_arrival():
    """两侧必须同时到达：误差不超过 1 个 ARR 的量。"""
    deadband = int(re.search(r"#define\s+CHASSIS_SYNC_DEADBAND_PM\s+(\d+)",
                             read_gbk(CHASSIS_H)).group(1))
    for pl, pr in [(5000, 2500), (26500, 26200), (1000, 900),
                   (3000, 2000), (1200, 4000), (26500, 5300)]:
        base = 300
        al, ar = sync_arr(pl, pr, base, deadband)
        tl, tr = duration(pl, al), duration(pr, ar)
        rel = abs(tl - tr) / max(tl, tr)
        assert rel < 0.005, f"{pl}:{pr} 两侧不同步，相对差 {rel:.3%}（{al}/{ar}）"


@test
def test_segment_duration_unchanged():
    """同步不拉长整段：总时长仍等于"最多那一侧在用户档位下走完"。"""
    deadband = 8
    worst = 0.0
    for pl, pr in [(5000, 2500), (26500, 5300), (26500, 26400), (900, 3000)]:
        base = 300
        al, ar = sync_arr(pl, pr, base, deadband)
        t = max(duration(pl, al), duration(pr, ar))
        t_ref = max(pl, pr) * base
        rel = abs(t - t_ref) / t_ref
        worst = max(worst, rel)
    assert worst < 0.005, f"整段被拉长了 {worst:.2%}，同步不该改变总时长"


@test
def test_slower_side_has_more_arr():
    """脉冲少的一侧 ARR 必须更大（更慢），反了就是往反方向拐。"""
    deadband = 8
    al, ar = sync_arr(5000, 2500, 300, deadband)   # 左多右少
    assert al < ar, f"左多右少时左 ARR 应更小（更快），实际 {al}/{ar}"
    al, ar = sync_arr(2500, 5000, 300, deadband)   # 左少右多
    assert al > ar, f"左少右多时左 ARR 应更大（更慢），实际 {al}/{ar}"


@test
def test_deadband_suppresses_tiny_ratio():
    """0.5% 的左右差要落进死区，不许为它改写寄存器。"""
    deadband = 8
    al, ar = sync_arr(26500, 26400, 300, deadband)   # 差 0.375%
    assert (al, ar) == (300, 300), f"死区没生效，算出 {al}/{ar}"


@test
def test_extreme_ratio_is_clamped():
    """极端比例（左右差 4 倍）会顶到最慢档，但不许溢出或回绕。"""
    deadband = 8
    al, ar = sync_arr(4000, 1000, 300, deadband)
    lo, hi = const("SPEED_FAST_ARR"), const("SPEED_SLOW_ARR")
    for v in (al, ar):
        assert lo <= v <= hi, f"ARR {v} 超出 {lo}~{hi}"
    assert al >= 300 and ar >= 300, "同步把某一侧调得比用户档位还快，方向错了"


@test
def test_huge_pulse_does_not_wrap():
    """超长行程 + 极小对侧：中间乘积会爆 32 位，结果必须饱和而不是回绕。

    回绕的表现是那一侧 ARR 突然变成很小的正数 —— 也就是说，本该最慢的
    那个轮子会在半路全速冲出去。这类 bug 只在极端参数下出现，现场很难复现，
    只能在数学上守住。
    """
    deadband = 8
    for pl, pr in [(400000, 50), (400000, 1), (50, 400000)]:
        al, ar = sync_arr(pl, pr, 300, deadband)
        lo, hi = const("SPEED_FAST_ARR"), const("SPEED_SLOW_ARR")
        for v in (al, ar):
            assert lo <= v <= hi, f"{pl}:{pr} 算出 ARR={v}，超出 {lo}~{hi}（回绕了）"
        # 快的永远是脉冲多的那一侧
        if pl > pr:
            assert al < ar, f"{pl}:{pr} 应左快右慢，实际 {al}/{ar}"
        else:
            assert ar < al, f"{pl}:{pr} 应右快左慢，实际 {al}/{ar}"


def main():
    for fn in TESTS:
        try:
            fn()
        except AssertionError as exc:
            print(f"FAIL {fn.__name__}: {exc}")
            return 1
        print(f"ok {fn.__name__}")
    print("全部通过")
    return 0


if __name__ == "__main__":
    sys.exit(main())
