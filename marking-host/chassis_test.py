"""双车型固件的工程级检查。

这两个 Target 的差别只在 Define 里那一个宏，别的地方（文件列表）必须完全
一样。这种"只差一点点"的配置最怕被无声改坏：

    两个 Target 都不加宏     -> chassis.h 的 #error 会拦住，编译失败
    两个 Target 都加同一个宏  -> 编译通过，但两个 hex 一模一样
                                现场刷了"差速版"却是麦轮固件，横移命令还能用
    输出目录相同            -> 两套 .o 互相覆盖
                                先编 A 再编 B，A 的部分目标文件是旧的，
                                链接出来的固件是两套代码拼起来的
                                而 Keil 不会提醒，因为每个文件"都存在"

第三种最隐蔽，而且症状是随机的（取决于改了哪些文件）。所以这几条都必须钉住。
"""
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
FIRMWARE = ROOT.parent / "marking-firmware"
PROJECT = FIRMWARE / "Project.uvprojx"


def read_gbk(path):
    return path.read_bytes().decode("gbk", errors="replace")


def targets():
    """从工程文件里取出每个 Target 的关键配置。"""
    text = PROJECT.read_text(encoding="utf-8", errors="replace")
    out = []
    for block in re.split(r"(?=<Target>)", text):
        if not block.strip().startswith("<Target>"):
            continue
        name = re.search(r"<TargetName>([^<]*)</TargetName>", block)
        define = re.search(r"<Define>([^<]*)</Define>", block)
        objdir = re.search(r"<OutputDirectory>([^<]*)</OutputDirectory>", block)
        outname = re.search(r"<OutputName>([^<]*)</OutputName>", block)
        hexfile = re.search(r"<CreateHexFile>([^<]*)</CreateHexFile>", block)
        files = re.findall(r"<FilePath>([^<]*)</FilePath>", block)
        out.append({
            "name": name.group(1) if name else "",
            "define": define.group(1) if define else "",
            "objdir": objdir.group(1) if objdir else "",
            "outname": outname.group(1) if outname else "",
            "hex": hexfile.group(1) if hexfile else "",
            "files": tuple(sorted(files)),
        })
    return out


def test_two_targets_exist():
    ts = targets()
    names = [t["name"] for t in ts]
    assert names == ["Mec_Car", "4WD_Car"], f"Target 列表不对: {names}"


def test_each_target_defines_exactly_one_chassis():
    """每个 Target 必须且只能带一个车型宏。"""
    by_name = {t["name"]: t for t in targets()}

    assert "MEC_CAR" in by_name["Mec_Car"]["define"]
    assert "_4WD_CAR" not in by_name["Mec_Car"]["define"]

    assert "_4WD_CAR" in by_name["4WD_Car"]["define"]
    assert "MEC_CAR" not in by_name["4WD_Car"]["define"]


def test_targets_share_the_same_file_list():
    """文件列表必须完全一样。

    不一样的话，说明加 .c 的时候只加到了一个 Target 上 —— 另一个编译
    不会报"缺文件"，只会在链接时报一堆 undefined，或者更糟：某个模块
    悄悄用了旧版本的目标文件。
    """
    ts = targets()
    a, b = ts[0]["files"], ts[1]["files"]
    only_a = set(a) - set(b)
    only_b = set(b) - set(a)
    assert not only_a, f"只在 Mec_Car 里: {only_a}"
    assert not only_b, f"只在 4WD_Car 里: {only_b}"


def test_targets_do_not_share_output_directory():
    """输出目录必须分开，否则两套 .o 互相覆盖。"""
    ts = targets()
    dirs = [t["objdir"].lower() for t in ts]
    assert dirs[0] != dirs[1], f"两个 Target 共用输出目录 {dirs[0]}"

    names = [t["outname"] for t in ts]
    assert len(set(names)) == 2, f"输出名重复: {names}"


def test_each_target_produces_hex():
    """要 hex 才能归档、才能拿给现场刷。

    Keil 默认是不生成的（CreateHexFile=0），所以两个 Target 里都得显式打开。
    """
    for t in targets():
        assert t["hex"] == "1", f"{t['name']} 没开 CreateHexFile，不会生成 hex"


def test_chassis_header_guards_against_missing_define():
    """chassis.h 必须拦住"一个宏都没定义"和"两个都定义"。"""
    text = read_gbk(FIRMWARE / "Hardware" / "chassis.h")

    assert "#error" in text, "chassis.h 里一个 #error 都没有"

    # 两个都定义
    assert re.search(
        r"#if\s+defined\s*\(?\s*MEC_CAR\s*\)?\s*&&\s*defined\s*\(?\s*_4WD_CAR",
        text,
    ), "缺少「两个宏同时定义」的拦截"

    # 一个都没定义
    assert re.search(
        r"#if\s+!\s*defined\s*\(?\s*MEC_CAR.*!\s*defined\s*\(?\s*_4WD_CAR",
        text,
        re.S,
    ), "缺少「两个宏都没定义」的拦截"


def test_chassis_capabilities_differ():
    """两个车型的横移能力必须真的不同 —— 否则条件编译等于没写。"""
    text = read_gbk(FIRMWARE / "Hardware" / "chassis.h")

    mec = re.search(
        r"#define\s+CHASSIS_HAS_MECANUM\s+(\d)", text
    )
    assert mec, "找不到 CHASSIS_HAS_MECANUM"
    values = re.findall(r"#define\s+CHASSIS_HAS_MECANUM\s+(\d)", text)
    assert len(values) == 2, f"应该每个车型一份 CHASSIS_HAS_MECANUM，实际 {values}"
    assert values[0] == "1" and values[1] == "0", (
        f"顺序必须是麦轮(1)在前、差速(0)在后，实际 {values}"
    )


def test_motion_layer_reads_chassis_header():
    """motion.h 必须包含 chassis.h，否则 CHASSIS_* 宏在那边是未定义的。

    未定义时 #if 把标识符当 0 处理 —— 差速分支会被静默选中。
    编麦轮固件却变成"不能横移"，编译还不报错。
    """
    text = read_gbk(FIRMWARE / "Hardware" / "motion.h")
    assert '#include "chassis.h"' in text, "motion.h 没有包含 chassis.h"


def test_lateral_code_is_compiled_out_for_4wd():
    """横移的调用点必须在 #if CHASSIS_HAS_MECANUM 里面。

    只把函数体 #if 掉是不够的：调用点还在，链接时找不到符号。
    只把调用点 #if 掉也不够：函数还在，白占 Flash 还得维护。
    两头都要。
    """
    motion_c = read_gbk(FIRMWARE / "Hardware" / "motion.c")
    system_c = read_gbk(FIRMWARE / "Hardware" / "system.c")

    # motion.c：两个横移函数必须在同一个 #if 块里
    assert re.search(
        r"#if\s+MOTION_HAS_MECANUM\s*\n+uint8_t\s+Motion_MoveLeft", motion_c
    ), "motion.c 里 Motion_MoveLeft 没被 #if 包住"

    # system.c：StartMove 里的调用点也要包住
    m = re.search(r"static void StartMove\(void\)\s*\{(.{0,400})", system_c, re.S)
    assert m, "找不到 StartMove"
    body = m.group(1)
    assert "#if CHASSIS_HAS_MECANUM" in body, "StartMove 里的横移调用没被 #if 包住"


def test_both_targets_built_binaries_if_present():
    """如果两个 hex 都已经编出来了，体积必须不同。

    相同 = 条件编译没生效，两套固件是一份代码。
    这一条只在已经编译过的时候检查，没编译过就跳过 —— 测试不该依赖构建产物。
    """
    hec = FIRMWARE / "Objects" / "Mec" / "Marking_Mec.hex"
    hwd = FIRMWARE / "Objects" / "4WD" / "Marking_4WD.hex"
    if not (hec.exists() and hwd.exists()):
        return

    size_mec = hec.stat().st_size
    size_4wd = hwd.stat().st_size
    assert size_mec != size_4wd, (
        f"两套固件体积相同（{size_mec} 字节），说明条件编译没生效"
    )
    assert size_4wd < size_mec, (
        f"差速版（{size_4wd}）应该比麦轮版（{size_mec}）小 —— 它少了横移代码"
    )


if __name__ == "__main__":
    tests = [(k, v) for k, v in sorted(globals().items()) if k.startswith("test_")]
    failed = 0
    for name, fn in tests:
        try:
            fn()
        except AssertionError as e:
            failed += 1
            print(f"FAIL {name}: {e}")
        else:
            print(f"ok {name}")

    print()
    if failed:
        print(f"{failed} 项失败")
        sys.exit(1)
    print("全部通过")
