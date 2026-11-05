"""
指令表自检。跑了它才知道面板发的字节和固件认的字节对得上：

    python3 protocol_test.py     # 或 pytest protocol_test.py

不对齐的代价不是报错，是按下按钮车不动 —— 所以让它在改代码时就响。
"""

import re
import time
from pathlib import Path

from core import frame
from core.protocol import COMMANDS, PANEL_BUTTONS, CAMERA_BUTTONS, Protocol

# 下位机 HandleCmd 认识的全部指令 —— 直接从固件源码里抠，不再手抄。
# 原来这里是 set("+-ADFGJKPRTUWXYZ")，手抄的，加个指令忘了同步就查不出来。
FIRMWARE_HW = (
    Path(__file__).resolve().parent.parent
    / "marking-firmware" / "Hardware"
)

FIRMWARE_C = FIRMWARE_HW / "system.c"
FIRMWARE_PROTO_H = FIRMWARE_HW / "proto.h"

FIRMWARE_COMMANDS = set(
    re.findall(r"case '([^']+)':", FIRMWARE_C.read_text(encoding="gbk"))
)

# 下位机还没实现的指令：按下去是个空操作。写在这儿是为了让"还没做"和
# "悄悄漏了"分得开 —— 固件一补上，test_not_implemented_is_still_true 就响。
NOT_IMPLEMENTED = {"N"}


def test_panel_commands_are_known():
    """每个按钮发的字节，上位机自己得能翻译，否则日志全是"未知"。"""
    for label, cmd in PANEL_BUTTONS:
        assert len(cmd) == 1, f"{label}: 必须是单字节"
        assert cmd[0] in COMMANDS, f"{label}: {cmd!r} 不在指令表里"


def test_covers_firmware():
    """下位机能回显的每个字节都要有翻译，不然面板会显示"未知"。"""
    missing = sorted(c for c in FIRMWARE_COMMANDS if ord(c) not in COMMANDS)
    assert not missing, f"固件能回但指令表里没有: {missing}"


def test_panel_commands_reach_firmware():
    """面板发的每个字节固件都得认识 —— 不认识不是报错，是按下车不动。

    这条就是 C 当初漏掉的那条：指令表里有、按钮也在，固件 switch 里没有。
    """
    sent = {cmd[0] for _, cmd in PANEL_BUTTONS}
    known = {ord(ch) for ch in FIRMWARE_COMMANDS}
    unknown = sorted(
        chr(b) for b in sent - known if chr(b) not in NOT_IMPLEMENTED
    )
    assert not unknown, f"面板发得出去、固件不认: {unknown}"


def test_firmware_has_no_code_swallowed_by_a_comment():
    """插代码插到了行尾 // 注释里 —— 只有编译器看得见。

    实际踩过两次：system.h 枚举那个逗号落进了注释（STATE_PAINT = 'P'  //,），
    以及 system.c 里 Pump_Off(); 被插到 StepMotor_StopAll(); 前面同一行。
    两次都是「在行尾注释后面插东西」——捕获组连着注释一起匹配了。
    """
    bad = []

    for path in sorted(FIRMWARE_C.parent.glob("*.c")) + sorted(
        FIRMWARE_C.parent.glob("*.h")
    ):
        # 固件编码不一（GBK/UTF-8/纯 ASCII），这里只看 ASCII 结构，乱码无所谓
        text = path.read_bytes().decode("gbk", errors="replace")

        for lineno, line in enumerate(text.splitlines(), 1):
            if "//" not in line:
                continue

            code, comment = line.split("//", 1)

            # 整行注释不算：那是正常注释掉的旧代码，不是被吃掉的活代码
            if not code.strip():
                continue

            # 1) 前面是真代码、又不以分隔符收尾，注释里却带着 ; 或 ,
            #    —— 枚举那个逗号就是这么进注释的
            dangling = (
                code.strip()
                and not code.rstrip().endswith((",", ";", "{", "}"))
                and (";" in comment or "," in comment)
            )

            # 2) 注释里藏着一整条语句（...(...);  或  ...=...;）
            #    —— Pump_Off(); 被插到 StopAll(); 前面同一行就是这样，
            #    这种情况前面那条语句是完整的，光看它收没收尾看不出来
            buried = re.search(r"\)\s*;|=\s*[^;]*;", comment)

            if dangling or buried:
                bad.append(f"{path.name}:{lineno}  {line.strip()}")

    assert not bad, "代码被行尾注释吃掉了:\n" + "\n".join(bad)


def test_not_implemented_is_still_true():
    """固件补上 N 之后，把它从 NOT_IMPLEMENTED 里划掉，别再留个谎。"""
    done = sorted(NOT_IMPLEMENTED & FIRMWARE_COMMANDS)
    assert not done, f"固件已经实现了，从 NOT_IMPLEMENTED 里删掉: {done}"


def test_camera_buttons_exist():
    """弹摄像头的那两个标签必须真的是面板上的按钮，写错就静默不弹。"""
    labels = {label for label, _ in PANEL_BUTTONS}
    assert CAMERA_BUTTONS <= labels, f"不在面板上的按钮: {CAMERA_BUTTONS - labels}"


def test_parse_splits_stream():
    """一次收到多个字节要逐个拆开；跨包边界也不能丢。"""
    p = Protocol()
    assert p.parse(b"") == []
    assert p.parse(b"P")[0]["status"] == "画车位（标准）"
    assert [r["char"] for r in p.parse(b"PGT")] == ["P", "G", "T"]
    assert [r["char"] for r in p.parse(b"K")] == ["K"]
    assert p.parse(b"\x00")[0]["status"] == "未知"


# ============================================================================
# 帧协议 —— 上下位机必须逐字节一致，所以这里的向量就是两边共用的契约
# ============================================================================


def test_crc8_matches_reference():
    """CRC-8/ATM 的标准校验值。两边都用它锚定，不是自己跟自己对。

    crc8(b"123456789") == 0xF4 是 CRC-8 poly=0x07 init=0x00 的公开标准值。
    如果哪天有人「优化」成查表或者改成 poly 0x31，这条会立刻响。
    """
    assert frame.crc8(b"123456789") == 0xF4
    assert frame.crc8(b"") == 0x00
    # 逐字节推进等价于一次算完，解析器分片喂数据时依赖这个性质
    acc = 0
    for b in b"123456789":
        acc = frame.crc8(bytes([b]), acc)
    assert acc == 0xF4


def test_frame_layout_is_fixed_10_bytes():
    f = frame.build_frame(frame.CMD_PING, seq=0x5A)
    assert len(f) == frame.FRAME_LEN == 10
    assert f[0] == 0x7B and f[-1] == 0x7D
    assert f[1] == frame.CMD_PING and f[2] == 0x5A
    assert f[8] == frame.crc8(f[:8])


def test_respray_is_big_endian_millimetres():
    """400mm 的帧：DATA[0]=0x01 DATA[1]=0x90，下位机按大端拼回。

    这是「视觉的偏移量怎么传下去」的实际字节：不再是两条命令，
    而是一个 uint16 参数。
    """
    f = frame.build_respray(400, seq=1)
    assert f[1] == frame.CMD_RESPRAY
    assert f[3] == 0x01 and f[4] == 0x90
    assert frame.check_frame(f)
    assert frame.describe(f[1], f[3:8]) == "补喷 400mm"


def test_move_frame_carries_direction_and_distance():
    f = frame.build_move(frame.MOVE_BACK, 1200, seq=7)
    assert f[3] == frame.MOVE_BACK
    assert (f[4] << 8 | f[5]) == 1200
    assert frame.check_frame(f)


def test_payload_too_long_is_rejected():
    """超长要报错，不能静默截断 —— 截断的帧比发不出去更难查。"""
    try:
        frame.build_frame(frame.CMD_RESPRAY, 0, b"\x00" * 6)
    except ValueError:
        pass
    else:
        raise AssertionError("6 字节 payload 应该报错")


def test_frame_constants_match_firmware():
    """上下位机各写一遍常量，靠这条钉住。

    TYPE 表和单字节命令表一样是「两处配置」：固件 proto.h 一份、
    上位机 frame.py 一份。改一边忘另一边，症状是帧发出去被回
    「未知命令」——不报错、不崩溃，只是车不动。
    """
    text = FIRMWARE_PROTO_H.read_bytes().decode("gbk", errors="replace")
    defines = {
        m.group(1): int(m.group(2), 0)
        for m in re.finditer(
            r"#define\s+(PROTO_\w+)\s+(0x[0-9A-Fa-f]+|\d+)", text
        )
    }

    expected = {
        "PROTO_SOF": frame.SOF,
        "PROTO_EOF": frame.EOF,
        "PROTO_FRAME_LEN": frame.FRAME_LEN,
        "PROTO_DATA_LEN": frame.DATA_LEN,
        "PROTO_CMD_RESPRAY": frame.CMD_RESPRAY,
        "PROTO_CMD_MOVE": frame.CMD_MOVE,
        "PROTO_CMD_LEVEL": frame.CMD_LEVEL,
        "PROTO_CMD_PING": frame.CMD_PING,
        "PROTO_CMD_ACK": frame.CMD_ACK,
        "PROTO_CMD_STATUS": frame.CMD_STATUS,
        "PROTO_OK": frame.RESULT_OK,
        "PROTO_ERR_STATE": frame.RESULT_ERR_STATE,
        "PROTO_ERR_RANGE": frame.RESULT_ERR_RANGE,
        "PROTO_ERR_TYPE": frame.RESULT_ERR_TYPE,
        "PROTO_ERR_HW": frame.RESULT_ERR_HW,
        "PROTO_MOVE_FWD": frame.MOVE_FWD,
        "PROTO_MOVE_BACK": frame.MOVE_BACK,
        "PROTO_MOVE_LEFT": frame.MOVE_LEFT,
        "PROTO_MOVE_RIGHT": frame.MOVE_RIGHT,
        "PROTO_RESPRAY_MIN_MM": frame.RESPRAY_MIN_MM,
        "PROTO_RESPRAY_MAX_MM": frame.RESPRAY_MAX_MM,
        "PROTO_MOVE_MAX_MM": frame.MOVE_MAX_MM,
        "PROTO_CMD_TRIM": frame.CMD_TRIM,
        "PROTO_CMD_PUMP": frame.CMD_PUMP,
        "PROTO_CMD_PAINT": frame.CMD_PAINT,
        "PROTO_TRIM_MAX_MM": frame.TRIM_MAX_MM,
        "PROTO_PAINT_MIN_MM": frame.PAINT_MIN_MM,
        "PROTO_PAINT_MAX_MM": frame.PAINT_MAX_MM,
        "PROTO_CMD_GYRO": frame.CMD_GYRO,
        "PROTO_CMD_HEADING": frame.CMD_HEADING,
        "PROTO_GYRO_OFF": frame.GYRO_OFF,
        "PROTO_GYRO_ON": frame.GYRO_ON,
        "PROTO_GYRO_RECAL": frame.GYRO_RECAL,
        "PROTO_GYRO_MEASURE": frame.GYRO_MEASURE,
        "PROTO_ERR_BUSY": frame.RESULT_ERR_BUSY,
    }

    mismatched = {
        name: (defines.get(name), want)
        for name, want in expected.items()
        if defines.get(name) != want
    }
    assert not mismatched, f"固件与上位机常量不一致 (proto.h, frame.py): {mismatched}"


def test_trim_cap_is_the_same_everywhere():
    """纠偏上限写在了四个地方，必须一样。

        proto.h        PROTO_TRIM_MAX_MM      固件收帧时判越界用
        motion.c       MOTION_TRIM_MAX_MM     Motion_Trim 自己再夹一次
        heading.h      HDG_TRIM_ONCE_MAX_MM   陀螺仪闭环的单次输出
        lane_control   GAINS["MAX_TRIM_MM"]   视觉控制律输出

    四处不一致的症状各不相同，而且都不会报错：
        上位机给得比固件大 -> 帧被回 ERR_RANGE，车的表现是"完全不纠"
        上位机给得比 motion 大 -> Motion_Trim 静默夹掉，ACK 还是 OK
        陀螺仪给得比 motion 大 -> 同样被静默夹掉，但闭环还以为自己修了
                                 那么多，下次接着按这个量算，越纠越偏
    前两条像"协议不兼容"或"纠偏没效果"，第三条最隐蔽 —— 查起来都很远。
    """
    from core import lane_control

    proto_h = FIRMWARE_PROTO_H.read_bytes().decode("gbk", errors="replace")
    motion_c = (FIRMWARE_HW / "motion.c").read_bytes().decode("gbk", errors="replace")
    heading_h = (FIRMWARE_HW / "heading.h").read_bytes().decode("gbk", errors="replace")
    chassis_h = (FIRMWARE_HW / "chassis.h").read_bytes().decode("gbk", errors="replace")

    firmware_proto = int(
        re.search(r"#define\s+PROTO_TRIM_MAX_MM\s+(\d+)", proto_h).group(1)
    )
    firmware_heading = float(
        re.search(r"#define\s+HDG_TRIM_ONCE_MAX_MM\s+([\d.]+)f", heading_h).group(1)
    )

    # chassis.h 里每个车型分支各写了一遍，都要查。
    # 只查第一个的话，改车型参数时漏掉另一支（比如差速车给 30mm）不会被发现，
    # 而那一支恰恰是这台车要跑的那一支。
    chassis_limits = [
        float(v) for v in
        re.findall(r"#define\s+CHASSIS_TRIM_MAX_MM\s+([\d.]+)f", chassis_h)
    ]
    assert len(chassis_limits) >= 2, (
        f"chassis.h 里应该有每个车型一份 CHASSIS_TRIM_MAX_MM，实际 {chassis_limits}"
    )

    # motion.c 必须引用 chassis.h，不能又写死一个数字。
    # 写死的话，chassis.h 改了它不跟着变，而编译不报错、运行也不报错。
    assert re.search(
        r"#define\s+MOTION_TRIM_MAX_MM\s+CHASSIS_TRIM_MAX_MM", motion_c
    ), "motion.c 的 MOTION_TRIM_MAX_MM 必须引用 CHASSIS_TRIM_MAX_MM"

    assert firmware_proto == frame.TRIM_MAX_MM == firmware_heading
    assert all(v == frame.TRIM_MAX_MM for v in chassis_limits), (
        f"chassis.h 各车型上限 {chassis_limits} 与上位机 {frame.TRIM_MAX_MM} 不一致"
    )
    assert lane_control.GAINS["MAX_TRIM_MM"] <= frame.TRIM_MAX_MM


def test_move_frame_carries_both_wheels():
    """左右轮距离编码在 DATA[1:3] 和 DATA[3:5]。

    这是视觉纠偏唯一的下发通道：右轮 == 左轮是直行，不等是差速。
    布局错了不会报错 —— 车会朝一个固定的方向偏，像"机械装歪了"。
    """
    f = frame.build_move(frame.MOVE_FWD, 200, 195, seq=9)

    assert frame.check_frame(f)
    assert f[3] == frame.MOVE_FWD
    assert (f[4] << 8 | f[5]) == 200       # 左轮
    assert (f[6] << 8 | f[7]) == 195       # 右轮


def test_move_defaults_to_straight():
    """省掉 right_mm 时左右同距 —— 老的调用点不用改。"""
    f = frame.build_move(frame.MOVE_FWD, 1200, seq=1)
    assert (f[4] << 8 | f[5]) == 1200
    assert (f[6] << 8 | f[7]) == 1200


def test_trim_sign_survives_the_wire():
    """负数纠偏（向左）要能原样传过去。

    int16 走大端塞进 DATA[0:2]，解析端忘了减 0x10000 的话，-8 会变成
    65528 —— 固件判越界回 ERR_RANGE，表现是"只往一个方向纠得动"。
    """
    for value in (-20, -8, -1, 0, 1, 8, 20):
        f = frame.build_trim(value, seq=3)
        assert frame.check_frame(f)

        raw = (f[3] << 8) | f[4]
        if raw >= 0x8000:
            raw -= 0x10000

        assert raw == value, f"{value} 传成了 {raw}"


def test_pump_frame_encodes_on_off():
    on = frame.build_pump(True, seq=1)
    off = frame.build_pump(False, seq=2)

    assert on[3] == 1 and off[3] == 0
    assert frame.describe(on[1], on[3:8]) == "水泵 开"
    assert frame.describe(off[1], off[3:8]) == "水泵 关"


def test_paint_frame_defaults_are_the_standard_bay():
    """0 表示用固件默认值，两边默认值要一样（5.3m x 2.5m）。"""
    from core.line_quality import SPRAY_MM_PER_LEVEL  # noqa: F401  只为确认模块可导入

    f = frame.build_paint(0, 0, seq=1)
    assert frame.check_frame(f)
    assert frame.PAINT_LEN_DEFAULT_MM == 5300
    assert frame.PAINT_WID_DEFAULT_MM == 2500
    assert "5300x2500" in frame.describe(f[1], f[3:8])


def test_parser_resyncs_mid_stream():
    """帧后面跟着裸字节、裸字节后面跟着帧，两头都不能丢。"""
    p = Protocol()
    blob = (
        b"W"                                   # 老的单字节命令
        + frame.build_respray(600, seq=2)
        + b"C"                                 # 又一条裸字节
        + frame.build_level(3, seq=3)
    )
    out = p.parse(blob)
    assert [r["kind"] for r in out] == [
        "byte", "frame", "byte", "frame"
    ]
    assert out[0]["char"] == "W"
    assert out[1]["status"] == "补喷 600mm"
    assert out[2]["char"] == "C"
    assert out[3]["status"] == "设档 3（600mm）"
    assert p.dropped == 0


def test_parser_survives_false_header():
    """假帧头后面紧跟真帧，解析器必须收敛到真帧上。

    这是「失败只丢 1 字节」那条规则的唯一理由：如果 CRC 错就丢掉整 10 字节，
    真帧会被一起吞掉，然后永久失步 —— 而失步是静默的，看起来像「协议不兼容」。
    """
    p = Protocol()
    real = frame.build_respray(200, seq=9)
    garbage = b"\x7b\x01\x00" + b"\x00" * 6      # 半个假帧头，长度刚好 9
    out = p.parse(garbage + real)

    assert [r["kind"] for r in out] == ["frame"]
    assert out[0]["status"] == "补喷 200mm"
    assert p.dropped > 0          # 丢的是垃圾字节，不是真帧


def test_parser_rejects_corrupted_payload():
    """payload 被改一个字节，CRC 必须拦下 —— 这是「噪声不是指令」的全部意义。"""
    p = Protocol()
    f = bytearray(frame.build_respray(1000, seq=4))
    f[4] ^= 0x01                  # 1000 -> 1001，CRC 不再匹配
    out = p.parse(bytes(f))
    assert out == []
    assert p.dropped > 0


def test_corrupt_frame_payload_is_not_executed_as_commands():
    """坏帧的 payload 不能被逐字节当成命令执行。

    这条是加静默期的全部理由。构造一个 payload 里含 0x50（'P' = 画车位）的
    补喷帧，再把 CRC 改坏：如果只丢 1 字节重扫，0x50 会作为一条单字节命令
    被执行 —— 车开始画 5.3 米并喷漆，而 CRC 明明「拦住」了这个帧。
    """
    p = Protocol()
    f = bytearray(frame.build_frame(frame.CMD_RESPRAY, 1, b"\x50\x00\x00\x00\x00"))
    f[8] ^= 0xFF                  # 只坏 CRC，payload 原样
    out = p.parse(bytes(f))

    assert not [r for r in out if r.get("char") == "P"], "坏帧里的 'P' 被执行了"
    assert out == []
    assert p.dropped >= frame.FRAME_LEN - 1


def test_lone_frame_header_does_not_block_the_port():
    """一个孤立的 0x7B 不能把这一路彻底堵死。

    噪声里出现单个 0x7B 的概率是 1/256，很容易。如果它一直躺在队首，
    后面每条命令都会被当成「帧的第 2、3…个字节」，永远等不齐 10 字节 ——
    表现是「串口突然不响应了」，而且没有任何报错。所以半帧要有超时。
    """
    p = Protocol()
    t0 = time.monotonic()
    assert p.parse(b"\x7b", now=t0) == []

    out = p.parse(b"W", now=t0 + frame.PARTIAL_TIMEOUT_S + 0.01)
    assert [r["char"] for r in out] == ["W"]
    assert p.dropped >= 1


def test_resync_ends_after_idle_gap():
    """静默期不能把后面的正常命令一起吞掉：隔一段空闲就恢复。"""
    p = Protocol()
    f = bytearray(frame.build_respray(300, seq=1))
    f[8] ^= 0xFF
    p.parse(bytes(f), now=100.0)
    assert p.resync

    out = p.parse(b"W", now=100.0 + frame.RESYNC_IDLE_S + 0.05)
    assert [r["char"] for r in out] == ["W"]


def test_parser_splits_across_reads():
    """帧被串口按字节切成几段到达，必须能拼起来（read 的边界不可控）。"""
    p = Protocol()
    f = frame.build_move(frame.MOVE_FWD, 250, seq=11)
    out = []
    for byte in f:
        out.extend(p.parse(bytes([byte])))
    assert len(out) == 1
    assert out[0]["status"] == "前进 250mm"


def test_parser_drops_stale_partial_frame():
    """半帧卡住之后不能把 payload 当单字节命令执行。

    残帧里可能含有 'P'（0x50）—— 只丢帧头再往下解析的话，这个字节会被
    当成「画车位」执行。所以超时的残帧整段丢弃，一个字节都不执行。
    """
    p = Protocol()
    t0 = time.monotonic()
    f = frame.build_frame(frame.CMD_PING, 0, b"P\x00\x00\x00\x00")
    p.last_rx = t0
    out = p.parse(f[:5], now=t0)                 # 只到一半就断了
    assert out == []

    later = t0 + frame.PARTIAL_TIMEOUT_S + 0.01
    out = p.parse(b"", now=later)
    assert out == []
    assert p.pending() == 0
    assert p.dropped > 0


def test_status_frame_is_readable():
    """上行状态帧：状态字符 + 段号 + 标志 + 16 位脉冲。"""
    f = frame.build_frame(
        frame.CMD_STATUS, 0, bytes([ord("C"), 2, 0x01, 0x13, 0x88])
    )
    assert frame.check_frame(f)
    assert frame.describe(f[1], f[3:8]) == "状态 C 段2 脉冲5000"


def test_ack_frame_is_readable():
    f = frame.build_frame(
        frame.CMD_ACK, 5, bytes([frame.CMD_RESPRAY, frame.RESULT_ERR_STATE])
    )
    assert frame.describe(f[1], f[3:8]) == "ACK 补喷 状态不允许"


def test_heading_frame_decodes_sign_and_units():
    """航向帧：0.01 度 / 0.1 dps，两个都是有符号量。

    这一条专门盯符号。固件用 int16 传，上位机如果按无符号解析，
    车往左偏 12.34 度会显示成 643.02 度 —— 界面看起来像传感器坏了，
    实际只是少了一次补码转换。
    """
    deg100 = int(-12.34 * 100)          # -1234
    dps10 = int(-5.0 * 10)              # -50
    payload = bytes([
        (deg100 >> 8) & 0xFF, deg100 & 0xFF,
        (dps10 >> 8) & 0xFF, dps10 & 0xFF,
        (3 << 4) | 2,                   # 状态=闭环(3)，掉线 2 次
    ])
    f = frame.build_frame(frame.CMD_HEADING, 0, payload)
    assert frame.check_frame(f)

    rec = frame.make_frame_record(f)
    assert rec["cmd"] == frame.CMD_HEADING
    assert abs(rec["heading_deg"] - (-12.34)) < 1e-6
    assert abs(rec["rate_dps"] - (-5.0)) < 1e-6
    assert rec["gyro_state"] == 3
    assert rec["gyro_errors"] == 2

    # 正数也要能解，别只测了负数
    deg100 = 4500                       # +45.00 度
    payload = bytes([(deg100 >> 8) & 0xFF, deg100 & 0xFF, 0, 100, 0x30])
    rec = frame.make_frame_record(frame.build_frame(frame.CMD_HEADING, 0, payload))
    assert abs(rec["heading_deg"] - 45.0) < 1e-6
    assert abs(rec["rate_dps"] - 10.0) < 1e-6


def test_heading_frame_flows_through_stream_parser():
    """航向帧要和状态帧一样能从滚动缓冲里切出来，不能被当成噪声。"""
    payload = bytes([0x00, 0x64, 0xFF, 0xF6, 0x30])   # +1.00 度, -1.0 dps
                                                      # (-1.0dps x10 = -10 = 0xFFF6)
    buf = bytearray()
    records = []
    for _ in range(3):
        recs, _, _, _ = frame.parse_stream(buf, frame.build_frame(frame.CMD_HEADING, 0, payload))
        records.extend(recs)

    assert len(records) == 3
    assert all(r["cmd"] == frame.CMD_HEADING for r in records)
    assert abs(records[0]["heading_deg"] - 1.0) < 1e-6
    assert abs(records[0]["rate_dps"] - (-1.0)) < 1e-6


def test_gyro_state_codes_match_firmware_enum():
    """陀螺仪状态码要和固件 heading.h 的 HeadingState 枚举逐项对齐。

    这个码走 DATA[4] 的高 4 位，错了不会崩，只会让界面把一个
    "正在标零"的传感器显示成"闭环"—— 然后人就会奇怪为什么车不动。
    """
    text = (FIRMWARE_HW / "heading.h").read_bytes().decode("gbk", errors="replace")

    m = re.search(r"typedef\s+enum\s*\{(.*?)\}\s*HeadingState\s*;", text, re.S)
    assert m, "heading.h 里找不到 HeadingState 枚举"

    # 去掉注释后按逗号切，逐项求值：第一个显式 = 0，后面依次 +1
    body = re.sub(r"/\*.*?\*/", "", m.group(1), flags=re.S)
    items = [x.strip() for x in body.split(",") if x.strip()]

    values = {}
    nxt = 0
    for item in items:
        if "=" in item:
            name, val = item.split("=", 1)
            nxt = int(val.strip(), 0)
        else:
            name = item
        values[name.strip()] = nxt
        nxt += 1

    expected = {
        values["HDG_OFF"]: "未就绪",
        values["HDG_CALIBRATING"]: "标零中",
        values["HDG_READY"]: "就绪",
        values["HDG_CLOSED"]: "闭环",
        values["HDG_FAULT"]: "故障",
    }

    assert set(frame.GYRO_STATE_NAMES) == set(expected), (
        f"状态码集合不一致: 固件 {expected}, 上位机 {frame.GYRO_STATE_NAMES}"
    )
    assert frame.GYRO_STATE_NAMES == expected, (
        f"状态名不一致: 固件 {expected}, 上位机 {frame.GYRO_STATE_NAMES}"
    )


def test_gyro_measure_distance_matches_firmware():
    """轮距自标定的行程两边写了一遍，钉住。

    上位机用它估算"这一下大概会转多少度"，固件用它算轮距。
    两边不一致时，界面上那句提示会骗人，但不会崩 —— 正是最难发现的那类。
    """
    text = (FIRMWARE_HW / "system.c").read_bytes().decode("gbk", errors="replace")
    m = re.search(r"#define\s+GYRO_MEASURE_DIFF_MM\s+([0-9.]+)f", text)
    assert m, "system.c 里找不到 GYRO_MEASURE_DIFF_MM"
    assert int(float(m.group(1))) == frame.GYRO_MEASURE_DIFF_MM


def test_busy_result_code_is_reported():
    """陀螺仪闭环接管差速时，TRIM 的 ACK 必须能被读懂。

    回 BUSY 不是故障，是"这条车道归下位机了"。上位机看到它应该停止发
    TRIM，而不是以为"纠了没效果"然后加大增益 —— 那会让两个控制器打架。
    """
    f = frame.build_frame(
        frame.CMD_ACK, 7, bytes([frame.CMD_TRIM, frame.RESULT_ERR_BUSY])
    )
    assert frame.describe(f[1], f[3:8]) == "ACK 纠偏 陀螺仪闭环接管中"


def test_gyro_frame_roundtrip():
    """build_gyro 发的字节，固件按 DATA[0] 当子命令读。"""
    for sub, name in [
        (frame.GYRO_OFF, "关闭环"),
        (frame.GYRO_ON, "开闭环"),
        (frame.GYRO_RECAL, "重标零"),
        (frame.GYRO_MEASURE, "标轮距"),
    ]:
        f = frame.build_gyro(sub)
        assert frame.check_frame(f)
        assert f[1] == frame.CMD_GYRO
        assert f[3] == sub
        assert frame.describe(f[1], f[3:8]) == f"陀螺仪 {name}"


if __name__ == "__main__":
    for name in sorted(n for n in globals() if n.startswith("test_")):
        globals()[name]()
        print("ok", name)
    print("全部通过")
