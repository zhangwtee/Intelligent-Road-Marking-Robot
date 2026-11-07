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
FIRMWARE_BATTERY_H = FIRMWARE_HW / "battery.h"

FIRMWARE_COMMANDS = set(
    re.findall(r"case '([^']+)':", FIRMWARE_C.read_text(encoding="gbk"))
)


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
    现在没有任何白名单：面板上不许挂固件还没实现的按钮。以前 `N`（画车位
    （路沿））挂在这儿、靠一个 NOT_IMPLEMENTED 集合放行，结果是界面上有个
    按了没反应的按钮。既然功能不要了，白名单一起撤掉 —— 以后谁再挂一个
    空按钮，这条会直接拦住。
    """
    sent = {cmd[0] for _, cmd in PANEL_BUTTONS}
    known = {ord(ch) for ch in FIRMWARE_COMMANDS}
    unknown = sorted(chr(b) for b in sent - known)
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


def test_camera_buttons_exist():
    """弹摄像头的那些标签必须真的是面板上的按钮，写错就静默不弹。"""
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
        "PROTO_CMD_BATTERY": frame.CMD_BATTERY,
        "PROTO_BATT_FLAG_VALID": frame.BATT_FLAG_VALID,
        "PROTO_BATT_FLAG_LOW": frame.BATT_FLAG_LOW,
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
        "PROTO_CMD_SPEED": frame.CMD_SPEED,
        "PROTO_PAINT_FLAG_HEADING": frame.PAINT_FLAG_HEADING,
        "PROTO_PAINT_FLAG_RESUME": frame.PAINT_FLAG_RESUME,
        "PROTO_SPEED_MIN_ARR": frame.SPEED_MIN_ARR,
        "PROTO_SPEED_MAX_ARR": frame.SPEED_MAX_ARR,
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


def test_every_frame_the_host_sends_has_a_firmware_branch():
    """上位机发得出去的每一种帧，固件 switch 里都要有 case。

    单字节那条线有 test_panel_commands_reach_firmware 守着，帧这条线原来
    没人看：常量对得上（proto.h 有 PROTO_CMD_PAINT）不等于固件真的处理它。
    缺一个 case 的症状是回 ERR_TYPE、车不动 —— 和当年那个按了没反应的
    `N` 是同一类事。画车位按钮改走帧之后，这条更不能少。
    """
    text = FIRMWARE_PROTO_H.read_bytes().decode("gbk", errors="replace")
    defines = {
        m.group(1): int(m.group(2), 0)
        for m in re.finditer(
            r"#define\s+(PROTO_CMD_\w+)\s+(0x[0-9A-Fa-f]+|\d+)", text
        )
    }

    source = FIRMWARE_C.read_bytes().decode("gbk", errors="replace")
    branches = set(re.findall(r"case\s+(PROTO_CMD_\w+)\s*:", source))

    host_sent = {
        "RESPRAY": frame.CMD_RESPRAY,
        "MOVE": frame.CMD_MOVE,
        "TRIM": frame.CMD_TRIM,
        "PUMP": frame.CMD_PUMP,
        "GYRO": frame.CMD_GYRO,
        "PAINT": frame.CMD_PAINT,
        "LEVEL": frame.CMD_LEVEL,
        "SPEED": frame.CMD_SPEED,
        "PING": frame.CMD_PING,
    }

    broken = []
    for name, value in host_sent.items():
        macro = f"PROTO_CMD_{name}"
        if defines.get(macro) != value:
            broken.append(f"{macro}={defines.get(macro)} 与 frame.py 的 {value} 不符")
        elif macro not in branches:
            broken.append(f"{macro} 在 system.c 里没有 case")

    assert not broken, broken


def test_speed_range_is_the_same_in_three_places():
    """速度范围写在三处，必须一样。

        proto.h            PROTO_SPEED_MIN/MAX_ARR   收帧时判越界
        StepMotor_New.c    SPEED_FAST_ARR / SLOW_ARR 夹紧
        frame.py           SPEED_MIN_ARR / MAX_ARR   滑条范围
                         + SPEED_DEFAULT_ARR         上电默认

    不一致的症状都不会报错：上位机给得比固件大 -> 回 ERR_RANGE，表现为
    "滑条拖到底，车还是那么快"；给得比底层小 -> 被静默夹紧，界面上的数
    和车上跑的数就不一样了，而人会拿界面上那个数去算漆量。
    """
    proto_h = FIRMWARE_PROTO_H.read_bytes().decode("gbk", errors="replace")
    motor_c = (FIRMWARE_HW / "StepMotor_New.c").read_bytes().decode("gbk", errors="replace")

    def num(text, name, pattern=r"#define\s+%s\s+(\d+)"):
        m = re.search(pattern % name, text)
        assert m, f"找不到 {name}"
        return int(m.group(1))

    assert num(proto_h, "PROTO_SPEED_MIN_ARR") == frame.SPEED_MIN_ARR
    assert num(proto_h, "PROTO_SPEED_MAX_ARR") == frame.SPEED_MAX_ARR
    assert num(motor_c, "SPEED_FAST_ARR") == frame.SPEED_MIN_ARR
    assert num(motor_c, "SPEED_SLOW_ARR") == frame.SPEED_MAX_ARR
    assert num(motor_c, "SPEED_DEFAULT_ARR") == frame.SPEED_DEFAULT_ARR


def test_speed_frame_carries_the_arr():
    """速度帧：DATA[0:2] 大端 ARR，越小越快。

    发的是 ARR 不是档位号 —— 档位号要多一层"两边表一样"的假设，
    补喷长度就是这么从档位改回毫米的。
    """
    f = frame.build_speed(600, seq=4)
    assert frame.check_frame(f)
    assert f[1] == frame.CMD_SPEED
    assert (f[3] << 8) | f[4] == 600
    assert frame.describe(f[1], f[3:8]) == "速度 600"


def test_paint_frame_carries_the_heading_flag():
    """DATA[4] bit0 = 四条边用航向闭环。默认不带。"""
    plain = frame.build_paint(5000, 2300)
    guided = frame.build_paint(5000, 2300, heading=True)

    assert plain[7] == 0, "没要求纠偏却把标志置上了"
    assert guided[7] & frame.PAINT_FLAG_HEADING
    assert "纠偏" not in frame.describe(plain[1], plain[3:8])
    assert "纠偏" in frame.describe(guided[1], guided[3:8])


def test_status_flag_bits_match_firmware():
    """状态帧标志位的取值，上位机和固件必须一样。

    这些位没有协议常量可对账（固件那边是 SendStatusFrame 里直接 |= 的），
    所以只能扫源码。位写错不会报错：比如续画位写成 0x20，按钮就会跟着
    "陀螺仪读错过"一起亮 —— 而且只在陀螺仪出问题的时候亮，平时反而正常。
    """
    text = FIRMWARE_C.read_bytes().decode("gbk", errors="replace")

    # 固件写的是 flags |= 0xNN; 后面跟行内注释，逐条对
    pairs = {
        "STATUS_FLAG_RUNNING": 0x01,
        "STATUS_FLAG_SILENT": 0x02,
        "STATUS_FLAG_DROPPED": 0x04,
        "STATUS_FLAG_GYRO_READY": 0x08,
        "STATUS_FLAG_GYRO_CLOSED": 0x10,
        "STATUS_FLAG_GYRO_ERR": 0x20,
        "STATUS_FLAG_RESUME": 0x40,
        "STATUS_FLAG_BATTERY_LOW": 0x80,
    }
    missing = []
    for name, value in pairs.items():
        want = f"flags |= 0x{value:02X}"
        if want not in text:
            missing.append(f"{name}={want}")

    assert not missing, f"固件的 SendStatusFrame 里找不到这些位: {missing}"

    for name, value in pairs.items():
        assert getattr(frame, name) == value, name


def test_second_estop_does_not_wipe_the_breakpoint():
    """连按两下急停，第一下记的断点不能被第二下抹掉。

    `T` 的处理是"先记断点、再切 TING"。第二下进来时状态已经是 TING ——
    PaintCaptureResume 如果开头无条件清 paint_resume_valid，刚记好的断点
    就没了：状态帧的 0x40 灭、面板上"续画"变灰，而人只看到"按了两下急停，
    续画不见了"。急的时候连点两下是常态，这不是假想。

    真没有可续的情况（停在转弯上、剩不到 50mm）还是必须清，所以判断语句
    得排在清零前面，而不是简单把清零删掉。
    """
    text = FIRMWARE_C.read_bytes().decode("gbk", errors="replace")

    match = re.search(
        r"static void PaintCaptureResume\(void\)\s*\{(.*?)\n\}", text, re.S
    )
    assert match, "固件里找不到 PaintCaptureResume"
    body = match.group(1)

    assert re.search(r"paint_resume_valid\s*=\s*1", body), "抠到的不是断点那段代码"

    guard = body.find("sys_ctrl.state != STATE_PAINT")
    clear = body.find("paint_resume_valid = 0")

    assert guard != -1, "PaintCaptureResume 里没有状态判断"
    assert clear != -1, "PaintCaptureResume 里没有清零"
    assert clear > guard, (
        "paint_resume_valid = 0 排在了状态判断前面：连按两下急停会把断点抹掉"
    )


def test_speed_info_is_the_firmware_side_of_the_same_constant():
    """速度回传帧：上位机认的号、固件发的号，必须是同一个。

    这条帧以前不存在，所以"界面显示 300、车上跑 200"是常态。号对不上时
    不会报错 —— 上位机只是把这一帧当成"未知帧"写进日志，滑条一动不动，
    看起来和没加这个功能一样。
    """
    text = FIRMWARE_C.read_bytes().decode("gbk", errors="replace")
    head = FIRMWARE_PROTO_H.read_bytes().decode("gbk", errors="replace")

    assert f"PROTO_CMD_SPEED_INFO 0x{frame.CMD_SPEED_INFO:02X}" in head, "proto.h 里没有这个号"
    assert f"PROTO_SPEED_FLAG_SAVED  0x{frame.SPEED_FLAG_SAVED:02X}" in head, "标志位对不上"

    # 固件必须真的发它：光有常量、没有发送点，等于上位机永远收不到
    assert "SendFrame(SOURCE_PI, PROTO_CMD_SPEED_INFO, 0, d)" in text, "固件里没有发送点"


def test_speed_info_frame_is_readable():
    """速度帧能被翻译成人话，并且给出结构化字段。

    上位机不该为了拿一个整数自己去切 data[0:2] —— 那样每个调用点都要
    处理一遍字节序，迟早有一处写反。
    """
    f = frame.build_frame(
        frame.CMD_SPEED_INFO, 0,
        bytes([(600 >> 8) & 0xFF, 600 & 0xFF, frame.SPEED_FLAG_SAVED]),
    )
    assert frame.check_frame(f)

    record = frame.make_frame_record(f)
    assert record["speed_arr"] == 600, record
    assert record["speed_saved"] is True, record
    assert "600" in record["status"], record["status"]
    assert "已存" in record["status"], record["status"]

    unsaved = frame.build_frame(frame.CMD_SPEED_INFO, 0, bytes([0, 200, 0]))
    record = frame.make_frame_record(unsaved)
    assert record["speed_saved"] is False, record
    assert "待存" in record["status"], record["status"]


def test_speed_save_waits_until_the_car_is_standing_still():
    """存速度的那次擦除，必须等车停稳。

    擦一页期间取指是停的、中断排在后面 —— 正在发脉冲的
    那一两路会少几步，而开环步进丢的步补不回来。症状是"偶尔画线短一截"，
    没人会把它和"保存速度"联系起来。所以两个条件都得在：状态空闲、电机停着。
    """
    text = FIRMWARE_C.read_bytes().decode("gbk", errors="replace")

    match = re.search(r"static void SpeedSaveService\(void\)\s*\{(.*?)\n\}", text, re.S)
    assert match, "固件里找不到 SpeedSaveService"
    body = match.group(1)

    assert "StepMotor_IsAnyRunning()" in body, "存速度前没检查电机在不在转"
    assert "STATE_WAIT" in body and "STATE_TING" in body, "存速度前没检查状态"
    assert "SpeedSaveService();" in text, "主循环里没有调用，等于永远不保存"


def test_speed_save_backs_off_after_a_failed_write():
    """写 flash 失败之后不能每圈重试。

    擦一页期间 CPU 取指是停的。失败原因（页被写保护、片子来路不正）
    不会自己好，每圈重试一次等于让主循环一直被擦除拖住：串口收不进字节、
    状态帧跟着断，而现场只会看到"连不上了"，没人会想到是存速度引起的。
    所以重试之前必须有一次按时间计的退避。
    """
    text = FIRMWARE_C.read_bytes().decode("gbk", errors="replace")

    match = re.search(r"static void SpeedSaveService\(void\)\s*\{(.*?)\n\}", text, re.S)
    assert match, "固件里找不到 SpeedSaveService"
    body = match.group(1)

    guard = body.find("SPEED_SAVE_RETRY_MS")
    call = body.find("Cfg_SaveSpeed(arr)")
    assert guard != -1, "写失败后没有退避，会每圈擦一次页"
    assert call != -1, "抠到的不是写 flash 那一段"
    assert guard < call, "退避的判断排在写调用后面，等于没退避"


def test_flash_page_is_the_last_page_of_the_image():
    """保存速度用的那一页，必须落在 IROM 声明的范围里，而且是最后一页。

    这台机器的 IROM 是 0x08000000 + 0x80000（512K）。将来换成 C8 那种 128K
    的板子、或者把 IROM 改小，cfg.c 里写死的地址就会跑到器件外面 ——
    擦写直接失败，而失败的表现只是"速度存不住"，一点别的痕迹都没有。
    页大小同理：F103 大容量是 2KB，中容量是 1KB，用错了会连带擦掉前一页。
    """
    proj = FIRMWARE_HW.parent / "Project.uvprojx"
    xml = proj.read_text(encoding="utf-8", errors="replace")

    roms = re.findall(
        r"<IROM>\s*<Type>1</Type>\s*<StartAddress>(0x[0-9A-Fa-f]+)</StartAddress>"
        r"\s*<Size>(0x[0-9A-Fa-f]+)</Size>",
        xml,
    )
    assert roms, "工程里找不到 IROM 配置"
    assert len(set(roms)) == 1, f"两个 Target 的 IROM 不一样: {roms}"

    start, size = (int(v, 16) for v in roms[0])

    cfg = (FIRMWARE_HW / "cfg.c").read_text(encoding="gbk", errors="replace")
    page = int(re.search(r"#define\s+CFG_PAGE_SIZE\s+(\d+)u", cfg).group(1))
    image_end = re.search(
        r"#define\s+CFG_IMAGE_END\s+\(0x([0-9A-Fa-f]+)u\s*\+\s*(\d+)u\s*\*\s*(\d+)u\)",
        cfg,
    )
    assert image_end, "cfg.c 里 CFG_IMAGE_END 的写法变了，测试要跟着改"

    cfg_start = int(image_end.group(1), 16)
    cfg_end = cfg_start + int(image_end.group(2)) * int(image_end.group(3))

    assert page == 2048, f"F103 大容量的页是 2KB，cfg.c 写的是 {page}"
    assert cfg_start == start, f"cfg.c 的基址 0x{cfg_start:08X} != IROM 0x{start:08X}"
    assert cfg_end == start + size, (
        f"cfg.c 认为镜像到 0x{cfg_end:08X}，工程 IROM 到 0x{start + size:08X}"
    )


def test_paint_default_size_matches_firmware():
    """界面上预填的默认尺寸，得和固件里烧死的默认值一样。

    帧里带了尺寸时以帧为准，所以不一致不会报错 —— 只是"什么都不改、
    直接按画车位"画出来的大小和以前发单字节 'P' 不一样，而没人会想到
    去看这个。真要改默认尺寸就两边一起改。
    """
    text = FIRMWARE_C.read_bytes().decode("gbk", errors="replace")

    firmware_length = float(
        re.search(r"#define\s+PAINT_LEN_DEFAULT_MM\s+([\d.]+)f", text).group(1)
    )
    firmware_width = float(
        re.search(r"#define\s+PAINT_WID_DEFAULT_MM\s+([\d.]+)f", text).group(1)
    )

    assert firmware_length == frame.PAINT_LEN_DEFAULT_MM, firmware_length
    assert firmware_width == frame.PAINT_WID_DEFAULT_MM, firmware_width


def test_trim_limits_line_up_with_the_axis_they_belong_to():
    """纠偏上限有三层，各管一件事，不是一个数。

        proto.h        PROTO_TRIM_MAX_MM         上位机一帧能发多大差速
        lane_control   GAINS["MAX_TRIM_MM"]      视觉控制律的输出上限
        chassis.h      CHASSIS_TRIM_ACCUM_MAX_MM 这一段累计能纠多少（按车型）

    以前四处（加上 motion.c 和 heading.h）都写 20，看起来整齐，但那是
    “每次纠一点”时代的每步限幅。入参改成累计量以后，20mm 差速在 500mm
    轮距上只折合两度多 —— 5 度的偏差永远纠不回来，而且永远撞不到累计
    上限，Motion_Trim 的 return 2 成了死代码，_exhausted 再也不会置位。

    现在的关系只有两条，都是“不能比中间那个小”：
        协议层   <= 一帧能发的量   小了 -> 帧回 ERR_RANGE，车完全不纠
        累计上限 >= 一帧能发的量   小了 -> Motion_Trim 静默夹掉，ACK 还是 OK
    """
    from core import lane_control

    proto_h = FIRMWARE_PROTO_H.read_bytes().decode("gbk", errors="replace")
    motion_c = (FIRMWARE_HW / "motion.c").read_bytes().decode("gbk", errors="replace")
    motion_h = (FIRMWARE_HW / "motion.h").read_bytes().decode("gbk", errors="replace")
    heading_h = (FIRMWARE_HW / "heading.h").read_bytes().decode("gbk", errors="replace")
    chassis_h = (FIRMWARE_HW / "chassis.h").read_bytes().decode("gbk", errors="replace")

    firmware_proto = int(
        re.search(r"#define\s+PROTO_TRIM_MAX_MM\s+(\d+)", proto_h).group(1)
    )

    assert firmware_proto == frame.TRIM_MAX_MM, (
        f"proto.h 的 {firmware_proto} 与上位机的 {frame.TRIM_MAX_MM} 不一致"
    )
    assert lane_control.GAINS["MAX_TRIM_MM"] <= frame.TRIM_MAX_MM

    # 累计上限按车型分，每个分支都要查。只查第一个的话，改车型参数时漏掉
    # 另一支不会被发现，而那一支恰恰是这台车要跑的那一支。
    chassis_accum = [
        float(v) for v in
        re.findall(r"#define\s+CHASSIS_TRIM_ACCUM_MAX_MM\s+([\d.]+)f", chassis_h)
    ]

    assert len(chassis_accum) >= 2, (
        f"chassis.h 里应该有每个车型一份 CHASSIS_TRIM_ACCUM_MAX_MM，实际 {chassis_accum}"
    )
    assert all(v >= frame.TRIM_MAX_MM for v in chassis_accum), (
        f"累计上限 {chassis_accum} 比一帧能发的 {frame.TRIM_MAX_MM} 还小，"
        "Motion_Trim 会静默夹掉，ACK 还是 OK"
    )

    assert re.search(
        r"#define\s+MOTION_TRIM_ACCUM_MAX_MM\s+CHASSIS_TRIM_ACCUM_MAX_MM", motion_h
    ), "motion.h 的 MOTION_TRIM_ACCUM_MAX_MM 没有引用 chassis.h"

    assert "MOTION_TRIM_ACCUM_MAX_MM" in motion_c, "Motion_Trim 没按累计上限钳位"
    assert not re.search(r"#define\s+MOTION_TRIM_MAX_MM", motion_c), (
        "motion.c 里还留着一个 20mm 的 MOTION_TRIM_MAX_MM —— 那是增量语义的"
        "每步限幅，改成累计量之后会把这一段能纠的总量锁在两度多"
    )

    # 陀螺仪的单次输出也要能给出几何上真正需要的量（偏 5 度要 43mm）。
    assert re.search(
        r"#define\s+HDG_TRIM_ONCE_MAX_MM\s+CHASSIS_TRIM_ACCUM_MAX_MM", heading_h
    ), "heading.h 的 HDG_TRIM_ONCE_MAX_MM 还在写死一个数"


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


def test_battery_frame_roundtrip():
    """电池帧的字节和显示。

    mV 一直是整数，只有最后显示那一步才除 1000 —— 中途换浮点，日志里
    的数和界面上的数就会对不上（12.35 显示成 12.3 还是 12.4，两边四舍五入
    的时机不同）。
    """
    ok = frame.make_frame_record(
        frame.build_frame(frame.CMD_BATTERY, 0, b"\x30\xD4\x01")   # 12500mV
    )
    assert ok["batt_mv"] == 12500
    assert ok["batt_valid"] is True
    assert ok["batt_low"] is False
    assert ok["status"] == "电池 12.5V"

    low = frame.make_frame_record(
        frame.build_frame(frame.CMD_BATTERY, 0, b"\x2C\xEC\x03")   # 11500mV 低压
    )
    assert low["batt_low"] is True
    assert low["status"] == "电池 11.5V 低压"

    dead = frame.make_frame_record(
        frame.build_frame(frame.CMD_BATTERY, 0, b"\x00\x00\x00")
    )
    assert dead["batt_valid"] is False
    assert dead["batt_absent"] is False
    assert dead["status"] == "电池 未读到"

    # 分压电阻没焊：固件判出来"引脚上的不是电池"，单独一位。以前这两种
    # 情况都显示 `---`，人分不清是固件坏了还是少焊两个电阻。
    absent = frame.make_frame_record(
        frame.build_frame(frame.CMD_BATTERY, 0, b"\x00\x00\x04")
    )
    assert absent["batt_absent"] is True
    assert absent["batt_valid"] is False
    assert absent["batt_low"] is False
    assert absent["status"] == "电池 未接（无分压）"


def test_battery_divider_stays_within_adc_range():
    """分压比算错，读数不是"偏一点"，是整段被削平。

    ADC 满量程 3.3V。11:1 意味着 36.3V 才顶到天花板，12V / 24V 都有余量。
    改电阻忘了重算的症状是"电压卡在某个数不再上升"，看着像电池充不进电，
    实际是 ADC 早就饱和了。

    另一头是源阻抗：R_TOP//R_BOT 太大，采样保持电容在 239.5 个周期里充
    不满，读数会系统性偏低 —— 而且偏低得很有规律，容易被当成"电池真的虚"。
    """
    text = FIRMWARE_BATTERY_H.read_bytes().decode("gbk", errors="replace")
    top = int(re.search(r"#define\s+BATT_DIV_TOP_OHM\s+(\d+)", text).group(1))
    bot = int(re.search(r"#define\s+BATT_DIV_BOT_OHM\s+(\d+)", text).group(1))

    full_scale_mv = 3300 * (top + bot) // bot
    assert full_scale_mv >= 16000, f"满量程只有 {full_scale_mv}mV，12V 系统余量不够"
    assert full_scale_mv <= 60000, f"满量程 {full_scale_mv}mV 太高，12V 只剩几个 LSB"

    src = top * bot // (top + bot)
    assert src <= 20000, f"源阻抗 {src} 欧超过 20k，读数会偏低"


def test_battery_pin_does_not_collide():
    """采样脚必须是 PC0，而且不能和 M4 的 PC6 撞。

    这块板子上 GPIOC 只被 M4 的脉冲用过（PC6）。以后谁再往 GPIOC 加外设，
    先看这一条 —— 撞了不会报错，只会让两边都读到乱七八糟的数。
    """
    text = FIRMWARE_BATTERY_H.read_bytes().decode("gbk", errors="replace")
    assert re.search(r"#define\s+BATT_GPIO_PORT\s+GPIOC", text), "采样口不是 GPIOC"
    assert re.search(r"#define\s+BATT_GPIO_PIN\s+GPIO_Pin_0", text), "采样脚不是 PC0"
    assert re.search(r"#define\s+BATT_ADC_CHANNEL\s+ADC_Channel_10", text), \
        "PC0 对应的是 ADC_Channel_10"

    motor_h = (FIRMWARE_HW / "StepMotor_New.h").read_bytes().decode("gbk", errors="replace")
    assert re.search(r"#define\s+M4_PUL_PIN\s+GPIO_Pin_6", motor_h), \
        "M4 的脉冲脚变了，电池采样脚要重新挑（现在选 PC0 就是因为 PC6 被占了）"


def test_battery_round_is_dropped_when_a_sample_times_out():
    """一次没读到就整轮作废，不能拿 0 去凑平均。

    8 次平均里掺一个 0，读数会掉一大截 —— 12V 直接落到 10V 出头，正好
    跨过低电压门限，车上开始叫、界面变红，而电池是满的。误报比不报更坏：
    人会去换电池，换完还是叫，最后开始怀疑板子。
    """
    text = (FIRMWARE_HW / "battery.c").read_bytes().decode("gbk", errors="replace")

    assert "static int16_t Batt_ReadOnce" in text, "采样函数要给调用方一个超时信号"
    assert "return -1;" in text, "超时要能和真实的 0（线断了）分开"
    assert "sum += Batt_ReadOnce()" not in text, \
        "直接把采样值累加进去了，超时那次的 0 会被算进平均"

    # 超时分支里既要作废整轮，也要把这次转换停掉 —— 否则下一次启动会
    # 立刻看到残留的 EOC，读到的是上一次那个没等到结果的值。
    body = text[text.index("static int16_t Batt_ReadOnce"):]
    body = body[:body.index("\n}")]
    clear_at = body.find("ADC_ClearFlag(ADC1, ADC_FLAG_EOC)")
    start_at = body.find("ADC_RegularChannelConfig")
    assert 0 <= clear_at < start_at, "启动转换之前要先清 EOC，不然可能读到上一次的残留"


def test_low_battery_chirp_leaves_the_buzzer_to_the_state_machine():
    """低压提示音只在 WAIT 里响。

    蜂鸣器只有一个脚，状态机也在用（画完那声、自检）。要是这边按自己的
    计时去关，关掉的可能是状态机刚打开的那一声 —— 表现是"画完不响了"，
    而原因在一个跟画车位毫无关系的地方。
    """
    text = FIRMWARE_C.read_bytes().decode("gbk", errors="replace")
    i = text.index("/* ---- 低压提示音")
    body = text[i:i + 2000]

    assert "sys_ctrl.state != STATE_WAIT" in body, \
        "离开 WAIT 要把这一轮丢掉，不去关别人响着的那一声"
    assert "sys_ctrl.state == STATE_WAIT && Battery_IsLow()" in body, \
        "只在 WAIT 里起新的提示"


def test_battery_oled_code_survives_a_24v_system():
    """0.1V 一格的编号要能装下两位数的电压。

    用 uint8 存 mv/100，24V 系统充到 25.6V 就绕回来了，绕过的值可能和
    另一个电压撞上，结果是"电压变了但那一行不重画"。12V 撞不到，换车才暴露。
    """
    text = FIRMWARE_C.read_bytes().decode("gbk", errors="replace")
    assert re.search(r"static uint16_t batt_oled_code;", text), \
        "batt_oled_code 要用 16 位"


def test_no_two_peripherals_share_a_pin():
    """全板引脚不重样。

    两个外设配到同一个脚不会报错：两边都"初始化成功"，然后轮流改对方
    的电平。症状取决于谁最后写，看起来像其中一个坏了，实际是两个都还在。
    这种问题只有上电才看得见，所以在这里对一遍。

    泵、蜂鸣器、OLED、指示灯、串口这几路的脚是各写各的，没有集中的宏，
    所以下面写死，并且逐条回到源码里确认 —— 源码挪了脚而测试没跟，
    这条会先红，不会悄悄过期。
    """
    pins = {}

    def take(port, pin_no, who):
        key = (port, pin_no)
        assert key not in pins, \
            f"{who} 和 {pins[key]} 都在 {port} 的 Pin_{pin_no} 上"
        pins[key] = who

    def port_pin(text, prefix):
        port = re.search(rf"#define\s+{prefix}_PORT\s+(GPIO[A-G])", text)
        pin = re.search(rf"#define\s+{prefix}_PIN\s+GPIO_Pin_(\d+)", text)
        assert port and pin, f"{prefix} 的端口 / 脚号宏不全"
        return port.group(1), int(pin.group(1))

    def source(name, encoding="gbk"):
        return (FIRMWARE_HW / name).read_bytes().decode(encoding, errors="replace")

    # 电机八根
    motor_h = source("StepMotor_New.h")
    for i in range(1, 5):
        take(*port_pin(motor_h, f"M{i}_DIR"), f"M{i}_DIR")
        take(*port_pin(motor_h, f"M{i}_PUL"), f"M{i}_PUL")

    # 电池采样
    take(*port_pin(source("battery.h"), "BATT_GPIO"), "电池采样")

    # 陀螺仪
    mpu_h = source("mpu6050.h")
    take(*port_pin(mpu_h, "MPU_SCL"), "MPU6050 SCL")
    take(*port_pin(mpu_h, "MPU_SDA"), "MPU6050 SDA")

    # 硬编码的几路
    hardcoded = [
        ("PumpBuzzer.c", "#define PUMP_PIN    GPIO_Pin_13", ("GPIOB", 13), "水泵"),
        ("PumpBuzzer.c", "#define BUZZER_PIN  GPIO_Pin_14", ("GPIOB", 14), "蜂鸣器"),
        ("OLED.c", "GPIO_WriteBit(GPIOB, GPIO_Pin_8", ("GPIOB", 8), "OLED SCL"),
        ("OLED.c", "GPIO_WriteBit(GPIOB, GPIO_Pin_9", ("GPIOB", 9), "OLED SDA"),
        ("system.c", "GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;", ("GPIOB", 15), "心跳灯"),
        ("DPWM.c", "GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;", ("GPIOB", 6), "舵机 1"),
        ("DPWM.c", "GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;", ("GPIOB", 7), "舵机 2"),
    ]
    for fname, needle, key, who in hardcoded:
        assert needle in source(fname), \
            f"{fname} 里找不到 {needle!r}，这张引脚表要跟着改"
        take(*key, who)

    # 两路串口。USART.c 是 UTF-8，其余固件文件是 GBK。
    uart = source("USART.c", encoding="utf-8")
    for needle, key, who in (
        ("GPIO_Pin_9;                 // PA9", ("GPIOA", 9), "USART1_TX"),
        ("GPIO_Pin_10;                // PA10", ("GPIOA", 10), "USART1_RX"),
        ("GPIO_Pin_2;                 // PA2", ("GPIOA", 2), "USART2_TX"),
        ("GPIO_Pin_3;                 // PA3", ("GPIOA", 3), "USART2_RX"),
    ):
        assert needle in uart, f"USART.c 里找不到 {needle!r}，这张引脚表要跟着改"
        take(*key, who)

    # 上面一共 22 个脚。数不对说明有一条被漏掉了 —— 漏掉的那条正好可能是撞的。
    assert len(pins) == 22, f"表里只有 {len(pins)} 个脚，应为 22"


def _firmware_body(text, signature):
    """抠出一个函数的函数体。签名按正则给，函数体到行首的 } 为止。"""
    match = re.search(signature + r"\s*\{(.*?)\n\}", text, re.S)
    assert match, "固件里找不到 " + signature
    return match.group(1)


def test_breakpoint_survives_a_power_cut():
    """断点要能过掉电。

    以前它只活在 RAM 里：干完活关机、或者干脆电池掉了，这条边剩多少就没了，
    上电只能整圈重画 —— 前几条边白喷，而且人是在"车已经停在那儿"的位置上
    重画，框必然对不齐。

    存的是"还剩多少毫米"，不是位置。车要是被人推走了，续出来还是歪的 ——
    和不掉电时按续画按钮完全一样，靠人看着车判断，代码不假装知道。
    """
    cfg_c = (FIRMWARE_HW / "cfg.c").read_bytes().decode("gbk", errors="replace")
    cfg_h = (FIRMWARE_HW / "cfg.h").read_bytes().decode("gbk", errors="replace")
    text = FIRMWARE_C.read_bytes().decode("gbk", errors="replace")

    for name in ("Cfg_LoadResume", "Cfg_SaveResume", "Cfg_ClearResume"):
        assert name + "(" in cfg_c, f"cfg.c 里没有 {name}"

    # 记录里必须有左右两侧和这次的长宽：只存一侧，续画会把另一侧也按它走；
    # 不存长宽，续完这条边就不知道后面三条边该多长。
    for field in ("left_mm", "right_mm", "len_mm", "wid_mm"):
        assert field in cfg_h, f"CfgResume 里少了 {field}"

    boot = _firmware_body(text, r"static void PaintResumeBootLoad\(void\)")
    assert "Cfg_LoadResume" in boot, "上电时没有把断点读回来"
    assert "paint_resume_valid" in boot, "读回来没有点亮续画"

    # 读回来的长宽要还原：续完这条边还要转三次弯、走三条边。
    assert "paint_len_mm" in boot and "paint_wid_mm" in boot, \
        "断点没把长宽一起还原，后面三条边会用默认尺寸"

    # 上电路径里真的调了它，而不是写了个没人用的函数。
    assert "PaintResumeBootLoad();" in text, "Hardware_Init 里没调 PaintResumeBootLoad"


def test_breakpoint_on_a_turn_step_is_rejected():
    """段号是 PaintStep 的原值，只认"边"。

    PaintStep 是 IDLE、EDGE1、TURN1、EDGE2 …… 交替排下来的，边在奇数位。
    认了一个 TURN 当断点，续画会顺着转弯那一步直接往前开 —— 框歪掉，而且
    歪得特别像 CHASSIS_TURN_SCALE 没标定，人会去调错地方。
    """
    text = FIRMWARE_C.read_bytes().decode("gbk", errors="replace")

    # 断点只在边上记
    body = _firmware_body(text, r"static void PaintCaptureResume\(void\)")
    assert "PaintEdgeRunning()" in body, "断点没有限制在「边」上记"

    # 读回来的时候也要挡一道：flash 里的值可能是半截的、或者被别的东西写过。
    cfg_c = (FIRMWARE_HW / "cfg.c").read_bytes().decode("gbk", errors="replace")
    sane = _firmware_body(cfg_c, r"static uint8_t CfgResumeLooksSane\(const CfgResume \*rec\)")
    assert "rec->step & 1u" in sane, \
        "CfgResumeLooksSane 没有排除偶数段号（TURN1/TURN2/TURN3）"


def test_starting_a_new_frame_clears_the_stored_breakpoint_first():
    """新画一条之前，flash 里的旧断点要当场擦掉。

    service 只在车停着的时候写 flash。只置一个脏标记的话，整条画车位期间
    flash 里都留着上一份断点 —— 中途断电，上电之后那个旧断点会自己冒出来，
    按下去就是从"上一次停下的地方"再走一段，而车已经往前走了几条边。

    两个入口都是"车正停着"的时刻（WAIT 才让开），当场擦是安全的。
    """
    text = FIRMWARE_C.read_bytes().decode("gbk", errors="replace")

    head_frame = "paint_heading      = (d[4] & PROTO_PAINT_FLAG_HEADING) ? 1 : 0;"
    head_char = "paint_heading      = 0;"

    for head, who in ((head_frame, "帧版画车位"), (head_char, "单字节 P")):
        i = text.find(head)
        assert i != -1, f"{who}：找不到那条赋值"
        nxt = text[i + len(head):i + len(head) + 120]
        assert "PaintResumeClearNow" in nxt, (
            f"{who}：起步前没有当场擦断点，只置了脏标记 —— service 要等车停下才写，"
            f"整条画车位期间 flash 里都是旧断点"
        )

    # W（停止复位）仍然交给 service：那一下车可能还在跑，不能插一次擦除。
    forget = text.find("PaintResumeForget();")
    assert forget != -1, "W 那条路上没有作废断点"

    body = _firmware_body(text, r"static void PaintResumeClearNow\(void\)")
    assert "Cfg_ClearResume" in body, "当场擦的入口没有真去擦"
    assert "paint_resume_dirty = 0" in body, "擦成功之后还留着脏标记，service 会再跑一趟"


def test_clearing_the_breakpoint_updates_the_ram_copy():
    """擦掉之后，RAM 里那份"flash 里有什么"的副本也要跟着清。

    抄漏这一句，下一次判断会以为断点还在，把一次成功的擦除报成失败：
    上层于是每 10 秒重试一次，而每次只是读一页 —— 现象是"擦不掉"，
    实际早就擦掉了。
    """
    cfg_c = (FIRMWARE_HW / "cfg.c").read_bytes().decode("gbk", errors="replace")
    body = _firmware_body(cfg_c, r"uint8_t Cfg_ClearResume\(void\)")
    assert "CfgWriteResume(0)" in body, "Cfg_ClearResume 里没写 flash"

    # 落盘和更新副本都在 CfgWriteResume 里 —— 写断点和抹断点走同一条路,
    # 少一处"只在一半的分支里更新副本"的机会。
    inner = _firmware_body(cfg_c, r"static uint8_t CfgWriteResume\(const CfgResume \*rec\)")
    write = inner.find("CfgAppend")
    clear = inner.find("cfg_resume.valid = 0")
    assert write != -1, "CfgWriteResume 里没写 flash"
    assert clear != -1, "擦完没有更新 RAM 里的副本"
    assert clear > write, "先清副本再写 flash，写失败了副本就已经是错的"


def test_breakpoint_is_written_only_when_the_car_is_standing_still():
    """擦一页期间取指是停的。落在一个正在发脉冲的时刻，那一段就走少了。

    所以按下急停的那一刻只置脏标记，真正的写由 service 等车停稳再做 ——
    急停是"状态先切 TING、电机下一拍才停"，当场写必然撞上还在转的那几拍。
    """
    text = FIRMWARE_C.read_bytes().decode("gbk", errors="replace")

    capture = _firmware_body(text, r"static void PaintCaptureResume\(void\)")
    assert "paint_resume_dirty" in capture, "记断点时没有置脏标记"
    assert "Cfg_SaveResume" not in capture, \
        "在急停那一刻直接写 flash 了：电机可能还在转"

    service = _firmware_body(text, r"static void PaintResumeService\(void\)")
    stop = service.find("StepMotor_IsAnyRunning")
    write = service.find("PaintResumeFlush")
    assert stop != -1, "写断点之前没有查车停没停"
    assert write != -1, "PaintResumeService 里没有写 flash"
    assert stop < write, "先写 flash 再查车停没停，顺序反了"

    # 真正落盘那一步在 PaintResumeFlush 里。多一层是因为画车位每条边
    # 开头都要写一次（见 test_every_edge_start_writes_the_breakpoint），
    # 那些地方和这里用的是同一段代码，不能各抄一遍。
    flush = _firmware_body(text, r"static uint8_t PaintResumeFlush\(void\)")
    assert "Cfg_SaveResume" in flush, "写断点的入口没有真去写 flash"

    # 失败要退避，不然每一圈都重擦一次页，主循环一直被它按住。
    assert "PAINT_RESUME_RETRY_MS" in service, "写失败没有退避"

    # 主循环里真的调了这个 service。
    assert "PaintResumeService();" in text, "主循环里没调 PaintResumeService"


def test_resume_keeps_the_two_sides_apart():
    """续画不能拿一侧的剩余喂给四路。

    急停那一刻左右本来就不等长 —— 差速纠偏改的就是左右各自的目标脉冲。
    拿 MOTOR1 的剩余给四路统一用，等于把那笔还没走完的差速丢掉：车会先
    回正再接着走，续出来的那条边接不上前一段。
    """
    text = FIRMWARE_C.read_bytes().decode("gbk", errors="replace")

    capture = _firmware_body(text, r"static void PaintCaptureResume\(void\)")
    assert "StepMotor_GetRemainOf(MOTOR1)" in capture, "没取左侧剩余"
    assert "StepMotor_GetRemainOf(MOTOR2)" in capture, "没取右侧剩余"

    call = re.search(r"Motion_ForwardFlatLR\(([^;]*)\);", text)
    assert call, "续画没有走左右分开的那条路径"
    assert call.group(1).count(",") == 1, "Motion_ForwardFlatLR 只传了一个数"

    # 两侧的剩余都要清零，不然下一次续画会复用一个早就走完的数
    tail = text[call.end():call.end() + 400]
    assert "paint_resume_left_mm" in tail and "paint_resume_right_mm" in tail, \
        "用完没有清掉两侧的剩余"


def test_resume_clears_flash_before_it_starts_moving():
    """先抹 flash 再起步，顺序不能反。

    反过来就是：车已经在走了，主循环被擦除按住，那几十毫秒里串口收到的
    字节丢掉。而这一条断点还留在 flash 里 —— 这条边没走完之前断电，上电
    再续就会把那一段从头再走一遍。
    """
    text = FIRMWARE_C.read_bytes().decode("gbk", errors="replace")

    call = text.find("Motion_ForwardFlatLR(")
    assert call != -1, "找不到续画那段"

    window = text[max(0, call - 1200):call]
    clear = window.rfind("Cfg_ClearResume()")
    assert clear != -1, "续画之前没有抹掉 flash 里的断点"


def test_resume_does_not_apply_the_wheel_factor_twice():
    """续画的脉冲是"真剩多少"，不能再乘一次轮径系数。

    Motion_DriveLR 会按 k 再乘一次，而这两个数是从 StepMotor 的剩余脉冲
    反算回来的，已经含过 k 了。再乘一次，这一截就走成 k 倍 —— 轮径系数
    标到 1.02，五米多的边就要多走一截，而界面上一切正常。
    """
    motion = (FIRMWARE_HW / "motion.c").read_bytes().decode("gbk", errors="replace")
    body = _firmware_body(motion, r"void Motion_ForwardFlatLR\(float left_mm, float right_mm\)")

    assert "DriveAll" in body, "没有走按脉冲直接驱动的路径"
    assert "Motion_DriveLR" not in body, \
        "Motion_ForwardFlatLR 走了 DriveLR：轮径系数会被乘两次"


def test_breakpoint_needs_both_sides_to_have_something_left():
    """断点要求两侧都还剩得下东西。

    存进 flash 时两侧各自四舍五入成整数毫米。放过"一侧 0.3mm、另一侧
    3 米"这种组合，存下去那一侧就是 0，cfg 的合理性检查会整条拒掉：
    RAM 里断点还在、面板上按钮亮着，掉电之后什么都没留下，service 又
    每 10 秒白跑一次。所以这里是"任一侧不够就不记"，不是"两侧都不够
    才不记"。

    这种组合只在差速纠偏把某一侧目标往前挪了一大截、又恰好在收尾前
    按下急停时出现，窗口很窄 —— 但结果是"看着能续、实际续不了"。
    """
    text = FIRMWARE_C.read_bytes().decode("gbk", errors="replace")
    body = _firmware_body(text, r"static void PaintCaptureResume\(void\)")

    i = body.find("PAINT_RESUME_MIN_MM")
    assert i != -1, "找不到最小剩余量那段判断"
    cond = body[i:body.find("return;", i)]
    assert "return;" in body[i:], "最小剩余量那段判断里没有 return"

    assert "&&" not in cond, (
        "写的是「两侧都小于 50mm 才放弃」：一侧只剩几个脉冲时会把 0 毫米存进 flash，"
        "整条断点被 cfg 拒掉，而面板上按钮是亮的"
    )
    assert "||" in cond, "最小剩余量的判断既不是 || 也不是 &&"


def test_paint_watchdog_times_out_per_segment():
    """画车位的超时按"段"计，不按"整圈"计。

    最慢档下走完一整圈就可能超过 PAINT_TIMEOUT_MS —— 按整圈计时会把
    "慢但正常"判成卡住：泵关、车停，四条边只画了一半。

    按段计才叫"卡住"：同一个段号停留超过这个时间。超时退出之前还要按
    断点记一份，否则已经喷完的那几条边跟着白费。
    """
    # 不抠函数体：System_StateMachine 里有好几处行首的 }，那个辅助函数会在
    # 第一处就停下，抠不到看门狗这一段。这两处判断在全文里都是唯一的。
    text = FIRMWARE_C.read_bytes().decode("gbk", errors="replace")

    guard = re.search(
        r"if \(\(uint8_t\)paint_step != paint_step_seen\)\s*\{(.*?)\n\s*\}",
        text, re.S,
    )
    assert guard, "看门狗没有跟着段号走"
    assert "paint_progress_ms = current_time" in guard.group(1), \
        "段号变了没有重起计时，等于还在按整圈算"

    i = text.find("PAINT_TIMEOUT_MS)")
    assert i != -1, "找不到画车位超时判断"

    cond = text[max(0, i - 200):i]
    assert "paint_progress_ms" in cond, \
        "超时还在按进 STATE_PAINT 的时刻算：慢速画一整圈会被误判成卡住"
    assert "state_timestamp" not in cond, "超时还在用 state_timestamp"

    tail = text[i:i + 400]
    assert "PaintCaptureResume();" in tail, \
        "超时退出时没有记断点：已经喷完的几条边白费"
    assert tail.find("PaintCaptureResume();") < tail.find("STATE_WAIT"), \
        "先切状态再记断点 —— 切状态那一下电机就停了，剩余量读出来是空的"


def test_resume_info_frame_carries_the_dimensions():
    """断点的长宽要真的报给面板。

    固件上电已经把断点的长宽还原了（续画要用），但以前没有任何一帧带这个
    信息：面板上那两个输入框是它自己上次的值，人要核对“车上记的到底是
    多大”只能靠猜或者按一下续画看车动没动 —— 而那时漆已经在喷了。

    这一帧补的就是这个缺口。续画本身不受影响，所以它不是功能缺陷，
    是“一切正常但有一个数没人能证实”，最难被发现的那类。
    """
    rec = frame.make_frame_record(
        frame.build_frame(frame.CMD_RESUME_INFO, 0,
                          b"\x14\xB4\x09\xC4\x05")   # 5300x2500，第 3 条边
    )
    assert rec["resume_len_mm"] == 5300, rec
    assert rec["resume_wid_mm"] == 2500, rec
    assert rec["resume_step"] == 5, rec
    assert rec["resume_valid"] is True
    assert rec["status"] == "断点 5300x2500mm 第3条边", rec["status"]

    # 边号 0 才是“没有断点”。不能拿长宽是不是 0 去判 —— 一份 0x0 的记录
    # 也可能是坏帧，而“没有断点”有明确的编码。
    none = frame.make_frame_record(
        frame.build_frame(frame.CMD_RESUME_INFO, 0, b"\x00\x00\x00\x00\x00")
    )
    assert none["resume_valid"] is False
    assert none["status"] == "断点 无"


def test_every_edge_start_writes_the_breakpoint():
    """四条边的开头都要把断点写进 flash，不然断电就整圈白画。

    急停只有人按了才有，断电是随时都可能发生的。以前只有按 T 才留得下
    断点：画到第三条边的时候电池掉了，上电什么都没有，只能从第一条边
    重来，而车早就不在起点了。

    写点挑在“车刚停稳、泵还关着”的那一拍（上一条边走完、转弯、电机停了），
    所以那一次擦页既不会多走一步，也不会在漆线上留一个点。
    """
    text = FIRMWARE_C.read_bytes().decode("gbk", errors="replace")

    assert text.count("PaintResumeMarkEdge(paint_len_mm);") == 2, \
        "第 1、3 条边（长边）的开头没有记断点"
    assert text.count("PaintResumeMarkEdge(paint_wid_mm);") == 2, \
        "第 2、4 条边（宽边）的开头没有记断点"

    body = _firmware_body(text, r"static void PaintResumeMarkEdge\(float edge_mm\)")
    assert "StepMotor_IsAnyRunning" in body, \
        "车还在转也去擦页了：擦一页期间取指是停的，那一段会走少"
    assert "PaintResumeFlush" in body, "只记了 RAM，没有落 flash"

    # 画完一圈要把断点作废，否则下次上电会冒出“第 4 条边还剩 5.3 米”。
    done = text.find("Paint Done")
    assert done != -1
    tail = text[max(0, done - 900):done]
    assert "PaintResumeClearNow" in tail, \
        "画完之后没有清断点：下次上电会从画完的地方再走一遍"


def test_flash_keeps_two_copies_of_each_record():
    """速度和断点各两份，写新的那份时旧的那份原封不动。

    以前两个挤在同一页：写一次要先擦整页、再把两样都从头写回去。擦完还没
    写完的那一刻掉电，两样一起没了 —— 校验字只能判出“这条无效”，保留不了
    上一份。拆成四页之后最坏情况变成“这一次没存上”。
    """
    cfg = (FIRMWARE_HW / "cfg.c").read_bytes().decode("gbk", errors="replace")

    pages = {}
    for name in ("CFG_SPEED_PAGE0", "CFG_SPEED_PAGE1",
                 "CFG_RESUME_PAGE0", "CFG_RESUME_PAGE1"):
        m = re.search(
            r"#define\s+" + name + r"\s+\(CFG_IMAGE_END - (\d+)u \* CFG_PAGE_SIZE\)",
            cfg,
        )
        assert m, f"cfg.c 里没有 {name}"
        pages[name] = int(m.group(1))
    assert len(set(pages.values())) == 4, f"四页地址有重叠: {pages}"

    # 两种记录都要走页内追加：写的时候记着"上一次写到哪页"，页满才换。
    speed = _firmware_body(cfg, r"uint8_t Cfg_SaveSpeed\(uint16_t arr\)")
    assert "CfgAppend(&cfg_speed_page" in speed, \
        "速度没有走页内追加，改一次速就擦一页"
    resume = _firmware_body(cfg, r"static uint8_t CfgWriteResume\(const CfgResume \*rec\)")
    assert "CfgAppend(&cfg_resume_page" in resume, \
        "断点没有走页内追加，记一次就擦一页"

    # 读的时候要比序号，不是“有一份就用”。
    assert "(int16_t)(seq - cfg_speed_seq) > 0" in cfg, "速度两份没比序号"
    assert "(int16_t)(seq - cfg_resume_seq) > 0" in cfg, "断点两份没比序号"


def test_clearing_a_breakpoint_writes_a_tombstone():
    """抹断点是写一条“没有断点”的记录，不是把那一页擦空。

    两页交替写：最新那份在 A，B 里躺着更旧的一份。抹的时候只把 A 擦空的
    话，B 立刻又变成“最新” —— 断点自己复活了，而人刚刚按了 W。
    """
    cfg = (FIRMWARE_HW / "cfg.c").read_bytes().decode("gbk", errors="replace")

    body = _firmware_body(cfg, r"static uint8_t CfgWriteResume\(const CfgResume \*rec\)")
    assert "rec != 0" in body, "写函数没有区分“墓碑”和真记录"

    read = _firmware_body(cfg, r"static uint8_t CfgScanResume\(uint32_t page, uint16_t \*seq, CfgResume \*out\)")
    assert "rec.step != 0u" in read, "扫页时没有区分“墓碑”和真记录"
    assert "best_seq" in read and "(int16_t)(s - best_seq) > 0" in read, \
        "墓碑没参与“哪条最新”的比较，另一页那份断点会复活"


def test_battery_can_tell_a_missing_divider_from_a_flat_battery():
    """分压电阻没焊时不能显示一个像样的电压。

    PC0 悬空读回来的是一串乱跳的数。按“读到的就是电压”去显示，界面上会
    出现 8.3V 或者 21.6V —— 看着像一块快没电的电池，人端着万用表去量，
    电池是满的，最后开始怀疑板子。
    """
    h = FIRMWARE_BATTERY_H.read_bytes().decode("gbk", errors="replace")
    c = (FIRMWARE_HW / "battery.c").read_bytes().decode("gbk", errors="replace")

    for name in ("BATT_MIN_MV", "BATT_MAX_MV", "BATT_UNSTABLE_MV", "BATT_HISTORY"):
        assert re.search(r"#define\s+" + name, h), f"battery.h 里没有 {name}"

    body = _firmware_body(c, r"void Battery_Poll\(void\)")
    assert "mv < BATT_MIN_MV" in body and "mv > BATT_MAX_MV" in body, \
        "只按“读不到”判：读到一个离谱的值照样会显示成人话"
    assert "BATT_UNSTABLE_MV" in body, "没有按极差判悬空脚"

    # 前两条（读数落进电池不可能出现的区间）才是真的没接。
    assert body.count("batt_absent = 1") >= 2, \
        "“读不到”和“超量程”这两条应该标成没接"

    # 极差那条不能跟着标 absent、更不能清 valid：电机启停那一下真电池也会
    # 掉一两伏，一并判成“没接”就等于把低压保护关掉，而那正是最需要它的
    # 时刻（batt_valid=0 → IsLow() 恒 0 → 0x80 告警消失）。它只拒绝这一次读数。
    tail = body[body.find("BATT_UNSTABLE_MV"):]
    branch = tail[:tail.find("return;")]
    assert "batt_absent = 1" not in branch, \
        "极差判据把“带载波动”当成了“没接”，0x80 低压告警会跟着消失"
    assert "batt_valid = 0" not in branch, \
        "极差判据清了 valid，低压保护也一起关了"

    assert "Battery_IsAbsent" in c, "没有对外报“没接”的接口"


def test_resume_flag_only_lights_up_when_it_can_really_resume():
    """0x40 的含义是“现在按续画能走”，不是“flash 里有个断点”。

    现在每条边开头都写一份断点（断电保护），整个画车位期间都有。只看
    “有没有断点”的话，按钮在车正跑着的时候也亮着，按下去被回绝 ——
    比灰着更让人糊涂。
    """
    text = FIRMWARE_C.read_bytes().decode("gbk", errors="replace")
    i = text.find("flags |= 0x40;")
    assert i != -1, "状态帧里没有 0x40"

    cond = text[max(0, i - 900):i]
    assert "STATE_WAIT" in cond and "STATE_TING" in cond, \
        "0x40 没跟着“现在能不能真的续”走"


def test_breakpoint_write_does_not_erase_a_page():
    """记一条断点不能擦页 —— 一圈要写八九条。

    F103 擦一页期间取指是停的，而画车位在每条边开头、每条边走完都要记一次。
    如果每次写都擦一页，一万次的擦写寿命几圈之内就磨掉了。页内追加只编程
    五个字，一页能装一百多条，十几卷才轮到一次擦除。

    这条不是"优化"，是"每条边都记"能不能成立的前提：不改成追加，那个功能
    的代价就是 flash 寿命，而寿命耗尽的表现是"速度和断点都存不住了"，看着
    像颗坏片子。
    """
    cfg = (FIRMWARE_HW / "cfg.c").read_bytes().decode("gbk", errors="replace")
    body = _firmware_body(
        cfg,
        r"static uint8_t CfgAppend\(uint32_t \*page, uint8_t slot, uint8_t words_n,[^)]*\)",
    )

    assert "CFG_WORD_BLANK" in body, "追加时没有找空白格"
    assert "CfgProgramWords" in body, "没有走「只编程不擦除」的路径"

    # 擦除只允许出现在「这页满了」那条分支里。
    m = re.search(r"if \(addr \+ span > page_end\) \{(.*?)\n    \}", body, re.S)
    assert m, "CfgAppend 里找不到「这页满了」的判断"
    assert "CfgErasePage" in m.group(1), "擦页不在「这页满了」的分支里"

    outside = body[:m.start()] + body[m.end():]
    assert "CfgErasePage" not in outside, "除了页满，还有别的地方会擦页"


def test_walking_off_an_edge_voids_the_breakpoint():
    """一条边走完、进转弯之前要把断点作废。

    不作废的后果：转弯那几步里断电，上电之后断点还写着"这条边从起点开始"，
    按续画车就从当前位置、朝一个转了半截的方向走完 5.3 米 —— 漆喷到别处去，
    而现象很像转角系数标错了，人会去调错地方。

    转弯本身续不了（那是"少转了 90 度"），所以这里宁可让上电之后没有断点。
    """
    text = FIRMWARE_C.read_bytes().decode("gbk", errors="replace")

    for edge in ("PAINT_EDGE1", "PAINT_EDGE2", "PAINT_EDGE3"):
        i = text.find("case " + edge + ":")
        assert i != -1, f"找不到 {edge}"
        turn = text.find("Motion_TurnRight", i)
        assert turn != -1, f"{edge} 后面没有转弯"
        seg = text[i:turn]
        assert "PaintResumeClearNow" in seg, \
            f"{edge} 走完没有作废断点：转弯中断电会从边的起点错续一整条"


def test_status_frame_pulse_is_big_endian_on_the_firmware_side():
    """固件发脉冲必须高字节在前。

    全协议的多字节字段都是大端（速度、航向、电池、断点），只有这一处以前
    写成低字节在前。上位机按大端解，5000 脉冲会显示成 34835 —— 看着像
    编码器丢步，其实只是字节序反了。
    """
    src = FIRMWARE_C.read_text(encoding="gbk")

    assert "d[3] = (uint8_t)((pulse >> 8) & 0xFF);" in src, "脉冲高字节没放前面"
    assert "d[4] = (uint8_t)(pulse & 0xFF);" in src
    assert "d[3] = (uint8_t)(pulse & 0xFF);" not in src, "还是低字节在前"


def test_trim_is_a_total_not_an_increment():
    """Motion_Trim 收的是"这一段累计该纠到多少"，不是"再纠这么多"。

    两个调用方算出来的都是几何上的绝对量。当增量加的话，同一个期望值每发
    一次就累加一次：误差 2 度一次十几毫米，几百毫秒就撞上累计上限，这条边
    剩下的路一个脉冲都不纠 —— 而日志里每一拍都是 OK。
    """
    src = (FIRMWARE_HW / "motion.c").read_text(encoding="gbk")

    start = src.find("uint8_t Motion_Trim")
    assert start != -1
    body = src[start:src.find("\n}", start)]

    assert "step = want - s_trim_accum;" in body, "没有按差值落地"
    assert "StepMotor_AdjustTarget(MOTOR1, +step)" in body
    assert "s_trim_accum + " not in body, "还在把这一次的量加到上一次上"


def test_stopping_the_tracking_loop_returns_the_firmware_to_wait():
    """撤相机发的必须是 'W'。

    'T' 停在 TING，而新画一条车位只认 WAIT：撤完相机紧接着发画车位会被回
    ERR_STATE，用户得按第二次才知道发生了什么事。
    """
    src = (Path(__file__).resolve().parent / "ui" / "camera_widget.py").read_text(
        encoding="utf-8"
    )

    start = src.find("def stop_track")
    assert start != -1
    body = src[start:src.find("\n    def ", start + 10)]

    assert 'send(b"W")' in body, "停车发的不是 'W'，撤完相机回不到 WAIT"
    assert 'send(b"T")' not in body, "还在发 'T'，状态会停在 TING"
    assert "self.pump_on = False" in body, "pump_on 没清，下次巡线头几帧不喷"


def test_every_downstream_frame_carries_a_seq():
    """主窗口发帧必须带上自己递增的序号。

    固件按 (seq, cmd) 去重，不看内容。序号恒为 0 的话，同一类命令 1 秒内
    发第二次会被当成重传 —— 命令根本没执行，回执还是上一次的 OK。
    """
    src = (Path(__file__).resolve().parent / "ui" / "main_window.py").read_text(
        encoding="utf-8"
    )

    for call in ("build_gyro(sub,", "build_gyro(frame.GYRO_RECAL,",
                 "build_gyro(frame.GYRO_MEASURE,", "build_speed(arr,"):
        assert call in src, f"{call} 没带 seq"

    assert "self._next_seq()" in src
    assert "build_paint(length, width, heading=heading," in src


def test_jogging_voids_the_stale_breakpoint():
    """点动、自检之后断点必须作废。

    这些命令会让车离开断点记下的那个位置（点动几百毫米，'D' 自检 400mm）。
    不作废的话，急停 -> 点动挪车 -> 按续画，车会从偏了的位置把整条边重走
    一遍，泵还开着。地上的表现是"歪了一条边"，很像转角系数标错。
    """
    src = FIRMWARE_C.read_bytes().decode("gbk", errors="replace")

    guard = src.find("&& paint_resume_valid) {")
    assert guard != -1, "点动 / 自检没有任何作废断点的动作"

    switch = src.find("switch(cmd) {")
    assert 0 < guard < switch, "作废断点那段应该在 switch 之前，否则点动还会先动起来"

    cond = src[max(0, guard - 320):guard]
    for key in ("'U'", "'R'", "'Y'", "'Z'", "'A'", "'X'", "'D'"):
        assert key in cond, f"{key} 不在作废断点的条件里"

    seg = src[guard:guard + 200]
    assert "PaintResumeForget()" in seg
    assert "PaintResumeShowOnOled()" in seg, "清了断点但 OLED 第 4 行还挂着 Resume"


def test_w_turns_the_pump_off():
    """'W' 必须自己关泵。

    CMD_PUMP 能在 WAIT 下把泵打开而不改状态。之后按 'W'，last_state 已经是
    WAIT，状态退出块不跑，泵就一直转着 —— 人以为按了停止，地上还在出漆。
    """
    src = FIRMWARE_C.read_bytes().decode("gbk", errors="replace")

    i = src.find("case 'W':")
    assert i != -1
    seg = src[i:i + 700]
    assert "Pump_Off();" in seg, "'W' 没关泵"


def test_respray_gives_up_when_the_pulses_stop_not_when_the_clock_runs_out():
    """补喷的超时按"脉冲还在不在动"判，不按固定总时长。

    慢档 133mm/s 走 3000mm 要 22 秒，固定 10 秒总时限会在 1.3 米处把一次
    正常的补喷掐断，回 WAIT 且不给错误码。按脉冲判与速度无关。
    """
    src = FIRMWARE_C.read_bytes().decode("gbk", errors="replace")

    assert "RESPRAY_TIMEOUT_MS" not in src, "还留着固定的总时长"
    assert "RESPRAY_STALL_MS" in src

    i = src.find("respray_last_remain = remain")
    assert i != -1, "没按剩余脉冲判卡死"

    seg = src[max(0, i - 900):i + 300]
    assert "StepMotor_GetRemainOf(MOTOR1)" in seg, "没按剩余脉冲判卡死"
    assert "RESPRAY_STALL_MS" in seg, "没有卡死判据的时间窗口"


def test_turning_derives_direction_from_the_forward_table():
    """原地转的方向不能写死 DIR 电平。

    四个轮子装在车两侧、镜像接线，"物理向前"对应的电平本来就左右相反
    （M1_FWD 是 CCW、M2_FWD 是 CW）。写死一个电平等于假设四个驱动器极性
    一致 —— 那样左后、右后两个轮子会反着转，车不是转弯，是两两对拧。
    """
    src = (FIRMWARE_HW / "motion.c").read_bytes().decode("gbk", errors="replace")
    body = _firmware_body(src, r"void Motion_TurnRight\(float deg\)")

    assert "DIR_CW" not in body and "DIR_CCW" not in body, \
        "转弯还在写死 DIR 电平"

    for name in ("M1_REV", "M2_FWD", "M3_REV", "M4_FWD"):
        assert name in body, f"转弯没用 {name} 派生方向 —— 左反右正就对不上"


def test_mecanum_turn_restores_the_speed():
    """麦轮转弯也要把左右 ARR 恢复成同一档。

    上一段可能是 DriveLR 设成左右不同 ARR 的（差速纠偏改完目标会按剩余
    脉冲重算）。不恢复的话，转弯这四个轮子同脉冲不同频：先到的一侧停住，
    另一侧继续拧。
    """
    src = (FIRMWARE_HW / "motion.c").read_bytes().decode("gbk", errors="replace")
    body = _firmware_body(src, r"void Motion_TurnRight\(float deg\)")

    i = body.find("CHASSIS_TURN_ARR")
    assert i != -1
    seg = body[i:i + 900]
    assert "StepMotor_RestoreSpeed()" in seg, "麦轮转弯没恢复同速"


def test_power_up_does_not_step_the_wheels():
    """上电不能给四路下发脉冲。

    SetPulse 是真的会发脉冲，不是"把通道置成确定状态"。原来那句
    SetPulse(MOTORx, 1) 等于每轮上电四个轮子各走一步。
    """
    src = FIRMWARE_C.read_bytes().decode("gbk", errors="replace")

    i = src.find("void Hardware_Init")
    assert i != -1
    i = src.find("{", i)
    seg = src[i:i + 2200]

    assert "StepMotor_SetPulse" not in seg, "上电还在下发脉冲，每轮上电各走一步"
    assert "StepMotor_StopAll()" in seg, "四路的目标没有清成确定状态"


def test_gyro_bias_uses_the_same_sign_as_the_runtime():
    """标定算出的零偏必须和运行时用同一个符号。

    标定时累加的是传感器原始读数，运行时先乘 HDG_Z_SIGN 再减零偏。两边不
    统一的话静止时 rate = -2*bias：零偏 1dps 就是每秒 2 度漂移，5.3 米一段
    走 24 秒差 48 度；而且静止判据恒等于 2|bias|，超过 0.6dps 就再也进不了
    "静止"分支，零偏连自愈的路都没有。
    """
    src = (FIRMWARE_HW / "heading.c").read_bytes().decode("gbk", errors="replace")

    i = src.find("hdg_bias_dps = ((float)cal_sum")
    assert i != -1, "找不到零偏的计算"

    seg = src[i:i + 240]
    assert "HDG_Z_SIGN" in seg, "标定没乘 HDG_Z_SIGN"


def test_truncated_frame_tail_is_not_executed():
    """残帧超时清掉之后，紧接着到的字节不能当命令执行。

    串口把一次发送切成两段、中间卡 250ms 以上时，后一段就是半帧的尾巴。
    逐字节执行等于把 payload 重放一遍 —— 里面随便一个 'T' 就是急停，
    一个 'C' 就是开泵补喷。
    """
    from core import frame

    buf = bytearray()

    # 帧头 + 两个 payload 字节，然后断流
    recs, last_rx, dropped, resync = frame.parse_stream(
        buf, bytes([frame.SOF, 0x01, 0x02]), last_rx=0.0, now=0.0)
    assert recs == []

    # 尾巴到了：这一批必须整批丢掉
    recs, last_rx, dropped, resync = frame.parse_stream(
        buf, b"T", last_rx=last_rx, now=0.4)
    assert recs == [], f"残帧的尾巴被当成命令执行了: {recs}"
    assert buf == bytearray()

    # 之后来一条正常命令，必须照样收得到
    recs, last_rx, dropped, resync = frame.parse_stream(
        buf, b"W", last_rx=last_rx, now=0.45)
    assert [r["char"] for r in recs] == ["W"], "丢尾巴把后面的正常命令也堵住了"


if __name__ == "__main__":
    for name in sorted(n for n in globals() if n.startswith("test_")):
        globals()[name]()
        print("ok", name)
    print("全部通过")
