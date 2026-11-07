"""帧协议：上下位机共用的字节级定义。

为什么要有这个文件
-----------------
改造前「一个字节就是一条命令」，好处是调试时敲一个字母车就动，坏处有两个，
而且都是安全问题：

  1. 线上任何噪声都是合法指令。9600 波特率下一个毛刺落在 'P' 上，
     车就开始自动画 5.3 米并喷漆。
  2. 带不了参数。补喷长度只能拆成「先发 '3' 设档、再发 'C' 开喷」两条，
     两条之间被打断就变成「档位变了、车没动」，而且完全静默。

所以加一层帧：帧头和 CRC 把「合法字节」从「任何字节」收缩成「校验通过的
10 个字节」，payload 让一条命令能带两个参数字节。

为什么是定长 10 字节，不是变长
-----------------------------
变长帧要在解析器里多做一次判断（先看 LEN 合不合法，再等够长度），而 LEN 被
破坏时解析器会安静地等一个永远不来的长度。定长帧一次判断就够，且本项目的
参数最多 2 字节，变长带来的复杂度换不到任何好处。

帧结构
------
    0    0x7B  '{'   帧头
    1    CMD         命令/类型
    2    SEQ         序号，ACK 原样回显；也是重发去重的依据
    3-7  DATA[5]     参数，多字节一律大端
    8    CRC8        覆盖第 0~7 字节（poly 0x07，初值 0，不反射）
    9    0x7D  '}'   帧尾

0x7B / 0x7D 恰好是 '{' 和 '}'，而所有单字节命令都是可打印 ASCII 里的字母和
数字，天然不冲突 —— 这是「单字节老路径」和「帧」能共用一条串口的前提。

CRC8 而不是求和他们
------------------
逐字节求和校验对「两个字节互换」「一处出错另一处抵消」完全无效。CRC8 多
20 行代码，能挡住这两类错误。poly 0x07 是 CRC-8/ATM 的标准参数，标准校验值
CRC8(b"123456789") == 0xF4，两边都用这个值自检（见 protocol_test.py）。

为什么不带长度也不带转义
----------------------
帧里只有 0x7B/0x7D 两个边界字节和固定长度，不需要字节填充（escaping）。
payload 里出现 0x7B/0x7D 也没关系 —— 解析器只按固定偏移找帧尾。
"""

import time

# ---------- 帧结构 ----------
SOF = 0x7B          # '{'
EOF = 0x7D          # '}'
FRAME_LEN = 10
DATA_LEN = 5

# ---------- 下行命令（上位机 -> STM32） ----------
CMD_RESPRAY = 0x01      # DATA[0:2] = 补喷长度 mm（大端 uint16）
CMD_MOVE = 0x02         # DATA[0]=方向，DATA[1:3]=左轮 mm，DATA[3:5]=右轮 mm
CMD_LEVEL = 0x03        # DATA[0] = 档位 1~5，兼容旧的 '1'~'5'
CMD_TRIM = 0x04         # DATA[0:1] = int16 毫米，>0 向右修正，只在移动中有效
CMD_PUMP = 0x05         # DATA[0] = 0 关 / 1 开
CMD_PAINT = 0x06        # DATA[0:1]=长 mm，DATA[2:3]=宽 mm，0=默认 5300/2500
                        # DATA[4]=标志，bit0 = 四条边走陀螺仪航向闭环
CMD_GYRO = 0x07         # DATA[0]=子命令，陀螺仪航向闭环
CMD_SPEED = 0x08        # DATA[0:1]=速度 ARR（大端），越小越快，150~1500
CMD_PING = 0x09         # 无参数，只回 ACK

# CMD_PAINT 的 DATA[4] 标志
PAINT_FLAG_HEADING = 0x01   # 四条边用航向闭环。陀螺仪没标定时固件回 ERR_HW
PAINT_FLAG_RESUME = 0x02    # 从急停断点接着画。没有断点时固件回 ERR_STATE

# 画车位子状态（固件的 PaintStep 枚举，顺序一一对应）。
# 状态帧 DATA[1] 报的就是这个数，上位机据此显示"第几条边"。
# 以前这个字段一直没人用，人在树莓派前只能看车上那块 OLED。
PAINT_STEP_LABELS = {
    0: "空闲",
    1: "第1条边", 2: "转弯1",
    3: "第2条边", 4: "转弯2",
    5: "第3条边", 6: "转弯3",
    7: "第4条边", 8: "画完",
}

# CMD_GYRO 的子命令
GYRO_OFF = 0x00         # 关闭环，直线交回视觉/开环
GYRO_ON = 0x01          # 开闭环（未标零时固件回 ERR_STATE）
GYRO_RECAL = 0x02       # 重新标零，车必须停着
GYRO_MEASURE = 0x03     # 轮距自标定：固件单侧走 300mm 再反算，会在第 4 行显示结果

# ---------- 上行（STM32 -> 上位机） ----------
CMD_ACK = 0x81          # DATA[0]=原命令 DATA[1]=结果码
CMD_STATUS = 0x82       # DATA[0]=状态 DATA[1]=段号 DATA[2]=标志 DATA[3:5]=脉冲
CMD_HEADING = 0x83      # DATA[0:1]=int16 航向 0.01°，DATA[2:3]=int16 角速度 0.1dps
                        # DATA[4] 高 4 位=陀螺仪状态，低 4 位=掉线次数
CMD_SPEED_INFO = 0x84   # DATA[0:1]=固件当前档位 ARR 大端，DATA[2]=标志
                        # 1Hz 兜底、值一变立刻补发。以前速度只有下行：
                        # 手机上按 +/- 改完，这边滑条还停在老值。
CMD_BATTERY = 0x85      # DATA[0:1]=电池电压 mV 大端，DATA[2]=标志
                        # 1Hz 一帧。以前车上还剩多少电只有那块 OLED 知道，
                        # 人在树莓派前跑到一半断电，事后再回想是"电池没电"
                        # 还是"车停了"根本分不清。
CMD_RESUME_INFO = 0x86  # DATA[0:1]=断点的长 mm，DATA[2:3]=宽 mm
                        # DATA[4]=边号（PaintStep 原值），0=没有断点
                        # 固件上电已经把断点的长宽还原了，但以前没有任何一帧
                        # 告诉面板 —— 输入框还是它自己上次的值，人要核对
                        # "车上记的到底是 5.3x2.5 还是 5.0x2.3"没地方看。
                        # 续画本身不受影响（固件只认断点），所以这是可观测性
                        # 缺口，不是功能缺陷。

# STATUS 帧 DATA[2] 的标志位，和固件 SendStatusFrame 里那几行 |= 一一对应。
# 以前是散落在 UI 里的魔法数字，加一位就要在两边各改一处，改漏了不报错。
STATUS_FLAG_RUNNING = 0x01      # 电机正在输出脉冲
STATUS_FLAG_SILENT = 0x02       # 还在上电静默期（只放行急停）
STATUS_FLAG_DROPPED = 0x04      # 串口丢过字节
STATUS_FLAG_GYRO_READY = 0x08   # 陀螺仪在线且已标零
STATUS_FLAG_GYRO_CLOSED = 0x10  # 航向闭环正在跑
STATUS_FLAG_GYRO_ERR = 0x20     # 这一局读过 I2C 错误（精度已打折扣）
STATUS_FLAG_RESUME = 0x40       # 画车位有急停断点，可以续画
STATUS_FLAG_BATTERY_LOW = 0x80  # 电池低于门限。分压线断了不算 —— 采样无效时
                                # 固件不置这一位，界面显示 --- 而不是报警

# SPEED_INFO 帧 DATA[2] 的标志位
SPEED_FLAG_SAVED = 0x01     # 已写进固件 flash，断电不丢

# BATTERY 帧 DATA[2] 的标志位。门限值不在这一侧 —— 只认固件给的 LOW 位。
# 两边各写一遍门限，改一处忘另一处就会出现"车上在响、界面还是绿的"，
# 而且这种不一致只在真没电那天才暴露。
BATT_FLAG_VALID = 0x01      # 采样有效，电压可信
BATT_FLAG_LOW = 0x02        # 低于固件 battery.h 的门限
BATT_FLAG_ABSENT = 0x04     # 引脚上根本没有电池（分压没接 / 线掉了）。
                            # 和"采样无效"分开：那个是这一轮没读到，下一轮
                            # 可能就好了；这个是得有人去接那两颗电阻。

# 陀螺仪状态码，和固件 heading.h 的 HeadingState 一一对应
GYRO_STATE_NAMES = {
    0: "未就绪",     # 没接上 / 初始化失败
    1: "标零中",     # 正在采零偏，车必须静止
    2: "就绪",       # 标好了，闭环关着
    3: "闭环",       # 标好了，正在纠偏
    4: "故障",       # 跑着跑着掉线
}

# ---------- ACK 结果码 ----------
RESULT_OK = 0x00
RESULT_ERR_STATE = 0x01     # 当前状态不接受这条命令（比如正在画车位时补喷）
RESULT_ERR_RANGE = 0x02     # 参数越界
RESULT_ERR_TYPE = 0x03      # 不认识的命令类型
RESULT_ERR_HW = 0x04        # 硬件不支持（普通轮没有横移）
RESULT_ERR_BUSY = 0x05      # 差速被执行器占用：陀螺仪闭环开着时收到 TRIM

RESULT_NAMES = {
    RESULT_OK: "OK",
    RESULT_ERR_STATE: "状态不允许",
    RESULT_ERR_RANGE: "参数越界",
    RESULT_ERR_TYPE: "未知命令",
    RESULT_ERR_HW: "硬件不支持",
    RESULT_ERR_BUSY: "陀螺仪闭环接管中",
}

# ---------- MOVE 方向 ----------
MOVE_FWD = 0
MOVE_BACK = 1
MOVE_LEFT = 2
MOVE_RIGHT = 3

MOVE_NAMES = {
    MOVE_FWD: "前进",
    MOVE_BACK: "后退",
    MOVE_LEFT: "左移",
    MOVE_RIGHT: "右移",
}

CMD_NAMES = {
    CMD_RESPRAY: "补喷",
    CMD_MOVE: "移动",
    CMD_LEVEL: "设档",
    CMD_TRIM: "纠偏",
    CMD_PUMP: "水泵",
    CMD_PAINT: "画车位",
    CMD_GYRO: "陀螺仪",
    CMD_SPEED: "速度",
    CMD_BATTERY: "电池",
    CMD_PING: "PING",
    CMD_ACK: "ACK",
    CMD_STATUS: "状态",
    CMD_HEADING: "航向",
    CMD_SPEED_INFO: "当前速度",
    CMD_RESUME_INFO: "断点",
}

# 参数范围（和固件 proto.h 里的 PROTO_* 一一对应）
RESPRAY_MIN_MM = 50
RESPRAY_MAX_MM = 3000
MOVE_MAX_MM = 5000
TRIM_MAX_MM = 20        # 必须等于 proto.h 的 PROTO_TRIM_MAX_MM
PAINT_MIN_MM = 1000
PAINT_MAX_MM = 8000

# 速度参数就是固件写进定时器那个 ARR（计数时钟 1MHz，ARR 即微秒数）：
# 越小越快。范围取固件 StepMotor_New.c 的 SPEED_FAST_ARR / SPEED_SLOW_ARR，
# protocol_test 会去对账 —— 对不上时界面上显示的档位就是假的。
SPEED_MIN_ARR = 150         # 最快：6.7kHz 脉冲
SPEED_MAX_ARR = 1500        # 最慢：667Hz 脉冲
SPEED_DEFAULT_ARR = 300     # 上电默认，和固件 SPEED_DEFAULT_ARR 一致

# 画车位的默认尺寸，和固件 system.c 的 PAINT_*_DEFAULT_MM 一致
PAINT_LEN_DEFAULT_MM = 5300
PAINT_WID_DEFAULT_MM = 2500

# 轮距自标定时固件会单侧走这么远（system.c 的 GYRO_MEASURE_DIFF_MM）。
# 上位机只用它来估算"大概要转多少度"，不参与计算 —— 真正的反算在固件里。
GYRO_MEASURE_DIFF_MM = 300

# 半帧超时。一帧传到一半链路断了（USB 抖动、STM32 复位），残帧会永远躺在
# 缓冲里等后续字节接上来，然后 CRC 一直错 —— 而且错得很有规律，看起来像
# 「协议不兼容」。超过这个时间就当作残帧丢掉。
PARTIAL_TIMEOUT_S = 0.25

# 坏帧之后的重同步静默期。
#
# 只丢 1 字节重新扫帧头是必须的（丢整帧会把紧随其后的真帧一起吞掉），但光这样
# 还不够：坏帧剩下的 9 个字节会被当成 9 条单字节命令依次执行，其中可能正好有
# 'P'（0x50）—— 于是「CRC 把坏帧拦下来了」变成「拦下来之后又逐字节执行了一遍」，
# CRC 等于白加。
#
# 所以坏帧之后进入静默：在重新找到合法帧之前，裸字节一律丢弃不执行。
# 一段数据流结束（空闲超过这个时间）就退出静默，正常单字节命令照旧。
RESYNC_IDLE_S = 0.02


def crc8(data: bytes, crc: int = 0) -> int:
    """CRC-8/ATM：poly 0x07，初值 0，不反射，不异或输出。

    和固件 proto.c 的 Proto_Crc8 必须逐位一致。标准校验值：
    crc8(b"123456789") == 0xF4。
    """
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


def build_frame(cmd: int, seq: int = 0, data: bytes = b"") -> bytes:
    """组一帧。data 不足 5 字节补 0，超了直接报错（不静默截断）。"""
    if len(data) > DATA_LEN:
        raise ValueError(f"payload 最多 {DATA_LEN} 字节，收到 {len(data)}")
    body = bytes([SOF, cmd & 0xFF, seq & 0xFF]) + data.ljust(DATA_LEN, b"\x00")
    return body + bytes([crc8(body), EOF])


def check_frame(frame: bytes) -> bool:
    """整帧校验：帧头、帧尾、CRC 三者都对才算数。"""
    if len(frame) != FRAME_LEN:
        return False
    if frame[0] != SOF or frame[FRAME_LEN - 1] != EOF:
        return False
    return crc8(frame[:FRAME_LEN - 2]) == frame[FRAME_LEN - 2]


def _u16(value: int) -> bytes:
    value = max(0, min(0xFFFF, int(value)))
    return bytes([value >> 8, value & 0xFF])


def _s16(hi: int, lo: int) -> int:
    """两字节大端 -> 有符号 16 位。

    航向和角速度都可以是负的（左偏、逆时针），而 Python 的 int 是无符号的
    位运算，不补这一步 -0.01° 会变成 655.35°。
    """
    value = (hi << 8) | lo
    return value - 0x10000 if value >= 0x8000 else value


def build_respray(mm: int, seq: int = 0) -> bytes:
    """补喷指定毫米。这是「视觉的偏移量怎么传下去」的答案。"""
    return build_frame(CMD_RESPRAY, seq, _u16(mm))


def build_move(direction: int, left_mm: int, right_mm: int = None,
               seq: int = 0) -> bytes:
    """走一段。right_mm 省略时左右同距（普通直行）。

    左右不等就是差速 —— 视觉纠偏就靠它。为什么纠偏走"每段重设"而不是
    "运动中加减"，见 core/lane_control.py 开头的说明：固件那边四个轮子共用
    一个 ARR，左右不可能同时到，差速只能表现为"内侧轮先停"。这种偏差如果
    在一条 5.3 米的线上一直累积，最后会横过来；每 200mm 重设一次，段尾
    四轮计数归零，误差就锁在一段之内。
    """
    if right_mm is None:
        right_mm = left_mm
    return build_frame(CMD_MOVE, seq,
                       bytes([direction & 0xFF]) + _u16(left_mm) + _u16(right_mm))


def build_trim(delta_mm: int, seq: int = 0) -> bytes:
    """设定段内差速。>0 = 整车向右偏一点，<0 = 向左。

    delta_mm 是"这一段累计该纠到多少"，不是"再纠这么多"。固件记住已经
    落地的量，只把差值加到左右剩余目标上 —— 同一个期望值连发多次不会
    越纠越多。这一点必须和调用方的算法一致，否则几百毫秒就撞上累计上限。

    和 build_move 的区别：它不改"这段还差多远"，只改左右两侧的剩余目标。
    适合响应快、段很细的场合（陀螺仪闭环、视觉逐帧纠偏）。
    """
    value = max(-32768, min(32767, int(delta_mm))) & 0xFFFF
    return build_frame(CMD_TRIM, seq, bytes([value >> 8, value & 0xFF]))


def build_pump(on: bool, seq: int = 0) -> bytes:
    """开/关水泵。画线要"走到哪喷到哪"，泵必须能跟移动命令一起下发。"""
    return build_frame(CMD_PUMP, seq, bytes([1 if on else 0]))


def build_paint(length_mm: int = 0, width_mm: int = 0, heading: bool = False,
                resume: bool = False, seq: int = 0) -> bytes:
    """按尺寸画一个矩形车位。传 0 表示用固件里的默认值 5.3m x 2.5m。

    heading=True 让四条边走陀螺仪航向闭环：画车位的边是 ForwardFlat（匀速、
    无斜坡），本来没有任何航向兜底，一条 5.3 米的边左右轮差 1% 就是末端 5cm，
    四条边还往同一个方向偏。

    这个开关随帧走而不是单独一条命令：一帧原子，"尺寸和纠偏一起生效"，
    不会出现「开关改了但这次画车位没赶上」。

    resume=True 是"从急停断点接着画"：只走那条边剩下的那段。此时长宽
    不被采纳（断点里带着这一圈的尺寸），固件只认断点。没有断点时回
    ERR_STATE，而不是默默当成新的一条从头画 —— 后者会把已经喷过的两条边
    再喷一遍。
    """
    flags = 0
    if heading:
        flags |= PAINT_FLAG_HEADING
    if resume:
        flags |= PAINT_FLAG_RESUME
    return build_frame(CMD_PAINT, seq,
                       _u16(length_mm) + _u16(width_mm) + bytes([flags]))


def build_speed(arr: int, seq: int = 0) -> bytes:
    """设速度。arr 是脉冲定时器的 ARR（越小越快），不是"第几档"。

    为什么不发档位号：ARR 是固件真正写进寄存器的那两个数之一，中间再加
    一层档位表，就多一处"两边表不一样"的机会 —— 补喷长度当年就是这么
    从档位改成毫米的。越界值固件回 ERR_RANGE，界面上的滑条已经先挡了一道。
    """
    return build_frame(CMD_SPEED, seq, _u16(arr))


def build_level(level: int, seq: int = 0) -> bytes:
    return build_frame(CMD_LEVEL, seq, bytes([level & 0xFF]))


def build_gyro(sub: int, seq: int = 0) -> bytes:
    """陀螺仪闭环的开关与标定。

    GYRO_MEASURE 会让车自己动起来（单侧走 300mm），所以固件只在 WAIT 状态
    接受它 —— 界面上的按钮也必须只在空闲时可点。
    """
    return build_frame(CMD_GYRO, seq, bytes([sub & 0xFF]))


def build_ping(seq: int = 0) -> bytes:
    return build_frame(CMD_PING, seq)


def describe(cmd: int, data: bytes) -> str:
    """把一帧翻译成人话，给日志和状态栏用。"""
    if cmd == CMD_RESPRAY:
        mm = (data[0] << 8) | data[1]
        return f"补喷 {mm}mm"
    if cmd == CMD_MOVE:
        left = (data[1] << 8) | data[2]
        right = (data[3] << 8) | data[4]
        if right == 0 or right == left:
            dist = f"{left}mm"
        else:
            dist = f"左{left} 右{right}mm"
        return f"{MOVE_NAMES.get(data[0], '?')} {dist}"
    if cmd == CMD_LEVEL:
        return f"设档 {data[0]}（{data[0] * 200}mm）"
    if cmd == CMD_TRIM:
        value = (data[0] << 8) | data[1]
        if value >= 0x8000:
            value -= 0x10000
        return f"纠偏 {value:+d}mm"
    if cmd == CMD_PUMP:
        return "水泵 开" if data[0] else "水泵 关"
    if cmd == CMD_PAINT:
        if data[4] & PAINT_FLAG_RESUME:
            # 续画时帧里那四个字节不生效，日志再显示长宽会误导
            return "画车位 从断点续画"
        length = (data[0] << 8) | data[1] or PAINT_LEN_DEFAULT_MM
        width = (data[2] << 8) | data[3] or PAINT_WID_DEFAULT_MM
        tail = " +陀螺仪纠偏" if data[4] & PAINT_FLAG_HEADING else ""
        return f"画车位 {length}x{width}mm{tail}"
    if cmd == CMD_SPEED:
        return f"速度 {((data[0] << 8) | data[1])}"
    if cmd == CMD_PING:
        return "PING"
    if cmd == CMD_ACK:
        return f"ACK {CMD_NAMES.get(data[0], hex(data[0]))} {RESULT_NAMES.get(data[1], hex(data[1]))}"
    if cmd == CMD_STATUS:
        pulse = (data[3] << 8) | data[4]
        state = chr(data[0]) if 32 <= data[0] < 127 else str(data[0])
        step = PAINT_STEP_LABELS.get(data[1], str(data[1])) if state == "P" else str(data[1])
        return f"状态 {state} 段{step} 脉冲{pulse}"
    if cmd == CMD_GYRO:
        names = {GYRO_OFF: "关闭环", GYRO_ON: "开闭环",
                 GYRO_RECAL: "重标零", GYRO_MEASURE: "标轮距"}
        return f"陀螺仪 {names.get(data[0], hex(data[0]))}"
    if cmd == CMD_SPEED_INFO:
        arr = (data[0] << 8) | data[1]
        tail = " 已存" if data[2] & SPEED_FLAG_SAVED else " 待存"
        return f"速度 {arr}{tail}"
    if cmd == CMD_RESUME_INFO:
        if data[4] == 0:
            return "断点 无"
        length = (data[0] << 8) | data[1]
        width = (data[2] << 8) | data[3]
        step = PAINT_STEP_LABELS.get(data[4], str(data[4]))
        return f"断点 {length}x{width}mm {step}"
    if cmd == CMD_BATTERY:
        mv = (data[0] << 8) | data[1]
        if data[2] & BATT_FLAG_ABSENT:
            # 分压电阻没接的时候 PC0 是悬空的，读回来是个乱跳的值。
            return "电池 未接（无分压）"
        if not data[2] & BATT_FLAG_VALID:
            # 这一轮没读到（ADC 超时），下一轮可能就好了。
            return "电池 未读到"
        tail = " 低压" if data[2] & BATT_FLAG_LOW else ""
        return f"电池 {mv / 1000.0:.1f}V{tail}"
    if cmd == CMD_HEADING:
        deg = _s16(data[0], data[1]) / 100.0
        rate = _s16(data[2], data[3]) / 10.0
        state = GYRO_STATE_NAMES.get((data[4] >> 4) & 0x0F, "?")
        return f"航向 {deg:+.2f}° {rate:+.1f}dps {state}"
    return f"未知帧 0x{cmd:02X}"


def make_frame_record(frame: bytes) -> dict:
    """一帧 -> UI 能直接用的记录，键和单字节记录保持同一套。"""
    cmd = frame[1]
    data = frame[3:8]
    record = {
        "kind": "frame",
        "raw": frame.hex(),
        "char": chr(cmd) if 32 <= cmd <= 126 else "·",
        "value": cmd,
        "seq": frame[2],
        "cmd": cmd,
        "data": data,
        "status": describe(cmd, data),
    }

    # 航向帧额外给几个结构化字段。UI 不该为了拿一个角度去切片 data 数组 ——
    # 那样每个调用点都要自己处理符号和单位，迟早有一处漏掉。
    if cmd == CMD_SPEED_INFO:
        # 和航向帧同一个理由：UI 不该为了拿个整数去自己切片。
        record["speed_arr"] = (data[0] << 8) | data[1]
        record["speed_saved"] = bool(data[2] & SPEED_FLAG_SAVED)

    if cmd == CMD_RESUME_INFO:
        record["resume_len_mm"] = (data[0] << 8) | data[1]
        record["resume_wid_mm"] = (data[2] << 8) | data[3]
        record["resume_step"] = data[4]
        record["resume_valid"] = bool(data[4])
    if cmd == CMD_BATTERY:
        record["batt_mv"] = (data[0] << 8) | data[1]
        record["batt_valid"] = bool(data[2] & BATT_FLAG_VALID)
        record["batt_low"] = bool(data[2] & BATT_FLAG_LOW)
        record["batt_absent"] = bool(data[2] & BATT_FLAG_ABSENT)

    if cmd == CMD_HEADING:
        record["heading_deg"] = _s16(data[0], data[1]) / 100.0
        record["rate_dps"] = _s16(data[2], data[3]) / 10.0
        record["gyro_state"] = (data[4] >> 4) & 0x0F
        record["gyro_errors"] = data[4] & 0x0F

    return record


def make_byte_record(value: int) -> dict:
    return {
        "kind": "byte",
        "raw": f"{value:02x}",
        "char": chr(value),
        "value": value,
        "status": "未知",      # 由 protocol.py 填，避免这里反向依赖命令表
    }


def parse_stream(buffer: bytearray, data: bytes = b"",
                 last_rx: float = None, now: float = None,
                 resync: bool = False):
    """从滚动缓冲里切出所有完整的消息。

    返回 (records, last_rx, dropped, resync)。records 里既有帧，也有裸字节 ——
    这就是「重同步」和「兼容单字节老协议」是同一个动作的意思：

        首字节是 0x7B？ --否--> 静默期？--是--> 丢掉不执行
               |                            --否--> 当成一条单字节命令
               |是
        够 10 字节？   --否--> 等下一批（且记时间，超时清残帧）
               |是
        帧尾 + CRC 对？ --否--> 只丢 1 字节并进入静默，重新找帧头
               |对
        交给帧处理，丢 10 字节，退出静默

    错误时只丢 1 字节是这里最关键的一条：真帧的 0x7B 恰好落在假帧头后面时，
    丢整帧会把真帧一起吞掉，然后永久失步。而静默期是第二条：坏帧的 payload
    不能再被逐字节执行一遍。
    """
    now = time.monotonic() if now is None else now
    dropped = 0
    prev_rx = last_rx

    # 先处理上一批留下的半帧。卡了太久就是断流了，整段丢掉 ——
    # 一个孤立的 0x7B（噪声）如果一直躺在队首，后面每条命令都会被当成
    # "帧的第 2、3…个字节"，永远等不齐 10 字节，整条链路静默失效。
    #
    # 清掉的不只是缓冲：如果残帧里已经有 payload（长度 > 1），这一拍同时
    # 到达的字节很可能就是它的尾巴，也要一起丢 —— 下面的分支处理这个。
    partial_timed_out = False
    partial_had_payload = False

    if (prev_rx is not None and buffer and buffer[0] == SOF
            and len(buffer) < FRAME_LEN
            and (now - prev_rx) > PARTIAL_TIMEOUT_S):
        # 只有"帧头 + 至少一个字节"才算收到了 payload。孤立的 0x7B（长度 1）
        # 后面不可能有它的尾巴，清掉就该正常往下走 —— 那是噪声，不是半帧。
        partial_had_payload = len(buffer) > 1
        dropped += len(buffer)
        buffer.clear()
        partial_timed_out = True

    if data and partial_timed_out and partial_had_payload:
        # 刚清掉的是一个有 payload 的残帧，而这批字节很可能是它的尾巴
        # （串口把一次发送切成两段，中间卡了 200ms 以上）。逐字节执行等于
        # 把半个帧的 payload 当命令重放一遍 —— 里面随便一个 'T' 就是急停、
        # 一个 'C' 就是开泵补喷。
        #
        # 整批丢掉。代价是"如果它其实是一条新的单字节命令，这条会丢一次"，
        # 但丢一条命令远好过执行半帧里恰好凑成合法字符的那几个字节。
        dropped += len(data)
        last_rx = now
        data = b""

    if data:
        # 上一段数据之后空闲了很久，说明新数据是新的一次发送，不是坏帧的尾巴
        if resync and prev_rx is not None and (now - prev_rx) > RESYNC_IDLE_S:
            resync = False
        buffer.extend(data)
        last_rx = now

    if last_rx is None:
        last_rx = now

    records = []

    while buffer:
        if buffer[0] == SOF:
            if len(buffer) < FRAME_LEN:
                break
            frame = bytes(buffer[:FRAME_LEN])
            if check_frame(frame):
                del buffer[:FRAME_LEN]
                records.append(make_frame_record(frame))
                resync = False
            else:
                dropped += 1
                del buffer[:1]
                resync = True
            continue

        if resync:
            dropped += 1
            del buffer[:1]
            continue

        value = buffer[0]
        del buffer[:1]
        records.append(make_byte_record(value))

    return records, last_rx, dropped, resync
