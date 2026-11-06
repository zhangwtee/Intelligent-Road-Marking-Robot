# 下位机 HandleCmd 认识的全部指令（见 marking-firmware/Hardware/system.c）。
# 下位机收到什么就原样回显什么，所以这张表既是接收日志的翻译，
# 也是面板按钮取名字的依据 —— 改指令集时两边一起改。
from core import frame

COMMANDS = {
    ord('P'): "画车位（标准）",
    ord('C'): "二次喷涂",         # 单字节版：按当前档位走一段。相机闭环走帧 RESPRAY，见下
    ord('D'): "整机自检",
    ord('K'): "水泵清洗",
    ord('G'): "关水泵",
    ord('W'): "停止复位",
    ord('T'): "紧急停止",
    ord('J'): "夹臂关",
    ord('F'): "蜂鸣提示",
    ord('U'): "前进",
    ord('R'): "后退",
    ord('Y'): "右转",
    ord('Z'): "左转",
    ord('A'): "左移",
    ord('X'): "右移",
    ord('+'): "加速",
    ord('-'): "减速",
    # 补喷长度档位：'1'~'5' = 0.2 / 0.4 / 0.6 / 0.8 / 1.0 m。
    # 面板上不出现，只在串口助手里用；帧协议那边有 CMD_LEVEL 和直接的毫米数。
    # 保留它们是因为「敲个数字再敲 C」在调试时比手拼 10 字节帧方便得多。
    ord('1'): "补喷 0.2m",
    ord('2'): "补喷 0.4m",
    ord('3'): "补喷 0.6m",
    ord('4'): "补喷 0.8m",
    ord('5'): "补喷 1.0m",
}


# 面板按钮：(标签, 指令字节)。标签同时是 send_button 的查表键，所以按钮
# 只在这里定义一次 —— 原来标签在 init_ui、cmd_map、摄像头弹窗判断里各写了
# 一遍，改错一个字就有一个按钮悄悄失效（C 就是这么漏掉的）。
# 规矩：只放固件 system.c 里真的认的字节。没实现的先别挂上来，别让界面
# 上出现按了没反应的按钮。
PANEL_BUTTONS = [
    ("画车位（标准）", b'P'),
    ("整机自检",       b'D'),
    ("水泵清洗",       b'K'),
    ("关水泵",         b'G'),
    ("二次喷涂",       b'C'),
    ("停止复位",       b'W'),
    ("紧急停止",       b'T'),
]

# 按下去要弹出摄像头画面的按钮
CAMERA_BUTTONS = {"二次喷涂"}

# 按下去要收起摄像头画面的按钮
CAMERA_OFF_BUTTONS = {"紧急停止"}


class Protocol:
    """接收解析器：帧和裸字节共用一条串口。

    frame_len = 1（定长切分）是改造前的全部协议假设 —— 它保证「永不错位」，
    代价是「永远发现不了错误」：噪声不需要骗过校验，它天生合法。
    现在改成滚动缓冲：首字节是帧头就走帧路径，不是就当成一条单字节命令。
    """

    def __init__(self):
        self.buffer = bytearray()
        self.last_rx = 0.0
        self.dropped = 0        # 因 CRC 错 / 残帧超时 / 重同步而丢弃的字节数
        self.resync = False     # 坏帧之后、重新找到合法帧之前，裸字节不执行

    def parse(self, data: bytes = b"", now: float = None):
        """
        把收到的字节流切成一条条消息。

        返回的记录两类共用同一套键（raw / char / value / status），
        所以 UI 的接收日志一行都不用改：
            kind = "frame"  带 seq / cmd / data
            kind = "byte"   老的单字节命令
        """
        records, self.last_rx, dropped, self.resync = frame.parse_stream(
            self.buffer, data, self.last_rx, now, self.resync
        )
        self.dropped += dropped

        for item in records:
            if item["kind"] == "byte":
                item["status"] = COMMANDS.get(item["value"], "未知")
            elif item["cmd"] == frame.CMD_ACK:
                item["acked"] = True

        return records

    def pending(self) -> int:
        """缓冲里还没凑成一条消息的字节数（调试用）。"""
        return len(self.buffer)

