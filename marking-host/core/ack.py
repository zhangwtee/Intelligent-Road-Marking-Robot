"""ACK 配对表：把「我发了」和「它收了」对上。

为什么需要这个
--------------
接上帧协议之后，上位机其实已经有了 ACK，但当时只做到"ACK 进日志、人眼看得见"。
那等于没有：一条 10.4ms 的帧发出去，串口线松了、下位机在刷 OLED 卡住 300ms、
状态机在 STATE_TING 里把命令回绝了 —— 这三种情况在人眼里都是"日志上出现过
一个 ACK"，但实际行为完全不同：

    线松了      -> 根本没有 ACK
    被回绝      -> 有 ACK，result = ERR_STATE
    被接受的    -> 有 ACK，result = OK，而且 RTT 很小

不配对，这三件事分不开。而"按了按钮车没动"这种问题，90% 的时间花在区分
它们上。

怎么配
------
帧里有 SEQ。发的时候登记 (seq -> cmd, 时刻)，收到 ACK 时按 seq 查回来，
顺手算出 RTT。超时没回来的，标记、可选重发一次。

SEQ 只有 8 位，256 条就绕一圈。用 OrderedDict 按发送顺序存，收到 ACK 时
从**最近的**开始找同 seq 的那条 —— 因为串口是 FIFO，最近的还没配上的那条
最可能就是它。找不到就当 ACK 属于一条早就超时的命令，丢掉但不报错。

重发为什么有用，为什么只重发一次
------------------------------
固件对 (seq, cmd) 有 1 秒去重：同 seq 同 cmd 再来一次，只回 ACK 不执行。
所以重发是安全的 —— 不会画两个车位。

但只重发一次：如果第一次没回是"下位机在刷 OLED"（100ms），重发能救；
如果是"线断了"，重发一百次也救不了，只会把日志刷满，掩盖真正的故障。
一次重发足够把"偶发"和"断开"分开。
"""

import time
from collections import OrderedDict

from core import frame

# 一次命令多久没回 ACK 就认为丢了。
# 9600 波特率下一帧 10.4ms，下位机刷一次 OLED 要 80~100ms（这期间主循环
# 是停的，不处理串口），再加上它自己的状态帧也是 5Hz —— 300ms 大约等于
# "连续错过两次处理机会"。再短会误判，再长就追不上人按键的节奏了。
ACK_TIMEOUT_S = 0.30

# 最多重发几次。见文件头。
MAX_RETRIES = 1


class Pending:
    """一条发出去还没被确认的帧。"""

    __slots__ = ("seq", "cmd", "payload", "sent_at", "retries", "rtt", "result")

    def __init__(self, seq, cmd, payload, sent_at):
        self.seq = seq
        self.cmd = cmd
        self.payload = payload
        self.sent_at = sent_at
        self.retries = 0
        self.rtt = None
        self.result = None

    def age(self, now):
        return now - self.sent_at


class AckTracker:
    """发-确认配对。上层只管 register() / on_ack() / expire()。"""

    def __init__(self, timeout_s=ACK_TIMEOUT_S, max_retries=MAX_RETRIES):
        self.timeout_s = timeout_s
        self.max_retries = max_retries

        self.pending = OrderedDict()   # seq -> Pending，按发送顺序

        self.sent = 0
        self.acked = 0
        self.rejected = 0              # 收到 ACK 但 result != OK
        self.timed_out = 0
        self.retried = 0

        self.last_rtt = None
        self.worst_rtt = None
        self.last_error = None         # (cmd, result, 时刻)

    # ------------------------------------------------------------ 发送侧
    def register(self, raw: bytes, now=None):
        """登记一条刚发出去的帧。raw 是完整帧（含帧头帧尾）。

        单字节命令不登记 —— 固件对它们只回显、不 ACK，登记了必然超时，
        等于给自己制造假警报。
        """
        now = time.monotonic() if now is None else now

        if len(raw) != frame.FRAME_LEN or not frame.check_frame(raw):
            return None

        cmd = raw[1]
        if cmd == frame.CMD_ACK or cmd == frame.CMD_STATUS:
            return None

        seq = raw[2]
        item = Pending(seq, cmd, raw, now)

        # 256 条绕回来之前，同 seq 的旧记录早就该超时清掉了。真撞上就覆盖，
        # 不能让表无限长 —— 那是一个只在"发得很多又都丢"的情况下才发作的泄漏。
        self.pending[seq] = item
        self.sent += 1
        return item

    # ------------------------------------------------------------ 接收侧
    def on_ack(self, seq: int, cmd: int, result: int, now=None):
        """收到一帧 ACK。返回配对上的 Pending（已填好 rtt / result），
        或者 None（说明这条 ACK 属于一条早就被清掉的命令）。"""
        now = time.monotonic() if now is None else now

        # 先看再删。用 pop 的话，一条 cmd 对不上的迟到 ACK 会把表里
        # 正主一起删掉 —— 而且它自己还被判成"未匹配"，两败俱伤。
        item = self.pending.get(seq)

        if item is None:
            return None

        if item.cmd != cmd:
            # cmd 对不上：seq 被复用过了，这条 ACK 是更早那一条的。
            # 不报错，也不消耗统计 —— 这是正常现象，不是故障。
            return None

        del self.pending[seq]

        item.rtt = now - item.sent_at
        item.result = result

        self.acked += 1
        self.last_rtt = item.rtt
        if self.worst_rtt is None or item.rtt > self.worst_rtt:
            self.worst_rtt = item.rtt

        if result != frame.RESULT_OK:
            self.rejected += 1
            self.last_error = (cmd, result, now)

        return item

    # ------------------------------------------------------------ 超时
    def expire(self, now=None, resend=None):
        """把所有超时的条目摘出来。

        resend：可选回调，签名 resend(raw_bytes) -> bool。给了就自动重发
        一次（同一 SEQ —— 固件按 SEQ 去重，重发不会导致重复执行）。
        返回 (超时且不再重发的条目列表, 重发过的条目列表)。
        """
        now = time.monotonic() if now is None else now

        dead = []
        resent = []

        for seq in list(self.pending.keys()):
            item = self.pending[seq]

            if item.age(now) < self.timeout_s:
                continue

            if resend is not None and item.retries < self.max_retries:
                # 先看写没写出去再记账。写失败（串口断了、写异常）时把
                # retries 加一、把计时推到 now，等于告诉调用方"这条重发过"——
                # 真正断线要再等一个超时才暴露，retried 也是虚的。
                # 而且这条命令根本没上总线，等下去也不会有人来 ACK 它。
                if not resend(item.payload):
                    del self.pending[seq]
                    self.timed_out += 1
                    dead.append(item)
                    continue

                item.retries += 1
                item.sent_at = now          # 重新计时，不是"宽限"
                self.retried += 1
                resent.append(item)
                continue

            del self.pending[seq]
            self.timed_out += 1
            dead.append(item)

        return dead, resent

    # ------------------------------------------------------------ 健康度
    def health(self):
        """给界面用的一句话摘要。不用查表也不用算，直接显示。"""
        total = self.acked + self.timed_out
        loss = (self.timed_out / total * 100.0) if total else 0.0

        rtt = "—" if self.last_rtt is None else f"{self.last_rtt * 1000:.0f}ms"

        return {
            "sent": self.sent,
            "acked": self.acked,
            "rejected": self.rejected,
            "timed_out": self.timed_out,
            "retried": self.retried,
            "inflight": len(self.pending),
            "loss_pct": loss,
            "last_rtt_ms": None if self.last_rtt is None else self.last_rtt * 1000,
            "rtt": rtt,
            "healthy": loss < 5.0 and self.rejected == 0,
        }

    def describe(self):
        h = self.health()
        return (f"链路 收{h['acked']}/{h['sent']} 丢{h['timed_out']} "
                f"拒{h['rejected']} RTT {h['rtt']}")
