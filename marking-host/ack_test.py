"""ACK 配对表自检。

    python3 ack_test.py     # 或 pytest ack_test.py

这张表的价值在于「把三种完全不同的故障分开」：没 ACK = 链路断了，
ACK 带错误码 = 命令被回绝，ACK 带 OK = 真的执行了。分开之后，
"按了按钮车没动"才有得查。所以测试也围绕这三件事写。
"""

from core import frame
from core.ack import AckTracker


def _sent_frame(cmd, seq, data=b""):
    return frame.build_frame(cmd, seq, data)


def _recorder(calls):
    """假的写串口函数：记下每一次重发，并如实返回"写出去了"。

    返回 None 的话新逻辑会当成"串口断了"，重发不记账、直接判超时 ——
    这正是 test_resend_is_not_counted_when_the_port_is_closed 要验的那条。
    """
    def _fn(raw):
        calls.append(raw)
        return True
    return _fn


def test_ok_round_trip():
    t = AckTracker()
    raw = _sent_frame(frame.CMD_RESPRAY, 7)

    item = t.register(raw, now=100.0)
    assert item is not None and item.seq == 7

    acked = t.on_ack(7, frame.CMD_RESPRAY, frame.RESULT_OK, now=100.042)

    assert acked is not None
    assert abs(acked.rtt - 0.042) < 1e-9

    h = t.health()
    assert h["acked"] == 1 and h["timed_out"] == 0 and h["inflight"] == 0
    assert h["healthy"] is True


def test_rejected_counts_as_not_healthy():
    """ACK 回来了但结果是"状态不允许" —— 链路是好的，命令是没执行的。

    这两种必须分开：链路好而命令被拒，问题在逻辑（比如车还在画车位时
    发了补喷）；没 ACK 才是线的问题。
    """
    t = AckTracker()
    t.register(_sent_frame(frame.CMD_RESPRAY, 3), now=0.0)
    t.on_ack(3, frame.CMD_RESPRAY, frame.RESULT_ERR_STATE, now=0.01)

    h = t.health()
    assert h["acked"] == 1
    assert h["rejected"] == 1
    assert h["healthy"] is False
    assert t.last_error[0] == frame.CMD_RESPRAY


def test_timeout_without_resend():
    t = AckTracker(timeout_s=0.3)
    t.register(_sent_frame(frame.CMD_PING, 1), now=0.0)

    dead, resent = t.expire(now=0.1)
    assert dead == [] and resent == []      # 还没到

    dead, resent = t.expire(now=0.31)
    assert len(dead) == 1 and resent == []
    assert t.timed_out == 1


def test_resend_once_then_give_up():
    """重发只做一次。

    第一次没回可能是下位机在刷 OLED（100ms），重发能救；
    线断了重发一百次也救不了，只会把日志刷满掩盖真正的故障。
    """
    t = AckTracker(timeout_s=0.3, max_retries=1)
    t.register(_sent_frame(frame.CMD_MOVE, 9), now=0.0)

    calls = []
    dead, resent = t.expire(now=0.31, resend=_recorder(calls))
    assert len(resent) == 1 and dead == []
    assert len(calls) == 1
    assert t.retried == 1

    # 重发后又过了一个超时窗口，这次不再重发
    dead, resent = t.expire(now=0.62, resend=_recorder(calls))
    assert len(dead) == 1 and resent == []
    assert len(calls) == 1
    assert t.timed_out == 1


def test_resend_keeps_same_seq():
    """重发必须用同一个 SEQ —— 固件按 (SEQ, CMD) 去重，同 SEQ 再来一次
    只回 ACK 不执行。换了 SEQ 就会真的执行第二遍：'P' 画两个车位。"""
    t = AckTracker(timeout_s=0.3)
    raw = _sent_frame(frame.CMD_PAINT, 42)
    t.register(raw, now=0.0)

    calls = []
    t.expire(now=0.31, resend=_recorder(calls))

    assert calls[0] == raw
    assert calls[0][2] == 42


def test_single_byte_is_not_registered():
    """单字节命令没有 ACK，登记了必然超时，等于自己制造假警报。"""
    t = AckTracker()
    assert t.register(b"P", now=0.0) is None
    assert t.register(b"", now=0.0) is None
    assert t.sent == 0


def test_bad_frame_is_not_registered():
    """CRC 错的帧本来就没发出去（或者是内部算错了），不该进表。"""
    t = AckTracker()
    raw = bytearray(_sent_frame(frame.CMD_MOVE, 5))
    raw[4] ^= 0xFF

    assert t.register(bytes(raw), now=0.0) is None


def test_status_and_ack_frames_are_not_registered():
    """上行帧不该被当成"我发的命令"登记。"""
    t = AckTracker()
    assert t.register(_sent_frame(frame.CMD_STATUS, 0), now=0.0) is None
    assert t.register(_sent_frame(frame.CMD_ACK, 0), now=0.0) is None


def test_unknown_seq_is_ignored():
    """收到一条早已被清掉的命令的 ACK：正常现象（比如重发后第一条才到），
    不该抛异常，也不该污染统计。"""
    t = AckTracker()
    assert t.on_ack(99, frame.CMD_PING, frame.RESULT_OK, now=1.0) is None
    assert t.acked == 0


def test_seq_reuse_does_not_mismatch():
    """SEQ 只有 8 位，绕一圈之后同号复用。旧的 ACK 晚到时，cmd 对不上 ——
    必须丢掉，不能把一条 200 秒前发的命令标成"刚刚确认"。

    这条是这张表里最容易出静默错误的地方：标错了不报错，但 RTT 会变成
    200 秒，而人会盯着那个数判断"串口是不是很慢"。
    """
    t = AckTracker(timeout_s=1.0)

    t.register(_sent_frame(frame.CMD_MOVE, 5), now=0.0)
    t.register(_sent_frame(frame.CMD_RESPRAY, 5), now=100.0)

    # 迟到的、属于 CMD_MOVE 的 ACK
    assert t.on_ack(5, frame.CMD_MOVE, frame.RESULT_OK, now=100.01) is None
    assert t.acked == 0

    # 真正属于 CMD_RESPRAY 的那条
    acked = t.on_ack(5, frame.CMD_RESPRAY, frame.RESULT_OK, now=100.02)
    assert acked is not None and abs(acked.rtt - 0.02) < 1e-9


def test_worst_rtt_tracks_maximum():
    t = AckTracker()
    for seq, rtt in ((1, 0.01), (2, 0.05), (3, 0.02)):
        t.register(_sent_frame(frame.CMD_PING, seq), now=0.0)
        t.on_ack(seq, frame.CMD_PING, frame.RESULT_OK, now=rtt)

    assert abs(t.worst_rtt - 0.05) < 1e-9
    assert abs(t.health()["last_rtt_ms"] - 20.0) < 1e-6


def test_health_loss_percent():
    t = AckTracker(timeout_s=0.1)

    t.register(_sent_frame(frame.CMD_PING, 1), now=0.0)
    t.on_ack(1, frame.CMD_PING, frame.RESULT_OK, now=0.01)

    t.register(_sent_frame(frame.CMD_PING, 2), now=0.0)
    t.expire(now=0.5)

    h = t.health()
    assert h["timed_out"] == 1 and h["acked"] == 1
    assert abs(h["loss_pct"] - 50.0) < 1e-9
    assert h["healthy"] is False


def test_resend_is_not_counted_when_the_port_is_closed():
    """写不出去就不算重发。

    串口断着的时候把 retries 加一、把计时推到 now，等于骗调用方"这条救过
    一次"：真正断线要再等一个超时才暴露，retried 也虚高。而且这条命令根本
    没上总线，等下去不会有人来 ACK。
    """
    t = AckTracker(timeout_s=0.3, max_retries=1)
    t.register(_sent_frame(frame.CMD_MOVE, 9), now=0.0)

    tried = []

    def dead_port(raw):
        tried.append(raw)
        return False

    dead, resent = t.expire(now=0.31, resend=dead_port)

    assert len(tried) == 1, "连试都没试就判死了"
    assert resent == [], "写失败不能算重发成功"
    assert len(dead) == 1, "这条命令根本没发出去，应该当场判超时"
    assert t.retried == 0
    assert t.timed_out == 1
    assert t.pending == {}, "写不出去的条目不能留在表里"


def test_pending_does_not_leak_on_seq_wrap():
    """同 SEQ 再发一次，表里只留一条 —— 否则长期高频下发（视觉伺服 10Hz）
    会攒出一张上万条的待确认表。"""
    t = AckTracker(timeout_s=999.0)
    for _ in range(50):
        t.register(_sent_frame(frame.CMD_MOVE, 7), now=0.0)

    assert len(t.pending) == 1


if __name__ == "__main__":
    for name in sorted(n for n in globals() if n.startswith("test_")):
        globals()[name]()
        print("ok", name)
    print("全部通过")
