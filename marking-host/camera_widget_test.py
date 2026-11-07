"""
camera_widget 自检。跑它才知道"检测到破损 -> 真的发出去了"这条路是通的：

    python3 camera_widget_test.py     # 或 pytest camera_widget_test.py

这条链断过两次，所以断言分三层盯着：
  1. 链路      —— 判出破损之后，字节到底有没有上总线
  2. 投票      —— 单帧噪点不能触发补喷（以前一帧反光就能喷出去一米）
  3. 三态      —— "看不清"必须和"完好"分开，而且都要关泵

Qt 不是必须的：没有 PyQt5 就就地补一个只够本文件用的替身（见 _install_stub），
所以没装 PyQt5 的开发机也能跑。cv2 是必须的 —— 检测逻辑本身要用。
"""

import os
import sys
import types
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

HAVE_REAL_QT = True
try:
    import PyQt5  # noqa: F401
except ImportError:
    HAVE_REAL_QT = False


def _install_stub():
    """补上 camera_widget 用到的那几个 Qt 名字，别的什么都不做。

    这不是"模拟 Qt"，只是让模块能 import、setPixmap 之类不炸 ——
    被验证的是 update_frame 里的检测与闭环逻辑，那些和 Qt 无关。
    """
    class _Sig:
        def connect(self, *a):
            pass

    class QLabel:
        def __init__(self, *a, **k): pass
        def setAlignment(self, *a): pass
        def setText(self, *a): pass
        def setPixmap(self, *a): pass
        def clear(self): pass
        def size(self): return (640, 480)

    class QTimer:
        def __init__(self, *a, **k): self.timeout = _Sig()
        def start(self, *a): pass
        def stop(self, *a): pass

    class _Qt:
        AlignCenter = KeepAspectRatio = SmoothTransformation = 0

    class QImage:
        Format_RGB888 = 0
        def __init__(self, *a, **k): pass

    class QPixmap:
        @staticmethod
        def fromImage(img): return QPixmap()
        def scaled(self, *a): return self

    for mod, attrs in (
        ("PyQt5", {}),
        ("PyQt5.QtWidgets", {"QLabel": QLabel}),
        ("PyQt5.QtCore", {"QTimer": QTimer, "Qt": _Qt}),
        ("PyQt5.QtGui", {"QImage": QImage, "QPixmap": QPixmap}),
    ):
        m = types.ModuleType(mod)
        for k, v in attrs.items():
            setattr(m, k, v)
        sys.modules[mod] = m


if not HAVE_REAL_QT:
    _install_stub()

import ui.camera_widget as camera_widget  # noqa: E402  必须在替身之后
from core import frame  # noqa: E402
from core.line_quality import (  # noqa: E402
    QUALITY_OK, QUALITY_WORN, QUALITY_BREAK, QUALITY_UNSURE,
)


_app = None
if HAVE_REAL_QT:
    # 真的 Qt 下 QLabel 之前必须先有 QApplication，否则 "Must construct a
    # QApplication before a QWidget" 直接 abort。树莓派上可能没有 DISPLAY。
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    from PyQt5.QtWidgets import QApplication
    _app = QApplication.instance() or QApplication([])


class FakeSerial:
    def __init__(self):
        self.sent = []

    def send(self, data):
        self.sent.append(data)


def frames_of(serial, cmd=None):
    """从发送记录里挑出合法的帧。单字节命令不是帧，会被过滤掉。"""
    out = []
    for data in serial.sent:
        if len(data) == frame.FRAME_LEN and frame.check_frame(data):
            if cmd is None or data[1] == cmd:
                out.append(data)
    return out


def line_frame(length=380, y0=50):
    """一帧带竖黄线的画面，长宽比够格被认成线状。"""
    img = np.zeros((480, 640, 3), np.uint8)
    cv2.line(img, (320, y0), (320, y0 + length), (0, 255, 255), 25)
    return img


def tilted_line_frame(deg=8.0, x0=320, length=380, y0=50):
    """倾斜的线，用来验证纠偏的符号。"""
    img = np.zeros((480, 640, 3), np.uint8)
    dx = int(length * np.tan(np.deg2rad(deg)))
    cv2.line(img, (x0, y0), (x0 + dx, y0 + length), (0, 255, 255), 25)
    return img


def blank_frame():
    return np.zeros((480, 640, 3), np.uint8)


def feed(widget, frame_img):
    """把一帧塞进 update_frame —— cap 换成一个只会回这一帧的假摄像头。"""
    widget.cap = type("Cap", (), {"read": lambda self: (True, frame_img)})()
    widget.update_frame()


def patch_quality(monkeypatch_value):
    """替换判据。测的是"判出来之后发不发得出去"，判据本身在 vision_test.py。"""
    camera_widget.classify_line_quality = monkeypatch_value


def restore_quality():
    import importlib
    importlib.reload(camera_widget)


# ================================================================ 基础
def test_queue_clears_when_line_is_lost():
    """认不到线必须清空平滑队列。

    不清的话，线跳走再回来、或者换到旁边那条，队列里最多 5 帧的旧中心会被和
    新中心一起平均 —— ROI 落在上一处，判出来的破损也是那一处的。
    """
    w = camera_widget.CameraWidget()

    for _ in range(3):
        feed(w, line_frame())
    assert len(w.center_queue) == 3, "认到线应当进队列"

    feed(w, blank_frame())
    assert len(w.center_queue) == 0, "没认到线时没清空队列 —— 会拿着旧中心算 ROI"

    feed(w, line_frame())
    assert len(w.center_queue) == 1


# ================================================================ 投票
def test_single_bad_frame_does_not_spray():
    """一帧噪点不能触发补喷。这是改造前后行为差别最大的地方。

    以前 30fps 下 3% 的误判率等于每秒喷一次，现场看起来是"它在乱喷"。
    现在要求 5 帧里至少 3 帧同意。
    """
    original = camera_widget.classify_line_quality
    s = FakeSerial()
    w = camera_widget.CameraWidget(serial_mgr=s, mode="respray")
    w.SPRAY_INTERVAL_S = 0.0        # 关掉限流，只验投票

    sequence = [
        (QUALITY_WORN, 3, "worn"),          # 噪点
        (QUALITY_OK, 0, "ok"),
        (QUALITY_OK, 0, "ok"),
        (QUALITY_OK, 0, "ok"),
        (QUALITY_OK, 0, "ok"),
    ]
    iterator = iter(sequence)

    patch_quality(lambda px: next(iterator, (QUALITY_OK, 0, "ok")))
    try:
        for _ in range(5):
            feed(w, line_frame())
    finally:
        camera_widget.classify_line_quality = original

    assert frames_of(s, frame.CMD_RESPRAY) == [], "单帧噪点就发了补喷"


def test_respray_uses_the_same_streak_rule_as_tracking():
    """补喷和巡线对"算不算破损成立"的判断必须一致。

    以前补喷是"投票通过就发"，巡线在投票之上还要连续 2 帧。同一个缺口，
    站着看和边走边看给两个答案，调试时会觉得两个模式脾气不一样 ——
    而且找不出哪一边才是"对的"。现在两边共用 _want_spray。
    """
    original = camera_widget.classify_line_quality
    s = FakeSerial()
    w = camera_widget.CameraWidget(serial_mgr=s, mode="respray")
    w.VOTE_MAJORITY = 1          # 只看"连续帧确认"这一层，不掺投票窗口

    patch_quality(lambda px: (QUALITY_BREAK, 4, "break"))
    try:
        feed(w, line_frame())
        assert not frames_of(s, frame.CMD_RESPRAY), "一帧破损就补喷了"

        feed(w, line_frame())
    finally:
        camera_widget.classify_line_quality = original

    assert frames_of(s, frame.CMD_RESPRAY), "连续两帧破损还没补喷"


def test_pump_off_needs_more_than_one_good_frame():
    """关泵也要连续帧：一处阴影把泵关了，下一帧又开，出漆断断续续。

    这条守着 _want_spray 的另一半 —— 只测开泵的话，把 PUMP_OFF_AFTER 的
    判断删掉也照样能过。
    """
    original = camera_widget.classify_line_quality
    s = FakeSerial()
    w = camera_widget.CameraWidget(serial_mgr=s, mode="track")
    w.start_track()
    w.TRIM_INTERVAL_S = 1000.0

    patch_quality(lambda px: (QUALITY_BREAK, 4, "break"))
    try:
        for _ in range(w.VOTE_WINDOW + w.PUMP_ON_AFTER):
            feed(w, line_frame())
        assert w.pump_on is True

        patch_quality(lambda px: (QUALITY_OK, 0, "ok"))
        feed(w, line_frame())
        assert w.pump_on is True, "一帧完好的就关泵了"
    finally:
        camera_widget.classify_line_quality = original


def test_sustained_damage_does_spray():
    """真的破损会连续几十帧成立，必须照常触发。"""
    original = camera_widget.classify_line_quality
    s = FakeSerial()
    w = camera_widget.CameraWidget(serial_mgr=s, mode="respray")
    w.SPRAY_INTERVAL_S = 0.0

    patch_quality(lambda px: (QUALITY_WORN, 3, "worn"))
    try:
        for _ in range(5):
            feed(w, line_frame())
    finally:
        camera_widget.classify_line_quality = original

    sent = frames_of(s, frame.CMD_RESPRAY)
    assert len(sent) >= 1, f"持续破损也没发: {s.sent}"


def test_respray_is_a_frame_not_two_bytes():
    """改造前是 b'2' + b'C' 两次 write，中间被打断就"档位变了车没动"。"""
    s = FakeSerial()
    w = camera_widget.CameraWidget(serial_mgr=s, mode="respray")

    w.maybe_send_respray(3)

    assert s.sent, "什么都没发"
    assert all(len(d) == frame.FRAME_LEN for d in s.sent), s.sent
    assert frames_of(s, frame.CMD_RESPRAY)


def test_respray_level_becomes_millimetres():
    """档位不是命令，毫米才是。3 档 = 600mm。"""
    s = FakeSerial()
    w = camera_widget.CameraWidget(serial_mgr=s, mode="respray")

    w.maybe_send_respray(3)

    sent = frames_of(s, frame.CMD_RESPRAY)[0]
    mm = (sent[3] << 8) | sent[4]
    assert mm == 600, mm


def test_respray_waits_until_firmware_is_idle():
    """固件还在跑上一段时不许发新的补喷帧 —— 节拍看状态，不看秒表。

    以前用固定 2 秒间隔猜"上一段跑完没有"，而 1m 的补喷段在默认速度下要
    2~3 秒。下一帧正好落在 STATE_RESPRAY 里，被固件回 ERR_STATE 丢掉：
    日志里只是一个结果码 0x01，车实际少喷一段，什么都不报。
    """
    s = FakeSerial()
    w = camera_widget.CameraWidget(serial_mgr=s, mode="respray")

    w.on_status("C")                       # 正在补喷，1m 还没走完
    w.last_spray = 0.0                     # 排除最小间隔的干扰
    w.maybe_send_respray(2)
    assert frames_of(s, frame.CMD_RESPRAY) == [], "固件还在跑就发了下一帧"

    w.on_status("W")                       # 走完，回到空闲
    w.maybe_send_respray(2)
    assert len(frames_of(s, frame.CMD_RESPRAY)) == 1, "空闲了却没发"


def test_respray_respects_minimum_interval():
    """空闲也不能连喷：状态帧 5Hz，不限速就一停发好几帧，漆堆在同一处。"""
    s = FakeSerial()
    w = camera_widget.CameraWidget(serial_mgr=s, mode="respray")

    w.on_status("W")
    w.last_spray = 0.0
    w.maybe_send_respray(2)
    w.maybe_send_respray(2)                # 紧接着再来一次
    assert len(frames_of(s, frame.CMD_RESPRAY)) == 1


def test_respray_still_sends_without_status_frames():
    """一帧回执都没收到时不能彻底不动 —— 退回"只看最小间隔"。"""
    s = FakeSerial()
    w = camera_widget.CameraWidget(serial_mgr=s, mode="respray")
    assert w._fw_state == "?", "刚建出来应当是未知，不是 W"

    w.last_spray = 0.0
    w.maybe_send_respray(1)
    assert len(frames_of(s, frame.CMD_RESPRAY)) == 1


# ================================================================ 只读
def test_view_mode_sends_nothing():
    """只是取景（画车位那个窗口）时一个字节都不该发。"""
    original = camera_widget.classify_line_quality
    s = FakeSerial()
    w = camera_widget.CameraWidget(serial_mgr=s, mode="view")

    patch_quality(lambda px: (QUALITY_WORN, 3, "worn"))
    try:
        for _ in range(6):
            feed(w, line_frame())
    finally:
        camera_widget.classify_line_quality = original

    assert s.sent == [], s.sent


def test_no_serial_manager_does_not_crash():
    """没接串口时（比如单独调试）不能炸，只是不发。"""
    w = camera_widget.CameraWidget(serial_mgr=None, mode="track")
    w.start_track()
    feed(w, line_frame())
    feed(w, blank_frame())


# ================================================================ 巡线
def test_start_track_sends_move_frame():
    """开始巡线要先发一段 MOVE，否则车根本不动。"""
    s = FakeSerial()
    w = camera_widget.CameraWidget(serial_mgr=s, mode="track")

    w.start_track(3000)

    moves = frames_of(s, frame.CMD_MOVE)
    assert len(moves) == 1
    left = (moves[0][4] << 8) | moves[0][5]
    right = (moves[0][6] << 8) | moves[0][7]
    assert left == 3000 and right == 3000


def test_stop_track_turns_pump_off_before_stopping():
    """先关泵再停车。顺序反了车停了泵还在出漆。

    停车的单字节是 'W' 不是 'T'：'T' 把固件停在 TING，而画车位、二次喷涂
    都只认 WAIT，撤完相机还得再补一条才能发命令。'W' 一条就回到 WAIT。
    """
    s = FakeSerial()
    w = camera_widget.CameraWidget(serial_mgr=s, mode="track")
    w.start_track()
    w.pump_on = True

    w.stop_track()

    assert frames_of(s, frame.CMD_PUMP), "没发关泵"
    assert s.sent[-1] == b"W", "最后一条应该是回 WAIT 的停止命令"


def test_stop_track_clears_the_pump_flag():
    """只发关泵帧、不改 pump_on 的话，下次巡线头几帧该喷的地方不喷。

    _apply_pump(True) 会因为"值没变"直接 return，而固件那边泵已经关了。
    """
    s = FakeSerial()
    w = camera_widget.CameraWidget(serial_mgr=s, mode="track")
    w.start_track()
    w.pump_on = True

    w.stop_track()

    assert w.pump_on is False, "pump_on 还留着，下次巡线发不出开泵帧"

    # 再巡线，第一次"该喷"必须真的发出一帧开泵
    before = len(frames_of(s, frame.CMD_PUMP))
    w._apply_pump(True)
    after = frames_of(s, frame.CMD_PUMP)
    assert len(after) > before, "重新巡线时开泵被「值没变」挡掉了"
    assert after[-1][3] == 1, "最后一帧应该是开泵"


def test_voted_level_comes_from_the_frames_that_voted():
    """补喷档位取"与投票结论一致的那几帧"，不是当前帧。

    当前帧判"看不清"时档位是 0，直接拿它去补会被夹成 200mm。
    """
    w = camera_widget.CameraWidget()

    w.votes.append((QUALITY_WORN, 5))
    w.votes.append((QUALITY_WORN, 5))
    w.votes.append((QUALITY_WORN, 4))
    w.votes.append((QUALITY_UNSURE, 0))

    assert w._voted_quality() == QUALITY_WORN
    assert w._voted_level(QUALITY_WORN) == 4, "应该取最近一帧磨损给出的档位"

    w.votes.clear()
    for _ in range(3):
        w.votes.append((QUALITY_UNSURE, 0))

    assert w._voted_level(QUALITY_WORN) == 1, "没有证据时退回最保守的档位"


def test_trim_is_rate_limited():
    """10Hz 的纠偏节流：不节流会把 9600 的带宽全占掉，状态帧收不回来。"""
    original = camera_widget.classify_line_quality
    s = FakeSerial()
    w = camera_widget.CameraWidget(serial_mgr=s, mode="track")
    w.start_track()
    w.TRIM_INTERVAL_S = 1000.0      # 只允许发第一帧

    patch_quality(lambda px: (QUALITY_OK, 0, "ok"))
    try:
        for _ in range(5):
            feed(w, tilted_line_frame(deg=10))
    finally:
        camera_widget.classify_line_quality = original

    trims = frames_of(s, frame.CMD_TRIM)
    assert len(trims) <= 1, f"节流没生效: {len(trims)} 帧"


def test_trim_direction_follows_line_tilt():
    """线在画面里向右倾 -> 车头要向右修 -> trim 为正。

    符号错了车会越纠越歪，而且现场看着像"机械间隙"。
    """
    original = camera_widget.classify_line_quality
    s = FakeSerial()
    w = camera_widget.CameraWidget(serial_mgr=s, mode="track")
    w.start_track()
    w.TRIM_INTERVAL_S = 0.0

    patch_quality(lambda px: (QUALITY_OK, 0, "ok"))
    try:
        for _ in range(3):
            feed(w, tilted_line_frame(deg=10))
    finally:
        camera_widget.classify_line_quality = original

    assert w.heading_deg > 0, f"航向符号错了: {w.heading_deg}"
    assert w.param_trim > 0, f"纠偏方向错了: {w.param_trim}"

    trims = frames_of(s, frame.CMD_TRIM)
    assert trims, "没发纠偏帧"
    value = (trims[0][3] << 8) | trims[0][4]
    if value >= 0x8000:
        value -= 0x10000
    assert value > 0


def test_pump_needs_sustained_damage_to_turn_on():
    """泵的滞回 + 投票：单帧破损绝不开泵。

    先把窗口喂成 5 帧"完好"，再插一帧破损 —— 投票里 4:1，结论还是完好。
    这正是现场最常见的那个场景：走得好好的，一帧反光糊住镜头。
    """
    original = camera_widget.classify_line_quality
    s = FakeSerial()
    w = camera_widget.CameraWidget(serial_mgr=s, mode="track")
    w.start_track()
    w.TRIM_INTERVAL_S = 1000.0

    patch_quality(lambda px: (QUALITY_OK, 0, "ok"))
    try:
        for _ in range(w.VOTE_WINDOW):
            feed(w, line_frame())
        assert w.pump_on is False

        patch_quality(lambda px: (QUALITY_BREAK, 4, "break"))
        feed(w, line_frame())
        assert w.pump_on is False, "一帧就开泵了"

        # 3 帧之后投票里破损占多数，才允许开
        for _ in range(w.VOTE_MAJORITY):
            feed(w, line_frame())
        assert w.pump_on is True, "连续多帧破损还没开泵"
    finally:
        camera_widget.classify_line_quality = original

    assert frames_of(s, frame.CMD_PUMP)


def test_unsure_keeps_pump_off():
    """看不清就不喷。镜头糊了、车开出线外了，继续喷是在空地上倒漆。"""
    original = camera_widget.classify_line_quality
    s = FakeSerial()
    w = camera_widget.CameraWidget(serial_mgr=s, mode="track")
    w.start_track()
    w.TRIM_INTERVAL_S = 1000.0

    patch_quality(lambda px: (QUALITY_UNSURE, 0, "No line"))
    try:
        for _ in range(10):
            feed(w, line_frame())
    finally:
        camera_widget.classify_line_quality = original

    assert w.pump_on is False, "看不清的时候泵开了"
    assert w.quality == QUALITY_UNSURE


def test_lost_line_turns_pump_off():
    """线突然看不见了（比如开出标线区域），泵必须在几帧内关掉。"""
    original = camera_widget.classify_line_quality
    s = FakeSerial()
    w = camera_widget.CameraWidget(serial_mgr=s, mode="track")
    w.start_track()
    w.TRIM_INTERVAL_S = 1000.0

    patch_quality(lambda px: (QUALITY_BREAK, 4, "break"))
    try:
        for _ in range(w.VOTE_WINDOW):
            feed(w, line_frame())
        assert w.pump_on is True, "持续破损没开泵"

        # 之后画面全黑，连判据都调不到（走的是 found_line=False 那条路）
        camera_widget.classify_line_quality = original
        for _ in range(w.PUMP_OFF_AFTER + w.VOTE_WINDOW):
            feed(w, blank_frame())
    finally:
        camera_widget.classify_line_quality = original

    assert w.pump_on is False, "线丢了泵还开着"


def test_on_status_sends_next_segment():
    """一段走完要接下一段，否则车走 5 米就停在原地，界面还显示在巡线。"""
    s = FakeSerial()
    w = camera_widget.CameraWidget(serial_mgr=s, mode="track")
    w.start_track()

    before = len(frames_of(s, frame.CMD_MOVE))

    w.on_status("M")                # 先看到车真的在走
    w._last_segment_time = 0.0      # 假装上一段是很久以前发的
    w.on_status("W")

    after = len(frames_of(s, frame.CMD_MOVE))
    assert after == before + 1, "段结束后没接下一段"


def test_on_status_ignores_early_return():
    """上一段刚发出去就"结束"了 —— 那是被别的东西打断，不该接着发。"""
    s = FakeSerial()
    w = camera_widget.CameraWidget(serial_mgr=s, mode="track")
    w.start_track()

    before = len(frames_of(s, frame.CMD_MOVE))

    w.on_status("M")
    w.on_status("W")                # 刚发完，时间闸会挡住

    assert len(frames_of(s, frame.CMD_MOVE)) == before


def test_on_status_does_not_resend_when_move_was_rejected():
    """MOVE 被回绝时状态一直是 W，不能每隔半秒就重发一帧。

    这是在"上电静默"和"状态不对"两种情况下都会发生的：命令被回绝，
    状态机没动，还是 W。只看"现在是 W"就续段的话，日志会被 MOVE 刷满，
    而真正的原因（一条 ERR_STATE）埋在中间。
    """
    s = FakeSerial()
    w = camera_widget.CameraWidget(serial_mgr=s, mode="track")
    w.start_track()

    before = len(frames_of(s, frame.CMD_MOVE))

    w._last_segment_time = 0.0
    for _ in range(10):
        w.on_status("W")            # 从头到尾没走起来

    assert len(frames_of(s, frame.CMD_MOVE)) == before, "被回绝的命令在无限重发"


if __name__ == "__main__":
    for name in sorted(n for n in globals() if n.startswith("test_")):
        globals()[name]()
        print("ok", name)
    print("全部通过")
