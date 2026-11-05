"""主窗口的航向显示和陀螺仪开关。

用 Qt 的 offscreen 平台跑，不需要真的弹窗口 —— 树莓派上做部署自检时
一样能跑通。

为什么这几条值得单独测：它们全是"界面看着对、实际没生效"的高发区。
勾选框点了但固件回绝、航向显示了一个没换算单位的原值、关的时候发成了
开的子命令 —— 这三种都不会报错，只会让人怀疑是标定不准。
"""

import os
import sys

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

try:
    from PyQt5.QtWidgets import QApplication
    from PyQt5.QtCore import QObject, pyqtSignal
except ImportError:                                    # pragma: no cover
    print("skip: 这台机器没装 PyQt5，跳过界面测试")
    raise SystemExit(0)

from core import frame
from core.protocol import Protocol
from ui.main_window import MainWindow

# 一个进程只能有一个 QApplication
_app = QApplication.instance() or QApplication([])


class FakeSerial(QObject):
    """只记录发出去的字节，不碰真串口。"""

    data_received = pyqtSignal(object)
    error_signal = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.sent = []

    def open(self):
        pass

    def send(self, data):
        self.sent.append(bytes(data))


def make_window():
    return MainWindow(FakeSerial(), Protocol())


def status_frame(state_char="M", step=1, flags=0, pulse=0):
    return frame.build_frame(
        frame.CMD_STATUS, 0,
        bytes([ord(state_char), step, flags, (pulse >> 8) & 0xFF, pulse & 0xFF]),
    )


def heading_frame(deg, rate=0.0, state=3, errors=0):
    deg100 = int(round(deg * 100))
    dps10 = int(round(rate * 10))
    return frame.build_frame(
        frame.CMD_HEADING, 0,
        bytes([
            (deg100 >> 8) & 0xFF, deg100 & 0xFF,
            (dps10 >> 8) & 0xFF, dps10 & 0xFF,
            ((state & 0x0F) << 4) | (errors & 0x0F),
        ]),
    )


def test_heading_label_shows_signed_degrees():
    w = make_window()

    w.on_data_received(heading_frame(+3.25))
    assert "航向 +3.25" in w.heading_label.text(), w.heading_label.text()

    w.on_data_received(heading_frame(-12.5))
    assert "航向 -12.50" in w.heading_label.text(), w.heading_label.text()


def test_heading_color_warns_when_far_off():
    """2 度以内绿、5 度以内橙、再往外红。

    这一条不是为了好看：现场调试时人盯的是这行数字，颜色是
    "现在还能不能继续"的唯一即时反馈。
    """
    w = make_window()

    w.on_data_received(heading_frame(1.0))
    assert "#008000" in w.heading_label.styleSheet()

    w.on_data_received(heading_frame(3.0))
    assert "#c07000" in w.heading_label.styleSheet()

    w.on_data_received(heading_frame(8.0))
    assert "#c00000" in w.heading_label.styleSheet()


def test_checkbox_follows_firmware_not_clicks():
    """勾选状态必须由状态帧决定。

    用户点了"开"，但固件回绝（没标零）时，勾必须弹回去 ——
    否则界面显示"已接管"，车其实一点没纠。
    """
    w = make_window()
    assert not w.gyro_check.isEnabled(), "还没收到状态帧就能点"

    # 陀螺仪就绪 + 闭环中
    w.on_data_received(status_frame(flags=0x08 | 0x10))
    assert w.gyro_check.isEnabled()
    assert w.gyro_check.isChecked()
    assert "闭环" in w.gyro_label.text()

    # 就绪但闭环关着
    w.on_data_received(status_frame(flags=0x08))
    assert w.gyro_check.isEnabled()
    assert not w.gyro_check.isChecked()
    assert "就绪" in w.gyro_label.text()

    # 未就绪
    w.on_data_received(status_frame(flags=0x00))
    assert not w.gyro_check.isEnabled()
    assert not w.gyro_check.isChecked()
    assert "未就绪" in w.gyro_label.text()


def test_toggle_sends_the_right_subcommand():
    w = make_window()

    w.on_gyro_toggle(True)
    w.on_gyro_toggle(False)

    assert len(w.serial_mgr.sent) == 2
    on, off = w.serial_mgr.sent

    assert frame.check_frame(on)
    assert on[1] == frame.CMD_GYRO
    assert on[3] == frame.GYRO_ON

    assert frame.check_frame(off)
    assert off[1] == frame.CMD_GYRO
    assert off[3] == frame.GYRO_OFF


def test_measure_button_sends_measure_and_warns():
    """标轮距会让车自己动，按钮必须同时给出提示。"""
    w = make_window()

    w.on_gyro_measure()

    sent = w.serial_mgr.sent
    assert len(sent) == 1
    assert sent[0][1] == frame.CMD_GYRO
    assert sent[0][3] == frame.GYRO_MEASURE

    # 发送日志里要有一句"车会动"的提示，不能只有一串 hex
    log = w.send_log.toPlainText()
    assert "陀螺仪 标定轮距" in log, log


def test_recal_button_sends_recal():
    w = make_window()
    w.on_gyro_recal()

    sent = w.serial_mgr.sent
    assert len(sent) == 1
    assert sent[0][3] == frame.GYRO_RECAL
    assert "静止" in w.send_log.toPlainText()


def test_heading_frames_do_not_flood_the_log():
    """航向帧 5Hz，不能写进接收日志。

    写进去的话，按钮回显、ACK、错误码会在几秒内被冲干净 ——
    而那三样才是排障时真正要看的。
    """
    w = make_window()

    for _ in range(20):
        w.on_data_received(heading_frame(1.0))

    assert w.recv_log.toPlainText().strip() == ""
    assert "航向 +1.00" in w.heading_label.text()


def test_camera_widget_stops_trim_when_gyro_takes_over():
    """陀螺仪闭环一开，视觉那条线必须闭嘴。

    两条控制律改同一组目标脉冲，谁后到谁覆盖，表现是车周期性左右摆，
    而两边日志都是"已下发、已确认"—— 这是最难查的那种故障。
    """
    w = make_window()

    class Stub:
        tracking = True
        mode = "track"
        fw_flags = 0
        gyro_closed = False

        def on_status(self, state_char, pulse=0, flags=None):
            if flags is not None:
                self.fw_flags = flags
                self.gyro_closed = bool(flags & 0x10)

    stub = Stub()
    w.camera_widget = stub

    w.on_data_received(status_frame(flags=0x08 | 0x10))
    assert stub.gyro_closed is True

    w.on_data_received(status_frame(flags=0x08))
    assert stub.gyro_closed is False

    w.camera_widget = None


if __name__ == "__main__":
    for name in sorted(n for n in globals() if n.startswith("test_")):
        globals()[name]()
        print("ok", name)
    print("全部通过")
