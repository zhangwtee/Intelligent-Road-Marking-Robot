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
        return True         # 和真实 SerialManager.send 一样回报成功与否


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


def speed_info_frame(arr, saved=True):
    """固件上行 0x84：当前档位 + 有没有写进 flash。"""
    return frame.build_frame(
        frame.CMD_SPEED_INFO, 0,
        bytes([(arr >> 8) & 0xFF, arr & 0xFF,
               frame.SPEED_FLAG_SAVED if saved else 0]),
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

    # 陀螺仪就绪 + 闭环中。文字由航向帧写，所以状态帧之后还得来一帧真实的
    # 航向数据 —— 只发状态帧的话这行字根本不该变（见下一个用例）。
    w.on_data_received(status_frame(flags=0x08 | 0x10))
    w.on_data_received(heading_frame(1.0))
    assert w.gyro_check.isEnabled()
    assert w.gyro_check.isChecked()
    assert "闭环" in w.gyro_label.text()

    # 就绪但闭环关着
    w.on_data_received(status_frame(flags=0x08))
    w.on_data_received(heading_frame(1.0))
    assert w.gyro_check.isEnabled()
    assert not w.gyro_check.isChecked()
    assert "仅监视" in w.gyro_label.text()

    # 未就绪：没有航向帧可等，状态帧必须自己给出原因
    w.on_data_received(status_frame(flags=0x00))
    assert not w.gyro_check.isEnabled()
    assert not w.gyro_check.isChecked()
    assert "未就绪" in w.gyro_label.text()


def test_gyro_label_does_not_flicker():
    """状态帧和航向帧都是 5Hz，抢着写同一行字就会每秒跳五次。

    就绪之后这行只归航向帧管，状态帧只更新勾选框和颜色依据。
    """
    w = make_window()
    w.on_data_received(status_frame(flags=0x08 | 0x10))
    w.on_data_received(heading_frame(2.0, rate=0.5))
    first = w.gyro_label.text()
    assert "闭环中" in first, first

    for _ in range(5):
        w.on_data_received(status_frame(flags=0x08 | 0x10))
        assert w.gyro_label.text() == first, "状态帧把航向帧写的文字冲掉了"
        w.on_data_received(heading_frame(2.0, rate=0.5))
        assert w.gyro_label.text() == first


def test_gyro_error_flag_is_shown():
    """bit 0x20 是固件特意加的"这一局读错过"，上位机不能吞掉。"""
    w = make_window()
    w.on_data_received(status_frame(flags=0x08 | 0x20))
    w.on_data_received(heading_frame(0.0))
    assert "读错过" in w.gyro_label.text(), w.gyro_label.text()
    assert "#c07000" in w.gyro_label.styleSheet()


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


def test_paint_button_sends_the_size_from_the_boxes():
    """画车位要带尺寸走帧，不能只发单字节 'P'。

    单字节 'P' 只能用固件里烧死的 5300x2500，现场换个规格就得重烧 ——
    这个按钮存在的全部理由就是这两个数能改，所以发的帧里必须是框里的值。
    """
    w = make_window()

    w.paint_len_spin.setValue(5000)
    w.paint_wid_spin.setValue(2300)
    w.send_paint()

    sent = w.serial_mgr.sent
    assert len(sent) == 1, sent
    assert frame.check_frame(sent[0]), sent[0].hex()
    assert sent[0][1] == frame.CMD_PAINT

    length = (sent[0][3] << 8) | sent[0][4]
    width = (sent[0][5] << 8) | sent[0][6]
    assert (length, width) == (5000, 2300), (length, width)


def test_paint_size_defaults_to_the_firmware_default():
    """什么都不改就按下去，画出来得和以前的 'P' 一样大。"""
    w = make_window()

    assert w.paint_len_spin.value() == frame.PAINT_LEN_DEFAULT_MM
    assert w.paint_wid_spin.value() == frame.PAINT_WID_DEFAULT_MM


def test_paint_size_cannot_leave_the_firmware_range():
    """越界的数发出去，固件回 ERR_RANGE，车一动不动只剩一个结果码。

    输入框自己就该关住，别让人有机会发出一个注定被拒的尺寸。
    """
    w = make_window()

    w.paint_len_spin.setValue(10)
    assert w.paint_len_spin.value() == frame.PAINT_MIN_MM

    w.paint_len_spin.setValue(99999)
    assert w.paint_len_spin.value() == frame.PAINT_MAX_MM

    w.paint_wid_spin.setValue(1)
    assert w.paint_wid_spin.value() == frame.PAINT_MIN_MM


def test_paint_cancels_the_vision_loop_before_it_starts():
    """先撤视觉再画车位。

    视觉那条线按 10Hz 发 MOVE / TRIM，抢在画车位前面到达时固件回
    ERR_STATE —— 画不成，日志里只有一个结果码，看着像链路问题。
    """
    w = make_window()

    class Stub:
        tracking = True
        mode = "track"
        stopped = False

        def stop_track(self):
            self.stopped = True

        def on_status(self, *a, **k):
            pass

    stub = Stub()
    w.camera_widget = stub
    w.btn_track.setChecked(True)

    # 记下"发帧那一刻视觉停了没有"——只看最后有没有停，顺序错了也照样通过。
    seen_at_send = []
    send = w.serial_mgr.send

    def spy(data):
        seen_at_send.append(stub.stopped)
        send(data)

    w.serial_mgr.send = spy

    w.send_paint()

    assert seen_at_send == [True], "视觉还在跑就发了画车位"
    assert w.btn_track.isChecked() is False

    w.camera_widget = None


def test_paint_heading_flag_rides_with_the_size():
    """纠偏开关必须和尺寸在同一帧里，默认关。

    做成单独的"开关"命令的话，两条之间被打断就变成「开关改了、这次画车位
    没赶上」—— 而日志里两条都发了，看不出哪次生效。
    """
    w = make_window()
    assert w.paint_heading_check.isChecked() is False, "纠偏不该默认打开"

    w.paint_heading_check.setChecked(True)
    w.send_paint()

    sent = w.serial_mgr.sent
    assert len(sent) == 1, sent
    assert sent[0][1] == frame.CMD_PAINT
    assert sent[0][7] & frame.PAINT_FLAG_HEADING, "DATA[4] 的纠偏位没置上"
    assert "纠偏" in w.send_log.toPlainText()


def test_speed_slider_sends_the_arr():
    """滑条的值就是固件的 ARR，大端塞进 DATA[0:2]。"""
    w = make_window()

    w.speed_slider.setValue(600)

    sent = w.serial_mgr.sent
    assert len(sent) == 1, sent
    assert frame.check_frame(sent[0]), sent[0].hex()
    assert sent[0][1] == frame.CMD_SPEED
    assert ((sent[0][3] << 8) | sent[0][4]) == 600
    assert w.speed_label.text() == "600"


def test_speed_slider_does_not_flood_while_dragging():
    """拖动过程中一帧都不发，松手才发最后那个值。

    QSlider 每移一个像素就 valueChanged 一次，而 9600 下一帧 10.4ms ——
    不挡的话拖一下就是几十帧，ACK 和错误码全被冲掉。
    """
    w = make_window()
    w.speed_slider.isSliderDown = lambda: True        # 假装正按着鼠标

    for arr in (250, 240, 230, 220):
        w.speed_slider.setValue(arr)

    assert w.serial_mgr.sent == [], "拖动过程中就发了帧"
    assert w.speed_label.text() == "220", "拖动时标签也得跟着走"

    w.on_speed_released()
    assert len(w.serial_mgr.sent) == 1
    assert ((w.serial_mgr.sent[0][3] << 8) | w.serial_mgr.sent[0][4]) == 220


def test_speed_retries_after_a_dropped_frame():
    """串口没写出去的时候，不能记成"已发"。

    记了的话，之后拖回同一个值会被"值没变"挡掉：界面上写着 600，车还
    按旧档跑，人拿这个数字判断漆量。
    """
    w = make_window()

    tried = []
    w.serial_mgr.send = lambda data: (tried.append(bytes(data)), False)[1]
    w.speed_slider.setValue(600)
    assert len(tried) == 1, "滑条动了却没尝试发送"

    # 串口恢复，同一个值再松一次手就该真的出去 —— 不能被"值没变"吃掉
    w.serial_mgr.send = lambda data: (tried.append(bytes(data)), True)[1]
    w.on_speed_released()
    assert len(tried) == 2, "串口恢复后没有补发"


def test_speed_is_not_resent_when_unchanged():
    """值没变就别发 —— 点一下滑条也会触发 sliderReleased。"""
    w = make_window()

    w.speed_slider.setValue(600)
    w.on_speed_released()
    w.on_speed_released()

    assert len(w.serial_mgr.sent) == 1, w.serial_mgr.sent


def test_speed_info_moves_the_slider_without_sending():
    """固件报回的速度要回填到滑条上，而且不能因此再发一帧。

    回填是这条帧存在的全部理由：手机蓝牙按 +/- 改完，树莓派面板上还写着老值，
    人会按那个数字判断漆量和拐角冲击。而回填时若顺手触发一次 valueChanged，
    就成了"收到 700、回发 700、再收到 700"的自激。
    """
    w = make_window()

    w.on_data_received(speed_info_frame(700))

    assert w.speed_slider.value() == 700, w.speed_slider.value()
    assert w.speed_label.text() == "700"
    assert w.serial_mgr.sent == [], "回填滑条时又发了一帧"


def test_speed_info_does_not_fight_the_drag():
    """手指按在滑条上时，固件报的值不能把滑条拽回去。

    拖动中的每一帧都是"过去的值"（固件还没收到我们最新的那帧），
    拿它回填等于和用户抢控件：滑条会一跳一跳。
    """
    w = make_window()
    w.speed_slider.isSliderDown = lambda: True     # 假装正按着鼠标

    w.speed_slider.setValue(250)
    w.on_data_received(speed_info_frame(900))

    assert w.speed_slider.value() == 250, "拖动中被固件值拽走了"


def test_speed_label_marks_the_value_that_is_not_in_flash():
    """没写进 flash 的值要看得出来，否则"断电还在"是个默认假设。"""
    w = make_window()

    w.on_data_received(speed_info_frame(600, saved=False))
    assert "#b06000" in w.speed_label.styleSheet(), "未保存没有标记"

    w.on_data_received(speed_info_frame(600, saved=True))
    assert "#b06000" not in w.speed_label.styleSheet(), "已保存还留着橙色"


def test_speed_info_right_after_sending_is_ignored():
    """刚发出去的那一两帧旧值不能把滑条拽回去。

    速度帧 1Hz，我们发完到固件处理完之间，上行的可能还是旧值 ——
    不挡的话松手瞬间滑条会"跳回旧值、再跳回来"，看着像没生效。
    过了窗口之后要认账：固件把值夹到边界（蓝牙按 + 停在 150）就是这么显出来的。
    """
    w = make_window()

    w.speed_slider.setValue(400)
    assert len(w.serial_mgr.sent) == 1, w.serial_mgr.sent

    w.on_data_received(speed_info_frame(300))        # 还带着旧值
    assert w.speed_slider.value() == 400, "被在途的旧帧拽回去了"

    w._speed_sent_at -= 1.0                          # 假装窗口已经过去
    w.on_data_received(speed_info_frame(150))        # 固件限幅的结果
    assert w.speed_slider.value() == 150, "限幅值没有回填"


def test_speed_info_is_not_written_to_the_log():
    """速度帧 1Hz，同样不能进接收日志。"""
    w = make_window()

    for _ in range(5):
        w.on_data_received(speed_info_frame(320))

    assert w.recv_log.toPlainText().strip() == "", w.recv_log.toPlainText()
    assert w.speed_slider.value() == 320


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


def test_estop_is_outside_the_scroll_area():
    """急停必须常驻可见，不能藏在滚动区里。

    左栏内容已经比树莓派那块屏高，放进可滚区就意味着现场出事时第一个
    动作是"先把左栏滚下去"，而滚动条只有十几像素宽。这类按钮不该有
    "得先找到它"这个前提 —— 所以它固定在滚动区外面。
    """
    from PyQt5.QtWidgets import QScrollArea

    w = make_window()
    w.resize(1000, 560)
    w.show()

    assert w.btn_estop.isVisible(), "急停没显示出来"

    scroll = w.findChild(QScrollArea)
    assert scroll is not None, "左栏的滚动区没了？"
    assert not scroll.widget().isAncestorOf(w.btn_estop), "急停被塞回滚动区里了"

    # 而且在窗口范围内：不能被挤到窗口下沿之外
    bottom = w.btn_estop.mapTo(w, w.btn_estop.rect().bottomLeft()).y()
    assert bottom <= w.height(), f"急停超出窗口下沿: {bottom} > {w.height()}"


def test_resume_button_is_dead_until_firmware_reports_a_breakpoint():
    """续画按钮只有在固件说有断点时才亮。

    断点有没有只有固件知道：停在转弯上不记、剩不到 50mm 不记、按了 W
    会清掉。上位机自己记"刚才发过急停"的话，一半情况下按钮亮着却续不上 ——
    那是最糟的交互：人以为在续画，车一动没动。
    """
    w = make_window()

    assert w.btn_resume.isEnabled() is False, "一上电就亮了"

    w.on_data_received(status_frame(state_char="T", flags=0x40))
    assert w.btn_resume.isEnabled() is True

    # 按「停止复位」：固件清断点，0x40 掉了，按钮跟着灭
    w.on_data_received(status_frame(state_char="W", flags=0x00))
    assert w.btn_resume.isEnabled() is False


def test_resume_sends_the_resume_flag_not_a_size():
    """续画帧带 bit1，长宽留 0。

    带尺寸的话固件虽然不采纳，但日志里会显示一个"这次的尺寸"，
    和实际在走的那一圈对不上 —— 排查时会以为尺寸被改了。
    """
    w = make_window()

    w.paint_len_spin.setValue(6000)
    w.paint_wid_spin.setValue(3000)
    w.send_resume()

    sent = w.serial_mgr.sent
    assert len(sent) == 1, sent
    assert frame.check_frame(sent[0])
    assert sent[0][1] == frame.CMD_PAINT

    flags = sent[0][7]
    assert flags & frame.PAINT_FLAG_RESUME, flags

    length = (sent[0][3] << 8) | sent[0][4]
    width = (sent[0][5] << 8) | sent[0][6]
    assert (length, width) == (0, 0), "续画不该带尺寸"


def test_resume_and_heading_flags_do_not_collide():
    """续画带纠偏是合法的（断点那条边本来就在纠），两个位要能同时置。"""
    both = frame.build_paint(0, 0, heading=True, resume=True)
    assert both[7] == frame.PAINT_FLAG_HEADING | frame.PAINT_FLAG_RESUME

    assert frame.build_paint(heading=True)[7] == frame.PAINT_FLAG_HEADING
    assert frame.build_paint(resume=True)[7] == frame.PAINT_FLAG_RESUME
    assert frame.build_paint()[7] == 0


def test_status_bar_shows_which_edge_is_being_painted():
    """状态帧的 DATA[1] 是画车位子状态，固件一直在发，以前没人用。

    人在树莓派前面，没有这一行就得扭头看车上那块 OLED 才知道画到第几条边。
    """
    w = make_window()

    w.on_data_received(status_frame(state_char="P", step=5))
    assert "第3条边" in w.paint_label.text(), w.paint_label.text()

    w.on_data_received(status_frame(state_char="P", step=6))
    assert "转弯3" in w.paint_label.text(), w.paint_label.text()

    # 别的状态还按原来的方式显示，别把段号硬套到所有状态上
    w.on_data_received(status_frame(state_char="M", step=0))
    assert "第" not in w.paint_label.text(), w.paint_label.text()


def test_paint_progress_is_not_overwritten_by_a_command_echo():
    """画线期间来一条 ACK，不能把"第几条边"盖成"状态: OK"。

    进度和"最后按了哪个键"挤同一个 label 的时候，就是这样闪的：ACK 先盖掉
    进度，200ms 后 5Hz 的状态帧再盖回来。画线时命令很少，所以现场偶尔才
    看到闪一下 —— 而闪掉的那一行正好是画线时最需要看的。
    """
    w = make_window()

    w.on_data_received(status_frame(state_char="P", step=5))
    w.on_data_received(frame.build_frame(frame.CMD_ACK, 0, bytes([0x10, 0])))

    assert "第3条边" in w.paint_label.text(), w.paint_label.text()
    assert "OK" in w.status_label.text(), w.status_label.text()


def battery_frame(mv, valid=True, low=False, absent=False):
    flags = 0
    if valid:
        flags |= frame.BATT_FLAG_VALID
    if low:
        flags |= frame.BATT_FLAG_LOW
    if absent:
        flags |= frame.BATT_FLAG_ABSENT
    return frame.build_frame(
        frame.CMD_BATTERY, 0,
        bytes([(mv >> 8) & 0xFF, mv & 0xFF, flags, 0, 0]),
    )


def test_battery_label_shows_voltage_and_turns_red():
    """电压要显示成 12.5V；低压变红；采样无效显示 --- 而不是 0.0V。

    显示 0.0V 会把人的注意力引到"电池没电"上，而分压线掉了的时候电池
    其实是满的。现场被支到错误的方向，比没有这个读数更贵。
    """
    w = make_window()

    w.on_data_received(battery_frame(12500))
    assert "12.5V" in w.battery_label.text(), w.battery_label.text()
    assert "#c00000" not in w.battery_label.styleSheet(), "正常电压不该是红的"

    w.on_data_received(battery_frame(11400, low=True))
    assert "11.4V" in w.battery_label.text(), w.battery_label.text()
    assert "#c00000" in w.battery_label.styleSheet(), "低压没有变红"

    w.on_data_received(battery_frame(0, valid=False))
    assert "---" in w.battery_label.text(), w.battery_label.text()
    assert "0.0V" not in w.battery_label.text(), "分压线断了却显示 0.0V"

    # 分压电阻没焊是另一回事：固件读到了数，但那不是电池。界面要说"未接"，
    # 不能和上面那条"这一轮没读到"混成一句话 —— 前者要去焊电阻，后者等一轮就好。
    w.on_data_received(battery_frame(0, valid=False, absent=True))
    assert "未接" in w.battery_label.text(), w.battery_label.text()
    assert "0.0V" not in w.battery_label.text(), "没接分压却显示了一个电压"


def test_battery_frame_stays_out_of_the_log():
    """1Hz 的电池帧进日志会把 ACK 和错误码冲掉，和速度帧一个道理。"""
    w = make_window()
    before = w.recv_log.document().blockCount()

    for _ in range(5):
        w.on_data_received(battery_frame(12500))

    assert w.recv_log.document().blockCount() == before, "电池帧刷进接收日志了"


def resume_info_frame(length, width, step):
    """固件上行 0x86：flash 里那份断点的长宽和边号。"""
    return frame.build_frame(
        frame.CMD_RESUME_INFO, 0,
        bytes([(length >> 8) & 0xFF, length & 0xFF,
               (width >> 8) & 0xFF, width & 0xFF, step]),
    )


def test_resume_label_shows_what_the_firmware_remembers():
    """断点行显示固件 flash 里那一圈的长宽，而且不许动输入框。

    这一行是为“核对不了”准备的：上电之后固件已经把断点的尺寸还原了，
    续画按它走，但以前面板上没有任何地方能看到那个数。

    反过来说，尺寸也不能自动填进输入框 —— 那两个框是“我打算画多大”，
    而续画只认断点。填进去等于告诉人“改了尺寸就按新的画”，实际改了也
    白改、车还是照断点走。
    """
    w = make_window()
    w.paint_len_spin.setValue(5000)
    w.paint_wid_spin.setValue(2300)

    w.on_data_received(resume_info_frame(5300, 2500, 5))
    text = w.resume_label.text()
    assert "5300x2500" in text, text
    assert "第3条边" in text, text
    assert w.paint_len_spin.value() == 5000, "断点把长边输入框改掉了"
    assert w.paint_wid_spin.value() == 2300, "断点把宽边输入框改掉了"

    w.on_data_received(resume_info_frame(0, 0, 0))
    assert "无" in w.resume_label.text(), w.resume_label.text()


def test_resume_info_frame_stays_out_of_the_log():
    """1Hz 的断点帧进日志会把 ACK 和错误码冲掉，和速度帧一个道理。"""
    w = make_window()
    before = w.recv_log.document().blockCount()

    for _ in range(5):
        w.on_data_received(resume_info_frame(5300, 2500, 5))

    assert w.recv_log.document().blockCount() == before, "断点帧刷进接收日志了"


if __name__ == "__main__":
    for name in sorted(n for n in globals() if n.startswith("test_")):
        globals()[name]()
        print("ok", name)
    print("全部通过")
