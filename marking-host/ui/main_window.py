import time

import cv2
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QTextEdit, QLabel,
    QComboBox, QCheckBox, QSlider, QStackedLayout, QSpacerItem, QSizePolicy,
    QSpinBox, QScrollArea
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QTextCursor

from ui.camera_widget import CameraWidget
from core import frame
from core.ack import AckTracker
from core.protocol import (
    PANEL_BUTTONS, CAMERA_BUTTONS, CAMERA_OFF_BUTTONS, PAINT_BUTTON, ESTOP_BUTTON
)

# 发出 CMD_SPEED 之后，多久开始相信速度帧。
#
# 我们自己那帧发出去到固件生效之间，上行里可能还有一两帧是旧值（速度帧
# 1Hz，最坏要 1 秒才刷新）。不挡的话松手瞬间滑条会先跳回旧值、再跳回来。
# 0.5 秒是折中：够固件把帧收完，又不至于把手机上改的值挡太久。
SPEED_SETTLE_S = 0.5


class MainWindow(QWidget):
    append_send_signal = pyqtSignal(str)
    append_recv_signal = pyqtSignal(object)

    def __init__(self, serial_mgr, protocol):
        super().__init__()
        self.serial_mgr = serial_mgr
        self.protocol = protocol

        # ACK 配对表：改造前 ACK 只进日志，人眼看着"有一个 ACK"，
        # 但分不清"链路断了"和"命令被回绝"。现在按 SEQ 配起来。
        self.ack = AckTracker()

        # 下行帧序号。固件按 (seq, cmd) 做 1 秒去重，而且只比这两个字段、
        # 不比内容 —— 序号恒为 0 的时候，同一类命令 1 秒内发第二次会被当成
        # 重传，固件直接回上一次的结果，命令根本没执行。ACK 表也以 seq 为键，
        # 恒为 0 等于永远只有一条在飞。
        self._seq = 0

        self.append_send_signal.connect(self._append_send)
        self.append_recv_signal.connect(self._append_recv)

        self.init_ui()

        self.serial_mgr.data_received.connect(self.on_data_received)
        # error_signal 原来一个槽都没接：串口没打开时 send() 是静默失败，
        # 发送日志照记、摄像头上的计数照涨，看着像在干活，
        # 其实一个字节都没上总线。
        self.serial_mgr.error_signal.connect(self.on_serial_error)
        self.serial_mgr.open()

        # 超时检查。100ms 一次：ACK_TIMEOUT_S 是 300ms，检查得比它密，
        # 才不会出现"其实 200ms 就超了，但要等到 400ms 才被发现"。
        self.tick_timer = QTimer()
        self.tick_timer.timeout.connect(self._tick)
        self.tick_timer.start(100)

    def on_serial_error(self, msg):
        self.append_send_signal.emit(f"串口错误: {msg} —— 字节没发出去")

    # ---------------- 链路健康 ----------------
    def _tick(self):
        dead, _ = self.ack.expire(resend=self.serial_mgr.send)

        for item in dead:
            name = frame.CMD_NAMES.get(item.cmd, f"0x{item.cmd:02X}")
            self.append_send_signal.emit(
                f"无响应 {name} seq={item.seq}（300ms 内没收到 ACK）"
            )

        self._update_link_label()

    def _update_link_label(self):
        h = self.ack.health()

        if h["sent"] == 0:
            self.link_label.setText("链路：还没发过帧")
            self.link_label.setStyleSheet("")
            return

        self.link_label.setText(
            f"链路  确认 {h['acked']}/{h['sent']}   超时 {h['timed_out']}   "
            f"回绝 {h['rejected']}   RTT {h['rtt']}"
        )

        if h["rejected"]:
            # 链路是通的，命令被下位机回绝了 —— 这是逻辑问题，不是线的问题，
            # 用橙色而不是红色，免得跟"断线"混在一起。
            color = "#c07000"
        elif h["loss_pct"] >= 30:
            color = "#c00000"
        elif h["loss_pct"] > 0:
            color = "#c07000"
        else:
            color = "#008000"

        self.link_label.setStyleSheet(f"color:{color}; font-size:14px;")

    # ---------------- 帧发送 ----------------
    def _next_seq(self):
        """下一个下行序号，0~255 循环。"""
        self._seq = (self._seq + 1) & 0xFF
        return self._seq

    def send_frame(self, raw, label=""):
        """发一帧并登记 ACK。所有走帧协议的动作都要经过这里 ——
        绕过它发送，链路健康度就少了这一条，而且永远不会被发现。

        返回是否真的写进了串口。没写出去就不登记 ACK —— 登记了会在超时后
        由 expire 重发，而串口还是断的，白白多刷一轮。
        """
        ok = self.serial_mgr.send(raw)
        if ok:
            self.ack.register(raw)
        self.append_send_log(raw, label)
        self._update_link_label()
        return ok

    # ---------------- 车位尺寸输入 ----------------
    @staticmethod
    def _make_size_label(text):
        label = QLabel(text)
        label.setStyleSheet("font-size:16px;")
        return label

    @staticmethod
    def _make_size_spin(value):
        """范围直接卡固件的 PROTO_PAINT_MIN/MAX_MM。

        用 QSpinBox 而不是放行手输：超范围的数发出去，固件回 ERR_RANGE，
        车一动不动而日志里只有一个结果码。这里从一开始就输不进去越界值。
        """
        spin = QSpinBox()
        spin.setRange(frame.PAINT_MIN_MM, frame.PAINT_MAX_MM)
        spin.setSingleStep(100)
        spin.setSuffix(" mm")
        spin.setValue(value)
        spin.setMinimumWidth(100)
        spin.setToolTip(
            f"车位尺寸 {frame.PAINT_MIN_MM}~{frame.PAINT_MAX_MM}mm，"
            "按上下键一次 100mm。改完直接按下面的「画车位」。"
        )
        spin.setStyleSheet("QSpinBox { font-size:16px; min-height:32px; }")
        return spin

    def init_ui(self):
        self.resize(1000, 560)
        self.setWindowTitle("道路画线机控制面板")
        self.stack_layout = QStackedLayout()
        self.setLayout(self.stack_layout)

        # ---------------- 主界面 ----------------
        self.main_widget = QWidget()
        main_layout = QHBoxLayout()
        self.main_widget.setLayout(main_layout)

        # 左侧
        left_layout = QVBoxLayout()
        left_layout.setAlignment(Qt.AlignTop)

        self.status_label = QLabel("状态:\nwait")
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setStyleSheet("font-size:32px; font-weight:bold;")
        left_layout.addSpacerItem(QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Fixed))
        left_layout.addWidget(self.status_label)

        # 画车位进度单独一行。它和"最后按了哪个键"是两件事：共用一个 label
        # 的时候，画线期间来一条 ACK 就把进度盖成"状态: OK"，200ms 后状态帧
        # 再盖回来 —— 看着就是在闪，而且刚好盖住最需要看的那一行。
        self.paint_label = QLabel("")
        self.paint_label.setAlignment(Qt.AlignCenter)
        self.paint_label.setStyleSheet("font-size:22px; font-weight:bold; color:#1a7f37;")
        left_layout.addWidget(self.paint_label)

        self.link_label = QLabel("链路：还没发过帧")
        self.link_label.setAlignment(Qt.AlignCenter)
        left_layout.addWidget(self.link_label)

        # 航向。单独一行大字：它是"车到底直不直"的唯一直接读数，
        # 比状态栏里那串脉冲数有用得多。
        self.heading_label = QLabel("航向  --")
        self.heading_label.setAlignment(Qt.AlignCenter)
        self.heading_label.setStyleSheet("font-size:24px; font-weight:bold; color:#555;")
        left_layout.addWidget(self.heading_label)

        self.gyro_label = QLabel("陀螺仪：等状态帧")
        self.gyro_label.setAlignment(Qt.AlignCenter)
        self.gyro_label.setStyleSheet("font-size:14px; color:#777;")
        left_layout.addWidget(self.gyro_label)

        # 电池电压。放状态区而不是日志里：这是"还能不能干完这一趟"的判断
        # 依据，滚在日志里没人会回头翻。1Hz 的电池帧进了日志反而会把
        # ACK 和错误码冲干净（和速度帧同一个理由）。
        self.battery_label = QLabel("电池  --")
        self.battery_label.setAlignment(Qt.AlignCenter)
        self.battery_label.setStyleSheet("font-size:14px; color:#777;")
        left_layout.addWidget(self.battery_label)

        # 断点：固件 flash 里存着的那一圈长宽和第几条边。
        # 以前它只活在下位机里 —— 面板上的长宽输入框还是自己上次的值，
        # 人要核对"车上记的到底是多大"没有任何地方能看。续画本身不受
        # 影响（固件只认断点），所以这是可观测性缺口，不是功能缺陷。
        self.resume_label = QLabel("断点  --")
        self.resume_label.setAlignment(Qt.AlignCenter)
        self.resume_label.setStyleSheet("font-size:14px; color:#777;")
        left_layout.addWidget(self.resume_label)

        # 状态帧（5Hz）和航向帧（5Hz）都会想改上面这行字。两个源各写各的，
        # 界面就会每秒跳五次。约定：就绪之后这行只归航向帧管，状态帧只
        # 负责把这三个标记更新下来 —— 见 _apply_gyro_flags。
        self._gyro_ready = False      # bit 0x08：在线且已标零
        self._gyro_closed = False     # bit 0x10：下位机正在闭环
        self._gyro_err = False        # bit 0x20：这一局读错过（接线/干扰）

        # 闭环开关。勾选状态由下位机的状态帧回填，不是"点了就认为成功"——
        # 传感器没标零时固件会回绝，勾选必须弹回去，否则界面在骗人。
        self.gyro_check = QCheckBox("陀螺仪闭环接管直线")
        self.gyro_check.setStyleSheet("font-size:16px;")
        self.gyro_check.setEnabled(False)      # 没收到状态帧之前不知道能不能开
        self.gyro_check.toggled.connect(self.on_gyro_toggle)
        left_layout.addWidget(self.gyro_check)

        # ---------------- 车位尺寸 + 画车位 ----------------
        # 这两个数字以前只能改 system.c 的 PAINT_*_DEFAULT_MM 再重烧固件。
        # 现在面板直接发帧 CMD_PAINT，长宽随帧走：改一次尺寸不用动固件。
        # 范围取 frame 里的常量，那两个又是和固件 proto.h 对账过的
        # （protocol_test.test_frame_constants_match_firmware）。
        size_row = QHBoxLayout()
        size_row.setSpacing(6)

        size_row.addWidget(self._make_size_label("长"))
        self.paint_len_spin = self._make_size_spin(frame.PAINT_LEN_DEFAULT_MM)
        size_row.addWidget(self.paint_len_spin)

        size_row.addWidget(self._make_size_label("宽"))
        self.paint_wid_spin = self._make_size_spin(frame.PAINT_WID_DEFAULT_MM)
        size_row.addWidget(self.paint_wid_spin)

        size_row.addStretch(1)
        left_layout.addLayout(size_row)

        # ---------------- 速度 ----------------
        # 以前只有蓝牙的 '+'/'-' 能调速，上位机这边根本没有通路：用树莓派
        # 跑的时候速度只能靠手机改，或者烧死在一个档位。而速度直接决定画线
        # 质量（漆量、拐角冲击），这是现场最需要当场调的一个量。
        #
        # 滑条的值就是固件的 ARR（越小越快）。不换算成 mm/s 是因为那要经过
        # CALIB_STEPS_PER_REV，而这个数本身还没和驱动器 OLED 上的 MStep
        # 对过 —— 显示一个可能差一倍的速度比不显示更糟。
        speed_row = QHBoxLayout()
        speed_row.setSpacing(6)
        speed_row.addWidget(self._make_size_label("速度"))

        self.speed_slider = QSlider(Qt.Horizontal)
        self.speed_slider.setRange(frame.SPEED_MIN_ARR, frame.SPEED_MAX_ARR)
        self.speed_slider.setSingleStep(50)
        self.speed_slider.setPageStep(100)
        self.speed_slider.setValue(frame.SPEED_DEFAULT_ARR)
        # 反过来：往右拖 = ARR 变小 = 变快。跟"数值越小越快"的直觉一致，
        # 也跟手机上连按 '+' 的方向一致。
        self.speed_slider.setInvertedAppearance(True)
        self.speed_slider.setMinimumHeight(32)
        self.speed_slider.setToolTip(
            f"数值越小越快：{frame.SPEED_MIN_ARR}（最快）~ "
            f"{frame.SPEED_MAX_ARR}（最慢）。"
            "松手才发一帧 —— 拖动过程中每变一个值就发一帧会把串口刷满。\n"
            "滑条跟着固件回的速度帧（0x84）走：手机上按 +/- 改完，这里会自动跟过去，"
            "所以它显示的一直是车上正在跑的值，不是本界面最后一次发出去的值。\n"
            "数字变成橙色表示还没写进 flash（改完 2 秒、车停着才写），"
            "断电可能回到上一次存下的档位。"
        )
        self.speed_slider.valueChanged.connect(self.on_speed_changed)
        self.speed_slider.sliderReleased.connect(self.on_speed_released)
        speed_row.addWidget(self.speed_slider, 1)

        self.speed_label = QLabel(str(frame.SPEED_DEFAULT_ARR))
        self.speed_label.setMinimumWidth(46)
        self.speed_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.speed_label.setStyleSheet("font-size:16px; font-weight:bold;")
        speed_row.addWidget(self.speed_label)

        left_layout.addLayout(speed_row)
        self._speed_sent = None        # 上一次真的发出去的 ARR
        self._speed_sent_at = 0.0      # 上一次发 CMD_SPEED 的时刻

        self.btn_paint = QPushButton(PAINT_BUTTON)
        self.btn_paint.setMinimumHeight(40)
        self.btn_paint.setStyleSheet("font-size:20px;")
        self.btn_paint.clicked.connect(self.send_paint)
        left_layout.addWidget(self.btn_paint)

        # 画车位时要不要航向纠偏。不常开：纠偏要改左右目标脉冲，而画车位的
        # 边是匀速段（漆量才均匀），为了直要牺牲多少匀速由人决定。
        self.paint_heading_check = QCheckBox("画车位时用陀螺仪纠偏（四条边）")
        self.paint_heading_check.setStyleSheet("font-size:14px;")
        self.paint_heading_check.setToolTip(
            "四条边每段起点以当时车头为「直」，边走边纠。\n"
            "陀螺仪没标定时固件会回绝整条命令（ERR_HW），而不是照画不纠。\n"
            "角度的误差归 CHASSIS_TURN_SCALE 管，和这个开关无关。"
        )
        left_layout.addWidget(self.paint_heading_check)

        # 续画：急停停在某条边中途之后，从这里把这一圈走完。
        # 默认禁用，只有固件在状态帧里报了断点（flags 的 0x40）才亮。
        # 常亮按钮 + 命令回 ERR_STATE 是最糟的交互：人以为在续画，其实车
        # 一动没动，而画布上那条线看起来"就是断在这儿"。
        self.btn_resume = QPushButton("续画（从急停处接着画）")
        self.btn_resume.setMinimumHeight(36)
        self.btn_resume.setStyleSheet("font-size:16px;")
        self.btn_resume.setEnabled(False)
        self.btn_resume.setToolTip(
            "急停停在某条边中途时可用：只走那条边剩下的部分，"
            "已经画过的边不会重喷。\n"
            "停在转弯上不能续 —— 那不是少喷一段，是少转了 90 度，"
            "接上去的框是歪的，而且歪得像转角系数没标定。\n"
            "按「停止复位」视为这条活不干了，断点作废，这个按钮随之熄灭。"
        )
        self.btn_resume.clicked.connect(self.send_resume)
        left_layout.addWidget(self.btn_resume)

        # 按钮表在 core/protocol.py 里，和指令表放在一起，避免标签写两遍对不上。
        # 急停不在这批里 —— 它固定在滚动区外面，见下面。
        self.control_buttons = []
        for label, _cmd in PANEL_BUTTONS:
            if label == ESTOP_BUTTON:
                continue
            btn = QPushButton(label)
            btn.clicked.connect(self.send_button)
            btn.setMinimumHeight(40)
            btn.setStyleSheet("font-size:20px;")
            left_layout.addWidget(btn)
            self.control_buttons.append(btn)

        # 这一条不走单字节协议：它要连发几百帧（每段 MOVE + 10Hz 纠偏 + 泵开关），
        # 一个字节表达不了。做成开关式，按下去开始，再按一次停。
        self.btn_track = QPushButton("自动巡线喷涂（开始 / 停止）")
        self.btn_track.setCheckable(True)
        self.btn_track.setMinimumHeight(40)
        self.btn_track.setStyleSheet("font-size:20px; background:#dff0d8;")
        self.btn_track.clicked.connect(self.toggle_track)
        left_layout.addWidget(self.btn_track)

        self.btn_settings = QPushButton("设置")
        self.btn_settings.setMinimumHeight(40)
        self.btn_settings.setStyleSheet("font-size:20px;")
        self.btn_settings.clicked.connect(self.show_settings)
        left_layout.addWidget(self.btn_settings)

        # 左栏已经比 1000x560 高出一截（树莓派那块屏常常只有 600 高），
        # 套一层滚动区：装得下就看不见滚动条，装不下能滚 —— 总比把
        # 「设置」按钮顶出窗口外、点了没反应好。
        left_scroll = QScrollArea()
        left_scroll.setWidgetResizable(True)
        left_scroll.setFrameShape(QScrollArea.NoFrame)
        left_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        left_inner = QWidget()
        left_inner.setLayout(left_layout)
        left_scroll.setWidget(left_inner)

        # 急停固定在滚动区外面，任何窗口高度下都看得见。
        #
        # 左栏内容已经 792px，树莓派那块屏常常只有 600 高 —— 把它留在可滚
        # 区里，意味着现场出事时第一个动作是"先把左栏滚下去"，而滚动条本身
        # 只有十几像素宽。这类按钮不该有"得先找到它"这个前提。
        self.btn_estop = QPushButton(ESTOP_BUTTON)
        self.btn_estop.setMinimumHeight(56)
        self.btn_estop.setStyleSheet(
            "QPushButton { font-size:22px; font-weight:bold;"
            " background:#d9534f; color:white; border-radius:4px; }"
            "QPushButton:pressed { background:#c9302c; }"
        )
        self.btn_estop.setToolTip(
            "任何状态下都立即停车、关泵、断舵机，状态停在 TING。\n"
            "不会自动恢复：要接着干活按「停止复位」，要接着画这一圈按「续画」。"
        )
        self.btn_estop.clicked.connect(self.send_button)
        self.control_buttons.append(self.btn_estop)

        left_column = QVBoxLayout()
        left_column.addWidget(left_scroll, 1)
        left_column.addWidget(self.btn_estop)
        main_layout.addLayout(left_column, 1)

        # ---------------- 中间（发送日志） ----------------
        center_layout = QVBoxLayout()
        self.send_log_label = QLabel("发送日志")
        self.send_log = QTextEdit()
        self.send_log.setReadOnly(True)
        center_layout.addWidget(self.send_log_label)
        center_layout.addWidget(self.send_log, stretch=1)

        self.btn_clear_send = QPushButton("清空发送日志")
        self.btn_clear_send.clicked.connect(self.clear_send_log)
        center_layout.addWidget(self.btn_clear_send)

        self.send_pause_checkbox = QCheckBox("暂停自动滚动")
        center_layout.addWidget(self.send_pause_checkbox)

        main_layout.addLayout(center_layout, 1)

        # ---------------- 右侧（接收日志） ----------------
        right_layout = QVBoxLayout()
        self.recv_log_label = QLabel("接收日志")
        self.recv_log = QTextEdit()
        self.recv_log.setReadOnly(True)
        right_layout.addWidget(self.recv_log_label)
        right_layout.addWidget(self.recv_log, stretch=1)

        self.btn_clear_recv = QPushButton("清空接收日志")
        self.btn_clear_recv.clicked.connect(self.clear_recv_log)
        right_layout.addWidget(self.btn_clear_recv)

        self.recv_mode_box = QComboBox()
        self.recv_mode_box.addItems(["自动", "HEX", "ASCII"])
        self.recv_mode_box.setStyleSheet("""
            QComboBox { font-size:14px; min-height:30px; }
            QComboBox QAbstractItemView { font-size:14px; }
        """)
        right_layout.addWidget(QLabel("接收模式:"))
        right_layout.addWidget(self.recv_mode_box)

        self.recv_pause_checkbox = QCheckBox("暂停自动滚动")
        right_layout.addWidget(self.recv_pause_checkbox)

        main_layout.addLayout(right_layout, 1)
        self.stack_layout.addWidget(self.main_widget)

        # ---------------- 设置界面 ----------------
        self.settings_widget = QWidget()
        settings_layout = QVBoxLayout()
        self.settings_widget.setLayout(settings_layout)

        self.settings_title = QLabel("设置")
        self.settings_title.setAlignment(Qt.AlignCenter)
        self.settings_title.setStyleSheet("font-size:32px; font-weight:bold;")
        settings_layout.addWidget(self.settings_title)

        gyro_hint = QLabel(
            "陀螺仪标定\n\n"
            "「重新标零」：车必须完全静止。做的事是采 1 秒静止数据求零偏，"
            "驱动器的振动、风扇、人在旁边走动都会让这次标定失败重来。\n\n"
            "「标定轮距」：车会自己单侧走 300mm，然后按实际转过的角度反算"
            "左右轮中心距。场地要留出约 1.5 米见方，车会绕一个弧。"
            "结果只显示在 OLED 第 4 行和接收日志里，掉电就没了 —— "
            "想固化就抄进固件的 calib.h。"
        )
        gyro_hint.setWordWrap(True)
        gyro_hint.setStyleSheet("font-size:14px; color:#555;")
        settings_layout.addWidget(gyro_hint)

        self.btn_gyro_recal = QPushButton("重新标零（车必须静止）")
        self.btn_gyro_recal.setMinimumHeight(40)
        self.btn_gyro_recal.clicked.connect(self.on_gyro_recal)
        settings_layout.addWidget(self.btn_gyro_recal)

        self.btn_gyro_measure = QPushButton("标定轮距（车会自己走 300mm）")
        self.btn_gyro_measure.setMinimumHeight(40)
        self.btn_gyro_measure.setStyleSheet("background:#fcf8e3;")
        self.btn_gyro_measure.clicked.connect(self.on_gyro_measure)
        settings_layout.addWidget(self.btn_gyro_measure)

        self.btn_back_settings = QPushButton("返回主界面")
        self.btn_back_settings.clicked.connect(self.show_main)
        settings_layout.addWidget(self.btn_back_settings)

        self.stack_layout.addWidget(self.settings_widget)

    # ---------------- 页面切换 ----------------
    def show_settings(self):
        self.setWindowTitle("设置")
        self.stack_layout.setCurrentWidget(self.settings_widget)

    def show_main(self):
        self.setWindowTitle("道路画线机控制面板")
        self.stack_layout.setCurrentWidget(self.main_widget)

    # ---------------- 发送/接收日志 ----------------
    def clear_send_log(self):
        self.send_log.clear()

    def clear_recv_log(self):
        self.recv_log.clear()

    def append_send_log(self, data, label=""):
        if not self.send_pause_checkbox.isChecked():
            try:
                text = f"{data.decode()} ({label})"
            except:
                text = f"{data.hex()} ({label})"
            self.append_send_signal.emit(text)

    def _append_send(self, text):
        self.send_log.append(text)
        self.send_log.moveCursor(QTextCursor.End)

    def on_data_received(self, data):
        results = self.protocol.parse(data)
        for item in results:
            if item.get("kind") == "frame":
                self._dispatch_frame(item)
            self.append_recv_signal.emit(item)

    def _dispatch_frame(self, item):
        """把上行帧分发给该处理它们的地方。

        以前只有状态帧被特判了一下（更新状态栏），ACK 只写进日志。
        现在 ACK 要进配对表，状态帧还要喂给摄像头控件 —— 它靠"段走完了"
        这个信号来接下一段。
        """
        cmd = item.get("cmd")
        payload = item.get("data") or b""

        if cmd == frame.CMD_ACK and len(payload) >= 2:
            # payload[0] = 被确认的原命令，payload[1] = 结果码
            self.ack.on_ack(item["seq"], payload[0], payload[1])
            self._update_link_label()

        elif cmd == frame.CMD_STATUS and len(payload) >= 2:
            state_char = chr(payload[0]) if 32 <= payload[0] < 127 else "?"
            flags = payload[2] if len(payload) >= 3 else 0

            widget = getattr(self, "camera_widget", None)
            if widget is not None:
                widget.on_status(state_char, flags=flags)

            self._apply_gyro_flags(flags)
            self._apply_resume_flag(flags)

        elif cmd == frame.CMD_SPEED_INFO and len(payload) >= 2:
            self._apply_speed_info(item)

        elif cmd == frame.CMD_BATTERY:
            self._apply_battery(item)

        elif cmd == frame.CMD_RESUME_INFO:
            self._apply_resume_info(item)

        elif cmd == frame.CMD_HEADING:
            self._update_heading(item)

    def _append_recv(self, item):
        # 状态帧是 5Hz 的，全写进日志会把按钮回显、ACK 全刷没。
        # 它只更新状态栏 —— 这一栏以前是"最后按了哪个键"，现在变成
        # "下位机现在真实在干什么"，这是接帧之后最直观的一个变化。
        if item.get("kind") == "frame" and item.get("cmd") == frame.CMD_STATUS:
            data = item.get("data") or b""
            state = chr(data[0]) if data and 32 <= data[0] < 127 else "?"
            if state == "P" and len(data) >= 2:
                # 状态帧的 DATA[1] 是画车位子状态，固件一直在发，之前没人用。
                # 人在树莓派前只能扭头看车上那块 OLED 才知道画到第几条边。
                step = frame.PAINT_STEP_LABELS.get(data[1], str(data[1]))
                self.paint_label.setText(f"画车位:\n{step}")
            else:
                self.paint_label.setText("")
            self.status_label.setText(f"状态:\n{item['status']}")
            return

        # 航向帧同样是 5Hz，只更新界面，不进日志 ——
        # 进日志的话按钮回显和 ACK 会被它淹没，而那两样才是排障时要看的。
        if item.get("kind") == "frame" and item.get("cmd") == frame.CMD_HEADING:
            return

        # 速度帧 1Hz，同理由：只更新滑条，不进日志。
        if item.get("kind") == "frame" and item.get("cmd") == frame.CMD_SPEED_INFO:
            return

        # 电池帧 1Hz，同上：只更新上面那行字。
        if item.get("kind") == "frame" and item.get("cmd") == frame.CMD_BATTERY:
            return

        # 断点帧 1Hz，同上。
        if item.get("kind") == "frame" and item.get("cmd") == frame.CMD_RESUME_INFO:
            return

        if not self.recv_pause_checkbox.isChecked():
            mode = self.recv_mode_box.currentText()
            if mode == "HEX":
                text = f"{item['raw']} ({item['status']})"
            elif mode == "ASCII":
                text = f"{item['char']} ({item['status']})"
            else:
                if 32 <= item['value'] <= 126:
                    text = f"{item['char']} ({item['status']})"
                else:
                    text = f"{item['raw']} ({item['status']})"
            self.recv_log.append(text)
            self.recv_log.moveCursor(QTextCursor.End)
        self.status_label.setText(f"状态:\n{item['status']}")

    # ---------------- 陀螺仪 ----------------
    def _apply_gyro_flags(self, flags):
        """按状态帧的 flags 回填界面。

        界面的勾选状态和"下位机真的在闭环"必须是同一件事。
        用户点了勾但固件回绝（没标零、传感器不在）时，这里会把勾弹回去 ——
        不做这一步的话，界面显示"已接管"，车其实一点没纠，人还会去怪标定。
        """
        ready = bool(flags & 0x08)
        closed = bool(flags & 0x10)

        self._gyro_ready = ready
        self._gyro_closed = closed
        # 0x20 = 这一局读到过 I2C 错误，角度精度已经打过折扣。
        # 只能置位不能清，所以显示上说的是"读错过"，不是"正在错"。
        self._gyro_err = bool(flags & 0x20)

        self.gyro_check.blockSignals(True)
        self.gyro_check.setChecked(closed)
        self.gyro_check.blockSignals(False)
        self.gyro_check.setEnabled(ready)

        # 就绪之后不写文本：航向帧 5Hz 也在写同一行，两边交替覆盖会让
        # 这行字每秒跳五次（"闭环中"和"READY +0.3 dps"来回切）。
        # 未就绪时没有航向帧可等，必须在这里给出原因。
        if not ready:
            self.gyro_label.setText("陀螺仪：未就绪（没接 / 标零中 / 掉线）")
            self.gyro_label.setStyleSheet("font-size:14px; color:#c00000;")

    def _apply_resume_flag(self, flags):
        """续画按钮跟着固件的断点走。

        为什么不由上位机自己记"刚才发了急停，所以有断点"：断点有没有，
        只有固件知道 —— 停在转弯上不记、剩不到 50mm 不记、按了 W 会清掉。
        上位机猜的话，一半的情况下按钮亮着但续不上。
        """
        self.btn_resume.setEnabled(bool(flags & frame.STATUS_FLAG_RESUME))

    def _update_heading(self, item):
        deg = item.get("heading_deg", 0.0)
        rate = item.get("rate_dps", 0.0)
        state = frame.GYRO_STATE_NAMES.get(item.get("gyro_state", 0), "?")

        self.heading_label.setText(f"航向 {deg:+.2f}\u00b0")

        # 就绪之后这一行只由航向帧写（理由见 _apply_gyro_flags）。
        # 闭环状态从状态帧带过来的标记里取，不在这里自己猜。
        tail = "闭环中" if self._gyro_closed else "仅监视"
        if self._gyro_err:
            tail += " · 读错过（查接线）"
        self.gyro_label.setText(f"陀螺仪：{state}  {rate:+.1f} dps · {tail}")

        if self._gyro_closed:
            color = "#008000"
        elif self._gyro_err:
            color = "#c07000"
        else:
            color = "#555555"
        self.gyro_label.setStyleSheet(f"font-size:14px; color:{color};")

        # 偏得多就变色：2 度以内算直，5 度以外已经能看出来了
        if abs(deg) < 2.0:
            color = "#008000"
        elif abs(deg) < 5.0:
            color = "#c07000"
        else:
            color = "#c00000"
        self.heading_label.setStyleSheet(
            f"font-size:24px; font-weight:bold; color:{color};"
        )

    def on_gyro_toggle(self, checked):
        sub = frame.GYRO_ON if checked else frame.GYRO_OFF
        self.send_frame(frame.build_gyro(sub, self._next_seq()),
                        "陀螺仪 开闭环" if checked else "陀螺仪 关闭环")

    def on_gyro_recal(self):
        self.send_frame(frame.build_gyro(frame.GYRO_RECAL, self._next_seq()),
                        "陀螺仪 重新标零")
        self.append_send_signal.emit(
            "标零期间车必须完全静止，约 1 秒。失败会自动重试。"
        )

    def on_gyro_measure(self):
        # 这条会让车自己动起来，先确认一次再发 —— 和别的按钮不一样，
        # 它不是"发一个字节"，而是"车要绕一个弧走 300mm"。
        self.send_frame(frame.build_gyro(frame.GYRO_MEASURE, self._next_seq()),
                        "陀螺仪 标定轮距")
        self.append_send_signal.emit(
            f"轮距标定：车将单侧走 {frame.GYRO_MEASURE_DIFF_MM}mm，"
            "请让开并留出约 1.5 米空间。结果看 OLED 第 4 行或接收日志。"
        )

    # ---------------- 核心按钮功能 ----------------
    def _detach_camera_auto(self):
        """把摄像头的自动流程撤下来。

        按了面板上任何一个键 = 不想让它自己干活了。不撤的话按"停止复位"
        或者"关水泵"，摄像头还在按帧下发指令 —— 人按的停停不停得住
        全看手速，这不能接受。下位机那边也堵了一道：RESPRAY / MOVE
        只在 STATE_WAIT 被接受。

        撤完之后状态就是 WAIT，所以调用方可以紧接着发画车位 / 二次喷涂。
        """
        widget = getattr(self, 'camera_widget', None)
        if widget is None:
            return

        if widget.mode == CameraWidget.MODE_TRACK:
            widget.stop_track()
            self.btn_track.setChecked(False)
        else:
            widget.mode = CameraWidget.MODE_VIEW

    def send_paint(self):
        """按界面上的尺寸画一个车位。

        改发帧而不是单字节 'P'，就为了这两个数能改。固件收到一帧会先校验
        范围、再写尺寸、最后置 STATE_PAINT —— 三者一步完成，不存在
        "尺寸改了但车没动"或者"用旧尺寸开跑"的中间态。

        纠偏开关也随这一帧走：DATA[4] 的 bit0。做成帧里的一个位而不是
        单独一条"开关"命令，是因为后者会出现「开关改了但这次画车位没赶上」
        的中间态 —— 而那种状态在日志里看不出来。

        先撤相机再发帧：视觉那条线会周期发 MOVE / TRIM，抢在画车位前面
        到达时固件回 ERR_STATE，画不成还只在日志里留一个结果码。
        """
        length = self.paint_len_spin.value()
        width = self.paint_wid_spin.value()
        heading = self.paint_heading_check.isChecked()

        self._detach_camera_auto()
        self.send_frame(
            frame.build_paint(length, width, heading=heading,
                              seq=self._next_seq()),
            f"画车位 {length}x{width}mm" + (" +纠偏" if heading else ""),
        )

    def send_resume(self):
        """从急停断点接着画。

        帧里的长宽不生效：断点里带着这一圈的尺寸，固件只认断点。
        没有断点时固件回 ERR_STATE，而不是当成新的一条从头画 ——
        后者会把已经喷过的两条边再喷一遍，而人是从"车动了"才发现。

        先撤相机，理由和 send_paint 一样：视觉那条线 10Hz 发 MOVE/TRIM，
        抢在前面到达时固件回 ERR_STATE，续不上还只在日志里留一个结果码。
        """
        self._detach_camera_auto()
        self.send_frame(frame.build_paint(resume=True, seq=self._next_seq()),
                        "画车位 续画")

    # ---------------- 速度 ----------------
    def _apply_speed_info(self, item):
        """按固件报回来的速度帧回填滑条。

        为什么必须回填：改速有两条路（本界面滑条、手机蓝牙 +/-）。不回填的话，
        手机改完这里还停在老值，人按界面上的数字判断漆量 —— 而车跑的是另一个
        速度。固件那边把值夹到边界（蓝牙连按 `+` 停在 150）也是靠这条帧才看得见。
        滑条的取值范围本来就是 150~1500，所以这里的"限幅"只可能来自另一边。

        手指还按在滑条上时不回填：那会和用户抢，滑条自己跳回去。
        松手后我们会发一帧，固件再回一帧确认，中间的差异不会留到最后。
        """
        arr = item.get("speed_arr")
        if not isinstance(arr, int) or not (frame.SPEED_MIN_ARR <= arr <= frame.SPEED_MAX_ARR):
            return          # 帧坏了（CRC 放过但值不合理）就别拿它动界面

        # 刚发出去、还没被固件处理：这一两帧是旧值，别拿它把滑条拽回去。
        if (arr != self._speed_sent
                and time.monotonic() - self._speed_sent_at < SPEED_SETTLE_S):
            return

        if not self.speed_slider.isSliderDown():
            if self.speed_slider.value() != arr:
                self.speed_slider.blockSignals(True)
                self.speed_slider.setValue(arr)
                self.speed_slider.blockSignals(False)
            self.speed_label.setText(str(arr))

        # 固件已经在这个值上了，再按"值没变不发"的规则也就不会重发 ——
        # 但对齐一下 _speed_sent，省得"手机改到 200、界面拖回 200"时误发一帧。
        self._speed_sent = arr

        # 橙色 = 还在等固件写 flash。等它置上 0x01 这位，颜色回到默认。
        saved = item.get("speed_saved", False)
        self.speed_label.setStyleSheet(
            "font-size:16px; font-weight:bold;" if saved
            else "font-size:16px; font-weight:bold; color:#b06000;"
        )
        self.speed_label.setToolTip(
            "已写进固件 flash，断电不丢" if saved
            else "固件还没写 flash（改完 2 秒、车停着才写），现在断电会回到上次存的档位"
        )

    def _apply_resume_info(self, item):
        """把固件 flash 里那份断点画到状态区。"

        这一行解决的是"核对不了"：固件上电已经把断点的长宽还原了，续画也
        按它走，但面板一直没地方看。人不放心的时候只能按一下续画看车动没动，
        而那会儿漆已经在喷了。

        长宽不往上面那两个输入框里填：输入框是"我打算画多大"，这个断点是
        "车上记着多大"。自动填进去会让人以为尺寸改过了，而续画只认断点 ——
        改了也白改，人还以为生效了。
        """
        if not item.get("resume_valid"):
            self.resume_label.setText("断点  无")
            self.resume_label.setStyleSheet("font-size:14px; color:#777;")
            self.resume_label.setToolTip("固件 flash 里没有断点")
            return

        length = item.get("resume_len_mm", 0)
        width = item.get("resume_wid_mm", 0)
        step = frame.PAINT_STEP_LABELS.get(item.get("resume_step", 0), "?")

        self.resume_label.setText(f"断点  {length}x{width}mm  停{step}")
        self.resume_label.setStyleSheet("font-size:14px; color:#1a7f37;")
        self.resume_label.setToolTip(
            "固件 flash 里存着的断点，续画就按这个尺寸和位置接下去。\n"
            "它和上面输入框里的值不是一回事 —— 续画只认断点。"
        )

    def _apply_battery(self, item):
        """把电池帧画到状态区。

        采样无效（分压线掉了、ADC 读到接近 0）显示 `---`，不显示 0.0V ——
        后者会让人去充电，而真正的问题是那根线。

        门限不由这一侧判断，只认固件给的 BATT_FLAG_LOW：两边各写一遍，
        改一处忘另一处就会出现"车上在响、界面还是绿的"。
        """
        if item.get("batt_absent"):
            # 分压电阻没焊的时候 PC0 悬空，读回来是乱跳的值。以前这种情况
            # 显示 `---`，看着像固件坏了 —— 实际是少两个电阻。
            self.battery_label.setText("电池  未接")
            self.battery_label.setStyleSheet("font-size:14px; color:#777;")
            self.battery_label.setToolTip(
                "引脚上读到的不是电池的电压：分压电阻没焊，或者那根线断了。\n"
                "接法：电池正极 -> 100k -> PC0 -> 10k -> 地，中间并 100nF 到地。\n"
                "接上之后这一行会自己变成电压，不用重启。"
            )
            return

        if not item.get("batt_valid"):
            self.battery_label.setText("电池  ---")
            self.battery_label.setStyleSheet("font-size:14px; color:#777;")
            self.battery_label.setToolTip(
                "这一轮没读到（ADC 超时），下一轮会重试。"
                "一直是这样就查分压线。"
            )
            return

        mv = item.get("batt_mv", 0)
        self.battery_label.setText(f"电池  {mv / 1000.0:.1f}V")
        if item.get("batt_low"):
            self.battery_label.setStyleSheet(
                "font-size:14px; font-weight:bold; color:#c00000;"
            )
            self.battery_label.setToolTip("低于固件门限，画完这一圈就换电池")
        else:
            self.battery_label.setStyleSheet("font-size:14px; color:#2a7d2a;")
            self.battery_label.setToolTip("电压正常")

    def on_speed_changed(self, arr):
        self.speed_label.setText(str(arr))
        # 刚动过滑条，这个值一定还没写进 flash —— 先标橙，等下一条速度帧
        # 回来说"存好了"再变回去。不这么做的话，松手后到固件写完之间的
        # 一两秒里，标签是黑的，看着像已经存上了。
        self.speed_label.setStyleSheet(
            "font-size:16px; font-weight:bold; color:#b06000;"
        )

        # 拖动过程中不发。QSlider 每移一个像素就 valueChanged 一次，9600 下
        # 一帧 10.4ms —— 拖一下就几十条，ACK 和错误码会被冲干净，固件那边
        # 也要处理一串注定被后到的值覆盖的命令。松手时由 sliderReleased 发一次。
        if self.speed_slider.isSliderDown():
            return
        self._send_speed(arr)

    def on_speed_released(self):
        self._send_speed(self.speed_slider.value())

    def _send_speed(self, arr):
        # 值没变就不发。点一下滑条（不拖动）也会触发 sliderReleased，
        # 每次都发一帧等于给日志刷噪声。
        if arr == self._speed_sent:
            return
        # 写出去了才记账。串口断着的时候拖滑条，记成"已发"的话，之后
        # 拖回同一个值会被上面那句"值没变"挡掉，界面上写着 600 而车
        # 还按旧档跑。
        if self.send_frame(frame.build_speed(arr, self._next_seq()), f"速度 {arr}"):
            self._speed_sent = arr
            self._speed_sent_at = time.monotonic()

    def send_button(self):
        label = self.sender().text()
        data = dict(PANEL_BUTTONS).get(label)
        if data is None:
            return

        # 先撤相机，再发这个字节。反过来的话，撤相机那一下的停止命令会落到
        # 按钮之后：按「停止复位」实际停在 TING，用户还得再按一次。
        #
        # 二次喷涂也要撤：它要车停着（固件只在 WAIT 收 RESPRAY），而原来
        # 这条路径跳过了撤相机 —— 相机窗口关了、车还在走完这一段、泵还开着。
        if label not in CAMERA_BUTTONS or label == "二次喷涂":
            self._detach_camera_auto()

        # 串口没开时不能再写"状态: 紧急停止"—— 那会让人以为按下去生效了。
        if not self.serial_mgr.send(data):
            self.append_send_log(data, f"{label}（未发出）")
            self.status_label.setText(f"状态:\n{label} 未发出")
            self._update_link_label()
            return

        self.append_send_log(data, label)
        self.status_label.setText(f"状态:\n{label}")

        # ---------------- 摄像头弹窗 ----------------
        if label in CAMERA_BUTTONS:
            self.close_camera_window()
            # 自动巡线走的是 toggle_track，不经过这里 —— 它没有对应的
            # 单字节指令，进不了 PANEL_BUTTONS 这张表。
            mode = (CameraWidget.MODE_RESPRAY
                    if label == "二次喷涂"
                    else CameraWidget.MODE_VIEW)
            self.open_camera_window(label, mode)
        elif label in CAMERA_OFF_BUTTONS:
            self.close_camera_window()

    # ---------------- 自动巡线 ----------------
    def toggle_track(self):
        if self.btn_track.isChecked():
            self.close_camera_window()
            self.open_camera_window("自动巡线喷涂", CameraWidget.MODE_TRACK)

            widget = getattr(self, "camera_widget", None)
            if widget is None or not widget.camera_ok:
                # 画面都没起来就别发 MOVE。没有视觉还按帧往前跑，是在漆线上
                # 开一台瞎车 —— 而界面上写着"正在巡线"，日志里全是已确认。
                self.append_send_log(b"(track refused)", "自动巡线：摄像头未就绪")
                self.status_label.setText("状态:\n摄像头未就绪，未开始巡线")
                self.btn_track.setChecked(False)
                self.close_camera_window()
                return

            widget.start_track()
            self.append_send_log(b"(track start)", "自动巡线")

        else:
            widget = getattr(self, "camera_widget", None)
            if widget is not None:
                widget.stop_track()
                self.append_send_log(b"(track stop)", "自动巡线")
            self.close_camera_window()

    # ---------------- 摄像头弹窗 ----------------
    def close_camera_window(self):
        win = getattr(self, 'camera_window', None)
        if win is None:
            return
        widget = getattr(self, 'camera_widget', None)
        if widget is not None:
            widget.stop_camera()
        win.close()
        win.deleteLater()      # 不然每按一次就漏一个 QWidget
        self.camera_window = None
        self.camera_widget = None

    def open_camera_window(self, label, mode=None):
        if mode is None:
            mode = CameraWidget.MODE_VIEW

        self.camera_window = QWidget()
        self.camera_window.setWindowTitle(f"摄像头画面 - {label}")
        self.camera_window.resize(800, 600)
        layout = QVBoxLayout()
        self.camera_window.setLayout(layout)
        self.camera_widget = CameraWidget(
            self.serial_mgr,
            mode=mode,
            ack_tracker=self.ack,
        )
        layout.addWidget(self.camera_widget)
        self.camera_widget.start_camera()
        self.camera_window.closeEvent = self.camera_close_event
        self.camera_window.show()

    # ---------------- 主窗口关闭 ----------------
    def closeEvent(self, event):
        # 摄像头窗口是没有父窗口的顶层窗口（camera_window = QWidget()），
        # 主界面关掉它不会跟着关，还会继续按帧下发指令。
        widget = getattr(self, "camera_widget", None)
        if widget is not None and widget.mode == CameraWidget.MODE_TRACK:
            widget.stop_track()

        self.close_camera_window()
        event.accept()

    def camera_close_event(self, event):
        # 用户点右上角 X 时走这里。子控件的 closeEvent 不一定会触发
        # （父窗口关闭只是把子控件隐藏），所以摄像头必须在这里自己停。
        widget = getattr(self, 'camera_widget', None)
        if widget is not None:
            if widget.mode == CameraWidget.MODE_TRACK:
                widget.stop_track()
                self.btn_track.setChecked(False)
            widget.stop_camera()

        # 点 X 之后不会再走 close_camera_window，引用得自己放掉：
        # 留着的话，下次 _detach_camera_auto 拿到的是一个已经关掉的控件，
        # 窗口对象也一直挂着不释放。
        win = getattr(self, 'camera_window', None)
        if win is not None:
            win.deleteLater()
        self.camera_window = None
        self.camera_widget = None
        event.accept()
