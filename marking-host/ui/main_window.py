import cv2
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QTextEdit, QLabel,
    QComboBox, QCheckBox, QSlider, QStackedLayout, QSpacerItem, QSizePolicy,
    QSpinBox
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QTextCursor

from ui.camera_widget import CameraWidget
from core import frame
from core.ack import AckTracker
from core.protocol import (
    PANEL_BUTTONS, CAMERA_BUTTONS, CAMERA_OFF_BUTTONS, PAINT_BUTTON
)


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
    def send_frame(self, raw, label=""):
        """发一帧并登记 ACK。所有走帧协议的动作都要经过这里 ——
        绕过它发送，链路健康度就少了这一条，而且永远不会被发现。"""
        self.serial_mgr.send(raw)
        self.ack.register(raw)
        self.append_send_log(raw, label)
        self._update_link_label()

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

        self.btn_paint = QPushButton(PAINT_BUTTON)
        self.btn_paint.setMinimumHeight(40)
        self.btn_paint.setStyleSheet("font-size:20px;")
        self.btn_paint.clicked.connect(self.send_paint)
        left_layout.addWidget(self.btn_paint)

        # 按钮表在 core/protocol.py 里，和指令表放在一起，避免标签写两遍对不上
        self.control_buttons = []
        for label, _cmd in PANEL_BUTTONS:
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

        main_layout.addLayout(left_layout, 1)

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

        elif cmd == frame.CMD_HEADING:
            self._update_heading(item)

    def _append_recv(self, item):
        # 状态帧是 5Hz 的，全写进日志会把按钮回显、ACK 全刷没。
        # 它只更新状态栏 —— 这一栏以前是"最后按了哪个键"，现在变成
        # "下位机现在真实在干什么"，这是接帧之后最直观的一个变化。
        if item.get("kind") == "frame" and item.get("cmd") == frame.CMD_STATUS:
            self.status_label.setText(f"状态:\n{item['status']}")
            return

        # 航向帧同样是 5Hz，只更新界面，不进日志 ——
        # 进日志的话按钮回显和 ACK 会被它淹没，而那两样才是排障时要看的。
        if item.get("kind") == "frame" and item.get("cmd") == frame.CMD_HEADING:
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
        self.send_frame(frame.build_gyro(sub),
                        "陀螺仪 开闭环" if checked else "陀螺仪 关闭环")

    def on_gyro_recal(self):
        self.send_frame(frame.build_gyro(frame.GYRO_RECAL), "陀螺仪 重新标零")
        self.append_send_signal.emit(
            "标零期间车必须完全静止，约 1 秒。失败会自动重试。"
        )

    def on_gyro_measure(self):
        # 这条会让车自己动起来，先确认一次再发 —— 和别的按钮不一样，
        # 它不是"发一个字节"，而是"车要绕一个弧走 300mm"。
        self.send_frame(frame.build_gyro(frame.GYRO_MEASURE), "陀螺仪 标定轮距")
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

        先撤相机再发帧：视觉那条线会周期发 MOVE / TRIM，抢在画车位前面
        到达时固件回 ERR_STATE，画不成还只在日志里留一个结果码。
        """
        length = self.paint_len_spin.value()
        width = self.paint_wid_spin.value()

        self._detach_camera_auto()
        self.send_frame(
            frame.build_paint(length, width),
            f"画车位 {length}x{width}mm",
        )

    def send_button(self):
        label = self.sender().text()
        data = dict(PANEL_BUTTONS).get(label)
        if data is None:
            return

        self.serial_mgr.send(data)
        self.append_send_log(data, label)
        self.status_label.setText(f"状态:\n{label}")

        if label not in CAMERA_BUTTONS:
            self._detach_camera_auto()

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

            if self.camera_widget is not None:
                self.camera_widget.start_track()
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
        self.camera_widget.stop_camera()
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
        event.accept()
