"""摄像头窗口：标线检测 -> 三态判定 -> 多帧投票 -> 下发。

三种用途，用 mode 区分（以前只有"发不发 C"一个布尔，画车位取景和补喷共用
同一个控件，只能靠一个 auto_paint 开关区分，脆得很）：

    MODE_VIEW     只看。一个字节都不发。
    MODE_RESPRAY  车停着，看到破损发一帧补喷（走一段、停、再看）。
    MODE_TRACK    车在走，按帧下发纠偏（CMD_TRIM）和泵开关（CMD_PUMP）。
                  这是"真正能干活"的那个模式：走到哪喷到哪，边走边纠。

为什么要有投票
--------------
单帧判定会抖。一段 30 帧里有一帧被反光糊住，coverage 从 0.7 掉到 0.3，
判据当场给出"磨损、补 400mm"。改造前就是这样：一帧噪点 = 车喷出去一米。

现场看到的不是"偶尔多喷"，是"它在乱喷" —— 因为 30fps 下 3% 的误判率
意味着每秒钟一次。改成连续 5 帧里至少 3 帧同意才行动，误判率降到
C(5,3)*0.03^3 量级，而真实破损会连续几十帧都成立，丝毫不受影响。

    P(5 帧里 ≥3 帧误判) ≈ 10 * 0.03^3 ≈ 2.7e-4

代价是响应慢 2~3 帧（约 100ms）。车走 0.16m/s，100ms 是 16mm —— 补喷
本来就有几百毫米的量级，这点延迟无所谓。

为什么要"看不清"这一态
--------------------
线被阴影盖住、镜头脏了、车开出了标线区域 —— 这些都是"没看到"，不是
"看到了、很好"。二值判据把它们混成一件，表现是"它不工作，但日志正常"。
现在 QUALITY_UNSURE 会单独显示，也会让泵关掉（看不到就别喷）。
"""

import time
from collections import deque

import cv2
import numpy as np

from PyQt5.QtWidgets import QLabel
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QImage, QPixmap

from core import frame, lane_control
from core.line_quality import (
    close_cracks,
    extract_line_pixels,
    classify_line_quality,
    QUALITY_OK,
    QUALITY_WORN,
    QUALITY_BREAK,
    QUALITY_UNSURE,
    QUALITY_NAMES,
    SPRAY_LEVELS,
    SPRAY_MM_PER_LEVEL,
)


class CameraWidget(QLabel):

    MODE_VIEW = "view"
    MODE_RESPRAY = "respray"
    MODE_TRACK = "track"

    # ---------------- 投票 ----------------
    VOTE_WINDOW = 5          # 看最近几帧
    VOTE_MAJORITY = 3        # 至少几帧同意

    # ---------------- 补喷模式 ----------------
    # 两次补喷之间的**最小**间隔，不再是"一段跑完的时间"。
    # 真正的节拍由固件状态帧驱动（见 maybe_send_respray）：一段走完、状态回到
    # W 才允许发下一帧。这里只挡住"同一位置连喷"—— 状态帧 5Hz，不设下限的话
    # 车一停就可能连续发几帧，漆会堆在同一处。
    SPRAY_INTERVAL_S = 1.0

    # ---------------- 巡线模式 ----------------
    # 纠偏下发周期。100ms = 10Hz，9600 下一帧 10.4ms，占空比约 10%。
    # 再快没有意义：车的机械响应跟不上，而且会和 5Hz 的状态帧抢带宽。
    TRIM_INTERVAL_S = 0.10

    # 泵的滞回。判破损要连续几帧才开，判完好要连续几帧才关 ——
    # 开和关的阈值不一样，避免在一段漆的边界上来回跳，把泵当继电器打。
    PUMP_ON_AFTER = 2
    PUMP_OFF_AFTER = 6

    # 一段走多长。5000mm 是固件的上界（PROTO_MOVE_MAX_MM），
    # 默认速度下约 31 秒。段越长，段与段之间的空档越少。
    SEGMENT_MM = 5000

    def __init__(self, serial_mgr=None, mode=None, auto_paint=None, ack_tracker=None):
        super().__init__()

        self.setAlignment(Qt.AlignCenter)
        self.setText("摄像头未开启")

        self.cap = None

        # auto_paint 是改造前的参数（True = 补喷闭环）。保留是为了不让
        # 老的调用方静默失效 —— 传了它就映射到 mode，但会明确提示一遍。
        if mode is None:
            if auto_paint is None:
                mode = self.MODE_VIEW
            else:
                mode = self.MODE_RESPRAY if auto_paint else self.MODE_VIEW

        self.mode = mode
        self.serial_mgr = serial_mgr
        self.ack_tracker = ack_tracker

        self.last_spray = 0.0
        self.spray_count = 0
        self.last_level = 0
        self._seq = 0

        # ---------------- 检测状态 ----------------
        self.center_queue = deque(maxlen=5)
        # 每个元素是 (质量结论, 这一帧给出的补喷档位)。档位跟着一起存，
        # 是为了让"补多长"和"该不该补"出自同一批帧 —— 见 _voted_level。
        self.votes = deque(maxlen=self.VOTE_WINDOW)
        self.quality = QUALITY_UNSURE
        self.quality_note = ""

        # ---------------- 巡线状态 ----------------
        self.tracking = False
        self.param_trim = 0.0              # 上一段实际发出去的差速
        self.last_trim_time = 0.0
        self.heading_deg = 0.0
        self.lateral_ratio = 0.0
        self.pump_on = False
        self._bad_streak = 0
        self._good_streak = 0
        self._segments_sent = 0
        self._last_segment_time = 0.0
        self._fw_state = "?"
        # 下位机状态帧里带回来的标志位（和固件 SendStatusFrame 的 flags 同义）
        self.fw_flags = 0
        # 陀螺仪航向闭环是否接管了差速。为真时视觉不再发 TRIM ——
        # 不是"少发一点"，是"一个都不发"：两个控制器抢同一个执行器
        # 只会互相抵消，而且双方都认为自己没错。
        self.gyro_closed = False

        # 摄像头到底开没开起来。巡线要用它决定"能不能发 MOVE"——
        # 画面都没起来还按帧往前跑，等于一台瞎车在漆线上开。
        self.camera_ok = False

        # 连续读失败多少帧才当成"相机掉了"。30fps 下 10 帧约 0.3 秒，
        # USB 抖一下不会误停车；真的拔了线，0.3 秒内就该停。
        self._camera_fail_streak = 0

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_frame)

        # ================= HSV 黄色范围 =================
        self.HSV_YELLOW_LOW = np.array([15, 50, 150])
        self.HSV_YELLOW_HIGH = np.array([30, 255, 255])

        # ================= 参数 =================
        self.RECT_WIDTH = 100
        self.MIN_AREA = 200
        self.LINEAR_ASPECT_RATIO_THRESH = 4.0
        self.CAMERA_LOST_FRAMES = 10
        # 破损判据的阈值统一在 core/line_quality.py 的 THRESHOLDS 里，
        # 现场标定只改那一处，别在这儿再放一份。

    # ==========================================================
    # 摄像头
    # ==========================================================
    def start_camera(self):

        if self.cap is not None:
            return True

        self.cap = cv2.VideoCapture(0)

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        if not self.cap.isOpened():
            # 失败必须把句柄扔掉。留着一个没打开的 VideoCapture，开头那句
            # `if self.cap is not None: return` 会让之后每次 start_camera()
            # 都直接返回 —— 摄像头再也起不来，还漏个句柄。
            self.cap.release()
            self.cap = None
            self.camera_ok = False
            self.setText("摄像头打开失败")
            return False

        self.camera_ok = True
        self._camera_fail_streak = 0
        self.timer.start(30)
        return True

    def stop_camera(self):

        self.timer.stop()
        self.camera_ok = False

        if self.cap:
            self.cap.release()

        self.cap = None

        self.clear()
        self.setText("摄像头已关闭")

    def _camera_hiccup(self):
        """读相机失败。单帧抖动不算，连续到门限才当"相机掉了"。

        以前这里直接 return：读失败既不关泵、也不停车，而跟踪循环还在按
        状态帧一段段发 MOVE —— 相机死了车照样往前走，泵还开着，面板上只剩
        一帧冻住的画面，什么提示都没有。0.3 秒的滞回是为了 USB 抖一下不至于
        停掉整条线。
        """
        self._camera_fail_streak += 1
        if self._camera_fail_streak < self.CAMERA_LOST_FRAMES:
            return
        if not self.camera_ok and not self.tracking:
            return                      # 已经处理过了

        self.camera_ok = False
        self.quality = QUALITY_UNSURE
        self.quality_note = "相机中断"
        self.setText("摄像头中断")

        if self.tracking:
            # 看不见就不喷、也不接着发下一段。stop_track 会关泵再发 'W'，
            # 顺序和用户手动停一样。
            self.stop_track()

    # ==========================================================
    # 串口发送
    # ==========================================================
    def _next_seq(self):
        self._seq = (self._seq + 1) & 0xFF
        return self._seq

    def _send(self, raw):
        """发一帧，顺手登记 ACK。没有串口就什么都不做 —— 但调用方要能看出来。"""
        if self.serial_mgr is None:
            return None

        self.serial_mgr.send(raw)

        if self.ack_tracker is not None:
            return self.ack_tracker.register(raw)

        return None

    # ==========================================================
    # 投票
    # ==========================================================
    def _voted_quality(self):
        """最近几帧的多数结论。凑不够多数就报"看不清"。

        为什么门槛是 VOTE_MAJORITY(3) 而不是"窗口必须满 5 帧"：
        要求窗口填满的话，摄像头刚开的那 5 帧（150ms）一律是 UNSURE，
        泵关着、纠偏不发 —— 表现是"按了开始，车先愣一下"。而 3 帧一致
        其实比"5 帧里 3 帧"更强：3 帧全一样，没有任何分歧。

        报 UNSURE 而不是沿用上一帧的结论：宁可不喷，也不要拿着一个过期的
        "完好"当依据继续往前走。
        """
        if len(self.votes) < self.VOTE_MAJORITY:
            return QUALITY_UNSURE

        counts = {}
        for value, _level in self.votes:
            counts[value] = counts.get(value, 0) + 1

        state, hits = max(counts.items(), key=lambda kv: kv[1])

        if hits < self.VOTE_MAJORITY:
            return QUALITY_UNSURE

        return state

    def _voted_level(self, quality):
        """投票结论对应哪一档补喷。

        取与结论一致的那些帧里最近一帧的档位。补多长和该不该补必须出自
        同一批帧：用当前帧的档位的话，一旦投票和当前帧判断不同，补出来的
        长度就和投票结论对不上，而日志里只看得到"发了一帧补喷"。

        一帧都没有（不该发生，结论就是从这些帧里投出来的）就退回 1 档：
        补短了还能再补一次，补长了没有回头路。
        """
        for value, level in reversed(self.votes):
            if value == quality and level > 0:
                return level
        return 1

    # ==========================================================
    # 补喷模式
    # ==========================================================
    def maybe_send_respray(self, level=5):
        """破损还在、而且车已经停下，就补一帧 CMD_RESPRAY。

        改造前这里是两次独立 write（先 b'2' 设档、再 b'C' 开喷），两次之间
        没有任何原子性：中间插进任何一次发送，下位机就只收到设档、没收到
        开喷 —— 档位变了、车没动，日志上看着两条都发了，完全静默。
        现在一条命令一次 write，帧自带 CRC。

        节拍靠固件状态帧，不靠猜时间。以前用固定 2 秒间隔，而 1m 的补喷段
        在默认速度下要 2~3 秒才走完 —— 下一帧正好落在 STATE_RESPRAY 里，
        被固件回 ERR_STATE 丢掉。日志里只是一个结果码 0x01，车实际少喷了
        一段，而且什么都不报。固件每 200ms 主动发一次状态帧，直接问它
        "你现在是不是空闲的"比按时间推更可靠。

        _fw_state 还是 "?" 表示一帧状态都没收到过（串口只发不收、或者刚开
        窗口），这时候退回"只看最小间隔"——不能因为收不到回执就完全不动。
        """
        if self.mode != self.MODE_RESPRAY or self.serial_mgr is None:
            return

        if self._fw_state not in ("?", "W"):
            return

        now = time.monotonic()

        if now - self.last_spray < self.SPRAY_INTERVAL_S:
            return

        level = max(1, min(SPRAY_LEVELS, int(level)))
        self.last_spray = now
        self.spray_count += 1
        self.last_level = level

        self._send(frame.build_respray(level * SPRAY_MM_PER_LEVEL, self._next_seq()))

    # ==========================================================
    # 巡线模式
    # ==========================================================
    def start_track(self, segment_mm=None):
        """开始边走边喷边纠。"""
        if self.mode != self.MODE_TRACK:
            return False

        if segment_mm is None:
            segment_mm = self.SEGMENT_MM

        self.tracking = True
        self._segments_sent = 0
        self.param_trim = 0.0
        self.last_trim_time = 0.0
        self._bad_streak = 0
        self._good_streak = 0

        self._send_segment(segment_mm)
        return True

    def stop_track(self):
        """停车。先关泵再发命令 —— 顺序反了的话，车停了泵还在出漆。

        用 'W' 而不是 'T'：'T' 把状态停在 TING，而画车位、二次喷涂都只认
        WAIT，撤完相机还得再补一条 'W' 才能发命令，两条命令还要求到达顺序。
        'W' 一条就做完"停电机 + 回 WAIT"。代价是它会作废画车位断点 ——
        巡线途中本来就没有正在进行的画车位，留下的旧断点在车已经挪走之后
        也没法续，清掉不影响什么。

        pump_on 必须跟着清。只发一帧关泵而不改这个标志的话，下次巡线
        _apply_pump(True) 会因为"值没变"直接 return，头几帧该喷的地方不喷，
        HUD 也一直显示泵开着。
        """
        self.tracking = False
        self.pump_on = False

        self._send(frame.build_pump(False, self._next_seq()))

        # 裸字节不登记 ACK：固件对它们只回显，登记了必然超时。
        if self.serial_mgr is not None:
            self.serial_mgr.send(b"W")

    def _send_segment(self, segment_mm=None):
        if segment_mm is None:
            segment_mm = self.SEGMENT_MM

        self._segments_sent += 1
        self._last_segment_time = time.monotonic()

        # 固件在新的一段把累计差速清零（Motion_DriveLR 里的 s_trim_accum），
        # 这边的速率限制也得从 0 起算。不归零的话，上一段末尾的 20mm 会让
        # 新段第一次纠偏被限成"只能变 8mm"，而车上早就回到 0 了。
        self.param_trim = 0.0

        self._send(frame.build_move(frame.MOVE_FWD, int(segment_mm), int(segment_mm),
                                    self._next_seq()))

    def _track_tick(self, heading_deg, lateral_ratio):
        """每帧调一次。按 TRIM_INTERVAL_S 节流下发纠偏。

        为什么用 CMD_TRIM（段内改目标）而不是每段重发 MOVE（重设）：
        重设要等一段走完，5000mm 一段就是 31 秒一次，等于没有反馈。
        TRIM 在段内直接改左右轮的目标脉冲数，31 秒里能纠 300 次。

        trim_mm 算出来的是"这一段累计该给多少差速"（带速率限制的绝对量），
        和固件 Motion_Trim 的语义一致：它记住已经落地的量，只落地差值。

        代价是差速会在段内累积（内侧轮一路提前停）。所以固件那边给
        Motion_Trim 加了累计上限，上位机这边有限幅 + 速率限制，
        两边一起把"越纠越歪"挡在外面。
        """
        # 陀螺仪闭环接管了差速，视觉这条就不发言了。
        #
        # 为什么是"完全闭嘴"而不是"少发一点"：两条控制律都在改同一组
        # 目标脉冲，一个按角度算、一个按像素算，谁后到谁覆盖。表现上会
        # 是车周期性左一下右一下，而两边日志都是"已下发、已确认"。
        # 分工才是对的：航向归下位机（10ms 环、不受光照影响），
        # 视觉只管"这块漆破没破"（判破损、开关泵）。
        if self.gyro_closed:
            return

        now = time.monotonic()

        if now - self.last_trim_time < self.TRIM_INTERVAL_S:
            return

        self.last_trim_time = now

        trim = lane_control.trim_mm(heading_deg, lateral_ratio,
                                    previous=self.param_trim)

        # param_trim 既是速率限制的"上一次输出"，也是"上一次真正发出去的值"
        # （两者只有死区那一种情况会分开）。先存下来再覆盖。
        last_sent = self.param_trim
        self.param_trim = trim

        # 死区之内不发 —— 但"从非零回到 0"必须发。固件 Motion_Trim 记的是
        # 这一段已经落地的累计差速，不发这一帧 0，它就带着上一次那点差速
        # 把这一段走完（5 米的段能有三十秒），车会一直朝一边慢慢偏，而界面
        # 上看不出任何异常。发了也是 0 这句话只在"本来就是 0"时成立。
        if (abs(trim) >= lane_control.GAINS["DEADBAND_MM"]
                or (trim == 0.0 and last_sent != 0.0)):
            self._send(frame.build_trim(int(round(trim)), self._next_seq()))

    def _want_spray(self, quality):
        """这一帧该不该喷。带连续帧滞回，两个模式共用同一套判据。

        补喷走 RESPRAY 帧、巡线走 PUMP 帧，动作不同，但"哪一帧才算破损
        成立"必须是同一个判据。以前的差别是没理由的：巡线在投票之上还压
        一道 PUMP_ON_AFTER，补喷投票通过就直接喷。同一个缺口，站着看和
        边走边看给两个答案，调试时会觉得"这两个模式脾气不一样"。

        滞回在两边都必要，不只是巡线：
          开泵 —— 连 PUMP_ON_AFTER 帧判破损才动，滤掉单帧反光；
          关泵 —— 连 PUMP_OFF_AFTER 帧判完好才停，不然一处阴影就把泵
                  关了，接着又开，出漆断断续续。
        """
        if quality in (QUALITY_WORN, QUALITY_BREAK):
            self._bad_streak += 1
            self._good_streak = 0
        else:
            self._good_streak += 1
            self._bad_streak = 0

        if not self.pump_on:
            return self._bad_streak >= self.PUMP_ON_AFTER

        return self._good_streak < self.PUMP_OFF_AFTER

    def _apply_pump(self, want):
        """把"该不该喷"落到泵上。值没变就不发 —— 5Hz 的重发会把 ACK 表刷满。"""
        if want == self.pump_on:
            return
        self.pump_on = want
        self._send(frame.build_pump(want, self._next_seq()))

    def _update_pump(self, quality):
        """按三态控制泵，带滞回。

        UNSURE 一律关泵：看不到就不喷。这条比"多喷一点"重要得多 ——
        镜头糊了、车开出线外了，继续喷就是在空地上倒漆。
        """
        self._apply_pump(self._want_spray(quality))

    def on_status(self, state_char, pulse=0, flags=None):
        """接收下位机上行的状态帧，由主窗口转发进来。

        做两件事：
          1. 一段走完了就接下一段。没有这条的话，发完 5000mm 车就停在
             原地了，而界面还显示"正在巡线"。
          2. 记下"陀螺仪闭环有没有接管差速"，据此决定还要不要发 TRIM。

        必须看到过 'M' 才允许续段。只判断"现在是 W"是不够的：MOVE 被回绝
        （上电静默、状态不对、串口没通）时状态一直是 W，那样这里会每 0.5 秒
        刷一帧 MOVE，日志瞬间刷满，真正的问题被埋在中间。
        """
        previous = self._fw_state
        self._fw_state = state_char

        if flags is not None:
            self.fw_flags = flags
            # bit 0x10 = 固件那边航向闭环正在跑（见 SendStatusFrame）
            self.gyro_closed = bool(flags & 0x10)

        if not self.tracking:
            return

        if state_char != "W":
            return

        if previous != "M":
            return

        # 状态是 W 也可能是急停回来的。加一道时间闸：上一段刚发出去不到
        # 半秒就"已结束"，那是被别的东西打断了，不该接着发下一段。
        if time.monotonic() - self._last_segment_time < 0.5:
            return

        self._send_segment()

    # ==========================================================
    # PCA 主方向
    # ==========================================================
    def get_orientation_pca(self, points):
        """返回 (中心, 角度, vx, vy)。

        vx/vy 也一起返回，因为纠偏要的是"线的倾斜方向"，而 angle 把圆的
        另一半折叠掉了（-90~90），用它算航向会丢掉符号。见
        core/lane_control.heading_error_deg 的说明。
        """
        mean = np.mean(points, axis=0)

        centered = points - mean

        cov = np.cov(centered.T)

        # eigh 而不是 eig：协方差是对称阵，eigh 保证实数且更快；
        # eig 在这里偶尔会返回复数，后面 atan2 直接崩。
        eig_vals, eig_vecs = np.linalg.eigh(cov)

        main_dir = eig_vecs[:, int(np.argmax(eig_vals))]

        angle = np.arctan2(main_dir[1], main_dir[0]) * 180.0 / np.pi

        if angle > 90:
            angle -= 180

        elif angle < -90:
            angle += 180

        return mean, angle, float(main_dir[0]), float(main_dir[1])

    def is_line_shape(self, contour):

        rect = cv2.minAreaRect(contour)

        (w, h) = rect[1]

        if w == 0 or h == 0:
            return False

        aspect_ratio = max(w, h) / min(w, h)

        return aspect_ratio >= self.LINEAR_ASPECT_RATIO_THRESH

    # ==========================================================
    # 绘制定向矩形
    # ==========================================================
    def draw_oriented_rect(self, image, center, angle, length, width,
                           color, thickness=2):

        rect = np.array([
            [-length / 2, -width / 2],
            [length / 2, -width / 2],
            [length / 2, width / 2],
            [-length / 2, width / 2]
        ])

        rad = np.deg2rad(angle)

        rot = np.array([
            [np.cos(rad), -np.sin(rad)],
            [np.sin(rad), np.cos(rad)]
        ])

        rect_rot = np.dot(rect, rot.T) + center

        rect_rot = rect_rot.astype(np.int32)

        cv2.polylines(image, [rect_rot], True, color, thickness)

    # ==========================================================
    # 每帧
    # ==========================================================
    def update_frame(self):

        if self.cap is None:
            return

        ret, frame_in = self.cap.read()

        if not ret:
            self._camera_hiccup()
            return

        self._camera_fail_streak = 0
        self.camera_ok = True

        result = frame_in.copy()

        hsv = cv2.cvtColor(frame_in, cv2.COLOR_BGR2HSV)

        yellow_mask = cv2.inRange(hsv, self.HSV_YELLOW_LOW, self.HSV_YELLOW_HIGH)

        kernel_open = np.ones((3, 3), np.uint8)

        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, kernel_open)

        # 找轮廓要用闭运算补过裂缝的那份：破损处的黄漆被裂缝切成碎块，
        # 不补的话长宽比会塌到 1.x，is_line_shape 一个都选不中 ——
        # 于是烂得越狠越选不出来，正好反了。
        # 量覆盖率仍然用上面那份原始 yellow_mask，闭过的把掉漆也填平了。
        contours, _ = cv2.findContours(
            close_cracks(yellow_mask),
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        found_line = False

        if contours:

            contours_sorted = sorted(contours, key=cv2.contourArea, reverse=True)

            line_contour = None

            for cnt in contours_sorted:

                area = cv2.contourArea(cnt)

                if area < self.MIN_AREA:
                    continue

                if self.is_line_shape(cnt):
                    line_contour = cnt
                    break

            if line_contour is not None:

                found_line = True

                points = line_contour.reshape(-1, 2)

                center, angle, vx, vy = self.get_orientation_pca(points)

                rad = np.deg2rad(angle)

                dir_vec = np.array([np.cos(rad), np.sin(rad)])

                projections = np.dot(points, dir_vec)

                line_length = np.max(projections) - np.min(projections)

                rect_length = max(line_length + 20, 80)

                # ================= 平滑 =================
                cx, cy = int(center[0]), int(center[1])

                self.center_queue.append((cx, cy))

                smooth_x = int(np.mean([p[0] for p in self.center_queue]))
                smooth_y = int(np.mean([p[1] for p in self.center_queue]))

                # ================= 判据 =================
                # 抠出矩形范围内的【原始黄像素】再判。不能拿闭过的那份，
                # 也不能拿填实的剪影 —— 两者都把掉漆填平了，判出来永远是"完好"。
                line_pixels = extract_line_pixels(
                    yellow_mask,
                    (smooth_x, smooth_y),
                    angle,
                    rect_length,
                    self.RECT_WIDTH
                )

                quality, spray_level, reason = classify_line_quality(line_pixels)

                self.votes.append((quality, spray_level))
                voted = self._voted_quality()

                self.quality = voted
                self.quality_note = reason

                # ================= 误差 =================
                image_width = frame_in.shape[1]

                heading = lane_control.heading_error_deg(vx, vy)
                lateral = lane_control.lateral_error_ratio(smooth_x, image_width)

                self.heading_deg = heading
                self.lateral_ratio = lateral

                # ================= 闭环 =================
                if self.mode == self.MODE_RESPRAY:
                    # 和巡线用同一个"连续 N 帧"判据（见 _want_spray）：
                    # 投票已经把单帧抖动滤掉，这一层滤的是"投票窗口正好
                    # 卡在破损边缘"那种来回摆的情况 —— 补喷一发就是几百
                    # 毫米的漆，比巡线的开泵贵得多。
                    if self._want_spray(voted):
                        self.maybe_send_respray(self._voted_level(voted))

                elif self.mode == self.MODE_TRACK and self.tracking:
                    self._update_pump(voted)
                    self._track_tick(heading, lateral)

                # ================= 画 =================
                color = self._quality_color(voted)

                self.draw_oriented_rect(
                    result, (smooth_x, smooth_y), angle, rect_length,
                    self.RECT_WIDTH, color, thickness=3
                )

                cv2.putText(
                    result,
                    f"{QUALITY_NAMES.get(voted, '?')} L{spray_level}",
                    (max(0, smooth_x - 60), max(20, smooth_y - 20)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2
                )

                # 指向画面中心的方向标：纠偏向哪边一眼能看出来
                h, w = result.shape[:2]
                cv2.line(result, (w // 2, h - 20),
                         (w // 2 + int(-heading * 6), h - 20), (0, 255, 255), 2)

        if not found_line:
            # 这一帧没认出线（压根没轮廓，或者轮廓都不像线状）就把平滑队列清掉。
            # 不清的话，线跳走再回来、或者换到旁边那条，队列里最多 5 帧的旧中心
            # 会被和新中心一起平均，ROI 落在上一处 —— 判出来的破损也是那一处的。
            self.center_queue.clear()

            # 认不到线也要进投票：连续几帧看不到，结论就该变成"看不清"，
            # 而不是一直沿用最后一帧的"完好"。泵那边靠这个关掉。
            self.votes.append((QUALITY_UNSURE, 0))
            self.quality = self._voted_quality()
            self.quality_note = "No line"

            if self.mode == self.MODE_TRACK and self.tracking:
                self._update_pump(self.quality)

        self._draw_hud(result)

        # ======================================================
        # Qt 显示
        # ======================================================
        result = cv2.cvtColor(result, cv2.COLOR_BGR2RGB)

        h, w, ch = result.shape

        image = QImage(result.data, w, h, ch * w, QImage.Format_RGB888)

        pixmap = QPixmap.fromImage(image)

        self.setPixmap(pixmap.scaled(self.size(), Qt.KeepAspectRatio,
                                     Qt.SmoothTransformation))

    # ==========================================================
    # 画面上的状态字
    # ==========================================================
    def _quality_color(self, quality):
        if quality in (QUALITY_WORN, QUALITY_BREAK):
            return (0, 0, 255)
        if quality == QUALITY_OK:
            return (0, 255, 0)
        return (0, 200, 255)      # UNSURE：黄，表示"我没看清，不是在报好"

    def _draw_hud(self, image):
        lines = [self.quality_note]

        if self.mode == self.MODE_RESPRAY:
            lines.append(f"respray x{self.spray_count} L{self.last_level}")

        elif self.mode == self.MODE_TRACK:
            lines.append(
                f"head {self.heading_deg:+.1f}deg  lat {self.lateral_ratio:+.2f}"
            )
            lines.append(
                f"trim {self.param_trim:+.1f}mm  pump={'ON' if self.pump_on else 'off'}"
            )
            lines.append(
                f"{'TRACK' if self.tracking else 'idle'} seg#{self._segments_sent} "
                f"fw={self._fw_state}"
            )

        for index, text in enumerate(lines):
            cv2.putText(image, text, (10, 30 + index * 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)

    # ==========================================================
    # 关闭事件
    # ==========================================================
    def closeEvent(self, event):

        self.stop_camera()

        event.accept()
