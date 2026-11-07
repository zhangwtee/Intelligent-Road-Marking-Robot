import serial
import threading
import time

from PyQt5.QtCore import QObject, pyqtSignal


class SerialManager(QObject):

    # 接收到数据
    data_received = pyqtSignal(bytes)

    # 错误信号
    error_signal = pyqtSignal(str)

    # 默认值和 main.py 的 PORT 保持一致（/dev/serial0 = 排针串口，见那边的注释）。
    # main.py 总是显式传进来，这儿只是别留第二个会过期的默认值。
    def __init__(self, port="/dev/serial0", baudrate=9600):
        super().__init__()

        self.port = port
        self.baudrate = baudrate

        self.ser = None
        self.running = False

    # ================= 打开串口 =================
    def open(self):

        try:
            self.ser = serial.Serial(
                self.port,
                self.baudrate,
                timeout=0.1
            )

            self.running = True

            threading.Thread(
                target=self._read_loop,
                daemon=True
            ).start()

            print(f"[串口] 已打开: {self.port} @ {self.baudrate}")

        except Exception as e:
            self.error_signal.emit(str(e))
            print("[串口] 打开失败:", e)

    # ================= 接收线程 =================
    def _read_loop(self):

        while self.running:

            try:

                if self.ser:

                    # read(1024) 会在 timeout 模式下「等到收满 1024 字节或超时」
                    # 才返回。下位机一次只回 1~10 个字节，永远凑不满 1024，
                    # 于是每次都要等足 100ms —— 100ms 就是接收延迟的硬上限。
                    # 有积压就只读积压量，没有就阻塞等 1 个字节。
                    data = self.ser.read(self.ser.in_waiting or 1)

                    if data:

                        # 发 Qt 信号
                        self.data_received.emit(data)

                time.sleep(0.005)

            except Exception as e:

                # 拔线之后 read() 每次都会立刻抛，而这里原来既不退避，
                # 也不把 self.ser 置空 —— 于是每秒上万次 emit + print，
                # CPU 打满、日志刷屏，而且永远不会重连。线程直接退出，
                # 状态交回上层：一个已经停掉的接收线程 + 一条 error 信号，
                # 比一个还在空转的线程更容易被发现，也不会掩盖真正的故障。
                self.error_signal.emit(str(e))
                print("[串口] 读取异常，接收线程退出:", e)

                self.running = False
                self.ser = None
                break

    # ================= 发送 =================
    def send(self, data: bytes) -> bool:

        # 串口没开（或已断开）时，原来这里 if self.ser 为假就直接返回，
        # 不 emit 任何东西 —— 用户按了按钮、日志写着「已发送」，车一动不动，
        # 而且没有任何提示。静默失败比报错更难查，所以这条必须走 error_signal。
        #
        # 返回值给调用方判断：这一帧到底出去没有。速度那一路要拿它决定
        # 记不记账，见 main_window 的 _send_speed。
        if not self.ser:
            self.error_signal.emit("串口未打开，发送被丢弃")
            print("[串口] 未打开，丢弃发送:", data)
            return False

        try:

            self.ser.write(data)
            print("[发送]", data)
            return True

        except Exception as e:

            self.error_signal.emit(str(e))
            print("[串口] 发送失败:", e)
            return False

    # ================= 关闭 =================
    def close(self):

        self.running = False

        if self.ser:

            self.ser.close()

            print("[串口] 已关闭")