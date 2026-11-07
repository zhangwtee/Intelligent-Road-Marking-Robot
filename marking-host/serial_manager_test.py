"""串口接收线程的自检。不需要真串口 —— 用一个"拔掉的端口"模拟。

    python3 serial_manager_test.py

拔线之后 read() 每次都会立刻抛。原来的异常分支既不退避、也不把 ser 置空，
于是每秒上万次 emit + print：CPU 打满、日志刷屏，而且永远不会重连。
逻辑一坏就在这里响。
"""

import sys
import types

from PyQt5.QtCore import QCoreApplication

# serial_manager 顶层就 import serial，而这台开发机上未必装了 pyserial
# （树莓派上有）。这个测试只碰 _read_loop 的异常路径、不打开任何端口，
# 用个空壳顶上就够了 —— 装了的话还是用真的。
try:
    import serial  # noqa: F401
except ImportError:
    sys.modules["serial"] = types.ModuleType("serial")

from core.serial_manager import SerialManager


class DeadPort:
    """模拟拔掉的串口：下次 read 立刻抛，in_waiting 恒 0。"""

    def __init__(self):
        self.reads = 0

    @property
    def in_waiting(self):
        return 0

    def read(self, n):
        self.reads += 1
        raise OSError("device reports readiness to read but returned no data")


def test_read_loop_exits_when_the_port_dies():
    """读到异常就退出线程，不能空转。

    空转的后果不只是 CPU：它每秒把几千条 error 信号打进日志，真正的故障
    （比如"一直是这个错"）反而被淹掉。
    """
    m = SerialManager()
    port = DeadPort()
    m.ser = port
    m.running = True

    errors = []
    m.error_signal.connect(errors.append)

    # 放进线程里跑并限时：逻辑坏了的时候 _read_loop 不会返回（读到异常
    # 继续 while，一秒上万轮），直接在主线程调用会把测试挂死，看不出是哪条。
    import threading

    done = []
    th = threading.Thread(
        target=lambda: (m._read_loop(), done.append(True)), daemon=True)
    th.start()
    th.join(2.0)

    assert done, "_read_loop 没退出 —— 端口已经死了它还在空转"
    assert m.running is False, "线程退出了但 running 还是 True"
    assert m.ser is None, "死掉的句柄还留着，send() 会继续往里写"

    # 只看 read() 被调了几次，不看 errors：信号跨线程是排队派发的，这个测试
    # 没有事件循环，收到 0 条不代表没 emit。
    assert port.reads == 1, f"读了 {port.reads} 次，说明报完错还在转"


def test_send_after_the_port_died_reports_failure():
    """句柄清掉之后 send 必须如实返回 False。

    上层拿这个返回值决定"这条命令记不记账"。返回 True 的话，日志写着
    "已发送"而车一动不动。
    """
    m = SerialManager()
    m.ser = None

    errors = []
    m.error_signal.connect(errors.append)

    assert m.send(b"T") is False
    assert errors, "沉默的失败比报错更难查"


if __name__ == "__main__":
    QCoreApplication(sys.argv)

    for name in sorted(n for n in globals() if n.startswith("test_")):
        globals()[name]()
        print("ok", name)

    print("全部通过")
