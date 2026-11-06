"""四路脉冲输出的配置检查。

四个轮子从「一个 TIM3 的四个通道」改成「四个定时器各出一路」，这件事的
错法都很安静，不会编译报错，只会让某一个轮子不动或者转弯变成拖胎：

    M3 配到 TIM5_CH1  -> PA0 和 M2 抢同一个脚，两个轮子一起乱转
    TIM8 忘了开 MOE   -> CCR、CCER 全对，PC6 一个脉冲都没有
                         现象是「三路转、一路不转」，最像虚焊
    中断函数名写错     -> 强符号没定义，落到启动文件的 B . 死循环
                         电机转到一半突然全停，看不出和代码有关
    四路又退回同一个   -> 编译通过、能跑，但差速重新变成"内侧硬停"
    起坡四路同一个起点 -> 比例在起坡段被抹平，短的一侧反而跑得快

这几条都能从源码静态看出来，不需要上电。
"""
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
FIRMWARE = ROOT.parent / "marking-firmware"
SM_C = FIRMWARE / "Hardware" / "StepMotor_New.c"
SM_H = FIRMWARE / "Hardware" / "StepMotor_New.h"
STARTUP = FIRMWARE / "Start" / "startup_stm32f10x_hd.s"

TESTS = []


def test(fn):
    TESTS.append(fn)
    return fn


def read_gbk(path):
    return path.read_bytes().decode("gbk", errors="replace")


def pul_table():
    """从 s_m 的静态初始化里取出 (定时器, 通道号)。"""
    text = read_gbk(SM_C)
    block = re.search(r"s_m\[NUM_MOTORS\]\s*=\s*\{(.*?)\};", text, re.S)
    assert block, "找不到 s_m 的初始化表"
    rows = re.findall(r"\{\s*(TIM\d+)\s*,\s*(\d)\s*,\s*(TIM_CCER_CC\dE)", block.group(1))
    return [(t, int(ch), ccen) for t, ch, ccen in rows]


def pin_of(name):
    """把 Mx_PUL_PIN / Mx_DIR_PIN 解析成 (端口, 脚号)。"""
    text = read_gbk(SM_H)
    port = re.search(rf"#define\s+{name}_PORT\s+(GPIO[A-G])", text)
    pin = re.search(rf"#define\s+{name}_PIN\s+GPIO_Pin_(\d+)", text)
    assert port and pin, f"{name} 的引脚宏不全"
    return port.group(1), int(pin.group(1))


@test
def test_four_motors_use_four_timers():
    """四路必须是四个不同的定时器，否则又回到共用 ARR。"""
    table = pul_table()
    assert len(table) == 4, f"初始化表只有 {len(table)} 路"
    tims = [t for t, _, _ in table]
    assert len(set(tims)) == 4, f"定时器重复: {tims}"


@test
def test_tim5_uses_channel_2():
    """TIM5_CH1 在 F103 上是 PA0，和 TIM2_CH1 抢脚，所以 M3 必须走 CH2。"""
    table = pul_table()
    ch_of = {t: ch for t, ch, _ in table}
    assert ch_of.get("TIM5") == 2, f"TIM5 用的是通道 {ch_of.get('TIM5')}，应为 2"


@test
def test_channel_matches_enable_bit():
    """ch 号和 CCER 使能位必须对得上，否则关通道时关错一个。"""
    for tim, ch, ccen in pul_table():
        expect = f"TIM_CCER_CC{ch}E"
        assert ccen == expect, f"{tim} 的 ch={ch} 却配了 {ccen}"


@test
def test_pulse_pins_do_not_overlap():
    """四路脉冲脚两两不同。"""
    pins = [pin_of(f"M{i}_PUL") for i in range(1, 5)]
    assert len(set(pins)) == 4, f"脉冲脚重复: {pins}"


@test
def test_pulse_pins_do_not_collide_with_dir():
    """脉冲脚不能和任何 DIR 脚撞。"""
    pulses = {pin_of(f"M{i}_PUL") for i in range(1, 5)}
    dirs = {pin_of(f"M{i}_DIR") for i in range(1, 5)}
    clash = pulses & dirs
    assert not clash, f"脉冲脚和 DIR 撞了: {clash}"


@test
def test_all_four_handlers_defined():
    """四个中断服务函数都要有定义，否则落到启动文件的 B . 死循环。"""
    text = read_gbk(SM_C)
    for name in ("TIM2_IRQHandler", "TIM3_IRQHandler",
                 "TIM5_IRQHandler", "TIM8_UP_IRQHandler"):
        assert re.search(rf"void\s+{name}\s*\(\s*void\s*\)", text), f"{name} 没定义"


@test
def test_handlers_exist_in_startup():
    """中断向量表里也要有这四个入口（hd 版才有 TIM5/TIM8）。"""
    text = read_gbk(STARTUP) if STARTUP.exists() else ""
    if not text:
        text = STARTUP.read_text(encoding="utf-8", errors="replace")
    for name in ("TIM2_IRQHandler", "TIM3_IRQHandler",
                 "TIM5_IRQHandler", "TIM8_UP_IRQHandler"):
        assert name in text, f"启动文件里没有 {name} 向量"


@test
def test_tim8_master_output_enabled():
    """TIM8 是高级定时器，不开 MOE 就一个脉冲都没有。"""
    text = read_gbk(SM_C)
    assert re.search(r"TIM_CtrlPWMOutputs\s*\(\s*TIM8\s*,\s*ENABLE\s*\)", text), \
        "TIM8 没有调用 TIM_CtrlPWMOutputs"


@test
def test_arr_written_per_channel():
    """每路按自己的通道号写 CCR，不能四个 CCR 一起写。"""
    text = read_gbk(SM_C)
    assert "CCR1" in text and "CCR2" in text, "没有按通道分派 CCR"
    for dead in ("TIM3->CCR1", "TIM3->CCR2", "TIM3->CCR3", "TIM3->CCR4"):
        assert dead not in text, f"还有旧写法 {dead}"


@test
def test_ramp_state_per_motor():
    """斜坡状态必须每路一份，不能回到全局单套。"""
    text = read_gbk(SM_C)
    struct = re.search(r"typedef struct\s*\{(.*?)\}\s*MotorCh_t", text, re.S)
    assert struct, "找不到 MotorCh_t"
    body = struct.group(1)
    for field in ("ramp_on", "ramp_state", "ramp_period", "ramp_decel"):
        assert field in body, f"MotorCh_t 里没有 {field}，斜坡又变回全局了"


@test
def test_speed_split_api_declared():
    """分路调速的接口要在头文件里，motion 层才用得上。"""
    text = read_gbk(SM_H)
    for fn in ("StepMotor_SetSpeedLR", "StepMotor_SetSpeedOf",
               "StepMotor_GetSpeedOf", "StepMotor_GetPulseOf"):
        assert fn in text, f"头文件缺 {fn}"


@test
def test_ramp_start_is_scaled_per_motor():
    """起坡起点要按左右比例缩放，不能四路都从同一个 ARR 起。

    斜坡是按脉冲推进的：两路同时从 1800 出发、每步降同样的量，起坡段就
    完全同速，左右比例被抹平；短的那一路若没走出起坡段就停了，等于全程
    同速跑，比目标更快，车头往反方向偏。
    """
    text = read_gbk(SM_C)

    helper = re.search(r"static uint16_t Motor_RampStartArr\(uint8_t i\)\s*\{(.*?)\n\}", text, re.S)
    assert helper, "找不到 Motor_RampStartArr()"
    body = helper.group(1)
    assert "RAMP_SOFT_ARR" in body and "/ slow" in body, \
        "起点没有按最慢那一路做比例缩放"

    ramp = re.search(r"static void Motor_RampStart\(.*?\n\}", text, re.S)
    assert ramp, "找不到 Motor_RampStart"
    fn = ramp.group(0)
    assert "Motor_RampStartArr(i)" in fn, "起坡没用按比例算出来的起点"
    assert re.search(r"Motor_ApplyArr\(i,\s*start\)", fn), "起坡首帧不是从 start 写"
    assert "ramp_start" in fn, "没记住本路起点，减速段就没有回退目标"


@test
def test_ramp_steps_are_proportional():
    """斜坡每步按当前 ARR 的比例推进，而不是固定减 100。

    固定步长下，起点和目标都不同的两路会走成不同步数 —— 短的一路先到位，
    剩下的时间比例就偏了。按比例时两边步数相同，比例整段恒定。
    """
    text = read_gbk(SM_C)
    assert re.search(r"#define\s+RAMP_SOFT_SHIFT\s+\d+", text), "没有比例推进的位移宏"

    isr = re.search(r"static void Motor_ISR\(uint8_t i\)\s*\{(.*?)\n\}", text, re.S)
    assert isr, "找不到 Motor_ISR"
    body = isr.group(1)

    assert body.count(">> RAMP_SOFT_SHIFT") >= 2, "加减档没有都改成按比例"
    assert "arr_now - RAMP_SOFT_STEP_ARR" not in body, "加速还是固定步长"
    assert "arr_now + RAMP_SOFT_STEP_ARR" not in body, "减速还是固定步长"

    # 减速要回到本路的起点，不是固定的 RAMP_SOFT_ARR
    assert "a > m->ramp_start" in body, "减速上限没按本路起点夹"
    assert "a > RAMP_SOFT_ARR" not in body, "减速还在往 1800 拉，分速时会把比例拉歪"


def main():
    for fn in TESTS:
        try:
            fn()
        except AssertionError as exc:
            print(f"FAIL {fn.__name__}: {exc}")
            return 1
        print(f"ok {fn.__name__}")
    print("全部通过")
    return 0


if __name__ == "__main__":
    sys.exit(main())
