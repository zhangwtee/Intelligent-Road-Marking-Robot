"""纠偏控制律自检。

    python3 lane_control_test.py     # 或 pytest lane_control_test.py

控制律错的时候，机器不会报错：车会走出一条越来越弯的弧，看着像"机械
问题"。所以这里除了符号和限幅，还有一个闭环仿真 —— 把车 + 控制器接起来
跑几百段，看它收敛还是发散。发散的那条曲线在真车上要跑 5 米才看得出来，
在这里 200 行就看得出来。
"""

import math

from core import lane_control as lc


# ---------------------------------------------------------------- 符号
def test_heading_sign_is_right_positive():
    """线在画面里向右倾 -> 航向偏差为正 -> 车该往右修。

    画面坐标：x 向右，y 向下，车头朝 y 减小方向。单位向量 (vx, vy)。
    """
    # 完全竖直：v 沿 y 轴，不管朝上朝下，偏差都该是 0
    assert abs(lc.heading_error_deg(0.0, 1.0)) < 1e-6
    assert abs(lc.heading_error_deg(0.0, -1.0)) < 1e-6

    # 向右倾 30 度
    assert abs(lc.heading_error_deg(math.sin(math.radians(30)),
                                    math.cos(math.radians(30))) - 30.0) < 1e-6

    # 向左倾 30 度
    assert abs(lc.heading_error_deg(-math.sin(math.radians(30)),
                                    math.cos(math.radians(30))) + 30.0) < 1e-6


def test_heading_survives_pca_sign_flip():
    """PCA 的主轴没有正负，v 和 -v 是同一条线，结果必须一样。

    这条不是废话：numpy 的 eigh 在不同数据上给出的符号是不确定的，
    "同一根线有时 +30 度有时 -30 度"会让车左右乱打。
    """
    for deg in (-45, -10, 5, 30, 80):
        vx = math.sin(math.radians(deg))
        vy = math.cos(math.radians(deg))
        assert abs(lc.heading_error_deg(vx, vy) - lc.heading_error_deg(-vx, -vy)) < 1e-6


def test_lateral_sign():
    """线在画面右侧 -> 车相对线偏左 -> 该往右修 -> 值为正。"""
    assert lc.lateral_error_ratio(640, 640) > 0.9
    assert lc.lateral_error_ratio(0, 640) < -0.9
    assert abs(lc.lateral_error_ratio(320, 640)) < 1e-9


def test_lateral_is_clamped():
    """线跑出画面时不能给出 3.0 —— 归一化就是为了让它有上下界。"""
    assert lc.lateral_error_ratio(5000, 640) == 1.0
    assert lc.lateral_error_ratio(-5000, 640) == -1.0


# ---------------------------------------------------------------- 限幅
def test_deadband():
    """偏得很少就不动。开环步进每改一次目标都是一次真实机械动作。"""
    trim = lc.trim_mm(heading_deg=0.1, lateral_ratio=0.0)
    assert trim == 0.0


def test_output_is_capped():
    trim = lc.trim_mm(heading_deg=45.0, lateral_ratio=1.0, previous=20.0)
    assert -lc.GAINS["MAX_TRIM_MM"] <= trim <= lc.GAINS["MAX_TRIM_MM"]


def test_cap_matches_firmware():
    """超过固件上限，帧会被判越界回 ERR_RANGE，车的表现是"完全不纠"。"""
    from core import frame
    assert lc.GAINS["MAX_TRIM_MM"] <= frame.TRIM_MAX_MM


def test_step_is_rate_limited():
    """一帧坏数据不能把差速从 0 直接打到 20mm。"""
    step = lc.trim_mm(heading_deg=30.0, lateral_ratio=1.0, previous=0.0)
    assert abs(step) <= lc.GAINS["MAX_STEP_MM"] + 1e-9


def test_rate_limit_is_symmetric():
    """从满打回零，也得一段一段退。"""
    step = lc.trim_mm(heading_deg=0.0, lateral_ratio=0.0, previous=20.0)
    assert 20.0 - step <= lc.GAINS["MAX_STEP_MM"] + 1e-9


# ---------------------------------------------------------------- 分段
def test_segments_stay_symmetric():
    """差速摊到两侧，车中心走的还是 base，不是 base±半个差速。"""
    left, right = lc.segment_lengths(200.0, 12.0)
    assert abs((left + right) / 2.0 - 200.0) < 1e-9
    assert abs((left - right) - 12.0) < 1e-9


def test_positive_trim_turns_right():
    """trim > 0 = 向右，左边该走得多。符号反了车会越纠越歪。"""
    left, right = lc.segment_lengths(200.0, 10.0)
    assert left > right


def test_stall_detection():
    """段长 10mm、差速 20mm：内侧轮目标 = 10 - 10 = 0，固件那边等于停机。"""
    assert lc.will_stall(10.0, 20.0) is True
    assert lc.will_stall(200.0, 20.0) is False


# ---------------------------------------------------------------- 闭环仿真
def _simulate(theta_deg, offset_m, base_mm, segments, scale_m=0.30):
    """把车和控制器接起来跑。

    theta_deg  车头相对标线的偏角，正 = 向右偏
    offset_m   车的横向偏移，正 = 已向右偏出

    几何关系（这是整个仿真唯一需要想清楚的地方）：
      车头右偏 theta  ->  相机里线向左倾  ->  heading = -theta
      车向右移 x      ->  线在画面左侧    ->  lateral = -x/scale

    两条都是"看到和实际反号"，所以控制器的输出天然是"往回打"的，
    不需要额外取反。要是仿真里发现要取反才能收敛，说明真车上也得取反 ——
    那意味着符号约定错了，不是调参能救的。
    """
    theta = theta_deg
    x = offset_m
    prev = 0.0
    trace = []

    for _ in range(segments):
        heading = -theta
        lateral = max(-1.0, min(1.0, -x / scale_m))

        prev = lc.trim_mm(heading, lateral, previous=prev)

        # 这一段走完，车头转了多少
        theta += lc.trim_to_angle_deg(prev)

        # 车头有偏角，向前走一段就产生横向位移
        x += (base_mm / 1000.0) * math.sin(math.radians(theta))

        trace.append((theta, x, prev))

    return trace


def test_closed_loop_converges_from_heading_error():
    """车头偏 6 度起步：必须收敛，而且不能来回振荡。"""
    trace = _simulate(theta_deg=6.0, offset_m=0.0, base_mm=200.0, segments=60)

    theta_end, x_end, _ = trace[-1]

    assert abs(theta_end) < 0.5, f"航向没收敛: {theta_end:.2f} 度"
    assert abs(x_end) < 0.01, f"横向没收敛: {x_end * 1000:.1f} mm"


def test_closed_loop_converges_from_lateral_offset():
    """车头是正的，但整体偏出 12cm：平行偏移也要能纠回来。

    这条专门验证横向项有用 —— 只有航向项的话，这里会一直平行错开下去，
    曲线看起来"很稳"，其实一直是废品。
    """
    trace = _simulate(theta_deg=0.0, offset_m=0.12, base_mm=200.0, segments=80)

    theta_end, x_end, _ = trace[-1]

    assert abs(x_end) < 0.02, f"横向偏移没纠回来: {x_end * 1000:.1f} mm"


def test_closed_loop_does_not_oscillate():
    """收敛过程中方向反转不能太多次。

    比例增益调大了会蛇形：往左打过、再往右打过。真车上看是"车头在抖"，
    而且每抖一次轮胎就打滑一次，开环步进丢的步再也补不回来。
    """
    trace = _simulate(theta_deg=5.0, offset_m=0.0, base_mm=200.0, segments=60)

    signs = [1 if t[2] > 0 else (-1 if t[2] < 0 else 0) for t in trace]
    signs = [s for s in signs if s != 0]

    flips = sum(1 for a, b in zip(signs, signs[1:]) if a != b)

    assert flips <= 4, f"来回打了 {flips} 次，增益太大"


def test_closed_loop_is_not_slower_than_the_machine():
    """从 6 度纠回来不能超过 1.5 米。

    车的实际速度 0.16m/s，一段 200mm 要 1.25 秒。纠偏超过 1.5 米（7 段、
    9 秒）就追不上了 —— 线已经画歪了，再纠回来只是把歪的那段留在路上。
    """
    trace = _simulate(theta_deg=6.0, offset_m=0.0, base_mm=200.0, segments=60)

    for index, (theta, _, _) in enumerate(trace):
        if abs(theta) < 0.5:
            assert index * 200 <= 1500, f"要 {index * 200}mm 才纠回来，太慢"
            return

    raise AssertionError("60 段之内没收敛")


# ---------------------------------------------------------------- 边界
def test_no_nan_on_degenerate_input():
    """线只剩一个点、或者宽高为 0 的时候不能吐 nan —— nan 会一路传进
    固件参数，最后表现成"车一动不动"，而且日志里看不出原因。"""
    for vx, vy in ((0.0, 0.0), (1.0, 0.0), (0.0, 1.0)):
        value = lc.heading_error_deg(vx, vy)
        assert not math.isnan(value)

    assert lc.lateral_error_ratio(0, 0) == 0.0


if __name__ == "__main__":
    for name in sorted(n for n in globals() if n.startswith("test_")):
        globals()[name]()
        print("ok", name)
    print("全部通过")
