"""
标线破损判据。纯 cv2/numpy，不依赖 Qt —— 所以 vision_test.py 在哪台机器上都能跑。

判据两条，都算在"标线自己的坐标系"里，所以远近一个标准：
  coverage  该有漆的地方实际有多少漆 —— 掉漆、褪色、缺块
  gap       有没有一整段断掉 —— 断裂

标定方法：在真实路面上跑，画面左上角会打出 "cov=.. gap=.. w=.." 三个数。
在"完好的线"和"该补的线"上各记几组，再回来改 THRESHOLDS。
下面这组数是从 4960.jpg_wh860.jpg 那张俯视实拍图量的底，现场又按实际路面的
覆盖率收的 —— 换路面、换漆色、换相机高度之后照着上面那套再走一遍。
"""

import cv2
import numpy as np


# ---------------- 标定参数（全工程唯一一份）----------------
THRESHOLDS = {
    # 闭运算核，用来补裂缝。裂缝竖向断口长度实测 中位数 3px、90分位 15px，
    # 所以高度取 15。宽度不能超过两条平行黄线之间的间隔（实测 12~25px），
    # 否则两条会被粘成一条，所以取 3。
    "CLOSE_KERNEL": (3, 15),

    # 分析窗口最长多少像素。别让它无限长：窗口越长，一段破损占的比例越小，
    # 断裂判据 MAX_GAP_RATIO 就越难触发。
    "MAX_ANALYSIS_LEN": 300.0,

    "SLAB_COVERAGE_MIN": 0.35,  # 单行像素数低于"典型满行"的这个比例 = 这行没漆
    "MIN_COVERAGE": 0.55,       # 沿线平均覆盖率低于此 = 掉漆
    "MAX_GAP_RATIO": 0.10,      # 最长连续没漆段占全长比例超过此 = 断裂

    "MIN_LINE_PIXELS": 150,     # 像素数低于此不当它是标线
    "MIN_LINE_LENGTH": 30.0,    # 沿线长度低于此不当它是标线
    "MIN_ROW_PIXELS": 3.0,      # 典型满行的像素数低于此 = 太细，不当它是标线
}


# ---------------- 补喷档位 ----------------
# 下位机 '1'~'5' 对应 0.2 / 0.4 / 0.6 / 0.8 / 1.0 m（system.c 的 HandleCmd
# 里按 200mm × 档位算）。只给单字节老路径和这里的档位换算用：帧 RESPRAY
# 直接带毫米，不受档位约束。
SPRAY_LEVELS = 5
SPRAY_MM_PER_LEVEL = 200


# ---------------- 检测结论（四态，不是两种） ----------------
# 改造前 evaluate_line_quality 只回答"补 / 不补"。这个二值输出把两件完全
# 不同的事混成了一件：
#
#   线是完好的      -> 看清楚了，不需要补
#   根本没看清      -> 什么都没看到，也不知道该不该补
#
# 两种情况都返回 False。于是"相机被挡住""线太细""画面全是反光"和"这条线
# 画得很好"在下位机看来一模一样 —— 都是不发命令。现场的表现是"它不工作"，
# 但日志里一切正常。
#
# 分开之后，上层才能对"看不清"做正确的事：不喷，也不当成正常，而且要报出来。
QUALITY_OK = "ok"            # 看清了，完好
QUALITY_WORN = "worn"        # 看清了，磨薄了
QUALITY_BREAK = "break"      # 看清了，断了
QUALITY_UNSURE = "unsure"    # 没看清 —— 不下结论

QUALITY_NAMES = {
    QUALITY_OK: "完好",
    QUALITY_WORN: "磨损",
    QUALITY_BREAK: "断裂",
    QUALITY_UNSURE: "看不清",
}


def recommend_spray_level(coverage, gap):
    """把"破了多少"折算成"补多长"，返回 1~5（下位机 '1'~'5'）。

    两种损伤各算一个 0~1 的严重度，取大的：
      Worn  覆盖率从 MIN_COVERAGE(0.55) 掉到 0 —— 刚好该补是 0，掉光才是 1
      Break 断口比从 MAX_GAP_RATIO(0.10) 涨到 1.0 —— 刚好该补是 0，全断才是 1

    为什么不只用一种：一条线可能覆盖率还高但中间断了一大截，也可能没断
    但整片磨薄了，是两类损伤。取 max 保证任一种严重都补得够。

    阈值从 THRESHOLDS 读，不在这儿再写一份数字 —— 否则改判据的时候
    档位换算会悄悄跟着不同步。
    """
    th = THRESHOLDS
    worn = (th["MIN_COVERAGE"] - coverage) / th["MIN_COVERAGE"]
    broken = (gap - th["MAX_GAP_RATIO"]) / (1.0 - th["MAX_GAP_RATIO"])

    severity = max(0.0, min(1.0, max(worn, broken)))

    return 1 + int(severity * (SPRAY_LEVELS - 1) + 0.5)


def close_cracks(yellow_mask):
    """闭运算补裂缝。**找轮廓必须用这份。**

    破损处的黄漆被裂缝啃成几百块小方片，最大轮廓的长宽比会塌到 1.x，
    is_line_shape 要求的 4.0 一个都过不了 —— 于是烂得越狠越选不出来，
    正好反了。补上闭运算，3x3 就能把长宽比从 1.34 拉回 7.35。

    注意：这只用来【找轮廓】。量覆盖率仍然要用没闭过的原始黄像素，
    闭运算会把掉漆一并填平，正则判据就废了。
    """
    kernel = cv2.getStructuringElement(
        cv2.MORPH_RECT, THRESHOLDS["CLOSE_KERNEL"]
    )
    return cv2.morphologyEx(yellow_mask, cv2.MORPH_CLOSE, kernel)


def extract_line_pixels(yellow_mask, center, angle, length, width):
    """把标线定向矩形范围内的【原始黄像素】抠出来，矩形外的一律不算。

    必须是原始像素，不能是填实的剪影 —— 缺了多少漆正是靠数这些像素数出来的。
    """
    half_len, half_w = length / 2.0, width / 2.0

    rect = np.array([
        [-half_len, -half_w],
        [half_len, -half_w],
        [half_len, half_w],
        [-half_len, half_w],
    ])

    rad = np.deg2rad(angle)

    rot = np.array([
        [np.cos(rad), -np.sin(rad)],
        [np.sin(rad), np.cos(rad)],
    ])

    rect = (
        np.dot(rect, rot.T) + np.asarray(center, np.float64)
    ).astype(np.int32)

    roi = np.zeros(yellow_mask.shape, np.uint8)

    cv2.fillPoly(roi, [rect], 255)

    return cv2.bitwise_and(yellow_mask, yellow_mask, mask=roi)


def classify_line_quality(line_mask):
    """返回 (state, spray_level, 说明)。line_mask 必须是原始黄像素。

    state 是 QUALITY_OK / WORN / BREAK / UNSURE 之一。
    spray_level 只在 WORN / BREAK 时有效（1~5），其它情况是 0。

    说明里带 cov/gap/w 三个数，画面直接显示，标定就靠它。
    """
    th = THRESHOLDS

    ys, xs = np.nonzero(line_mask)

    if len(xs) < th["MIN_LINE_PIXELS"]:
        return QUALITY_UNSURE, 0, "No line"

    pts = np.stack([xs, ys], 1).astype(np.float32)

    rel = pts - pts.mean(0)

    # 2x2 协方差的主方向就是标线走向。用的是相对均值的坐标，
    # 所以图像坐标系怎么转都不影响结果。
    evals, evecs = np.linalg.eigh(rel.T @ rel)

    axis = evecs[:, int(np.argmax(evals))]   # 沿线方向

    t = rel @ axis

    # 只取中间一段，别让窗口无限长：越长，一段破损占的比例越小，
    # 断裂判据 MAX_GAP_RATIO 就越难触发
    span = float(t.max() - t.min())

    if span > th["MAX_ANALYSIS_LEN"]:
        mid = float(np.median(t))
        keep = np.abs(t - mid) <= th["MAX_ANALYSIS_LEN"] / 2.0
        t = t[keep]
        span = float(t.max() - t.min())

    if span < th["MIN_LINE_LENGTH"]:
        return QUALITY_UNSURE, 0, "Too short"

    # 沿线逐行数黄像素，分母取"典型满行"的像素数 —— 从实际行统计里来。
    # 所以双黄线中间那 12~25px 的空隙不会被算成没漆（两条线都在，每行自然就多），
    # 线弯了也没关系（弯了也每一行都有漆）。
    t0 = float(t.min())

    # 分箱宽度取 2 像素，而不是 1。
    #
    # 原来的写法是 np.bincount((t - t0).astype(np.int32))：整数截断等于把
    # 沿线投影按 1px 一格切。线是斜的时候（45 度），沿线相邻像素的投影间距
    # 是 1.41，截断之后正好每隔一格空一次 —— 一根完好的 45 度线只有 0.7 的
    # 覆盖率，30% 的磨损会被判成 50%。取 2px 之后任何角度下每格都落得到点；
    # 缺口判据要的是"整段没漆"，宽度远大于 2px，不受影响。
    bin_px = 2.0
    n_bins = max(8, int(span / bin_px) + 1)
    rows, _ = np.histogram(t, bins=n_bins)

    full = float(np.percentile(rows[rows > 0], 75))

    # 阈值的意思还是"每像素线宽能有几个像素"，箱宽变成 2 就要换算回去，
    # 否则"线太细"这条会跟着放松一倍。
    if full / bin_px < th["MIN_ROW_PIXELS"]:
        return QUALITY_UNSURE, 0, "Too thin"

    cover = np.clip(rows / full, 0.0, 1.0)

    coverage = float(cover.mean())

    # 最长连续"没漆段"，按全长的比例算 —— 这样远近一个标准
    longest = run = 0

    for empty in cover < th["SLAB_COVERAGE_MIN"]:
        run = run + 1 if empty else 0
        longest = max(longest, run)

    gap = longest / len(cover)

    note = f"cov={coverage:.2f} gap={gap:.2f} w={full:.0f}"

    level = recommend_spray_level(coverage, gap)

    if gap > th["MAX_GAP_RATIO"]:
        return QUALITY_BREAK, level, "Break " + note

    if coverage < th["MIN_COVERAGE"]:
        return QUALITY_WORN, level, "Worn " + note

    return QUALITY_OK, 0, "OK " + note


def evaluate_line_quality(line_mask):
    """兼容旧调用：返回 (need_paint, spray_level, 说明)。

    新代码请用 classify_line_quality —— 它能把"看不清"和"完好"分开，
    这个包装做不到（返回 False 的两种含义本来就不同）。
    """
    state, level, note = classify_line_quality(line_mask)
    return state in (QUALITY_WORN, QUALITY_BREAK), level, note
