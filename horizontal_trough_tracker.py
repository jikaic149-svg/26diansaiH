"""横向半截水管凹槽的颜色掩膜、逐列白段搜索与中轴拟合。

横向结构不再沿画面纵向逐行寻找左右边，而是：

1. 按LAB阈值生成严格黑白掩膜；
2. 在横向ROI内按固定列步长扫描；
3. 每一列提取白色纵向连续段，得到凹槽上边、下边和中心y；
4. 从画面中央附近向左右跟踪同一条连续白带；
5. 直线鲁棒拟合负责剔除离群点，再用少量折线节点贴合镜头造成的轻微弯曲；
6. 中间保持粗步长扫描，只在粗端点附近逐列精扫物理边界。

除 ``make_trough_mask`` 外不依赖MaixPy，方便电脑构造二值图回归测试。
"""

from alpha_beta_filter import AlphaBetaFilter1D


DEFAULT_SETTINGS = {
    "lab_thresholds": [(77, 100, -18, 18, -18, 22)],
    "roi_ratio": (0.00, 0.62, 1.00, 0.38),
    # 先裁ROI再做LAB二值化，避免每帧复制和处理整幅RGB图。
    # 二值调试画面需要完整尺寸掩膜时，由入口临时关闭此开关。
    "roi_mask_enabled": True,
    "column_step": 8,
    "min_run_height_px": 4,
    "max_run_height_ratio_roi": 0.45,
    "guided_scan_half_height_px": 0,
    "max_internal_gap_px": 14,
    "local_search_radius_px": 20,
    "max_track_gap_x_px": 48,
    "height_change_weight": 0.15,
    "gap_penalty_weight": 0.12,
    # 快速路径：只尝试少量中心种子，再沿预测中轴小窗口向左右生长；
    # 贯穿不足时自动回退原来的全采样列候选算法。
    "seeded_growth_enabled": True,
    "seed_column_trials": 11,
    "seed_track_trials": 4,
    "seed_growth_half_height_px": 16,
    "seed_early_accept_span_ratio": 0.70,
    "endpoint_band_px": 18,
    "endpoint_min_support_points": 3,
    "endpoint_min_aspect_ratio": 5.0,
    "endpoint_refine_enabled": True,
    "endpoint_refine_radius_px": 12,
    "endpoint_refine_max_misses": 0,
    # 中间水管仍按column_step粗采样；左右端点只先按2列粗定位，
    # 再在最后一个变化区间内逐列确认，兼顾速度与1像素端点分辨率。
    "endpoint_search_coarse_step_px": 2,
    # 半圆凹槽俯视时外壁轮廓可能略长；只接受中轴附近仍具有正常厚度的内缘。
    "endpoint_inner_edge_enabled": True,
    "endpoint_inner_search_radius_px": 16,
    "endpoint_inner_min_height_ratio": 0.55,
    "endpoint_inner_center_tolerance_ratio": 0.28,
    "endpoint_inner_confirm_columns": 2,
    "endpoint_occlusion_enabled": True,
    "endpoint_occlusion_edge_margin_px": 18,
    "endpoint_occlusion_min_inset_px": 8,
    "endpoint_occlusion_max_gap_ratio_roi": 0.18,
    "endpoint_occlusion_release_frames": 2,
    "min_points": 24,
    "min_length_ratio_frame": 0.45,
    "min_fit_residual_px": 3.0,
    "max_fit_residual_px": 8.0,
    "ball_exclusion_margin_px": 5,
    "cache_frames": 3,
    "update_interval_frames": 1,
    # 整体中轴斜率/截距仍做轻量EMA；左右动态端点改用独立alpha-beta，
    # 重测帧校正位置和速度，HELD中间帧只做有界预测。
    "axis_filter_enabled": True,
    "axis_filter_alpha": 0.25,
    "axis_filter_min_confidence": 0.30,
    "axis_filter_snap_shift_px": 14.0,
    "axis_filter_snap_slope_delta": 0.06,
    "axis_filter_endpoint_max_shift_px": 24.0,
    "axis_filter_endpoint_min_span_ratio": 0.75,
    "endpoint_ab_enabled": True,
    "endpoint_ab_fixed_right_enabled": False,
    "endpoint_ab_fixed_right_x": 0.0,
    "endpoint_ab_alpha": 0.70,
    "endpoint_ab_beta": 0.15,
    "endpoint_ab_max_innovation_px": 48.0,
    "endpoint_ab_max_velocity_px_s": 240.0,
    "endpoint_ab_max_predict_px": 12.0,
    "endpoint_ab_reset_gap_ms": 600,
    # PC直接调用且没有板端ticks_ms时，仅用于构造可重复的逻辑时间。
    "endpoint_ab_nominal_dt_ms": 100,
    "curved_centerline_enabled": True,
    "centerline_segments": 6,
    "centerline_residual_filter_enabled": True,
    "centerline_residual_filter_alpha": 0.20,
    "centerline_residual_max_step_px": 1.5,
    "positive_at_right": True,
}


def _settings(values=None):
    output = dict(DEFAULT_SETTINGS)
    output.update(values or {})
    return output


def make_trough_mask(frame, thresholds, copy_frame=True):
    """用Maix LAB阈值生成0/255掩膜。

    全帧路径先复制，保护YOLO/显示仍要使用的原图；ROI裁剪图本身已经是
    独立对象，可传 ``copy_frame=False`` 避免再复制一次。
    """
    mask = frame.copy() if copy_frame else frame
    converted = mask.binary(list(thresholds))
    if converted is not None:
        mask = converted
    return mask


def _localized_box(box, offset_x, offset_y):
    """把全帧YOLO框平移到裁剪ROI坐标；None保持不变。"""
    if not box:
        return None
    output = dict(box)
    output["x"] = float(output.get("x", 0.0)) - float(offset_x)
    output["y"] = float(output.get("y", 0.0)) - float(offset_y)
    return output


def _localized_axis(axis, offset_x, offset_y):
    """仅平移历史中轴的必要几何字段，供ROI内引导扫描使用。"""
    if not axis:
        return None
    output = dict(axis)
    for name in ("p0", "p1"):
        point = axis.get(name)
        if point is not None:
            output[name] = (
                float(point[0]) - float(offset_x),
                float(point[1]) - float(offset_y),
            )
    centerline = axis.get("centerline") or []
    if centerline:
        output["centerline"] = [
            (
                float(point[0]) - float(offset_x),
                float(point[1]) - float(offset_y),
            )
            for point in centerline
        ]
    roi = axis.get("roi")
    if roi is not None:
        output["roi"] = (
            int(roi[0]) - int(offset_x),
            int(roi[1]) - int(offset_y),
            int(roi[2]),
            int(roi[3]),
        )
    if axis.get("intercept_y") is not None:
        slope = float(axis.get("slope_y_per_x", 0.0))
        output["intercept_y"] = (
            float(axis["intercept_y"]) - float(offset_y)
            + slope * float(offset_x)
        )
    return output


def _globalized_axis(axis, offset_x, offset_y, global_roi):
    """把ROI局部提取结果恢复为全帧坐标。"""
    output = dict(axis)

    def shifted_point(point):
        copied = dict(point)
        copied["x"] = int(copied["x"]) + int(offset_x)
        copied["top"] = int(copied["top"]) + int(offset_y)
        copied["bottom"] = int(copied["bottom"]) + int(offset_y)
        copied["cy"] = float(copied["cy"]) + float(offset_y)
        return copied

    output["points"] = [
        shifted_point(point) for point in axis.get("points", [])]
    output["inliers"] = [
        shifted_point(point) for point in axis.get("inliers", [])]
    for name in ("p0", "p1"):
        point = axis.get(name)
        if point is not None:
            output[name] = (
                float(point[0]) + float(offset_x),
                float(point[1]) + float(offset_y),
            )
    centerline = axis.get("centerline") or []
    if centerline:
        output["centerline"] = [
            (
                float(point[0]) + float(offset_x),
                float(point[1]) + float(offset_y),
            )
            for point in centerline
        ]
    if axis.get("intercept_y") is not None:
        slope = float(axis.get("slope_y_per_x", 0.0))
        output["intercept_y"] = (
            float(axis["intercept_y"]) + float(offset_y)
            - slope * float(offset_x)
        )
    output["roi"] = tuple(int(value) for value in global_roi)
    return output


class BinaryBuffer:
    """直接读取Maix二值图内存，避免逐像素调用 ``get_pixel``。"""

    def __init__(self, binary):
        self.width = int(binary.width())
        self.height = int(binary.height())
        self.raw = binary.to_bytes(False)
        self.view = memoryview(self.raw)
        try:
            self.view = self.view.cast("B")
        except Exception:
            pass
        pixels = self.width * self.height
        self.channels = len(self.view) // max(1, pixels)
        if self.channels not in (1, 2, 3, 4):
            raise RuntimeError(
                "unsupported binary buffer channels: %d" % self.channels)


def _roi_pixels(roi_ratio, width, height):
    rx, ry, rw, rh = [float(value) for value in roi_ratio]
    x = max(0, min(width - 1, int(round(rx * width))))
    y = max(0, min(height - 1, int(round(ry * height))))
    right = max(x + 1, min(width, int(round((rx + rw) * width))))
    bottom = max(y + 1, min(height, int(round((ry + rh) * height))))
    return x, y, right - x, bottom - y


def column_white_runs(buffer_view, x, y_min, y_max):
    """返回一列指定ROI内的全部白色连续段，下边界为闭区间。"""
    x = int(x)
    top_limit = max(0, min(buffer_view.height, int(y_min)))
    bottom_limit = max(0, min(buffer_view.height, int(y_max)))
    if x < 0 or x >= buffer_view.width or top_limit >= bottom_limit:
        return []

    runs = []
    view = buffer_view.view
    channels = int(buffer_view.channels)
    stride = buffer_view.width * channels
    y = top_limit

    if channels == 1:
        index = y * buffer_view.width + x
        while y < bottom_limit:
            while y < bottom_limit and view[index] <= 128:
                y += 1
                index += buffer_view.width
            if y >= bottom_limit:
                break
            top = y
            y += 1
            index += buffer_view.width
            while y < bottom_limit and view[index] > 128:
                y += 1
                index += buffer_view.width
            bottom = y - 1
            runs.append((top, bottom, bottom - top + 1,
                         (top + bottom) * 0.5))
        return runs

    index = (y * buffer_view.width + x) * channels
    if channels == 2:
        while y < bottom_limit:
            while y < bottom_limit and \
                    view[index] == 0 and view[index + 1] == 0:
                y += 1
                index += stride
            if y >= bottom_limit:
                break
            top = y
            y += 1
            index += stride
            while y < bottom_limit and \
                    (view[index] != 0 or view[index + 1] != 0):
                y += 1
                index += stride
            bottom = y - 1
            runs.append((top, bottom, bottom - top + 1,
                         (top + bottom) * 0.5))
        return runs

    # binary()常保留RGB888/BGRA格式，黑白像素各通道相同，读取首通道即可。
    while y < bottom_limit:
        while y < bottom_limit and view[index] <= 128:
            y += 1
            index += stride
        if y >= bottom_limit:
            break
        top = y
        y += 1
        index += stride
        while y < bottom_limit and view[index] > 128:
            y += 1
            index += stride
        bottom = y - 1
        runs.append((top, bottom, bottom - top + 1,
                     (top + bottom) * 0.5))
    return runs


def merge_nearby_runs(runs, max_gap_px):
    """合并同一凹槽内因反光或钢球边缘形成的小黑色间隙。"""
    if not runs:
        return []
    output = []
    current_top = int(runs[0][0])
    current_bottom = int(runs[0][1])
    for run in runs[1:]:
        top, bottom = int(run[0]), int(run[1])
        gap = top - current_bottom - 1
        if gap <= int(max_gap_px):
            current_bottom = max(current_bottom, bottom)
        else:
            height = current_bottom - current_top + 1
            output.append((current_top, current_bottom, height,
                           (current_top + current_bottom) * 0.5))
            current_top = top
            current_bottom = bottom
    height = current_bottom - current_top + 1
    output.append((current_top, current_bottom, height,
                   (current_top + current_bottom) * 0.5))
    return output


def _excluded_x(x, exclusion_box, margin):
    if not exclusion_box:
        return False
    left = float(exclusion_box.get("x", 0.0)) - float(margin)
    right = float(exclusion_box.get("x", 0.0)) + \
        float(exclusion_box.get("w", 0.0)) + float(margin)
    return left <= float(x) <= right


def _axis_y_at(axis, x):
    if not axis or not axis.get("valid"):
        return None
    centerline = axis.get("centerline") or []
    if len(centerline) >= 2:
        return _polyline_y_at(centerline, x)
    p0 = axis.get("p0")
    p1 = axis.get("p1")
    if p0 is None or p1 is None:
        return None
    dx = float(p1[0]) - float(p0[0])
    if abs(dx) < 1e-6:
        return (float(p0[1]) + float(p1[1])) * 0.5
    ratio = (float(x) - float(p0[0])) / dx
    return float(p0[1]) + ratio * (float(p1[1]) - float(p0[1]))


def _median(values):
    ordered = sorted(float(value) for value in values)
    if not ordered:
        return 0.0
    middle = len(ordered) // 2
    if len(ordered) % 2:
        return ordered[middle]
    return (ordered[middle - 1] + ordered[middle]) * 0.5


def _polyline_y_at(points, x):
    """按横坐标在线性折线内插；两端之外沿首/末段短距离外推。"""
    ordered = sorted(
        ((float(point[0]), float(point[1])) for point in points),
        key=lambda point: point[0],
    )
    if len(ordered) < 2:
        return ordered[0][1] if ordered else None
    target_x = float(x)
    segment = (ordered[0], ordered[1])
    if target_x >= ordered[-1][0]:
        segment = (ordered[-2], ordered[-1])
    elif target_x > ordered[0][0]:
        for index in range(len(ordered) - 1):
            if target_x <= ordered[index + 1][0]:
                segment = (ordered[index], ordered[index + 1])
                break
    (x0, y0), (x1, y1) = segment
    if abs(x1 - x0) < 1e-6:
        return 0.5 * (y0 + y1)
    ratio = (target_x - x0) / (x1 - x0)
    return y0 + ratio * (y1 - y0)


def _polyline_length(points):
    total = 0.0
    for start, end in zip(points, points[1:]):
        dx = float(end[0]) - float(start[0])
        dy = float(end[1]) - float(start[1])
        total += (dx * dx + dy * dy) ** 0.5
    return total


def _line_fit(points):
    """最小二乘拟合y=a*x+b；凹槽安装要求长轴大致水平。"""
    count = len(points)
    if count < 2:
        return None
    mean_x = sum(float(point["x"]) for point in points) / count
    mean_y = sum(float(point["cy"]) for point in points) / count
    denominator = sum(
        (float(point["x"]) - mean_x) ** 2 for point in points)
    if denominator <= 1e-9:
        return None
    slope = sum(
        (float(point["x"]) - mean_x) *
        (float(point["cy"]) - mean_y)
        for point in points) / denominator
    intercept = mean_y - slope * mean_x
    return slope, intercept


def _robust_fit(points, minimum_residual, maximum_residual):
    initial = _line_fit(points)
    if initial is None:
        return None
    slope, intercept = initial
    residuals = [
        abs(float(point["cy"]) -
            (slope * float(point["x"]) + intercept))
        for point in points
    ]
    median_residual = _median(residuals)
    mad = _median([abs(value - median_residual) for value in residuals])
    limit = median_residual + 2.5 * max(0.5, mad)
    limit = max(float(minimum_residual), limit)
    limit = min(float(maximum_residual), limit)
    inliers = [
        point for point, residual in zip(points, residuals)
        if residual <= limit
    ]
    refined = _line_fit(inliers)
    if refined is None:
        return None
    slope, intercept = refined
    refined_residuals = [
        abs(float(point["cy"]) -
            (slope * float(point["x"]) + intercept))
        for point in inliers
    ]
    rms = (
        sum(value * value for value in refined_residuals) /
        max(1, len(refined_residuals))
    ) ** 0.5
    return slope, intercept, inliers, rms


def _build_centerline(points, left_x, right_x, slope, intercept, settings):
    """用少量残差中位节点构造折线，保留直线拟合的抗离群能力。"""
    left_x = float(left_x)
    right_x = float(right_x)
    span = right_x - left_x
    enabled = bool(settings.get("curved_centerline_enabled", True))
    segments = max(1, int(settings.get("centerline_segments", 6)))
    if not enabled or span <= 1.0:
        return [
            (left_x, slope * left_x + intercept),
            (right_x, slope * right_x + intercept),
        ]

    buckets = [[] for _ in range(segments + 1)]
    for point in points:
        point_x = float(point["x"])
        ratio = max(0.0, min(1.0, (point_x - left_x) / span))
        index = max(0, min(segments, int(round(ratio * segments))))
        residual = float(point["cy"]) - (slope * point_x + intercept)
        buckets[index].append(residual)

    offsets = [
        _median(bucket) if bucket else None
        for bucket in buckets
    ]
    for index, value in enumerate(offsets):
        if value is not None:
            continue
        left_index = index - 1
        while left_index >= 0 and offsets[left_index] is None:
            left_index -= 1
        right_index = index + 1
        while right_index <= segments and offsets[right_index] is None:
            right_index += 1
        if left_index >= 0 and right_index <= segments:
            ratio = (index - left_index) / float(right_index - left_index)
            offsets[index] = (
                offsets[left_index] +
                ratio * (offsets[right_index] - offsets[left_index])
            )
        elif left_index >= 0:
            offsets[index] = offsets[left_index]
        elif right_index <= segments:
            offsets[index] = offsets[right_index]
        else:
            offsets[index] = 0.0

    centerline = []
    for index in range(segments + 1):
        ratio = index / float(segments)
        point_x = left_x + ratio * span
        point_y = slope * point_x + intercept + float(offsets[index])
        centerline.append((point_x, point_y))
    return centerline


def _remap_centerline(points, raw_slope, raw_intercept,
                      raw_left_x, raw_right_x,
                      filtered_slope, filtered_intercept,
                      filtered_left_x, filtered_right_x,
                      positive_at_right, curve_residuals=None):
    """把当前弯曲残差附着到滤波后的直线和动态端点上。"""
    ordered = sorted(
        ((float(point[0]), float(point[1])) for point in (points or [])),
        key=lambda point: point[0],
    )
    if len(ordered) < 2:
        ordered = [
            (
                float(raw_left_x),
                raw_slope * float(raw_left_x) + raw_intercept,
            ),
            (
                float(raw_right_x),
                raw_slope * float(raw_right_x) + raw_intercept,
            ),
        ]
    raw_span = max(1e-6, float(raw_right_x) - float(raw_left_x))
    filtered_span = float(filtered_right_x) - float(filtered_left_x)
    remapped = []
    use_filtered_residuals = curve_residuals is not None and \
        len(curve_residuals) == len(ordered)
    for index, (point_x, point_y) in enumerate(ordered):
        ratio = max(
            0.0,
            min(1.0, (point_x - float(raw_left_x)) / raw_span),
        )
        target_x = float(filtered_left_x) + ratio * filtered_span
        curve_residual = (
            float(curve_residuals[index])
            if use_filtered_residuals else
            point_y - (raw_slope * point_x + raw_intercept)
        )
        target_y = (
            filtered_slope * target_x + filtered_intercept +
            curve_residual
        )
        remapped.append((target_x, target_y))
    if bool(positive_at_right):
        return remapped
    return list(reversed(remapped))


def _best_endpoint_run(buffer_view, x, roi, settings,
                       expected_y, expected_height):
    """在端点单列中寻找与已确认凹槽中心和厚度最接近的白段。"""
    roi_x, roi_y, roi_width, roi_height = roi
    if x < roi_x or x >= roi_x + roi_width:
        return None
    local_radius = max(2.0, float(settings["local_search_radius_px"]))
    scan_top = max(roi_y, int(expected_y - local_radius))
    scan_bottom = min(
        roi_y + roi_height, int(expected_y + local_radius) + 1)
    runs = merge_nearby_runs(
        column_white_runs(buffer_view, x, scan_top, scan_bottom),
        max(0, int(settings["max_internal_gap_px"])),
    )
    min_height = max(1, int(settings["min_run_height_px"]))
    max_height = max(
        min_height,
        int(round(roi_height * float(
            settings["max_run_height_ratio_roi"]))),
    )
    candidates = [
        run for run in runs
        if min_height <= int(run[2]) <= max_height and
        abs(float(run[3]) - float(expected_y)) <= local_radius
    ]
    if not candidates:
        return None
    return min(
        candidates,
        key=lambda run: (
            abs(float(run[3]) - float(expected_y)) +
            0.15 * abs(float(run[2]) - float(expected_height))
        ),
    )


def _refine_endpoint_x(buffer_view, start_x, direction, roi, settings,
                       exclusion_box, centerline, expected_height):
    """从粗端点向外做步长2粗扫，再在末端变化区间逐列确认。

    末端精扫会从倒数第二个有效粗采样点重新检查，因此若两个粗采样点
    之间存在单列断裂，后方偶发毛刺不会被误当成连续端点。
    """
    if not bool(settings.get("endpoint_refine_enabled", True)):
        return float(start_x), False
    radius = max(0, int(settings.get("endpoint_refine_radius_px", 12)))
    max_misses = max(
        0, int(settings.get("endpoint_refine_max_misses", 0)))
    coarse_step = max(
        1, int(settings.get("endpoint_search_coarse_step_px", 2)))
    exclusion_margin = max(
        0, int(settings["ball_exclusion_margin_px"]))
    start_x = int(round(start_x))
    best_x = start_x
    misses = 0
    coarse_valid_x = [start_x]
    coarse_runs = {}
    fine_limit = radius

    # 第一级只探测稀疏列，快速找到端点由白变黑的大致区间。
    for offset in range(coarse_step, radius + 1, coarse_step):
        x = start_x + int(direction) * offset
        if _excluded_x(x, exclusion_box, exclusion_margin):
            fine_limit = max(0, offset - 1)
            break
        expected_y = _polyline_y_at(centerline, x)
        run = _best_endpoint_run(
            buffer_view, x, roi, settings, expected_y, expected_height)
        coarse_runs[x] = run
        if run is None:
            misses += 1
            if misses > max_misses:
                fine_limit = offset
                break
            continue
        coarse_valid_x.append(x)
        misses = 0

    # 第二级从倒数第二个有效粗采样点开始逐列确认。已粗扫的列直接复用
    # 结果，不重复读取缓冲；这既找回奇数列端点，也能识别粗点间的断裂。
    fine_anchor_x = (
        coarse_valid_x[-2]
        if len(coarse_valid_x) >= 2 else start_x
    )
    fine_anchor_offset = abs(int(fine_anchor_x) - start_x)
    # 锚点本身已经在粗扫中确认有效；精扫若在下一列遇到断裂，应保留
    # 该锚点，而不是错误退回最初的主体粗端点。
    best_x = int(fine_anchor_x)
    misses = 0
    for offset in range(fine_anchor_offset + 1, fine_limit + 1):
        x = start_x + int(direction) * offset
        if _excluded_x(x, exclusion_box, exclusion_margin):
            break
        if x in coarse_runs:
            run = coarse_runs[x]
        else:
            expected_y = _polyline_y_at(centerline, x)
            run = _best_endpoint_run(
                buffer_view, x, roi, settings,
                expected_y, expected_height)
        if run is None:
            misses += 1
            if misses > max_misses:
                break
            continue
        best_x = x
        misses = 0
    return float(best_x), best_x != start_x


def _seed_column_candidates(buffer_view, x, roi, settings,
                            exclusion_box, expected_y=None,
                            half_height=None):
    """只扫描一列的局部窗口，返回满足厚度门限的白段候选。"""
    roi_x, roi_y, roi_width, roi_height = roi
    if x < roi_x or x >= roi_x + roi_width:
        return []
    exclusion_margin = max(
        0, int(settings["ball_exclusion_margin_px"]))
    if _excluded_x(x, exclusion_box, exclusion_margin):
        return []

    scan_top = roi_y
    scan_bottom = roi_y + roi_height
    if expected_y is not None and half_height is not None:
        radius = max(2, int(half_height))
        scan_top = max(roi_y, int(round(expected_y)) - radius)
        scan_bottom = min(
            roi_y + roi_height,
            int(round(expected_y)) + radius + 1,
        )
    runs = merge_nearby_runs(
        column_white_runs(buffer_view, x, scan_top, scan_bottom),
        max(0, int(settings["max_internal_gap_px"])),
    )
    min_height = max(1, int(settings["min_run_height_px"]))
    max_height = max(
        min_height,
        int(round(roi_height * float(
            settings["max_run_height_ratio_roi"]))),
    )
    return [
        run for run in runs
        if min_height <= int(run[2]) <= max_height
    ]


def _seeded_white_track(buffer_view, roi, settings, exclusion_box,
                        previous_axis, allow_guided_scan=True):
    """从少量种子直接局部生长；成功时避免预扫全部采样列。"""
    if not bool(settings.get("seeded_growth_enabled", True)):
        return []

    roi_x, roi_y, roi_width, _ = roi
    column_step = max(1, int(settings["column_step"]))
    local_radius = max(1.0, float(settings["local_search_radius_px"]))
    max_gap_x = max(column_step, int(settings["max_track_gap_x_px"]))
    height_change_weight = max(
        0.0, float(settings.get("height_change_weight", 0.15)))
    gap_penalty_weight = max(
        0.0, float(settings.get("gap_penalty_weight", 0.12)))
    growth_half_height = max(
        2, int(settings.get("seed_growth_half_height_px", 16)))
    guided_half_height = max(
        0, int(settings.get("guided_scan_half_height_px", 0)))
    seed_column_trials = max(
        1, int(settings.get("seed_column_trials", 11)))
    seed_track_trials = max(
        1, int(settings.get("seed_track_trials", 4)))
    minimum_points = max(2, int(settings["min_points"]))
    early_span_ratio = max(
        float(settings["min_length_ratio_frame"]),
        min(1.0, float(settings.get(
            "seed_early_accept_span_ratio", 0.70))),
    )
    roi_center_y = roi_y + roi[3] * 0.5

    def point_from_run(x, run):
        return {
            "x": int(x),
            "top": int(run[0]),
            "bottom": int(run[1]),
            "height": int(run[2]),
            "cy": float(run[3]),
        }

    def follow(seed_x, seed_run, direction):
        output = [point_from_run(seed_x, seed_run)]
        current_x = int(seed_x)
        current_y = float(seed_run[3])
        current_height = float(seed_run[2])
        slope = 0.0
        cost = 0.0
        x = int(seed_x) + int(direction) * column_step
        end_x = roi_x + roi_width
        while roi_x <= x < end_x:
            signed_gap = int(x) - current_x
            gap_x = abs(signed_gap)
            if gap_x > max_gap_x:
                break
            predicted_y = current_y + slope * signed_gap
            candidates = _seed_column_candidates(
                buffer_view, x, roi, settings, exclusion_box,
                expected_y=predicted_y,
                half_height=growth_half_height,
            )
            best = None
            best_cost = None
            for run in candidates:
                center_delta = abs(float(run[3]) - predicted_y)
                if center_delta > local_radius + 0.20 * gap_x:
                    continue
                candidate_cost = center_delta + \
                    height_change_weight * abs(
                        float(run[2]) - current_height) + \
                    gap_penalty_weight * max(0.0, gap_x - column_step)
                if best_cost is None or candidate_cost < best_cost:
                    best = run
                    best_cost = candidate_cost
            if best is not None:
                instantaneous_slope = (
                    float(best[3]) - current_y) / signed_gap
                slope = 0.70 * slope + 0.30 * instantaneous_slope
                current_x = int(x)
                current_y = float(best[3])
                current_height = float(best[2])
                output.append(point_from_run(x, best))
                cost += float(best_cost)
            x += int(direction) * column_step
        return output, cost

    columns = list(range(roi_x, roi_x + roi_width, column_step))
    center_x = roi_x + roi_width * 0.5
    columns.sort(key=lambda x: abs(float(x) - center_x))
    best_track = []
    best_key = None
    tracks_tried = 0

    seed_columns_checked = 0
    for seed_x in columns:
        # 中心种子可能恰好被钢球排除区覆盖；这类列不消耗试探额度，
        # 继续向外扩展寻找可生长种子。
        if _excluded_x(
                seed_x, exclusion_box,
                max(0, int(settings["ball_exclusion_margin_px"]))):
            continue
        seed_columns_checked += 1
        if seed_columns_checked > seed_column_trials:
            break
        expected_y = _axis_y_at(previous_axis, seed_x)
        if expected_y is None:
            expected_y = roi_center_y
        use_guided_seed = bool(
            allow_guided_scan and previous_axis and
            guided_half_height > 0)
        candidates = _seed_column_candidates(
            buffer_view, seed_x, roi, settings, exclusion_box,
            expected_y=expected_y if use_guided_seed else None,
            half_height=guided_half_height if use_guided_seed else None,
        )
        ordered = sorted(
            candidates,
            key=lambda run: (
                abs(float(run[3]) - float(expected_y)),
                -int(run[2]),
            ),
        )
        for seed_run in ordered[:2]:
            leftward, left_cost = follow(seed_x, seed_run, -1)
            rightward, right_cost = follow(seed_x, seed_run, 1)
            leftward.reverse()
            track = leftward[:-1] + rightward
            span_x = track[-1]["x"] - track[0]["x"] if track else 0
            expected_points = max(
                1.0, span_x / float(column_step) + 1.0)
            coverage = min(1.0, len(track) / expected_points)
            span_ratio = min(
                1.0, span_x / max(1.0, float(roi_width)))
            average_cost = (
                left_cost + right_cost) / max(1, len(track))
            key = (
                coverage * (0.5 + 0.5 * span_ratio),
                span_x, -average_cost, len(track),
            )
            if best_key is None or key > best_key:
                best_key = key
                best_track = track
            tracks_tried += 1
            if len(track) >= minimum_points and \
                    span_ratio >= early_span_ratio and coverage >= 0.72:
                return track
            if tracks_tried >= seed_track_trials:
                return best_track
    return best_track


def _exhaustive_white_track(buffer_view, roi, settings, exclusion_box,
                            previous_axis, allow_guided_scan=True):
    roi_x, roi_y, roi_width, roi_height = roi
    column_step = max(1, int(settings["column_step"]))
    min_height = max(1, int(settings["min_run_height_px"]))
    max_height = max(
        min_height,
        int(round(roi_height * float(
            settings["max_run_height_ratio_roi"]))),
    )
    max_gap = max(0, int(settings["max_internal_gap_px"]))
    local_radius = max(1.0, float(settings["local_search_radius_px"]))
    max_gap_x = max(column_step, int(settings["max_track_gap_x_px"]))
    height_change_weight = max(
        0.0, float(settings.get("height_change_weight", 0.15)))
    gap_penalty_weight = max(
        0.0, float(settings.get("gap_penalty_weight", 0.12)))
    exclusion_margin = max(
        0, int(settings["ball_exclusion_margin_px"]))
    guided_half_height = max(
        0, int(settings.get("guided_scan_half_height_px", 0)))
    guided = bool(
        allow_guided_scan and previous_axis and guided_half_height > 0)
    roi_center_y = roi_y + roi_height * 0.5

    column_candidates = []
    end_x = roi_x + roi_width
    for x in range(roi_x, end_x, column_step):
        if _excluded_x(x, exclusion_box, exclusion_margin):
            continue
        scan_top = roi_y
        scan_bottom = roi_y + roi_height
        if guided:
            expected_y = _axis_y_at(previous_axis, x)
            if expected_y is not None:
                scan_top = max(
                    roi_y, int(expected_y) - guided_half_height)
                scan_bottom = min(
                    roi_y + roi_height,
                    int(expected_y) + guided_half_height + 1,
                )
        raw_runs = column_white_runs(
            buffer_view, x, scan_top, scan_bottom)
        runs = merge_nearby_runs(raw_runs, max_gap)
        candidates = [run for run in runs
                      if min_height <= int(run[2]) <= max_height]
        if candidates:
            column_candidates.append((int(x), candidates))

    if not column_candidates:
        return []

    def point_from_run(x, run):
        return {
            "x": int(x),
            "top": int(run[0]),
            "bottom": int(run[1]),
            "height": int(run[2]),
            "cy": float(run[3]),
        }

    def follow(seed_index, seed_run, direction):
        seed_x = column_candidates[seed_index][0]
        output = [point_from_run(seed_x, seed_run)]
        current_y = float(seed_run[3])
        current_height = float(seed_run[2])
        current_x = int(seed_x)
        slope = 0.0
        cost = 0.0
        index = seed_index + direction
        while 0 <= index < len(column_candidates):
            x, candidates = column_candidates[index]
            signed_gap = int(x) - current_x
            gap_x = abs(signed_gap)
            if gap_x > max_gap_x:
                break
            predicted_y = current_y + slope * signed_gap
            best = None
            best_cost = None
            for run in candidates:
                center_delta = abs(float(run[3]) - predicted_y)
                if center_delta > local_radius + 0.20 * gap_x:
                    continue
                candidate_cost = center_delta + \
                    height_change_weight * abs(
                        float(run[2]) - current_height) + \
                    gap_penalty_weight * max(0.0, gap_x - column_step)
                if best_cost is None or candidate_cost < best_cost:
                    best = run
                    best_cost = candidate_cost
            if best is not None:
                instantaneous_slope = (
                    float(best[3]) - current_y) / signed_gap
                slope = 0.70 * slope + 0.30 * instantaneous_slope
                current_y = float(best[3])
                current_height = float(best[2])
                current_x = int(x)
                output.append(point_from_run(x, best))
                cost += float(best_cost)
            index += direction
        return output, cost

    center_x = roi_x + roi_width * 0.5
    seed_columns = sorted(
        range(len(column_candidates)),
        key=lambda index: abs(column_candidates[index][0] - center_x),
    )
    seeds = []
    for seed_index in seed_columns:
        x, candidates = column_candidates[seed_index]
        expected_y = _axis_y_at(previous_axis, x)
        if expected_y is None:
            expected_y = roi_center_y
        ordered = sorted(
            candidates,
            key=lambda run: (abs(float(run[3]) - expected_y),
                             -int(run[2])),
        )
        for run in ordered[:2]:
            seeds.append((seed_index, run))
            if len(seeds) >= 6:
                break
        if len(seeds) >= 6:
            break

    best_track = []
    best_key = None
    for seed_index, seed_run in seeds:
        leftward, leftward_cost = follow(seed_index, seed_run, -1)
        rightward, rightward_cost = follow(seed_index, seed_run, 1)
        leftward.reverse()
        track = leftward[:-1] + rightward
        span_x = track[-1]["x"] - track[0]["x"] if track else 0
        average_cost = (
            leftward_cost + rightward_cost) / max(1, len(track))
        expected_points = max(
            1.0, span_x / float(column_step) + 1.0)
        coverage = min(1.0, len(track) / expected_points)
        # 连续覆盖率优先于单纯点数，避免稀疏反光列拼成“最长”假轨迹。
        span_ratio = min(1.0, span_x / max(1.0, float(roi_width)))
        track_score = coverage * (0.5 + 0.5 * span_ratio)
        key = (track_score, span_x, -average_cost, len(track))
        if best_key is None or key > best_key:
            best_key = key
            best_track = track
    return best_track


def _longest_white_track(buffer_view, roi, settings, exclusion_box,
                         previous_axis, allow_guided_scan=True):
    """优先少种子局部生长；候选不足时回退原全列预扫。"""
    minimum_points = max(2, int(settings["min_points"]))
    if bool(settings.get("seeded_growth_enabled", True)):
        seeded = _seeded_white_track(
            buffer_view, roi, settings, exclusion_box,
            previous_axis, allow_guided_scan=allow_guided_scan)
        if len(seeded) >= minimum_points:
            return seeded, "SEEDED"
        exhaustive = _exhaustive_white_track(
            buffer_view, roi, settings, exclusion_box,
            previous_axis, allow_guided_scan=allow_guided_scan)
        return exhaustive, "EXHAUSTIVE_FALLBACK"
    return _exhaustive_white_track(
        buffer_view, roi, settings, exclusion_box,
        previous_axis, allow_guided_scan=allow_guided_scan), "EXHAUSTIVE"


def _inner_endpoint_supported(buffer_view, x, roi, settings,
                              centerline, expected_height):
    """判断端点列是否仍包含中部、接近正常厚度的凹槽截面。"""
    expected_y = _polyline_y_at(centerline, x)
    candidates = _seed_column_candidates(
        buffer_view, x, roi, settings, exclusion_box=None,
        expected_y=expected_y,
        half_height=max(2, int(settings["local_search_radius_px"])),
    )
    minimum_height = max(
        float(settings["min_run_height_px"]),
        float(expected_height) * float(settings.get(
            "endpoint_inner_min_height_ratio", 0.55)),
    )
    center_tolerance = max(
        1.5,
        float(expected_height) * float(settings.get(
            "endpoint_inner_center_tolerance_ratio", 0.28)),
    )
    return any(
        float(run[2]) >= minimum_height and
        abs(float(run[3]) - float(expected_y)) <= center_tolerance
        for run in candidates
    )


def _trim_endpoint_to_inner_edge(buffer_view, start_x, direction, roi,
                                 settings, exclusion_box,
                                 centerline, expected_height):
    """从外轮廓向内粗定位完整截面，再逐列确认较短的凹槽内缘。"""
    if not bool(settings.get("endpoint_inner_edge_enabled", True)):
        return float(start_x), False
    radius = max(
        0, int(settings.get("endpoint_inner_search_radius_px", 16)))
    confirm = max(
        1, int(settings.get("endpoint_inner_confirm_columns", 2)))
    coarse_step = max(
        1, int(settings.get("endpoint_search_coarse_step_px", 2)))
    exclusion_margin = max(
        0, int(settings["ball_exclusion_margin_px"]))
    start_x = int(round(start_x))
    support_cache = {}

    def supported_at(offset):
        x = start_x + int(direction) * int(offset)
        if _excluded_x(x, exclusion_box, exclusion_margin):
            return False
        if x not in support_cache:
            support_cache[x] = _inner_endpoint_supported(
                buffer_view, x, roi, settings,
                centerline, expected_height)
        return support_cache[x]

    # 先每隔2列寻找可能的完整截面。每次命中后，只在命中点前后的小区间
    # 做步长1连续确认；若只是孤立毛刺，确认失败后继续寻找下一个粗命中。
    for coarse_offset in range(0, radius + 1, coarse_step):
        if not supported_at(coarse_offset):
            continue
        fine_start = max(0, coarse_offset - coarse_step)
        fine_end = min(
            radius, coarse_offset + coarse_step + confirm - 1)
        streak_start = None
        streak = 0
        for offset in range(fine_start, fine_end + 1):
            if supported_at(offset):
                if streak == 0:
                    streak_start = offset
                streak += 1
                if streak >= confirm:
                    inner_x = start_x + int(direction) * streak_start
                    return float(inner_x), inner_x != start_x
            else:
                streak_start = None
                streak = 0
    return float(start_x), False


def _finish_stage_timing(result, field, clock_ms, started_ms):
    """结束一个内部阶段计时；未传板端时钟时保持零开销兼容。"""
    if clock_ms is None or started_ms is None:
        return None
    ended_ms = clock_ms()
    result[field] = max(0, int(ended_ms - started_ms))
    return ended_ms


def extract_trough_axis(binary, settings=None, exclusion_box=None,
                        previous_axis=None, clock_ms=None):
    """从二值图提取横向凹槽中轴，失败时返回带reason的诊断字典。"""
    settings = _settings(settings)
    result = {
        "valid": False,
        "reason": "UNKNOWN",
        "source": "MEASURED",
        "orientation": "HORIZONTAL",
        "cache_age": 0,
        "roi": None,
        "points": [],
        "inliers": [],
        "p0": None,
        "p1": None,
        "centerline": [],
        "centerline_mode": "NONE",
        "centerline_curve_px": 0.0,
        "length_px": 0.0,
        "median_width_px": 0.0,
        "median_thickness_px": 0.0,
        "fit_rms_px": None,
        "confidence": 0.0,
        "scan_mode": "FULL",
        "track_mode": "NONE",
        "endpoint_refine": "NONE",
        "endpoint_refine_left_px": 0.0,
        "endpoint_refine_right_px": 0.0,
        "seed_ms": 0,
        "fit_ms": 0,
        "refine_ms": 0,
        "inner_ms": 0,
    }
    try:
        buffer_view = BinaryBuffer(binary)
    except Exception as error:
        result["reason"] = "BUFFER_FAILED"
        result["error"] = str(error)
        return result

    roi = _roi_pixels(
        settings["roi_ratio"], buffer_view.width, buffer_view.height)
    result["roi"] = roi
    guided = bool(
        previous_axis and
        int(settings.get("guided_scan_half_height_px", 0)) > 0
    )
    stage_started_ms = clock_ms() if clock_ms else None
    points, track_mode = _longest_white_track(
        buffer_view, roi, settings, exclusion_box, previous_axis,
        allow_guided_scan=True)
    result["track_mode"] = track_mode
    minimum_points = max(2, int(settings["min_points"]))
    if guided and len(points) < minimum_points:
        points, track_mode = _longest_white_track(
            buffer_view, roi, settings, exclusion_box, previous_axis,
            allow_guided_scan=False)
        result["scan_mode"] = "FULL_FALLBACK"
        result["track_mode"] = track_mode
    elif guided:
        result["scan_mode"] = "GUIDED"
    stage_started_ms = _finish_stage_timing(
        result, "seed_ms", clock_ms, stage_started_ms)
    result["points"] = points
    if not points:
        result["reason"] = "NO_WHITE_TRACK"
        return result
    if len(points) < minimum_points:
        result["reason"] = "TOO_FEW_POINTS"
        return result

    fitted = _robust_fit(
        points,
        settings["min_fit_residual_px"],
        settings["max_fit_residual_px"],
    )
    if fitted is None:
        result["reason"] = "FIT_FAILED"
        _finish_stage_timing(
            result, "fit_ms", clock_ms, stage_started_ms)
        return result
    slope, intercept, inliers, rms = fitted
    result["inliers"] = inliers
    result["fit_rms_px"] = rms
    if len(inliers) < minimum_points:
        result["reason"] = "TOO_FEW_INLIERS"
        _finish_stage_timing(
            result, "fit_ms", clock_ms, stage_started_ms)
        return result

    left_x = min(int(point["x"]) for point in inliers)
    right_x = max(int(point["x"]) for point in inliers)
    left = (float(left_x), slope * left_x + intercept)
    right = (float(right_x), slope * right_x + intercept)
    dx = right[0] - left[0]
    dy = right[1] - left[1]
    length_px = (dx * dx + dy * dy) ** 0.5
    result["length_px"] = length_px
    thickness = _median([point["height"] for point in inliers])
    result["median_width_px"] = thickness
    result["median_thickness_px"] = thickness

    # 最长轨迹不等于细长物体：端点附近若只有少数孤立列，或者候选段
    # 过厚，都会把背景反光/车体边缘接到轨迹两端。这里做动态形状门控，
    # 不固定端点坐标，只拒绝明显不符合“横向长、纵向窄”的轨迹。
    span_x = max(0.0, float(right_x - left_x))
    aspect_ratio = span_x / max(1.0, float(thickness))
    result["aspect_ratio"] = aspect_ratio
    endpoint_band = max(
        int(settings.get("endpoint_band_px", 18)),
        2 * max(1, int(settings["column_step"])),
    )
    endpoint_min_points = max(
        1, int(settings.get("endpoint_min_support_points", 3)))
    left_edge_points = sum(
        1 for point in inliers
        if int(point["x"]) <= left_x + endpoint_band)
    right_edge_points = sum(
        1 for point in inliers
        if int(point["x"]) >= right_x - endpoint_band)
    left_endpoint_support = min(
        1.0, left_edge_points / float(endpoint_min_points))
    right_endpoint_support = min(
        1.0, right_edge_points / float(endpoint_min_points))
    endpoint_support = min(
        left_endpoint_support, right_endpoint_support)
    left_endpoint_strong = left_edge_points >= endpoint_min_points
    right_endpoint_strong = right_edge_points >= endpoint_min_points
    if left_endpoint_strong and right_endpoint_strong:
        endpoint_quality = "BOTH_STRONG"
    elif left_endpoint_strong:
        endpoint_quality = "RIGHT_WEAK"
    elif right_endpoint_strong:
        endpoint_quality = "LEFT_WEAK"
    else:
        endpoint_quality = "BOTH_WEAK"
    result.update({
        "endpoint_support": endpoint_support,
        "endpoint_left_support": left_endpoint_support,
        "endpoint_right_support": right_endpoint_support,
        "endpoint_quality": endpoint_quality,
    })
    if aspect_ratio < float(settings.get("endpoint_min_aspect_ratio", 5.0)):
        result["reason"] = "NOT_SLENDER"
        _finish_stage_timing(
            result, "fit_ms", clock_ms, stage_started_ms)
        return result

    minimum_length = float(settings["min_length_ratio_frame"]) * \
        buffer_view.width
    if length_px < minimum_length:
        result["reason"] = "TROUGH_TOO_SHORT"
        _finish_stage_timing(
            result, "fit_ms", clock_ms, stage_started_ms)
        return result

    # 单侧端点正是钢球遮挡时最容易缺少一个拟合内点的区域。若主体已经
    # 通过长度、细长度和RMS门限，不能在这里提前拒绝，否则后续的
    # HOLD_OCCLUDED/BOOTSTRAP逻辑永远没有机会修复该侧端点。只有左右
    # 两侧都缺乏局部支撑时，才把它视为无法可靠确定跨度的假轨迹。
    if not left_endpoint_strong and not right_endpoint_strong:
        result["reason"] = "ENDPOINT_SUPPORT_LOW"
        _finish_stage_timing(
            result, "fit_ms", clock_ms, stage_started_ms)
        return result

    provisional_centerline = _build_centerline(
        inliers, left_x, right_x, slope, intercept, settings)
    stage_started_ms = _finish_stage_timing(
        result, "fit_ms", clock_ms, stage_started_ms)
    refined_left_x, left_refined = _refine_endpoint_x(
        buffer_view, left_x, -1, roi, settings, exclusion_box,
        provisional_centerline, thickness)
    refined_right_x, right_refined = _refine_endpoint_x(
        buffer_view, right_x, 1, roi, settings, exclusion_box,
        provisional_centerline, thickness)
    left_refine_px = max(0.0, float(left_x) - refined_left_x)
    right_refine_px = max(0.0, refined_right_x - float(right_x))
    if left_refined and right_refined:
        endpoint_refine = "BOTH"
    elif left_refined:
        endpoint_refine = "LEFT"
    elif right_refined:
        endpoint_refine = "RIGHT"
    else:
        endpoint_refine = "NONE"
    stage_started_ms = _finish_stage_timing(
        result, "refine_ms", clock_ms, stage_started_ms)

    # 俯视半圆槽时，薄外壁可能比槽底真实有效长度多伸出数像素。
    # 从最外轮廓向内找连续完整截面，取中部较短的内缘作为最终端点。
    inner_left_x, left_inner_trimmed = _trim_endpoint_to_inner_edge(
        buffer_view, refined_left_x, 1, roi, settings, exclusion_box,
        provisional_centerline, thickness)
    inner_right_x, right_inner_trimmed = _trim_endpoint_to_inner_edge(
        buffer_view, refined_right_x, -1, roi, settings, exclusion_box,
        provisional_centerline, thickness)
    if inner_left_x >= inner_right_x:
        inner_left_x = refined_left_x
        inner_right_x = refined_right_x
        left_inner_trimmed = False
        right_inner_trimmed = False
    if left_inner_trimmed and right_inner_trimmed:
        endpoint_inner = "BOTH"
    elif left_inner_trimmed:
        endpoint_inner = "LEFT"
    elif right_inner_trimmed:
        endpoint_inner = "RIGHT"
    else:
        endpoint_inner = "NONE"
    inner_left_px = max(0.0, inner_left_x - refined_left_x)
    inner_right_px = max(0.0, refined_right_x - inner_right_x)

    left_to_right = _build_centerline(
        inliers, inner_left_x, inner_right_x,
        slope, intercept, settings)
    if bool(settings["positive_at_right"]):
        centerline = left_to_right
    else:
        centerline = list(reversed(left_to_right))
    p0 = centerline[0]
    p1 = centerline[-1]
    length_px = _polyline_length(centerline)
    centerline_curve_px = max(
        abs(
            float(point[1]) -
            (slope * float(point[0]) + intercept)
        )
        for point in centerline
    )
    expected_points = max(
        1, int(round((right_x - left_x) /
                     max(1, int(settings["column_step"])))) + 1)
    support = min(1.0, len(inliers) / float(expected_points))
    residual_score = max(
        0.0,
        1.0 - rms / max(1.0, float(settings["max_fit_residual_px"])),
    )
    result.update({
        "valid": True,
        "reason": "OK",
        "p0": p0,
        "p1": p1,
        "centerline": centerline,
        "centerline_mode": "POLYLINE"
            if bool(settings.get("curved_centerline_enabled", True))
            else "LINE",
        "centerline_curve_px": centerline_curve_px,
        "length_px": length_px,
        "endpoint_refine": endpoint_refine,
        "endpoint_refine_left_px": left_refine_px,
        "endpoint_refine_right_px": right_refine_px,
        "endpoint_inner": endpoint_inner,
        "endpoint_inner_left_px": inner_left_px,
        "endpoint_inner_right_px": inner_right_px,
        "confidence": support * residual_score,
        "slope_y_per_x": slope,
        "intercept_y": intercept,
    })
    _finish_stage_timing(
        result, "inner_ms", clock_ms, stage_started_ms)
    return result


class HorizontalTroughTracker:
    """管理横向凹槽掩膜、低频更新和短时中轴缓存。"""

    def __init__(self, settings=None, clock_ms=None, clock_diff_ms=None):
        self.settings = _settings(settings)
        self.clock_ms = clock_ms
        self.clock_diff_ms = clock_diff_ms
        self._fallback_timestamp_ms = 0
        self.last_axis = None
        self.cache_age = 0
        self.frame_index = 0
        self._filtered_slope = None
        self._filtered_intercept = None
        self._filtered_left_x = None
        self._filtered_right_x = None
        self._filtered_width = None
        self._filtered_curve_residuals = None
        self._endpoint_occlusion_side = "NONE"
        self._endpoint_clear_frames = 0
        self._left_endpoint_ab = self._new_endpoint_filter()
        self._right_endpoint_ab = self._new_endpoint_filter()

    def reset(self):
        self.last_axis = None
        self.cache_age = 0
        self.frame_index = 0
        self._filtered_slope = None
        self._filtered_intercept = None
        self._filtered_left_x = None
        self._filtered_right_x = None
        self._filtered_width = None
        self._filtered_curve_residuals = None
        self._endpoint_occlusion_side = "NONE"
        self._endpoint_clear_frames = 0
        self._fallback_timestamp_ms = 0
        self._left_endpoint_ab.reset()
        self._right_endpoint_ab.reset()

    def _new_endpoint_filter(self):
        return AlphaBetaFilter1D(
            alpha=self.settings.get("endpoint_ab_alpha", 0.70),
            beta=self.settings.get("endpoint_ab_beta", 0.15),
            max_velocity=self.settings.get(
                "endpoint_ab_max_velocity_px_s", 240.0),
            max_predict=self.settings.get("endpoint_ab_max_predict_px", 12.0),
            max_innovation=self.settings.get(
                "endpoint_ab_max_innovation_px", 48.0),
            reset_gap_ms=self.settings.get("endpoint_ab_reset_gap_ms", 600),
            clock_diff_ms=self.clock_diff_ms,
        )

    def _timestamp_ms(self):
        """读取板端时钟；PC纯逻辑调用没有时钟时使用固定步长。"""
        if self.clock_ms is not None:
            return int(self.clock_ms())
        current = int(self._fallback_timestamp_ms)
        self._fallback_timestamp_ms = (
            current + max(1, int(self.settings.get(
                "endpoint_ab_nominal_dt_ms", 100)))) & 0xFFFFFFFF
        return current

    def _sync_endpoint_positions(self, left_x, right_x, timestamp_ms,
                                 bounds=None, zero_velocity=False):
        """约束端点在ROI内且不交叉，并同步alpha-beta内部位置。"""
        left_x = float(left_x)
        right_x = float(right_x)
        if bounds is not None:
            left_x = max(float(bounds[0]), min(float(bounds[1]), left_x))
            right_x = max(float(bounds[0]), min(float(bounds[1]), right_x))
        if right_x <= left_x:
            midpoint = 0.5 * (left_x + right_x)
            left_x = midpoint - 0.5
            right_x = midpoint + 0.5
            if bounds is not None:
                left_x = max(float(bounds[0]), left_x)
                right_x = min(float(bounds[1]), right_x)
        self._left_endpoint_ab.set_position(
            left_x, timestamp_ms, zero_velocity=zero_velocity)
        self._right_endpoint_ab.set_position(
            right_x, timestamp_ms, zero_velocity=zero_velocity)
        self._filtered_left_x = left_x
        self._filtered_right_x = right_x
        return left_x, right_x

    @staticmethod
    def _raw_line(axis):
        """读取拟合直线参数，兼容旧结果中没有显式斜率的情况。"""
        p0 = axis.get("p0")
        p1 = axis.get("p1")
        if p0 is None or p1 is None:
            return None
        x0, y0 = float(p0[0]), float(p0[1])
        x1, y1 = float(p1[0]), float(p1[1])
        dx = x1 - x0
        if abs(dx) < 1e-6:
            return None
        slope = float(axis.get("slope_y_per_x", (y1 - y0) / dx))
        intercept = float(axis.get("intercept_y", y0 - slope * x0))
        return slope, intercept, min(x0, x1), max(x0, x1)

    def _filter_centerline_residuals(
            self, points, raw_slope, raw_intercept,
            raw_left_x, raw_right_x):
        """只平滑局部弯曲残差，整体平移和倾斜仍由原中轴EMA处理。"""
        ordered = sorted(
            ((float(point[0]), float(point[1]))
             for point in (points or [])),
            key=lambda point: point[0],
        )
        if len(ordered) < 2:
            ordered = [
                (
                    float(raw_left_x),
                    raw_slope * float(raw_left_x) + raw_intercept,
                ),
                (
                    float(raw_right_x),
                    raw_slope * float(raw_right_x) + raw_intercept,
                ),
            ]
        measured = [
            point_y - (raw_slope * point_x + raw_intercept)
            for point_x, point_y in ordered
        ]
        if not bool(self.settings.get(
                "centerline_residual_filter_enabled", True)):
            self._filtered_curve_residuals = None
            return measured, "DISABLED", 0.0

        previous = self._filtered_curve_residuals
        if previous is None or len(previous) != len(measured):
            self._filtered_curve_residuals = list(measured)
            return list(measured), "INIT", 0.0

        alpha = max(0.0, min(1.0, float(self.settings.get(
            "centerline_residual_filter_alpha", 0.20))))
        max_step = max(0.0, float(self.settings.get(
            "centerline_residual_max_step_px", 1.5)))
        filtered = []
        for old_value, measured_value in zip(previous, measured):
            new_value = old_value + alpha * (measured_value - old_value)
            if max_step > 0.0:
                new_value = max(
                    old_value - max_step,
                    min(old_value + max_step, new_value),
                )
            filtered.append(new_value)
        residual_shift = max(
            abs(filtered_value - measured_value)
            for filtered_value, measured_value in zip(filtered, measured)
        )
        self._filtered_curve_residuals = filtered
        return list(filtered), "EMA", residual_shift

    @staticmethod
    def _roi_x_bounds(axis):
        roi = axis.get("roi") if axis else None
        if roi is None or len(roi) < 4:
            return None
        left = float(roi[0])
        width = max(1.0, float(roi[2]))
        return left, left + width - 1.0

    def _edge_ball_side(self, raw_axis, exclusion_box):
        """用贴近ROI边缘的YOLO框判断哪一侧端点被钢球遮挡。"""
        if not bool(self.settings.get(
                "endpoint_occlusion_enabled", True)) or not exclusion_box:
            return "NONE"
        bounds = self._roi_x_bounds(raw_axis)
        if bounds is None:
            return "NONE"
        roi_left, roi_right = bounds
        box_left = float(exclusion_box.get("x", 0.0))
        box_width = max(0.0, float(exclusion_box.get("w", 0.0)))
        if box_width <= 1.0:
            return "NONE"
        box_right = box_left + box_width
        margin = max(
            float(self.settings.get(
                "endpoint_occlusion_edge_margin_px", 18)),
            0.25 * box_width,
        )
        near_left = box_left <= roi_left + margin
        near_right = box_right >= roi_right - margin
        if near_left and near_right:
            box_center = 0.5 * (box_left + box_right)
            roi_center = 0.5 * (roi_left + roi_right)
            return "LEFT" if box_center <= roi_center else "RIGHT"
        if near_left:
            return "LEFT"
        if near_right:
            return "RIGHT"
        return "NONE"

    def _mask_occlusion_side(self, raw_axis, raw_left_x, raw_right_x,
                             history_ready):
        """用单侧颜色缺口判断端点遮挡，兼容启动时还没有历史端点。"""
        if not bool(self.settings.get(
                "endpoint_occlusion_enabled", True)):
            return "NONE"
        bounds = self._roi_x_bounds(raw_axis)
        if bounds is None:
            return "NONE"
        roi_left, roi_right = bounds
        roi_width = max(1.0, roi_right - roi_left + 1.0)
        minimum_inset = max(1.0, float(self.settings.get(
            "endpoint_occlusion_min_inset_px", 8)))
        maximum_gap = max(
            minimum_inset + 1.0,
            roi_width * max(0.0, float(self.settings.get(
                "endpoint_occlusion_max_gap_ratio_roi", 0.18))),
        )
        other_side_limit = max(2.0, 0.5 * minimum_inset)
        if history_ready:
            left_inset = raw_left_x - self._filtered_left_x
            right_inset = self._filtered_right_x - raw_right_x
            opposite_side_limit = other_side_limit
        else:
            left_inset = raw_left_x - roi_left
            right_inset = roi_right - raw_right_x
            # 无历史时只有另一端确实贴边，才允许从ROI边界补全被遮挡端。
            opposite_side_limit = max(
                other_side_limit,
                float(self.settings.get(
                    "endpoint_occlusion_edge_margin_px", 18)),
            )

        left_candidate = minimum_inset <= left_inset <= maximum_gap and \
            right_inset <= opposite_side_limit
        right_candidate = minimum_inset <= right_inset <= maximum_gap and \
            left_inset <= opposite_side_limit
        if left_candidate and right_candidate:
            # 两侧都同时缩进时无法仅凭颜色掩膜判断球挡住哪一端；
            # 保留为普通动态端点或交给整段突缩保护，不能硬选一侧。
            return "NONE"
        if left_candidate:
            return "LEFT"
        if right_candidate:
            return "RIGHT"
        return "NONE"

    def _mask_gap_matches_ball(self, raw_axis, mask_side, exclusion_box,
                               raw_left_x, raw_right_x):
        """确认单侧颜色缺口确实紧邻YOLO框，而不是远处阈值波动。"""
        if not exclusion_box or mask_side not in ("LEFT", "RIGHT"):
            return False
        bounds = self._roi_x_bounds(raw_axis)
        if bounds is None:
            return False
        roi_left, roi_right = bounds
        box_left = float(exclusion_box.get("x", 0.0))
        box_width = max(0.0, float(exclusion_box.get("w", 0.0)))
        if box_width <= 1.0:
            return False
        box_right = box_left + box_width
        box_center = 0.5 * (box_left + box_right)
        roi_center = 0.5 * (roi_left + roi_right)
        proximity = max(
            float(self.settings.get(
                "endpoint_occlusion_edge_margin_px", 18)),
            0.5 * box_width,
        )
        if mask_side == "LEFT":
            return box_center <= roi_center and \
                abs(float(raw_left_x) - box_right) <= proximity
        return box_center >= roi_center and \
            abs(float(raw_right_x) - box_left) <= proximity

    def _resolve_endpoint_occlusion(self, raw_axis, raw_left_x, raw_right_x,
                                    exclusion_box, history_ready):
        box_side = self._edge_ball_side(raw_axis, exclusion_box)
        mask_side = self._mask_occlusion_side(
            raw_axis, raw_left_x, raw_right_x, history_ready)
        # YOLO有框但不贴画面边缘时，只有缺口边界与钢球框相邻才允许补全。
        # 这样保留右半段钢球遮挡补全，同时拒绝中部钢球与远端阈值波动
        # 偶然同时出现时的MASK误触发。
        mask_matches_box = self._mask_gap_matches_ball(
            raw_axis, mask_side, exclusion_box,
            raw_left_x, raw_right_x)
        if exclusion_box and not mask_matches_box:
            mask_side = "NONE"
        direct_side = box_side if box_side != "NONE" else mask_side
        if direct_side != "NONE":
            self._endpoint_occlusion_side = direct_side
            self._endpoint_clear_frames = 0
            if box_side != "NONE":
                source = "YOLO"
            elif exclusion_box:
                source = "YOLO_MASK"
            else:
                source = "MASK"
            prefix = "BOOTSTRAP_" if not history_ready else ""
            return direct_side, prefix + source + "_" + direct_side

        if self._endpoint_occlusion_side != "NONE":
            self._endpoint_clear_frames += 1
            release_frames = max(0, int(self.settings.get(
                "endpoint_occlusion_release_frames", 2)))
            if self._endpoint_clear_frames <= release_frames:
                side = self._endpoint_occlusion_side
                return side, "RELEASE_HOLD_" + side
            self._endpoint_occlusion_side = "NONE"
            self._endpoint_clear_frames = 0
        return "NONE", "CLEAR"

    def _filter_valid_axis(self, raw_axis, exclusion_box=None,
                           timestamp_ms=None):
        """对当前有效中轴做质量门控、整体EMA和端点alpha-beta。

        斜率、截距和宽度保留原轻量EMA；左右横向端点分别维护位置与
        速度。几何门控和遮挡状态机始终先执行，不能用预测替代候选筛选。
        """
        if timestamp_ms is None:
            timestamp_ms = self._timestamp_ms()
        raw_line = self._raw_line(raw_axis)
        if raw_line is None:
            return None, False

        raw_slope, raw_intercept, raw_left_x, raw_right_x = raw_line
        measured_left_x = raw_left_x
        measured_right_x = raw_right_x
        fixed_right_enabled = bool(self.settings.get(
            "endpoint_ab_fixed_right_enabled", False))
        if fixed_right_enabled:
            raw_right_x = float(self.settings.get(
                "endpoint_ab_fixed_right_x", raw_right_x))
            bounds = self._roi_x_bounds(raw_axis)
            if bounds is not None:
                raw_right_x = max(
                    float(bounds[0]), min(float(bounds[1]), raw_right_x))
            raw_right_x = max(raw_left_x + 1.0, raw_right_x)
        raw_width = max(0.0, float(raw_axis.get("median_width_px", 0.0)))
        enabled = bool(self.settings.get("axis_filter_enabled", True))
        endpoint_filter = "INIT"
        history_ready = self._filtered_slope is not None and \
            self._filtered_left_x is not None and \
            self._filtered_right_x is not None and \
            self._left_endpoint_ab.initialized and \
            self._right_endpoint_ab.initialized
        if not enabled or not history_ready:
            mode = "DISABLED" if not enabled else "INIT"
            alpha = 1.0
            shift_px = 0.0
            slope_delta = 0.0
        else:
            confidence = float(raw_axis.get("confidence", 0.0))
            minimum_confidence = float(
                self.settings.get("axis_filter_min_confidence", 0.30))
            if confidence < minimum_confidence:
                return None, False

            reference_x = (raw_left_x + raw_right_x) * 0.5
            raw_y = raw_slope * reference_x + raw_intercept
            filtered_y = (
                self._filtered_slope * reference_x +
                self._filtered_intercept
            )
            shift_px = abs(raw_y - filtered_y)
            slope_delta = abs(raw_slope - self._filtered_slope)
            snap_shift = float(self.settings.get(
                "axis_filter_snap_shift_px", 14.0))
            snap_slope = float(self.settings.get(
                "axis_filter_snap_slope_delta", 0.06))
            if shift_px >= snap_shift or slope_delta >= snap_slope:
                alpha = 1.0
                mode = "SNAP"
            else:
                alpha = max(0.0, min(1.0, float(self.settings.get(
                    "axis_filter_alpha", 0.25))))
                mode = "EMA"


        occlusion_side, endpoint_mode = self._resolve_endpoint_occlusion(
            raw_axis, measured_left_x, measured_right_x,
            exclusion_box, history_ready)
        bounds = self._roi_x_bounds(raw_axis)
        if not history_ready and occlusion_side != "NONE" and \
                bounds is not None:
            if occlusion_side == "LEFT":
                raw_left_x = bounds[0]
            else:
                raw_right_x = bounds[1]
            endpoint_filter = "BOOTSTRAP_OCCLUDED_" + occlusion_side
        elif history_ready:
            # 端点仍然动态变化，但限制单帧向外伸长。钢球只遮挡一侧时，
            # 仅保持该侧历史端点，另一侧继续更新；颜色恢复后自动释放。
            endpoint_limit = max(1.0, float(self.settings.get(
                "axis_filter_endpoint_max_shift_px", 24.0)))

            def limited(previous, measured):
                return max(
                    previous - endpoint_limit,
                    min(previous + endpoint_limit, measured),
                )

            if occlusion_side == "LEFT":
                raw_left_x = self._filtered_left_x
                raw_right_x = limited(
                    self._filtered_right_x, raw_right_x)
                endpoint_filter = "HOLD_OCCLUDED_LEFT"
            elif occlusion_side == "RIGHT":
                raw_left_x = limited(
                    self._filtered_left_x, raw_left_x)
                raw_right_x = self._filtered_right_x
                endpoint_filter = "HOLD_OCCLUDED_RIGHT"
            else:
                previous_span = max(
                    1.0,
                    float(self._filtered_right_x - self._filtered_left_x),
                )
                raw_span = max(1.0, float(raw_right_x - raw_left_x))
                min_span_ratio = max(0.0, min(1.0, float(
                    self.settings.get(
                        "axis_filter_endpoint_min_span_ratio", 0.75))))
                if raw_span < previous_span * min_span_ratio:
                    raw_left_x = self._filtered_left_x
                    raw_right_x = self._filtered_right_x
                    endpoint_filter = "HOLD_SHRINK"
                else:
                    raw_left_x = limited(
                        self._filtered_left_x, raw_left_x)
                    raw_right_x = limited(
                        self._filtered_right_x, raw_right_x)
                    endpoint_filter = "TRACKED"

        self._filtered_slope = (
            alpha * raw_slope + (1.0 - alpha) *
            (self._filtered_slope if self._filtered_slope is not None
             else raw_slope)
        )
        self._filtered_intercept = (
            alpha * raw_intercept + (1.0 - alpha) *
            (self._filtered_intercept
             if self._filtered_intercept is not None else raw_intercept)
        )

        endpoint_ab_enabled = enabled and bool(self.settings.get(
            "endpoint_ab_enabled", True))
        if not endpoint_ab_enabled:
            left_ab = self._left_endpoint_ab.reset(
                raw_left_x, timestamp_ms)
            right_ab = self._right_endpoint_ab.reset(
                raw_right_x, timestamp_ms)
            endpoint_ab_mode = "DISABLED"
        elif not history_ready:
            left_ab = self._left_endpoint_ab.reset(
                raw_left_x, timestamp_ms)
            right_ab = self._right_endpoint_ab.reset(
                raw_right_x, timestamp_ms)
            endpoint_ab_mode = "INIT"
        elif endpoint_filter == "HOLD_SHRINK":
            left_ab = self._left_endpoint_ab.hold(
                timestamp_ms, zero_velocity=True, mode="HOLD_SHRINK")
            right_ab = self._right_endpoint_ab.hold(
                timestamp_ms, zero_velocity=True, mode="HOLD_SHRINK")
            endpoint_ab_mode = "HOLD_SHRINK"
        elif occlusion_side == "LEFT":
            left_ab = self._left_endpoint_ab.hold(
                timestamp_ms, zero_velocity=True, mode="HOLD_OCCLUDED")
            right_ab = self._right_endpoint_ab.correct(
                raw_right_x, timestamp_ms)
            endpoint_ab_mode = "HOLD_LEFT_CORRECT_RIGHT"
        elif occlusion_side == "RIGHT":
            left_ab = self._left_endpoint_ab.correct(
                raw_left_x, timestamp_ms)
            right_ab = self._right_endpoint_ab.hold(
                timestamp_ms, zero_velocity=True, mode="HOLD_OCCLUDED")
            endpoint_ab_mode = "CORRECT_LEFT_HOLD_RIGHT"
        elif mode == "SNAP":
            left_ab = self._left_endpoint_ab.reset(
                raw_left_x, timestamp_ms)
            right_ab = self._right_endpoint_ab.reset(
                raw_right_x, timestamp_ms)
            endpoint_ab_mode = "SNAP"
        else:
            left_ab = self._left_endpoint_ab.correct(
                raw_left_x, timestamp_ms)
            right_ab = self._right_endpoint_ab.correct(
                raw_right_x, timestamp_ms)
            left_mode = left_ab.get("mode", "CORRECT")
            right_mode = right_ab.get("mode", "CORRECT")
            endpoint_ab_mode = (
                left_mode if left_mode == right_mode else
                left_mode + "_" + right_mode
            )

        if fixed_right_enabled:
            right_ab = self._right_endpoint_ab.reset(
                raw_right_x, timestamp_ms)
            endpoint_ab_mode = "%s_RIGHT_FIXED" % left_ab.get(
                "mode", "LEFT")

        self._sync_endpoint_positions(
            left_ab["position"], right_ab["position"], timestamp_ms,
            bounds=bounds,
            zero_velocity=not endpoint_ab_enabled,
        )
        if raw_width > 0.0:
            self._filtered_width = (
                alpha * raw_width + (1.0 - alpha) *
                (self._filtered_width
                 if self._filtered_width is not None else raw_width)
            )

        curve_residuals, residual_filter, residual_shift = \
            self._filter_centerline_residuals(
                raw_axis.get("centerline"),
                raw_slope,
                raw_intercept,
                raw_line[2],
                raw_line[3],
            )
        centerline = _remap_centerline(
            raw_axis.get("centerline"),
            raw_slope,
            raw_intercept,
            raw_line[2],
            raw_line[3],
            self._filtered_slope,
            self._filtered_intercept,
            self._filtered_left_x,
            self._filtered_right_x,
            self.settings.get("positive_at_right", True),
            curve_residuals=curve_residuals,
        )
        p0 = centerline[0]
        p1 = centerline[-1]
        filtered = dict(raw_axis)
        filtered.update({
            "p0": p0,
            "p1": p1,
            "centerline": centerline,
            "centerline_curve_px": max(
                abs(
                    float(point[1]) - (
                        self._filtered_slope * float(point[0]) +
                        self._filtered_intercept
                    )
                )
                for point in centerline
            ),
            "centerline_residual_filter": residual_filter,
            "centerline_residual_shift_px": residual_shift,
            "length_px": _polyline_length(centerline),
            "slope_y_per_x": self._filtered_slope,
            "intercept_y": self._filtered_intercept,
            "median_width_px": self._filtered_width
                if self._filtered_width is not None else raw_width,
            "median_thickness_px": self._filtered_width
                if self._filtered_width is not None else raw_width,
            "axis_filter": mode,
            "axis_filter_shift_px": shift_px,
            "endpoint_filter": endpoint_filter,
            "endpoint_mode": endpoint_mode,
            "endpoint_ab": endpoint_ab_mode,
            "endpoint_raw_left_x": measured_left_x,
            "endpoint_raw_right_x": measured_right_x,
            "endpoint_v_left": self._left_endpoint_ab.velocity,
            "endpoint_v_right": self._right_endpoint_ab.velocity,
            "endpoint_predict_px_left": left_ab.get("prediction_px", 0.0),
            "endpoint_predict_px_right": right_ab.get("prediction_px", 0.0),
        })
        return filtered, True

    def _predict_held_axis(self, axis, timestamp_ms):
        """在不重做颜色扫描的HELD帧，用端点速度生成一次有界预测。"""
        output = dict(axis)
        enabled = bool(self.settings.get("axis_filter_enabled", True)) and \
            bool(self.settings.get("endpoint_ab_enabled", True))
        if not enabled or not self._left_endpoint_ab.initialized or \
                not self._right_endpoint_ab.initialized:
            output.update({
                "endpoint_ab": "HOLD_DISABLED",
                "endpoint_raw_left_x": None,
                "endpoint_raw_right_x": None,
                "endpoint_predict_px_left": 0.0,
                "endpoint_predict_px_right": 0.0,
            })
            return output

        old_line = self._raw_line(output)
        if old_line is None:
            return output
        old_slope, old_intercept, old_left_x, old_right_x = old_line
        occlusion_side = self._endpoint_occlusion_side
        fixed_right_enabled = bool(self.settings.get(
            "endpoint_ab_fixed_right_enabled", False))
        if occlusion_side == "LEFT":
            left_ab = self._left_endpoint_ab.hold(
                timestamp_ms, zero_velocity=True, mode="HOLD_OCCLUDED")
            right_ab = self._right_endpoint_ab.predict(timestamp_ms)
            endpoint_ab_mode = "HOLD_LEFT_PREDICT_RIGHT"
        elif occlusion_side == "RIGHT":
            left_ab = self._left_endpoint_ab.predict(timestamp_ms)
            right_ab = self._right_endpoint_ab.hold(
                timestamp_ms, zero_velocity=True, mode="HOLD_OCCLUDED")
            endpoint_ab_mode = "PREDICT_LEFT_HOLD_RIGHT"
        else:
            left_ab = self._left_endpoint_ab.predict(timestamp_ms)
            right_ab = self._right_endpoint_ab.predict(timestamp_ms)
            endpoint_ab_mode = "PREDICT"

        if fixed_right_enabled:
            fixed_right_x = float(self.settings.get(
                "endpoint_ab_fixed_right_x", old_right_x))
            bounds = self._roi_x_bounds(output)
            if bounds is not None:
                fixed_right_x = max(
                    float(bounds[0]), min(float(bounds[1]), fixed_right_x))
            right_ab = self._right_endpoint_ab.reset(
                fixed_right_x, timestamp_ms)
            endpoint_ab_mode = "%s_RIGHT_FIXED" % left_ab.get(
                "mode", "LEFT")

        new_left_x, new_right_x = self._sync_endpoint_positions(
            left_ab["position"], right_ab["position"], timestamp_ms,
            bounds=self._roi_x_bounds(output),
        )
        centerline = _remap_centerline(
            output.get("centerline"),
            old_slope,
            old_intercept,
            old_left_x,
            old_right_x,
            old_slope,
            old_intercept,
            new_left_x,
            new_right_x,
            self.settings.get("positive_at_right", True),
        )
        output.update({
            "p0": centerline[0],
            "p1": centerline[-1],
            "centerline": centerline,
            "length_px": _polyline_length(centerline),
            "endpoint_filter": "AB_PREDICT",
            "endpoint_ab": endpoint_ab_mode,
            "endpoint_raw_left_x": None,
            "endpoint_raw_right_x": None,
            "endpoint_v_left": self._left_endpoint_ab.velocity,
            "endpoint_v_right": self._right_endpoint_ab.velocity,
            "endpoint_predict_px_left": left_ab.get("prediction_px", 0.0),
            "endpoint_predict_px_right": right_ab.get("prediction_px", 0.0),
        })
        return output

    def _reuse(self, source, fallback_reason=None,
               mask_ms=0, scan_ms=0, mask_scope=None,
               crop_error=None, stage_times=None, timestamp_ms=None):
        if self.last_axis is None:
            return None
        output = dict(self.last_axis)
        if timestamp_ms is None:
            timestamp_ms = self._timestamp_ms()
        if source == "HELD":
            output = self._predict_held_axis(output, timestamp_ms)
            self.last_axis = dict(output)
        elif source == "CACHED":
            # 提取失败时不能沿旧速度继续开放预测，否则错误趋势会越走越远。
            self._left_endpoint_ab.hold(
                timestamp_ms, zero_velocity=True, mode="HOLD_CACHED")
            self._right_endpoint_ab.hold(
                timestamp_ms, zero_velocity=True, mode="HOLD_CACHED")
            output.update({
                "endpoint_ab": "HOLD_CACHED",
                "endpoint_raw_left_x": None,
                "endpoint_raw_right_x": None,
                "endpoint_v_left": self._left_endpoint_ab.velocity,
                "endpoint_v_right": self._right_endpoint_ab.velocity,
                "endpoint_predict_px_left": 0.0,
                "endpoint_predict_px_right": 0.0,
            })
        output["source"] = source
        output["cache_age"] = self.cache_age
        output["mask_ms"] = int(mask_ms)
        output["scan_ms"] = int(scan_ms)
        stage_times = stage_times or {}
        for field in ("seed_ms", "fit_ms", "refine_ms", "inner_ms"):
            output[field] = int(stage_times.get(field, 0))
        if mask_scope is not None:
            output["mask_scope"] = str(mask_scope)
        if crop_error:
            output["crop_error"] = str(crop_error)
        if source == "HELD":
            output["scan_mode"] = "HELD"
        elif source == "CACHED":
            output["scan_mode"] = "CACHED_AFTER_FAIL"
        if fallback_reason:
            output["fallback_reason"] = fallback_reason
        return output

    def update(self, frame, exclusion_box=None):
        self.frame_index += 1
        frame_timestamp_ms = self._timestamp_ms()
        interval = max(
            1, int(self.settings["update_interval_frames"]))
        if self.last_axis is not None and \
                (self.frame_index - 1) % interval != 0:
            return self._reuse(
                "HELD", mask_ms=0, scan_ms=0,
                mask_scope="HELD",
                timestamp_ms=frame_timestamp_ms), None

        mask = None
        mask_ms = 0
        scan_ms = 0
        mask_scope = "FULL"
        crop_error = None
        try:
            started_ms = self.clock_ms() if self.clock_ms else None
            axis_settings = self.settings
            axis_exclusion_box = exclusion_box
            axis_previous = self.last_axis
            global_roi = None
            if bool(self.settings.get("roi_mask_enabled", False)):
                try:
                    frame_width = int(frame.width())
                    frame_height = int(frame.height())
                    global_roi = _roi_pixels(
                        self.settings["roi_ratio"],
                        frame_width,
                        frame_height,
                    )
                    roi_x, roi_y, roi_width, roi_height = global_roi
                    cropped = frame.crop(
                        roi_x, roi_y, roi_width, roi_height)
                    mask = make_trough_mask(
                        cropped,
                        self.settings["lab_thresholds"],
                        copy_frame=False,
                    )
                    axis_settings = dict(self.settings)
                    axis_settings["roi_ratio"] = (0.0, 0.0, 1.0, 1.0)
                    # 长度门限原来按全帧宽度定义；即使以后ROI不再全宽，
                    # 裁剪路径也必须保持完全相同的物理门限。
                    axis_settings["min_length_ratio_frame"] = (
                        float(self.settings["min_length_ratio_frame"])
                        * frame_width / max(1.0, float(roi_width))
                    )
                    axis_exclusion_box = _localized_box(
                        exclusion_box, roi_x, roi_y)
                    axis_previous = _localized_axis(
                        self.last_axis, roi_x, roi_y)
                    mask_scope = "ROI"
                except Exception as error:
                    # crop属于板端API；固件不兼容或裁剪失败时立即回退原
                    # 全帧路径，不让性能优化破坏既有主线。
                    crop_error = str(error)
                    global_roi = None
                    mask = None
                    axis_settings = self.settings
                    axis_exclusion_box = exclusion_box
                    axis_previous = self.last_axis
                    mask_scope = "FULL_FALLBACK"
            if mask is None:
                mask = make_trough_mask(
                    frame, self.settings["lab_thresholds"])
            if started_ms is not None:
                mask_ms = self.clock_ms() - started_ms
                started_ms = self.clock_ms()
            axis = extract_trough_axis(
                mask,
                axis_settings,
                exclusion_box=axis_exclusion_box,
                previous_axis=axis_previous,
                clock_ms=self.clock_ms,
            )
            if mask_scope == "ROI" and global_roi is not None:
                axis = _globalized_axis(
                    axis, global_roi[0], global_roi[1], global_roi)
            if started_ms is not None:
                scan_ms = self.clock_ms() - started_ms
        except Exception as error:
            axis = {
                "valid": False,
                "reason": "MASK_FAILED",
                "source": "MEASURED",
                "orientation": "HORIZONTAL",
                "error": str(error),
            }
        axis["mask_ms"] = int(mask_ms)
        axis["scan_ms"] = int(scan_ms)
        for field in ("seed_ms", "fit_ms", "refine_ms", "inner_ms"):
            axis[field] = int(axis.get(field, 0))
        axis["mask_scope"] = mask_scope
        if crop_error:
            axis["crop_error"] = crop_error

        if axis.get("valid"):
            filtered_axis, accepted = self._filter_valid_axis(
                axis, exclusion_box=exclusion_box,
                timestamp_ms=frame_timestamp_ms)
            if accepted:
                self.last_axis = dict(filtered_axis)
                self.cache_age = 0
                return filtered_axis, mask
            # 低置信度测量不覆盖上一条可靠中轴，短时间沿用旧结果。
            axis["valid"] = False
            axis["reason"] = "AXIS_FILTER_REJECTED"

        self.cache_age += 1
        if self.last_axis is not None and self.cache_age <= max(
                0, int(self.settings["cache_frames"])):
            return self._reuse(
                "CACHED", axis.get("reason", "UNKNOWN"),
                mask_ms=mask_ms, scan_ms=scan_ms,
                mask_scope=mask_scope,
                crop_error=crop_error,
                stage_times=axis,
                timestamp_ms=frame_timestamp_ms), mask
        return axis, mask
