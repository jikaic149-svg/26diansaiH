"""钢球候选筛选与横向凹槽轴向位置计算。

横向实验不使用旧透视标定，输出：

- ``t``：从有向左端P0到右端P1的0~1比例；
- ``position_norm``：以中点为0的-1~+1归一化位置；
- ``position_px``：以中点为0、沿凹槽中轴方向的有符号像素距离。
- ``position_cm``：按已知凹槽长度线性换算、以中点为0的有符号厘米位置。

厘米值只是 ``position_norm * 物理长度 / 2``，不会执行图像透视变换或旧机位打表。
"""


def _value(item, name, default=0.0):
    value = getattr(item, name, default)
    return value() if callable(value) else value


def snapshot_detections(objects):
    """立即复制YOLO原生对象，避免跨阶段持有NPU返回对象。"""
    output = []
    for obj in objects or ():
        width = float(_value(obj, "w", 0.0))
        height = float(_value(obj, "h", 0.0))
        if width <= 1.0 or height <= 1.0:
            continue
        output.append({
            "x": float(_value(obj, "x", 0.0)),
            "y": float(_value(obj, "y", 0.0)),
            "w": width,
            "h": height,
            "score": float(_value(obj, "score", 0.0)),
        })
    return output


def detection_center(detection):
    return (
        float(detection["x"]) + 0.5 * float(detection["w"]),
        float(detection["y"]) + 0.5 * float(detection["h"]),
    )


def highest_score_detection(detections):
    valid = [item for item in detections or ()
             if float(item.get("w", 0.0)) > 1.0
             and float(item.get("h", 0.0)) > 1.0]
    if not valid:
        return None
    return max(valid, key=lambda item: float(item.get("score", 0.0)))


def _axis_polyline(axis):
    if not axis or not axis.get("valid"):
        return None
    centerline = axis.get("centerline") or []
    if len(centerline) >= 2:
        points = [
            (float(point[0]), float(point[1]))
            for point in centerline
        ]
        if any(
            (points[index + 1][0] - points[index][0]) ** 2 +
            (points[index + 1][1] - points[index][1]) ** 2 > 1.0
            for index in range(len(points) - 1)
        ):
            return points
    p0 = axis.get("p0")
    p1 = axis.get("p1")
    if p0 is None or p1 is None:
        return None
    x0, y0 = float(p0[0]), float(p0[1])
    x1, y1 = float(p1[0]), float(p1[1])
    if (x1 - x0) ** 2 + (y1 - y0) ** 2 <= 1.0:
        return None
    return [(x0, y0), (x1, y1)]


def point_axis_measurement(x, y, axis):
    """将点投影到有向折线，返回弧长比例、投影点和垂距。"""
    points = _axis_polyline(axis)
    if points is None:
        return None
    target_x = float(x)
    target_y = float(y)
    segments = []
    total_length = 0.0
    for start, end in zip(points, points[1:]):
        vx = float(end[0]) - float(start[0])
        vy = float(end[1]) - float(start[1])
        length_sq = vx * vx + vy * vy
        if length_sq <= 1e-9:
            continue
        length = length_sq ** 0.5
        segments.append((start, vx, vy, length_sq, length, total_length))
        total_length += length
    if not segments or total_length <= 1.0:
        return None

    best = None
    for index, segment in enumerate(segments):
        start, vx, vy, length_sq, length, cumulative = segment
        dx = target_x - float(start[0])
        dy = target_y - float(start[1])
        segment_t = (dx * vx + dy * vy) / length_sq
        clipped_t = max(0.0, min(1.0, segment_t))
        projected_x = float(start[0]) + clipped_t * vx
        projected_y = float(start[1]) + clipped_t * vy
        off_x = target_x - projected_x
        off_y = target_y - projected_y
        distance_sq = off_x * off_x + off_y * off_y
        candidate = (
            distance_sq, index, segment_t, clipped_t,
            projected_x, projected_y, cumulative, length,
        )
        if best is None or candidate[0] < best[0]:
            best = candidate

    (distance_sq, index, segment_t, clipped_t,
     projected_x, projected_y, cumulative, segment_length) = best
    if index == 0 and segment_t < 0.0:
        along = segment_t * segment_length
        start, vx, vy = segments[index][0:3]
        projected_x = float(start[0]) + segment_t * vx
        projected_y = float(start[1]) + segment_t * vy
        off_x = target_x - projected_x
        off_y = target_y - projected_y
        distance_sq = off_x * off_x + off_y * off_y
    elif index == len(segments) - 1 and segment_t > 1.0:
        along = cumulative + segment_t * segment_length
        start, vx, vy = segments[index][0:3]
        projected_x = float(start[0]) + segment_t * vx
        projected_y = float(start[1]) + segment_t * vy
        off_x = target_x - projected_x
        off_y = target_y - projected_y
        distance_sq = off_x * off_x + off_y * off_y
    else:
        along = cumulative + clipped_t * segment_length
    raw_t = along / total_length
    return {
        "raw_t": raw_t,
        "projected_x": projected_x,
        "projected_y": projected_y,
        "axis_distance_px": distance_sq ** 0.5,
        "axis_length_px": total_length,
    }


def filter_detections_on_trough(
        detections, axis, min_distance_limit_px=24.0,
        width_factor=1.35, endpoint_margin_ratio=0.08,
        enforce_endpoint_bounds=True):
    """只保留横向凹槽走廊附近的YOLO框，并按几何优先级排序。"""
    if _axis_polyline(axis) is None:
        return list(detections or ())

    trough_width = max(
        1.0, float(axis.get("median_width_px", 1.0)))
    distance_limit = max(
        float(min_distance_limit_px),
        trough_width * float(width_factor),
    )
    margin = max(0.0, float(endpoint_margin_ratio))
    accepted = []
    for detection in detections or ():
        cx, cy = detection_center(detection)
        measurement = point_axis_measurement(cx, cy, axis)
        if measurement is None:
            continue
        raw_t = measurement["raw_t"]
        distance = measurement["axis_distance_px"]
        if bool(enforce_endpoint_bounds) and (
                raw_t < -margin or raw_t > 1.0 + margin) or \
                distance > distance_limit:
            continue
        cost = distance / distance_limit - \
            0.15 * float(detection.get("score", 0.0))
        accepted.append((cost, detection))
    accepted.sort(key=lambda item: item[0])
    return [item[1] for item in accepted]


def filter_detections_in_normalized_roi(
        detections, frame_width, frame_height, roi_ratio):
    """兼容旧测试/调用方：只保留中心落在归一化ROI内的检测框。"""
    frame_width = float(frame_width)
    frame_height = float(frame_height)
    if frame_width <= 0.0 or frame_height <= 0.0 or \
            roi_ratio is None or len(roi_ratio) != 4:
        return []
    roi_x, roi_y, roi_width, roi_height = (
        float(value) for value in roi_ratio)
    if roi_width <= 0.0 or roi_height <= 0.0:
        return []
    left = roi_x * frame_width
    top = roi_y * frame_height
    right = (roi_x + roi_width) * frame_width
    bottom = (roi_y + roi_height) * frame_height
    accepted = []
    for detection in detections or ():
        cx, cy = detection_center(detection)
        if left <= cx <= right and top <= cy <= bottom:
            accepted.append(detection)
    return accepted


def project_ball_axis_position(
        ball, axis, min_distance_limit_px=24.0,
        width_factor=1.35, endpoint_margin_ratio=0.08,
        mapping_enabled=False, physical_length_cm=25.0,
        enforce_endpoint_bounds=True, enforce_axis_distance=True):
    """把钢球中心投影到横向中轴，返回像素、归一化和线性厘米位置。"""
    mapping_enabled = bool(mapping_enabled)
    physical_length_cm = float(physical_length_cm)
    if mapping_enabled and physical_length_cm <= 0.0:
        raise ValueError("physical_length_cm must be greater than zero")
    mapping_name = (
        "LINEAR_CENTERED_CM" if mapping_enabled
        else "NONE_AXIS_NORMALIZED"
    )
    coordinate_mode = (
        "CENTERED_SIGNED_CM" if mapping_enabled
        else "CENTERED_AXIS_PX_NORM"
    )
    result = {
        "valid": False,
        "reason": "NO_BALL",
        "position_cm": None,
        "position_px": None,
        "position_norm": None,
        "t": None,
        "raw_t": None,
        "ball_cx": None,
        "ball_cy": None,
        "projected_x": None,
        "projected_y": None,
        "axis_distance_px": None,
        "axis_length_px": None,
        "axis_slope_y_per_x": None,
        "axis_intercept_y": None,
        "mapping": mapping_name,
        "coordinate_mode": coordinate_mode,
        "ball_source": "NONE",
    }
    if ball is None:
        return result
    if _axis_polyline(axis) is None:
        result["reason"] = "NO_TROUGH_AXIS"
        return result

    cx, cy = detection_center(ball)
    result["ball_cx"] = cx
    result["ball_cy"] = cy
    result["ball_source"] = str(ball.get("source", "MEASURED"))
    measurement = point_axis_measurement(cx, cy, axis)
    if measurement is None:
        result["reason"] = "BAD_TROUGH_AXIS"
        return result
    result.update(measurement)
    p0 = axis.get("p0")
    p1 = axis.get("p1")
    if p0 is not None and p1 is not None:
        dx = float(p1[0]) - float(p0[0])
        if abs(dx) > 1.0:
            slope = float(axis.get(
                "slope_y_per_x",
                (float(p1[1]) - float(p0[1])) / dx,
            ))
            result["axis_slope_y_per_x"] = slope
            result["axis_intercept_y"] = float(axis.get(
                "intercept_y", float(p0[1]) - slope * float(p0[0])))

    trough_width = max(
        1.0, float(axis.get("median_width_px", 1.0)))
    distance_limit = max(
        float(min_distance_limit_px),
        trough_width * float(width_factor),
    )
    if bool(enforce_axis_distance) and \
            measurement["axis_distance_px"] > distance_limit:
        result["reason"] = "BALL_OFF_TROUGH"
        return result

    margin = max(0.0, float(endpoint_margin_ratio))
    raw_t = float(measurement["raw_t"])
    if bool(enforce_endpoint_bounds) and (
            raw_t < -margin or raw_t > 1.0 + margin):
        result["reason"] = "BALL_OUTSIDE_TROUGH"
        return result

    mapped_t = max(0.0, min(1.0, raw_t))
    axis_length_px = float(measurement["axis_length_px"])
    result.update({
        "valid": True,
        "reason": "OK",
        "position_cm": (
            (mapped_t - 0.5) * physical_length_cm
            if mapping_enabled else None
        ),
        "position_px": (mapped_t - 0.5) * axis_length_px,
        "position_norm": 2.0 * mapped_t - 1.0,
        "t": mapped_t,
    })
    return result
