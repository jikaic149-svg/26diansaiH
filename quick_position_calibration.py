"""钢球位置快速现场标定与虚拟右端锚定模板变换。

颜色阈值菜单完成后，依次采集-10/-5/0/+5/+10 cm五个中间点，再把
钢球放到左机械极限验证-12 cm。+10 cm点同时用于推算画面外的固定右端；
+12 cm能拍到时可继续验证，拍不到时允许跳过并沿+5到+10 cm末段外推。

摄像头与右端机构固定，运行时使用当前检测到的左端点对参考中轴做二维旋转
缩放；现有颜色扫描、中轴拟合和滤波算法保持不变。
"""

import gc

from maix import app, image, touchscreen

from ball_axis_position import (
    filter_detections_on_trough,
    highest_score_detection,
    point_axis_measurement,
    project_ball_axis_position,
    snapshot_detections,
)
from yolo_roi import prepare_yolo_input, map_detections_to_frame


MIDDLE_TARGETS_CM = (-10.0, -5.0, 0.0, 5.0, 10.0)
LEFT_ENDPOINT_CM = -12.0
RIGHT_ENDPOINT_CM = 12.0
CAPTURE_SAMPLES = 20
MAX_CAPTURE_MAD_T = 0.015
MIN_POINT_GAP_T = 0.01
ENDPOINT_TOLERANCE_CM = 0.60
# 凹槽物理边缘为-12.5～+12.5 cm；+10 cm球心位于全长的90%处。
TROUGH_LEFT_EDGE_CM = -12.5
TROUGH_RIGHT_EDGE_CM = 12.5
VIRTUAL_RIGHT_ANCHOR_CM = 10.0
VIRTUAL_RIGHT_ANCHOR_FRACTION = (
    (VIRTUAL_RIGHT_ANCHOR_CM - TROUGH_LEFT_EDGE_CM) /
    (TROUGH_RIGHT_EDGE_CM - TROUGH_LEFT_EDGE_CM)
)

WHITE = image.Color.from_rgb(255, 255, 255)
BLACK = image.Color.from_rgb(0, 0, 0)
RED = image.Color.from_rgb(255, 0, 0)
GREEN = image.Color.from_rgb(0, 255, 0)
CYAN = image.Color.from_rgb(0, 230, 230)
YELLOW = image.Color.from_rgb(255, 220, 0)
MAGENTA = image.Color.from_rgb(255, 0, 255)


def _median(values):
    ordered = sorted(float(value) for value in values)
    if not ordered:
        return None
    middle = len(ordered) // 2
    if len(ordered) % 2:
        return ordered[middle]
    return 0.5 * (ordered[middle - 1] + ordered[middle])


def _mad(values, center=None):
    if not values:
        return None
    center = _median(values) if center is None else float(center)
    return _median([abs(float(value) - center) for value in values])


def _polyline_length(points):
    total = 0.0
    for start, end in zip(points, points[1:]):
        dx = float(end[0]) - float(start[0])
        dy = float(end[1]) - float(start[1])
        total += (dx * dx + dy * dy) ** 0.5
    return total


def map_t_to_cm(raw_t, pairs):
    """按t排序后分段插值；两端使用首末两段做线性外推。"""
    ordered = sorted(
        (float(item[0]), float(item[1])) for item in pairs)
    if len(ordered) < 2:
        return None
    target = float(raw_t)
    if target <= ordered[0][0]:
        left, right = ordered[0], ordered[1]
    elif target >= ordered[-1][0]:
        left, right = ordered[-2], ordered[-1]
    else:
        left, right = ordered[0], ordered[1]
        for candidate_left, candidate_right in zip(
                ordered, ordered[1:]):
            if candidate_left[0] <= target <= candidate_right[0]:
                left, right = candidate_left, candidate_right
                break
    span = right[0] - left[0]
    if abs(span) <= 1e-9:
        return None
    ratio = (target - left[0]) / span
    return left[1] + ratio * (right[1] - left[1])


def _axis_snapshot(axis):
    if not axis or not axis.get("valid"):
        return None
    centerline = axis.get("centerline") or []
    if len(centerline) < 2:
        return None
    return {
        "valid": True,
        "centerline": [
            (float(point[0]), float(point[1])) for point in centerline
        ],
        "median_width_px": float(axis.get("median_width_px", 1.0)),
        "roi": axis.get("roi"),
    }


def build_reference_axis(axis_samples):
    """对同一参考姿态的多帧中轴逐节点取中位数。"""
    valid = [item for item in axis_samples if item]
    if not valid:
        return None
    counts = {}
    for item in valid:
        count = len(item.get("centerline") or [])
        counts[count] = counts.get(count, 0) + 1
    point_count = max(counts, key=lambda count: counts[count])
    aligned = [
        item for item in valid
        if len(item.get("centerline") or []) == point_count
    ]
    if point_count < 2 or not aligned:
        return None

    centerline = []
    for index in range(point_count):
        centerline.append((
            _median([item["centerline"][index][0] for item in aligned]),
            _median([item["centerline"][index][1] for item in aligned]),
        ))
    return {
        "valid": True,
        "reason": "OK",
        "source": "CAL_REFERENCE",
        "p0": centerline[0],
        "p1": centerline[-1],
        "centerline": centerline,
        "length_px": _polyline_length(centerline),
        "median_width_px": _median([
            item.get("median_width_px", 1.0) for item in aligned
        ]),
        "roi": aligned[-1].get("roi"),
    }


def build_virtual_right_reference(reference_axis, records):
    """由左端和+10 cm投影点反推固定的画面外右端。

    +10 cm球心距左侧物理边缘22.5 cm，占25 cm凹槽全长的90%。因此
    虚拟右端R0=L0+(P10-L0)/0.9。保留原中轴内部节点，只裁掉超过R0的
    节点并在末端加入R0，避免依赖当前画面是否拍全真实右端。
    """
    if not reference_axis or not reference_axis.get("valid"):
        return None
    centerline = reference_axis.get("centerline") or []
    left = reference_axis.get("p0")
    if left is None or len(centerline) < 2:
        return None
    anchor = next(
        (record for record in records
         if abs(float(record.get("cm", 999.0)) -
                VIRTUAL_RIGHT_ANCHOR_CM) <= 1e-6),
        None,
    )
    if anchor is None:
        return None
    measurement = point_axis_measurement(
        anchor["cx"], anchor["cy"], reference_axis)
    if measurement is None:
        return None

    fraction = float(VIRTUAL_RIGHT_ANCHOR_FRACTION)
    left_x, left_y = float(left[0]), float(left[1])
    anchor_x = float(measurement["projected_x"])
    anchor_y = float(measurement["projected_y"])
    right_x = left_x + (anchor_x - left_x) / fraction
    right_y = left_y + (anchor_y - left_y) / fraction
    vector_x = right_x - left_x
    vector_y = right_y - left_y
    length_sq = vector_x * vector_x + vector_y * vector_y
    if length_sq <= 1.0:
        return None

    virtual_centerline = [(left_x, left_y)]
    for point in centerline[1:]:
        point_x, point_y = float(point[0]), float(point[1])
        progress = (
            (point_x - left_x) * vector_x +
            (point_y - left_y) * vector_y
        ) / length_sq
        if 1e-6 < progress < 1.0 - 1e-6:
            virtual_centerline.append((point_x, point_y))
    virtual_centerline.append((right_x, right_y))

    output = dict(reference_axis)
    output.update({
        "p0": virtual_centerline[0],
        "p1": virtual_centerline[-1],
        "centerline": virtual_centerline,
        "length_px": _polyline_length(virtual_centerline),
        "virtual_right": 1,
        "virtual_right_x": right_x,
        "virtual_right_y": right_y,
        "virtual_right_anchor_cm": VIRTUAL_RIGHT_ANCHOR_CM,
        "virtual_right_anchor_fraction": fraction,
    })
    return output


def _extrapolated_t_for_cm(target_cm, pairs):
    """用最后两个标定点反算右侧目标厘米对应的t，仅用于跳过诊断。"""
    ordered = sorted(
        (float(item[1]), float(item[0])) for item in pairs)
    if len(ordered) < 2:
        return None
    left, right = ordered[-2], ordered[-1]
    span_cm = right[0] - left[0]
    if abs(span_cm) <= 1e-9:
        return None
    ratio = (float(target_cm) - left[0]) / span_cm
    return left[1] + ratio * (right[1] - left[1])


def transform_calibrated_axis(raw_axis, calibration):
    """以标定右端p1为固定点，用当前左端p0旋转缩放参考中轴。"""
    if not calibration or not calibration.get("enabled"):
        return raw_axis
    reference = calibration.get("reference_axis") or {}
    if not raw_axis or not raw_axis.get("valid") or \
            not reference.get("valid"):
        return raw_axis
    current_left = raw_axis.get("p0")
    reference_left = reference.get("p0")
    fixed_right = reference.get("p1")
    centerline = reference.get("centerline") or []
    if current_left is None or reference_left is None or \
            fixed_right is None or len(centerline) < 2:
        return raw_axis

    v0x = float(reference_left[0]) - float(fixed_right[0])
    v0y = float(reference_left[1]) - float(fixed_right[1])
    vx = float(current_left[0]) - float(fixed_right[0])
    vy = float(current_left[1]) - float(fixed_right[1])
    denominator = v0x * v0x + v0y * v0y
    if denominator <= 1.0:
        return raw_axis
    a = (vx * v0x + vy * v0y) / denominator
    b = (v0x * vy - v0y * vx) / denominator
    scale = (a * a + b * b) ** 0.5
    if scale < 0.45 or scale > 1.55:
        output = dict(raw_axis)
        output["calibration_axis_reason"] = "SCALE_REJECTED"
        output["calibration_axis_scale"] = scale
        return output

    transformed = []
    rx, ry = float(fixed_right[0]), float(fixed_right[1])
    for point in centerline:
        dx = float(point[0]) - rx
        dy = float(point[1]) - ry
        transformed.append((
            rx + a * dx - b * dy,
            ry + b * dx + a * dy,
        ))

    output = dict(raw_axis)
    output.update({
        "p0": transformed[0],
        "p1": transformed[-1],
        "centerline": transformed,
        "length_px": _polyline_length(transformed),
        "calibration_axis": 1,
        "calibration_axis_reason": "OK",
        "calibration_axis_scale": scale,
        "calibration_left_x": float(current_left[0]),
        "calibration_left_y": float(current_left[1]),
        "calibration_right_x": rx,
        "calibration_right_y": ry,
    })
    return output


def apply_calibrated_position(position, calibration):
    """用现场t-cm查表覆盖原线性厘米值，保留原值用于诊断。"""
    output = dict(position or {})
    if not calibration or not calibration.get("enabled") or \
            not output.get("valid") or output.get("t") is None:
        return output
    calibrated_cm = map_t_to_cm(
        output["t"], calibration.get("pairs") or [])
    if calibrated_cm is None:
        output["valid"] = False
        output["reason"] = "CALIBRATION_MAPPING_FAILED"
        return output
    output["position_cm_linear"] = output.get("position_cm")
    output["position_cm"] = calibrated_cm
    output["mapping"] = "QUICK_CAL_%d" % len(
        calibration.get("pairs") or [])
    output["coordinate_mode"] = "CENTERED_SIGNED_CM_CALIBRATED"
    return output


def _point_in_rect(x, y, rect):
    rx, ry, width, height = rect
    return rx <= x < rx + width and ry <= y < ry + height


def _map_touch(raw_x, raw_y, frame, disp):
    try:
        mapped = image.resize_map_pos_reverse(
            frame.width(), frame.height(),
            disp.width(), disp.height(), image.Fit.FIT_CONTAIN,
            int(raw_x), int(raw_y))
        return int(mapped[0]), int(mapped[1])
    except Exception:
        return (
            int(float(raw_x) * frame.width() / max(1, disp.width())),
            int(float(raw_y) * frame.height() / max(1, disp.height())),
        )


def _draw_text(frame, x, y, text, color=WHITE):
    frame.draw_string(int(x) + 1, int(y) + 1, str(text), color=BLACK)
    frame.draw_string(int(x), int(y), str(text), color=color)


def _buttons(width, height):
    margin = 8
    gap = 8
    button_height = max(42, min(52, height // 6))
    button_width = (width - margin * 2 - gap) // 2
    y = height - button_height - 5
    return {
        "RESET": (margin, y, button_width, button_height),
        "ACTION": (margin + button_width + gap,
                   y, button_width, button_height),
    }


def _draw_button(frame, rect, label, color):
    x, y, width, height = rect
    frame.draw_rect(x, y, width, height, color=color, thickness=2)
    text_x = x + max(4, (width - len(label) * 8) // 2)
    _draw_text(frame, text_x, y + max(4, (height - 16) // 2), label, color)


def _draw_axis(frame, axis):
    if not axis or not axis.get("valid"):
        return
    points = axis.get("centerline") or []
    for start, end in zip(points, points[1:]):
        frame.draw_line(
            int(round(start[0])), int(round(start[1])),
            int(round(end[0])), int(round(end[1])),
            color=CYAN, thickness=2)
    if points:
        frame.draw_cross(
            int(round(points[0][0])), int(round(points[0][1])),
            color=YELLOW, size=7, thickness=2)
        frame.draw_cross(
            int(round(points[-1][0])), int(round(points[-1][1])),
            color=MAGENTA, size=7, thickness=2)


def _draw_ball(frame, ball):
    if not ball:
        return
    x = int(round(ball["x"]))
    y = int(round(ball["y"]))
    width = int(round(ball["w"]))
    height = int(round(ball["h"]))
    frame.draw_rect(x, y, width, height, color=RED, thickness=2)
    frame.draw_cross(
        int(round(float(ball["x"]) + 0.5 * float(ball["w"]))),
        int(round(float(ball["y"]) + 0.5 * float(ball["h"]))),
        color=GREEN, size=7, thickness=2)


def _target_for_phase(phase, middle_index):
    if phase == "MIDDLE":
        return MIDDLE_TARGETS_CM[middle_index]
    if phase == "LEFT":
        return LEFT_ENDPOINT_CM
    if phase == "RIGHT":
        return RIGHT_ENDPOINT_CM
    return None


def _pairs_from_records(records, reference_axis):
    pairs = []
    for record in records:
        measurement = point_axis_measurement(
            record["cx"], record["cy"], reference_axis)
        if measurement is None:
            return None
        pairs.append((float(measurement["raw_t"]), float(record["cm"])))
    pairs.sort(key=lambda item: item[1])
    if any(
        pairs[index + 1][0] - pairs[index][0] < MIN_POINT_GAP_T
        for index in range(len(pairs) - 1)
    ):
        return None
    return pairs


def _finish_capture(samples):
    t_values = [item["t"] for item in samples]
    center_t = _median(t_values)
    return {
        "cx": _median([item["cx"] for item in samples]),
        "cy": _median([item["cy"] for item in samples]),
        "t": center_t,
        "mad_t": _mad(t_values, center_t),
    }


def run_quick_position_calibration(
        cam, disp, detector, trough_tracker, config):
    """运行五点标定与可选端点验证；完成返回标定字典，退出返回None。"""
    if cam is None or disp is None:
        print("QUICK_CAL_SKIPPED|reason=CAMERA_OR_DISPLAY_UNAVAILABLE")
        return {"enabled": False, "reason": "NO_DISPLAY"}
    try:
        touch = touchscreen.TouchScreen()
    except Exception as error:
        print("QUICK_CAL_SKIPPED|reason=TOUCH_UNAVAILABLE|error=%s" % error)
        return {"enabled": False, "reason": "NO_TOUCH"}

    phase = "MIDDLE"
    middle_index = 0
    records = []
    axis_samples = []
    pairs = None
    reference_axis = None
    endpoint_checks = []
    capture_active = False
    capture_samples = []
    status = "Keep lift fixed; place ball then CAPTURE"
    was_pressed = False
    frame = None
    detections = []
    trough_axis = None
    selected_ball = None
    last_yolo_input_source = None

    def reset_all():
        return (
            "MIDDLE", 0, [], [], None, None, [],
            False, [], "Calibration reset",
        )

    try:
        while not app.need_exit():
            frame = cam.read()
            if frame is None:
                continue

            yolo_input, yolo_metadata = prepare_yolo_input(
                frame,
                detector.input_width(),
                detector.input_height(),
                getattr(config, "YOLO_ROI_RATIO", config.TROUGH_ROI_RATIO),
                enabled=bool(getattr(config, "YOLO_ROI_ENABLED", False)),
            )
            objects = detector.detect(
                yolo_input,
                conf_th=float(config.CONF_TH),
                iou_th=float(config.IOU_TH),
            )
            detections = map_detections_to_frame(
                snapshot_detections(objects), yolo_metadata)
            del objects
            if yolo_input is not frame:
                del yolo_input
            yolo_input_source = yolo_metadata.get("source", "FULL")
            if yolo_input_source != last_yolo_input_source:
                print(
                    "QUICK_CAL_YOLO_INPUT|source=%s|roi=%s|error=%s"
                    % (
                        yolo_input_source,
                        str(yolo_metadata.get("roi")),
                        yolo_metadata.get("error") or "NONE",
                    )
                )
                last_yolo_input_source = yolo_input_source
            preliminary = highest_score_detection(detections)
            trough_axis, _ = trough_tracker.update(
                frame, exclusion_box=preliminary)
            candidates = filter_detections_on_trough(
                detections,
                trough_axis,
                min_distance_limit_px=config.BALL_MIN_AXIS_DISTANCE_LIMIT_PX,
                width_factor=config.BALL_AXIS_DISTANCE_WIDTH_FACTOR,
                endpoint_margin_ratio=config.BALL_ENDPOINT_MARGIN_RATIO,
            )
            selected_ball = highest_score_detection(candidates)
            measurement = project_ball_axis_position(
                selected_ball,
                trough_axis,
                min_distance_limit_px=config.BALL_MIN_AXIS_DISTANCE_LIMIT_PX,
                width_factor=config.BALL_AXIS_DISTANCE_WIDTH_FACTOR,
                endpoint_margin_ratio=config.BALL_ENDPOINT_MARGIN_RATIO,
                mapping_enabled=False,
            )

            pressed = False
            touch_x = -1
            touch_y = -1
            try:
                raw_x, raw_y, raw_pressed = touch.read()
                pressed = bool(raw_pressed)
                if pressed:
                    touch_x, touch_y = _map_touch(
                        raw_x, raw_y, frame, disp)
            except Exception as error:
                print("QUICK_CAL_TOUCH_ERROR|%s" % error)
            clicked = pressed and not was_pressed
            was_pressed = pressed
            buttons = _buttons(frame.width(), frame.height())

            if clicked and _point_in_rect(
                    touch_x, touch_y, buttons["RESET"]):
                if phase == "RIGHT" and pairs is not None:
                    extrapolated_t = _extrapolated_t_for_cm(
                        RIGHT_ENDPOINT_CM, pairs)
                    capture_active = False
                    capture_samples = []
                    endpoint_checks.append({
                        "side": "RIGHT",
                        "target_cm": RIGHT_ENDPOINT_CM,
                        "predicted_cm": RIGHT_ENDPOINT_CM,
                        "error_cm": None,
                        "added": False,
                        "skipped": True,
                        "extrapolated_t": extrapolated_t,
                    })
                    print(
                        "QUICK_CAL_ENDPOINT|side=RIGHT|"
                        "target_cm=%+.1f|mode=VIRTUAL|t=%s|"
                        "added=0|skipped=1"
                        % (
                            RIGHT_ENDPOINT_CM,
                            "%.6f" % extrapolated_t
                            if extrapolated_t is not None else "-1",
                        )
                    )
                    phase = "DONE"
                    status = "RIGHT virtual; press START"
                else:
                    (phase, middle_index, records, axis_samples,
                     pairs, reference_axis, endpoint_checks,
                     capture_active, capture_samples, status) = reset_all()
            elif clicked and _point_in_rect(
                    touch_x, touch_y, buttons["ACTION"]):
                if phase == "DONE":
                    right_check = endpoint_checks[1]
                    calibration = {
                        "enabled": True,
                        "version": 2,
                        "pairs": sorted(pairs, key=lambda item: item[0]),
                        "reference_axis": reference_axis,
                        "endpoint_checks": list(endpoint_checks),
                        "middle_points": len(MIDDLE_TARGETS_CM),
                        "point_count": len(pairs),
                        "virtual_right": True,
                        "right_validation_skipped": bool(
                            right_check.get("skipped", False)),
                    }
                    print(
                        "QUICK_CAL_START|pairs=%d|left_added=%d|"
                        "right_added=%d|right_skipped=%d|virtual_right=1|"
                        "virtual_right_x=%.2f|virtual_right_y=%.2f"
                        % (
                            len(pairs),
                            1 if endpoint_checks[0]["added"] else 0,
                            1 if right_check["added"] else 0,
                            1 if right_check.get("skipped", False) else 0,
                            float(reference_axis["virtual_right_x"]),
                            float(reference_axis["virtual_right_y"]),
                        )
                    )
                    return calibration
                capture_active = True
                capture_samples = []
                status = "Capturing stable YOLO + axis samples"

            if capture_active and measurement.get("valid"):
                cx = float(measurement["ball_cx"])
                cy = float(measurement["ball_cy"])
                capture_samples.append({
                    "cx": cx,
                    "cy": cy,
                    "t": float(measurement["raw_t"]),
                    "axis": _axis_snapshot(trough_axis),
                })

            if capture_active and len(capture_samples) >= CAPTURE_SAMPLES:
                completed_samples = capture_samples
                result = _finish_capture(completed_samples)
                capture_active = False
                capture_samples = []
                if result["mad_t"] > MAX_CAPTURE_MAD_T:
                    status = "UNSTABLE: MAD %.4f; recapture" % result["mad_t"]
                elif phase == "MIDDLE":
                    target_cm = MIDDLE_TARGETS_CM[middle_index]
                    axis_samples.extend([
                        item["axis"] for item in completed_samples
                        if item.get("axis") is not None
                    ])
                    records.append({
                        "cm": target_cm,
                        "cx": result["cx"],
                        "cy": result["cy"],
                        "mad_t": result["mad_t"],
                    })
                    print(
                        "QUICK_CAL_POINT|cm=%+.1f|raw_t=%.6f|mad_t=%.6f"
                        % (target_cm, result["t"], result["mad_t"]))
                    middle_index += 1
                    if middle_index < len(MIDDLE_TARGETS_CM):
                        status = "Saved; move ball to next marked point"
                    else:
                        reference_axis = build_reference_axis(axis_samples)
                        reference_axis = build_virtual_right_reference(
                            reference_axis, records)
                        pairs = _pairs_from_records(records, reference_axis)
                        if reference_axis is None or pairs is None:
                            (phase, middle_index, records, axis_samples,
                             pairs, reference_axis, endpoint_checks,
                             capture_active, capture_samples,
                             status) = reset_all()
                            status = "POINT ORDER/AXIS ERROR; redo five points"
                        else:
                            print(
                                "QUICK_CAL_VIRTUAL_RIGHT|anchor_cm=%+.1f|"
                                "fraction=%.3f|x=%.2f|y=%.2f|length_px=%.2f"
                                % (
                                    VIRTUAL_RIGHT_ANCHOR_CM,
                                    VIRTUAL_RIGHT_ANCHOR_FRACTION,
                                    float(reference_axis["virtual_right_x"]),
                                    float(reference_axis["virtual_right_y"]),
                                    float(reference_axis["length_px"]),
                                )
                            )
                            phase = "LEFT"
                            status = "Five points OK; verify LEFT limit"
                else:
                    endpoint_cm = (
                        LEFT_ENDPOINT_CM if phase == "LEFT"
                        else RIGHT_ENDPOINT_CM)
                    endpoint_measurement = point_axis_measurement(
                        result["cx"], result["cy"], reference_axis)
                    endpoint_t = (
                        endpoint_measurement["raw_t"]
                        if endpoint_measurement is not None else None)
                    order_ok = endpoint_t is not None and (
                        endpoint_t < pairs[0][0] - MIN_POINT_GAP_T
                        if phase == "LEFT"
                        else endpoint_t > pairs[-1][0] + MIN_POINT_GAP_T
                    )
                    if not order_ok:
                        status = "%s endpoint order invalid; recapture" % phase
                    else:
                        predicted_cm = map_t_to_cm(endpoint_t, pairs)
                        error_cm = abs(predicted_cm - endpoint_cm)
                        added = error_cm > ENDPOINT_TOLERANCE_CM
                        if added:
                            pairs.append((float(endpoint_t), endpoint_cm))
                            pairs.sort(key=lambda item: item[0])
                        endpoint_checks.append({
                            "side": phase,
                            "target_cm": endpoint_cm,
                            "predicted_cm": predicted_cm,
                            "error_cm": error_cm,
                            "added": added,
                            "skipped": False,
                        })
                        print(
                            "QUICK_CAL_ENDPOINT|side=%s|target_cm=%+.1f|"
                            "predicted_cm=%+.3f|error_cm=%.3f|"
                            "added=%d|skipped=0"
                            % (phase, endpoint_cm, predicted_cm,
                               error_cm, 1 if added else 0))
                        if phase == "LEFT":
                            phase = "RIGHT"
                            status = "LEFT %s; CAPTURE or SKIP RIGHT" % (
                                "ADDED" if added else "PASS")
                        else:
                            phase = "DONE"
                            status = "Endpoint check done; press START"

            _draw_axis(frame, trough_axis)
            _draw_ball(frame, selected_ball)
            target_cm = _target_for_phase(phase, middle_index)
            _draw_text(frame, 5, 4, "QUICK POSITION CAL", YELLOW)
            if phase == "DONE":
                _draw_text(
                    frame, 5, 27,
                    "5 middle + %d endpoint points" % (len(pairs) - 5),
                    GREEN)
                for index, check in enumerate(endpoint_checks):
                    if check.get("skipped"):
                        summary = "%s +12: VIRTUAL" % check["side"]
                        summary_color = CYAN
                    else:
                        summary = "%s pred:%+.2f err:%.2f %s" % (
                            check["side"], check["predicted_cm"],
                            check["error_cm"],
                            "ADDED" if check["added"] else "PASS")
                        summary_color = (
                            YELLOW if check["added"] else GREEN)
                    _draw_text(
                        frame, 5, 50 + index * 23,
                        summary, summary_color)
            else:
                _draw_text(
                    frame, 5, 27,
                    "Place ball at %+.1f cm" % target_cm,
                    MAGENTA)
                _draw_text(
                    frame, 5, 50,
                    "samples:%d/%d  ball:%s axis:%s" % (
                        len(capture_samples), CAPTURE_SAMPLES,
                        "OK" if selected_ball else "NO",
                        "OK" if trough_axis and trough_axis.get("valid")
                        else "NO"),
                    GREEN if measurement.get("valid") else RED,
                )
                if measurement.get("valid"):
                    _draw_text(
                        frame, 5, 73,
                        "raw t:%.5f" % float(measurement["raw_t"]), CYAN)
            _draw_text(frame, 5, 101, status, WHITE)
            _draw_button(
                frame, buttons["RESET"],
                "SKIP +12" if phase == "RIGHT" else "RESET",
                MAGENTA if phase == "RIGHT" else CYAN)
            _draw_button(
                frame, buttons["ACTION"],
                "START" if phase == "DONE" else "CAPTURE",
                GREEN if phase == "DONE" else YELLOW,
            )
            disp.show(frame)
            frame = None
            detections = []
        return None
    finally:
        frame = None
        detections = []
        selected_ball = None
        touch = None
        gc.collect()
