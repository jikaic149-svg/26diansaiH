"""MaixCAM2钢球YOLO + 横向水管凹槽轴向位置初测程序。

数据流：

    当前帧 -> YOLO钢球框 ------------------------┐
           -> LAB颜色二值 -> 逐列白段 -> 横向中轴 ├-> 球心投影
                                                  └-> 厘米 + 像素 + 归一化位置

本入口只负责相机、模型、模块调度、画面和诊断日志；算法参数集中在
``vision_config.py``。F407通信由独立模块负责，可选VOFA+ UDP只用于调试显示。
"""

import gc
import os

from maix import app, camera, display, image, nn, time

import vision_config as config
from ball_target_touch import BallTargetTouchController
from competition_task_manager import CompetitionTaskManager
from ball_axis_position import (
    filter_detections_on_trough,
    highest_score_detection,
    project_ball_axis_position,
    snapshot_detections,
)
from f4_vision_link import F4VisionLink
from kalman_tracker import PrimaryBallTracker
from horizontal_trough_tracker import HorizontalTroughTracker
from quick_position_calibration import (
    apply_calibrated_position,
    get_fixed_position_calibration,
    run_quick_position_calibration,
    transform_calibrated_axis,
)
from rtsp_debug_stream import RtspDebugStream
from threshold_tuning_menu import run_threshold_tuning_menu
from ui_size_calibration import calibrate_frozen_ui
from vofa_udp_telemetry import VofaUdpTelemetry
from yolo_roi import prepare_yolo_input, map_detections_to_frame


RED = image.Color.from_rgb(255, 0, 0)
GREEN = image.Color.from_rgb(0, 255, 0)
BLUE = image.Color.from_rgb(30, 130, 255)
CYAN = image.Color.from_rgb(0, 230, 230)
MAGENTA = image.Color.from_rgb(255, 0, 255)
YELLOW = image.Color.from_rgb(255, 220, 0)
WHITE = image.Color.from_rgb(255, 255, 255)


# maixpy.skill 的真机运行器只在显式传入 --debug-images 时设置该环境变量。
# 正常应用运行不会写图，避免闪存写入和JPEG编码影响视觉闭环时序。
DEBUG_IMAGES = os.environ.get("MAIXPY_DEBUG_IMAGES") == "1"
DEBUG_IMAGE_INTERVAL_FRAMES = 100
DEBUG_MAX_FRAMES = 300
LIVE_CAPTURE_TRIGGER = "/tmp/steelball_live_capture.trigger"
LIVE_CAPTURE_PATH = "/tmp/steelball_live_capture.jpg"


def _save_debug_image(img, name):
    """把当前调试帧保存到运行器可自动拉取的目录。"""
    if not DEBUG_IMAGES or img is None:
        return
    run_dir = os.environ.get(
        "MAIXPY_SKILL_RUN_DIR", "/tmp/maixpy_skill/manual")
    artifact_dir = "%s/artifacts" % run_dir
    os.makedirs(artifact_dir, exist_ok=True)
    path = "%s/%s" % (artifact_dir, name)
    img.save(path)
    print("MAIXPY_SKILL_IMAGE %s" % path)


def _save_requested_live_frame(img, frame_index):
    """Save one annotated frame only when the observer requests it."""
    if img is None or not os.path.exists(LIVE_CAPTURE_TRIGGER):
        return False
    try:
        img.save(LIVE_CAPTURE_PATH)
        print("STEELBALL_LIVE_CAPTURE|frame=%d|path=%s" % (
            int(frame_index), LIVE_CAPTURE_PATH))
        return True
    finally:
        try:
            os.remove(LIVE_CAPTURE_TRIGGER)
        except Exception:
            pass


class PerformanceMeter:
    """按一秒窗口分别统计采集、NPU、凹槽视觉、显示和完整循环。"""

    def __init__(self, window_ms=1000):
        self.window_ms = int(window_ms)
        self.window_start_ms = time.ticks_ms()
        self.counts = {
            "loop": 0,
            "camera": 0,
            "yolo": 0,
            "trough": 0,
            "draw": 0,
            "display": 0,
        }
        self.totals = {
            "camera": 0,
            "yolo": 0,
            "trough": 0,
            "draw": 0,
            "display": 0,
        }
        self.loop_fps = 0.0
        self.yolo_fps = 0.0
        self.display_fps = 0.0
        self.camera_avg_ms = 0.0
        self.yolo_avg_ms = 0.0
        self.trough_avg_ms = 0.0
        self.draw_avg_ms = 0.0
        self.display_avg_ms = 0.0

    def record(self, stage, elapsed_ms):
        self.counts[stage] += 1
        self.totals[stage] += int(elapsed_ms)

    def finish_loop(self):
        self.counts["loop"] += 1
        now_ms = time.ticks_ms()
        elapsed_ms = now_ms - self.window_start_ms
        if elapsed_ms < self.window_ms:
            return
        scale = 1000.0 / max(1, elapsed_ms)
        self.loop_fps = self.counts["loop"] * scale
        self.yolo_fps = self.counts["yolo"] * scale
        self.display_fps = self.counts["display"] * scale
        self.camera_avg_ms = self.totals["camera"] / max(
            1, self.counts["camera"])
        self.yolo_avg_ms = self.totals["yolo"] / max(
            1, self.counts["yolo"])
        self.trough_avg_ms = self.totals["trough"] / max(
            1, self.counts["trough"])
        self.draw_avg_ms = self.totals["draw"] / max(
            1, self.counts["draw"])
        self.display_avg_ms = self.totals["display"] / max(
            1, self.counts["display"])
        self.window_start_ms = now_ms
        for key in self.counts:
            self.counts[key] = 0
        for key in self.totals:
            self.totals[key] = 0


def _trough_settings():
    return {
        "lab_thresholds": config.TROUGH_LAB_THRESHOLDS,
        "roi_ratio": config.TROUGH_ROI_RATIO,
        # 二值调试必须保留全尺寸掩膜；正常闭环才启用ROI裁剪提速。
        "roi_mask_enabled": bool(getattr(
            config, "TROUGH_ROI_MASK_ENABLED", False)) and not bool(
                getattr(config, "SHOW_TROUGH_BINARY", False)),
        "column_step": config.TROUGH_COLUMN_STEP,
        "min_run_height_px": config.TROUGH_MIN_RUN_HEIGHT_PX,
        "max_run_height_ratio_roi":
            config.TROUGH_MAX_RUN_HEIGHT_RATIO_ROI,
        "guided_scan_half_height_px": getattr(
            config, "TROUGH_GUIDED_SCAN_HALF_HEIGHT_PX", 0),
        "max_internal_gap_px": config.TROUGH_MAX_INTERNAL_GAP_PX,
        "local_search_radius_px": config.TROUGH_LOCAL_SEARCH_RADIUS_PX,
        "max_track_gap_x_px": config.TROUGH_MAX_TRACK_GAP_X_PX,
        "height_change_weight": getattr(
            config, "TROUGH_HEIGHT_CHANGE_WEIGHT", 0.15),
        "gap_penalty_weight": getattr(
            config, "TROUGH_GAP_PENALTY_WEIGHT", 0.12),
        "seeded_growth_enabled": getattr(
            config, "TROUGH_SEEDED_GROWTH_ENABLED", True),
        "seed_column_trials": getattr(
            config, "TROUGH_SEED_COLUMN_TRIALS", 11),
        "seed_track_trials": getattr(
            config, "TROUGH_SEED_TRACK_TRIALS", 4),
        "seed_growth_half_height_px": getattr(
            config, "TROUGH_SEED_GROWTH_HALF_HEIGHT_PX", 16),
        "seed_early_accept_span_ratio": getattr(
            config, "TROUGH_SEED_EARLY_ACCEPT_SPAN_RATIO", 0.70),
        "min_points": config.TROUGH_MIN_POINTS,
        "min_length_ratio_frame": config.TROUGH_MIN_LENGTH_RATIO_FRAME,
        "min_fit_residual_px": config.TROUGH_MIN_FIT_RESIDUAL_PX,
        "max_fit_residual_px": config.TROUGH_MAX_FIT_RESIDUAL_PX,
        "ball_exclusion_margin_px":
            config.TROUGH_BALL_EXCLUSION_MARGIN_PX,
        "cache_frames": config.TROUGH_CACHE_FRAMES,
        "update_interval_frames": config.TROUGH_UPDATE_INTERVAL_FRAMES,
        # 兼容板端只更新了main.py而未同步vision_config.py的情况；完整上传时
        # 仍会使用新版参数，缺失时回退到tracker中的安全默认值。
        "endpoint_band_px": getattr(
            config, "TROUGH_ENDPOINT_BAND_PX", 18),
        "endpoint_min_support_points":
            getattr(config, "TROUGH_ENDPOINT_MIN_SUPPORT_POINTS", 3),
        "endpoint_min_aspect_ratio": getattr(
            config, "TROUGH_ENDPOINT_MIN_ASPECT_RATIO", 5.0),
        "endpoint_refine_enabled": getattr(
            config, "TROUGH_ENDPOINT_REFINE_ENABLED", True),
        "endpoint_refine_radius_px": getattr(
            config, "TROUGH_ENDPOINT_REFINE_RADIUS_PX", 12),
        "endpoint_refine_max_misses": getattr(
            config, "TROUGH_ENDPOINT_REFINE_MAX_MISSES", 0),
        "endpoint_search_coarse_step_px": getattr(
            config, "TROUGH_ENDPOINT_SEARCH_COARSE_STEP_PX", 2),
        "endpoint_inner_edge_enabled": getattr(
            config, "TROUGH_ENDPOINT_INNER_EDGE_ENABLED", True),
        "endpoint_inner_search_radius_px": getattr(
            config, "TROUGH_ENDPOINT_INNER_SEARCH_RADIUS_PX", 16),
        "endpoint_inner_min_height_ratio": getattr(
            config, "TROUGH_ENDPOINT_INNER_MIN_HEIGHT_RATIO", 0.55),
        "endpoint_inner_center_tolerance_ratio": getattr(
            config, "TROUGH_ENDPOINT_INNER_CENTER_TOLERANCE_RATIO", 0.28),
        "endpoint_inner_confirm_columns": getattr(
            config, "TROUGH_ENDPOINT_INNER_CONFIRM_COLUMNS", 2),
        "endpoint_occlusion_enabled": getattr(
            config, "TROUGH_ENDPOINT_OCCLUSION_ENABLED", True),
        "endpoint_occlusion_edge_margin_px": getattr(
            config, "TROUGH_ENDPOINT_OCCLUSION_EDGE_MARGIN_PX", 18),
        "endpoint_occlusion_min_inset_px": getattr(
            config, "TROUGH_ENDPOINT_OCCLUSION_MIN_INSET_PX", 8),
        "endpoint_occlusion_max_gap_ratio_roi": getattr(
            config, "TROUGH_ENDPOINT_OCCLUSION_MAX_GAP_RATIO_ROI", 0.18),
        "endpoint_occlusion_release_frames": getattr(
            config, "TROUGH_ENDPOINT_OCCLUSION_RELEASE_FRAMES", 2),
        "axis_filter_enabled": config.TROUGH_AXIS_FILTER_ENABLED,
        "axis_filter_alpha": config.TROUGH_AXIS_FILTER_ALPHA,
        "axis_filter_span_alpha": config.TROUGH_AXIS_FILTER_SPAN_ALPHA,
        "axis_filter_min_confidence":
            config.TROUGH_AXIS_FILTER_MIN_CONFIDENCE,
        "axis_filter_snap_shift_px":
            config.TROUGH_AXIS_FILTER_SNAP_SHIFT_PX,
        "axis_filter_snap_slope_delta":
            config.TROUGH_AXIS_FILTER_SNAP_SLOPE_DELTA,
        "axis_filter_endpoint_max_shift_px":
            getattr(config, "TROUGH_AXIS_FILTER_ENDPOINT_MAX_SHIFT_PX", 24.0),
        "axis_filter_endpoint_min_span_ratio":
            getattr(
                config, "TROUGH_AXIS_FILTER_ENDPOINT_MIN_SPAN_RATIO", 0.75),
        "endpoint_ab_enabled": getattr(
            config, "TROUGH_ENDPOINT_AB_ENABLED", True),
        "endpoint_ab_fixed_right_enabled": getattr(
            config, "TROUGH_ENDPOINT_AB_FIXED_RIGHT_ENABLED", False),
        "endpoint_ab_fixed_right_x": getattr(
            config, "TROUGH_ENDPOINT_AB_FIXED_RIGHT_X", 0.0),
        "endpoint_ab_alpha": getattr(
            config, "TROUGH_ENDPOINT_AB_ALPHA", 0.70),
        "endpoint_ab_beta": getattr(
            config, "TROUGH_ENDPOINT_AB_BETA", 0.15),
        "endpoint_ab_max_innovation_px": getattr(
            config, "TROUGH_ENDPOINT_AB_MAX_INNOVATION_PX", 48.0),
        "endpoint_ab_max_velocity_px_s": getattr(
            config, "TROUGH_ENDPOINT_AB_MAX_VELOCITY_PX_S", 240.0),
        "endpoint_ab_max_predict_px": getattr(
            config, "TROUGH_ENDPOINT_AB_MAX_PREDICT_PX", 12.0),
        "endpoint_ab_reset_gap_ms": getattr(
            config, "TROUGH_ENDPOINT_AB_RESET_GAP_MS", 600),
        "endpoint_ab_nominal_dt_ms": getattr(
            config, "TROUGH_ENDPOINT_AB_NOMINAL_DT_MS", 100),
        "curved_centerline_enabled": getattr(
            config, "TROUGH_CURVED_CENTERLINE_ENABLED", True),
        "centerline_segments": getattr(
            config, "TROUGH_CENTERLINE_SEGMENTS", 6),
        "centerline_residual_filter_enabled": getattr(
            config, "TROUGH_CENTERLINE_RESIDUAL_FILTER_ENABLED", True),
        "centerline_residual_filter_alpha": getattr(
            config, "TROUGH_CENTERLINE_RESIDUAL_FILTER_ALPHA", 0.20),
        "centerline_residual_max_step_px": getattr(
            config, "TROUGH_CENTERLINE_RESIDUAL_MAX_STEP_PX", 1.5),
        "positive_at_right": config.TROUGH_POSITIVE_AT_RIGHT,
    }


def _measured_ball(detection):
    if detection is None:
        return None
    output = dict(detection)
    output["source"] = "YOLO_RAW"
    return output


def _direct_frame_axis(frame, detection):
    """凹槽失效时用整幅水平轴做临时25 cm线性闭环。"""
    if frame is None or detection is None:
        return None
    center_y = float(detection["y"]) + 0.5 * float(detection["h"])
    p0 = (0.0, center_y)
    p1 = (float(max(1, frame.width() - 1)), center_y)
    return {
        "valid": True,
        "reason": "DIRECT_FRAME_AXIS",
        "source": "HELD",
        "fallback_reason": "TROUGH_INVALID",
        "orientation": "HORIZONTAL",
        "p0": p0,
        "p1": p1,
        "centerline": [p0, p1],
        "points": [],
        "inliers": [],
        "roi": None,
        "confidence": 0.0,
    }


def _format_number(value, digits=2, missing="-1"):
    if value is None:
        return missing
    return ("%%.%df" % int(digits)) % float(value)


def _elapsed_ms(now_ms, previous_ms):
    """计算Maix ticks_ms差值，兼容计数器回绕。

    MaixCDK签名是 ``ticks_diff(last, now)``，与MicroPython常见的
    ``ticks_diff(now, last)`` 顺序相反；传入ticks_ms时返回值仍是毫秒差。
    """
    ticks_diff = getattr(time, "ticks_diff", None)
    if ticks_diff is not None:
        try:
            return max(0, int(ticks_diff(
                int(previous_ms), int(now_ms))))
        except Exception:
            pass
    return max(0, (int(now_ms) - int(previous_ms)) & 0xFFFFFFFF)


def _draw_raw_boxes(frame, detections):
    if not config.DRAW_RAW_YOLO_BOXES:
        return
    for detection in detections:
        x = int(round(detection["x"]))
        y = int(round(detection["y"]))
        width = int(round(detection["w"]))
        height = int(round(detection["h"]))
        frame.draw_rect(x, y, width, height, color=RED, thickness=1)
        frame.draw_string(
            x,
            max(18, y) - 16,
            "ball %.2f" % float(detection.get("score", 0.0)),
            color=RED,
            scale=0.55,
        )


def _draw_tracked_ball(frame, tracked_ball):
    if tracked_ball is None:
        return
    source = tracked_ball.get("source")
    cx = float(tracked_ball["x"]) + 0.5 * float(tracked_ball["w"])
    cy = float(tracked_ball["y"]) + 0.5 * float(tracked_ball["h"])
    if source == "YOLO_RAW":
        # 原始YOLO红框已经由_draw_raw_boxes绘制，此处只补绿色原始球心，
        # 避免重复绿框盖住红框，让人误以为正常帧仍经过卡尔曼平滑。
        frame.draw_cross(
            int(round(cx)), int(round(cy)),
            color=GREEN, size=7, thickness=2)
        return

    frame.draw_rect(
        int(round(tracked_ball["x"])),
        int(round(tracked_ball["y"])),
        int(round(tracked_ball["w"])),
        int(round(tracked_ball["h"])),
        color=YELLOW,
        thickness=2,
    )
    frame.draw_cross(
        int(round(cx)), int(round(cy)), color=YELLOW, size=7, thickness=2)


def _draw_trough(frame, trough_axis):
    if not trough_axis:
        return
    roi = trough_axis.get("roi")
    if roi is not None:
        frame.draw_rect(
            int(roi[0]), int(roi[1]), int(roi[2]), int(roi[3]),
            color=YELLOW, thickness=1)

    points = trough_axis.get("inliers") or \
        trough_axis.get("points") or []
    if config.DRAW_TROUGH_SAMPLE_POINTS:
        step = max(1, len(points) // 35)
        for point in points[::step]:
            frame.draw_cross(
                int(point["x"]), int(point["top"]),
                color=GREEN, size=2, thickness=1)
            frame.draw_cross(
                int(point["x"]), int(point["bottom"]),
                color=BLUE, size=2, thickness=1)

    if not trough_axis.get("valid"):
        return
    p0 = trough_axis["p0"]
    p1 = trough_axis["p1"]
    centerline = trough_axis.get("centerline") or [p0, p1]
    for start, end in zip(centerline, centerline[1:]):
        frame.draw_line(
            int(round(start[0])), int(round(start[1])),
            int(round(end[0])), int(round(end[1])),
            color=CYAN, thickness=2)
    frame.draw_cross(
        int(round(p0[0])), int(round(p0[1])),
        color=YELLOW, size=8, thickness=2)
    frame.draw_cross(
        int(round(p1[0])), int(round(p1[1])),
        color=MAGENTA, size=8, thickness=2)


def _draw_position(frame, position):
    if not position.get("valid"):
        return
    frame.draw_cross(
        int(round(position["projected_x"])),
        int(round(position["projected_y"])),
        color=MAGENTA,
        size=10,
        thickness=2,
    )


def _draw_status(frame, perf, detections, trough_axis,
                 tracked_ball, position, f4_status=None,
                 competition_task=None):
    # 不再铺设实心黑色状态栏，让文字直接叠加在实时画面上。
    valid = bool(position.get("valid"))
    status_color = GREEN if valid else RED
    if valid and position.get("position_cm") is not None:
        position_text = "%+.2fcm %+.1fpx n:%+.3f" % (
            float(position["position_cm"]),
            float(position["position_px"]),
            float(position["position_norm"]),
        )
    elif valid:
        position_text = "%+.1fpx n:%+.3f" % (
            float(position["position_px"]),
            float(position["position_norm"]),
        )
    else:
        position_text = position.get("reason", "INVALID")
    ball_source = (
        str(tracked_ball.get("source", "M")) if tracked_ball else "-")
    trough_source = str(trough_axis.get(
        "source", "-")) if trough_axis else "-"
    competition_mode = int((f4_status or {}).get("competition_mode", 0))
    mode_text = str(competition_mode) if 2 <= competition_mode <= 6 else "-"
    frame.draw_string(
        3, 3,
        "AXIS %s t:%s M%s" % (
            position_text,
            _format_number(position.get("t"), 3, "-"),
            mode_text),
        color=status_color,
        scale=0.68,
    )
    if not bool(getattr(config, "DISPLAY_DETAILED_STATUS", True)):
        frame.draw_string(
            3, 27,
            "FPS A:%.1f D:%.1f AI:%.1f P:%.1f" % (
                perf.loop_fps, perf.display_fps,
                perf.yolo_avg_ms, perf.trough_avg_ms),
            color=WHITE,
            scale=0.55,
        )
        return
    frame.draw_string(
        3, 26,
        "BALLS:%d T:%s TROUGH:%s P:%d" % (
            len(detections), ball_source, trough_source,
            len(trough_axis.get("inliers", [])) if trough_axis else 0),
        color=WHITE,
        scale=0.58,
    )
    frame.draw_string(
        3, 47,
        "FPS L:%.1f Y:%.1f AI:%.1fms" % (
            perf.loop_fps, perf.yolo_fps, perf.yolo_avg_ms),
        color=WHITE,
        scale=0.58,
    )
    frame.draw_string(
        3, 68,
        "CAM:%.1f TR:%.1f SHOW:%.1fms" % (
            perf.camera_avg_ms, perf.trough_avg_ms,
            perf.display_avg_ms),
        color=WHITE,
        scale=0.58,
    )


def _log_result(frame_index, detections, candidates, tracked_ball,
                trough_axis, position, perf,
                current_ai_ms, current_trough_ms,
                frame_timestamp_ms, measurement_age_ms,
                send_timestamp_ms):
    print(
        "BALL_AXIS|frame=%d|frame_ts_ms=%d|measure_age_ms=%d|"
        "send_ts_ms=%d|valid=%d|reason=%s|position_cm=%s|position_px=%s|"
        "position_norm=%s|coord_mode=%s|t=%s|"
        "ball_source=%s|ball_cx=%s|ball_cy=%s|balls=%d|candidates=%d|"
        "trough_valid=%d|trough_source=%s|trough_reason=%s|"
        "trough_fallback=%s|"
        "axis_filter=%s|axis_conf=%s|axis_filter_shift_px=%s|"
        "endpoint_filter=%s|endpoint_mode=%s|endpoint_ab=%s|"
        "endpoint_raw_left_x=%s|endpoint_v_left=%s|"
        "endpoint_predict_px_left=%s|endpoint_support=%s|"
        "endpoint_left_support=%s|endpoint_right_support=%s|"
        "endpoint_quality=%s|endpoint_refine=%s|"
        "endpoint_refine_left_px=%s|endpoint_refine_right_px=%s|"
        "centerline=%s|centerline_nodes=%d|centerline_curve_px=%s|"
        "centerline_residual_filter=%s|centerline_residual_shift_px=%s|"
        "axis_p0_x=%s|axis_p1_x=%s|aspect_ratio=%s|"
        "trough_scan=%s|trough_points=%d|"
        "trough_length_px=%s|trough_thickness_px=%s|fit_rms_px=%s|"
        "mask_scope=%s|mask_ms=%d|scan_ms=%d|"
        "axis_distance_px=%s|axis_yolo_fallback=%d|mapping=%s|"
        "ai_ms=%d|trough_ms=%d|fps=%.2f|"
        "display_fps=%.2f|cam_avg_ms=%.2f|ai_avg_ms=%.2f|"
        "trough_avg_ms=%.2f|draw_avg_ms=%.2f|show_avg_ms=%.2f"
        % (
            frame_index,
            int(frame_timestamp_ms) & 0xFFFFFFFF,
            int(measurement_age_ms),
            int(send_timestamp_ms) & 0xFFFFFFFF,
            1 if position.get("valid") else 0,
            position.get("reason", "UNKNOWN"),
            _format_number(position.get("position_cm"), 3),
            _format_number(position.get("position_px"), 3),
            _format_number(position.get("position_norm"), 5),
            position.get("coordinate_mode", "UNKNOWN"),
            _format_number(position.get("t"), 5),
            tracked_ball.get("source", "NONE") if tracked_ball else "NONE",
            _format_number(position.get("ball_cx"), 2),
            _format_number(position.get("ball_cy"), 2),
            len(detections),
            len(candidates),
            1 if trough_axis and trough_axis.get("valid") else 0,
            trough_axis.get("source", "NONE") if trough_axis else "NONE",
            trough_axis.get("reason", "NONE") if trough_axis else "NONE",
            trough_axis.get("fallback_reason", "NONE")
            if trough_axis else "NONE",
            trough_axis.get("axis_filter", "NONE") if trough_axis else "NONE",
            _format_number(
                trough_axis.get("confidence") if trough_axis else None, 3),
            _format_number(
                trough_axis.get("axis_filter_shift_px")
                if trough_axis else None, 2),
            trough_axis.get("endpoint_filter", "NONE")
            if trough_axis else "NONE",
            trough_axis.get("endpoint_mode", "NONE")
            if trough_axis else "NONE",
            trough_axis.get("endpoint_ab", "NONE")
            if trough_axis else "NONE",
            _format_number(
                trough_axis.get("endpoint_raw_left_x")
                if trough_axis else None, 2),
            _format_number(
                trough_axis.get("endpoint_v_left")
                if trough_axis else None, 2),
            _format_number(
                trough_axis.get("endpoint_predict_px_left")
                if trough_axis else None, 2),
            _format_number(
                trough_axis.get("endpoint_support")
                if trough_axis else None, 3),
            _format_number(
                trough_axis.get("endpoint_left_support")
                if trough_axis else None, 3),
            _format_number(
                trough_axis.get("endpoint_right_support")
                if trough_axis else None, 3),
            trough_axis.get("endpoint_quality", "NONE")
            if trough_axis else "NONE",
            trough_axis.get("endpoint_refine", "NONE")
            if trough_axis else "NONE",
            _format_number(
                trough_axis.get("endpoint_refine_left_px")
                if trough_axis else None, 2),
            _format_number(
                trough_axis.get("endpoint_refine_right_px")
                if trough_axis else None, 2),
            trough_axis.get("centerline_mode", "NONE")
            if trough_axis else "NONE",
            len(trough_axis.get("centerline", []))
            if trough_axis else 0,
            _format_number(
                trough_axis.get("centerline_curve_px")
                if trough_axis else None, 2),
            trough_axis.get("centerline_residual_filter", "NONE")
            if trough_axis else "NONE",
            _format_number(
                trough_axis.get("centerline_residual_shift_px")
                if trough_axis else None, 2),
            _format_number(
                trough_axis.get("p0", (None, None))[0]
                if trough_axis and trough_axis.get("p0") else None, 2),
            _format_number(
                trough_axis.get("p1", (None, None))[0]
                if trough_axis and trough_axis.get("p1") else None, 2),
            _format_number(
                trough_axis.get("aspect_ratio")
                if trough_axis else None, 2),
            trough_axis.get("scan_mode", "NONE") if trough_axis else "NONE",
            len(trough_axis.get("inliers", [])) if trough_axis else 0,
            _format_number(
                trough_axis.get("length_px") if trough_axis else None, 2),
            _format_number(
                trough_axis.get("median_thickness_px")
                if trough_axis else None, 2),
            _format_number(
                trough_axis.get("fit_rms_px")
                if trough_axis else None, 2),
            trough_axis.get("mask_scope", "NONE")
            if trough_axis else "NONE",
            int(trough_axis.get("mask_ms", 0)) if trough_axis else 0,
            int(trough_axis.get("scan_ms", 0)) if trough_axis else 0,
            _format_number(position.get("axis_distance_px"), 2),
            int(position.get("axis_yolo_fallback", 0)),
            position.get("mapping", "NONE"),
            int(current_ai_ms),
            int(current_trough_ms),
            perf.loop_fps,
            perf.display_fps,
            perf.camera_avg_ms,
            perf.yolo_avg_ms,
            perf.trough_avg_ms,
            perf.draw_avg_ms,
            perf.display_avg_ms,
        )
    )


def main():
    detector = None
    cam = None
    stream_cam = None
    disp = None
    trough_tracker = None
    position_calibration = None
    ball_tracker = None
    rtsp_debug = None
    rtsp_active = False
    vofa = None
    vofa_active = False
    f4_link = None
    f4_active = False
    ball_target_touch = None
    competition_task = None
    try:
        # 位置计算要求框与横向凹槽来自当前帧，因此保持dual_buff=False。
        detector = nn.YOLO11(model=config.MODEL_PATH, dual_buff=False)
        camera_fps = float(getattr(config, "CAMERA_FPS", 60))
        rtsp_requested = bool(getattr(
            config, "RTSP_DEBUG_ENABLED", False))

        if rtsp_requested:
            try:
                rtsp_debug = RtspDebugStream(config)
                # RTSP硬件编码绑定NV21主通道；算法读取同一传感器的模型格式
                # 子通道。必须先add_channel，再bind_camera；绑定后主循环不再
                # 直接read() stream_cam。
                stream_cam = camera.Camera(
                    detector.input_width(),
                    detector.input_height(),
                    image.Format.FMT_YVU420SP,
                    fps=camera_fps,
                )
                cam = stream_cam.add_channel(
                    detector.input_width(),
                    detector.input_height(),
                    detector.input_format(),
                )
                if cam is None:
                    raise RuntimeError(
                        "create model-format camera sub-channel failed")
                rtsp_active = bool(rtsp_debug.open(stream_cam))
            except Exception as exc:
                print("RTSP_CAMERA_ERROR|%s" % exc)
                if rtsp_debug is not None:
                    rtsp_debug.close()
                rtsp_debug = None
                cam = None
                stream_cam = None
                gc.collect()
                if bool(getattr(config, "RTSP_REQUIRED", False)):
                    raise
                print("RTSP_CAMERA_FALLBACK|single_channel=1")

        if cam is None:
            # 图传关闭或可选图传初始化失败时，保持原来的单通道主线。
            cam = camera.Camera(
                detector.input_width(),
                detector.input_height(),
                detector.input_format(),
                fps=camera_fps,
            )
        display_enabled = bool(getattr(config, "DISPLAY_ENABLED", True))
        display_every_frame = bool(getattr(
            config, "DISPLAY_EVERY_FRAME", False))
        show_trough_binary = bool(getattr(
            config, "SHOW_TROUGH_BINARY", False))
        display_interval = max(
            1, int(getattr(config, "DISPLAY_INTERVAL_FRAMES", 1)))
        if display_every_frame:
            display_interval = 1
        disp = display.Display() if display_enabled else None
        if disp is not None and bool(getattr(
                config, "UI_SIZE_FROZEN", False)):
            frozen_ui = calibrate_frozen_ui(
                detector.input_width(), detector.input_height(),
                disp.width(), disp.height(),
                int(getattr(config, "UI_FROZEN_FRAME_WIDTH", 416)),
                int(getattr(config, "UI_FROZEN_FRAME_HEIGHT", 320)),
                int(getattr(config, "UI_FROZEN_DISPLAY_WIDTH", 480)),
                int(getattr(config, "UI_FROZEN_DISPLAY_HEIGHT", 640)),
            )
            content = frozen_ui["content"]
            slider_touch = frozen_ui["slider_touch"]
            print(
                "UI_SIZE_FROZEN|frame=%dx%d|display=%dx%d|"
                "content=%d,%d,%d,%d|slider_touch=%d,%d,%d,%d" % (
                    frozen_ui["frame_width"], frozen_ui["frame_height"],
                    frozen_ui["display_width"],
                    frozen_ui["display_height"],
                    content["x"], content["y"],
                    content["width"], content["height"],
                    slider_touch["x"], slider_touch["y"],
                    slider_touch["width"], slider_touch["height"],
                ))
        # 交互式阈值菜单可独立关闭；关闭时直接使用vision_config中的固定值。
        threshold_tuning_enabled = bool(getattr(
            config, "STARTUP_THRESHOLD_TUNING_ENABLED", True))
        if threshold_tuning_enabled:
            threshold_selection = run_threshold_tuning_menu(
                cam, disp, config)
            if threshold_selection is None:
                return
        else:
            print(
                "THRESHOLD_MENU_SKIPPED|mode=%s|lab=%s" % (
                    str(getattr(config, "TROUGH_COLOR_MODE", "CUSTOM")),
                    str(getattr(config, "TROUGH_LAB_THRESHOLDS", ())),
                )
            )
        try:
            camera_actual_fps = float(cam.fps())
        except Exception:
            camera_actual_fps = -1.0
        perf = PerformanceMeter(1000)
        trough_tracker = HorizontalTroughTracker(
            _trough_settings(), clock_ms=time.ticks_ms,
            clock_diff_ms=_elapsed_ms)
        quick_calibration_enabled = bool(getattr(
            config, "QUICK_POSITION_CALIBRATION_ENABLED", True))
        fixed_calibration_enabled = bool(getattr(
            config, "FIXED_POSITION_CALIBRATION_ENABLED", False))
        if fixed_calibration_enabled:
            try:
                position_calibration = get_fixed_position_calibration()
            except Exception as error:
                print("FIXED_CAL_INVALID|error=%s" % error)
                return
            print(
                "FIXED_CAL_LOADED|version=%d|pairs=%d|virtual_right=%d"
                % (
                    int(position_calibration.get("version", 0)),
                    len(position_calibration.get("pairs") or []),
                    1 if position_calibration.get("virtual_right") else 0,
                )
            )
        elif quick_calibration_enabled:
            position_calibration = run_quick_position_calibration(
                cam, disp, detector, trough_tracker, config)
            if position_calibration is None:
                return
        else:
            position_calibration = {
                "enabled": False,
                "reason": "DISABLED_BY_CONFIG",
            }
            print(
                "QUICK_CAL_SKIPPED|reason=DISABLED_BY_CONFIG|"
                "mapping=LINEAR_%.2fCM" % float(getattr(
                    config, "TROUGH_PHYSICAL_LENGTH_CM", 25.0))
            )
        if config.TRACKER_ENABLED:
            ball_tracker = PrimaryBallTracker()

        vofa_requested = bool(getattr(config, "VOFA_UDP_ENABLED", False))
        if vofa_requested:
            vofa = VofaUdpTelemetry(
                enabled=True,
                host=str(getattr(
                    config, "VOFA_UDP_HOST", "255.255.255.255")),
                port=int(getattr(config, "VOFA_UDP_PORT", 1347)),
                interval_ms=int(getattr(
                    config, "VOFA_UDP_INTERVAL_MS", 25)),
                broadcast=bool(getattr(
                    config, "VOFA_UDP_BROADCAST", True)),
            )
            vofa_active = bool(vofa.open())

        f4_requested = bool(getattr(config, "F4_UART_ENABLED", True))
        if f4_requested:
            f4_link = F4VisionLink(
                enabled=True,
                required=bool(getattr(config, "F4_UART_REQUIRED", False)),
                send_interval_ms=int(getattr(
                    config, "F4_UART_SEND_INTERVAL_MS", 20)),
                tx_timeout_ms=int(getattr(
                    config, "F4_UART_TX_TIMEOUT_MS", 100)),
                rx_limit=int(getattr(config, "F4_UART_RX_LIMIT", 256)),
                clock_ms=time.ticks_ms,
            )
            f4_active = bool(f4_link.start())
            if f4_active and bool(getattr(
                    config, "F4_PID_TUNING_ENABLED", False)):
                try:
                    tuning_queued = f4_link.send_pid_gains(
                        float(getattr(config, "F4_PID_KP_PER_S2", 1.2)),
                        float(getattr(config, "F4_PID_KI_PER_S3", 0.15)),
                        float(getattr(config, "F4_PID_KD_PER_S", 1.8)),
                    )
                    print(
                        "F4_PID_TUNING_REQUEST|queued=%d|"
                        "kp=%.3f|ki=%.3f|kd=%.3f" % (
                            1 if tuning_queued else 0,
                            float(getattr(
                                config, "F4_PID_KP_PER_S2", 1.2)),
                            float(getattr(
                                config, "F4_PID_KI_PER_S3", 0.15)),
                            float(getattr(
                                config, "F4_PID_KD_PER_S", 1.8)),
                        )
                    )
                except Exception as error:
                    print("F4_PID_TUNING_ERROR|%s" % error)

        touch_target_enabled = bool(getattr(
            config, "F4_TOUCH_TARGET_ENABLED", False))
        competition_task_enabled = bool(getattr(
            config, "F4_COMPETITION_TASK_ENABLED", True))
        if touch_target_enabled or competition_task_enabled:
            ball_target_touch = BallTargetTouchController(
                enabled=touch_target_enabled,
                minimum_mm=float(getattr(
                    config, "F4_TOUCH_TARGET_MIN_MM", -120.0)),
                maximum_mm=float(getattr(
                    config, "F4_TOUCH_TARGET_MAX_MM", 120.0)),
                step_mm=float(getattr(
                    config, "F4_TOUCH_TARGET_STEP_MM", 1.0)),
                retry_ms=int(getattr(
                    config, "F4_TOUCH_TARGET_RETRY_MS", 250)),
                max_attempts=int(getattr(
                    config, "F4_TOUCH_TARGET_MAX_ATTEMPTS", 20)),
                offline_ms=int(getattr(
                    config, "F4_TOUCH_TARGET_OFFLINE_MS", 500)),
                target_to_f4_polarity=float(getattr(
                    config, "F4_TOUCH_TARGET_TO_F407_POLARITY", -1.0)),
                question3_enabled=bool(getattr(
                    config, "F4_TOUCH_QUESTION3_ENABLED", True)),
                question3_button_enabled=bool(getattr(
                    config, "F4_TOUCH_QUESTION3_BUTTON_ENABLED", False)),
                question3_center_mm=float(getattr(
                    config, "F4_TOUCH_QUESTION3_CENTER_MM", 0.0)),
                question3_center_tolerance_mm=float(getattr(
                    config, "F4_TOUCH_QUESTION3_CENTER_TOLERANCE_MM", 10.0)),
                question3_right_command_mm=float(getattr(
                    config, "F4_TOUCH_QUESTION3_RIGHT_COMMAND_MM", 50.0)),
                question3_right_reached_mm=float(getattr(
                    config, "F4_TOUCH_QUESTION3_RIGHT_REACHED_MM", 50.0)),
                question3_left_command_mm=float(getattr(
                    config, "F4_TOUCH_QUESTION3_LEFT_COMMAND_MM", -50.0)),
                question3_left_reached_mm=float(getattr(
                    config, "F4_TOUCH_QUESTION3_LEFT_REACHED_MM", -50.0)),
                question3_final_target_mm=float(getattr(
                    config, "F4_TOUCH_QUESTION3_FINAL_TARGET_MM", -50.0)),
                question3_limit_ms=int(getattr(
                    config, "F4_TOUCH_QUESTION3_LIMIT_MS", 5000)),
                question3_hard_timeout_ms=int(getattr(
                    config, "F4_TOUCH_QUESTION3_HARD_TIMEOUT_MS", 8000)),
            )
            if touch_target_enabled and disp is not None:
                ball_target_touch.start()
            elif touch_target_enabled:
                print("BALL_TARGET_TOUCH_UNAVAILABLE|error=DISPLAY_DISABLED")
            if competition_task_enabled:
                competition_task = CompetitionTaskManager(
                    ball_target_touch,
                    offline_ms=int(getattr(
                        config, "F4_TOUCH_TARGET_OFFLINE_MS", 500)),
                    center_target_mm=float(getattr(
                        config, "F4_TOUCH_QUESTION3_CENTER_MM", 0.0)),
                )

        frame_index = 0
        # SHOW_TROUGH_BINARY开启时，隔帧复用中轴也要复用最近一次二值图，
        # 否则trough_mask=None会让显示逻辑退回原图，造成画面闪烁。
        last_trough_mask = None
        last_yolo_input_source = None
        print(
            "STEELBALL_HORIZONTAL_START|model=%s|size=%dx%d|"
            "trough_color=%s|trough=%d|binary_view=%d|trough_roi_mask=%d|"
            "mapping_enabled=%d|trough_length_cm=%.2f|positive_at_right=%d|"
            "threshold_menu=%d|quick_calibration=%d|f4_calibration_valid=%d|"
            "cam_fps_req=%.1f|cam_fps_actual=%.1f|display_interval=%d|"
            "display_every_frame=%d|"
            "trough_column_step=%d|endpoint_refine_radius=%d|"
            "centerline_segments=%d|guided_half_height=%d|draw_samples=%d|"
            "rtsp_requested=%d|rtsp_active=%d|rtsp_wifi=%s|rtsp_url=%s|"
            "vofa_requested=%d|vofa_active=%d|vofa_target=%s:%d|"
            "f4_requested=%d|f4_active=%d"
            % (
                config.MODEL_PATH,
                detector.input_width(),
                detector.input_height(),
                str(getattr(config, "TROUGH_COLOR_MODE", "CUSTOM")),
                1 if config.TROUGH_TRACKING_ENABLED else 0,
                1 if show_trough_binary else 0,
                1 if bool(getattr(
                    config, "TROUGH_ROI_MASK_ENABLED", False)) and not bool(
                        show_trough_binary) else 0,
                1 if bool(getattr(
                    config, "POSITION_MAPPING_ENABLED", False)) else 0,
                float(getattr(config, "TROUGH_PHYSICAL_LENGTH_CM", 25.0)),
                1 if config.TROUGH_POSITIVE_AT_RIGHT else 0,
                1 if threshold_tuning_enabled else 0,
                1 if quick_calibration_enabled else 0,
                1 if bool(getattr(
                    config, "F4_UART_CALIBRATION_VALID", False)) else 0,
                camera_fps,
                camera_actual_fps,
                display_interval,
                1 if display_every_frame else 0,
                int(config.TROUGH_COLUMN_STEP),
                int(getattr(
                    config, "TROUGH_ENDPOINT_REFINE_RADIUS_PX", 12)),
                int(getattr(config, "TROUGH_CENTERLINE_SEGMENTS", 6)),
                int(getattr(
                    config, "TROUGH_GUIDED_SCAN_HALF_HEIGHT_PX", 0)),
                1 if bool(getattr(
                    config, "DRAW_TROUGH_SAMPLE_POINTS", True)) else 0,
                1 if rtsp_requested else 0,
                1 if rtsp_active else 0,
                str(getattr(config, "RTSP_WIFI_MODE", "EXISTING")),
                rtsp_debug.url if rtsp_debug is not None and
                rtsp_active else "-",
                1 if vofa_requested else 0,
                1 if vofa_active else 0,
                str(getattr(config, "VOFA_UDP_HOST", "-")),
                int(getattr(config, "VOFA_UDP_PORT", 1347)),
                1 if f4_requested else 0,
                1 if f4_active else 0,
            )
        )

        while not app.need_exit():
            link_now_ms = time.ticks_ms()
            if f4_link is not None and f4_active:
                f4_link.service(link_now_ms)
            if ball_target_touch is not None:
                ball_target_touch.update_feedback(
                    f4_link.last_vst if f4_link is not None else None)
                # 已经处于等待确认状态时，即使相机偶发空帧，也继续按有限周期重发。
                ball_target_touch.try_send(f4_link, link_now_ms)
            camera_started_ms = time.ticks_ms()
            frame = cam.read()
            perf.record("camera", time.ticks_ms() - camera_started_ms)
            if frame is None:
                continue
            # MaixPy未必提供传感器曝光时间，这里记录read()完成时刻，
            # 作为本帧可用时间的近似采集时间。
            frame_timestamp_ms = time.ticks_ms()
            frame_index += 1
            if ball_target_touch is not None:
                ball_target_touch.poll_touch(frame, disp)

            yolo_started_ms = time.ticks_ms()
            yolo_input, yolo_metadata = prepare_yolo_input(
                frame,
                detector.input_width(),
                detector.input_height(),
                getattr(config, "YOLO_ROI_RATIO", config.TROUGH_ROI_RATIO),
                enabled=bool(getattr(config, "YOLO_ROI_ENABLED", False)),
            )
            objects = detector.detect(
                yolo_input,
                conf_th=config.CONF_TH,
                iou_th=config.IOU_TH,
            )
            current_ai_ms = time.ticks_ms() - yolo_started_ms
            perf.record("yolo", current_ai_ms)
            detections = map_detections_to_frame(
                snapshot_detections(objects), yolo_metadata)
            del objects
            if yolo_input is not frame:
                del yolo_input
            yolo_input_source = yolo_metadata.get("source", "FULL")
            if yolo_input_source != last_yolo_input_source:
                print(
                    "YOLO_INPUT|source=%s|roi=%s|error=%s" % (
                        yolo_input_source,
                        str(yolo_metadata.get("roi")),
                        yolo_metadata.get("error") or "NONE",
                    )
                )
                last_yolo_input_source = yolo_input_source

            preliminary = highest_score_detection(detections)
            trough_started_ms = time.ticks_ms()
            trough_mask = None
            if config.TROUGH_TRACKING_ENABLED:
                trough_axis, trough_mask = trough_tracker.update(
                    frame, exclusion_box=preliminary)
                trough_axis = transform_calibrated_axis(
                    trough_axis, position_calibration)
                if show_trough_binary and trough_mask is not None:
                    last_trough_mask = trough_mask
            else:
                trough_axis = {
                    "valid": False,
                    "reason": "DISABLED",
                    "source": "NONE",
                    "orientation": "HORIZONTAL",
                    "inliers": [],
                }
                last_trough_mask = None
            current_trough_ms = time.ticks_ms() - trough_started_ms
            perf.record("trough", current_trough_ms)

            calibration_enabled = bool(position_calibration.get("enabled"))
            candidates = filter_detections_on_trough(
                detections,
                trough_axis,
                min_distance_limit_px=
                    config.BALL_MIN_AXIS_DISTANCE_LIMIT_PX,
                width_factor=config.BALL_AXIS_DISTANCE_WIDTH_FACTOR,
                endpoint_margin_ratio=config.BALL_ENDPOINT_MARGIN_RATIO,
                enforce_endpoint_bounds=not calibration_enabled,
            )
            axis_yolo_fallback = False
            if calibration_enabled and not candidates and \
                    preliminary is not None and \
                    float(preliminary.get("score", 0.0)) >= float(getattr(
                        config, "CALIBRATED_YOLO_AXIS_FALLBACK_CONF_TH",
                        0.45)):
                candidates = [preliminary]
                axis_yolo_fallback = True
            now_ms = time.ticks_ms()
            if ball_tracker is not None:
                tracked_ball = ball_tracker.update(candidates, now_ms)
            else:
                tracked_ball = _measured_ball(
                    highest_score_detection(candidates))

            position = project_ball_axis_position(
                tracked_ball,
                trough_axis,
                min_distance_limit_px=
                    config.BALL_MIN_AXIS_DISTANCE_LIMIT_PX,
                width_factor=config.BALL_AXIS_DISTANCE_WIDTH_FACTOR,
                endpoint_margin_ratio=config.BALL_ENDPOINT_MARGIN_RATIO,
                mapping_enabled=bool(getattr(
                    config, "POSITION_MAPPING_ENABLED", False)),
                physical_length_cm=float(getattr(
                    config, "TROUGH_PHYSICAL_LENGTH_CM", 25.0)),
                enforce_endpoint_bounds=not calibration_enabled,
                enforce_axis_distance=not axis_yolo_fallback,
            )
            position = apply_calibrated_position(
                position, position_calibration)
            position["axis_yolo_fallback"] = (
                1 if axis_yolo_fallback else 0)

            # 记录视觉结果真正准备好并可发送的时刻，供MCU补偿算法延迟。
            send_timestamp_ms = time.ticks_ms()
            measurement_age_ms = _elapsed_ms(
                send_timestamp_ms, frame_timestamp_ms)
            position.update({
                "frame_timestamp_ms": int(frame_timestamp_ms),
                "send_timestamp_ms": int(send_timestamp_ms),
                "measurement_age_ms": int(measurement_age_ms),
                "frame_index": int(frame_index),
            })

            if competition_task is not None:
                competition_task.update(
                    position,
                    f4_link.last_vst if f4_link is not None else None,
                    send_timestamp_ms)

            if vofa is not None and vofa_active:
                vofa.send_position(
                    position,
                    tracked_ball=tracked_ball,
                    trough_axis=trough_axis,
                    now_ms=send_timestamp_ms,
                    measurement_age_ms=measurement_age_ms,
                    frame_timestamp_ms=frame_timestamp_ms,
                    frame_index=frame_index,
                    f4_status=(
                        f4_link.last_vst if f4_link is not None else None),
                )

            if f4_link is not None and f4_active:
                if ball_target_touch is not None:
                    # VBT优先于本轮VIS；F407仍负责完整杆球模型与加速度前馈。
                    ball_target_touch.try_send(
                        f4_link, send_timestamp_ms)
                f4_link.send_position(
                    position,
                    tracked_ball=tracked_ball,
                    trough_axis=trough_axis,
                    sample_ms=frame_timestamp_ms,
                    now_ms=send_timestamp_ms,
                    calibration_valid=bool(getattr(
                        config, "F4_UART_CALIBRATION_VALID", False)),
                )

            display_frame = None
            should_display = display_enabled and (
                display_every_frame or
                (frame_index - 1) % display_interval == 0
            )
            if should_display:
                draw_started_ms = time.ticks_ms()
                display_frame = frame
                binary_for_display = (
                    trough_mask if trough_mask is not None
                    else last_trough_mask
                )
                if show_trough_binary and \
                        binary_for_display is not None:
                    try:
                        converted = binary_for_display.to_format(
                            image.Format.FMT_RGB888)
                        display_frame = converted \
                            if converted is not None else binary_for_display
                    except Exception:
                        display_frame = binary_for_display

                _draw_raw_boxes(display_frame, detections)
                _draw_trough(display_frame, trough_axis)
                _draw_tracked_ball(display_frame, tracked_ball)
                _draw_position(display_frame, position)
                _draw_status(
                    display_frame, perf, detections,
                    trough_axis, tracked_ball, position,
                    f4_link.last_vst if f4_link is not None else None,
                    competition_task)
                if competition_task is not None:
                    display_frame.draw_string(
                        3, 89, competition_task.status_text(),
                        color=YELLOW, scale=0.55)
                if ball_target_touch is not None:
                    ball_target_touch.draw(
                        display_frame, time.ticks_ms(),
                        trough_axis=trough_axis,
                        calibration_pairs=(
                            position_calibration.get("pairs")
                            if position_calibration else None),
                    )
                _save_requested_live_frame(display_frame, frame_index)
                perf.record("draw", time.ticks_ms() - draw_started_ms)

                if DEBUG_IMAGES and (
                        frame_index == 1 or
                        frame_index % DEBUG_IMAGE_INTERVAL_FRAMES == 0):
                    _save_debug_image(
                        display_frame, "steelball_display_current.jpg")
                    # 二值调试显示时，额外保留未替换成掩膜的相机画面。
                    if display_frame is not frame:
                        _save_debug_image(
                            frame, "steelball_camera_current.jpg")

                display_started_ms = time.ticks_ms()
                disp.show(display_frame)
                perf.record(
                    "display", time.ticks_ms() - display_started_ms)
            perf.finish_loop()

            if frame_index % max(1, config.PRINT_INTERVAL_FRAMES) == 0:
                _log_result(
                    frame_index, detections, candidates, tracked_ball,
                    trough_axis, position, perf,
                    current_ai_ms, current_trough_ms,
                    frame_timestamp_ms, measurement_age_ms,
                    send_timestamp_ms)

            trough_mask = None
            display_frame = None
            frame = None

            if DEBUG_IMAGES and frame_index >= DEBUG_MAX_FRAMES:
                print("STEELBALL_HORIZONTAL_DEBUG_DONE|frames=%d" %
                      frame_index)
                break

    finally:
        if ball_target_touch is not None:
            ball_target_touch.close()
        ball_target_touch = None
        competition_task = None
        if f4_link is not None:
            f4_link.close()
        f4_link = None
        if vofa is not None:
            vofa.close()
        vofa = None
        # RTSP必须先停止，之后才能释放被编码器绑定的主相机通道。
        if rtsp_debug is not None:
            rtsp_debug.close()
        rtsp_debug = None
        ball_tracker = None
        trough_tracker = None
        position_calibration = None
        disp = None
        cam = None
        stream_cam = None
        detector = None
        gc.collect()
        print("STEELBALL_HORIZONTAL_RESOURCES_CLOSED")


if __name__ == "__main__":
    main()
