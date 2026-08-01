"""MaixCAM2 钢球 YOLO11 真机配置探针。

该文件刻意保持单文件、无工程内模块依赖，便于 maixpy.skill 运行器先验证
模型文件、MaixPy 运行库、摄像头、NPU 和屏幕，再调试完整应用。
"""

import gc
import os

from maix import app, camera, display, image, nn, time


MODEL_PATH = (
    "/root/mymodel/"
    "steelball_yolo11n_v4_leftedge_best_320x416_maixcam2.mud"
)
CONF_TH = 0.12
IOU_TH = 0.45
CAMERA_FPS = 60
MAX_FRAMES = 120

DEBUG_IMAGES = os.environ.get("MAIXPY_DEBUG_IMAGES") == "1"


def save_debug_image(img, name):
    if not DEBUG_IMAGES or img is None:
        return
    run_dir = os.environ.get(
        "MAIXPY_SKILL_RUN_DIR", "/tmp/maixpy_skill/manual")
    artifact_dir = "%s/artifacts" % run_dir
    os.makedirs(artifact_dir, exist_ok=True)
    path = "%s/%s" % (artifact_dir, name)
    img.save(path)
    print("MAIXPY_SKILL_IMAGE %s" % path)


def _label(detector, obj):
    try:
        return str(detector.labels[obj.class_id])
    except Exception:
        return "class_%d" % int(obj.class_id)


def main():
    detector = None
    cam = None
    disp = None
    frame = None
    frame_index = 0
    read_attempts = 0
    detection_frames = 0
    total_detections = 0
    detection_image_saved = False
    started_ms = time.ticks_ms()

    try:
        if not os.path.isfile(MODEL_PATH):
            raise FileNotFoundError(
                "missing MUD model or referenced AXMODEL: %s" % MODEL_PATH)

        # 位置调试要求检测框属于当前画面。官方文档说明 dual_buff=True
        # 返回上一帧结果，因此这里与正式控制程序一致地关闭双缓冲。
        detector = nn.YOLO11(model=MODEL_PATH, dual_buff=False)
        model_width = int(detector.input_width())
        model_height = int(detector.input_height())
        model_format = detector.input_format()
        cam = camera.Camera(
            model_width,
            model_height,
            model_format,
            fps=CAMERA_FPS,
        )
        disp = display.Display()

        try:
            actual_fps = float(cam.fps())
        except Exception:
            actual_fps = -1.0

        print(
            "MAIXPY_AI_CONFIG|model=%s|size=%dx%d|format=%s|"
            "dual_buff=0|conf=%.3f|iou=%.3f|cam_fps_req=%d|"
            "cam_fps_actual=%.1f|max_frames=%d"
            % (
                MODEL_PATH,
                model_width,
                model_height,
                str(model_format),
                CONF_TH,
                IOU_TH,
                CAMERA_FPS,
                actual_fps,
                MAX_FRAMES,
            )
        )

        while not app.need_exit() and read_attempts < MAX_FRAMES:
            read_attempts += 1
            frame = cam.read()
            if frame is None:
                print("MAIXPY_AI_FRAME_EMPTY|attempt=%d" % read_attempts)
                continue

            frame_index += 1
            detect_started_ms = time.ticks_ms()
            objects = detector.detect(
                frame,
                conf_th=CONF_TH,
                iou_th=IOU_TH,
            )
            detect_ms = time.ticks_ms() - detect_started_ms

            count = len(objects)
            total_detections += count
            if count:
                detection_frames += 1

            best_score = 0.0
            for obj in objects:
                score = float(obj.score)
                if score > best_score:
                    best_score = score
                frame.draw_rect(
                    int(obj.x), int(obj.y), int(obj.w), int(obj.h),
                    color=image.COLOR_RED,
                )
                frame.draw_string(
                    int(obj.x), max(0, int(obj.y) - 16),
                    "%s %.2f" % (_label(detector, obj), score),
                    color=image.COLOR_RED,
                )

            if DEBUG_IMAGES and frame_index == 1:
                save_debug_image(frame, "ai_probe_first.jpg")
            if DEBUG_IMAGES and count and not detection_image_saved:
                save_debug_image(frame, "ai_probe_detection.jpg")
                detection_image_saved = True

            disp.show(frame)
            if frame_index == 1 or frame_index % 20 == 0 or count:
                print(
                    "MAIXPY_AI_FRAME|frame=%d|detections=%d|"
                    "best_score=%.3f|detect_ms=%d"
                    % (frame_index, count, best_score, detect_ms)
                )

        elapsed_ms = max(1, time.ticks_ms() - started_ms)
        if DEBUG_IMAGES and frame is not None:
            save_debug_image(frame, "ai_probe_last.jpg")
        print(
            "MAIXPY_AI_DONE|frames=%d|detection_frames=%d|"
            "total_detections=%d|elapsed_ms=%d|loop_fps=%.2f"
            % (
                frame_index,
                detection_frames,
                total_detections,
                elapsed_ms,
                frame_index * 1000.0 / elapsed_ms,
            )
        )
    finally:
        disp = None
        cam = None
        detector = None
        gc.collect()
        print("MAIXPY_AI_RESOURCES_CLOSED")


if __name__ == "__main__":
    main()
