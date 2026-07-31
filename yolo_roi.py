"""YOLO局部ROI输入与检测框全图坐标回映。

模型输入尺寸固定，ROI会按比例缩放并上下/左右补黑边到模型尺寸；这样不会
把钢球拉伸变形。检测结果随后裁掉补边区域并映射回原始相机画面坐标。
"""


def roi_pixels(roi_ratio, frame_width, frame_height):
    """把归一化ROI转换为至少1像素的合法整数区域。"""
    rx, ry, rw, rh = [float(value) for value in roi_ratio]
    width = int(frame_width)
    height = int(frame_height)
    x = max(0, min(width - 1, int(round(rx * width))))
    y = max(0, min(height - 1, int(round(ry * height))))
    right = max(x + 1, min(width, int(round((rx + rw) * width))))
    bottom = max(y + 1, min(height, int(round((ry + rh) * height))))
    return x, y, right - x, bottom - y


def _full_metadata(frame_width, frame_height, source="FULL", error=None):
    return {
        "source": source,
        "roi": (0, 0, int(frame_width), int(frame_height)),
        "scale": 1.0,
        "pad_x": 0.0,
        "pad_y": 0.0,
        "content_width": float(frame_width),
        "content_height": float(frame_height),
        "error": error,
    }


def prepare_yolo_input(frame, model_width, model_height,
                       roi_ratio, enabled=True):
    """返回固定模型尺寸的ROI图像及坐标变换信息。

    板端crop/resize接口异常时安全回退全图，避免实验开关让主线退出。
    """
    frame_width = int(frame.width())
    frame_height = int(frame.height())
    if not enabled:
        return frame, _full_metadata(frame_width, frame_height)

    roi = roi_pixels(roi_ratio, frame_width, frame_height)
    roi_x, roi_y, roi_width, roi_height = roi
    if roi == (0, 0, frame_width, frame_height):
        return frame, _full_metadata(frame_width, frame_height)

    try:
        # 延迟导入，PC纯逻辑测试无需安装maix模块。
        from maix import image

        cropped = frame.crop(roi_x, roi_y, roi_width, roi_height)
        resized = cropped.resize(
            int(model_width), int(model_height),
            fit=image.Fit.FIT_CONTAIN,
        )
        if resized is None:
            raise RuntimeError("ROI resize returned None")

        scale = min(
            float(model_width) / max(1.0, float(roi_width)),
            float(model_height) / max(1.0, float(roi_height)),
        )
        content_width = float(roi_width) * scale
        content_height = float(roi_height) * scale
        return resized, {
            "source": "ROI",
            "roi": roi,
            "scale": scale,
            "pad_x": 0.5 * (float(model_width) - content_width),
            "pad_y": 0.5 * (float(model_height) - content_height),
            "content_width": content_width,
            "content_height": content_height,
            "error": None,
        }
    except Exception as error:
        return frame, _full_metadata(
            frame_width, frame_height,
            source="FULL_FALLBACK", error=str(error))


def map_detections_to_frame(detections, metadata):
    """将ROI模型坐标中的字典检测框还原为相机全图坐标。"""
    if not metadata or metadata.get("source") != "ROI":
        return [dict(item) for item in detections or ()]

    roi_x, roi_y, roi_width, roi_height = metadata["roi"]
    scale = max(1e-9, float(metadata["scale"]))
    pad_x = float(metadata["pad_x"])
    pad_y = float(metadata["pad_y"])
    content_right = pad_x + float(metadata["content_width"])
    content_bottom = pad_y + float(metadata["content_height"])
    output = []

    for detection in detections or ():
        x1 = max(pad_x, float(detection["x"]))
        y1 = max(pad_y, float(detection["y"]))
        x2 = min(
            content_right,
            float(detection["x"]) + float(detection["w"]),
        )
        y2 = min(
            content_bottom,
            float(detection["y"]) + float(detection["h"]),
        )
        if x2 - x1 <= 1.0 or y2 - y1 <= 1.0:
            continue

        global_x1 = float(roi_x) + (x1 - pad_x) / scale
        global_y1 = float(roi_y) + (y1 - pad_y) / scale
        global_x2 = min(
            float(roi_x + roi_width),
            float(roi_x) + (x2 - pad_x) / scale,
        )
        global_y2 = min(
            float(roi_y + roi_height),
            float(roi_y) + (y2 - pad_y) / scale,
        )
        item = dict(detection)
        item.update({
            "x": global_x1,
            "y": global_y1,
            "w": global_x2 - global_x1,
            "h": global_y2 - global_y1,
        })
        if item["w"] > 1.0 and item["h"] > 1.0:
            output.append(item)
    return output
