"""Pure UI size calibration for the frozen MaixCAM2 display layout."""


def fit_contain_rect(source_width, source_height,
                     display_width, display_height):
    source_width = int(source_width)
    source_height = int(source_height)
    display_width = int(display_width)
    display_height = int(display_height)
    if min(source_width, source_height,
           display_width, display_height) <= 0:
        raise ValueError("UI dimensions must be positive")
    scale = min(
        float(display_width) / source_width,
        float(display_height) / source_height)
    content_width = int(round(source_width * scale))
    content_height = int(round(source_height * scale))
    return {
        "x": (display_width - content_width) // 2,
        "y": (display_height - content_height) // 2,
        "width": content_width,
        "height": content_height,
    }

def map_frame_rect_to_display(rectangle, frame_width, frame_height,
                              display_width, display_height):
    content = fit_contain_rect(
        frame_width, frame_height, display_width, display_height)
    x0 = content["x"] + int(round(
        float(rectangle["x"]) * content["width"] / frame_width))
    y0 = content["y"] + int(round(
        float(rectangle["y"]) * content["height"] / frame_height))
    x1 = content["x"] + int(round(
        float(rectangle["x"] + rectangle["width"]) *
        content["width"] / frame_width))
    y1 = content["y"] + int(round(
        float(rectangle["y"] + rectangle["height"]) *
        content["height"] / frame_height))
    return {
        "x": x0,
        "y": y0,
        "width": max(1, x1 - x0),
        "height": max(1, y1 - y0),
    }


def calibrate_frozen_ui(frame_width, frame_height,
                        display_width, display_height,
                        expected_frame_width, expected_frame_height,
                        expected_display_width, expected_display_height):
    actual = (
        int(frame_width), int(frame_height),
        int(display_width), int(display_height))
    expected = (
        int(expected_frame_width), int(expected_frame_height),
        int(expected_display_width), int(expected_display_height))
    if actual != expected:
        raise RuntimeError(
            "frozen UI size mismatch: actual=%dx%d->%dx%d "
            "expected=%dx%d->%dx%d" % (actual + expected))

    slider_frame = {
        "x": 12,
        "y": frame_height - 58,
        "width": frame_width - 24,
        "height": 58,
    }
    return {
        "frame_width": int(frame_width),
        "frame_height": int(frame_height),
        "display_width": int(display_width),
        "display_height": int(display_height),
        "content": fit_contain_rect(
            frame_width, frame_height, display_width, display_height),
        "slider_touch": map_frame_rect_to_display(
            slider_frame, frame_width, frame_height,
            display_width, display_height),
    }
