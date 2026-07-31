"""横向水管颜色阈值触摸调节菜单。

运行流程：

1. 一级菜单选择绿色水管或白色水管；
2. 二级菜单显示当前LAB阈值生成的实时二值图；
3. 点击参数行选择Lmin/Lmax/Amin/Amax/Bmin/Bmax；
4. 点击-3或+3单步调整，点击START进入原正式程序。

本模块只修改本次进程内的 ``vision_config`` 对象，不写配置文件。重新运行或
断电后仍从 ``vision_config.py`` 中的当前默认阈值开始。
"""

import gc

from maix import app, image, touchscreen


WHITE = image.Color.from_rgb(255, 255, 255)
BLACK = image.Color.from_rgb(0, 0, 0)
GREEN = image.Color.from_rgb(0, 255, 0)
CYAN = image.Color.from_rgb(0, 230, 230)
YELLOW = image.Color.from_rgb(255, 220, 0)
MAGENTA = image.Color.from_rgb(255, 0, 255)

PARAMETERS = (
    ("Lmin", 0, 100, 0),
    ("Lmax", 0, 100, 1),
    ("Amin", -128, 127, 2),
    ("Amax", -128, 127, 3),
    ("Bmin", -128, 127, 4),
    ("Bmax", -128, 127, 5),
)
STEP = 3


def _point_in_rect(x, y, rect):
    rx, ry, width, height = rect
    return rx <= x < rx + width and ry <= y < ry + height


def _first_threshold(value, fallback):
    """读取当前配置中的第一组LAB阈值，同时兼容tuple/list两种写法。"""
    try:
        first = value[0]
        if len(first) == 6:
            return [int(item) for item in first]
    except Exception:
        pass
    return [int(item) for item in fallback]


def default_threshold_for_mode(config, mode):
    """返回代码当前生效的绿色或白色默认阈值副本。"""
    normalized = str(mode).upper()
    if normalized == "GREEN":
        return _first_threshold(
            getattr(config, "TROUGH_GREEN_LAB_THRESHOLDS", ()),
            (35, 95, -60, -10, 5, 70),
        )
    return _first_threshold(
        getattr(config, "TROUGH_WHITE_LAB_THRESHOLDS", ()),
        (77, 100, -18, 18, -18, 22),
    )


def adjust_threshold(values, parameter_index, delta):
    """单步调整阈值，并保持每一组min不大于max。"""
    output = [int(item) for item in values]
    index = max(0, min(len(PARAMETERS) - 1, int(parameter_index)))
    _, minimum, maximum, value_index = PARAMETERS[index]
    output[value_index] = max(
        minimum,
        min(maximum, output[value_index] + int(delta)),
    )

    pair_start = (value_index // 2) * 2
    pair_end = pair_start + 1
    if output[pair_start] > output[pair_end]:
        if value_index == pair_start:
            output[pair_start] = output[pair_end]
        else:
            output[pair_end] = output[pair_start]
    return output


def _map_touch_to_frame(raw_x, raw_y, frame, disp):
    """把屏幕触摸坐标映射到FIT_CONTAIN显示的相机画面坐标。"""
    try:
        mapped = image.resize_map_pos_reverse(
            frame.width(), frame.height(),
            disp.width(), disp.height(),
            image.Fit.FIT_CONTAIN,
            int(raw_x), int(raw_y),
        )
        return int(mapped[0]), int(mapped[1])
    except Exception:
        display_width = max(1, int(disp.width()))
        display_height = max(1, int(disp.height()))
        return (
            int(float(raw_x) * frame.width() / display_width),
            int(float(raw_y) * frame.height() / display_height),
        )


def _draw_text(frame, x, y, text, color):
    """直接叠字，只加1像素文字阴影，不绘制黑色底板。"""
    frame.draw_string(int(x) + 1, int(y) + 1, str(text), color=BLACK)
    frame.draw_string(int(x), int(y), str(text), color=color)


def _draw_outline_button(frame, rect, label, color=WHITE):
    """绘制透明按钮：只有边框和文字，不遮挡当前画面。"""
    x, y, width, height = rect
    frame.draw_rect(x, y, width, height, color=color, thickness=2)
    estimated_width = max(1, len(str(label))) * 8
    text_x = x + max(4, (width - estimated_width) // 2)
    text_y = y + max(3, (height - 16) // 2)
    _draw_text(frame, text_x, text_y, label, color)


def _mode_button_rects(width, height):
    margin = max(10, width // 20)
    gap = max(10, width // 24)
    button_width = (width - margin * 2 - gap) // 2
    button_height = max(54, min(76, height // 4))
    top = max(58, (height - button_height) // 2)
    return {
        "GREEN": (margin, top, button_width, button_height),
        "WHITE": (margin + button_width + gap,
                  top, button_width, button_height),
    }


def _tuning_layout(width, height):
    top = 36
    footer_height = max(42, min(52, height // 6))
    footer_y = height - footer_height - 4
    available = max(120, footer_y - top - 4)
    row_height = max(20, min(30, available // len(PARAMETERS)))
    rows = []
    for index in range(len(PARAMETERS)):
        rows.append((6, top + index * row_height,
                     width - 12, row_height - 2))

    margin = 6
    gap = 5
    button_width = (width - margin * 2 - gap * 3) // 4
    buttons = {}
    for index, name in enumerate(("BACK", "MINUS", "PLUS", "START")):
        buttons[name] = (
            margin + index * (button_width + gap),
            footer_y,
            button_width,
            footer_height,
        )
    return rows, buttons


def _make_binary_preview(frame, thresholds):
    preview = frame.copy()
    converted = preview.binary([tuple(int(item) for item in thresholds)])
    if converted is not None:
        preview = converted
    # 部分固件binary后仍是RGB，部分固件可能返回灰度；尽量转换，若已经是
    # 目标格式而固件抛异常，则保留原二值图继续绘制。
    try:
        converted = preview.to_format(image.Format.FMT_RGB888)
        if converted is not None:
            preview = converted
    except Exception:
        pass
    return preview


def _draw_mode_page(frame):
    width = frame.width()
    height = frame.height()
    rects = _mode_button_rects(width, height)
    _draw_text(frame, 8, 8, "SELECT TROUGH COLOR", YELLOW)
    _draw_text(frame, 8, 28, "Choose before threshold tuning", CYAN)
    _draw_outline_button(frame, rects["GREEN"], "GREEN", GREEN)
    _draw_outline_button(frame, rects["WHITE"], "WHITE", WHITE)
    return rects


def _draw_tuning_page(frame, mode, thresholds, selected_index):
    width = frame.width()
    height = frame.height()
    rows, buttons = _tuning_layout(width, height)
    title_color = GREEN if mode == "GREEN" else WHITE
    _draw_text(frame, 6, 4, "%s LAB  step:%d" % (mode, STEP), title_color)
    _draw_text(frame, 6, 21, "Tap row, then -3 / +3", MAGENTA)

    for index, (name, _, _, value_index) in enumerate(PARAMETERS):
        selected = index == selected_index
        color = YELLOW if selected else CYAN
        prefix = ">" if selected else " "
        _draw_text(
            frame,
            rows[index][0] + 3,
            rows[index][1] + 2,
            "%s %-4s %4d" % (prefix, name, thresholds[value_index]),
            color,
        )
        if selected:
            frame.draw_rect(*rows[index], color=YELLOW, thickness=1)

    _draw_outline_button(frame, buttons["BACK"], "BACK", CYAN)
    _draw_outline_button(frame, buttons["MINUS"], "-3", YELLOW)
    _draw_outline_button(frame, buttons["PLUS"], "+3", YELLOW)
    _draw_outline_button(frame, buttons["START"], "NEXT", GREEN)
    return rows, buttons


def _apply_selection(config, mode, thresholds):
    """仅覆盖当前Python进程内的生效阈值，不写磁盘。"""
    selected = tuple(int(item) for item in thresholds)
    config.TROUGH_COLOR_MODE = str(mode)
    config.TROUGH_LAB_THRESHOLDS = [selected]
    print(
        "THRESHOLD_MENU_START|mode=%s|lab=%d,%d,%d,%d,%d,%d" % (
            mode,
            selected[0], selected[1], selected[2],
            selected[3], selected[4], selected[5],
        )
    )
    return {
        "mode": str(mode),
        "threshold": selected,
    }


def run_threshold_tuning_menu(cam, disp, config):
    """运行两级触摸菜单；START返回选择，物理退出返回None。"""
    if cam is None or disp is None:
        mode = str(getattr(config, "TROUGH_COLOR_MODE", "GREEN")).upper()
        threshold = default_threshold_for_mode(config, mode)
        print("THRESHOLD_MENU_SKIPPED|reason=CAMERA_OR_DISPLAY_UNAVAILABLE")
        return _apply_selection(config, mode, threshold)

    try:
        touch = touchscreen.TouchScreen()
    except Exception as error:
        mode = str(getattr(config, "TROUGH_COLOR_MODE", "GREEN")).upper()
        threshold = default_threshold_for_mode(config, mode)
        print("THRESHOLD_MENU_SKIPPED|reason=TOUCH_UNAVAILABLE|error=%s" % error)
        return _apply_selection(config, mode, threshold)

    page = "MODE"
    mode = None
    thresholds = None
    selected_index = 0
    was_pressed = False
    frame = None
    preview = None
    try:
        while not app.need_exit():
            frame = cam.read()
            if frame is None:
                continue

            pressed = False
            touch_x = -1
            touch_y = -1
            try:
                raw_x, raw_y, raw_pressed = touch.read()
                pressed = bool(raw_pressed)
                if pressed:
                    touch_x, touch_y = _map_touch_to_frame(
                        raw_x, raw_y, frame, disp)
            except Exception as error:
                print("THRESHOLD_MENU_TOUCH_ERROR|%s" % error)

            # 一次按下只触发一个动作，必须松手后才能再次调整。
            clicked = pressed and not was_pressed
            was_pressed = pressed

            if page == "MODE":
                rects = _mode_button_rects(frame.width(), frame.height())
                if clicked:
                    for candidate in ("GREEN", "WHITE"):
                        if _point_in_rect(
                                touch_x, touch_y, rects[candidate]):
                            mode = candidate
                            thresholds = default_threshold_for_mode(
                                config, mode)
                            selected_index = 0
                            page = "TUNE"
                            break
                preview = frame
                _draw_mode_page(preview)
            else:
                rows, buttons = _tuning_layout(
                    frame.width(), frame.height())
                if clicked:
                    row_selected = False
                    for index, rect in enumerate(rows):
                        if _point_in_rect(touch_x, touch_y, rect):
                            selected_index = index
                            row_selected = True
                            break
                    if not row_selected:
                        if _point_in_rect(
                                touch_x, touch_y, buttons["BACK"]):
                            page = "MODE"
                            mode = None
                            thresholds = None
                        elif _point_in_rect(
                                touch_x, touch_y, buttons["MINUS"]):
                            thresholds = adjust_threshold(
                                thresholds, selected_index, -STEP)
                        elif _point_in_rect(
                                touch_x, touch_y, buttons["PLUS"]):
                            thresholds = adjust_threshold(
                                thresholds, selected_index, STEP)
                        elif _point_in_rect(
                                touch_x, touch_y, buttons["START"]):
                            return _apply_selection(
                                config, mode, thresholds)

                if page == "TUNE":
                    preview = _make_binary_preview(frame, thresholds)
                    _draw_tuning_page(
                        preview, mode, thresholds, selected_index)
                else:
                    preview = frame
                    _draw_mode_page(preview)

            disp.show(preview)
            preview = None
            frame = None
        return None
    finally:
        preview = None
        frame = None
        touch = None
        gc.collect()
