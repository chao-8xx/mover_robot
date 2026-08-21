"""物料识别触摸 UI：页面绘制、按钮、拖拽和坐标映射。"""

import copy

import cv2
import numpy as np
from maix import image

from vision_blob import threshold_swatch_bgr
from vision_config import (
    COLOR_LABELS,
    COLOR_NUMBERS,
    COLOR_ORDER,
    default_threshold,
    validate_config,
)


PAGE_HOME = "HOME"
PAGE_RUN = "RUN"
PAGE_CALIBRATE = "CALIBRATE"
PAGE_ROI = "ROI"
PAGE_CENTER = "CENTER"
PAGE_COLOR_MODE = "COLOR_MODE"
PAGE_COLOR_LIST = "COLOR_LIST"
PAGE_THRESHOLD = "THRESHOLD"

VIEW_RAW = "RAW"
VIEW_MASK = "MASK"
VIEW_OVERLAY = "OVERLAY"

BG = (24, 27, 35)
PANEL = (38, 43, 55)
CARD = (50, 57, 72)
CARD_ACTIVE = (54, 105, 82)
BORDER = (91, 103, 124)
WHITE = (245, 247, 250)
MUTED = (167, 177, 194)
ACCENT = (45, 210, 145)
YELLOW = (0, 210, 255)
RED = (60, 75, 235)
BLACK = (0, 0, 0)


def _clamp(value, low, high):
    return max(low, min(high, int(value)))


class TouchInput:
    """兼容没有 clear() 的旧版触摸固件；按钮只在松开时触发。"""

    def __init__(self, touch_screen):
        self.touch_screen = touch_screen
        self.has_available = hasattr(touch_screen, "available")

    def poll(self):
        try:
            if self.has_available and not self.touch_screen.available(0):
                return None
            data = self.touch_screen.read()
        except Exception as exc:
            print("[VISION UI] touch read failed:", exc)
            return None
        if data is None or len(data) < 3:
            return None
        return int(data[0]), int(data[1]), bool(data[2])


class VisionUI:
    def __init__(self, screen_w, screen_h, config, config_status):
        self.screen_w = max(320, int(screen_w))
        self.screen_h = max(240, int(screen_h))
        self.portrait = self.screen_h >= self.screen_w
        self.saved_config = validate_config(config)
        self.draft_config = None
        self.config_status = config_status

        self.page = PAGE_HOME
        self.page_stack = []
        self.buttons = []
        self.text_ops = []
        self.preview_rect = None
        self.mode_popup = False
        self.color_popup = False
        self.message = "视觉系统已就绪"
        self.view_mode = VIEW_OVERLAY

        # 中文字体和圆角按钮绘制开销较大。静态菜单只在页面/参数变化时重建，
        # 正常视频帧只覆盖预览区域和英文实时数据，避免每帧重复画整页中文。
        self.ui_revision = 0
        self.static_cache = None
        self.static_cache_revision = -1
        self.static_cache_page = None
        self.static_cache_buttons = []
        self.static_cache_preview_rect = None
        self.building_static = False

        self.roi_selection = None
        self.roi_drag = None
        self.center_selected = False
        self.color_edit_mode = self.saved_config["mode"]
        self.color_edit_key = self.saved_config["target_color"]
        self.threshold_param = 0
        self.threshold_step_index = 0
        self.pixel_step = 5

        self.touch_down = False
        self.touch_start = (0, 0)
        self.touch_last = (0, 0)
        self.pressed_button = None
        self.touch_moved = False

        self.font_ready = False
        self.use_named_font = True
        self.text_error_reported = False
        self.fonts = {"small": None, "normal": None, "title": None}
        self._load_fonts()

    def _load_fonts(self):
        font_path = "/maixapp/share/font/SourceHanSansCN-Regular.otf"
        try:
            image.load_font("vision_cn_18", font_path, size=18)
            image.load_font("vision_cn_22", font_path, size=22)
            image.load_font("vision_cn_28", font_path, size=28)
            self.fonts = {
                "small": "vision_cn_18",
                "normal": "vision_cn_22",
                "title": "vision_cn_28",
            }
            if hasattr(image, "set_default_font"):
                image.set_default_font("vision_cn_22")
            self.font_ready = True
        except Exception as exc:
            print("[VISION UI] Chinese font load failed:", exc)

    def set_saved_config(self, config, status="保存成功"):
        self.saved_config = validate_config(config)
        self.config_status = status
        self.message = status
        self._invalidate()

    def save_succeeded(self, config, status="保存成功"):
        self.set_saved_config(config, status)
        self.draft_config = None
        self.roi_selection = None
        self.center_selected = False
        self._back()

    def save_failed(self, status="保存失败"):
        self.message = status
        self._invalidate()

    def detection_config(self):
        if self.draft_config is not None and self.page in (
            PAGE_ROI, PAGE_CENTER, PAGE_THRESHOLD
        ):
            return self.draft_config
        return self.saved_config

    def current_view_mode(self):
        return self.view_mode if self.page == PAGE_THRESHOLD else VIEW_OVERLAY

    def _invalidate(self):
        """页面、选中状态或参数变化后，让下一帧重新生成静态菜单。"""
        self.ui_revision += 1
        self.static_cache = None

    def _open(self, page):
        self.page_stack.append(self.page)
        self.page = page
        self.mode_popup = False
        self.color_popup = False
        self._invalidate()

    def _back(self):
        self.page = self.page_stack.pop() if self.page_stack else PAGE_HOME
        self.mode_popup = False
        self.color_popup = False
        self._invalidate()

    def _begin_editor(self, page):
        self.draft_config = copy.deepcopy(self.saved_config)
        self.roi_selection = None
        self.center_selected = False
        self._open(page)

    def _begin_threshold(self, color_key):
        self.draft_config = copy.deepcopy(self.saved_config)
        self.color_edit_key = color_key
        self.threshold_param = 0
        self.threshold_step_index = 0
        self.view_mode = VIEW_OVERLAY
        self._open(PAGE_THRESHOLD)

    # ------------------------------ 绘制基础件 ------------------------------
    def _round_rect(self, canvas, rect, color, radius=10, thickness=-1):
        x, y, w, h = [int(v) for v in rect]
        radius = max(1, min(radius, w // 2, h // 2))
        if thickness != -1:
            cv2.rectangle(canvas, (x + radius, y), (x + w - radius, y + h), color, thickness)
            cv2.rectangle(canvas, (x, y + radius), (x + w, y + h - radius), color, thickness)
            cv2.circle(canvas, (x + radius, y + radius), radius, color, thickness)
            cv2.circle(canvas, (x + w - radius, y + radius), radius, color, thickness)
            cv2.circle(canvas, (x + radius, y + h - radius), radius, color, thickness)
            cv2.circle(canvas, (x + w - radius, y + h - radius), radius, color, thickness)
            return
        cv2.rectangle(canvas, (x + radius, y), (x + w - radius, y + h), color, -1)
        cv2.rectangle(canvas, (x, y + radius), (x + w, y + h - radius), color, -1)
        cv2.circle(canvas, (x + radius, y + radius), radius, color, -1)
        cv2.circle(canvas, (x + w - radius, y + radius), radius, color, -1)
        cv2.circle(canvas, (x + radius, y + h - radius), radius, color, -1)
        cv2.circle(canvas, (x + w - radius, y + h - radius), radius, color, -1)

    def _measure(self, text, font_kind):
        font_name = self.fonts.get(font_kind)
        try:
            if font_name:
                return image.string_size(str(text), font=font_name)
            return image.string_size(str(text))
        except Exception:
            size = {"small": 18, "normal": 22, "title": 28}.get(font_kind, 22)
            return max(1, len(str(text)) * size), size

    def _text(self, text, x, y, color=WHITE, font="normal", center_rect=None):
        if center_rect is not None:
            tx, ty, tw, th = center_rect
            text_w, text_h = self._measure(text, font)
            x = tx + (tw - text_w) // 2
            y = ty + (th - text_h) // 2
        self.text_ops.append((int(x), int(y), str(text), color, font))

    def _button(self, canvas, button_id, rect, label, active=False, enabled=True,
                fill=None, text_color=WHITE, font="normal"):
        color = fill if fill is not None else (CARD_ACTIVE if active else CARD)
        if not enabled:
            color = PANEL
            text_color = MUTED
        self._round_rect(canvas, rect, color, 9)
        self._round_rect(canvas, rect, ACCENT if active else BORDER, 9, 1)
        if enabled:
            self.buttons.append((button_id,) + tuple(int(v) for v in rect))
        self._text(label, 0, 0, text_color, font, rect)

    def _chip(self, canvas, rect, label, active=False):
        self._round_rect(canvas, rect, CARD_ACTIVE if active else PANEL, 8)
        self._round_rect(canvas, rect, ACCENT if active else BORDER, 8, 1)
        self._text(label, 0, 0, WHITE if active else MUTED, "small", rect)

    def _header(self, canvas, title, back=False, exit_run=False):
        cv2.rectangle(canvas, (0, 0), (self.screen_w, 48), PANEL, -1)
        if back:
            self._button(canvas, "BACK", (7, 7, 68, 34), "返回", font="small")
        if exit_run:
            self._button(
                canvas, "EXIT_RUN", (self.screen_w - 103, 7, 96, 34),
                "退出运行", font="small"
            )
        self._text(title, 0, 0, WHITE, "title", (80, 4, self.screen_w - 160, 40))

    def _fit_rect(self, area):
        ax, ay, aw, ah = area
        target_w = min(aw, int(ah * 4 / 3))
        target_h = int(target_w * 3 / 4)
        if target_h > ah:
            target_h = ah
            target_w = int(target_h * 4 / 3)
        return (
            int(ax + (aw - target_w) / 2),
            int(ay + (ah - target_h) / 2),
            max(1, int(target_w)),
            max(1, int(target_h)),
        )

    def _preview(self, canvas, preview_bgr, area):
        rect = self._fit_rect(area)
        x, y, w, h = rect
        resized = cv2.resize(preview_bgr, (w, h), interpolation=cv2.INTER_LINEAR)
        canvas[y:y + h, x:x + w] = resized
        cv2.rectangle(canvas, (x, y), (x + w - 1, y + h - 1), BORDER, 1)
        self.preview_rect = rect
        return rect

    def _img_to_screen(self, img_x, img_y):
        if self.preview_rect is None:
            return 0, 0
        x, y, w, h = self.preview_rect
        return x + int(img_x * w / 320), y + int(img_y * h / 240)

    def _screen_to_img(self, screen_x, screen_y):
        if self.preview_rect is None:
            return None
        x, y, w, h = self.preview_rect
        if not (x <= screen_x < x + w and y <= screen_y < y + h):
            return None
        return (
            _clamp((screen_x - x) * 320 // w, 0, 319),
            _clamp((screen_y - y) * 240 // h, 0, 239),
        )

    # ------------------------------ 各页面 ------------------------------
    def _draw_home(self, canvas, preview_bgr, result):
        self._header(canvas, "物料视觉")
        cv2.circle(canvas, (18, 25), 6, ACCENT, -1)
        self._text("LIVE", 29, 14, ACCENT, "small")

        if self.portrait:
            rect = self._preview(canvas, preview_bgr, (8, 55, self.screen_w - 16, 348))
            chip_y = rect[1] + rect[3] + 10
            chip_gap = 6
            chip_w = (self.screen_w - 20 - chip_gap * 2) // 3
            mode_rect = (7, chip_y, chip_w, 42)
            self._button(
                canvas, "HOME_MODE", mode_rect,
                "模式 " + self.saved_config["mode"], active=True, font="small"
            )
            color_key = self.saved_config["target_color"]
            self._button(
                canvas, "HOME_COLOR",
                (7 + chip_w + chip_gap, chip_y, chip_w, 42),
                "颜色 " + COLOR_LABELS[color_key], active=True, font="small"
            )
            using_default = "默认" in self.config_status or "损坏" in self.config_status
            status_text = "参数 默认" if using_default else "参数 已加载"
            self._chip(
                canvas, (7 + (chip_w + chip_gap) * 2, chip_y, chip_w, 42), status_text
            )
            button_y = chip_y + 55
            self._button(canvas, "START_RUN", (12, button_y, self.screen_w - 24, 58),
                         "运行", active=True, font="title")
            self._button(canvas, "OPEN_CALIBRATE",
                         (12, button_y + 68, self.screen_w - 24, 58),
                         "调参", font="title")
        else:
            rect = self._preview(canvas, preview_bgr, (8, 55, int(self.screen_w * 0.64), self.screen_h - 63))
            panel_x = rect[0] + rect[2] + 10
            panel_w = self.screen_w - panel_x - 8
            self._button(canvas, "HOME_MODE", (panel_x, 65, panel_w, 44),
                         "模式 " + self.saved_config["mode"], active=True, font="small")
            self._button(canvas, "HOME_COLOR", (panel_x, 117, panel_w, 42),
                         "颜色 " + COLOR_LABELS[self.saved_config["target_color"]],
                         active=True, font="small")
            self._chip(canvas, (panel_x, 167, panel_w, 42), self.config_status)
            self._button(canvas, "START_RUN", (panel_x, 228, panel_w, 66), "运行", active=True)
            self._button(canvas, "OPEN_CALIBRATE", (panel_x, 306, panel_w, 66), "调参")
        if self.mode_popup:
            self._draw_mode_popup(canvas)
        elif self.color_popup:
            self._draw_color_popup(canvas)

    def _draw_mode_popup(self, canvas):
        # 弹窗打开时禁止底层的运行、调参和状态按钮响应。
        self.buttons = []
        self.text_ops = []
        overlay = canvas.copy()
        cv2.rectangle(overlay, (0, 48), (self.screen_w, self.screen_h), (8, 10, 14), -1)
        cv2.addWeighted(overlay, 0.64, canvas, 0.36, 0, canvas)
        card_w = min(380, self.screen_w - 40)
        card_h = 220
        x = (self.screen_w - card_w) // 2
        y = (self.screen_h - card_h) // 2
        self._round_rect(canvas, (x, y, card_w, card_h), PANEL, 14)
        self._round_rect(canvas, (x, y, card_w, card_h), ACCENT, 14, 2)
        self._text("选择当前识别模式", 0, 0, WHITE, "title", (x, y + 12, card_w, 42))
        self._button(canvas, "QUICK_HSV", (x + 24, y + 72, card_w - 48, 48), "HSV",
                     active=self.saved_config["mode"] == "HSV")
        self._button(canvas, "QUICK_LAB", (x + 24, y + 128, card_w - 48, 48), "LAB",
                     active=self.saved_config["mode"] == "LAB")
        self._button(canvas, "CLOSE_MODE", (x + card_w - 82, y + 8, 70, 32), "取消", font="small")
        # 弹窗覆盖预览区时暂停画面粘贴，否则实时预览会盖住弹窗。
        self.preview_rect = None

    def _draw_color_popup(self, canvas):
        """首页快速选择赛题六色；阈值仍在调参菜单内修改。"""
        self.buttons = []
        self.text_ops = []
        overlay = canvas.copy()
        cv2.rectangle(overlay, (0, 48), (self.screen_w, self.screen_h), (8, 10, 14), -1)
        cv2.addWeighted(overlay, 0.64, canvas, 0.36, 0, canvas)
        card_w = min(430, self.screen_w - 24)
        card_h = min(468, self.screen_h - 90)
        x = (self.screen_w - card_w) // 2
        y = (self.screen_h - card_h) // 2
        self._round_rect(canvas, (x, y, card_w, card_h), PANEL, 14)
        self._round_rect(canvas, (x, y, card_w, card_h), ACCENT, 14, 2)
        self._text("选择当前颜色", 0, 0, WHITE, "title", (x, y + 10, card_w, 42))
        self._button(canvas, "CLOSE_COLOR", (x + card_w - 82, y + 8, 70, 32),
                     "取消", font="small")

        gap = 10
        margin = 18
        cell_w = (card_w - margin * 2 - gap) // 2
        cell_h = (card_h - 92 - gap * 2) // 3
        for index, color_key in enumerate(COLOR_ORDER):
            col = index % 2
            row = index // 2
            rect = (
                x + margin + col * (cell_w + gap),
                y + 64 + row * (cell_h + gap),
                cell_w,
                cell_h,
            )
            bgr = threshold_swatch_bgr(
                self.saved_config, self.saved_config["mode"], color_key
            )
            label = "{}{}".format(COLOR_LABELS[color_key], COLOR_NUMBERS[color_key])
            self._button(
                canvas, "QUICK_COLOR_" + color_key, rect, label,
                active=self.saved_config["target_color"] == color_key,
                fill=bgr, text_color=self._swatch_text_color(bgr), font="normal"
            )
        self.preview_rect = None

    def _draw_run(self, canvas, preview_bgr, result):
        self._header(canvas, "运行中", exit_run=True)
        cv2.circle(canvas, (18, 25), 6, ACCENT, -1)
        area = (8, 55, self.screen_w - 16, self.screen_h - 105)
        self._preview(canvas, preview_bgr, area)
        config = self.saved_config
        self._text(
            "{} · {}{}".format(
                config["mode"], COLOR_LABELS[config["target_color"]],
                COLOR_NUMBERS[config["target_color"]]
            ), 12, self.screen_h - 32, MUTED, "small"
        )

    def _draw_calibrate(self, canvas, preview_bgr):
        self._header(canvas, "调参", back=True)
        if self.portrait:
            rect = self._preview(canvas, preview_bgr, (8, 55, self.screen_w - 16, 205))
            start_y = rect[1] + rect[3] + 18
            self._button(canvas, "EDIT_ROI", (12, start_y, self.screen_w - 24, 82),
                         "ROI 识别区域", active=True)
            self._button(canvas, "EDIT_CENTER", (12, start_y + 94, self.screen_w - 24, 82),
                         "取中心点")
            self._button(canvas, "EDIT_COLOR", (12, start_y + 188, self.screen_w - 24, 82),
                         "颜色阈值")
            self._text(self.message, 14, self.screen_h - 28, ACCENT, "small")
        else:
            rect = self._preview(canvas, preview_bgr, (8, 55, int(self.screen_w * 0.58), self.screen_h - 63))
            x = rect[0] + rect[2] + 12
            w = self.screen_w - x - 8
            self._button(canvas, "EDIT_ROI", (x, 68, w, 86), "ROI 识别区域", active=True)
            self._button(canvas, "EDIT_CENTER", (x, 166, w, 86), "取中心点")
            self._button(canvas, "EDIT_COLOR", (x, 264, w, 86), "颜色阈值")

    def _draw_roi_overlay(self, canvas):
        roi = self.draft_config["roi"]
        points = {
            "TL": self._img_to_screen(roi["x"], roi["y"]),
            "TR": self._img_to_screen(roi["x"] + roi["w"] - 1, roi["y"]),
            "BL": self._img_to_screen(roi["x"], roi["y"] + roi["h"] - 1),
            "BR": self._img_to_screen(roi["x"] + roi["w"] - 1, roi["y"] + roi["h"] - 1),
        }
        for name, point in points.items():
            active = self.roi_selection == name
            cv2.circle(canvas, point, 10, ACCENT if active else YELLOW, -1)
            cv2.circle(canvas, point, 12, WHITE, 2)

    def _draw_editor_controls(self, canvas, selected_text, kind):
        if self.portrait:
            y = self.preview_rect[1] + self.preview_rect[3] + 8
            self._text(selected_text, 12, y, ACCENT if "未" not in selected_text else MUTED, "small")
            if (kind == "ROI" and self.roi_selection) or (kind == "CENTER" and self.center_selected):
                for index, name in enumerate(("左", "上", "下", "右")):
                    self._button(canvas, "MOVE_" + name, (8 + index * 56, y + 28, 50, 44), name, font="small")
            self._text("步长", 238, y + 39, MUTED, "small")
            for index, step in enumerate((1, 5, 10)):
                self._button(canvas, "PIXEL_STEP_" + str(step),
                             (288 + index * 61, y + 28, 55, 44), str(step),
                             active=self.pixel_step == step, font="small")
            action_y = self.screen_h - 58
            labels = (
                (("SAVE_EDITOR", "保存"), ("RESTORE_EDITOR", "恢复上次"),
                 ("RESET_ROI", "重置全图"))
                if kind == "ROI" else
                (("SAVE_EDITOR", "保存"), ("RESTORE_EDITOR", "恢复上次"),
                 ("CLEAR_CENTER", "清除中心"))
            )
            gap = 6
            button_w = (self.screen_w - 24 - gap * 2) // 3
            for index, item in enumerate(labels):
                self._button(canvas, item[0], (8 + index * (button_w + gap), action_y, button_w, 48),
                             item[1], active=index == 0, font="small")
        else:
            x = self.preview_rect[0] + self.preview_rect[2] + 10
            w = self.screen_w - x - 8
            self._text(selected_text, x, 62, ACCENT, "small")
            for index, name in enumerate(("左", "上", "下", "右")):
                self._button(canvas, "MOVE_" + name,
                             (x + (index % 2) * (w // 2), 96 + (index // 2) * 48,
                              w // 2 - 4, 42), name, font="small")
            self._text("步长", x, 198, MUTED, "small")
            for index, step in enumerate((1, 5, 10)):
                self._button(canvas, "PIXEL_STEP_" + str(step),
                             (x + index * (w // 3), 224, w // 3 - 4, 42), str(step),
                             active=self.pixel_step == step, font="small")
            reset_id = "RESET_ROI" if kind == "ROI" else "CLEAR_CENTER"
            reset_label = "重置全图" if kind == "ROI" else "清除中心"
            self._button(canvas, "SAVE_EDITOR", (x, 286, w, 45), "保存", active=True, font="small")
            self._button(canvas, "RESTORE_EDITOR", (x, 337, w, 45), "恢复上次", font="small")
            self._button(canvas, reset_id, (x, 388, w, 45), reset_label, font="small")

    def _draw_roi(self, canvas, preview_bgr):
        self._header(canvas, "ROI 识别区域", back=True)
        area = ((8, 55, self.screen_w - 16, 348) if self.portrait else
                (8, 55, int(self.screen_w * 0.64), self.screen_h - 63))
        self._preview(canvas, preview_bgr, area)
        if not self.building_static:
            self._draw_roi_overlay(canvas)
        selection_names = {None: "未选择：拖动四角或 ROI 内部", "BODY": "已选择：整个 ROI",
                           "TL": "已选择：左上角", "TR": "已选择：右上角",
                           "BL": "已选择：左下角", "BR": "已选择：右下角"}
        self._draw_editor_controls(canvas, selection_names[self.roi_selection], "ROI")

    def _draw_center_marker(self, canvas):
        roi = self.draft_config["roi"]
        center = self.draft_config["center"]
        sx, sy = self._img_to_screen(roi["x"] + center["x"], roi["y"] + center["y"])
        color = ACCENT if self.center_selected else WHITE
        cv2.drawMarker(canvas, (sx, sy), color, cv2.MARKER_CROSS, 24, 3)
        cv2.circle(canvas, (sx, sy), 13, color, 2)

    def _draw_center(self, canvas, preview_bgr):
        self._header(canvas, "取中心点", back=True)
        area = ((8, 55, self.screen_w - 16, 348) if self.portrait else
                (8, 55, int(self.screen_w * 0.64), self.screen_h - 63))
        self._preview(canvas, preview_bgr, area)
        if not self.building_static:
            self._draw_center_marker(canvas)
        center = self.draft_config["center"]
        if not center["custom"]:
            text = "未设置：请在 ROI 内点击取点"
        elif self.center_selected:
            text = "已选择中心点，可点击方向微调"
        else:
            text = "已显示上次中心点，点击中心点后微调"
        self._draw_editor_controls(canvas, text, "CENTER")

    def _draw_color_mode(self, canvas):
        self._header(canvas, "颜色空间", back=True)
        self._text("选择要调节的阈值类型", 0, 0, MUTED, "normal",
                   (20, 82, self.screen_w - 40, 40))
        card_w = self.screen_w - 40
        self._button(canvas, "COLOR_SPACE_HSV", (20, 145, card_w, 150), "HSV",
                     active=self.saved_config["mode"] == "HSV", font="title")
        self._button(canvas, "COLOR_SPACE_LAB", (20, 315, card_w, 150), "LAB",
                     active=self.saved_config["mode"] == "LAB", font="title")
        self._text("进入颜色并保存后，该颜色和模式会成为当前运行参数", 20,
                   self.screen_h - 66, MUTED, "small")

    def _swatch_text_color(self, bgr):
        luminance = 0.114 * bgr[0] + 0.587 * bgr[1] + 0.299 * bgr[2]
        return BLACK if luminance > 155 else WHITE

    def _draw_color_list(self, canvas):
        self._header(canvas, self.color_edit_mode + " 六种赛题颜色", back=True)
        gap = 10
        margin = 12
        card_w = (self.screen_w - margin * 2 - gap) // 2
        card_h = 145 if self.portrait else 106
        start_y = 68
        for index, color_key in enumerate(COLOR_ORDER):
            col = index % 2
            row = index // 2
            rect = (margin + col * (card_w + gap), start_y + row * (card_h + gap), card_w, card_h)
            bgr = threshold_swatch_bgr(self.saved_config, self.color_edit_mode, color_key)
            label = "{}{}".format(COLOR_LABELS[color_key], COLOR_NUMBERS[color_key])
            active = (
                self.saved_config["mode"] == self.color_edit_mode and
                self.saved_config["target_color"] == color_key
            )
            self._button(canvas, "EDIT_COLOR_" + color_key, rect, label, active=active,
                         fill=bgr, text_color=self._swatch_text_color(bgr), font="title")
        self._text("按钮颜色由当前阈值的中心值生成", 14, self.screen_h - 34, MUTED, "small")
        if self.message in ("保存成功", "保存失败"):
            self._text(self.message, self.screen_w - 102, self.screen_h - 34,
                       ACCENT if self.message == "保存成功" else RED, "small")

    def _threshold_values(self):
        threshold = self.draft_config["thresholds"][self.color_edit_mode][self.color_edit_key]
        return [
            threshold["lower"][0], threshold["upper"][0],
            threshold["lower"][1], threshold["upper"][1],
            threshold["lower"][2], threshold["upper"][2],
        ]

    def _threshold_labels(self):
        if self.color_edit_mode == "HSV":
            if self.color_edit_key == "red":
                # 红色跨过 HSV 色相首尾：高端接近 179，低端接近 0。
                return ("H高端", "H低端", "S最小", "S最大", "V最小", "V最大")
            return ("H起点", "H终点", "S最小", "S最大", "V最小", "V最大")
        return ("L最小", "L最大", "A最小", "A最大", "B最小", "B最大")

    def _threshold_steps(self):
        if self.color_edit_mode == "HSV" and self.threshold_param < 2:
            return (1, 2, 5)
        return (1, 5, 10)

    def _draw_threshold(self, canvas, preview_bgr):
        title = "{} · {}".format(self.color_edit_mode, COLOR_LABELS[self.color_edit_key])
        self._header(canvas, title, back=True)
        if self.portrait:
            self._preview(canvas, preview_bgr, (8, 55, int(self.screen_w * 0.64), 225))
            tab_x = self.preview_rect[0] + self.preview_rect[2] + 8
            tab_w = self.screen_w - tab_x - 8
            for index, item in enumerate(((VIEW_RAW, "原图"), (VIEW_MASK, "掩膜"),
                                          (VIEW_OVERLAY, "叠加"))):
                self._button(canvas, "VIEW_" + item[0], (tab_x, 62 + index * 54, tab_w, 46),
                             item[1], active=self.view_mode == item[0], font="small")
            if (self.color_edit_mode == "HSV" and self.color_edit_key == "red" and
                    self.draft_config["thresholds"]["HSV"]["red"]["lower"][0] >
                    self.draft_config["thresholds"]["HSV"]["red"]["upper"][0]):
                red_value = self.draft_config["thresholds"]["HSV"]["red"]
                self._chip(canvas, (tab_x, 222, tab_w, 29),
                           "高端 {}~179".format(red_value["lower"][0]))
                self._chip(canvas, (tab_x, 255, tab_w, 29),
                           "低端 0~{}".format(red_value["upper"][0]))
            grid_y = 290
        else:
            self._preview(canvas, preview_bgr, (8, 55, int(self.screen_w * 0.46), 225))
            grid_y = 60

        labels = self._threshold_labels()
        values = self._threshold_values()
        if self.portrait:
            gap = 8
            margin = 8
            cell_w = (self.screen_w - margin * 2 - gap) // 2
            for index in range(6):
                rect = (margin + (index % 2) * (cell_w + gap),
                        grid_y + (index // 2) * 54, cell_w, 47)
                self._button(canvas, "THRESH_PARAM_" + str(index), rect,
                             "{}  {}".format(labels[index], values[index]),
                             active=index == self.threshold_param, font="small")
            step_y = grid_y + 168
            self._text("步长", 10, step_y + 10, MUTED, "small")
            steps = self._threshold_steps()
            for index, step in enumerate(steps):
                self._button(canvas, "THRESH_STEP_" + str(index),
                             (74 + index * 66, step_y, 60, 42), str(step),
                             active=index == self.threshold_step_index, font="small")
            self._button(canvas, "THRESH_MINUS", (286, step_y, 82, 42), "－", font="title")
            self._button(canvas, "THRESH_PLUS", (378, step_y, 82, 42), "＋", active=True, font="title")
            action_y = self.screen_h - 58
            action_w = (self.screen_w - 28) // 3
            for index, item in enumerate((("SAVE_THRESHOLD", "保存"),
                                          ("RESTORE_THRESHOLD", "恢复上次"),
                                          ("DEFAULT_THRESHOLD", "恢复默认"))):
                self._button(canvas, item[0], (8 + index * (action_w + 6), action_y,
                                               action_w, 48), item[1], active=index == 0,
                             font="small")
        else:
            panel_x = self.preview_rect[0] + self.preview_rect[2] + 10
            panel_w = self.screen_w - panel_x - 8
            for index in range(6):
                rect = (panel_x + (index % 2) * (panel_w // 2),
                        grid_y + (index // 2) * 52, panel_w // 2 - 4, 46)
                self._button(canvas, "THRESH_PARAM_" + str(index), rect,
                             "{} {}".format(labels[index], values[index]),
                             active=index == self.threshold_param, font="small")
            self._button(canvas, "THRESH_MINUS", (panel_x, 232, panel_w // 2 - 4, 44), "－")
            self._button(canvas, "THRESH_PLUS", (panel_x + panel_w // 2, 232,
                                                  panel_w // 2 - 4, 44), "＋", active=True)
            self._button(canvas, "SAVE_THRESHOLD", (panel_x, 292, panel_w, 42), "保存", active=True)
            self._button(canvas, "RESTORE_THRESHOLD", (panel_x, 340, panel_w, 42), "恢复上次")
            self._button(canvas, "DEFAULT_THRESHOLD", (panel_x, 388, panel_w, 42), "恢复默认")

    def _draw_page(self, canvas, preview_bgr, result):
        """生成页面结构；实时循环通过缓存避免重复执行这里的中文绘制。"""
        if self.page == PAGE_HOME:
            self._draw_home(canvas, preview_bgr, result)
        elif self.page == PAGE_RUN:
            self._draw_run(canvas, preview_bgr, result)
        elif self.page == PAGE_CALIBRATE:
            self._draw_calibrate(canvas, preview_bgr)
        elif self.page == PAGE_ROI:
            self._draw_roi(canvas, preview_bgr)
        elif self.page == PAGE_CENTER:
            self._draw_center(canvas, preview_bgr)
        elif self.page == PAGE_COLOR_MODE:
            self._draw_color_mode(canvas)
        elif self.page == PAGE_COLOR_LIST:
            self._draw_color_list(canvas)
        elif self.page == PAGE_THRESHOLD:
            self._draw_threshold(canvas, preview_bgr)

    def _draw_text_ops(self, output):
        """把中文文字一次性烘焙进静态页面缓存。"""
        for x, y, text, color, font_kind in self.text_ops:
            maix_color = image.Color.from_rgb(color[2], color[1], color[0])
            font_name = self.fonts.get(font_kind)
            try:
                if font_name and self.use_named_font:
                    output.draw_string(x, y, text, maix_color, font=font_name)
                else:
                    output.draw_string(x, y, text, maix_color)
            except TypeError:
                # 旧固件可能支持自定义默认字体，但 draw_string 没有 font 关键字。
                self.use_named_font = False
                try:
                    output.draw_string(x, y, text, maix_color)
                except Exception as exc:
                    if not self.text_error_reported:
                        print("[VISION UI] draw text failed:", exc)
                        self.text_error_reported = True
            except Exception as exc:
                if not self.text_error_reported:
                    print("[VISION UI] draw text failed:", exc)
                    self.text_error_reported = True

    def _build_static_cache(self, result):
        """只在换页、点按钮或参数变化后重新生成 480x640 中文菜单。"""
        self.buttons = []
        self.text_ops = []
        self.preview_rect = None
        canvas = np.zeros((self.screen_h, self.screen_w, 3), dtype=np.uint8)
        canvas[:, :] = BG
        placeholder = np.zeros((240, 320, 3), dtype=np.uint8)
        placeholder[:, :] = (28, 31, 40)
        self.building_static = True
        self._draw_page(canvas, placeholder, result)
        self.building_static = False

        static_image = image.cv2image(canvas)
        self._draw_text_ops(static_image)
        # 主循环本来就依赖 image2cv；这里只在缓存重建时多转换一次。
        self.static_cache = image.image2cv(static_image, ensure_bgr=True, copy=True)
        self.static_cache_revision = self.ui_revision
        self.static_cache_page = self.page
        self.static_cache_buttons = list(self.buttons)
        self.static_cache_preview_rect = self.preview_rect

    def _paste_live_preview(self, canvas, preview_bgr):
        if self.preview_rect is None:
            return
        x, y, w, h = self.preview_rect
        resized = cv2.resize(preview_bgr, (w, h), interpolation=cv2.INTER_LINEAR)
        canvas[y:y + h, x:x + w] = resized
        cv2.rectangle(canvas, (x, y), (x + w - 1, y + h - 1), BORDER, 1)

    def _draw_realtime_info(self, canvas, result, fps):
        """实时文字用 OpenCV 小字体绘制，避免每帧调用中文字体渲染。"""
        if self.preview_rect is None:
            return
        x, y, w, h = self.preview_rect
        cv2.rectangle(canvas, (x + 4, y + 4), (x + 104, y + 25), (16, 19, 24), -1)
        cv2.putText(canvas, "FPS:{:.1f}".format(fps), (x + 9, y + 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.43, (80, 255, 170), 1)
        if result["found"]:
            info = "AREA:{}  DX:{}  DY:{}".format(
                result["area"], result["dx"], result["dy"]
            )
        else:
            info = "NO TARGET"
        text_y = y + h - 8
        cv2.rectangle(canvas, (x + 4, text_y - 17),
                      (min(x + w - 4, x + 230), text_y + 3), (16, 19, 24), -1)
        cv2.putText(canvas, info, (x + 9, text_y), cv2.FONT_HERSHEY_SIMPLEX,
                    0.4, (230, 235, 240), 1)

    def render(self, preview_bgr, result, fps=0.0):
        cache_invalid = (
            self.static_cache is None or
            self.static_cache_revision != self.ui_revision or
            self.static_cache_page != self.page
        )
        if cache_invalid:
            self._build_static_cache(result)

        self.buttons = list(self.static_cache_buttons)
        self.preview_rect = self.static_cache_preview_rect
        canvas = self.static_cache.copy()
        self._paste_live_preview(canvas, preview_bgr)
        self._draw_realtime_info(canvas, result, fps)

        # ROI 四角和中心点属于动态标记，覆盖实时画面后再补画。
        if self.page == PAGE_ROI:
            self._draw_roi_overlay(canvas)
        elif self.page == PAGE_CENTER:
            self._draw_center_marker(canvas)
        return image.cv2image(canvas)

    # ------------------------------ 触摸处理 ------------------------------
    def _button_at(self, x, y):
        for button in reversed(self.buttons):
            button_id, bx, by, bw, bh = button
            if bx <= x <= bx + bw and by <= y <= by + bh:
                return button_id
        return None

    def _roi_handle_at(self, x, y):
        if self.preview_rect is None:
            return None
        roi = self.draft_config["roi"]
        points = {
            "TL": self._img_to_screen(roi["x"], roi["y"]),
            "TR": self._img_to_screen(roi["x"] + roi["w"] - 1, roi["y"]),
            "BL": self._img_to_screen(roi["x"], roi["y"] + roi["h"] - 1),
            "BR": self._img_to_screen(roi["x"] + roi["w"] - 1, roi["y"] + roi["h"] - 1),
        }
        for name, point in points.items():
            if (x - point[0]) ** 2 + (y - point[1]) ** 2 <= 22 ** 2:
                return name
        img_pos = self._screen_to_img(x, y)
        if img_pos:
            ix, iy = img_pos
            if (roi["x"] <= ix < roi["x"] + roi["w"] and
                    roi["y"] <= iy < roi["y"] + roi["h"]):
                return "BODY"
        return None

    def _begin_roi_drag(self, x, y):
        selection = self._roi_handle_at(x, y)
        if selection is None:
            return False
        image_pos = self._screen_to_img(x, y)
        if image_pos is None:
            return False
        self.roi_selection = selection
        self.roi_drag = {
            "kind": selection,
            "start": image_pos,
            "roi": copy.deepcopy(self.draft_config["roi"]),
        }
        self._invalidate()
        return True

    def _set_roi_edges(self, left, top, right, bottom):
        left = _clamp(left, 0, 300)
        top = _clamp(top, 0, 220)
        right = _clamp(right, left + 20, 320)
        bottom = _clamp(bottom, top + 20, 240)
        if right - left < 20 or bottom - top < 20:
            return
        self.draft_config["roi"] = {
            "x": left, "y": top, "w": right - left, "h": bottom - top
        }
        self.draft_config = validate_config(self.draft_config)

    def _update_roi_drag(self, x, y):
        if self.roi_drag is None:
            return
        image_pos = self._screen_to_img(x, y)
        if image_pos is None:
            return
        start_x, start_y = self.roi_drag["start"]
        dx = image_pos[0] - start_x
        dy = image_pos[1] - start_y
        roi = self.roi_drag["roi"]
        left, top = roi["x"], roi["y"]
        right, bottom = left + roi["w"], top + roi["h"]
        kind = self.roi_drag["kind"]
        if kind == "BODY":
            new_left = _clamp(left + dx, 0, 320 - roi["w"])
            new_top = _clamp(top + dy, 0, 240 - roi["h"])
            self.draft_config["roi"]["x"] = new_left
            self.draft_config["roi"]["y"] = new_top
        else:
            if "L" in kind:
                left = _clamp(left + dx, 0, right - 20)
            if "R" in kind:
                right = _clamp(right + dx, left + 20, 320)
            if "T" in kind:
                top = _clamp(top + dy, 0, bottom - 20)
            if "B" in kind:
                bottom = _clamp(bottom + dy, top + 20, 240)
            self._set_roi_edges(left, top, right, bottom)
        self.draft_config = validate_config(self.draft_config)
        self._invalidate()

    def _move_roi_selection(self, direction):
        if not self.roi_selection:
            return
        roi = self.draft_config["roi"]
        left, top = roi["x"], roi["y"]
        right, bottom = left + roi["w"], top + roi["h"]
        dx = -self.pixel_step if direction == "左" else self.pixel_step if direction == "右" else 0
        dy = -self.pixel_step if direction == "上" else self.pixel_step if direction == "下" else 0
        kind = self.roi_selection
        if kind == "BODY":
            left = _clamp(left + dx, 0, 320 - roi["w"])
            top = _clamp(top + dy, 0, 240 - roi["h"])
            right, bottom = left + roi["w"], top + roi["h"]
        else:
            if "L" in kind and dx:
                left = _clamp(left + dx, 0, right - 20)
            if "R" in kind and dx:
                right = _clamp(right + dx, left + 20, 320)
            if "T" in kind and dy:
                top = _clamp(top + dy, 0, bottom - 20)
            if "B" in kind and dy:
                bottom = _clamp(bottom + dy, top + 20, 240)
        self._set_roi_edges(left, top, right, bottom)

    def _handle_center_tap(self, x, y):
        image_pos = self._screen_to_img(x, y)
        if image_pos is None:
            return
        roi = self.draft_config["roi"]
        ix, iy = image_pos
        if not (roi["x"] <= ix < roi["x"] + roi["w"] and
                roi["y"] <= iy < roi["y"] + roi["h"]):
            self.message = "请在 ROI 内取点"
            self._invalidate()
            return
        center = self.draft_config["center"]
        marker_x, marker_y = self._img_to_screen(roi["x"] + center["x"], roi["y"] + center["y"])
        near_marker = (x - marker_x) ** 2 + (y - marker_y) ** 2 <= 24 ** 2
        if center["custom"] and not self.center_selected and not near_marker:
            self.message = "请先点击上次保存的中心点"
            self._invalidate()
            return
        if near_marker and center["custom"] and not self.center_selected:
            self.center_selected = True
            self.message = "中心点已选中"
            self._invalidate()
            return
        center["x"] = ix - roi["x"]
        center["y"] = iy - roi["y"]
        center["custom"] = True
        self.center_selected = True
        self.draft_config = validate_config(self.draft_config)
        self.message = "中心点已更新"
        self._invalidate()

    def _move_center(self, direction):
        if not self.center_selected:
            return
        center = self.draft_config["center"]
        roi = self.draft_config["roi"]
        if direction == "左":
            center["x"] -= self.pixel_step
        elif direction == "右":
            center["x"] += self.pixel_step
        elif direction == "上":
            center["y"] -= self.pixel_step
        else:
            center["y"] += self.pixel_step
        center["x"] = _clamp(center["x"], 0, roi["w"] - 1)
        center["y"] = _clamp(center["y"], 0, roi["h"] - 1)
        center["custom"] = True

    def _adjust_threshold(self, sign):
        threshold = self.draft_config["thresholds"][self.color_edit_mode][self.color_edit_key]
        mapping = (("lower", 0), ("upper", 0), ("lower", 1),
                   ("upper", 1), ("lower", 2), ("upper", 2))
        side, channel = mapping[self.threshold_param]
        steps = self._threshold_steps()
        self.threshold_step_index = min(self.threshold_step_index, len(steps) - 1)
        limit = 179 if self.color_edit_mode == "HSV" and channel == 0 else 255
        value = _clamp(threshold[side][channel] + sign * steps[self.threshold_step_index], 0, limit)
        other = "upper" if side == "lower" else "lower"
        allow_hue_wrap = self.color_edit_mode == "HSV" and channel == 0
        if not allow_hue_wrap:
            if side == "lower":
                value = min(value, threshold[other][channel])
            else:
                value = max(value, threshold[other][channel])
        threshold[side][channel] = value

    def _activate(self, button_id):
        # 所有按钮动作都会改变页面或选中状态，下一帧应重建一次静态菜单。
        self._invalidate()
        if button_id == "HOME_MODE":
            self.mode_popup = True
            self.color_popup = False
        elif button_id == "HOME_COLOR":
            self.color_popup = True
            self.mode_popup = False
        elif button_id == "CLOSE_MODE":
            self.mode_popup = False
        elif button_id == "CLOSE_COLOR":
            self.color_popup = False
        elif button_id in ("QUICK_HSV", "QUICK_LAB"):
            mode = "HSV" if button_id == "QUICK_HSV" else "LAB"
            value = copy.deepcopy(self.saved_config)
            value["mode"] = mode
            self.mode_popup = False
            return {"type": "save", "config": value, "stay": True}
        elif button_id.startswith("QUICK_COLOR_"):
            color_key = button_id[len("QUICK_COLOR_"):]
            value = copy.deepcopy(self.saved_config)
            value["target_color"] = color_key
            self.color_popup = False
            return {"type": "save", "config": value, "stay": True}
        elif button_id == "START_RUN":
            self._open(PAGE_RUN)
        elif button_id == "EXIT_RUN":
            self.page = PAGE_HOME
            self.page_stack = []
        elif button_id == "OPEN_CALIBRATE":
            self._open(PAGE_CALIBRATE)
        elif button_id == "BACK":
            self.draft_config = None
            self.roi_drag = None
            self._back()
        elif button_id == "EDIT_ROI":
            self._begin_editor(PAGE_ROI)
        elif button_id == "EDIT_CENTER":
            self._begin_editor(PAGE_CENTER)
        elif button_id == "EDIT_COLOR":
            self._open(PAGE_COLOR_MODE)
        elif button_id in ("COLOR_SPACE_HSV", "COLOR_SPACE_LAB"):
            self.color_edit_mode = "HSV" if button_id.endswith("HSV") else "LAB"
            self._open(PAGE_COLOR_LIST)
        elif button_id.startswith("EDIT_COLOR_"):
            self._begin_threshold(button_id[len("EDIT_COLOR_"):])
        elif button_id.startswith("PIXEL_STEP_"):
            self.pixel_step = int(button_id.split("_")[-1])
        elif button_id.startswith("MOVE_"):
            direction = button_id[len("MOVE_"):]
            if self.page == PAGE_ROI:
                self._move_roi_selection(direction)
            else:
                self._move_center(direction)
        elif button_id == "RESTORE_EDITOR":
            self.draft_config = copy.deepcopy(self.saved_config)
            self.roi_selection = None
            self.center_selected = False
            self.message = "已恢复上次保存参数"
        elif button_id == "RESET_ROI":
            self.draft_config["roi"] = {"x": 0, "y": 0, "w": 320, "h": 240}
            self.draft_config = validate_config(self.draft_config)
            self.roi_selection = "BODY"
        elif button_id == "CLEAR_CENTER":
            roi = self.draft_config["roi"]
            self.draft_config["center"] = {
                "x": roi["w"] // 2, "y": roi["h"] // 2, "custom": False
            }
            self.center_selected = False
        elif button_id == "SAVE_EDITOR":
            return {"type": "save", "config": validate_config(self.draft_config)}
        elif button_id.startswith("VIEW_"):
            self.view_mode = button_id[len("VIEW_"):]
        elif button_id.startswith("THRESH_PARAM_"):
            self.threshold_param = int(button_id.split("_")[-1])
            self.threshold_step_index = 0
        elif button_id.startswith("THRESH_STEP_"):
            self.threshold_step_index = int(button_id.split("_")[-1])
        elif button_id == "THRESH_MINUS":
            self._adjust_threshold(-1)
        elif button_id == "THRESH_PLUS":
            self._adjust_threshold(1)
        elif button_id == "RESTORE_THRESHOLD":
            saved_value = self.saved_config["thresholds"][self.color_edit_mode][self.color_edit_key]
            self.draft_config["thresholds"][self.color_edit_mode][self.color_edit_key] = copy.deepcopy(saved_value)
            self.message = "已恢复上次阈值"
        elif button_id == "DEFAULT_THRESHOLD":
            self.draft_config["thresholds"][self.color_edit_mode][self.color_edit_key] = default_threshold(
                self.color_edit_mode, self.color_edit_key
            )
            self.message = "已恢复默认阈值"
        elif button_id == "SAVE_THRESHOLD":
            self.draft_config["mode"] = self.color_edit_mode
            self.draft_config["target_color"] = self.color_edit_key
            return {"type": "save", "config": validate_config(self.draft_config)}
        return None

    def handle_touch(self, x, y, pressed):
        x = _clamp(x, 0, self.screen_w - 1)
        y = _clamp(y, 0, self.screen_h - 1)
        if pressed:
            if not self.touch_down:
                self.touch_down = True
                self.touch_start = (x, y)
                self.touch_last = (x, y)
                self.touch_moved = False
                self.pressed_button = self._button_at(x, y)
                if self.page == PAGE_ROI and self.pressed_button is None:
                    self._begin_roi_drag(x, y)
            else:
                if abs(x - self.touch_start[0]) > 4 or abs(y - self.touch_start[1]) > 4:
                    self.touch_moved = True
                if self.page == PAGE_ROI and self.roi_drag is not None:
                    self._update_roi_drag(x, y)
                self.touch_last = (x, y)
            return None

        if not self.touch_down:
            return None
        self.touch_down = False
        if self.page == PAGE_ROI and self.roi_drag is not None:
            self._update_roi_drag(x, y)
            self.roi_drag = None
            self.pressed_button = None
            return None

        release_button = self._button_at(x, y)
        pressed_button = self.pressed_button
        self.pressed_button = None
        if pressed_button is not None and release_button == pressed_button:
            return self._activate(pressed_button)
        if self.page == PAGE_CENTER and not self.touch_moved:
            self._handle_center_tap(x, y)
        return None
