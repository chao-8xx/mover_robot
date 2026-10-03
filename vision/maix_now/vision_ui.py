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
    MORPH_CLOSE_ONLY,
    MORPH_NONE,
    MORPH_OPEN_CLOSE,
    MORPH_OPEN_ONLY,
    default_threshold,
    validate_config,
)
from vision_tasks import (
    TASK_LABELS,
    TASK_MATERIAL,
    TASK_NONE,
    TASK_RING,
    TASK_STACK,
)


PAGE_TASKS = "TASKS"
PAGE_HOME = "HOME"
PAGE_RUN = "RUN"
PAGE_CALIBRATE = "CALIBRATE"
PAGE_ROI = "ROI"
PAGE_CENTER = "CENTER"
PAGE_MORPH = "MORPH"
PAGE_COLOR_MODE = "COLOR_MODE"
PAGE_COLOR_LIST = "COLOR_LIST"
PAGE_THRESHOLD = "THRESHOLD"
PAGE_RING_HOME = "RING_HOME"
PAGE_RING_RUN = "RING_RUN"
PAGE_RING_CALIBRATE = "RING_CALIBRATE"
PAGE_RING_BLACK = "RING_BLACK"
PAGE_RING_SIZE = "RING_SIZE"

VIEW_RAW = "RAW"
VIEW_MASK = "MASK"
VIEW_OVERLAY = "OVERLAY"

MORPH_LABELS = {
    MORPH_NONE: "无处理",
    MORPH_OPEN_ONLY: "仅开运算",
    MORPH_CLOSE_ONLY: "仅闭运算",
    MORPH_OPEN_CLOSE: "开运算 + 闭运算",
}

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
    """管理页面状态和触摸交互，不参与相机初始化或识别算法调度。

    saved_config 是已保存参数；编辑页使用 draft_config 即时预览，只有点击
    “保存”才替换 saved_config。所有屏幕到图像的映射都以 320x240 为基准。
    """

    def __init__(self, screen_w, screen_h, config, config_status):
        self.screen_w = max(320, int(screen_w))
        self.screen_h = max(240, int(screen_h))
        self.portrait = self.screen_h >= self.screen_w
        self.saved_config = validate_config(config)
        self.draft_config = None
        self.config_status = config_status

        self.page = PAGE_TASKS
        self.page_stack = []
        self.selected_task = TASK_NONE
        self.external_control = False
        self.buttons = []
        self.text_ops = []
        self.preview_rect = None
        self.mode_popup = False
        self.color_popup = False
        self.message = "视觉系统已就绪"
        self.view_mode = VIEW_OVERLAY
        self.ring_view_mode = VIEW_OVERLAY

        # 中文字体和圆角按钮绘制开销较大。静态菜单只在页面/参数变化时重建，
        # 正常视频帧只覆盖预览区域和英文实时数据，避免每帧重复画整页中文。
        self.ui_revision = 0
        self.static_cache = None
        self.static_cache_revision = -1
        self.static_cache_page = None
        self.static_cache_buttons = []
        self.static_cache_preview_rect = None
        self.building_static = False
        # cv2image(copy=False) 借用 NumPy 内存，因此必须由 UI 持有画布引用到 disp.show 完成。
        self.live_canvas = None
        self.zero_copy_output = True

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
        """编辑页返回草稿参数供即时预览，其余页面只返回已保存参数。"""
        if self.draft_config is not None and self.page in (
            PAGE_ROI, PAGE_CENTER, PAGE_MORPH, PAGE_THRESHOLD,
            PAGE_RING_BLACK, PAGE_RING_SIZE,
        ):
            return self.draft_config
        return self.saved_config

    def _task_config(self, config=None):
        """复用ROI/中心编辑器，同时保持三个任务的几何参数相互独立。"""
        value = config if config is not None else self.draft_config
        if self.selected_task == TASK_RING:
            return value["ring"]
        if self.selected_task == TASK_STACK:
            return value["stack"]
        return value

    def is_run_mode(self):
        """运行页使用高速识别循环，主程序据此限制预览刷新频率。"""
        return (
            (self.selected_task in (TASK_MATERIAL, TASK_STACK) and
             self.page == PAGE_RUN) or
            (self.selected_task == TASK_RING and self.page == PAGE_RING_RUN)
        )

    def current_task(self):
        """首页为待机；进入具体页面后返回当前本地/外部任务。"""
        return self.selected_task if self.page != PAGE_TASKS else TASK_NONE

    def set_external_task(self, task_id):
        """F407拥有任务选择权时，直接切换到对应高速运行页。"""
        task_id = task_id if task_id in (TASK_MATERIAL, TASK_RING, TASK_STACK) else TASK_NONE
        target_page = PAGE_TASKS
        if task_id == TASK_MATERIAL:
            target_page = PAGE_RUN
        elif task_id == TASK_RING:
            target_page = PAGE_RING_RUN
        elif task_id == TASK_STACK:
            target_page = PAGE_RUN

        changed = (
            not self.external_control or
            self.selected_task != task_id or
            self.page != target_page
        )
        self.external_control = True
        self.selected_task = task_id
        if not changed:
            return
        self.page = target_page
        self.page_stack = []
        self.mode_popup = False
        self.color_popup = False
        if task_id == TASK_NONE:
            self.message = "等待 F407 视觉命令"
        elif task_id == TASK_MATERIAL:
            self.message = "F407 已启动物料识别"
        elif task_id == TASK_RING:
            self.message = "F407 已启动圆环中心定位"
        else:
            self.message = "F407 已启动码垛物料识别"
        self._invalidate()

    def set_external_run(self, enabled):
        """让 F407 任务状态进入/退出 RUN 页，不改动或保存调参内容。"""
        self.set_external_task(TASK_MATERIAL if enabled else TASK_NONE)

    def set_runtime_target(self, color_key):
        """显示 F407 当前指定颜色；只改内存，不主动写入 JSON。"""
        if color_key not in COLOR_ORDER:
            return
        if self.saved_config["target_color"] == color_key:
            return
        self.saved_config["target_color"] = color_key
        self.color_edit_key = color_key
        self.message = "F407 目标：" + COLOR_LABELS[color_key]
        self._invalidate()

    def set_runtime_ring_target(self, target_number):
        if target_number != 2:
            return
        if self.saved_config["ring"]["target_number"] == target_number:
            return
        self.saved_config["ring"]["target_number"] = target_number
        self.message = "F407 圆环目标：{}".format(target_number)
        self._invalidate()

    def current_view_mode(self):
        if self.selected_task == TASK_RING:
            return self.ring_view_mode
        # 形态学页固定观察二值 MASK；阈值页保留 RAW/MASK/OVERLAY 三档。
        if self.page == PAGE_MORPH:
            return VIEW_MASK
        return self.view_mode if self.page == PAGE_THRESHOLD else VIEW_RAW

    def needs_mask_for_display(self):
        if self.selected_task == TASK_RING:
            return self.ring_view_mode == VIEW_MASK
        return (
            self.page == PAGE_MORPH or
            (self.page == PAGE_THRESHOLD and self.view_mode in (VIEW_MASK, VIEW_OVERLAY))
        )

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
        if self.page == PAGE_TASKS:
            self.selected_task = TASK_NONE
        self.mode_popup = False
        self.color_popup = False
        self._invalidate()

    def _open_task(self, task_id):
        """本机调试从三任务首页进入任务；正式模式不调用此入口。"""
        self.external_control = False
        self.selected_task = task_id
        self.page_stack = [PAGE_TASKS]
        if task_id == TASK_MATERIAL:
            self.page = PAGE_HOME
        elif task_id == TASK_RING:
            self.page = PAGE_RING_HOME
        else:
            self.page = PAGE_HOME
        self.message = TASK_LABELS[task_id]
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
        self.draft_config["target_color"] = color_key
        self.draft_config["mode"] = self.color_edit_mode
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
                "返回", font="small"
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
    def _draw_task_selector(self, canvas):
        self._header(canvas, "视觉任务首页")
        self._text("请选择本机调试任务", 0, 0, MUTED, "normal",
                   (12, 58, self.screen_w - 24, 42))

        items = (
            ("SELECT_TASK_1", "任务一  物料识别", True),
            ("SELECT_TASK_2", "任务二  2号圆环定位", True),
            ("SELECT_TASK_3", "任务三  码垛堆放", True),
        )
        if self.portrait:
            start_y = 118
            gap = 18
            button_h = min(112, (self.screen_h - start_y - 104 - gap * 2) // 3)
            for index, item in enumerate(items):
                rect = (16, start_y + index * (button_h + gap), self.screen_w - 32, button_h)
                self._button(canvas, item[0], rect, item[1], active=item[2], font="title")
        else:
            gap = 12
            button_w = (self.screen_w - 32 - gap * 2) // 3
            for index, item in enumerate(items):
                rect = (16 + index * (button_w + gap), 128, button_w, self.screen_h - 190)
                self._button(canvas, item[0], rect, item[1], active=item[2], font="small")

        self._text("B2全图三色 · B3纯圆心 · RUN目标40Hz", 0, 0, ACCENT, "small",
                   (10, self.screen_h - 54, self.screen_w - 20, 36))
        self.preview_rect = None

    def _draw_ring_view_buttons(self, canvas, y, x=10, width=None):
        width = self.screen_w - 20 if width is None else width
        items = (
            (VIEW_RAW, "原图"),
            (VIEW_MASK, "MASK"),
            (VIEW_OVERLAY, "候选"),
        )
        gap = 8
        button_w = (width - gap * 2) // 3
        for index, item in enumerate(items):
            self._button(
                canvas,
                "RING_VIEW_" + item[0],
                (x + index * (button_w + gap), y, button_w, 42),
                item[1],
                active=self.ring_view_mode == item[0],
                font="small",
            )

    def _draw_ring_home(self, canvas, preview_bgr, result):
        self._header(canvas, "任务二 圆环中心定位", back=True)
        if self.portrait:
            rect = self._preview(canvas, preview_bgr, (8, 55, self.screen_w - 16, 292))
            y = rect[1] + rect[3] + 8
            self._draw_ring_view_buttons(canvas, y)
            self._button(canvas, "START_RING_RUN", (12, y + 52, self.screen_w - 24, 54),
                         "开始定位2号圆环", active=True, font="normal")
            self._button(canvas, "OPEN_RING_CALIBRATE",
                         (12, y + 114, self.screen_w - 24, 54),
                         "任务二调参", font="normal")
        else:
            rect = self._preview(canvas, preview_bgr,
                                 (8, 55, int(self.screen_w * 0.62), self.screen_h - 63))
            panel_x = rect[0] + rect[2] + 10
            panel_w = self.screen_w - panel_x - 8
            self._draw_ring_view_buttons(canvas, 66, panel_x, panel_w)
            self._button(canvas, "START_RING_RUN", (panel_x, 128, panel_w, 64),
                         "定位圆心", active=True, font="small")
            self._button(canvas, "OPEN_RING_CALIBRATE", (panel_x, 204, panel_w, 64),
                         "任务二调参", font="small")

    def _draw_ring_run(self, canvas, preview_bgr, result):
        self._draw_fast_run(canvas, ring=True)

    def _draw_ring_calibrate(self, canvas, preview_bgr):
        self._header(canvas, "任务二调参", back=True)
        items = (
            ("EDIT_RING_ROI", "圆环 ROI"),
            ("EDIT_RING_CENTER", "对准中心"),
            ("EDIT_RING_BLACK", "黑色阈值"),
            ("EDIT_RING_SIZE", "圆环尺寸"),
        )
        if self.portrait:
            rect = self._preview(canvas, preview_bgr, (8, 55, self.screen_w - 16, 175))
            start_y = rect[1] + rect[3] + 12
            for index, item in enumerate(items):
                self._button(canvas, item[0],
                             (12, start_y + index * 72, self.screen_w - 24, 64),
                             item[1], active=index == 0)
        else:
            rect = self._preview(canvas, preview_bgr,
                                 (8, 55, int(self.screen_w * 0.60), self.screen_h - 63))
            panel_x = rect[0] + rect[2] + 12
            panel_w = self.screen_w - panel_x - 8
            for index, item in enumerate(items):
                self._button(canvas, item[0],
                             (panel_x, 58 + index * 93, panel_w, 82),
                             item[1], active=index == 0, font="small")

    def _draw_ring_black(self, canvas, preview_bgr):
        self._header(canvas, "黑色阈值", back=True)
        ring = self.draft_config["ring"]
        if self.portrait:
            rect = self._preview(canvas, preview_bgr, (8, 55, self.screen_w - 16, 330))
            y = rect[1] + rect[3] + 12
            self._text("亮度上限 V = {}".format(ring["black_v_max"]), 12, y,
                       ACCENT, "normal")
            self._button(canvas, "RING_BLACK_MINUS", (12, y + 42, 132, 50), "－", font="title")
            self._button(canvas, "RING_BLACK_PLUS", (154, y + 42, 132, 50), "＋",
                         active=True, font="title")
            self._text("步长", 300, y + 54, MUTED, "small")
            for index, step in enumerate((1, 5, 10)):
                self._button(canvas, "RING_STEP_" + str(step),
                             (350, y + index * 48, 70, 42), str(step),
                             active=self.pixel_step == step, font="small")
            self._button(canvas, "SAVE_EDITOR", (12, self.screen_h - 58, 220, 48),
                         "保存", active=True)
            self._button(canvas, "RESTORE_EDITOR", (242, self.screen_h - 58, 220, 48),
                         "恢复上次")
        else:
            rect = self._preview(canvas, preview_bgr,
                                 (8, 55, int(self.screen_w * 0.64), self.screen_h - 63))
            x = rect[0] + rect[2] + 12
            w = self.screen_w - x - 8
            self._text("V = {}".format(ring["black_v_max"]), x, 78, ACCENT, "title")
            self._button(canvas, "RING_BLACK_MINUS", (x, 136, w // 2 - 4, 54), "－")
            self._button(canvas, "RING_BLACK_PLUS", (x + w // 2, 136, w // 2 - 4, 54), "＋", active=True)
            for index, step in enumerate((1, 5, 10)):
                self._button(canvas, "RING_STEP_" + str(step),
                             (x + index * (w // 3), 210, w // 3 - 4, 44), str(step),
                             active=self.pixel_step == step, font="small")
            self._button(canvas, "SAVE_EDITOR", (x, 286, w, 48), "保存", active=True)
            self._button(canvas, "RESTORE_EDITOR", (x, 344, w, 48), "恢复上次")

    def _draw_ring_size(self, canvas, preview_bgr):
        self._header(canvas, "圆环尺寸", back=True)
        ring = self.draft_config["ring"]
        if self.portrait:
            rect = self._preview(canvas, preview_bgr, (8, 55, self.screen_w - 16, 300))
            y = rect[1] + rect[3] + 10
            rows = (
                ("MIN", "最小外径 {} px".format(ring["min_diameter"])),
                ("MAX", "最大外径 {} px".format(ring["max_diameter"])),
            )
            for index, row in enumerate(rows):
                row_y = y + index * 78
                self._text(row[1], 14, row_y + 12, WHITE, "normal")
                self._button(canvas, "RING_SIZE_" + row[0] + "_MINUS",
                             (290, row_y, 72, 48), "－")
                self._button(canvas, "RING_SIZE_" + row[0] + "_PLUS",
                             (372, row_y, 72, 48), "＋", active=True)
            self._button(canvas, "SAVE_EDITOR", (12, self.screen_h - 58, 220, 48),
                         "保存", active=True)
            self._button(canvas, "RESTORE_EDITOR", (242, self.screen_h - 58, 220, 48),
                         "恢复上次")
        else:
            rect = self._preview(canvas, preview_bgr,
                                 (8, 55, int(self.screen_w * 0.60), self.screen_h - 63))
            x = rect[0] + rect[2] + 12
            w = self.screen_w - x - 8
            for index, item in enumerate((("MIN", "最小外径", "min_diameter"),
                                          ("MAX", "最大外径", "max_diameter"))):
                y = 74 + index * 120
                self._text("{} {} px".format(item[1], ring[item[2]]), x, y, WHITE, "small")
                self._button(canvas, "RING_SIZE_" + item[0] + "_MINUS",
                             (x, y + 38, w // 2 - 4, 48), "－")
                self._button(canvas, "RING_SIZE_" + item[0] + "_PLUS",
                             (x + w // 2, y + 38, w // 2 - 4, 48), "＋", active=True)
            self._button(canvas, "SAVE_EDITOR", (x, 330, w, 48), "保存", active=True)
            self._button(canvas, "RESTORE_EDITOR", (x, 388, w, 48), "恢复上次")

    def _draw_home(self, canvas, preview_bgr, result):
        self._header(canvas, TASK_LABELS.get(self.selected_task, "颜色识别"), back=True)
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
        self._draw_fast_run(canvas)

    def _draw_fast_run(self, canvas, ring=False):
        """高速RUN只绘制一次状态页，触摸返回继续有效。"""
        self._header(canvas, TASK_LABELS.get(self.selected_task, "运行中"),
                     back=ring and not self.external_control, exit_run=not ring)
        lines = ["高速识别运行中", "识别与发送目标：40Hz",
                 "画面暂停刷新，实际FPS与发送Hz查看日志"]
        if ring:
            lines.append("只定位圆环中心，不识别数字")
        else:
            lines.append("当前输出颜色：" + COLOR_LABELS[self.saved_config["target_color"]])
        if not self.external_control:
            lines.append("点击返回，恢复实时调参画面")
        for index, label in enumerate(lines):
            self._text(label, 16, 94 + index * 48,
                       ACCENT if index == 0 else WHITE, "normal" if index == 0 else "small")
        self.preview_rect = None

    def _draw_calibrate(self, canvas, preview_bgr):
        self._header(canvas, "调参", back=True)
        items = (
            ("EDIT_ROI", "ROI 识别区域"),
            ("EDIT_CENTER", "取中心点"),
            ("EDIT_COLOR", "颜色阈值"),
            ("EDIT_MORPH", "形态学实验"),
        )
        if self.selected_task == TASK_MATERIAL:
            items = items[1:]
        if self.portrait:
            rect = self._preview(canvas, preview_bgr, (8, 55, self.screen_w - 16, 175))
            start_y = rect[1] + rect[3] + 12
            for index, item in enumerate(items):
                self._button(
                    canvas, item[0], (12, start_y + index * 72, self.screen_w - 24, 64),
                    item[1], active=index == 0
                )
            self._text(self.message, 14, self.screen_h - 28, ACCENT, "small")
        else:
            rect = self._preview(canvas, preview_bgr, (8, 55, int(self.screen_w * 0.60), self.screen_h - 63))
            x = rect[0] + rect[2] + 12
            w = self.screen_w - x - 8
            for index, item in enumerate(items):
                self._button(
                    canvas, item[0], (x, 58 + index * 93, w, 82),
                    item[1], active=index == 0
                )

    def _draw_morph(self, canvas, preview_bgr):
        """四档形态学实验页：选项即时作用于 MASK，保存后才供 RUN 使用。"""
        self._header(canvas, "形态学实验", back=True)
        mode = self.draft_config["morph_mode"]
        options = (
            MORPH_NONE,
            MORPH_OPEN_ONLY,
            MORPH_CLOSE_ONLY,
            MORPH_OPEN_CLOSE,
        )

        if self.portrait:
            rect = self._preview(canvas, preview_bgr, (8, 55, self.screen_w - 16, 220))
            self._text("MASK 实时预览 · 当前：" + MORPH_LABELS[mode],
                       12, rect[1] + rect[3] + 8, ACCENT, "small")
            grid_y = rect[1] + rect[3] + 38
            gap = 8
            cell_w = (self.screen_w - 24 - gap) // 2
            for index, option in enumerate(options):
                col = index % 2
                row = index // 2
                self._button(
                    canvas, "SET_MORPH_" + option,
                    (8 + col * (cell_w + gap), grid_y + row * 66, cell_w, 58),
                    MORPH_LABELS[option], active=mode == option, font="small"
                )
            action_y = self.screen_h - 58
            action_w = (self.screen_w - 22) // 2
            self._button(canvas, "SAVE_EDITOR", (8, action_y, action_w, 48),
                         "保存", active=True)
            self._button(canvas, "RESTORE_EDITOR", (14 + action_w, action_y, action_w, 48),
                         "恢复上次")
        else:
            rect = self._preview(
                canvas, preview_bgr, (8, 55, int(self.screen_w * 0.62), self.screen_h - 63)
            )
            panel_x = rect[0] + rect[2] + 10
            panel_w = self.screen_w - panel_x - 8
            self._text("MASK · " + MORPH_LABELS[mode], panel_x, 58, ACCENT, "small")
            for index, option in enumerate(options):
                self._button(
                    canvas, "SET_MORPH_" + option,
                    (panel_x, 92 + index * 58, panel_w, 50),
                    MORPH_LABELS[option], active=mode == option, font="small"
                )
            self._button(canvas, "SAVE_EDITOR", (panel_x, 342, panel_w, 44),
                         "保存", active=True)
            self._button(canvas, "RESTORE_EDITOR", (panel_x, 396, panel_w, 44),
                         "恢复上次")

    def _draw_roi_overlay(self, canvas):
        roi = self._task_config()["roi"]
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
        if self.selected_task == TASK_RING:
            title = "圆环 ROI"
        elif self.selected_task == TASK_STACK:
            title = "码垛 ROI"
        else:
            title = "ROI 识别区域"
        self._header(canvas, title, back=True)
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
        task_config = self._task_config()
        roi = task_config["roi"]
        center = task_config["center"]
        sx, sy = self._img_to_screen(roi["x"] + center["x"], roi["y"] + center["y"])
        color = ACCENT if self.center_selected else WHITE
        cv2.drawMarker(canvas, (sx, sy), color, cv2.MARKER_CROSS, 24, 3)
        cv2.circle(canvas, (sx, sy), 13, color, 2)

    def _draw_center(self, canvas, preview_bgr):
        if self.selected_task == TASK_RING:
            title = "圆环对准中心"
        elif self.selected_task == TASK_STACK:
            title = "码垛对准中心"
        else:
            title = "取中心点"
        self._header(canvas, title, back=True)
        area = ((8, 55, self.screen_w - 16, 348) if self.portrait else
                (8, 55, int(self.screen_w * 0.64), self.screen_h - 63))
        self._preview(canvas, preview_bgr, area)
        if not self.building_static:
            self._draw_center_marker(canvas)
        center = self._task_config()["center"]
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
        if self.page == PAGE_TASKS:
            self._draw_task_selector(canvas)
        elif self.page == PAGE_HOME:
            self._draw_home(canvas, preview_bgr, result)
        elif self.page == PAGE_RUN:
            self._draw_run(canvas, preview_bgr, result)
        elif self.page == PAGE_RING_HOME:
            self._draw_ring_home(canvas, preview_bgr, result)
        elif self.page == PAGE_RING_RUN:
            self._draw_ring_run(canvas, preview_bgr, result)
        elif self.page == PAGE_RING_CALIBRATE:
            self._draw_ring_calibrate(canvas, preview_bgr)
        elif self.page == PAGE_RING_BLACK:
            self._draw_ring_black(canvas, preview_bgr)
        elif self.page == PAGE_RING_SIZE:
            self._draw_ring_size(canvas, preview_bgr)
        elif self.page == PAGE_CALIBRATE:
            self._draw_calibrate(canvas, preview_bgr)
        elif self.page == PAGE_ROI:
            self._draw_roi(canvas, preview_bgr)
        elif self.page == PAGE_CENTER:
            self._draw_center(canvas, preview_bgr)
        elif self.page == PAGE_MORPH:
            self._draw_morph(canvas, preview_bgr)
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

    def _draw_realtime_info(self, canvas, result, vision_fps, display_fps):
        """实时文字用 OpenCV 小字体绘制，避免每帧调用中文字体渲染。"""
        if self.preview_rect is None:
            return
        x, y, w, h = self.preview_rect
        cv2.rectangle(canvas, (x + 4, y + 4), (x + 142, y + 25), (16, 19, 24), -1)
        cv2.putText(canvas, "V:{:.1f} D:{:.1f}".format(vision_fps, display_fps),
                    (x + 9, y + 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.43, (80, 255, 170), 1)
        if self.selected_task == TASK_RING:
            info = "C:{} LOCK:{} {}".format(
                result.get("candidate_count", 0),
                1 if result.get("target_confirmed", False) else 0,
                result.get("state", "SEARCH"),
            )
        elif result["found"]:
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

    def render(self, preview_bgr, result, vision_fps=0.0, display_fps=0.0):
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
        self._draw_realtime_info(canvas, result, vision_fps, display_fps)

        # ROI 四角和中心点属于动态标记，覆盖实时画面后再补画。
        if self.page == PAGE_ROI:
            self._draw_roi_overlay(canvas)
        elif self.page == PAGE_CENTER:
            self._draw_center_marker(canvas)
        self.live_canvas = canvas
        if self.zero_copy_output:
            try:
                return image.cv2image(self.live_canvas, bgr=True, copy=False)
            except (TypeError, AttributeError) as exc:
                print("[VISION UI] zero-copy output unavailable:", exc)
                self.zero_copy_output = False
        return image.cv2image(self.live_canvas)

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
        roi = self._task_config()["roi"]
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
            "roi": copy.deepcopy(self._task_config()["roi"]),
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
        self._task_config()["roi"] = {
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
            task_config = self._task_config()
            task_config["roi"]["x"] = new_left
            task_config["roi"]["y"] = new_top
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
        roi = self._task_config()["roi"]
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
        task_config = self._task_config()
        roi = task_config["roi"]
        ix, iy = image_pos
        if not (roi["x"] <= ix < roi["x"] + roi["w"] and
                roi["y"] <= iy < roi["y"] + roi["h"]):
            self.message = "请在 ROI 内取点"
            self._invalidate()
            return
        center = task_config["center"]
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
        task_config = self._task_config()
        center = task_config["center"]
        roi = task_config["roi"]
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

    def _adjust_ring_value(self, key, delta):
        ring = self.draft_config["ring"]
        if key == "black_v_max":
            ring[key] = _clamp(ring[key] + delta, 0, 255)
        elif key == "min_diameter":
            ring[key] = _clamp(ring[key] + delta, 8, ring["max_diameter"])
        elif key == "max_diameter":
            ring[key] = _clamp(ring[key] + delta, ring["min_diameter"], 320)
        self.draft_config = validate_config(self.draft_config)
        self.message = "任务二参数已更新，保存后生效"

    def _activate(self, button_id):
        # 所有按钮动作都会改变页面或选中状态，下一帧应重建一次静态菜单。
        self._invalidate()
        if button_id == "SELECT_TASK_1":
            self._open_task(TASK_MATERIAL)
        elif button_id == "SELECT_TASK_2":
            self._open_task(TASK_RING)
        elif button_id == "SELECT_TASK_3":
            self._open_task(TASK_STACK)
        elif button_id == "HOME_MODE":
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
        elif button_id.startswith("RING_VIEW_"):
            self.ring_view_mode = button_id[len("RING_VIEW_"):]
        elif button_id == "START_RING_RUN":
            self._open(PAGE_RING_RUN)
        elif button_id == "OPEN_RING_CALIBRATE":
            self._open(PAGE_RING_CALIBRATE)
        elif button_id == "EDIT_RING_ROI":
            self._begin_editor(PAGE_ROI)
            self.ring_view_mode = VIEW_OVERLAY
        elif button_id == "EDIT_RING_CENTER":
            self._begin_editor(PAGE_CENTER)
            self.ring_view_mode = VIEW_OVERLAY
        elif button_id == "EDIT_RING_BLACK":
            self._begin_editor(PAGE_RING_BLACK)
            self.ring_view_mode = VIEW_MASK
        elif button_id == "EDIT_RING_SIZE":
            self._begin_editor(PAGE_RING_SIZE)
            self.ring_view_mode = VIEW_OVERLAY
        elif button_id.startswith("RING_STEP_"):
            self.pixel_step = int(button_id.split("_")[-1])
        elif button_id == "RING_BLACK_MINUS":
            self._adjust_ring_value("black_v_max", -self.pixel_step)
        elif button_id == "RING_BLACK_PLUS":
            self._adjust_ring_value("black_v_max", self.pixel_step)
        elif button_id == "RING_SIZE_MIN_MINUS":
            self._adjust_ring_value("min_diameter", -5)
        elif button_id == "RING_SIZE_MIN_PLUS":
            self._adjust_ring_value("min_diameter", 5)
        elif button_id == "RING_SIZE_MAX_MINUS":
            self._adjust_ring_value("max_diameter", -5)
        elif button_id == "RING_SIZE_MAX_PLUS":
            self._adjust_ring_value("max_diameter", 5)
        elif button_id == "START_RUN":
            self._open(PAGE_RUN)
        elif button_id == "EXIT_RUN":
            self.page = PAGE_HOME
            self.page_stack = []
            if not self.external_control:
                self.page_stack = [PAGE_TASKS]
        elif button_id == "OPEN_CALIBRATE":
            self._open(PAGE_CALIBRATE)
        elif button_id == "BACK":
            self.draft_config = None
            self.roi_drag = None
            self._back()
        elif button_id == "EDIT_ROI":
            if self.selected_task != TASK_MATERIAL:
                self._begin_editor(PAGE_ROI)
            else:
                self.message = "物料识别使用固定全图ROI"
        elif button_id == "EDIT_CENTER":
            self._begin_editor(PAGE_CENTER)
        elif button_id == "EDIT_COLOR":
            self._open(PAGE_COLOR_MODE)
        elif button_id == "EDIT_MORPH":
            self._begin_editor(PAGE_MORPH)
            self.view_mode = VIEW_MASK
        elif button_id.startswith("SET_MORPH_"):
            morph_mode = button_id[len("SET_MORPH_"):]
            if morph_mode in MORPH_LABELS:
                self.draft_config["morph_mode"] = morph_mode
                self.message = "当前形态学：" + MORPH_LABELS[morph_mode]
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
            self._task_config()["roi"] = {"x": 0, "y": 0, "w": 320, "h": 240}
            self.draft_config = validate_config(self.draft_config)
            self.roi_selection = "BODY"
        elif button_id == "CLEAR_CENTER":
            task_config = self._task_config()
            roi = task_config["roi"]
            task_config["center"] = {
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
