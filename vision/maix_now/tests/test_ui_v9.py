import copy
import sys
import types
import unittest
from pathlib import Path

import numpy as np


MODULE_DIR = Path(__file__).resolve().parents[1]
if str(MODULE_DIR) not in sys.path:
    sys.path.insert(0, str(MODULE_DIR))


class _FakeImageValue:
    def __init__(self, array):
        self.array = array

    def draw_string(self, *args, **kwargs):
        return None


class _FakeColor:
    @staticmethod
    def from_rgb(red, green, blue):
        return red, green, blue


class _FakeImageModule:
    Color = _FakeColor

    @staticmethod
    def load_font(*args, **kwargs):
        return None

    @staticmethod
    def set_default_font(*args, **kwargs):
        return None

    @staticmethod
    def cv2image(array, *args, **kwargs):
        return _FakeImageValue(array)

    @staticmethod
    def image2cv(value, *args, **kwargs):
        return value.array if isinstance(value, _FakeImageValue) else value


fake_maix = types.ModuleType("maix")
fake_maix.image = _FakeImageModule
sys.modules.setdefault("maix", fake_maix)

from vision_blob import empty_result  # noqa: E402
from vision_config import COLOR_ORDER, DEFAULT_CONFIG  # noqa: E402
from vision_tasks import TASK_MATERIAL, TASK_RING, TASK_STACK  # noqa: E402
from vision_ui import (  # noqa: E402
    PAGE_CENTER,
    PAGE_COLOR_LIST,
    PAGE_HOME,
    PAGE_RING_CALIBRATE,
    PAGE_RING_HOME,
    PAGE_ROI,
    VisionUI,
)


class VisionUiTests(unittest.TestCase):
    def setUp(self):
        self.ui = VisionUI(
            640, 480, copy.deepcopy(DEFAULT_CONFIG), "使用默认参数"
        )
        self.frame = np.zeros((240, 320, 3), dtype=np.uint8)

    def test_b4_uses_shared_color_pages_and_run_button(self):
        self.ui._open_task(TASK_STACK)
        self.assertEqual(self.ui.page, PAGE_HOME)
        self.ui.render(self.frame, empty_result())
        self.assertEqual(self.ui.current_task(), TASK_STACK)
        self.ui._activate("START_RUN")
        self.assertTrue(self.ui.is_run_mode())
        self.ui.render(self.frame, empty_result())
        self.assertEqual(self.ui.preview_rect[1], 7)

    def test_b4_center_editor_uses_stack_geometry(self):
        self.ui._open_task(TASK_STACK)
        self.ui._begin_editor(PAGE_CENTER)
        self.assertIs(
            self.ui._task_config(), self.ui.draft_config["stack"]
        )
        self.assertIsNot(
            self.ui._task_config()["center"],
            self.ui.draft_config["center"],
        )

    def test_ring_menu_has_no_digit_target_or_ocr_view(self):
        self.ui._open_task(TASK_RING)
        self.assertEqual(self.ui.page, PAGE_RING_HOME)
        self.ui.render(self.frame, empty_result())
        button_ids = {button[0] for button in self.ui.buttons}
        self.assertFalse(any(
            button_id.startswith("RING_TARGET_") for button_id in button_ids
        ))
        self.assertNotIn("RING_VIEW_OCR", button_ids)
        self.assertIn("START_RING_RUN", button_ids)

    def test_ring_calibrate_exposes_independent_roi_editor(self):
        self.ui._open_task(TASK_RING)
        self.ui._activate("OPEN_RING_CALIBRATE")
        self.assertEqual(self.ui.page, PAGE_RING_CALIBRATE)
        self.ui.render(self.frame, empty_result())
        button_ids = {button[0] for button in self.ui.buttons}
        self.assertIn("EDIT_RING_ROI", button_ids)

        self.ui._activate("EDIT_RING_ROI")
        self.assertEqual(self.ui.page, PAGE_ROI)
        self.assertIs(
            self.ui._task_config(), self.ui.draft_config["ring"]
        )

    def test_b2_color_page_still_runs(self):
        self.ui._open_task(TASK_MATERIAL)
        self.assertEqual(self.ui.page, PAGE_HOME)
        self.ui.render(self.frame, empty_result())
        self.ui._activate("START_RUN")
        self.assertTrue(self.ui.is_run_mode())
        self.ui.render(self.frame, empty_result())
        self.assertEqual(self.ui.preview_rect[1], 7)

    def test_b2_no_roi_editor_but_b4_keeps_one(self):
        for task in (TASK_MATERIAL, TASK_STACK):
            self.ui._open_task(task)
            self.ui._activate("OPEN_CALIBRATE")
            self.ui.render(self.frame, empty_result())
            ids = {button[0] for button in self.ui.buttons}
            self.assertEqual("EDIT_ROI" in ids, task == TASK_STACK)

    def test_threshold_preview_uses_only_edited_lab_color(self):
        self.ui._open_task(TASK_MATERIAL)
        self.ui._begin_threshold("blue")
        cfg = self.ui.detection_config()
        self.assertEqual(cfg["target_color"], "blue")
        self.assertNotIn("mode", cfg)
        self.assertEqual(self.ui.saved_config["target_color"], DEFAULT_CONFIG["target_color"])

    def test_b2_and_b4_home_and_editor_offer_all_six_colors_without_mode_buttons(self):
        for size in ((640, 480), (480, 640)):
            ui = VisionUI(*size, copy.deepcopy(DEFAULT_CONFIG), "使用默认参数")
            for task in (TASK_MATERIAL, TASK_STACK):
                ui._open_task(task)
                ui.render(self.frame, empty_result())
                self.assertNotIn("HOME_MODE", {b[0] for b in ui.buttons})
                ui._activate("HOME_COLOR")
                ui.render(self.frame, empty_result())
                ids = {b[0] for b in ui.buttons if b[0].startswith("QUICK_COLOR_")}
                self.assertEqual(ids, {"QUICK_COLOR_" + k for k in COLOR_ORDER})
                ui._activate("CLOSE_COLOR")
                ui._activate("OPEN_CALIBRATE")
                ui._activate("EDIT_COLOR")
                self.assertEqual(ui.page, PAGE_COLOR_LIST)
                ui.render(self.frame, empty_result())
                ids = {b[0] for b in ui.buttons if b[0].startswith("EDIT_COLOR_")}
                self.assertEqual(ids, {"EDIT_COLOR_" + k for k in COLOR_ORDER})

    def test_edit_and_save_each_color_preserves_other_thresholds(self):
        self.ui._open_task(TASK_MATERIAL)
        for key in COLOR_ORDER:
            original = copy.deepcopy(self.ui.saved_config["thresholds"]["LAB"])
            self.ui._begin_threshold(key)
            self.ui.threshold_param = 0
            self.ui._activate("THRESH_PLUS")
            action = self.ui._activate("SAVE_THRESHOLD")
            thresholds = action["config"]["thresholds"]["LAB"]
            self.assertEqual(thresholds[key]["lower"][0], original[key]["lower"][0] + 1)
            for other in set(original) - {key}:
                self.assertEqual(thresholds[other], original[other])
            self.ui.save_succeeded(action["config"])

    def test_all_six_quick_color_buttons_select_and_save_without_changing_thresholds(self):
        self.ui._open_task(TASK_MATERIAL)
        original = copy.deepcopy(self.ui.saved_config["thresholds"])
        for key in COLOR_ORDER:
            action = self.ui._activate("QUICK_COLOR_" + key)
            self.assertEqual(action["config"]["target_color"], key)
            self.assertEqual(action["config"]["thresholds"], original)
            self.ui.set_saved_config(action["config"])

    def test_ring_live_run_accepts_empty_preview_while_waiting_for_frame(self):
        self.ui._open_task(TASK_RING)
        self.ui._activate("START_RING_RUN")
        self.ui.render(None, {})
        self.assertTrue(self.ui.is_run_mode())
        self.assertEqual(self.ui.preview_rect[1], 7)

    def test_debug_preview_top_buttons_and_footer_have_matching_hit_regions(self):
        for size in ((640, 480), (480, 640)):
            for task in (TASK_MATERIAL, TASK_STACK, TASK_RING):
                ui = VisionUI(*size, copy.deepcopy(DEFAULT_CONFIG), "使用默认参数")
                ui._open_task(task)
                ui.render(self.frame, empty_result())
                self.assertEqual(ui.preview_rect[1], 7)
                title = next(t for t in ui.text_ops if t[4] == "title")
                self.assertGreaterEqual(title[1], size[1] - 48)
                for name, x, y, w, h in ui.buttons:
                    self.assertGreaterEqual(y, 0, name)
                    self.assertLessEqual(y + h, size[1], name)
                    self.assertEqual(ui._button_at(x + w // 2, y + h // 2), name)

    def test_shifted_threshold_touch_changes_only_current_draft(self):
        self.ui._open_task(TASK_MATERIAL)
        self.ui._begin_threshold("red")
        self.ui.render(self.frame, empty_result())
        before = copy.deepcopy(self.ui.saved_config)
        old_value = self.ui.draft_config["thresholds"]["LAB"]["red"]["lower"][0]
        _, x, y, w, h = next(b for b in self.ui.buttons if b[0] == "THRESH_PLUS")
        self.assertEqual(y, 184)
        self.ui.handle_touch(x + w // 2, y + h // 2, True)
        self.ui.handle_touch(x + w // 2, y + h // 2, False)
        self.assertEqual(self.ui.draft_config["thresholds"]["LAB"]["red"]["lower"][0], old_value + 1)
        self.assertEqual(self.ui.saved_config, before)

    def test_shifted_center_preview_maps_to_same_image_coordinates(self):
        self.ui.saved_config["stack"]["roi"] = dict(x=110,y=25,w=135,h=135)
        self.ui.saved_config["stack"]["center"] = dict(x=67,y=67,custom=False)
        self.ui._open_task(TASK_STACK)
        self.ui._begin_editor(PAGE_CENTER)
        self.ui.render(self.frame, empty_result())
        point = (177, 92)
        sx, sy = self.ui._img_to_screen(*point)
        ix, iy = self.ui._screen_to_img(sx, sy)
        self.assertAlmostEqual(ix, point[0], delta=1)
        self.assertAlmostEqual(iy, point[1], delta=1)
        self.ui._handle_center_tap(sx, sy)
        self.assertTrue(self.ui.center_selected)
        sx, sy = self.ui._img_to_screen(180, 95)
        self.ui._handle_center_tap(sx, sy)
        center = self.ui.draft_config["stack"]["center"]
        self.assertAlmostEqual(center["x"], 180 - 110, delta=1)
        self.assertAlmostEqual(center["y"], 95 - 25, delta=1)

    def test_shifted_roi_drag_moves_roi_in_image_units_without_moving_center(self):
        self.ui._open_task(TASK_STACK)
        self.ui._begin_editor(PAGE_ROI)
        self.ui.render(self.frame, empty_result())
        original = copy.deepcopy(self.ui.draft_config["stack"])
        sx, sy = self.ui._img_to_screen(170, 85)
        self.assertTrue(self.ui._begin_roi_drag(sx, sy))
        tx, ty = self.ui._img_to_screen(180, 95)
        self.ui._update_roi_drag(tx, ty)
        roi = self.ui.draft_config["stack"]["roi"]
        self.assertAlmostEqual(roi["x"], original["roi"]["x"] + 10, delta=1)
        self.assertAlmostEqual(roi["y"], original["roi"]["y"] + 10, delta=1)
        self.assertEqual((roi["w"], roi["h"]), (original["roi"]["w"], original["roi"]["h"]))
        self.assertEqual(self.ui.draft_config["stack"]["center"], original["center"])

    def test_layout_move_preserves_cached_hit_regions_and_live_run_footer(self):
        self.ui._open_task(TASK_MATERIAL)
        self.ui.render(self.frame, empty_result())
        buttons, rect = list(self.ui.buttons), self.ui.preview_rect
        self.ui.render(self.frame, empty_result())
        self.assertEqual(self.ui.buttons, buttons)
        self.assertEqual(self.ui.preview_rect, rect)
        self.ui._activate("START_RUN")
        self.ui.render(None, {})
        exit_button = next(b for b in self.ui.buttons if b[0] == "EXIT_RUN")
        self.assertEqual(exit_button[2], self.ui.screen_h - 41)
        self.assertEqual(self.ui.preview_rect[1], 7)

    def test_all_run_pages_update_live_pixels_and_reuse_cached_layout(self):
        for size in ((640, 480), (480, 640)):
            for task in (TASK_MATERIAL, TASK_RING, TASK_STACK):
                ui = VisionUI(*size, copy.deepcopy(DEFAULT_CONFIG), "loaded")
                ui._open_task(task)
                ui._activate("START_RING_RUN" if task == TASK_RING else "START_RUN")
                dark = ui.render(np.zeros((240, 320, 3), np.uint8), empty_result(), 30, 8).array
                cache, buttons = ui.static_cache, list(ui.buttons)
                bright = ui.render(np.full((240, 320, 3), 255, np.uint8), empty_result(), 32, 9).array
                self.assertIs(ui.static_cache, cache)
                self.assertEqual(ui.buttons, buttons)
                self.assertEqual(ui.preview_rect[1], 7)
                px, py = ui._img_to_screen(80, 170)
                self.assertFalse(np.array_equal(dark[py, px], bright[py, px]))
                title = next(t for t in ui.text_ops if t[4] == "title")
                self.assertGreaterEqual(title[1], size[1] - 48)
                for name, x, y, w, h in ui.buttons:
                    self.assertTrue(0 <= y and y + h <= size[1])
                    self.assertEqual(ui._button_at(x + w // 2, y + h // 2), name)


if __name__ == "__main__":
    unittest.main()
