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
from vision_config import DEFAULT_CONFIG  # noqa: E402
from vision_tasks import TASK_MATERIAL, TASK_RING, TASK_STACK  # noqa: E402
from vision_ui import (  # noqa: E402
    PAGE_CENTER,
    PAGE_HOME,
    PAGE_RING_HOME,
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
        self.assertIsNotNone(self.ui.preview_rect)

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

    def test_b2_color_page_still_runs(self):
        self.ui._open_task(TASK_MATERIAL)
        self.assertEqual(self.ui.page, PAGE_HOME)
        self.ui.render(self.frame, empty_result())
        self.ui._activate("START_RUN")
        self.assertTrue(self.ui.is_run_mode())
        self.ui.render(self.frame, empty_result())
        self.assertIsNotNone(self.ui.preview_rect)


if __name__ == "__main__":
    unittest.main()
