"""B2搜索裁剪、夹爪屏蔽及原图参考点保持。"""
import copy
import tempfile
import unittest
from unittest.mock import patch

import cv2
import numpy as np

from test_vision_v9 import config, midpoint_bgr
from test_ui_v9 import VisionUI
import vision_blob
from vision_blob import clear_exclusions, empty_result, make_preview, make_run_geometry
from vision_config import COLOR_ORDER, DEFAULT_CONFIG, VisionConfigStore, validate_config
from vision_tasks import TASK_MATERIAL, TASK_STACK, TaskManager
from vision_ui import PAGE_CENTER


class ExclusionTests(unittest.TestCase):
    def test_six_colors_exclude_larger_claw_and_select_material_without_changing_image(self):
        for color in COLOR_ORDER:
            cfg = config()
            cfg["target_color"] = color
            cfg["exclude_rois"] = [dict(x=28, y=140, w=98, h=80),
                                   dict(x=210, y=150, w=108, h=70)]
            image = np.full((240, 320, 3), 255, np.uint8)
            bgr = midpoint_bgr(cfg, color)
            image[150:210, 45:115] = bgr
            image[155:210, 220:290] = bgr
            image[70:120, 150:200] = bgr
            original = image.copy()
            manager = TaskManager()
            manager.switch(TASK_MATERIAL, 1)
            for run in (False, True):
                frame = manager.process(image, cfg, run, True)
                result = frame["result"]
                self.assertTrue(result["found"], (color, run))
                self.assertAlmostEqual(result["cx"], 174, delta=2)
                self.assertAlmostEqual(result["cy"], 94, delta=2)
                self.assertEqual(result["cx"] - result["dx"], 177)
                self.assertEqual(result["cy"] - result["dy"], 92)
                self.assertEqual(np.count_nonzero(frame["mask_roi"][160:200, 50:100]), 0)
                np.testing.assert_array_equal(image, original)

    def test_roi_reduces_conversion_size_and_outside_material_is_not_selected(self):
        cfg = config()
        cfg["roi"] = dict(x=60, y=0, w=190, h=210)
        image = np.full((240, 320, 3), 255, np.uint8)
        image[70:120, 260:310] = midpoint_bgr(cfg, "red")
        manager = TaskManager()
        manager.switch(TASK_MATERIAL, 1)
        for run in (False, True):
            with patch("vision_blob.convert_color_space", wraps=vision_blob.convert_color_space) as convert:
                frame = manager.process(image, cfg, run, False)
            expected_shape = (round(210 * .75), round(190 * .75)) if run else (210, 190)
            self.assertEqual(convert.call_args.args[0].shape[:2], expected_shape)
            self.assertFalse(frame["result"]["found"])
            self.assertEqual(frame["roi_rect"], (60, 0, 190, 210))

    def test_reference_outside_search_roi_is_not_clamped_or_shifted(self):
        cfg = config()
        cfg["roi"] = dict(x=10, y=10, w=80, h=80)
        image = np.full((240, 320, 3), 255, np.uint8)
        image[30:70, 30:70] = midpoint_bgr(cfg, "red")
        manager = TaskManager()
        manager.switch(TASK_MATERIAL, 1)
        for run in (False, True):
            result = manager.process(image, cfg, run, False)["result"]
            self.assertTrue(result["found"])
            self.assertAlmostEqual(result["cx"], 49, delta=2)
            self.assertEqual(result["cx"] - result["dx"], 177)
            self.assertEqual(result["cy"] - result["dy"], 92)

        _, compatible = vision_blob.detect_largest_blob(image, cfg, False)
        self.assertEqual(compatible["cx"] - compatible["dx"], 177)
        self.assertEqual(compatible["cy"] - compatible["dy"], 92)

    def test_scaled_exclusions_exactly_match_nearest_neighbor_source_sampling(self):
        for roi in (dict(x=60, y=13, w=190, h=201), dict(x=91, y=61, w=135, h=135)):
            cfg = config()
            cfg["roi"] = roi
            cfg["exclude_rois"] = [dict(x=0, y=0, w=320, h=17),
                                   dict(x=103, y=53, w=41, h=67),
                                   dict(x=220, y=195, w=100, h=45)]
            geom = make_run_geometry(cfg, 320, 240, 240, 180)
            x, y, w, h = (roi[k] for k in ("x", "y", "w", "h"))
            source = np.full((h, w), 255, np.uint8)
            clear_exclusions(source, (x, y, w, h), cfg["exclude_rois"])
            _, _, pw, ph = geom["process_roi"]
            expected = cv2.resize(source, (pw, ph), interpolation=cv2.INTER_NEAREST)
            actual = np.full((ph, pw), 255, np.uint8)
            clear_exclusions(actual, (0, 0, pw, ph), geom["config"]["exclude_rois"])
            np.testing.assert_array_equal(actual, expected)

    def test_morphology_cannot_refill_masked_pixels(self):
        cfg = config()
        cfg["exclude_rois"] = [dict(x=160, y=20, w=1, h=100)]
        image = np.full((240, 320, 3), 255, np.uint8)
        image[30:110, 140:181] = midpoint_bgr(cfg, "red")
        manager = TaskManager()
        manager.switch(TASK_MATERIAL, 1)
        for mode in ("NONE", "OPEN_ONLY", "CLOSE_ONLY", "OPEN_CLOSE"):
            cfg["morph_mode"] = mode
            frame = manager.process(image, cfg, False, True)
            self.assertEqual(np.count_nonzero(frame["mask_roi"][20:120, 160]), 0)

    def test_partial_search_roi_does_not_shrink_grasp_denominator(self):
        cfg = config()
        cfg["roi"] = dict(x=170, y=70, w=20, h=60)
        image = np.full((240, 320, 3), 255, np.uint8)
        image[65:140, 150:205] = midpoint_bgr(cfg, "red")
        manager = TaskManager()
        manager.switch(TASK_MATERIAL, 1)
        for run in (False, True):
            result = manager.process(image, cfg, run, True)["result"]
            self.assertTrue(result["found"])
            self.assertEqual(result["ok"], 1)
            self.assertLessEqual(result["coverage_ratio"], .51)
            self.assertGreater(result["coverage_total"], result["coverage_pixels"])

    def test_runtime_exclusion_change_invalidates_geometry_and_old_state(self):
        cfg = config()
        image = np.full((240, 320, 3), 255, np.uint8)
        image[70:120, 150:200] = midpoint_bgr(cfg, "red")
        manager = TaskManager()
        manager.switch(TASK_MATERIAL, 1)
        self.assertTrue(manager.process(image, cfg, True)["result"]["found"])
        cfg["exclude_rois"] = [dict(x=0, y=0, w=320, h=240)]
        result = manager.process(image, cfg, True)["result"]
        self.assertEqual((result["ok"], result["dx"], result["dy"], result["coverage_ratio"]), (0, 0, 0, 0))

    def test_b4_does_not_inherit_b2_exclusions_or_outer_roi(self):
        cfg = config()
        cfg["roi"] = dict(x=0, y=0, w=20, h=20)
        cfg["exclude_rois"] = [dict(x=0, y=0, w=320, h=240)]
        image = np.full((240, 320, 3), 255, np.uint8)
        image[70:120, 150:200] = midpoint_bgr(cfg, "red")
        manager = TaskManager()
        manager.switch(TASK_STACK, 1)
        result = manager.process(image, cfg, True)["result"]
        self.assertTrue(result["found"])
        self.assertEqual(result["ok"], 1)

    def test_preview_only_shades_excluded_regions_and_does_not_modify_input(self):
        cfg = config()
        cfg["roi"] = dict(x=60, y=20, w=190, h=190)
        cfg["exclude_rois"] = [dict(x=80, y=140, w=30, h=40)]
        image = np.full((240, 320, 3), 255, np.uint8)
        with patch('vision_blob.cv2.drawMarker') as marker:
            preview = make_preview(image, None, empty_result(), cfg, "RAW")
        self.assertEqual(marker.call_args.args[1], (177, 92))
        self.assertEqual(tuple(preview[10, 10]), (85, 85, 85))
        self.assertEqual(tuple(preview[150, 90]), (85, 85, 85))
        self.assertEqual(tuple(preview[60, 130]), (255, 255, 255))
        self.assertTrue(np.all(image == 255))


class RoiConfigTests(unittest.TestCase):
    def test_crop_and_masks_round_trip_with_absolute_reference(self):
        cfg = config()
        cfg["roi"] = dict(x=60, y=20, w=190, h=190)
        cfg["exclude_rois"] = [dict(x=80, y=140, w=30, h=40)]
        original = copy.deepcopy(cfg)
        for _ in range(3):
            cfg = validate_config(cfg)
            self.assertEqual(cfg, original)
        with tempfile.TemporaryDirectory() as path:
            store = VisionConfigStore(path)
            self.assertEqual(store.save(cfg)[0], original)
            self.assertEqual(store.load()[0], original)

    def test_invalid_masks_clip_to_image_or_are_dropped(self):
        cfg = config()
        cfg["exclude_rois"] = [dict(x=-10, y=-10, w=30, h=40),
                               dict(x=310, y=230, w=50, h=50),
                               dict(x=500, y=500, w=10, h=10),
                               dict(x=0, y=0, w=0, h=0), None, "bad"]
        value = validate_config(cfg)
        self.assertEqual(value["exclude_rois"], [dict(x=0, y=0, w=20, h=30), dict(x=310, y=230, w=10, h=10)])
        cfg["exclude_rois"] = []
        self.assertEqual(validate_config(cfg)["exclude_rois"], [])

    def test_v10_migration_only_adds_masks_preserves_existing_parameters(self):
        old = config()
        old.pop("exclude_rois")
        old["schema_version"] = 10
        value = validate_config(old)
        for k in old.keys() - {"schema_version"}:
            self.assertEqual(value[k], old[k])
        self.assertEqual(value["exclude_rois"], validate_config(DEFAULT_CONFIG)["exclude_rois"])

    def test_b2_center_editor_uses_absolute_coordinates_after_crop(self):
        cfg = config()
        cfg["roi"] = dict(x=60, y=20, w=190, h=190)
        ui = VisionUI(552, 368, cfg, "loaded")
        ui._open_task(TASK_MATERIAL)
        ui._begin_editor(PAGE_CENTER)
        ui.render(np.zeros((240, 320, 3), np.uint8), empty_result())
        with patch('vision_ui.cv2.drawMarker') as marker:
            ui._draw_center_marker(np.zeros((368, 552, 3), np.uint8))
        self.assertEqual(marker.call_args.args[1], ui._img_to_screen(177, 92))
        ui.center_selected = True
        ui._handle_center_tap(*ui._img_to_screen(190, 100))
        self.assertAlmostEqual(ui.draft_config["center"]["x"], 190, delta=1)
        self.assertAlmostEqual(ui.draft_config["center"]["y"], 100, delta=1)
        ui._activate("CLEAR_CENTER")
        self.assertEqual(validate_config(ui.draft_config)["center"], dict(x=160, y=120, custom=False))


if __name__ == "__main__":
    unittest.main()
