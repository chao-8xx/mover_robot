"""诊断只在采样帧统计，不改变检测、坐标或串口状态。"""
import copy
import tempfile
import unittest
from unittest.mock import patch

import cv2
import numpy as np

from test_vision_v9 import config, midpoint_bgr
from vision_tasks import TASK_MATERIAL, TASK_STACK, TaskManager
from vision_config import DEFAULT_CONFIG, _LEGACY_JOINED_POLYGON, VisionConfigStore, validate_config
from vision_runtime import color_diagnostic_line


class DiagnosticTests(unittest.TestCase):
    def scene(self):
        cfg = config()
        cfg["target_color"] = "black"
        cfg["morph_mode"] = "NONE"
        cfg["blob"] = dict(min_area=100, max_area=1000)
        frame = np.full((240, 320, 3), 255, np.uint8)
        bgr = midpoint_bgr(cfg, "black")
        frame[10:60, 10:60] = bgr     # 过大
        frame[100:120, 100:120] = bgr # 合格
        frame[140:145, 140:145] = bgr # 过小
        return cfg, frame

    def test_candidates_and_limits_stay_in_processing_units_in_ui_and_run(self):
        cfg, image = self.scene()
        manager = TaskManager()
        manager.switch(TASK_MATERIAL, 5)
        for run in (False, True):
            plain = manager.process(image, cfg, run, False)["result"]
            detailed = manager.process(image, cfg, run, False, diagnostic=True)["result"]
            for key in ("found", "ok", "cx", "cy", "dx", "dy", "area", "coverage_pixels"):
                self.assertEqual(plain[key], detailed[key])
            info = detailed["diagnostic"]
            self.assertEqual((info["contours"], info["small"], info["large"], info["eligible"]), (3, 1, 1, 1))
            self.assertEqual(info["raw_pixels"], info["kept_pixels"])
            self.assertEqual(info["kept_pixels"], info["clean_pixels"])
            self.assertEqual(info["size"], (240, 180) if run else (320, 240))
            self.assertNotIn("area_scale", info)
            self.assertEqual(info["min_area"], 56 if run else 100)
            self.assertEqual(info["max_area"], 562 if run else 1000)
            line = color_diagnostic_line(detailed, cfg, run)
            for value in ("WHY=SELECTED", "SMALL=1", "LARGE=1", "PASS=1", ":LARGE", ":PASS", ":SMALL"):
                self.assertIn(value, line)
            self.assertIn("TOP_AREA_PROC=[{:.1f}:LARGE".format(info["top"][0]), line)
            self.assertIn("LIMIT_PROC=56..562" if run else "LIMIT_PROC=100..1000", line)
            self.assertNotIn("_SRC", line)

    def test_threshold_mask_morphology_and_area_failures_are_distinguished(self):
        for reason in ("THRESHOLD_EMPTY", "MASK_EMPTY", "MORPH_EMPTY", "AREA_REJECTED"):
            cfg, image = self.scene()
            if reason == "THRESHOLD_EMPTY": image[:] = 255
            elif reason == "MASK_EMPTY": cfg["exclude_rois"] = [dict(x=0, y=0, w=320, h=240)]
            elif reason == "MORPH_EMPTY":
                image[:] = 255
                image[100, 100] = midpoint_bgr(cfg, "black")
                cfg["morph_mode"] = "OPEN_ONLY"
            else: cfg["blob"] = dict(min_area=10000, max_area=20000)
            manager = TaskManager()
            manager.switch(TASK_MATERIAL, 5)
            result = manager.process(image, cfg, False, False, diagnostic=True)["result"]
            self.assertFalse(result["found"])
            self.assertIn("WHY=" + reason, color_diagnostic_line(result, cfg))

    def test_sampling_disabled_does_not_add_pixel_counts_or_repeat_color_conversion(self):
        cfg, image = self.scene()
        image[:] = 255
        manager = TaskManager()
        manager.switch(TASK_MATERIAL, 5)
        with patch("vision_blob.cv2.countNonZero", side_effect=AssertionError("unsampled count")):
            result = manager.process(image, cfg, False, False)["result"]
        self.assertNotIn("diagnostic", result)
        self.assertIsNone(color_diagnostic_line(result, cfg))
        with patch("vision_blob.convert_color_space", wraps=__import__("vision_blob").convert_color_space) as convert:
            manager.process(image, cfg, True, False, diagnostic=True)
        self.assertEqual(convert.call_count, 1)

    def test_b4_does_not_enable_b2_diagnostics(self):
        cfg, image = self.scene()
        manager = TaskManager()
        manager.switch(TASK_STACK, 5)
        self.assertNotIn("diagnostic", manager.process(image, cfg, True, False, diagnostic=True)["result"])

    def test_known_old_joined_polygon_migrates_without_overwriting_other_fields(self):
        old = config()
        old["schema_version"] = 12
        old["exclude_polygons"] = [copy.deepcopy(_LEGACY_JOINED_POLYGON)]
        value = validate_config(old)
        self.assertEqual(value["exclude_polygons"], DEFAULT_CONFIG["exclude_polygons"])
        self.assertEqual(len(value["exclude_polygons"]), 4)
        for key in old.keys() - {"schema_version", "exclude_polygons"}:
            self.assertEqual(value[key], old[key])
        for polygon in value["exclude_polygons"]:
            self.assertEqual(len(set(map(tuple, polygon))), len(polygon))
        for custom in ([], [[[1, 1], [10, 1], [1, 10]]]):
            old["exclude_polygons"] = custom
            self.assertEqual(validate_config(old)["exclude_polygons"], custom)

    def test_legacy_config_and_diagnostic_switch_round_trip(self):
        old = config()
        old["schema_version"] = 12
        old.pop("color_diagnostics")
        value = validate_config(old)
        self.assertTrue(value["color_diagnostics"])
        value["color_diagnostics"] = False
        value["exclude_polygons"] = [[[65, 30], [90, 30], [65, 60]]]
        with tempfile.TemporaryDirectory() as path:
            store = VisionConfigStore(path)
            saved, _ = store.save(value)
            self.assertEqual(saved, value)
            loaded, _ = store.load()
            self.assertEqual(loaded, value)


if __name__ == "__main__":
    unittest.main()
