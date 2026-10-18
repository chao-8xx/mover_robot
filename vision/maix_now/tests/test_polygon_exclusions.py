"""B2多边形/弧带屏蔽：形状、黑色、缩放、缓存与持久化。"""
import copy
import tempfile
import unittest
from unittest.mock import patch

import cv2
import numpy as np

from test_vision_v9 import config, midpoint_bgr
from vision_config import DEFAULT_CONFIG, SCHEMA_VERSION, VisionConfigStore, validate_config
from vision_blob import (empty_result, make_preview, make_run_geometry,
                         polygon_exclusion_mask)
from vision_tasks import TASK_MATERIAL, TASK_STACK, TASK_RING, TaskManager


class PolygonTests(unittest.TestCase):
    def test_six_colors_remove_triangle_but_keep_target_in_its_bounding_box(self):
        triangle = [[80, 40], [140, 40], [80, 120]]
        for color in DEFAULT_CONFIG["thresholds"]["LAB"]:
            cfg = config()
            cfg["target_color"] = color
            cfg["blob"] = dict(min_area=5, max_area=20000)
            cfg["morph_mode"] = "NONE"
            cfg["exclude_polygons"] = [triangle]
            image = np.full((240, 320, 3), 255, np.uint8)
            cv2.fillPoly(image, [np.array(triangle, np.int32)], midpoint_bgr(cfg, color))
            image[92:116, 118:142] = midpoint_bgr(cfg, color)
            original = image.copy()
            manager = TaskManager()
            manager.switch(TASK_MATERIAL, 1)
            for run in (False, True):
                result = manager.process(image, cfg, run, False)["result"]
                self.assertTrue(result["found"], (color, run))
                self.assertAlmostEqual(result["cx"], 130, delta=2)
                self.assertAlmostEqual(result["cy"], 104, delta=2)
                self.assertEqual(result["cx"] - result["dx"], cfg["center"]["x"])
            np.testing.assert_array_equal(image, original)

    def test_concave_arc_preserves_the_inside_and_overlap_is_union(self):
        # 凹多边形模拟弧带，凹口不能按外接框被一起屏蔽。
        arc = [[20, 20], [80, 20], [80, 80], [60, 80], [60, 40], [20, 40]]
        other = [[60, 20], [100, 20], [100, 60], [60, 60]]
        mask = polygon_exclusion_mask((0, 0, 320, 240), [arc, other])
        self.assertTrue(mask[30, 30])
        self.assertFalse(mask[60, 40])
        self.assertTrue(mask[30, 70])  # 重叠区域不能变回未屏蔽。
        self.assertTrue(mask[50, 90])

    def test_run_mask_equals_source_nearest_sampling_for_cropped_edge_polygons(self):
        polygons = [[[-20, -10], [110, 40], [80, 150], [20, 190]],
                    [[290, 200], [350, 210], [330, 270], [270, 230]]]
        source = np.zeros((240, 320), np.uint8)
        for polygon in polygons:
            cv2.fillPoly(source, [np.array(polygon, np.int32)], 255)
        for roi in (dict(x=0, y=0, w=320, h=240), dict(x=65, y=30, w=220, h=200),
                    dict(x=91, y=61, w=135, h=135)):
            cfg = config()
            cfg["roi"], cfg["exclude_polygons"] = roi, polygons
            geometry = make_run_geometry(cfg, 320, 240, 240, 180)
            _, _, pw, ph = geometry["process_roi"]
            x, y, w, h = (roi[k] for k in ("x", "y", "w", "h"))
            expected = cv2.resize(source[y:y+h, x:x+w], (pw, ph), interpolation=cv2.INTER_NEAREST)
            np.testing.assert_array_equal(geometry["config"]["_exclude_polygon_mask"], expected != 0)

    def test_static_masks_are_cached_and_parameter_change_rebuilds_run_geometry(self):
        cfg = config()
        cfg["exclude_polygons"] = [[[100, 50], [130, 50], [110, 70]]]
        first = polygon_exclusion_mask((0, 0, 320, 240), cfg["exclude_polygons"])
        with patch("vision_blob.cv2.fillPoly", side_effect=AssertionError("redrawn every frame")):
            self.assertIs(polygon_exclusion_mask((0, 0, 320, 240), cfg["exclude_polygons"]), first)
        image = np.full((240, 320, 3), 255, np.uint8)
        image[70:120, 150:200] = midpoint_bgr(cfg, "red")
        manager = TaskManager()
        manager.switch(TASK_MATERIAL, 1)
        self.assertTrue(manager.process(image, cfg, True)["result"]["found"])
        geometry = manager.color_tracker.run_geometry
        cfg["exclude_polygons"] = [[[0, 0], [319, 0], [319, 239], [0, 239]]]
        result = manager.process(image, cfg, True)["result"]
        self.assertIsNot(manager.color_tracker.run_geometry, geometry)
        self.assertEqual((result["ok"], result["dx"], result["coverage_ratio"]), (0, 0, 0))
        cfg["exclude_polygons"] = []
        self.assertTrue(manager.process(image, cfg, True)["result"]["found"])

    def test_morphology_and_rectangles_cannot_refill_polygon(self):
        cfg = config()
        cfg["blob"] = dict(min_area=5, max_area=20000)
        cfg["exclude_rois"] = [dict(x=150, y=30, w=3, h=100)]
        cfg["exclude_polygons"] = [[[160, 30], [161, 30], [161, 130], [160, 130]]]
        image = np.full((240, 320, 3), 255, np.uint8)
        image[40:120, 140:181] = midpoint_bgr(cfg, "red")
        manager = TaskManager()
        manager.switch(TASK_MATERIAL, 1)
        for mode in ("NONE", "OPEN_ONLY", "CLOSE_ONLY", "OPEN_CLOSE"):
            cfg["morph_mode"] = mode
            for run in (False, True):
                frame = manager.process(image, cfg, run, True)
                self.assertFalse(np.any(frame["mask_roi"][40:120, 160]))
                self.assertFalse(np.any(frame["mask_roi"][40:120, 151]))

    def test_masked_grasp_pixels_do_not_shrink_denominator(self):
        cfg = config()
        cfg["morph_mode"] = "NONE"
        cfg["grasp"] = dict(roi=dict(x=100, y=80, w=20, h=20), min_coverage_pct=60)
        cfg["exclude_polygons"] = [[[100, 80], [109, 80], [109, 99], [100, 99]]]
        image = np.full((240, 320, 3), 255, np.uint8)
        image[60:130, 80:140] = midpoint_bgr(cfg, "red")
        manager = TaskManager()
        manager.switch(TASK_MATERIAL, 1)
        for run, total in ((False, 400), (True, 225)):
            result = manager.process(image, cfg, run, True)["result"]
            self.assertEqual(result["coverage_total"], total)
            self.assertLess(result["coverage_ratio"], .6)
            self.assertEqual(result["ok"], 1)

    def test_preview_is_shaded_with_outline_without_modifying_input(self):
        cfg = config()
        cfg["exclude_polygons"] = [[[20, 20], [80, 20], [20, 80]]]
        image = np.full((240, 320, 3), 255, np.uint8)
        preview = make_preview(image, None, empty_result(), cfg, "RAW")
        self.assertEqual(tuple(preview[30, 30]), (85, 85, 85))
        self.assertEqual(tuple(preview[60, 60]), (255, 255, 255))
        self.assertTrue(np.all(image == 255))

    def test_b3_b4_do_not_inherit_b2_polygons(self):
        cfg = config()
        cfg["exclude_polygons"] = [[[0, 0], [319, 0], [319, 239], [0, 239]]]
        image = np.full((240, 320, 3), 255, np.uint8)
        image[70:120, 150:200] = midpoint_bgr(cfg, "red")
        manager = TaskManager()
        manager.switch(TASK_STACK, 1)
        frame = manager.process(image, cfg, True)
        self.assertNotIn("exclude_polygons", frame["config"])
        self.assertTrue(frame["result"]["found"])
        manager.switch(TASK_RING, 2)
        with patch.object(manager.current, "process", return_value={"ring": True}) as process:
            self.assertEqual(manager.process(image, cfg, True), {"ring": True})
            self.assertIs(process.call_args.args[1], cfg)


class PolygonConfigTests(unittest.TestCase):
    def test_old_json_adds_polygons_only_and_round_trips(self):
        old = config()
        old["schema_version"] = 11
        old.pop("exclude_polygons")
        value = validate_config(old)
        for key in old.keys() - {"schema_version"}:
            self.assertEqual(value[key], old[key])
        self.assertEqual(value["schema_version"], SCHEMA_VERSION)
        self.assertEqual(value["exclude_polygons"], DEFAULT_CONFIG["exclude_polygons"])
        value["exclude_polygons"] = [[[1, 1], [40, 1], [20, 30]]]
        with tempfile.TemporaryDirectory() as path:
            store = VisionConfigStore(path)
            self.assertEqual(store.save(value)[0], value)
            self.assertEqual(store.load()[0], value)

    def test_invalid_and_degenerate_polygons_are_dropped_and_empty_disables(self):
        cfg = config()
        valid = [[-10, 0], [20, 0], [20, 30], [-10, 30]]
        cfg["exclude_polygons"] = [None, "bad", [], [[1, 1], [2, 2]],
            [[1, 1], [2, 2], [3, 3]], [[1, 1], [2, 2], [None, 3]],
            [[1, 1], [2, 2], [3, float("nan")]], valid]
        self.assertEqual(validate_config(cfg)["exclude_polygons"], [valid])
        cfg["exclude_polygons"] = []
        self.assertEqual(validate_config(cfg)["exclude_polygons"], [])
        self.assertIsNone(polygon_exclusion_mask((0, 0, 320, 240), []))


if __name__ == "__main__":
    unittest.main()
