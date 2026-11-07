"""NOW版本电脑回归：合成图算法、配置、调度及持续串口解析。"""
import ast
import copy
import json
import struct
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

import cv2
import numpy as np

MODULE_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(MODULE_DIR))
import vision_blob
from vision_config import COLOR_ORDER, DEFAULT_CONFIG, VALID_MORPH_MODES, VisionConfigStore, validate_config
from vision_ring import detect_ring_candidates, make_ring_preview
from vision_tasks import (
    RingTask, TaskManager, TASK_MATERIAL, TASK_RING, TASK_STACK, safe_xy_result,
)
from vision_runtime import ResultSender, PerfStats, pack_xy_frame, send_interval_ms


def config():
    cfg = validate_config(copy.deepcopy(DEFAULT_CONFIG))
    # 合成图测试明确选择红色，不依赖现场默认颜色。
    cfg["target_color"] = "red"
    cfg["exclude_rois"] = []  # 基础检测测试与现场夹爪位置隔离；屏蔽单独验收。
    cfg["exclude_polygons"] = []
    # 合成场景固定几何，现场修改默认ROI/中心/抓取框不应改变测试输入。
    cfg["roi"] = dict(x=0, y=0, w=320, h=240)
    cfg["center"] = dict(x=177, y=92, custom=True)
    cfg["grasp"] = dict(roi=dict(x=157, y=72, w=40, h=40), min_coverage_pct=60)
    cfg["blob"] = dict(min_area=600, max_area=12000)
    cfg["stack"]["blob"] = dict(min_area=600, max_area=12000)
    return cfg


def midpoint_bgr(cfg, key):
    threshold = cfg["thresholds"]["LAB"][key]
    lower, upper = threshold["lower"], threshold["upper"]
    values = [(lo + hi) // 2 for lo, hi in zip(lower, upper)]
    return tuple(int(v) for v in cv2.cvtColor(
        np.uint8([[values]]), cv2.COLOR_LAB2BGR)[0, 0])


def color_frame(cfg, colors=("red", "yellow", "blue")):
    frame = np.full((240, 320, 3), 255, np.uint8)
    for key, cx in zip(("red", "yellow", "blue"), (35, 160, 285)):
        if key in colors:
            cv2.rectangle(frame, (cx - 25, 95), (cx + 25, 145),
                          midpoint_bgr(cfg, key), -1)
    return frame


def ring_frame(centers=((40, 120), (160, 120), (280, 120)), axes=None, angle=0, digits=2):
    frame = np.full((240, 320, 3), 255, np.uint8)
    for center in centers:
        # 小到单像素的圆周在斜向射线会混叠；使用可解析的环带样本。
        # 留出数字裁剪区，环带不穿过字形；仍保留4层几何确认。
        for radius in (32, 28, 24, 20):
            if axes is None:
                cv2.circle(frame, center, radius, (0, 0, 0), 2)
            else:
                cv2.ellipse(frame, center, (radius, int(radius * axes)),
                            angle, 0, 360, (0, 0, 0), 2)
        if digits is not None:
            cv2.putText(frame, str(digits), (center[0]-6, center[1]+7),
                        cv2.FONT_HERSHEY_SIMPLEX, .55, (0, 0, 0), 2, cv2.LINE_AA)
    return frame


def ring_config():
    cfg = config()
    cfg["ring"]["roi"] = dict(x=0, y=0, w=320, h=240)
    cfg["ring"]["center"] = dict(x=160, y=120, custom=True)
    cfg["ring"]["min_diameter"] = 24
    cfg["ring"]["min_contour_area"] = 45
    return cfg


class ConfigTests(unittest.TestCase):
    def previous_now_config(self):
        # 从2.1.3交付包提取的真实旧默认JSON快照，不依赖历史ZIP仍然存在。
        return json.loads((MODULE_DIR / "tests" / "fixtures" / "now_v8_config.json").read_text(encoding="utf-8"))

    def test_previous_saved_values_are_preserved_when_adding_grasp_and_save_once(self):
        old = self.previous_now_config()
        with tempfile.TemporaryDirectory() as path:
            store = VisionConfigStore(path)
            Path(store.path).write_text(json.dumps(old), encoding="utf-8")
            value, status = store.load()
            self.assertIn("迁移", status)
            self.assertEqual(value["thresholds"], old["thresholds"])
            self.assertEqual(value["blob"], old["blob"])
            self.assertEqual(value["center"], old["center"])
            self.assertEqual(value["ring"], old["ring"])
            self.assertEqual(value["stack"], old["stack"])
            saved, _ = store.save(value)
            self.assertEqual(store.load()[0], saved)
            self.assertEqual(validate_config(saved), saved)

    def test_migration_preserves_distinct_saved_custom_material_values(self):
        old = self.previous_now_config()
        old["thresholds"]["LAB"]["red"]["lower"][0] = 36
        old["blob"] = dict(min_area=900, max_area=8000)
        old["center"] = dict(x=180, y=95, custom=True)
        value = validate_config(old)
        self.assertEqual(value["thresholds"]["LAB"]["red"], old["thresholds"]["LAB"]["red"])
        self.assertEqual(value["blob"], old["blob"])
        self.assertEqual(value["center"], old["center"])
        self.assertEqual(value["ring"], old["ring"])
        self.assertEqual(value["stack"], old["stack"])

    def test_old_main_auto_roi_center_migrates_without_moving_reference(self):
        old = config()
        old["schema_version"] = 5
        old["roi"] = dict(x=110, y=25, w=135, h=135)
        old["center"] = dict(x=160, y=120, custom=False)
        value = validate_config(old)
        self.assertEqual(value["roi"], old["roi"])
        self.assertEqual(value["center"], dict(x=177, y=92, custom=True))
        self.assertEqual(validate_config(value), value)

    def test_current_schema_keeps_user_choice_of_auto_center_and_previous_threshold(self):
        cfg = config()
        cfg["center"] = dict(x=160, y=120, custom=False)
        cfg["blob"] = dict(min_area=1800, max_area=12000)
        cfg["thresholds"]["LAB"]["red"] = dict(lower=[35, 160, 150], upper=[110, 185, 170])
        value = validate_config(cfg)
        self.assertEqual(value, cfg)

    def test_missing_area_fields_use_single_default_config(self):
        value = validate_config({})
        self.assertEqual(value["blob"], DEFAULT_CONFIG["blob"])
        self.assertEqual(value["stack"]["blob"], DEFAULT_CONFIG["stack"]["blob"])
        self.assertEqual(value["stack"]["roi"], DEFAULT_CONFIG["stack"]["roi"])

    def test_old_small_b2_roi_preserves_search_and_migrates_absolute_center(self):
        cfg = config()
        cfg["schema_version"] = 10
        cfg["roi"] = {"x": 110, "y": 25, "w": 135, "h": 135}
        cfg["center"] = {"x": 60, "y": 70, "custom": True}
        original_thresholds = copy.deepcopy(cfg["thresholds"])
        migrated = validate_config(cfg)
        self.assertEqual(migrated["roi"], cfg["roi"])
        self.assertEqual(migrated["center"], dict(x=170, y=95, custom=True))
        self.assertEqual(validate_config(migrated), migrated)
        self.assertEqual(migrated["thresholds"], original_thresholds)

    def test_default_store_returns_normalized_stack_center(self):
        with tempfile.TemporaryDirectory() as path:
            value, _ = VisionConfigStore(path).load()
        self.assertEqual(value["stack"]["center"], validate_config(DEFAULT_CONFIG)["stack"]["center"])

    def test_save_load_preserves_calibration_and_separate_ring_geometry(self):
        cfg = config()
        cfg["ring"]["roi"] = dict(x=100, y=60, w=120, h=120)
        cfg["ring"]["center"] = dict(x=59, y=58, custom=True)
        cfg["stack"]["roi"] = dict(x=110, y=25, w=135, h=135)
        cfg["stack"]["center"] = dict(x=66, y=70, custom=True)
        cfg["stack"]["blob"] = dict(min_area=123, max_area=9000)
        for i, key in enumerate(COLOR_ORDER):
            cfg["thresholds"]["LAB"][key]["lower"][0] += i + 1
        with tempfile.TemporaryDirectory() as path:
            store = VisionConfigStore(path, "vision_now_config.json")
            saved, _ = store.save(cfg)
            loaded, _ = store.load()
            self.assertEqual(loaded, saved)
        self.assertEqual(loaded["blob"], cfg["blob"])
        self.assertEqual(loaded["ring"]["center"], dict(x=59, y=58, custom=True))
        self.assertEqual(loaded["stack"], cfg["stack"])
        self.assertEqual(loaded["thresholds"], cfg["thresholds"])

    def test_legacy_hsv_migrates_only_explicit_lab_and_preserves_calibration(self):
        cfg = config()
        cfg["mode"] = "HSV"
        cfg["schema_version"] = 6
        cfg["thresholds"]["HSV"] = {
            key: dict(lower=[160, 80, 50], upper=[10, 255, 255])
            for key in ("red", "yellow", "blue")}
        cfg["thresholds"]["LAB"]["red"]["lower"][0] = 36
        with tempfile.TemporaryDirectory() as path:
            store = VisionConfigStore(path)
            Path(store.path).write_text(json.dumps(cfg), encoding="utf-8")
            loaded, status = store.load()
            self.assertIn("迁移LAB", status)
            self.assertNotIn("mode", loaded)
            self.assertEqual(set(loaded["thresholds"]), {"LAB"})
            self.assertEqual(loaded["thresholds"]["LAB"], cfg["thresholds"]["LAB"])
            self.assertEqual(loaded["stack"], cfg["stack"])
            self.assertEqual(loaded["ring"], cfg["ring"])
            saved, _ = store.save(loaded)
            self.assertEqual(store.load()[0], saved)
        del cfg["thresholds"]["LAB"]
        fallback = validate_config(cfg)
        self.assertEqual(fallback["thresholds"], DEFAULT_CONFIG["thresholds"])

    def test_config_preserves_six_lab_colors_and_orders_limits(self):
        cfg = config()
        cfg["target_color"] = "green"
        cfg["thresholds"]["LAB"]["purple"] = dict(lower=[0]*3, upper=[255]*3)
        cfg["thresholds"]["LAB"]["green"] = dict(lower=[0]*3, upper=[255]*3)
        cfg["thresholds"]["LAB"]["red"] = dict(lower=[300, 170, 160], upper=[-1, 160, 150])
        result = validate_config(cfg)
        self.assertEqual(result["target_color"], "green")
        self.assertEqual(set(result["thresholds"]["LAB"]), set(COLOR_ORDER))
        self.assertEqual(result["thresholds"]["LAB"]["green"], cfg["thresholds"]["LAB"]["green"])
        self.assertEqual(result["thresholds"]["LAB"]["red"],
                         dict(lower=[0, 160, 150], upper=[255, 170, 160]))


class ColorTests(unittest.TestCase):
    def test_main_red_threshold_and_lower_area_admit_small_target_in_full_frame(self):
        cfg = config()
        sample = cv2.cvtColor(np.uint8([[[120, 172, 145]]]), cv2.COLOR_LAB2BGR)[0, 0]
        frame = np.full((240, 320, 3), 255, np.uint8)
        cv2.rectangle(frame, (25, 165), (75, 195), tuple(map(int, sample)), -1)
        for run in (False, True):
            manager = TaskManager()
            manager.switch(TASK_MATERIAL, 1)
            result = manager.process(frame, cfg, run, False)["result"]
            self.assertTrue(result["found"])
            self.assertGreaterEqual(result["area"], 1000)
            self.assertLess(result["area"], 1800)
            self.assertAlmostEqual(result["cx"], 50, delta=2)
            self.assertAlmostEqual(result["cy"], 180, delta=2)
            self.assertEqual(result["cx"] - result["dx"], 177)
            self.assertEqual(result["cy"] - result["dy"], 92)
            old_cfg = copy.deepcopy(cfg)
            old_cfg["blob"]["min_area"] = 1800
            self.assertFalse(manager.process(frame, old_cfg, run, False)["result"]["found"])

    def test_b2_and_b4_each_of_six_colors_still_processes_only_selected_color(self):
        for task in (TASK_MATERIAL, TASK_STACK):
            for key in COLOR_ORDER:
                cfg = config()
                cfg["target_color"] = key
                frame = np.full((240, 320, 3), 255, np.uint8)
                cv2.rectangle(frame, (135, 95), (185, 145), midpoint_bgr(cfg, key), -1)
                manager = TaskManager()
                manager.switch(task, key)
                for run in (False, True):
                    with patch.object(vision_blob, "apply_color_threshold", wraps=vision_blob.apply_color_threshold) as threshold:
                        result = manager.process(frame, cfg, run, False)["result"]
                    self.assertTrue(result["found"], (task, key, run))
                    self.assertEqual(result["color_count"], 1)
                    self.assertEqual(result["target_color"], key)
                    self.assertEqual(threshold.call_count, 1)

    def test_three_color_config_is_extended_without_overwriting_saved_lab_values(self):
        cfg = config()
        cfg["schema_version"] = 7
        saved = {key: cfg["thresholds"]["LAB"][key] for key in ("red", "yellow", "blue")}
        saved["red"]["lower"][0] = 36
        cfg["thresholds"]["LAB"] = saved
        value = validate_config(cfg)
        for key, threshold in saved.items():
            self.assertEqual(value["thresholds"]["LAB"][key], threshold)
        for key in ("green", "black", "light_blue"):
            self.assertEqual(value["thresholds"]["LAB"][key], DEFAULT_CONFIG["thresholds"]["LAB"][key])
    def test_selected_color_is_only_mask_and_blob_in_both_sizes(self):
        for key, cx in zip(("red", "yellow", "blue"), (35, 160, 285)):
            for run in (True, False):
                with self.subTest(color=key, run=run):
                    cfg = config()
                    cfg["target_color"] = key
                    manager = TaskManager()
                    manager.switch(TASK_MATERIAL, 0xC1)
                    with patch.object(vision_blob, "convert_color_space",
                                      wraps=vision_blob.convert_color_space) as convert, \
                         patch.object(vision_blob, "apply_color_threshold",
                                      wraps=vision_blob.apply_color_threshold) as threshold, \
                         patch.object(vision_blob, "clean_mask",
                                      wraps=vision_blob.clean_mask) as morph, \
                         patch.object(vision_blob, "find_largest_blob",
                                      wraps=vision_blob.find_largest_blob) as blob:
                        value = manager.process(color_frame(cfg), cfg, run, False)
                    self.assertEqual(convert.call_count, 1)
                    self.assertEqual(threshold.call_count, 1)
                    self.assertEqual(morph.call_count, 1)
                    self.assertEqual(blob.call_count, 1)
                    self.assertEqual(threshold.call_args[0][1]["target_color"], key)
                    self.assertEqual(convert.call_args[0][0].shape[:2],
                                     (180, 240) if run else (240, 320))
                    result = value["result"]
                    self.assertEqual(result["color_count"], 1)
                    self.assertNotIn("by_color", result)
                    self.assertTrue(result["found"], (key, result))
                    self.assertAlmostEqual(result["cx"], cx, delta=2)
                    self.assertAlmostEqual(result["cy"], 120, delta=2)
                    self.assertAlmostEqual(result["dx"], cx - cfg["center"]["x"], delta=2)
                    self.assertAlmostEqual(result["dy"], 120 - cfg["center"]["y"], delta=2)
                    self.assertAlmostEqual(result["area"], 2500, delta=200)
                    self.assertIsNone(value["mask_roi"])

    def test_selected_color_controls_result_not_other_visible_colors(self):
        cfg = config()
        cfg["target_color"] = "blue"
        manager = TaskManager()
        manager.switch(TASK_MATERIAL, 0xC3)
        result = manager.process(color_frame(cfg, ("red", "yellow")), cfg, True)["result"]
        self.assertFalse(result["found"])
        self.assertEqual(safe_xy_result(result), (False, 0, 0))
        result = manager.process(color_frame(cfg), cfg, True)["result"]
        self.assertTrue(result["found"])
        self.assertAlmostEqual(result["dx"], 285 - cfg["center"]["x"], delta=2)

    def test_b2_respects_configured_search_roi(self):
        cfg = config()
        cfg["roi"] = dict(x=100, y=60, w=120, h=120)
        manager = TaskManager()
        manager.switch(TASK_MATERIAL, 1)
        value = manager.process(color_frame(cfg), cfg, True)
        self.assertEqual(value["roi_rect"], (100, 60, 120, 120))
        self.assertFalse(value["result"]["found"])
        cfg["target_color"] = "yellow"
        value = manager.process(color_frame(cfg), cfg, True)
        self.assertTrue(value["result"]["found"])
        self.assertAlmostEqual(value["result"]["cx"], 160, delta=2)
        self.assertAlmostEqual(value["result"]["dx"], 160 - cfg["center"]["x"], delta=2)

    def test_b4_shared_tracker_single_conversion_single_color_own_center(self):
        cfg = config()
        cfg["stack"]["roi"] = dict(x=100, y=60, w=120, h=120)
        cfg["stack"]["center"] = dict(x=70, y=60, custom=True)
        cfg["target_color"] = "yellow"
        manager = TaskManager()
        self.assertIs(manager.tasks[TASK_MATERIAL], manager.tasks[TASK_STACK])
        manager.switch(TASK_STACK, 2)
        value = manager.process(color_frame(cfg), cfg, True, True)
        self.assertEqual(value["result"]["color_count"], 1)
        self.assertNotIn("by_color", value["result"])
        self.assertAlmostEqual(value["result"]["dx"], -10, delta=2)
        self.assertEqual(value["mask_roi"].shape, (120, 120))
        self.assertIs(value["config"]["thresholds"], cfg["thresholds"])
        self.assertEqual(cfg["roi"], dict(x=0, y=0, w=320, h=240))

    def test_b2_corners_remain_visible_and_custom_reference_is_preserved(self):
        cfg = config()
        cfg["center"] = dict(x=170, y=100, custom=True)
        for cx, cy in ((25, 25), (294, 25), (25, 214), (294, 214)):
            frame = np.full((240, 320, 3), 255, np.uint8)
            cv2.rectangle(frame, (cx - 25, cy - 25), (cx + 25, cy + 25),
                          midpoint_bgr(cfg, "red"), -1)
            for run in (False, True):
                manager = TaskManager()
                manager.switch(TASK_MATERIAL, 1)
                result = manager.process(frame, cfg, run, False)["result"]
                self.assertTrue(result["found"], (cx, cy, run))
                self.assertAlmostEqual(result["cx"], cx, delta=2)
                self.assertAlmostEqual(result["cy"], cy, delta=2)
                self.assertEqual(result["cx"] - result["dx"], 170)
                self.assertEqual(result["cy"] - result["dy"], 100)

    def test_changed_threshold_invalidates_threshold_cache_without_geometry_reset(self):
        cfg = config()
        manager = TaskManager()
        manager.switch(TASK_MATERIAL, 1)
        frame = color_frame(cfg)
        self.assertTrue(manager.process(frame, cfg, True)["result"]["found"])
        cfg["thresholds"]["LAB"]["red"] = dict(lower=[255, 255, 255], upper=[255, 255, 255])
        self.assertFalse(manager.process(frame, cfg, True)["result"]["found"])

    def test_area_filter_applies_in_source_units(self):
        cfg = config()
        cfg["blob"]["min_area"] = 3000
        manager = TaskManager()
        manager.switch(TASK_MATERIAL, 1)
        for run in (False, True):
            self.assertFalse(manager.process(color_frame(cfg), cfg, run)["result"]["found"])

    def test_b2_buffered_masks_and_results_match_original_for_all_morph_modes(self):
        rng = np.random.default_rng(17)
        for morph in VALID_MORPH_MODES:
            for key in ("red", "yellow", "blue"):
                cfg = config()
                cfg["target_color"], cfg["morph_mode"] = key, morph
                manager = TaskManager()
                manager.switch(TASK_MATERIAL, 1)
                image = color_frame(cfg)
                image[:80] = rng.integers(0, 256, image[:80].shape, dtype=np.uint8)
                geometry = vision_blob.make_run_geometry(cfg, 320, 240, 240, 180)
                roi = vision_blob.resize_run_roi(image, geometry)
                expected, mask, _ = vision_blob.detect_color(roi, geometry["config"], (0, 0, 240, 180))
                expected = vision_blob.map_run_result_to_source(expected, cfg, geometry)
                actual = manager.process(image, cfg, True, True)
                for name in ("found", "cx", "cy", "dx", "dy", "area", "rect"):
                    self.assertEqual(actual["result"][name], expected[name], (morph, key, name))
                np.testing.assert_array_equal(actual["mask_roi"], cv2.resize(mask, (320, 240), interpolation=cv2.INTER_NEAREST))
                self.assertTrue(actual["result"]["buffer_reuse"])

    def test_b2_reuses_destinations_and_does_not_keep_previous_valid_result(self):
        cfg = config()
        manager = TaskManager()
        manager.switch(TASK_MATERIAL, 1)
        first = manager.process(color_frame(cfg), cfg, True, False)
        buffers = manager.color_tracker.b2_buffers
        self.assertTrue(first["result"]["found"])
        with patch.object(vision_blob.cv2, "cvtColor", wraps=cv2.cvtColor) as convert, \
             patch.object(vision_blob.cv2, "resize", wraps=cv2.resize) as resize:
            second = manager.process(color_frame(cfg, ()), cfg, True, False)
        self.assertIs(manager.color_tracker.b2_buffers, buffers)
        self.assertIs(convert.call_args.kwargs["dst"], buffers.lab)
        self.assertIs(resize.call_args.kwargs["dst"], buffers.bgr)
        self.assertEqual(safe_xy_result(second["result"]), (False, 0, 0))
        self.assertTrue(first["result"]["found"])

    def test_b2_unsupported_destination_falls_back_once_for_current_image(self):
        cfg = config()
        manager = TaskManager()
        manager.switch(TASK_MATERIAL, 1)
        original = cv2.cvtColor
        def old_binding(*args, **kwargs):
            if "dst" in kwargs:
                raise TypeError("dst unavailable")
            return original(*args, **kwargs)
        image = color_frame(cfg)
        with patch.object(vision_blob.cv2, "cvtColor", side_effect=old_binding) as convert:
            first = manager.process(image, cfg, True, False)
            second = manager.process(image, cfg, True, False)
        self.assertFalse(manager.color_tracker.b2_buffers_supported)
        self.assertFalse(second["result"]["buffer_reuse"])
        self.assertTrue(first["result"]["found"])
        self.assertEqual(sum("dst" in c.kwargs for c in convert.call_args_list), 1)

    def test_b4_continues_original_path_without_b2_destinations(self):
        cfg = config()
        manager = TaskManager()
        manager.switch(TASK_MATERIAL, 1)
        manager.process(color_frame(cfg), cfg, True, False)
        manager.switch(TASK_STACK, 1)
        with patch.object(vision_blob.cv2, "cvtColor", wraps=cv2.cvtColor) as convert, \
             patch.object(vision_blob.cv2, "inRange", wraps=cv2.inRange) as threshold:
            result = manager.process(color_frame(cfg), cfg, True, False)
        self.assertTrue(all("dst" not in c.kwargs for c in convert.call_args_list + threshold.call_args_list))
        self.assertIsNone(manager.color_tracker.b2_buffers)
        self.assertNotIn("buffer_reuse", result["result"])

    def test_b2_small_noise_prefilter_preserves_contour_area_and_skips_work(self):
        cfg = config()
        mask = np.zeros((180, 240), dtype=np.uint8)
        for y in range(0, 90, 8):
            for x in range(0, 240, 8):
                mask[y:y+3, x:x+3] = 255
        cv2.rectangle(mask, (60, 105), (125, 160), 255, -1)
        expected = vision_blob.find_largest_blob(mask, cfg, (0, 0, 240, 180))
        with patch.object(vision_blob.cv2, "contourArea", wraps=cv2.contourArea) as area:
            actual = vision_blob.find_largest_blob(mask, cfg, (0, 0, 240, 180), fast_filter=True)
        self.assertEqual(actual, expected)
        self.assertEqual(area.call_count, 1)
        contours = vision_blob._find_contours(mask)
        # 同样检查旧固件tuple轮廓路径，以及面积等于min_area的边界仍被接受。
        tuples = [tuple(tuple(map(int, p[0])) for p in c) for c in contours]
        cfg["blob"]["min_area"] = expected["area"]
        with patch.object(vision_blob, "_find_contours", return_value=tuples):
            self.assertEqual(vision_blob.find_largest_blob(mask, cfg, (0, 0, 240, 180), True), expected)


class RingGeometryTests(unittest.TestCase):
    def test_three_geometric_candidates_without_digits(self):
        result, _, _ = detect_ring_candidates(ring_frame(), ring_config()["ring"])
        self.assertEqual(result["candidate_count"], 3)

    def test_small_roi_restores_absolute_center_and_relative_deviation(self):
        cfg = ring_config()
        cfg["ring"]["roi"] = dict(x=100, y=60, w=120, h=120)
        cfg["ring"]["center"] = dict(x=50, y=55, custom=True)
        result, mask, roi = detect_ring_candidates(ring_frame(), cfg["ring"])
        self.assertEqual(roi, (100, 60, 120, 120))
        self.assertEqual(mask.shape, (120, 120))
        self.assertEqual(result["candidate_count"], 1)
        ring = result["candidates"][0]
        self.assertEqual((ring["cx"], ring["cy"], ring["dx"], ring["dy"]),
                         (160, 120, 10, 5))

    def test_obvious_ellipse_rejected_even_rotated(self):
        for angle in (0, 45):
            value, _, _ = detect_ring_candidates(
                ring_frame(((160, 120),), axes=0.6, angle=angle), ring_config()["ring"])
            self.assertEqual(value["candidate_count"], 0)

    def test_solid_circle_rejected_on_full_layer_check(self):
        frame = np.full((240, 320, 3), 255, np.uint8)
        cv2.circle(frame, (160, 120), 32, (0, 0, 0), -1)
        value, _, _ = detect_ring_candidates(frame, ring_config()["ring"])
        self.assertEqual(value["candidate_count"], 0)
        self.assertGreater(value["reject_counts"]["LAYER"], 0)


class RingTrackingTests(unittest.TestCase):
    def setUp(self):
        self.cfg = ring_config()
        self.task = RingTask()
        self.task.enter(0xD2)

    def track(self):
        for _ in range(3):
            value = self.task.process(ring_frame(), self.cfg)["result"]
        self.assertEqual(value["state"], "TRACK")
        return value

    def test_digit_two_is_required_before_tracking(self):
        for _ in range(3):
            value = self.task.process(ring_frame(digits=None), self.cfg)["result"]
        self.assertFalse(value["found"])
        self.assertFalse(value["target_confirmed"])
        self.assertGreater(value["ocr_calls"], 0)
        self.task.enter(0xD2)
        for index in range(3):
            value = self.task.process(ring_frame(), self.cfg)["result"]
            self.assertEqual(value["found"], index == 2)
            self.assertEqual(value["ocr_calls"], 1 if index == 2 else 0)
        self.assertEqual((value["cx"], value["cy"]), (160, 120))
        self.assertEqual(value["recognized_digit"], 2)

    def test_lock_roi_shrinks_and_center_reference_does_not_move(self):
        self.track()
        for shift in (3, 6, 9):
            frame = self.task.process(ring_frame(((160 + shift, 120),)), self.cfg)
            result = frame["result"]
            self.assertEqual(result["state"], "TRACK")
            self.assertEqual(result["dx"], shift)
            self.assertEqual(result["dy"], 0)
            self.assertLess(frame["roi_rect"][2], 320)
            self.assertLess(frame["roi_rect"][3], 240)
            preview = make_ring_preview(ring_frame(), frame["mask_roi"],
                                        result, self.cfg["ring"], "MASK")
            self.assertEqual(preview.shape, (240, 320, 3))

    def test_layer_check_is_periodic_then_loss_forces_full_roi(self):
        self.track()
        checks = []
        for _ in range(10):
            result = self.task.process(ring_frame(), self.cfg)["result"]
            checks.append(result["layers_checked"])
        self.assertEqual(checks, [False] * 4 + [True] + [False] * 4 + [True])
        blank = ring_frame(())
        value = self.task.process(blank, self.cfg)["result"]
        self.assertEqual(value["state"], "LOST")
        self.assertEqual(safe_xy_result(value, True), (False, 0, 0))
        value = self.task.process(ring_frame(), self.cfg)["result"]
        self.assertTrue(value["layers_checked"])
        self.assertEqual(value["process_roi"], (0, 0, 320, 240))
        self.assertEqual(value["state"], "TRACK")

    def test_solid_replacement_cannot_survive_next_layer_check(self):
        self.track()
        solid = ring_frame(())
        cv2.circle(solid, (160, 120), 32, (0, 0, 0), -1)
        for _ in range(5):
            value = self.task.process(solid, self.cfg)["result"]
        self.assertFalse(value["found"])
        self.assertEqual(value["state"], "LOST")

    def test_loss_fifteen_frames_forgets_no_old_coordinates(self):
        self.track()
        for index in range(15):
            value = self.task.process(ring_frame(()), self.cfg)["result"]
            self.assertFalse(value["found"])
            self.assertEqual(safe_xy_result(value, True), (False, 0, 0))
            self.assertEqual(value["target_confirmed"], index < 14)
        self.assertEqual(value["state"], "SEARCH")
        self.assertEqual(self.task.process(ring_frame(), self.cfg)["result"]["state"], "CONFIRM")

    def test_jump_to_side_ring_does_not_hijack_lock(self):
        self.track()
        result = self.task.process(ring_frame(((40, 120), (280, 120))), self.cfg)["result"]
        self.assertFalse(result["found"])
        self.assertEqual(result["state"], "LOST")

    def test_task_switch_clears_ring_lock(self):
        manager = TaskManager()
        manager.switch(TASK_RING, 0xD2)
        for _ in range(3):
            manager.process(ring_frame(), self.cfg)
        manager.switch(TASK_STACK, 1)
        manager.switch(TASK_RING, 0xD2)
        self.assertFalse(manager.tasks[TASK_RING].target_confirmed)


class SenderTests(unittest.TestCase):
    result = {"found": True, "state": "TRACK", "dx": -17, "dy": 12}

    def test_b2_28ms_schedule_and_other_tasks_remain_25ms(self):
        self.assertEqual(send_interval_ms(TASK_MATERIAL), 28)
        self.assertEqual(send_interval_ms(TASK_RING), 25)
        self.assertEqual(send_interval_ms(TASK_STACK), 25)
        sender = ResultSender(interval_ms=send_interval_ms(TASK_MATERIAL))
        packets = [sender.take(self.result, tick, tick) for tick in range(2800)]
        self.assertEqual([tick for tick, value in enumerate(packets) if value is not None],
                         list(range(0, 2800, 28)))
        sender.reset(0)
        self.assertIsNotNone(sender.take(self.result, 113, 113))
        self.assertEqual(sender.next_send_ms, 140)
        self.assertIsNone(sender.take(self.result, 113, 113))

    def test_28ms_scheduler_uses_current_result_with_39hz_processing(self):
        sender = ResultSender(interval_ms=28)
        packets = []
        for tick in range(0, 28000, 26):
            value = sender.take(dict(self.result, dx=tick), tick, tick)
            if value is not None:
                self.assertEqual(value["dx"], tick)
                packets.append(value)
        self.assertEqual(len(packets), 1000)

    def test_packet_is_same_six_byte_payload_and_signed_int16(self):
        packet = pack_xy_frame(0xB3, 8, True, -17, 12)
        self.assertEqual(len(packet), 11)
        self.assertEqual(packet[:6], bytes([0xAA, 0x55, 0xB3, 6, 8, 1]))
        self.assertEqual(struct.unpack("<hh", packet[6:10]), (-17, 12))
        self.assertEqual(packet[-1], sum(packet[2:-1]) & 255)

    def test_invalid_packet_forces_zero_coordinates(self):
        self.assertEqual(pack_xy_frame(0xB2, 9, False, 500, -400)[5:10], bytes(5))

    def test_25ms_schedule_40_packets_per_second_and_no_catchup_burst(self):
        sender = ResultSender()
        sent = [sender.take(self.result, tick, tick) for tick in range(1000)]
        self.assertEqual(sum(value is not None for value in sent), 40)
        self.assertEqual([i for i, value in enumerate(sent) if value is not None],
                         list(range(0, 1000, 25)))
        sender.reset(0)
        tx = sender.take(self.result, 110, 120)
        self.assertTrue(tx["raw_ok"])
        self.assertIsNone(sender.take(self.result, 120, 120))
        self.assertEqual(sender.next_send_ms, 125)

    def test_stale_result_is_invalid_not_resent_old_xy(self):
        tx = ResultSender().take(self.result, 0, 51, True)
        self.assertFalse(tx["fresh"])
        self.assertEqual((tx["raw_ok"], tx["dx"], tx["dy"]), (False, 0, 0))

    def test_track_state_required_and_frame_id_wraps(self):
        sender = ResultSender()
        for index in range(257):
            tx = sender.take(dict(self.result, state="CONFIRM"), index * 25, index * 25, True)
            self.assertFalse(tx["raw_ok"])
        self.assertEqual(tx["frame_id"], 1)

    def test_fps_is_not_confused_with_successful_serial_hz(self):
        stats = PerfStats(0)
        for tick in range(40):
            tx = dict(age_ms=10, fresh=True) if tick % 2 == 0 else None
            stats.add(25, 5, 18, 0, {}, tx)
        log = stats.line(1000, TASK_RING, True, {})
        self.assertIn("FPS:40.0 TXHZ:20.0", log)
        self.assertIn("LOOP95:25.0", log)


class ControlTests(unittest.TestCase):
    def setUp(self):
        tree = ast.parse((MODULE_DIR / "main.py").read_text(encoding="utf-8"))
        names = {"checksum", "clear_color_target", "handle_cmd", "rx_handle"}
        selected = ast.Module(
            body=[node for node in tree.body if isinstance(node, ast.FunctionDef)
                  and node.name in names], type_ignores=[])
        self.acquire, self.release = Mock(), Mock()
        self.serial = Mock()
        self.serial.read.return_value = b""
        self.env = dict(CMD_NONE=0xB0, CMD_BLOB=0xB2, CMD_RING=0xB3, CMD_ONUP=0xB4,
                        COLOR_NONE=0xC0, SUM_2=0xD2, mode=0xB0, target=0xC0,
                        COLOR_CODE_TO_KEY={0xC0 + i: str(i) for i in range(1, 7)},
                        uart_tx_acquire=self.acquire, uart_tx_release=self.release,
                        uart_tx_owned=False, control_revision=0, rx_buf=bytearray(),
                        serial=self.serial, MAX_RX_DATA_LEN=32, uart_control_active=False,
                        print=Mock())
        exec(compile(selected, "main-control", "exec"), self.env)

    def feed(self, data):
        self.serial.read.return_value = data
        self.env["rx_handle"]()

    def frame(self, cmd, data):
        return bytes([0xAA, 0x55, cmd, len(data)] + list(data) +
                     [(cmd + len(data) + sum(data)) & 255])

    def test_fragmented_sticky_bad_checksum_unknown_and_stop(self):
        frame = self.frame(0xB2, [0xC1])
        self.feed(frame[:3])
        self.assertEqual(self.env["mode"], 0xB0)
        self.feed(frame[3:] + b"noise")
        self.assertEqual(self.env["mode"], 0xB2)
        bad = bytearray(self.frame(0xB4, [0xC3]))
        bad[-1] ^= 1
        self.feed(bad + self.frame(0xB1, [1]) + self.frame(0xB3, [2]) + self.frame(0xB0, []))
        self.assertEqual((self.env["mode"], self.env["target"]), (0xB0, 0xC0))
        self.release.assert_called_once()
        self.assertLess(len(self.env["rx_buf"]), 5)

    def test_pure_circle_rejects_1_and_3_without_acquiring_tx(self):
        for digit in (1, 3, 0xD1, 0xD3):
            self.feed(self.frame(0xB3, [digit]))
        self.acquire.assert_not_called()
        self.assertEqual(self.env["mode"], 0xB0)
        self.feed(self.frame(0xB3, [0xD2]))
        self.assertEqual(self.env["mode"], 0xB3)

    def test_repeated_start_does_not_reset_tracking_revision(self):
        self.feed(self.frame(0xB3, [2]))
        revision = self.env["control_revision"]
        self.feed(self.frame(0xB3, [2]))
        self.assertEqual(self.env["control_revision"], revision)

    def test_unsupported_colors_do_not_acquire_tx_or_switch_task(self):
        for cmd in (0xB2, 0xB4):
            for code in (0, 7, 99, 0xC0, 0xC7):
                self.feed(self.frame(cmd, [code]))
        self.acquire.assert_not_called()
        self.feed(self.frame(0xB3, [2]))
        revision = self.env["control_revision"]
        self.feed(self.frame(0xB2, [0xC7]))
        self.assertEqual((self.env["mode"], self.env["target"]), (0xB3, 0xD2))
        self.assertEqual(self.env["control_revision"], revision)

    def test_six_colors_numeric_and_coded_targets_are_supported_in_b2_and_b4(self):
        for cmd in (0xB2, 0xB4):
            for number in range(1, 7):
                for code in (number, 0xC0 + number):
                    self.feed(self.frame(cmd, [code]))
                    self.assertEqual((self.env["mode"], self.env["target"]), (cmd, 0xC0 + number))

    def test_invalid_length_recovers_and_switches_selected_color(self):
        self.feed(b"\xAA\x55\xB2\xFF" + self.frame(0xB4, [3]))
        self.assertEqual((self.env["mode"], self.env["target"]), (0xB4, 0xC3))


if __name__ == "__main__":
    unittest.main()
