"""B2抓取覆盖判定：掩膜、原图/缩放坐标、持久化和发送链路。"""
import copy
import json
import struct
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import cv2
import numpy as np

from test_vision_v9 import config, midpoint_bgr
from test_ui_v9 import VisionUI
import vision_blob
from vision_blob import (apply_grasp_status, empty_result, find_largest_blob,
                         make_run_geometry, map_run_result_to_source, scale_grasp_roi,
                         selected_coverage, make_preview)
from vision_config import DEFAULT_CONFIG, VisionConfigStore, validate_config
from vision_runtime import ResultSender, pack_xy_frame
from vision_tasks import TASK_MATERIAL, TASK_STACK, TaskManager, safe_xy_result


def grasp_config():
    cfg = config()
    cfg["blob"] = dict(min_area=5, max_area=76800)
    cfg["morph_mode"] = "NONE"
    cfg["grasp"] = dict(roi=dict(x=100, y=80, w=20, h=20), min_coverage_pct=60)
    return cfg


class GraspMaskTests(unittest.TestCase):
    def test_intersection_and_preview_match_independent_connected_components(self):
        rng = np.random.default_rng(127)
        for _ in range(20):
            mask = np.zeros((240, 320), np.uint8)
            cv2.ellipse(mask, (170, 110), (90, 60), 0, 0, 360, 255, -1)
            for _ in range(15):
                x, y = map(int, rng.integers((70, 40), (270, 180)))
                mask[y:y + 12, x:x + 15] = 0
            mask[85:125, 140:195] = 0
            mask[95:110, 155:175] = 255  # 外轮廓空洞内的独立同色岛
            contours = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
            contour = max(contours, key=cv2.contourArea)
            labels = cv2.connectedComponents(mask, connectivity=8)[1]
            seed_x, seed_y = map(int, contour.reshape(-1, 2)[0])
            selected = labels == labels[seed_y, seed_x]
            rois = [(0, 0, 320, 240), (140, 85, 55, 40), (300, 220, 20, 20)]
            rois += [(int(x), int(y), 40, 40) for x, y in rng.integers((0, 0), (280, 200), (5, 2))]
            for roi in rois:
                x, y, w, h = roi
                preview = {}
                pixels, total = selected_coverage(mask, contour, cv2.boundingRect(contour),
                                                   (0, 0, 320, 240), roi, preview)
                expected = selected[y:y + h, x:x + w]
                self.assertEqual(pixels, int(np.count_nonzero(expected)))
                self.assertEqual(total, w * h)
                np.testing.assert_array_equal(preview["mask"] != 0, expected)
                self.assertEqual(selected_coverage(mask, contour, cv2.boundingRect(contour),
                                                   (0, 0, 320, 240), roi), (pixels, total))

    def detect(self, mask, cfg=None):
        cfg = cfg or grasp_config()
        roi = cfg["grasp"]["roi"]
        result = find_largest_blob(mask, cfg, (0, 0, 320, 240),
                                   grasp_roi=tuple(roi[k] for k in ("x", "y", "w", "h")))
        return apply_grasp_status(result, cfg["grasp"])

    def test_no_target_clears_all_fields(self):
        result = self.detect(np.zeros((240, 320), np.uint8))
        self.assertEqual(result, empty_result())
        self.assertEqual(safe_xy_result(result, allow_grasp=True), (0, 0, 0))

    def test_center_and_coverage_matrix_including_exact_threshold(self):
        cases = (
            ((40, 40, 60, 60), 1, 0),       # 目标在框外
            ((106, 86, 114, 94), 1, 64),     # 中心在框内，16%
            ((100, 80, 111, 100), 1, 220),   # 55%
            ((100, 80, 112, 100), 2, 240),   # 恰好60%
            ((100, 80, 113, 100), 2, 260),   # 65%
            ((80, 80, 113, 100), 1, 260),    # 65%，但重心在框左侧
        )
        for rect, ok, count in cases:
            with self.subTest(rect=rect):
                mask = np.zeros((240, 320), np.uint8)
                x1, y1, x2, y2 = rect
                mask[y1:y2, x1:x2] = 255
                result = self.detect(mask)
                self.assertEqual(result["ok"], ok)
                self.assertEqual(result["coverage_pixels"], count)
                self.assertEqual(result["coverage_total"], 400)
                self.assertEqual(result["coverage_ratio"], count / 400)
                self.assertEqual(result["dx"], result["cx"] - grasp_config()["center"]["x"])
                self.assertEqual(result["dy"], result["cy"] - grasp_config()["center"]["y"])

    def test_other_same_color_target_cannot_contribute(self):
        mask = np.zeros((240, 320), np.uint8)
        mask[10:70, 10:70] = 255  # 最大合格目标位于框外
        mask[80:100, 100:120] = 255  # 另一个同色块覆盖整个抓取框
        result = self.detect(mask)
        self.assertEqual((result["ok"], result["coverage_pixels"]), (1, 0))

    def test_holes_and_disconnected_island_inside_selected_contour_are_excluded(self):
        mask = np.zeros((240, 320), np.uint8)
        mask[60:140, 80:160] = 255
        mask[70:130, 90:150] = 0
        mask[84:92, 104:112] = 255  # 外轮廓内部空洞中的独立岛
        before = mask.copy()
        result = self.detect(mask)
        self.assertEqual(result["coverage_pixels"], 0)
        self.assertEqual(result["ok"], 1)
        np.testing.assert_array_equal(mask, before)

    def test_hole_does_not_become_filled_target_area(self):
        mask = np.zeros((240, 320), np.uint8)
        mask[60:140, 80:160] = 255
        mask[85:95, 105:115] = 0
        result = self.detect(mask)
        self.assertEqual(result["coverage_pixels"], 300)
        self.assertEqual(result["coverage_ratio"], 0.75)

    def test_right_and_bottom_edges_are_excluded_from_center_condition(self):
        cfg = grasp_config()
        for cx, cy, expected in ((100, 80, 2), (119, 99, 2), (120, 90, 1), (110, 100, 1)):
            result = dict(found=True, cx=cx, cy=cy, dx=0, dy=0,
                          coverage_pixels=240, coverage_total=400)
            self.assertEqual(apply_grasp_status(result, cfg["grasp"])["ok"], expected)


class GraspIntegrationTests(unittest.TestCase):
    def test_only_displayed_ui_frames_create_counted_pixel_preview(self):
        cfg = grasp_config()
        frame = np.full((240, 320, 3), 255, np.uint8)
        frame[80:100, 100:112] = midpoint_bgr(cfg, "red")
        manager = TaskManager()
        manager.switch(TASK_MATERIAL, 1)
        for run, need_mask in ((False, True), (False, False), (True, True), (True, False)):
            result = manager.process(frame, cfg, run, need_mask)["result"]
            self.assertEqual("grasp_preview" in result, not run and need_mask)
            if "grasp_preview" in result:
                counted = result["grasp_preview"]
                self.assertEqual(counted["rect"], (100, 80, 20, 20))
                self.assertEqual(np.count_nonzero(counted["mask"]), result["coverage_pixels"])
                rendered = make_preview(frame, None, result, cfg, "RAW")
                self.assertEqual(rendered.shape, frame.shape)
                self.assertEqual(counted["contour"].dtype, np.int32)

    def test_exact_threshold_and_outside_center_through_lab_in_both_resolutions(self):
        for x1, x2, expected in ((100, 112, 2), (100, 113, 2), (80, 113, 1)):
            cfg = grasp_config()
            frame = np.full((240, 320, 3), 255, np.uint8)
            frame[80:100, x1:x2] = midpoint_bgr(cfg, "red")
            manager = TaskManager()
            manager.switch(TASK_MATERIAL, 1)
            for run in (False, True):
                result = manager.process(frame, cfg, run, False)["result"]
                self.assertEqual(result["ok"], expected, (x1, x2, run, result))
                self.assertGreaterEqual(result["coverage_ratio"], 0.6)
                if x1 == 100 and x2 == 112:
                    self.assertEqual(result["coverage_ratio"], 0.6)

    def test_preview_and_run_share_rule_and_reference_without_extra_conversion(self):
        cfg = grasp_config()
        cfg["grasp"] = copy.deepcopy(config()["grasp"])
        frame = np.full((240, 320, 3), 255, np.uint8)
        frame[60:140, 140:210] = midpoint_bgr(cfg, "red")
        mgr = TaskManager()
        mgr.switch(TASK_MATERIAL, 0xC1)
        for run in (False, True):
            with patch("vision_blob.convert_color_space", wraps=vision_blob.convert_color_space) as convert:
                result = mgr.process(frame, cfg, run, False)["result"]
            self.assertEqual(convert.call_count, 1)
            self.assertEqual(result["ok"], 2)
            self.assertEqual(result["coverage_ratio"], 1)
            self.assertEqual(result["cx"] - result["dx"], cfg["center"]["x"])
            self.assertEqual(result["cy"] - result["dy"], cfg["center"]["y"])
            lost = mgr.process(np.full_like(frame, 255), cfg, run, False)["result"]
            for key in ("ok", "dx", "dy", "cx", "cy", "coverage_ratio", "coverage_pixels"):
                self.assertEqual(lost[key], 0)

    def test_run_roi_rounding_at_borders_and_non_integer_positions(self):
        cfg = grasp_config()
        geom = make_run_geometry(cfg, 320, 240, 240, 180)
        for roi, expected in (
                ((157, 72, 40, 40), (117, 54, 31, 30)),
                ((319, 239, 1, 1), (239, 179, 1, 1)),
                ((0, 0, 1, 1), (0, 0, 1, 1)),
                ((300, 220, 20, 20), (225, 165, 15, 15))):
            grasp = dict(roi=dict(zip(("x", "y", "w", "h"), roi)))
            self.assertEqual(scale_grasp_roi(grasp, geom), expected)

    def test_edge_roi_full_coverage_in_preview_and_run(self):
        cfg = grasp_config()
        cfg["grasp"]["roi"] = dict(x=296, y=216, w=24, h=24)
        frame = np.full((240, 320, 3), 255, np.uint8)
        frame[212:240, 292:320] = midpoint_bgr(cfg, "red")
        mgr = TaskManager()
        mgr.switch(TASK_MATERIAL, 1)
        for run in (False, True):
            result = mgr.process(frame, cfg, run, False)["result"]
            self.assertEqual((result["ok"], result["coverage_ratio"]), (2, 1.0))

    def test_mapping_preserves_status_coverage_and_clears_lost_result(self):
        cfg = grasp_config()
        geometry = make_run_geometry(cfg, 320, 240, 240, 180)
        original = dict(found=True, ok=2, dx=3, dy=-3, rect=(70, 60, 20, 20),
                        area=300, coverage_pixels=240, coverage_total=400, coverage_ratio=0.6)
        mapped = map_run_result_to_source(original, cfg, geometry)
        for key in ("ok", "coverage_pixels", "coverage_total", "coverage_ratio"):
            self.assertEqual(mapped[key], original[key])
        original["found"] = False
        self.assertEqual(map_run_result_to_source(original, cfg, geometry), empty_result())

    def test_changing_grasp_parameters_takes_effect_without_geometry_reset(self):
        cfg = grasp_config()
        frame = np.full((240, 320, 3), 255, np.uint8)
        frame[60:140, 140:210] = midpoint_bgr(cfg, "red")
        mgr = TaskManager()
        mgr.switch(TASK_MATERIAL, 1)
        self.assertEqual(mgr.process(frame, cfg, True)["result"]["ok"], 1)
        geometry = mgr.color_tracker.run_geometry
        cfg["grasp"] = copy.deepcopy(config()["grasp"])
        self.assertEqual(mgr.process(frame, cfg, True)["result"]["ok"], 2)
        self.assertIs(mgr.color_tracker.run_geometry, geometry)

    def test_b4_does_not_run_coverage_even_with_b2_grasp_parameters(self):
        cfg = grasp_config()
        cfg["stack"]["roi"] = cfg["roi"]
        cfg["stack"]["blob"] = cfg["blob"]
        frame = np.full((240, 320, 3), 255, np.uint8)
        frame[60:140, 140:210] = midpoint_bgr(cfg, "red")
        mgr = TaskManager()
        mgr.switch(TASK_STACK, 1)
        with patch("vision_blob.selected_coverage", side_effect=AssertionError("B4 coverage")):
            result = mgr.process(frame, cfg, True)["result"]
        self.assertEqual(result["ok"], 1)


class GraspConfigTests(unittest.TestCase):
    def test_old_configs_preserve_all_existing_parameters_and_add_grasp(self):
        for schema in (8, 9):
            old = config()
            old.pop("grasp")
            old["schema_version"] = schema
            old["center"] = dict(x=310, y=5, custom=True)
            old["thresholds"]["LAB"]["red"] = dict(lower=[35, 160, 150], upper=[110, 185, 170])
            old["blob"] = dict(min_area=1800, max_area=12000)
            value = validate_config(old)
            for key in old.keys() - {"schema_version"}:
                self.assertEqual(value[key], old[key])
            w, h = (DEFAULT_CONFIG["grasp"]["roi"][k] for k in ("w", "h"))
            self.assertEqual(value["grasp"]["roi"], dict(x=min(310-w//2, 320-w), y=0, w=w, h=h))
            self.assertEqual(value["grasp"]["min_coverage_pct"], DEFAULT_CONFIG["grasp"]["min_coverage_pct"])

    def test_invalid_grasp_fields_are_clamped_to_valid_image_region(self):
        for roi in (dict(x=-10, y=999, w=0, h=-2), dict(x=999, y=-100, w=1000, h=999),
                    dict(x="bad", y=None, w=[], h={}), None):
            cfg = config()
            cfg["grasp"] = dict(roi=roi, min_coverage_pct=900)
            value = validate_config(cfg)["grasp"]
            r = value["roi"]
            self.assertTrue(1 <= r["w"] <= 320 and 1 <= r["h"] <= 240)
            self.assertTrue(0 <= r["x"] <= 320 - r["w"])
            self.assertTrue(0 <= r["y"] <= 240 - r["h"])
            self.assertEqual(value["min_coverage_pct"], 100)

    def test_saved_grasp_remains_independent_of_center_and_round_trips(self):
        cfg = config()
        cfg["grasp"] = dict(roi=dict(x=200, y=190, w=21, h=43), min_coverage_pct=73)
        cfg["center"] = dict(x=25, y=30, custom=True)
        with tempfile.TemporaryDirectory() as path:
            store = VisionConfigStore(path)
            saved, _ = store.save(cfg)
            loaded, _ = store.load()
            self.assertEqual(loaded, saved)
            self.assertEqual(json.loads(Path(store.path).read_text())["grasp"], cfg["grasp"])
        self.assertEqual(loaded["grasp"], cfg["grasp"])
        self.assertEqual(loaded["center"], cfg["center"])


class GraspSenderTests(unittest.TestCase):
    def test_ok2_survives_safe_sender_pack_and_signed_coordinates(self):
        result = dict(found=True, ok=2, dx=-123, dy=456)
        self.assertEqual(safe_xy_result(result, allow_grasp=True), (2, -123, 456))
        tx = ResultSender(28).take(result, 0, 10, allow_grasp=True)
        packet = pack_xy_frame(0xB2, tx["frame_id"], tx["raw_ok"], tx["dx"], tx["dy"])
        self.assertEqual(len(packet), 11)
        self.assertEqual(packet[:4], bytes([0xAA, 0x55, 0xB2, 6]))
        self.assertEqual(packet[5], 2)
        self.assertEqual(struct.unpack("<hh", packet[6:10]), (-123, 456))
        self.assertEqual(packet[-1], sum(packet[2:-1]) & 255)

    def test_stale_ok2_becomes_zero_coordinates_and_state(self):
        tx = ResultSender().take(dict(found=True, ok=2, dx=10, dy=20), 0, 51, allow_grasp=True)
        self.assertEqual((tx["raw_ok"], tx["dx"], tx["dy"]), (0, 0, 0))
        self.assertFalse(tx["fresh"])

    def test_b3_b4_never_emit_two_even_for_foreign_state(self):
        result = dict(found=True, ok=2, state="TRACK", dx=4, dy=8)
        self.assertEqual(safe_xy_result(result), (1, 4, 8))
        tx = ResultSender().take(result, 0, 1)
        self.assertEqual(tx["raw_ok"], 1)
        for cmd in (0xB3, 0xB4):
            self.assertEqual(pack_xy_frame(cmd, 1, 2, 4, 8)[5], 1)


class GraspUiTests(unittest.TestCase):
    def test_grasp_editor_saves_five_fields_without_moving_reference_or_search_roi(self):
        for size in ((552, 368), (640, 480), (480, 640)):
            ui = VisionUI(*size, config(), "loaded")
            frame = np.zeros((240, 320, 3), np.uint8)
            ui._open_task(TASK_MATERIAL)
            ui._activate("OPEN_CALIBRATE")
            ui.render(frame, empty_result())
            self.assertIn("EDIT_GRASP", {b[0] for b in ui.buttons})
            ui._activate("EDIT_GRASP")
            ui.render(frame, empty_result())
            for _, x, y, w, h in ui.buttons:
                self.assertTrue(0 <= x and x + w <= size[0] and 0 <= y and y + h <= size[1])
            self.assertEqual(ui.preview_rect[1], 7)
            for field in ("x", "y", "w", "h", "min_coverage_pct"):
                ui._activate("GRASP_PARAM_" + field)
                ui._activate("GRASP_PLUS")
            draft = ui.detection_config()
            self.assertIs(draft, ui.draft_config)
            self.assertEqual(draft["grasp"]["min_coverage_pct"], 65)
            self.assertEqual(draft["grasp"]["roi"], dict(x=162, y=77, w=45, h=45))
            self.assertEqual(draft["center"], ui.saved_config["center"])
            self.assertEqual(draft["roi"], dict(x=0, y=0, w=320, h=240))
            action = ui._activate("SAVE_EDITOR")
            self.assertEqual(action["config"]["grasp"], draft["grasp"])
            ui._activate("RESTORE_EDITOR")
            self.assertEqual(ui.detection_config()["grasp"], ui.saved_config["grasp"])

    def test_b4_has_no_grasp_editor(self):
        ui = VisionUI(552, 368, config(), "loaded")
        ui._open_task(TASK_STACK)
        ui._activate("OPEN_CALIBRATE")
        ui.render(np.zeros((240, 320, 3), np.uint8), empty_result())
        self.assertNotIn("EDIT_GRASP", {b[0] for b in ui.buttons})


if __name__ == "__main__":
    unittest.main()
