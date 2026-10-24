"""B2日志必须区分模拟/实发，优化必须保持掩膜和判定一致。"""
import copy
import unittest
from unittest.mock import patch

import cv2
import numpy as np

from test_vision_v9 import config, color_frame
from vision_blob import (apply_grasp_status, empty_result,
                         find_largest_blob, selected_coverage)
from vision_runtime import B2DebugLog, pack_xy_frame
from vision_tasks import ColorTracker


def result_at(cx=177, cy=92, pixels=960, found=True):
    value = empty_result()
    value.update(found=found, cx=cx, cy=cy, dx=cx - 177, dy=cy - 92,
                 coverage_pixels=pixels, coverage_total=1600, coverage_ratio=pixels / 1600)
    return apply_grasp_status(value, config()["grasp"])


class B2DebugTests(unittest.TestCase):
    def test_each_sample_keeps_fps_and_exact_pixel_fraction(self):
        value = result_at(pixels=799)
        line = B2DebugLog().line(value, config()["grasp"], 0, fps=35.6, tx_hz=0.0)
        self.assertIn("FPS=35.6 TXHZ=0.0", line)
        self.assertIn("PIX=799/1600", line)
        self.assertIn("COV=49.9%/60%", line)

    def test_preview_explains_all_states_and_coverage_above_threshold_outside_center(self):
        cases = ((result_at(found=False), "OK=0", "WHY=NO_TARGET", "IN=0"),
                 (result_at(pixels=800), "OK=1", "WHY=LOW_COVER", "IN=1"),
                 (result_at(cx=100, pixels=1200), "OK=1", "WHY=CENTER_OUT", "IN=0"),
                 (result_at(), "OK=2", "WHY=ALIGNED", "IN=1"))
        for result, status, reason, inside in cases:
            line = B2DebugLog().line(result, config()["grasp"], 0)
            self.assertIn("[VISION B2 PREVIEW] UI", line)
            for expected in (status, reason, inside, "ROI=(157,72,40,40)"):
                self.assertIn(expected, line)
            packet = bytes.fromhex(line.split("HEX=")[1])
            self.assertEqual(len(packet), 11)
            self.assertEqual(packet[5], result["ok"])
            self.assertEqual(packet[-1], sum(packet[2:-1]) & 255)

    def test_tx_print_is_exact_successful_packet_including_stale_zero(self):
        result = result_at()
        for fresh, ok in ((True, 2), (False, 0)):
            sent = dict(frame_id=183, raw_ok=ok, dx=0 if not fresh else -123,
                        dy=0 if not fresh else 456, fresh=fresh)
            line = B2DebugLog().line(result, config()["grasp"], 0, True, sent, True)
            packet = bytes.fromhex(line.split("HEX=")[1])
            self.assertIn("[VISION B2 TX] RUN", line)
            self.assertIn("DET_OK=2", line)
            self.assertIn("OK=" + str(ok), line)
            self.assertEqual(packet, pack_xy_frame(0xB2, 183, ok, sent["dx"], sent["dy"]))
            if not fresh:
                self.assertIn("WHY=STALE", line)
                self.assertEqual(packet[5:10], bytes(5))

    def test_active_uart_without_success_never_claims_tx_or_preview(self):
        self.assertIsNone(B2DebugLog().line(result_at(), config()["grasp"], 0,
                                           run_mode=True, tx_active=True))

    def test_stable_status_prints_once_per_second(self):
        logger = B2DebugLog()
        times = [t for t in range(0, 2001, 25)
                 if logger.line(result_at(), config()["grasp"], t) is not None]
        self.assertEqual(times, [0, 1000, 2000])

    def test_changes_are_limited_and_short_lived_state_is_recorded(self):
        logger = B2DebugLog()
        logger.line(result_at(pixels=800), config()["grasp"], 0)
        self.assertIsNone(logger.line(result_at(), config()["grasp"], 25))
        line = logger.line(result_at(pixels=800), config()["grasp"], 200)
        self.assertIn("OK=1", line)
        self.assertIn("SEEN=1,2", line)
        logger.reset()
        times = []
        for i, t in enumerate(range(0, 1001, 25)):
            result = result_at(found=i % 2 == 0)
            if logger.line(result, config()["grasp"], t) is not None:
                times.append(t)
        self.assertTrue(all(b - a >= 200 for a, b in zip(times, times[1:])))
        self.assertLessEqual(len(times), 6)  # 包括0和1000ms两个端点

    def test_reset_drops_previous_seen_state(self):
        logger = B2DebugLog()
        logger.line(result_at(), config()["grasp"], 0)
        logger.reset()
        line = logger.line(result_at(found=False), config()["grasp"], 1)
        self.assertIn("SEEN=0 HEX=", line)


class B2SmallOptimizationTests(unittest.TestCase):
    def test_zero_color_intersection_skips_flood_without_mutating_mask(self):
        mask = np.zeros((240, 320), np.uint8)
        mask[60:140, 80:160] = 255
        mask[70:130, 90:150] = 0
        contour = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0][0]
        before = mask.copy()
        with patch("vision_blob.cv2.floodFill", side_effect=AssertionError("unnecessary flood")):
            self.assertEqual(selected_coverage(mask, contour, cv2.boundingRect(contour),
                                               (0, 0, 320, 240), (100, 80, 20, 20)), (0, 400))
        np.testing.assert_array_equal(mask, before)

    def test_whole_candidate_uses_connected_pixel_count_excluding_nested_island(self):
        mask = np.zeros((240, 320), np.uint8)
        mask[60:140, 80:160] = 255
        mask[70:130, 90:150] = 0
        mask[80:90, 100:110] = 255
        contour = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0][0]
        count, total = selected_coverage(mask, contour, cv2.boundingRect(contour),
                                         (0, 0, 320, 240), (60, 40, 120, 120))
        self.assertEqual((count, total), (80 * 80 - 60 * 60, 120 * 120))

    def test_fast_filter_reuses_best_bounding_rectangle(self):
        cfg = config()
        mask = np.zeros((240, 320), np.uint8)
        mask[70:130, 130:190] = 255
        expected = find_largest_blob(mask, cfg, (0, 0, 320, 240))
        with patch("vision_blob.cv2.boundingRect", wraps=cv2.boundingRect) as bounds:
            actual = find_largest_blob(mask, cfg, (0, 0, 320, 240), fast_filter=True)
        self.assertEqual(actual, expected)
        self.assertEqual(bounds.call_count, 1)

    def test_ui_buffers_and_fast_filter_match_original_across_modes(self):
        cfg = config()
        image = color_frame(cfg)
        original, optimized = ColorTracker(), ColorTracker()
        for run in (False, False, True, True, False):
            before = copy.deepcopy(cfg)
            expected = original.process(image, cfg, run, True, False, cfg["grasp"])
            actual = optimized.process(image, cfg, run, True, True, cfg["grasp"])
            self.assertEqual(optimized.b2_buffers.mask.shape, (180, 240) if run else (240, 320))
            self.assertTrue(actual["result"]["buffer_reuse"])
            for name in ("found", "ok", "cx", "cy", "dx", "dy", "area", "rect",
                         "coverage_ratio", "coverage_pixels", "coverage_total"):
                self.assertEqual(actual["result"][name], expected["result"][name])
            np.testing.assert_array_equal(actual["mask_roi"], expected["mask_roi"])
            self.assertEqual(cfg, before)


if __name__ == "__main__":
    unittest.main()
