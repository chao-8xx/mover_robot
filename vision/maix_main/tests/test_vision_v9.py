import copy
import sys
import unittest
from pathlib import Path

import cv2
import numpy as np


MODULE_DIR = Path(__file__).resolve().parents[1]
if str(MODULE_DIR) not in sys.path:
    sys.path.insert(0, str(MODULE_DIR))

from vision_config import DEFAULT_CONFIG, validate_config  # noqa: E402
from vision_ocr import confirm_digit_two, warmup_two_templates  # noqa: E402
from vision_ring import detect_ring_candidates  # noqa: E402
from vision_tasks import (  # noqa: E402
    RingTask,
    TASK_MATERIAL,
    TASK_RING,
    TASK_STACK,
    TaskManager,
    safe_xy_result,
)


RINGS = ((30, 1), (160, 2), (290, 3))


def make_ring_frame(
        visible=(0, 1, 2), draw_digits=True, center_shift=0,
        middle_digit=None):
    image = np.full((240, 320, 3), 255, dtype=np.uint8)
    for index in visible:
        cx, digit = RINGS[index]
        if index == 1:
            cx += center_shift
        for radius in (21, 18, 15, 12):
            cv2.circle(image, (cx, 120), radius, (0, 0, 0), 1)
        if draw_digits:
            draw_digit = middle_digit if index == 1 and middle_digit else digit
            cv2.putText(
                image,
                str(draw_digit),
                (cx - 6, 127),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 0, 0),
                2,
                cv2.LINE_AA,
            )
    return image


class RingDetectorTests(unittest.TestCase):
    def setUp(self):
        self.config = copy.deepcopy(DEFAULT_CONFIG)

    def test_three_round_rings_remain_three_candidates(self):
        result, _, _ = detect_ring_candidates(
            make_ring_frame(), self.config["ring"]
        )
        self.assertEqual(result["candidate_count"], 3)
        self.assertEqual(result["geometry_count"], 3)
        self.assertEqual(result["layer_reject_count"], 0)
        self.assertNotIn("ellipse_fit_count", result)
        self.assertNotIn("ellipse_fix_count", result)

    def test_near_circular_perspective_rings_remain_detectable(self):
        image = np.full((240, 320, 3), 255, dtype=np.uint8)
        axes = ((21, 18), (18, 15), (15, 12), (12, 9))
        for cx, angle in ((40, -25), (160, 0), (280, 25)):
            for ellipse_axes in axes:
                cv2.ellipse(
                    image, (cx, 120), ellipse_axes, angle,
                    0, 360, (0, 0, 0), 1,
                )
        result, _, _ = detect_ring_candidates(image, self.config["ring"])
        self.assertEqual(result["candidate_count"], 3)
        self.assertTrue(all(
            value >= 0.85 for value in result["axis_ratio_values"]
        ))

    def test_obvious_ellipse_is_rejected_at_any_rotation(self):
        for angle in (0, 45):
            with self.subTest(angle=angle):
                image = np.full((240, 320, 3), 255, dtype=np.uint8)
                for axes in ((21, 12), (18, 10), (15, 8), (12, 6)):
                    cv2.ellipse(
                        image, (160, 120), axes, angle,
                        0, 360, (0, 0, 0), 1,
                    )
                result, _, _ = detect_ring_candidates(
                    image, self.config["ring"]
                )
                self.assertEqual(result["candidate_count"], 0)
                self.assertGreaterEqual(result["reject_counts"]["RATIO"], 1)

    def test_solid_circle_is_not_a_ring(self):
        image = np.full((240, 320, 3), 255, dtype=np.uint8)
        cv2.circle(image, (160, 120), 21, (0, 0, 0), -1)
        result, _, _ = detect_ring_candidates(image, self.config["ring"])
        self.assertEqual(result["candidate_count"], 0)
        self.assertEqual(result["reject_counts"]["LAYER"], 1)


class IsTwoTests(unittest.TestCase):
    def setUp(self):
        self.config = copy.deepcopy(DEFAULT_CONFIG)

    def test_only_digit_two_is_accepted(self):
        frame = make_ring_frame()
        result, _, _ = detect_ring_candidates(frame, self.config["ring"])
        decisions = [
            confirm_digit_two(frame, candidate)
            for candidate in result["candidates"]
        ]
        self.assertEqual(
            [decision["is_two"] for decision in decisions],
            [False, True, False],
        )
        self.assertEqual(decisions[1]["reason"], "TWO")
        for decision in (decisions[0], decisions[2]):
            self.assertIn(
                decision["reason"], ("SCORE", "MARGIN", "SCORE_MARGIN")
            )

    def test_blank_ring_stays_unknown(self):
        frame = make_ring_frame(visible=(1,), draw_digits=False)
        decision = confirm_digit_two(
            frame, {"cx": 160, "cy": 120, "radius": 21}
        )
        self.assertFalse(decision["is_two"])
        self.assertEqual(decision["digit"], 0)

    def test_template_warmup_builds_cache_before_first_confirmation(self):
        warmup = warmup_two_templates()
        cached = warmup_two_templates()
        self.assertEqual(warmup["template_count"], 12)
        self.assertEqual(cached["template_count"], 12)
        self.assertGreaterEqual(warmup["cost_ms"], 0.0)
        self.assertGreaterEqual(cached["cost_ms"], 0.0)


class RingTaskTests(unittest.TestCase):
    def setUp(self):
        self.config = copy.deepcopy(DEFAULT_CONFIG)
        self.task = RingTask()
        self.task.enter(2)

    def process(self, frame):
        return self.task.process(frame, self.config, run_mode=True)["result"]

    def enter_track(self):
        result = None
        calls = 0
        for _ in range(6):
            result = self.process(make_ring_frame())
            calls += result["ocr_calls"]
            if result["state"] == "TRACK":
                break
        return result, calls

    def test_center_candidate_confirms_once_then_stops_ocr(self):
        result, calls = self.enter_track()
        self.assertEqual(result["state"], "TRACK")
        self.assertTrue(result["found"])
        self.assertEqual(result["recognized_digit"], 2)
        self.assertEqual(result["cx"], 160)
        self.assertEqual(calls, 1)
        self.assertEqual(result["ocr_reason"], "TWO")
        self.assertLessEqual(result["ocr_score"], 0.42)
        self.assertGreaterEqual(result["ocr_margin"], 0.055)

        later_calls = 0
        for _ in range(300):
            result = self.process(make_ring_frame())
            later_calls += result["ocr_calls"]
        self.assertEqual(result["state"], "TRACK")
        self.assertEqual(later_calls, 0)

    def test_non_two_center_retries_slowly_and_never_becomes_valid(self):
        frame = make_ring_frame(visible=(1,), middle_digit=1)
        calls = 0
        result = None
        for _ in range(30):
            result = self.process(frame)
            calls += result["ocr_calls"]
        self.assertEqual(result["state"], "CONFIRM")
        self.assertFalse(result["found"])
        self.assertEqual(result["recognized_digit"], 0)
        self.assertGreaterEqual(calls, 1)
        self.assertLessEqual(calls, 4)

    def test_track_follows_center_ring_and_ignores_side_rings(self):
        result, _ = self.enter_track()
        self.assertEqual(result["state"], "TRACK")
        moved = self.process(make_ring_frame(center_shift=7))
        self.assertEqual(moved["state"], "TRACK")
        self.assertEqual(moved["cx"], 167)
        self.assertEqual(moved["ocr_calls"], 0)

    def test_short_loss_keeps_confirmation_and_reacquires_without_ocr(self):
        result, _ = self.enter_track()
        self.assertEqual(result["state"], "TRACK")
        side_only = make_ring_frame(visible=(0, 2))
        for expected_lost in range(1, 6):
            lost = self.process(side_only)
            self.assertEqual(lost["state"], "LOST")
            self.assertTrue(lost["is_two_confirmed"])
            self.assertEqual(lost["lost_frames"], expected_lost)
            self.assertEqual(lost["ocr_calls"], 0)
            self.assertFalse(lost["found"])

        recovered = self.process(make_ring_frame(center_shift=7))
        self.assertEqual(recovered["state"], "TRACK")
        self.assertTrue(recovered["is_two_confirmed"])
        self.assertEqual(recovered["lost_frames"], 0)
        self.assertEqual(recovered["ocr_calls"], 0)
        self.assertEqual(recovered["cx"], 167)

    def test_long_loss_eventually_forgets_confirmation(self):
        result, _ = self.enter_track()
        self.assertEqual(result["state"], "TRACK")
        side_only = make_ring_frame(visible=(0, 2))
        for _ in range(self.task.REACQUIRE_FRAME_LIMIT - 1):
            result = self.process(side_only)
        self.assertEqual(result["state"], "LOST")
        self.assertTrue(result["is_two_confirmed"])

        expired = self.process(side_only)
        self.assertEqual(expired["state"], "SEARCH")
        self.assertFalse(expired["is_two_confirmed"])
        self.assertEqual(expired["recognized_digit"], 0)
        self.assertEqual(expired["lost_frames"], 0)


class SharedColorTrackerTests(unittest.TestCase):
    def test_legacy_config_gets_safe_default_stack_geometry(self):
        legacy = copy.deepcopy(DEFAULT_CONFIG)
        legacy.pop("stack")
        legacy["schema_version"] = 3
        migrated = validate_config(legacy)
        self.assertEqual(migrated["stack"]["roi"], {
            "x": 0, "y": 0, "w": 320, "h": 240
        })
        self.assertEqual(migrated["stack"]["center"]["x"], 160)
        self.assertEqual(migrated["stack"]["center"]["y"], 120)
        self.assertFalse(migrated["stack"]["center"]["custom"])

    def test_b2_and_b4_share_one_color_tracker(self):
        manager = TaskManager()
        self.assertIs(
            manager.tasks[TASK_MATERIAL], manager.tasks[TASK_STACK]
        )

    def test_shared_tracker_keeps_lab_and_hsv_configs(self):
        self.assertIn("LAB", DEFAULT_CONFIG["thresholds"])
        self.assertIn("HSV", DEFAULT_CONFIG["thresholds"])

        frame = np.full((240, 320, 3), 255, dtype=np.uint8)
        cv2.rectangle(frame, (110, 80), (210, 180), (0, 0, 255), -1)
        config = copy.deepcopy(DEFAULT_CONFIG)
        manager = TaskManager()
        for task_id in (TASK_MATERIAL, TASK_STACK):
            manager.switch(task_id, 1)
            task_frame = manager.process(frame, config, run_mode=True)
            result = task_frame["result"]
            self.assertTrue(result["found"])
            self.assertEqual(result["cx"], 160)
            self.assertEqual(task_frame["config"]["mode"], config["mode"])
            self.assertIs(
                task_frame["config"]["thresholds"], config["thresholds"]
            )

    def test_b2_and_b4_use_independent_geometry(self):
        frame = np.full((240, 320, 3), 255, dtype=np.uint8)
        cv2.rectangle(frame, (140, 100), (180, 140), (0, 0, 255), -1)
        config = copy.deepcopy(DEFAULT_CONFIG)
        config["center"] = {"x": 100, "y": 120, "custom": True}
        config["stack"]["center"] = {
            "x": 180, "y": 120, "custom": True
        }
        config = validate_config(config)
        manager = TaskManager()

        manager.switch(TASK_MATERIAL, 1)
        material = manager.process(frame, config, run_mode=True)["result"]
        manager.switch(TASK_STACK, 1)
        stack = manager.process(frame, config, run_mode=True)["result"]

        self.assertTrue(material["found"])
        self.assertTrue(stack["found"])
        self.assertGreater(material["dx"], 0)
        self.assertLess(stack["dx"], 0)
        self.assertEqual(config["center"]["x"], 100)
        self.assertEqual(config["stack"]["center"]["x"], 180)


class SafeSerialResultTests(unittest.TestCase):
    def test_b2_and_b4_found_result_is_valid_without_track_state(self):
        self.assertEqual(
            safe_xy_result({"found": True, "dx": 14, "dy": -6}),
            (True, 14, -6),
        )

    def test_b3_only_track_and_found_is_valid(self):
        self.assertEqual(
            safe_xy_result(
                {"found": True, "state": "TRACK", "dx": -7, "dy": 9},
                require_track=True,
            ),
            (True, -7, 9),
        )
        for state in ("SEARCH", "CONFIRM", "LOST"):
            with self.subTest(state=state):
                self.assertEqual(
                    safe_xy_result(
                        {"found": True, "state": state, "dx": 88, "dy": -66},
                        require_track=True,
                    ),
                    (False, 0, 0),
                )

    def test_invalid_frame_never_carries_stale_coordinates(self):
        self.assertEqual(
            safe_xy_result(
                {"found": False, "state": "TRACK", "dx": 123, "dy": -321},
                require_track=True,
            ),
            (False, 0, 0),
        )


if __name__ == "__main__":
    unittest.main()
