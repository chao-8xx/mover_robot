"""NOW版本电脑回归：合成图算法、配置、调度及持续串口解析。"""
import ast
import copy
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
from vision_config import DEFAULT_CONFIG, VisionConfigStore, validate_config
from vision_ring import detect_ring_candidates, make_ring_preview
from vision_tasks import (
    RingTask, TaskManager, TASK_MATERIAL, TASK_RING, TASK_STACK, safe_xy_result,
)
from vision_runtime import ResultSender, PerfStats, pack_xy_frame


def config():
    return validate_config(copy.deepcopy(DEFAULT_CONFIG))


def midpoint_bgr(cfg, key):
    threshold = cfg["thresholds"][cfg["mode"]][key]
    lower, upper = threshold["lower"], threshold["upper"]
    values = [(lo + hi) // 2 for lo, hi in zip(lower, upper)]
    if cfg["mode"] == "HSV" and lower[0] > upper[0]:
        values[0] = 0
    conversion = cv2.COLOR_LAB2BGR if cfg["mode"] == "LAB" else cv2.COLOR_HSV2BGR
    return tuple(int(v) for v in cv2.cvtColor(
        np.uint8([[values]]), conversion)[0, 0])


def color_frame(cfg, colors=("red", "yellow", "blue")):
    frame = np.full((240, 320, 3), 255, np.uint8)
    for key, cx in zip(("red", "yellow", "blue"), (35, 160, 285)):
        if key in colors:
            cv2.rectangle(frame, (cx - 25, 95), (cx + 25, 145),
                          midpoint_bgr(cfg, key), -1)
    return frame


def ring_frame(centers=((40, 120), (160, 120), (280, 120)), axes=None, angle=0):
    frame = np.full((240, 320, 3), 255, np.uint8)
    for center in centers:
        # 小到单像素的圆周在斜向射线会混叠；使用可解析的环带样本。
        for radius in (32, 27, 22, 17):
            if axes is None:
                cv2.circle(frame, center, radius, (0, 0, 0), 2)
            else:
                cv2.ellipse(frame, center, (radius, int(radius * axes)),
                            angle, 0, 360, (0, 0, 0), 2)
    return frame


def ring_config():
    cfg = config()
    cfg["ring"]["min_diameter"] = 24
    cfg["ring"]["min_contour_area"] = 45
    return cfg


class ConfigTests(unittest.TestCase):
    def test_missing_area_fields_use_single_default_config(self):
        value = validate_config({})
        self.assertEqual(value["blob"], DEFAULT_CONFIG["blob"])
        self.assertEqual(value["stack"]["blob"], DEFAULT_CONFIG["stack"]["blob"])

    def test_old_small_b2_roi_migrates_full_and_preserves_absolute_center(self):
        cfg = config()
        cfg["roi"] = {"x": 110, "y": 25, "w": 135, "h": 135}
        cfg["center"] = {"x": 60, "y": 70, "custom": True}
        original_thresholds = copy.deepcopy(cfg["thresholds"])
        migrated = validate_config(cfg)
        self.assertEqual(migrated["roi"], dict(x=0, y=0, w=320, h=240))
        self.assertEqual(migrated["center"], dict(x=170, y=95, custom=True))
        self.assertEqual(validate_config(migrated), migrated)
        self.assertEqual(migrated["thresholds"], original_thresholds)

    def test_default_store_returns_normalized_stack_center(self):
        with tempfile.TemporaryDirectory() as path:
            value, _ = VisionConfigStore(path).load()
        self.assertEqual(value["stack"]["center"], dict(x=67, y=67, custom=False))

    def test_save_load_preserves_calibration_and_separate_ring_geometry(self):
        cfg = config()
        cfg["ring"]["roi"] = dict(x=100, y=60, w=120, h=120)
        cfg["ring"]["center"] = dict(x=59, y=58, custom=True)
        with tempfile.TemporaryDirectory() as path:
            store = VisionConfigStore(path, "vision_now_config.json")
            saved, _ = store.save(cfg)
            loaded, _ = store.load()
            self.assertEqual(loaded, saved)
        self.assertEqual(loaded["blob"], DEFAULT_CONFIG["blob"])
        self.assertEqual(loaded["ring"]["center"], dict(x=59, y=58, custom=True))


class ColorTests(unittest.TestCase):
    def test_three_colors_one_conversion_in_both_spaces_and_sizes(self):
        for mode in ("LAB", "HSV"):
            for run in (True, False):
                with self.subTest(mode=mode, run=run):
                    cfg = config()
                    cfg["mode"] = mode
                    manager = TaskManager()
                    manager.switch(TASK_MATERIAL, 0xC1)
                    with patch.object(vision_blob, "convert_color_space",
                                      wraps=vision_blob.convert_color_space) as convert:
                        value = manager.process(color_frame(cfg), cfg, run, False)
                    self.assertEqual(convert.call_count, 1)
                    self.assertEqual(convert.call_args[0][0].shape[:2],
                                     (180, 240) if run else (240, 320))
                    result = value["result"]
                    self.assertEqual(result["color_count"], 3)
                    for key, cx in zip(("red", "yellow", "blue"), (35, 160, 285)):
                        detected = result["by_color"][key]
                        self.assertTrue(detected["found"], (key, detected))
                        self.assertAlmostEqual(detected["cx"], cx, delta=2)
                        self.assertAlmostEqual(detected["cy"], 120, delta=2)
                        self.assertAlmostEqual(detected["dx"], cx - 160, delta=2)
                        self.assertAlmostEqual(detected["area"], 2500, delta=200)
                    self.assertIsNone(value["mask_roi"])

    def test_selected_color_controls_result_not_other_visible_colors(self):
        cfg = config()
        cfg["target_color"] = "blue"
        manager = TaskManager()
        manager.switch(TASK_MATERIAL, 0xC3)
        result = manager.process(color_frame(cfg, ("red", "yellow")), cfg, True)["result"]
        self.assertTrue(result["by_color"]["red"]["found"])
        self.assertFalse(result["found"])
        self.assertEqual(safe_xy_result(result), (False, 0, 0))
        result = manager.process(color_frame(cfg), cfg, True)["result"]
        self.assertEqual(result["dx"], result["by_color"]["blue"]["dx"])

    def test_b2_full_roi_even_if_unvalidated_legacy_roi_is_passed(self):
        cfg = config()
        cfg["roi"] = dict(x=100, y=60, w=120, h=120)
        manager = TaskManager()
        manager.switch(TASK_MATERIAL, 1)
        value = manager.process(color_frame(cfg), cfg, True)
        self.assertEqual(value["roi_rect"], (0, 0, 320, 240))
        self.assertTrue(all(v["found"] for v in value["result"]["by_color"].values()))

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

    def test_geometric_confirmation_without_ocr_even_blank_center(self):
        for index in range(3):
            value = self.task.process(ring_frame(), self.cfg)["result"]
            self.assertEqual(value["found"], index == 2)
            self.assertEqual(value["recognized_digit"], 0)
            self.assertEqual(value["ocr_calls"], 0)
        self.assertEqual((value["cx"], value["cy"]), (160, 120))
        self.assertNotIn("vision_ocr", sys.modules)

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

    def test_invalid_length_recovers_and_switches_selected_color(self):
        self.feed(b"\xAA\x55\xB2\xFF" + self.frame(0xB4, [3]))
        self.assertEqual((self.env["mode"], self.env["target"]), (0xB4, 0xC3))


if __name__ == "__main__":
    unittest.main()
