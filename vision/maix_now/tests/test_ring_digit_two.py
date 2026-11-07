"""B3数字2、ROI扩展保持对准点及串口链路验证；不代表真机验收。"""
import copy
import tempfile
import unittest
from unittest.mock import patch

import cv2
import numpy as np

from test_vision_v9 import config, ring_config, ring_frame
from vision_config import DEFAULT_CONFIG, VisionConfigStore, validate_config
from vision_tasks import RingTask, TaskManager, TASK_RING, TASK_STACK, safe_xy_result
from vision_runtime import ResultSender, pack_xy_frame
from vision_ocr import confirm_digit_two


class DigitTwoTests(unittest.TestCase):
    def test_two_passes_but_one_three_and_blank_do_not(self):
        for digit in (None, 1, 2, 3):
            frame = ring_frame(((160, 120),), digits=digit)
            decision = confirm_digit_two(frame, dict(cx=160, cy=120, radius=32))
            self.assertEqual(decision["is_two"], digit == 2, (digit, decision["reason"]))
            self.assertEqual(decision["digit"], 2 if digit == 2 else 0)
            task = RingTask(); task.enter(0xD2)
            for _ in range(14):
                result = task.process(frame, ring_config())["result"]
            self.assertEqual(result["found"], digit == 2)

    def test_nearest_non_two_does_not_hide_other_two(self):
        cfg = ring_config()
        cfg["ring"]["center"] = dict(x=90, y=120, custom=True)
        frame = ring_frame(((90, 120),), digits=1)
        other = ring_frame(((210, 120),), digits=2)
        frame = np.minimum(frame, other)
        task = RingTask(); task.enter(0xD2)
        for _ in range(3): result = task.process(frame, cfg)["result"]
        self.assertTrue(result["found"])
        self.assertEqual((result["cx"], result["dx"]), (210, 120))
        self.assertEqual(result["ocr_calls"], 2)

    def test_confirmed_track_skips_ocr_and_loss_does_not_send_old_position(self):
        task = RingTask(); task.enter(0xD2)
        cfg = ring_config()
        for _ in range(3): task.process(ring_frame(), cfg)
        with patch("vision_tasks.confirm_digit_two", side_effect=AssertionError("tracking OCR")):
            tracked = task.process(ring_frame(((165, 120),)), cfg)["result"]
            self.assertEqual(tracked["ocr_calls"], 0)
            self.assertEqual(tracked["dx"], 5)
            lost = task.process(ring_frame(()), cfg)["result"]
            self.assertEqual(safe_xy_result(lost, True), (0, 0, 0))
        for _ in range(14): task.process(ring_frame(()), cfg)
        self.assertFalse(task.target_confirmed)
        for _ in range(3): result = task.process(ring_frame(), cfg)["result"]
        self.assertEqual(result["recognized_digit"], 2)
        self.assertGreater(result["ocr_calls"], 0)

    def test_retry_is_throttled_and_task_switch_forgets_confirmation(self):
        task = RingTask(); task.enter(0xD2)
        calls = [task.process(ring_frame(digits=3), ring_config())["result"]["ocr_calls"]
                 for _ in range(12)]
        self.assertEqual([i for i,c in enumerate(calls) if c], [2, 11])
        manager = TaskManager(); manager.switch(TASK_RING, 0xD2)
        for _ in range(3): manager.process(ring_frame(), ring_config())
        manager.switch(TASK_STACK, 1); manager.switch(TASK_RING, 0xD2)
        result = manager.process(ring_frame(), ring_config())["result"]
        self.assertEqual(result["recognized_digit"], 0)
        self.assertFalse(result["found"])

    def test_default_and_legacy_expansion_preserve_actual_reference(self):
        cfg = validate_config(copy.deepcopy(DEFAULT_CONFIG))
        r,c = cfg["ring"]["roi"],cfg["ring"]["center"]
        self.assertEqual(r, dict(x=40,y=20,w=240,h=200))
        self.assertEqual((r["x"]+c["x"],r["y"]+c["y"]), (173,114))
        for custom in (False, True):
            old = config(); old["schema_version"] = 13
            old["ring"]["roi"] = dict(x=80,y=40,w=145,h=150)
            old["ring"]["center"] = dict(x=93,y=74,custom=custom)
            expected = (173,114) if custom else (152,115)
            migrated = validate_config(old)
            r,c = migrated["ring"]["roi"],migrated["ring"]["center"]
            self.assertEqual((r["x"]+c["x"],r["y"]+c["y"]), expected)
            for key in old.keys()-{"ring","schema_version"}:
                self.assertEqual(migrated[key],old[key])
            for key in old["ring"].keys()-{"roi","center"}:
                self.assertEqual(migrated["ring"][key],old["ring"][key])
            with tempfile.TemporaryDirectory() as path:
                store = VisionConfigStore(path); saved,_ = store.save(migrated)
                self.assertEqual(store.load()[0],saved)
            self.assertEqual(validate_config(migrated),migrated)

    def test_ui_and_run_keep_same_reference_and_signed_protocol(self):
        cfg = ring_config(); cfg["ring"]["roi"] = dict(x=40,y=20,w=240,h=200)
        cfg["ring"]["center"] = dict(x=133,y=94,custom=True)
        for run in (False,True):
            task = RingTask(); task.enter(0xD2)
            for _ in range(3): result = task.process(ring_frame(((163,120),)),cfg,run)["result"]
            self.assertTrue(result["found"])
            self.assertEqual((result["dx"],result["dy"]),(-10,6))
            sender = ResultSender()
            tx = sender.take(result,100,100,require_track=True)
            packet = pack_xy_frame(0xB3,tx["frame_id"],tx["raw_ok"],tx["dx"],tx["dy"])
            self.assertEqual(packet[5:10],bytes([1,246,255,6,0]))
            self.assertEqual(packet[-1],sum(packet[2:-1])&255)
            stale = sender.take(result,100,160,require_track=True)
            self.assertEqual((stale["raw_ok"],stale["dx"],stale["dy"]),(0,0,0))


if __name__ == "__main__":
    unittest.main()
