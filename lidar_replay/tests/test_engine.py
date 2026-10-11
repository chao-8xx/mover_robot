import copy
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from replay_engine import replay_frame, source_defaults


def make_frame(samples=None, pose=None):
    frame = {"distance": [0] * 360, "valid": [0] * 360, "intensity": [0] * 360,
             "scan_num": 1, "start_time": 100, "end_time": 150,
             "pose": pose or {"x": 0, "y": 0, "yaw_deg": 0, "valid": True}}
    for angle, distance in (samples or {}).items():
        frame["distance"][angle] = distance
        frame["valid"][angle] = 1
        frame["intensity"][angle] = 42
    return frame


class EngineTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.defaults = source_defaults(Path(__file__).resolve().parents[2])

    def params(self, **changes):
        result = copy.deepcopy(self.defaults)
        result.update(changes)
        return result

    def test_source_defaults_and_zero_masks_are_inclusive(self):
        self.assertEqual(self.defaults["wheel_ranges"], [[0, 0]] * 4)
        self.assertFalse(self.defaults["mask_enabled"])
        self.assertEqual(self.defaults["cluster_mode"], "circular")
        result = replay_frame(make_frame({0: 500, 90: 500}), self.params(mask_enabled=True))
        self.assertEqual(result["points"][0]["reason"], "wheel")
        self.assertEqual(result["counts"]["raw_valid"], 2)
        self.assertEqual(result["counts"]["kept"], 1)
        self.assertEqual(sum(result["counts"][key] for key in ("kept", "wheel", "distance", "invalid")), 360)

    def test_coordinate_orientation_and_composed_transform(self):
        params = self.params(mask_enabled=False)
        result = replay_frame(make_frame({0: 1000, 90: 1000, 180: 1000, 270: 1000}), params)
        for angle, xy in ((0, (0, 1000)), (90, (-1000, 0)), (180, (0, -1000)), (270, (1000, 0))):
            for axis, expected in zip(("x", "y"), xy):
                self.assertAlmostEqual(result["points"][angle]["lidar"][axis], expected, delta=0.001)
        params["tf"] = {"tx": 10, "ty": 20, "yaw_deg": 90, "mirror_x": False}
        frame = make_frame({0: 100}, {"x": 1000, "y": 2000, "yaw_deg": 90, "valid": True})
        point = replay_frame(frame, params)["points"][0]
        self.assertAlmostEqual(point["body"]["x"], -90, delta=0.001)
        self.assertAlmostEqual(point["body"]["y"], 20, delta=0.001)
        self.assertAlmostEqual(point["map"]["x"], 980, delta=0.001)
        self.assertAlmostEqual(point["map"]["y"], 1910, delta=0.001)

    def test_filter_precedence_and_inclusive_distance_boundaries(self):
        frame = make_frame({1: 49, 2: 50, 3: 4000, 4: 4001})
        frame["distance"][8] = 1
        params = self.params(mask_enabled=True, wheel_ranges=[[0, 0], [10, 10], [20, 20], [30, 30]])
        result = replay_frame(frame, params)
        self.assertEqual([result["points"][a]["reason"] for a in (0, 1, 2, 3, 4, 8)],
                         ["wheel", "distance", "kept", "kept", "distance", "invalid"])

    def test_missing_angle_threshold_uses_last_kept_point(self):
        params = self.params(mask_enabled=False)
        joined = replay_frame(make_frame({10: 500, 13: 500, 16: 500}), params)
        self.assertEqual(joined["clusters"][0]["angles"], [10, 13, 16])
        broken = replay_frame(make_frame({10: 500, 14: 500, 18: 500}), params)
        self.assertEqual(broken["clusters"], [])
        self.assertEqual(len(broken["rejected"]), 3)
        self.assertTrue(all(c["reason"] == "too_few_points" for c in broken["rejected"]))

    def test_gap_boundary_is_inclusive(self):
        frame = make_frame({0: 50, 180: 50})
        joined = replay_frame(frame, self.params(mask_enabled=False, max_miss=179, min_points=2, gap_mm=100))
        self.assertEqual(joined["clusters"][0]["point_count"], 2)
        broken = replay_frame(frame, self.params(mask_enabled=False, max_miss=179, min_points=2, gap_mm=99.999))
        self.assertEqual(broken["clusters"], [])

    def test_current_circular_cross_zero_collects_before_minimum_filter(self):
        params = self.params(mask_enabled=False)
        joined = replay_frame(make_frame({0: 500, 1: 500, 358: 500, 359: 500}), params)
        self.assertEqual(len(joined["clusters"]), 1)
        self.assertEqual(joined["clusters"][0]["angles"], [358, 359, 0, 1])
        self.assertEqual(joined["clusters"][0]["point_count"], 4)
        joined = replay_frame(make_frame({0: 500, 1: 500, 2: 500, 357: 500, 358: 500, 359: 500}), params)
        self.assertEqual(len(joined["clusters"]), 1)
        cluster = joined["clusters"][0]
        self.assertEqual((cluster["start_angle"], cluster["end_angle"], cluster["point_count"]), (357, 2, 6))
        self.assertEqual(cluster["angles"], [357, 358, 359, 0, 1, 2])

    def test_legacy_cross_zero_mode_preserves_old_minimum_before_merge(self):
        params = self.params(mask_enabled=False, cluster_mode="linear_merge")
        rejected = replay_frame(make_frame({0: 500, 1: 500, 358: 500, 359: 500}), params)
        self.assertEqual(rejected["clusters"], [])
        self.assertEqual([c["point_count"] for c in rejected["rejected"]], [2, 2])
        joined = replay_frame(make_frame({0: 500, 1: 500, 2: 500, 357: 500, 358: 500, 359: 500}), params)
        self.assertEqual(len(joined["clusters"]), 1)
        cluster = joined["clusters"][0]
        self.assertEqual((cluster["start_angle"], cluster["end_angle"], cluster["point_count"]), (357, 2, 6))
        self.assertEqual(cluster["angles"], [357, 358, 359, 0, 1, 2])

    def test_wrapping_mask_and_capacity_diagnostics(self):
        params = self.params(mask_enabled=True, wheel_ranges=[[350, 5], [100, 100], [200, 200], [300, 300]], max_clusters=1)
        samples = {a: 500 for a in [0, 5, 350, 359, 10, 11, 12, 30, 31, 32]}
        result = replay_frame(make_frame(samples), params)
        for angle in (0, 5, 350, 359):
            self.assertEqual(result["points"][angle]["reason"], "wheel")
        self.assertEqual(len(result["clusters"]), 1)
        self.assertEqual(result["rejected"][0]["reason"], "capacity")
        self.assertEqual(result["rejected"][0]["angles"], [30, 31, 32])
        self.assertIsNone(result["points"][30]["cluster_id"])

    def test_invalid_pose_and_exploratory_mirror_are_marked(self):
        frame = make_frame({90: 500}, {"x": 0, "y": 0, "yaw_deg": 0, "valid": False})
        frame["scan_num"] = 0
        params = self.params(mask_enabled=False)
        params["tf"]["mirror_x"] = True
        result = replay_frame(frame, params)
        self.assertGreater(result["points"][90]["body"]["x"], 0)
        self.assertEqual(len(result["warnings"]), 3)

    def test_recompute_does_not_accumulate_state(self):
        frame = make_frame({10: 500, 11: 500, 12: 500})
        result = replay_frame(frame, self.defaults)
        self.assertEqual(result, replay_frame(frame, self.defaults))
        modified = self.params(min_points=4)
        self.assertEqual(replay_frame(frame, modified)["clusters"], [])

    def test_bad_params_and_arrays_fail_clearly(self):
        for changes in ({"min_distance": 5000}, {"max_miss": 255}, {"min_points": 0}, {"gap_mm": float("nan")}, {"wheel_ranges": [[0, 361]] * 4}):
            with self.assertRaises(ValueError):
                replay_frame(make_frame(), self.params(**changes))
        frame = make_frame()
        frame["valid"] = [0] * 359
        with self.assertRaises(ValueError):
            replay_frame(frame, self.defaults)
        frame = make_frame({10: 500})
        frame["distance"][10] = -1
        with self.assertRaises(ValueError):
            replay_frame(frame, self.defaults)


if __name__ == "__main__":
    unittest.main()
