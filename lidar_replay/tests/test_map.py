"""Independent behavioural tests for the current road-level map replay."""

import json
from pathlib import Path
import shutil
import sys
import tempfile
import unittest

TOOL_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOL_ROOT))
from map_engine import MapReplay, MapSourceError, map_layout

REPO_ROOT = TOOL_ROOT.parent


def cluster(x=700, y=275, identity=0, half=5):
    return {"id": identity, "valid": True, "center_point": {"x": x, "y": y},
            "min_x": x - half, "max_x": x + half,
            "min_y": y - half, "max_y": y + half}


class MapTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.layout = map_layout(REPO_ROOT)

    def new(self, width=10, safety=0):
        return MapReplay(width, safety, layout=self.layout)

    def test_layout_matches_live_track_and_serializes(self):
        layout = self.layout
        self.assertEqual((layout["width"], layout["height"]), (2400, 2400))
        self.assertEqual(layout["boundaries"], [0, 550, 1000, 1400, 1850, 2400])
        self.assertEqual(layout["nodes"][0], {"id": 0, "x": 275, "y": 2125})
        self.assertEqual(layout["nodes"][8], {"id": 8, "x": 2125, "y": 275})
        self.assertEqual(layout["roads"][8], {"id": 8, "node_a": 1, "node_b": 4})
        self.assertEqual(layout["cells"][0][0], {"cell_type": "node", "cell_id": 6})
        self.assertEqual(layout["cells"][3][2], {"cell_type": "road", "cell_id": 8})
        self.assertEqual(layout["cells"][1][1], {"cell_type": "forbidden", "cell_id": None})
        self.assertEqual(len(layout["source_files"]), 7)
        json.dumps(layout, allow_nan=False)
        json.dumps(self.new().apply([], 1, 0), allow_nan=False)

    def test_reads_current_source_instead_of_hardcoded_layout(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            for source in self.layout["source_files"]:
                old = Path(source["path"])
                relative = old.relative_to(REPO_ROOT / "gongxun_1")
                new = root / relative
                new.parent.mkdir(parents=True, exist_ok=True)
                shutil.copyfile(old, new)
            source = root / "APP/Src/track_map.c"
            source.write_text(source.read_text(encoding="utf-8").replace("{275.0f,", "{300.0f,"), encoding="utf-8")
            header = root / "APP/Inc/map_update.h"
            header.write_text(header.read_text(encoding="utf-8").replace("MAP_CONFIRM_SCAN_NUM    2", "MAP_CONFIRM_SCAN_NUM    3"), encoding="utf-8")
            current = map_layout(root)
            self.assertEqual(current["nodes"][0]["x"], 300)
            self.assertEqual(current["parameters"]["confirm_scan_num"], 3)
            source.write_text(source.read_text(encoding="utf-8").replace("map_nodes[", "unsupported_nodes["), encoding="utf-8")
            with self.assertRaises(MapSourceError):
                map_layout(root)

    def test_missing_firmware_fails_clearly(self):
        with tempfile.TemporaryDirectory() as temp:
            with self.assertRaisesRegex(MapSourceError, "Cannot find"):
                map_layout(Path(temp))

    def test_cell_boundaries_and_forbidden_edges_differ(self):
        model = self.new()
        self.assertEqual(model.cell_at(549.99, 275)["x"], 0)
        self.assertEqual(model.cell_at(550, 275)["x"], 1)
        self.assertEqual(model.cell_at(1000, 275)["x"], 2)
        self.assertEqual(model.cell_at(2399.9, 2399.9)["x"], 4)
        self.assertIsNone(model.cell_at(2400, 0))
        self.assertIsNone(model.cell_at(-0.01, 0))
        # Static rectangles include their upper edge; cell partition uses <.
        result = model.apply([cluster(1000, 700)], 1, 0)
        self.assertEqual(result["candidates"][0]["cell"]["type"], "road")
        self.assertEqual(result["candidates"][0]["reason"], "static_forbidden")

    def test_two_sightings_confirm_only_affected_road(self):
        model = self.new()
        first = model.apply([cluster()], 1, 10)
        self.assertEqual(first["obstacles"][0]["state"], "candidate")
        self.assertTrue(all(road["state"] == "unknown" for road in first["roads"]))
        second = model.apply([cluster(705, 275)], 2, 20)
        self.assertEqual(second["obstacles"][0]["seen_num"], 2)
        self.assertEqual(second["obstacles"][0]["state"], "confirmed")
        self.assertEqual([road["id"] for road in second["roads"] if road["state"] == "blocked"], [4])
        self.assertEqual(second["change_num"], 1)
        self.assertEqual(model.apply([cluster()], 3, 30)["change_num"], 1)

    def test_same_scan_id_cannot_confirm(self):
        model = self.new()
        first = model.apply([cluster()], 10, 1)
        again = model.apply([cluster()], 10, 2)
        self.assertFalse(again["updated"])
        self.assertEqual(again["reason"], "stale_scan")
        self.assertEqual(again["obstacles"], first["obstacles"])

    def test_same_scan_duplicate_cannot_confirm(self):
        model = self.new()
        result = model.apply([cluster(), cluster(720, 275, 1)], 1, 0)
        self.assertEqual(len(result["obstacles"]), 1)
        self.assertEqual(result["obstacles"][0]["seen_num"], 1)
        self.assertEqual(result["candidates"][1]["reason"], "duplicate_this_scan")

    def test_one_missed_scan_keeps_candidate_and_allows_confirmation(self):
        model = self.new()
        model.apply([cluster()], 1, 0)
        missed = model.apply([], 2, 10)
        self.assertEqual(missed["obstacles"][0]["miss_num"], 1)
        result = model.apply([cluster()], 3, 20)
        self.assertEqual(result["obstacles"][0]["state"], "confirmed")
        self.assertEqual(result["obstacles"][0]["miss_num"], 0)

    def test_two_misses_clear_candidate(self):
        model = self.new()
        model.apply([cluster()], 1, 0)
        model.apply([], 2, 10)
        result = model.apply([], 3, 20)
        self.assertEqual(result["obstacles"], [])
        self.assertTrue(all(road["state"] == "unknown" for road in result["roads"]))

    def test_skipped_scans_count_before_matching(self):
        model = self.new()
        model.apply([cluster()], 1, 0)
        result = model.apply([cluster()], 4, 30)
        self.assertEqual(result["obstacles"][0]["seen_num"], 1)
        self.assertEqual(result["obstacles"][0]["state"], "candidate")
        model = self.new()
        model.apply([cluster()], 1, 0)
        result = model.apply([cluster()], 3, 20)
        self.assertEqual(result["obstacles"][0]["state"], "confirmed")

    def test_confirmed_obstacles_never_autoclear(self):
        model = self.new()
        model.apply([cluster()], 1, 0)
        confirmed = model.apply([cluster()], 2, 10)
        missing = model.apply([], 50000, 1000000000)
        self.assertEqual(missing["obstacles"], confirmed["obstacles"])
        self.assertEqual(missing["roads"], confirmed["roads"])

    def test_invalid_pose_preserves_map_without_inventing_free(self):
        model = self.new()
        model.apply([cluster()], 1, 0)
        before = model.apply([cluster()], 2, 10)
        ignored = model.apply([cluster(1200, 1200)], 3, 20, pose_valid=False)
        self.assertEqual(ignored["reason"], "pose_invalid")
        self.assertFalse(ignored["updated"])
        self.assertEqual(ignored["roads"], before["roads"])
        self.assertEqual(ignored["obstacles"], before["obstacles"])
        self.assertNotIn("free", [road["state"] for road in ignored["roads"]])
        # Ignoring pose did not consume the scan ID.
        self.assertTrue(model.apply([], 3, 20)["updated"])

    def test_geometry_rejection_reasons(self):
        model = self.new()
        malformed = cluster()
        malformed["center_point"]["x"] = float("nan")
        out = cluster(2400, 275, 1)
        forbidden = cluster(700, 700, 2)
        no_road = cluster(50, 50, 3)
        filtered = cluster(identity=4)
        filtered["valid"] = False
        result = model.apply([malformed, out, forbidden, no_road, filtered], 1, 0)
        self.assertEqual([item["reason"] for item in result["candidates"]],
                         ["invalid_coordinates", "outside_field", "static_forbidden", "no_affected_road", "cluster_rejected"])
        json.dumps(result, allow_nan=False)

    def test_inflation_hits_inclusive_road_boundary(self):
        model = self.new(width=100, safety=10)
        radius = model.safe_radius
        point = cluster(700, 275 + radius, half=0)
        result = model.apply([point], 1, 0)
        self.assertTrue(result["candidates"][0]["accepted"])
        point = cluster(700, 275 + radius + 0.01, half=0)
        self.assertEqual(self.new(width=100, safety=10).apply([point], 1, 0)["candidates"][0]["reason"], "no_affected_road")

    def test_node_intersection_hits_only_four_incident_roads(self):
        model = self.new()
        model.apply([cluster(1200, 1200)], 1, 0)
        result = model.apply([cluster(1200, 1200)], 2, 10)
        self.assertEqual(result["obstacles"][0]["road_ids"], [2, 3, 8, 9])
        self.assertEqual(result["change_num"], 4)

    def test_nearest_match_and_ties_follow_firmware_order(self):
        model = self.new()
        model.apply([cluster(600, 275, 0), cluster(800, 275, 1)], 1, 0)
        result = model.apply([cluster(700, 275)], 2, 10)
        self.assertEqual(result["candidates"][0]["obstacle_id"], 1)
        self.assertEqual(result["obstacles"][0]["miss_num"], 1)
        self.assertEqual(result["obstacles"][1]["state"], "confirmed")

    def test_confirmed_slots_not_replaced_at_capacity(self):
        model = self.new()
        many = [cluster(300 + i * 45, 275, i, half=1) for i in range(20)]
        model.apply(many, 1, 0)
        model.apply(many, 2, 10)
        result = model.apply([cluster(2000, 1200)], 3, 20)
        self.assertEqual(len(result["obstacles"]), 20)
        self.assertEqual(result["candidates"][0]["reason"], "obstacle_capacity")
        self.assertTrue(all(obs["state"] == "confirmed" for obs in result["obstacles"]))

    def test_invalid_config_and_oversized_scan_leave_state_unchanged(self):
        for car, safety in ((0, 0), (-1, 0), (10, -1), (float("inf"), 0), (2401, 0)):
            with self.subTest(car=car, safety=safety):
                result = self.new(car, safety).apply([cluster()], 1, 0)
                self.assertFalse(result["ready"])
                self.assertFalse(result["updated"])
        model = self.new()
        model.apply([cluster()], 1, 0)
        result = model.apply([cluster(identity=i) for i in range(21)], 2, 10)
        self.assertFalse(result["updated"])
        self.assertEqual(result["reason"], "too_many_clusters")
        self.assertEqual(result["obstacles"][0]["seen_num"], 1)

    def test_seen_counter_saturates(self):
        model = self.new()
        for scan in range(1, 261):
            result = model.apply([cluster()], scan, scan)
        self.assertEqual(result["obstacles"][0]["seen_num"], 255)
        self.assertEqual(result["change_num"], 1)


if __name__ == "__main__":
    unittest.main()
