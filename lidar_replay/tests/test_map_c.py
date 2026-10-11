"""Differential checks against the project's unchanged C map implementation.

No HAL is needed: map_update.c, map_data.c and track_map.c are portable C.
Set LIDAR_REPLAY_CC to a host GCC/Clang path when it is not on PATH. The tests
skip explicitly if no compiler exists; successful Python tests alone do not
claim C parity. Builds and binaries remain in a temporary directory.
"""

import ctypes
import os
from pathlib import Path
import random
import shutil
import subprocess
import sys
import tempfile
import unittest

TOOL_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOL_ROOT))
from map_engine import MapReplay, map_layout
from test_map import cluster


class CScanObs(ctypes.Structure):
    _fields_ = [(name, ctypes.c_float) for name in
                ("center_x", "center_y", "min_x", "min_y", "max_x", "max_y")]


class CMapObs(ctypes.Structure):
    _fields_ = [(name, ctypes.c_float) for name in
                ("center_x", "center_y", "min_x", "min_y", "max_x", "max_y")] + [
        ("last_scan_num", ctypes.c_uint32), ("last_tick", ctypes.c_uint32),
        ("seen_num", ctypes.c_uint8), ("miss_num", ctypes.c_uint8),
        ("state", ctypes.c_int),
    ]


def host_compiler():
    for name in (os.environ.get("LIDAR_REPLAY_CC"), shutil.which("gcc"), shutil.which("clang")):
        if name and Path(name).is_file():
            return str(Path(name).resolve())
    # The installed CLion runtime also supplies the host GCC on this machine.
    for path in Path("D:/CLion").glob("CLion*/bin/mingw/bin/gcc.exe"):
        return str(path)
    return None


class CParityTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        compiler = host_compiler()
        if compiler is None:
            raise unittest.SkipTest("C map parity not run: no host GCC/Clang found")
        cls.temp = tempfile.TemporaryDirectory(prefix="lidar_map_parity_")
        root = TOOL_ROOT.parent / "gongxun_1"
        cls.layout = map_layout(root)
        target = Path(cls.temp.name) / ("map_parity.dll" if os.name == "nt" else "map_parity.so")
        command = [compiler, "-shared", "-O0", "-std=c99", "-static-libgcc"]
        if os.name != "nt":
            command.append("-fPIC")
        command += ["-I", str(root / "APP/Inc")]
        command += [str(root / "APP/Src" / name) for name in ("map_update.c", "map_data.c", "track_map.c")]
        command += ["-lm", "-o", str(target)]
        environment = os.environ.copy()
        # cc1 lives below libexec and needs the compiler bin directory to find
        # the bundled runtime DLLs when CLion's terminal environment is absent.
        environment["PATH"] = str(Path(compiler).parent) + os.pathsep + environment.get("PATH", "")
        compiled = subprocess.run(command, capture_output=True, text=True, timeout=60, env=environment)
        if compiled.returncode:
            cls.temp.cleanup()
            raise RuntimeError("C map parity build failed:\n" + compiled.stderr)
        cls.lib = ctypes.CDLL(str(target))
        cls.lib.map_update_init.argtypes = [ctypes.c_float, ctypes.c_float]
        cls.lib.map_update_init.restype = ctypes.c_uint8
        cls.lib.map_update_scan.argtypes = [ctypes.POINTER(CScanObs), ctypes.c_uint8, ctypes.c_uint32, ctypes.c_uint32]
        cls.lib.map_update_scan.restype = ctypes.c_uint8
        cls.lib.map_obs_get.argtypes = [ctypes.c_uint8, ctypes.POINTER(CMapObs)]
        cls.lib.map_obs_get.restype = ctypes.c_uint8
        cls.lib.map_road_get.argtypes = [ctypes.c_uint8, ctypes.POINTER(ctypes.c_int)]
        cls.lib.map_road_get.restype = ctypes.c_uint8
        cls.lib.map_update_change_get.restype = ctypes.c_uint32
        cls.lib.map_check_area.argtypes = [ctypes.c_float, ctypes.c_float]
        cls.lib.map_check_area.restype = ctypes.c_int
        cls.lib.map_xy_to_cell.argtypes = [ctypes.c_float, ctypes.c_float, ctypes.POINTER(ctypes.c_uint8), ctypes.POINTER(ctypes.c_uint8)]
        cls.lib.map_xy_to_cell.restype = ctypes.c_uint8

    @classmethod
    def tearDownClass(cls):
        # Windows locks a loaded DLL. Release this test-only library before
        # TemporaryDirectory removes its own build output.
        import _ctypes
        handle = cls.lib._handle
        if os.name == "nt":
            _ctypes.FreeLibrary(handle)
        else:
            _ctypes.dlclose(handle)
        cls.lib = None
        cls.temp.cleanup()

    def reset(self, width=300, safety=20):
        self.assertEqual(self.lib.map_update_init(width, safety), 1)
        self.model = MapReplay(width, safety, layout=self.layout)

    def check_scan(self, clusters, scan, tick):
        result = self.model.apply(clusters, scan, tick)
        valid = [item for item in clusters if item.get("valid", True)]
        records = []
        for item in valid:
            point = item["center_point"]
            records.append(CScanObs(point["x"], point["y"], item["min_x"], item["min_y"], item["max_x"], item["max_y"]))
        array = (CScanObs * len(records))(*records)
        updated = self.lib.map_update_scan(array, len(records), scan, tick)
        self.assertEqual(result["updated"], bool(updated), (scan, result["reason"]))
        self.assertEqual(result["change_num"], self.lib.map_update_change_get())
        names = {0: "unknown", 1: "free", 2: "blocked"}
        for road in result["roads"]:
            state = ctypes.c_int()
            self.assertEqual(self.lib.map_road_get(road["id"], ctypes.byref(state)), 1)
            self.assertEqual(road["state"], names[state.value], (scan, road))
        python_records = {obs["id"]: obs for obs in result["obstacles"]}
        for identity in range(self.layout["parameters"]["obstacle_max"]):
            c_obs = CMapObs()
            self.assertEqual(self.lib.map_obs_get(identity, ctypes.byref(c_obs)), 1)
            if c_obs.state == 0:
                self.assertNotIn(identity, python_records, (scan, identity))
                continue
            self.assertIn(identity, python_records, (scan, identity))
            python_obs = python_records[identity]
            self.assertEqual(python_obs["state"], {1: "candidate", 2: "confirmed"}[c_obs.state])
            for name, _ in CMapObs._fields_:
                if name != "state":
                    self.assertEqual(python_obs[name], getattr(c_obs, name), (scan, identity, name))
        return result

    def test_exact_confirmation_gaps_duplicates_and_persistence(self):
        self.reset(10, 0)
        self.check_scan([cluster(600, 275, 0), cluster(800, 275, 1)], 1, 0)
        self.check_scan([cluster(700, 275), cluster(715, 275, 1)], 2, 5)
        self.check_scan([], 3, 10)
        self.check_scan([cluster(720, 275), cluster(1200, 1200, 1)], 5, 20)
        self.check_scan([cluster(1200, 1200)], 6, 25)
        self.check_scan([], 50000, 1000000000)
        self.check_scan([cluster()], 50000, 1000000001)

    def test_dense_capacity_replacement_and_counter_saturation(self):
        self.reset(10, 0)
        points = [cluster(300 + i * 45, 275, i, half=1) for i in range(20)]
        self.check_scan(points, 1, 0)
        self.check_scan([cluster(2000, 1200)], 2, 10)
        self.check_scan(points, 3, 20)
        for scan in range(4, 265):
            self.check_scan(points, scan, scan)
        self.check_scan([cluster(2000, 1200)], 265, 265)

    def test_inflation_and_matching_numeric_edges(self):
        self.reset(100, 10)
        radius = self.model.safe_radius
        boxes = [cluster(700, 275 + radius, 0, half=0),
                 cluster(2000, 1200 + radius + 0.01, 1, half=0)]
        self.check_scan(boxes, 1, 0)
        self.check_scan([cluster(820, 275 + radius, half=0)], 2, 10)
        self.check_scan([cluster(940.001, 275 + radius, half=0)], 3, 20)

    def test_seeded_mixed_frames_match_all_c_slots(self):
        rng = random.Random(20261003)
        self.reset(320, 25)
        scan = 0
        for frame in range(160):
            scan += rng.choice((1, 1, 1, 2, 3))
            points = []
            for identity in range(rng.randrange(0, 20)):
                x, y = rng.choice((275, 1200, 2125)), rng.uniform(0, 2399)
                if rng.choice((True, False)):
                    x, y = y, x
                x += rng.uniform(-100, 100)
                y += rng.uniform(-100, 100)
                points.append(cluster(x, y, identity, half=rng.uniform(0, 30)))
            if frame % 11 == 0:
                points.append(cluster(1000, 700, 100))
            if frame % 17 == 0 and points:
                points[0]["center_point"]["x"] = float("nan")
            self.check_scan(points[:20], scan, frame * 100)

    def test_cell_and_area_boundary_parity(self):
        self.reset()
        values = [-0.01, 0, 549.99, 550, 999.99, 1000, 1399.99,
                  1400, 1849.99, 1850, 2399.99, 2400, float("nan")]
        for x in values:
            for y in values:
                with self.subTest(x=x, y=y):
                    cx, cy = ctypes.c_uint8(), ctypes.c_uint8()
                    valid = self.lib.map_xy_to_cell(x, y, ctypes.byref(cx), ctypes.byref(cy))
                    cell = self.model.cell_at(x, y)
                    self.assertEqual(bool(valid), cell is not None)
                    if cell is not None:
                        self.assertEqual((cx.value, cy.value), (cell["x"], cell["y"]))
                    box = self.model._box(cluster(x, y))
                    reason = self.model._box_reason(box)
                    area = self.lib.map_check_area(x, y)
                    expected = 0 if cell is None else 2 if reason == "static_forbidden" else 1
                    self.assertEqual(area, expected)


if __name__ == "__main__":
    unittest.main()
