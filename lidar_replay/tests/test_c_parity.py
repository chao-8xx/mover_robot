"""Differential checks against unchanged firmware C on an optional host GCC.

Run with: python -m unittest discover -s tests -p test_c_parity.py -v
Set LIDAR_HOST_CC to a host gcc/clang executable if it is not on PATH.
Missing compilers cause explicit skips; ARM cross compilers cannot run this test.
"""
import copy
import json
import os
from pathlib import Path
import random
import shutil
import subprocess
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from replay_engine import firmware_dir, replay_frame, source_defaults
try:
    from .test_engine import make_frame
except ImportError:
    from test_engine import make_frame


HARNESS = r'''
#include "headfile.h"
#include <stdio.h>
int main(void) {
    lidar_scan_t scan;
    unsigned long scan_num, tick;
    float px, py, yaw, tx, ty, installation_yaw;
    tf_init();
    while (scanf("%lu %lu %f %f %f %f %f %f", &scan_num, &tick,
                 &px, &py, &yaw, &tx, &ty, &installation_yaw) == 8) {
        memset(&scan, 0, sizeof(scan));
        scan.scan_num = (uint32_t)scan_num;
        for (unsigned a = 0; a < 360; ++a) {
            unsigned distance, valid;
            if (scanf("%u %u", &distance, &valid) != 2) return 2;
            scan.distance[a] = (uint16_t)distance;
            scan.valid[a] = (uint8_t)valid;
        }
        tf_transform_set(&tf_base, tx, ty, installation_yaw);
        if (!point_cloud_update(&scan, px, py, yaw, (uint32_t)tick)) return 3;
        obstacle_cluster_update(&cloud, (uint32_t)tick);
        printf("{\"points\":[");
        for (unsigned a = 0; a < 360; ++a) {
            lidar_point_t *p = &cloud.points[a];
            printf("%s[%u,%.9g,%.9g]", a ? "," : "", p->valid,
                   (double)p->map_point.x, (double)p->map_point.y);
        }
        printf("],\"clusters\":[");
        for (unsigned c = 0; c < obstacle_result.cluster_count; ++c) {
            lidar_cluster_t *k = &obstacle_result.clusters[c];
            printf("%s{\"start_angle\":%u,\"end_angle\":%u,\"point_count\":%u,"
                   "\"center_point\":{\"x\":%.9g,\"y\":%.9g},"
                   "\"min_x\":%.9g,\"max_x\":%.9g,\"min_y\":%.9g,\"max_y\":%.9g,"
                   "\"width\":%.9g,\"height\":%.9g,\"nearest_distance\":%.9g}",
                   c ? "," : "", k->start_angle, k->end_angle, k->point_count,
                   (double)k->center_point.x, (double)k->center_point.y,
                   (double)k->min_x, (double)k->max_x, (double)k->min_y, (double)k->max_y,
                   (double)k->width, (double)k->height, (double)k->nearest_distance);
        }
        puts("]}");
    }
    return 0;
}
'''


def find_host_cc():
    requested = os.environ.get("LIDAR_REPLAY_CC") or os.environ.get("LIDAR_HOST_CC")
    if requested:
        return requested
    for name in ("gcc", "clang"):
        found = shutil.which(name)
        if found:
            return found
    for path in (r"D:\CLion\CLion 2025.2.2\bin\mingw\bin\gcc.exe", r"D:\MSYS2\ucrt64\bin\gcc.exe", r"D:\MSYS2\mingw64\bin\gcc.exe"):
        if Path(path).is_file():
            return path
    return None


def compile_harness(directory, project, params, compiler):
    header = '''#ifndef REPLAY_HOST_HEADFILE_H
#define REPLAY_HOST_HEADFILE_H
#include <stdint.h>
#include <string.h>
#include <math.h>
typedef void UART_HandleTypeDef;
#include "tf2d.h"
#include "lidar.h"
#include "obstacle_find.h"
'''
    defines = {"LIDAR_MIN_DISTANCE": params["min_distance"], "LIDAR_MAX_DISTANCE": params["max_distance"],
               "CLUSTER_MAX_POINT_GAP": params["gap_mm"], "CLUSTER_MIN_POINT_COUNT": params["min_points"],
               "CLUSTER_MAX_MISS_ANGLE": params["max_miss"], "CLUSTER_MAX_COUNT": params["max_clusters"],
               "LIDAR_WHEEL_MASK_ENABLE": int(params["mask_enabled"])}
    wheel_names = ["LEFT_FORNT_WHEEL", "RIGHT_FORNT_WHEEL", "LEFT_REAR_WHEEL", "RIGHT_REAR_WHEEL"]
    ranges = params["wheel_ranges"] if params["mask_enabled"] else [[360, 360]] * 4
    for name, (start, end) in zip(wheel_names, ranges):
        defines[name + "_START"] = start
        defines[name + "_END"] = end
    for name, value in defines.items():
        token = f"{float(value):.9g}f" if name == "CLUSTER_MAX_POINT_GAP" else str(value)
        if token.endswith("f") and "." not in token and "e" not in token:
            token = token[:-1] + ".0f"
        header += f"#undef {name}\n#define {name} {token}\n"
    header += "#endif\n"
    directory = Path(directory)
    (directory / "headfile.h").write_text(header, encoding="utf-8")
    (directory / "parity.c").write_text(HARNESS, encoding="utf-8")
    output = directory / ("parity.exe" if os.name == "nt" else "parity")
    env = os.environ.copy()
    env["PATH"] = str(Path(compiler).parent) + os.pathsep + env.get("PATH", "")
    command = [compiler, "-std=c99", "-O0", "-ffp-contract=off", "-I", str(directory),
               "-I", str(project / "APP/Inc"), "-I", str(project / "BSP/Inc"),
               str(directory / "parity.c"), str(project / "APP/Src/obstacle_find.c"),
               str(project / "APP/Src/tf2d.c"), "-lm", "-o", str(output)]
    subprocess.run(command, check=True, capture_output=True, text=True, env=env, timeout=30)
    return output, env


def frame_input(frame, params):
    pose, tf = frame["pose"], params["tf"]
    values = [frame["scan_num"], frame["end_time"], pose["x"], pose["y"], pose["yaw_deg"], tf["tx"], tf["ty"], tf["yaw_deg"]]
    for distance, valid in zip(frame["distance"], frame["valid"]):
        values.extend((distance, valid))
    return " ".join(str(v) for v in values) + "\n"


class CParityTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.compiler = find_host_cc()
        if cls.compiler is None:
            raise unittest.SkipTest("No runnable host C compiler found; C differential checks not performed")
        cls.project = firmware_dir(Path(__file__).resolve().parents[2])
        cls.defaults = source_defaults(cls.project)

    def compare(self, params, frames):
        with tempfile.TemporaryDirectory(prefix="lidar_parity_") as directory:
            executable, env = compile_harness(directory, self.project, params, self.compiler)
            for index, frame in enumerate(frames, 1):
                frame["scan_num"] = index
            payload = "".join(frame_input(frame, params) for frame in frames)
            completed = subprocess.run([str(executable)], input=payload, capture_output=True, text=True, check=True, env=env, timeout=20)
        lines = completed.stdout.splitlines()
        self.assertEqual(len(lines), len(frames))
        for index, (frame, line) in enumerate(zip(frames, lines)):
            c_result = json.loads(line)
            py_result = replay_frame(frame, params)
            with self.subTest(frame=index):
                for point, c_point in zip(py_result["points"], c_result["points"]):
                    self.assertEqual(int(point["reason"] == "kept"), c_point[0])
                    if c_point[0]:
                        for axis, value in zip(("x", "y"), c_point[1:]):
                            self.assertAlmostEqual(point["map"][axis], value, delta=0.003)
                self.assertEqual(len(py_result["clusters"]), len(c_result["clusters"]))
                for py_cluster, c_cluster in zip(py_result["clusters"], c_result["clusters"]):
                    for key in ("start_angle", "end_angle", "point_count"):
                        self.assertEqual(py_cluster[key], c_cluster[key])
                    for axis in ("x", "y"):
                        self.assertAlmostEqual(py_cluster["center_point"][axis], c_cluster["center_point"][axis], delta=0.003)
                    for key in ("min_x", "max_x", "min_y", "max_y", "width", "height", "nearest_distance"):
                        self.assertAlmostEqual(py_cluster[key], c_cluster[key], delta=0.003)

    def random_frames(self, count=80):
        rng = random.Random(20261003)
        frames = []
        for index in range(count):
            samples = {}
            for _ in range(rng.randrange(0, 70)):
                start = rng.randrange(360)
                distance = rng.choice([49, 50, 80, 300, 500, 1000, 3999, 4000, 4001])
                for angle in range(start, min(360, start + rng.randrange(1, 15))):
                    if rng.random() < 0.85:
                        samples[angle] = max(0, distance + rng.randrange(-15, 16))
            frames.append(make_frame(samples, {"x": rng.uniform(-500, 2500), "y": rng.uniform(-500, 2500),
                                                "yaw_deg": rng.uniform(-360, 360), "valid": True}))
        return frames

    def test_current_firmware_defaults_deterministic_and_random(self):
        frames = [make_frame(), make_frame({a: 500 for a in range(360)}),
                  make_frame({a: 500 for a in [0, 1, 2, 357, 358, 359]}),
                  make_frame({a: 500 for a in [10, 13, 16, 50, 54, 58]}),
                  make_frame({a: 500 for start in range(5, 350, 10) for a in range(start, start + 3)}),
                  make_frame({1: 49, 2: 50, 3: 4000, 4: 4001})]
        self.compare(copy.deepcopy(self.defaults), frames + self.random_frames())

    def test_parameter_edges_and_cross_zero(self):
        variants = [{"mask_enabled": False}, {"max_miss": 0, "min_points": 1, "max_clusters": 2},
                    {"wheel_ranges": [[350, 8], [90, 110], [170, 200], [270, 280]], "gap_mm": 63.5, "mask_enabled": True},
                    {"gap_mm": 0.0, "max_miss": 10, "min_points": 1},
                    {"max_miss": 254, "min_points": 2, "mask_enabled": False, "gap_mm": 65535.0}]
        for changes in variants:
            params = copy.deepcopy(self.defaults)
            params.update(changes)
            params["tf"] = {"tx": 35.5, "ty": -17.25, "yaw_deg": 13.4, "mirror_x": False}
            frames = [make_frame({a: 500 for a in [0, 1, 2, 357, 358, 359]}),
                      make_frame({a: 500 for a in [0, 1, 358, 359]}),
                      make_frame({a: 500 for start in range(5, 350, 10) for a in range(start, start + 3)})]
            self.compare(params, frames + self.random_frames(12))

    def test_inclusive_gap_threshold_exact_and_just_below(self):
        for gap in (100.0, 99.999):
            params = copy.deepcopy(self.defaults)
            params.update(mask_enabled=False, min_points=2, max_miss=179, gap_mm=gap)
            self.compare(params, [make_frame({0: 50, 180: 50})])


if __name__ == "__main__":
    unittest.main()
