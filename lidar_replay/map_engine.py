"""Replay the current firmware's conservative road-level obstacle map.

Geometry and tuning constants are read from source, never silently guessed.
Input clusters already use field coordinates in millimetres. This module does
not turn unknown roads into free roads: the firmware does not do that either.
"""

from __future__ import annotations

import copy
import hashlib
import math
from pathlib import Path
import re
import struct


class MapSourceError(ValueError):
    """The supplied firmware layout cannot be safely read by this replay."""


def _f32(value):
    """Use the same float precision as the embedded map calculations."""
    try:
        return struct.unpack("<f", struct.pack("<f", float(value)))[0]
    except (TypeError, ValueError, OverflowError):
        return math.nan


def _without_comments(text):
    return re.sub(r"/\*.*?\*/|//[^\n]*", "", text, flags=re.S)


def _number(token):
    token = token.strip()
    if not re.fullmatch(r"(?:0[xX][0-9a-fA-F]+|[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?)[fFuUlL]*", token):
        raise MapSourceError("Unsupported numeric source expression: " + token)
    # Hex digits can themselves end in F; only strip integer U/L suffixes.
    if token.lower().startswith("0x"):
        return int(re.sub(r"[uUlL]+$", "", token), 16)
    return float(re.sub(r"[fFuUlL]+$", "", token))


def _macro(text, name):
    match = re.search(r"^\s*#define\s+" + re.escape(name) + r"\s+(\S+)\s*$", text, re.M)
    if not match:
        raise MapSourceError("Missing or unsupported source constant: " + name)
    return _number(match.group(1))


def _initializer(source, name):
    match = re.search(r"\b" + re.escape(name) + r"\s*(?:\[[^\]]*\])+\s*=\s*\{", source)
    if not match:
        raise MapSourceError("Missing source array: " + name)
    start = match.end() - 1
    depth = 0
    for pos in range(start, len(source)):
        if source[pos] == "{":
            depth += 1
        elif source[pos] == "}":
            depth -= 1
            if depth == 0:
                return source[start + 1:pos]
    raise MapSourceError("Unclosed source array: " + name)


def _rows(source, name, length):
    rows = []
    for row in re.findall(r"\{([^{}]+)\}", _initializer(source, name)):
        values = [_number(token) for token in row.split(",") if token.strip()]
        if len(values) != length:
            raise MapSourceError("Unexpected entry in source array: " + name)
        rows.append(values)
    return rows


def _firmware_root(repo_root):
    root = Path(repo_root).resolve()
    for path in (root, root / "gongxun_1"):
        if (path / "APP" / "Src" / "track_map.c").is_file():
            return path
    raise MapSourceError("Cannot find APP/Src/track_map.c under " + str(root))


def map_layout(repo_root: Path) -> dict:
    """Read the 5x5 field, road graph and map thresholds from current C files.

    Recognises the explicit constant arrays currently used by the project.
    A future incompatible source format raises MapSourceError rather than
    returning a seemingly valid but obsolete field.
    """
    root = _firmware_root(repo_root)
    filenames = (
        "APP/Src/track_map.c", "APP/Inc/track_map.h",
        "APP/Inc/map_data.h", "APP/Inc/map_update.h",
        "APP/Src/map_update.c", "APP/Src/map_data.c", "APP/Src/map_lidar.c",
    )
    try:
        original = {name: (root / name).read_text(encoding="utf-8-sig") for name in filenames}
    except (OSError, UnicodeError) as exc:
        raise MapSourceError("Cannot read current firmware map source: " + str(exc)) from exc
    clean = {name: _without_comments(value) for name, value in original.items()}
    source = clean[filenames[0]]
    header = clean[filenames[1]]
    width = _macro(header, "MAP_WIDTH")
    height = _macro(header, "MAP_HEIGHT")
    cw, ch = int(_macro(header, "MAP_CELL_WIDTH")), int(_macro(header, "MAP_CELL_HEIGHT"))
    nodes = [{"id": i, "x": x, "y": y} for i, (x, y) in enumerate(_rows(source, "map_nodes", 2))]
    roads = [{"id": i, "node_a": int(a), "node_b": int(b)} for i, (a, b) in enumerate(_rows(source, "map_roads_info", 2))]
    forbidden = [dict(zip(("min_x", "max_x", "min_y", "max_y"), row)) for row in _rows(source, "map_notgo_areas", 4)]
    cells_flat = []
    for kind, identity in re.findall(r"\{\s*cell_(node|road|forbidden)\s*,\s*(\w+)\s*\}", _initializer(source, "map_cells")):
        cells_flat.append({"cell_type": kind, "cell_id": None if identity == "MAP_NO_ID" else int(_number(identity))})
    axis = re.search(r"\bmap_axis_to_cell\s*\([^)]*\)\s*\{([^}]*)\}", source, re.S)
    if not axis:
        raise MapSourceError("Cannot read map_axis_to_cell boundaries")
    axis_pairs = re.findall(r"if\s*\(\s*value\s*<\s*([^\s)]+)\s*\)\s*return\s+(\d+)\s*;", axis.group(1))
    interior = [_number(value) for value, _ in axis_pairs]
    boundaries = [0.0] + interior + [width]
    if (len(nodes) != int(_macro(header, "MAP_NODE_COUNT")) or
            len(roads) != int(_macro(clean[filenames[2]], "MAP_ROAD_COUNT")) or
            len(forbidden) != int(_macro(header, "MAP_FORBIDDEN_COUNT")) or
            len(cells_flat) != cw * ch or cw != ch or width != height or
            len(axis_pairs) != cw - 1 or
            [int(index) for _, index in axis_pairs] != list(range(cw - 1)) or
            not re.search(r"return\s+" + str(cw - 1) + r"\s*;\s*$", axis.group(1)) or
            any(a >= b for a, b in zip(boundaries, boundaries[1:]))):
        raise MapSourceError("Unexpected map dimensions or layout; review the replay source parser")
    for road in roads:
        if road["node_a"] not in range(len(nodes)) or road["node_b"] not in range(len(nodes)):
            raise MapSourceError("Road references a nonexistent map node")
        a, b = nodes[road["node_a"]], nodes[road["node_b"]]
        if a["x"] != b["x"] and a["y"] != b["y"]:
            raise MapSourceError("Replay requires the axis-aligned firmware road graph")
    for cell in cells_flat:
        count = len(nodes) if cell["cell_type"] == "node" else len(roads)
        if cell["cell_type"] != "forbidden" and cell["cell_id"] not in range(count):
            raise MapSourceError("Cell references a nonexistent node or road")
    tune = clean[filenames[3]]
    radius_match = re.search(r"map_safe_radius\s*=\s*car_width_mm\s*\*\s*([^\s;]+)\s*\+\s*safety_mm\s*;", clean[filenames[4]])
    if not radius_match:
        raise MapSourceError("Cannot read firmware obstacle inflation formula")
    parameters = {
        "match_distance_mm": _macro(tune, "MAP_MATCH_DISTANCE_MM"),
        "duplicate_distance_mm": _macro(tune, "MAP_DUP_DISTANCE_MM"),
        "confirm_scan_num": int(_macro(tune, "MAP_CONFIRM_SCAN_NUM")),
        "candidate_miss_num": int(_macro(tune, "MAP_CANDIDATE_MISS_NUM")),
        "obstacle_max": int(_macro(clean[filenames[2]], "MAP_OBS_MAX")),
        "car_radius_factor": _number(radius_match.group(1)),
    }
    if not all(math.isfinite(value) and value > 0 for value in parameters.values()):
        raise MapSourceError("Invalid firmware map parameters")
    return {
        "width": width, "height": height, "nodes": nodes, "roads": roads,
        "forbidden": forbidden, "boundaries": boundaries,
        "cells": [cells_flat[y * cw:(y + 1) * cw] for y in range(ch)],
        "parameters": parameters,
        "source_files": [{"path": str(root / name), "sha256": hashlib.sha256(original[name].encode("utf-8")).hexdigest()} for name in filenames],
    }


class MapReplay:
    """Stateful Python equivalent of map_update_scan + map_lidar_handle.

    Backward seeking needs a fresh instance and replay of preceding scans.
    Confirmed road blocks are deliberately persistent. No timeout or automatic
    free-road inference is added by this diagnostic tool.
    """

    def __init__(self, car_width_mm, safety_mm, repo_root=None, layout=None):
        if layout is None:
            layout = map_layout(repo_root or Path(__file__).resolve().parents[1])
        self.layout = copy.deepcopy(layout)
        self.parameters = self.layout["parameters"]
        car, safety = _f32(car_width_mm), _f32(safety_mm)
        self.ready = (math.isfinite(car) and math.isfinite(safety) and
                      0 < car <= self.layout["width"] and 0 <= safety <= self.layout["width"])
        self.safe_radius = _f32(_f32(car * _f32(self.parameters["car_radius_factor"])) + safety) if self.ready else 0.0
        self._roads = {road["id"]: "unknown" for road in self.layout["roads"]}
        self._obstacles = [None] * self.parameters["obstacle_max"]
        self._last_scan = None
        self.change_num = 0

    def cell_at(self, x, y):
        x, y = _f32(x), _f32(y)
        if not (0 <= x < self.layout["width"] and 0 <= y < self.layout["height"]):
            return None
        boundaries = self.layout["boundaries"]
        cx = sum(x >= value for value in boundaries[1:-1])
        cy = sum(y >= value for value in boundaries[1:-1])
        cell = self.layout["cells"][cy][cx]
        return {"x": cx, "y": cy, "type": cell["cell_type"], "id": cell["cell_id"]}

    def _inflated(self, box):
        return {"min_x": _f32(box["min_x"] - self.safe_radius),
                "max_x": _f32(box["max_x"] + self.safe_radius),
                "min_y": _f32(box["min_y"] - self.safe_radius),
                "max_y": _f32(box["max_y"] + self.safe_radius)}

    def _hit_roads(self, box):
        inflated = self._inflated(box)
        result = []
        for road in self.layout["roads"]:
            a = self.layout["nodes"][road["node_a"]]
            b = self.layout["nodes"][road["node_b"]]
            if a["y"] == b["y"]:
                hit = (inflated["max_x"] >= min(a["x"], b["x"]) and
                       inflated["min_x"] <= max(a["x"], b["x"]) and
                       inflated["min_y"] <= a["y"] <= inflated["max_y"])
            else:
                hit = (inflated["max_y"] >= min(a["y"], b["y"]) and
                       inflated["min_y"] <= max(a["y"], b["y"]) and
                       inflated["min_x"] <= a["x"] <= inflated["max_x"])
            if hit:
                result.append(road["id"])
        return result

    @staticmethod
    def _distance_squared(a, b):
        dx, dy = _f32(a["center_x"] - b["center_x"]), _f32(a["center_y"] - b["center_y"])
        return _f32(_f32(dx * dx) + _f32(dy * dy))

    @staticmethod
    def _box(cluster):
        try:
            center = cluster["center_point"]
            return {"center_x": _f32(center["x"]), "center_y": _f32(center["y"]),
                    **{key: _f32(cluster[key]) for key in ("min_x", "max_x", "min_y", "max_y")}}
        except (KeyError, TypeError):
            return None

    def _box_reason(self, box):
        if (box is None or not all(math.isfinite(value) for value in box.values()) or
                box["min_x"] > box["max_x"] or box["min_y"] > box["max_y"] or
                not box["min_x"] <= box["center_x"] <= box["max_x"] or
                not box["min_y"] <= box["center_y"] <= box["max_y"]):
            return "invalid_coordinates"
        if self.cell_at(box["center_x"], box["center_y"]) is None:
            return "outside_field"
        for rect in self.layout["forbidden"]:
            if (rect["min_x"] <= box["center_x"] <= rect["max_x"] and
                    rect["min_y"] <= box["center_y"] <= rect["max_y"]):
                return "static_forbidden"
        return None

    def _result(self, updated, reason, candidates):
        obstacles = []
        for identity, value in enumerate(self._obstacles):
            if value is not None:
                obstacles.append({"id": identity, **copy.deepcopy(value),
                                  "road_ids": self._hit_roads(value), "inflated": self._inflated(value)})
        return {"ready": bool(self.ready), "updated": bool(updated), "reason": reason,
                "roads": [{"id": identity, "state": state} for identity, state in self._roads.items()],
                "obstacles": obstacles, "change_num": self.change_num,
                "safe_radius_mm": self.safe_radius, "candidates": candidates}

    def apply(self, clusters: list, scan_num: int, tick: int, pose_valid: bool = True) -> dict:
        """Apply one complete scan; skipped IDs count as misses for candidates.

        Each candidate includes a rejection reason for visual diagnosis. A
        stale scan or invalid pose leaves all persistent map state untouched.
        tick is metadata only: current firmware has no time-based clearing.
        """
        candidates = []
        for index, cluster in enumerate(clusters):
            box = self._box(cluster)
            valid_box = box is not None and all(math.isfinite(value) for value in box.values())
            candidates.append({"cluster_id": cluster.get("id", index), "accepted": False,
                               "reason": "not_processed", "cell": self.cell_at(box["center_x"], box["center_y"]) if valid_box else None,
                               "road_ids": self._hit_roads(box) if valid_box else []})
        rejection = None
        if not pose_valid:
            rejection = "pose_invalid"
        elif not self.ready:
            rejection = "map_not_ready"
        elif (not isinstance(scan_num, int) or not 0 <= scan_num <= 0xFFFFFFFF or
              not isinstance(tick, int) or not 0 <= tick <= 0xFFFFFFFF):
            rejection = "invalid_scan_metadata"
        elif len(clusters) > self.parameters["obstacle_max"]:
            rejection = "too_many_clusters"
        elif self._last_scan is not None and scan_num <= self._last_scan:
            rejection = "stale_scan"
        if rejection:
            for candidate in candidates:
                candidate["reason"] = rejection
            return self._result(False, rejection, candidates)

        used = set()
        miss_limit = self.parameters["candidate_miss_num"]
        if self._last_scan is not None and scan_num > self._last_scan + 1:
            skipped = scan_num - self._last_scan - 1
            for identity, old in enumerate(self._obstacles):
                if old is not None and old["state"] == "candidate":
                    if old["miss_num"] + skipped >= miss_limit:
                        self._obstacles[identity] = None
                    else:
                        old["miss_num"] += skipped

        match_limit = _f32(self.parameters["match_distance_mm"] ** 2)
        duplicate_limit = _f32(self.parameters["duplicate_distance_mm"] ** 2)
        for index, cluster in enumerate(clusters):
            candidate = candidates[index]
            if not cluster.get("valid", True):
                candidate["reason"] = "cluster_rejected"
                continue
            box = self._box(cluster)
            reason = self._box_reason(box)
            if reason:
                candidate["reason"] = reason
                continue
            roads = self._hit_roads(box)
            if not roads:
                candidate["reason"] = "no_affected_road"
                continue
            if any(self._distance_squared(self._obstacles[identity], box) <= duplicate_limit for identity in used):
                candidate["reason"] = "duplicate_this_scan"
                continue
            identity, best = None, match_limit
            for old_id, old in enumerate(self._obstacles):
                if old is None or old_id in used:
                    continue
                distance = self._distance_squared(old, box)
                # C uses <=: a distance tie intentionally selects the later slot.
                if distance <= best:
                    identity, best = old_id, distance
            if identity is None:
                empty = [i for i, old in enumerate(self._obstacles) if old is None and i not in used]
                if empty:
                    identity = empty[0]
                else:
                    replacement = [(old["last_scan_num"], i) for i, old in enumerate(self._obstacles)
                                   if i not in used and old["state"] == "candidate"]
                    if replacement:
                        identity = min(replacement)[1]
                    else:
                        candidate["reason"] = "obstacle_capacity"
                        continue
                old = None
            else:
                old = self._obstacles[identity]
            seen = min(255, (old["seen_num"] if old else 0) + 1)
            state = "confirmed" if seen >= self.parameters["confirm_scan_num"] else "candidate"
            self._obstacles[identity] = {**box, "state": state, "seen_num": seen, "miss_num": 0,
                                         "last_scan_num": scan_num, "last_tick": tick}
            used.add(identity)
            candidate.update(accepted=True, reason=state, obstacle_id=identity)
            if state == "confirmed":
                for road in roads:
                    if self._roads[road] != "blocked":
                        self._roads[road] = "blocked"
                        self.change_num += 1

        for identity, old in enumerate(self._obstacles):
            if identity not in used and old is not None and old["state"] == "candidate":
                old["miss_num"] = min(255, old["miss_num"] + 1)
                if old["miss_num"] >= miss_limit:
                    self._obstacles[identity] = None
        self._last_scan = scan_num
        return self._result(True, "updated", candidates)
