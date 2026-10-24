"""Replay the current firmware's point cloud and adjacent-angle clustering.

Distances/coordinates are millimetres, angles are degrees.  The C sources remain
authoritative; this module adds rejection diagnostics without changing decisions.
Only the 360-bin published scan is replayed here, not UART packet parsing.
"""
from __future__ import annotations

import math
from pathlib import Path
import re
import struct


def _f(value: float) -> float:
    """Round like a C float after each operation (no fused multiply/add)."""
    return struct.unpack("f", struct.pack("f", value))[0]


_PI = _f(3.14159265358979323846)
_RAD = [_f(_f(angle * _PI) / 180.0) for angle in range(360)]
_SIN = [_f(math.sin(rad)) for rad in _RAD]
_COS = [_f(math.cos(rad)) for rad in _RAD]


def _strip_comments(source: str) -> str:
    return re.sub(r"/\*.*?\*/|//[^\n]*", "", source, flags=re.S)


def firmware_dir(repo_root: Path) -> Path:
    """Accept the project directory or its gongxun parent directory."""
    root = Path(repo_root).resolve()
    if (root / "APP/Inc/obstacle_find.h").is_file():
        return root
    if (root / "gongxun_1/APP/Inc/obstacle_find.h").is_file():
        return root / "gongxun_1"
    raise FileNotFoundError(f"找不到当前工程的 APP/Inc/obstacle_find.h: {root}")


def source_defaults(repo_root: Path) -> dict:
    """Read numeric defaults from live firmware, failing on unsupported syntax."""
    project = firmware_dir(repo_root)
    header = _strip_comments((project / "APP/Inc/obstacle_find.h").read_text(encoding="utf-8-sig"))
    defines = dict(re.findall(r"^\s*#define\s+(\w+)\s+([^\r\n]+)", header, re.M))

    def number(name: str) -> float:
        token = defines[name].strip()
        if not re.fullmatch(r"[-+]?\d+(?:\.\d+)?[fFuUlL]*", token):
            raise ValueError(f"固件常量 {name} 不是可直接解析的数值: {token}")
        return float(token.rstrip("fFuUlL"))

    tf_source = _strip_comments((project / "APP/Src/tf2d.c").read_text(encoding="utf-8-sig"))
    init = re.search(r"void\s+tf_init\s*\(\s*void\s*\)\s*\{(.*?)\}", tf_source, re.S)
    transform = re.search(r"tf_transform_set\s*\(\s*&tf_base\s*,\s*([^,]+),\s*([^,]+),\s*([^\)]+)\)", init.group(1) if init else "")
    if not transform:
        raise ValueError("无法从当前 tf_init 读取 tf_base 初始变换；请显式指定标定参数")
    try:
        tx, ty, yaw = [float(token.strip().rstrip("fF")) for token in transform.groups()]
    except ValueError as error:
        raise ValueError("tf_base 使用了表达式，需要人工核对标定参数") from error
    wheel_names = ["LEFT_FORNT_WHEEL", "RIGHT_FORNT_WHEEL", "LEFT_REAR_WHEEL", "RIGHT_REAR_WHEEL"]
    cluster_source = _strip_comments((project / "APP/Src/obstacle_find.c").read_text(encoding="utf-8-sig"))
    cluster_mode = "circular" if "cluster_start_angle" in cluster_source else "linear_merge"
    return {
        "min_distance": int(number("LIDAR_MIN_DISTANCE")),
        "max_distance": int(number("LIDAR_MAX_DISTANCE")),
        "gap_mm": number("CLUSTER_MAX_POINT_GAP"),
        "min_points": int(number("CLUSTER_MIN_POINT_COUNT")),
        "max_miss": int(number("CLUSTER_MAX_MISS_ANGLE")),
        "max_clusters": int(number("CLUSTER_MAX_COUNT")),
        "wheel_ranges": [[int(number(name + "_START")), int(number(name + "_END"))] for name in wheel_names],
        "mask_enabled": bool(number("LIDAR_WHEEL_MASK_ENABLE")) if "LIDAR_WHEEL_MASK_ENABLE" in defines else True,
        "cluster_mode": cluster_mode,
        "tf": {"tx": tx, "ty": ty, "yaw_deg": yaw, "mirror_x": False},
    }


def validate_params(params: dict) -> None:
    """Reject malformed experiments rather than silently change their meaning."""
    for key in ("min_distance", "max_distance", "gap_mm", "min_points", "max_miss", "max_clusters"):
        value = params.get(key)
        if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value):
            raise ValueError(f"参数 {key} 必须是有限数值")
    for key in ("min_distance", "max_distance", "min_points", "max_miss", "max_clusters"):
        if int(params[key]) != params[key]:
            raise ValueError(f"参数 {key} 必须是整数")
    if not 0 <= params["min_distance"] <= params["max_distance"] <= 65535:
        raise ValueError("距离范围必须满足 0 ≤ 最小距离 ≤ 最大距离 ≤ 65535")
    if not 0 <= params["gap_mm"] <= 65535:
        raise ValueError("点间距必须在 0 到 65535 mm 之间")
    if not 1 <= params["min_points"] <= 360 or not 1 <= params["max_clusters"] <= 360:
        raise ValueError("最小点数和聚类容量必须在 1 到 360 之间")
    # C's missing-angle counter is uint8_t.  Limiting experiments avoids wrap.
    if not 0 <= params["max_miss"] <= 254:
        raise ValueError("允许缺失角度数必须在 0 到 254 之间")
    ranges = params.get("wheel_ranges")
    if not isinstance(ranges, list) or len(ranges) != 4:
        raise ValueError("必须提供四组轮子遮挡角度")
    for pair in ranges:
        if not isinstance(pair, (list, tuple)) or len(pair) != 2 or any(isinstance(v, bool) or not isinstance(v, int) or not 0 <= v <= 359 for v in pair):
            raise ValueError("遮挡角度必须是 0 到 359 的整数起止角")
    if not isinstance(params.get("mask_enabled"), bool):
        raise ValueError("mask_enabled 必须是布尔值")
    if params.get("cluster_mode", "circular") not in ("circular", "linear_merge"):
        raise ValueError("cluster_mode 必须是 circular 或 linear_merge")
    tf = params.get("tf", {})
    for key in ("tx", "ty", "yaw_deg"):
        value = tf.get(key)
        if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value) or abs(value) > 1e7:
            raise ValueError(f"安装变换 {key} 必须是合理的有限数值")
    if not isinstance(tf.get("mirror_x", False), bool):
        raise ValueError("mirror_x 必须是布尔值")


def _transform(point: dict, tx: float, ty: float, yaw: float) -> dict:
    rad = _f(_f(_f(yaw) * _PI) / 180.0)
    sine, cosine = _f(math.sin(rad)), _f(math.cos(rad))
    return {
        "x": _f(_f(_f(cosine * point["x"]) - _f(sine * point["y"])) + _f(tx)),
        "y": _f(_f(_f(sine * point["x"]) + _f(cosine * point["y"])) + _f(ty)),
    }


def _blocked(angle: int, ranges: list) -> bool:
    return any(start <= angle <= end if start <= end else angle >= start or angle <= end for start, end in ranges)


def _connected(previous: dict, current: dict, gap_square: float) -> bool:
    dx = _f(current["map"]["x"] - previous["map"]["x"])
    dy = _f(current["map"]["y"] - previous["map"]["y"])
    return _f(_f(dx * dx) + _f(dy * dy)) <= gap_square


def _start(point: dict) -> dict:
    x, y = point["map"]["x"], point["map"]["y"]
    return {
        "start_angle": point["angle"], "end_angle": point["angle"], "point_count": 1,
        "center_point": {"x": x, "y": y}, "min_x": x, "max_x": x, "min_y": y, "max_y": y,
        "nearest_distance": point["distance"], "angles": [point["angle"]], "valid": False,
    }


def _add(cluster: dict, point: dict) -> None:
    count = cluster["point_count"] + 1
    for axis in ("x", "y"):
        old = cluster["center_point"][axis]
        value = point["map"][axis]
        cluster["center_point"][axis] = _f(old + _f(_f(value - old) / count))
        cluster["min_" + axis] = min(cluster["min_" + axis], value)
        cluster["max_" + axis] = max(cluster["max_" + axis], value)
    cluster["nearest_distance"] = min(cluster["nearest_distance"], point["distance"])
    cluster["point_count"] = count
    cluster["end_angle"] = point["angle"]
    cluster["angles"].append(point["angle"])


def _circular_start(points: list, max_miss: int, gap_square: float) -> int:
    """Match current cluster_start_angle: begin at a real break in the ring."""
    valid_angles = [point["angle"] for point in points if point["reason"] == "kept"]
    if not valid_angles:
        return 0
    first, last = valid_angles[0], valid_angles[-1]
    if first + 359 - last > max_miss or not _connected(points[last], points[first], gap_square):
        return first
    previous = points[first]
    missing = 0
    for angle in range(first + 1, last + 1):
        point = points[angle]
        if point["reason"] != "kept":
            missing += 1
            continue
        if missing > max_miss or not _connected(previous, point, gap_square):
            return angle
        previous, missing = point, 0
    return first


def replay_frame(frame: dict, params: dict) -> dict:
    """Process one normalized frame; recomputation deliberately has no stale state.

    Firmware skips scan_num=0 and repeats through its static scan-number guard.
    Offline replay accepts them so users can revisit/recompute a frame; warnings
    report zero IDs, and ingestion handles repeated IDs separately.
    """
    validate_params(params)
    arrays = [frame.get(name, []) for name in ("distance", "valid", "intensity")]
    if any(not isinstance(values, list) or len(values) != 360 for values in arrays):
        raise ValueError("回放帧必须包含各 360 项的 distance、valid、intensity 数组")
    pose = frame.get("pose") or {"x": 0, "y": 0, "yaw_deg": 0, "valid": False}
    for key in ("x", "y", "yaw_deg"):
        value = pose.get(key, 0)
        if not isinstance(value, (int, float)) or isinstance(value, bool) or not math.isfinite(value) or abs(value) > 1e7:
            raise ValueError(f"位姿 {key} 必须是合理的有限数值")
    warnings = []
    if not pose.get("valid", False):
        warnings.append("本圈没有有效车体位姿；场地坐标仅按提供值示意，地图不得据此更新。")
    if frame.get("scan_num", 0) == 0:
        warnings.append("扫描编号为 0：离线工具允许查看，但固件首次 point_cloud_update 会跳过此编号。")
    if params["tf"].get("mirror_x", False):
        warnings.append("已开启 X 镜像实验；当前固件没有此开关，本结果不代表固件行为。")
    if params.get("cluster_mode", "circular") == "linear_merge":
        warnings.append("使用旧版线性聚类对比模式；当前 circular 固件会先收集跨零点，再判断最小点数。")
    if params["mask_enabled"] and any(pair == [0, 0] for pair in params["wheel_ranges"]):
        warnings.append("已开启轮子遮挡：区间 [0, 0] 会过滤 0°；关闭遮挡开关才会保留该角度。")
    if params["max_clusters"] > 20:
        warnings.append("实验聚类容量超过当前固件固定数组容量 20；需要结合固件结构重新评估。")
    counts = {"raw_valid": 0, "kept": 0, "wheel": 0, "distance": 0, "invalid": 0}
    points = []
    tf = params["tf"]
    for angle, (distance, valid, intensity) in enumerate(zip(*arrays)):
        if isinstance(distance, bool) or not isinstance(distance, int) or not 0 <= distance <= 65535:
            raise ValueError(f"{angle}° 的距离不是 uint16 整数")
        if valid not in (0, 1, False, True):
            raise ValueError(f"{angle}° 的有效标记必须为 0 或 1")
        if isinstance(intensity, bool) or not isinstance(intensity, int) or not 0 <= intensity <= 255:
            raise ValueError(f"{angle}° 的强度不是 uint8 整数")
        counts["raw_valid"] += int(bool(valid))
        lidar = {"x": _f(-distance * _SIN[angle]), "y": _f(distance * _COS[angle])}
        if tf.get("mirror_x", False):
            lidar["x"] = -lidar["x"]
        body = _transform(lidar, tf["tx"], tf["ty"], tf["yaw_deg"])
        mapped = _transform(body, pose.get("x", 0), pose.get("y", 0), pose.get("yaw_deg", 0))
        # Match point_cloud_update ordering, including wheel masks on empty bins.
        if params["mask_enabled"] and _blocked(angle, params["wheel_ranges"]):
            reason = "wheel"
        elif not valid:
            reason = "invalid"
        elif not params["min_distance"] <= distance <= params["max_distance"]:
            reason = "distance"
        else:
            reason = "kept"
        counts[reason] += 1
        points.append({"angle": angle, "distance": distance, "valid": bool(valid), "intensity": intensity,
                       "reason": reason, "lidar": lidar, "body": body, "map": mapped, "cluster_id": None})

    accepted, rejected = [], []
    gap_square = _f(_f(params["gap_mm"]) * _f(params["gap_mm"]))

    def finish(cluster: dict | None) -> None:
        if cluster is None:
            return
        cluster["width"] = _f(cluster["max_x"] - cluster["min_x"])
        cluster["height"] = _f(cluster["max_y"] - cluster["min_y"])
        # Circular mode collects the entire cross-zero run before this gate.
        # Legacy mode keeps the old minimum-before-merge semantics for comparison.
        if cluster["point_count"] < params["min_points"]:
            cluster["reason"] = "too_few_points"
            rejected.append(cluster)
        elif len(accepted) >= params["max_clusters"]:
            cluster["reason"] = "capacity"
            rejected.append(cluster)
        else:
            cluster["valid"] = True
            accepted.append(cluster)

    current, previous, missing = None, None, 0
    circular = params.get("cluster_mode", "circular") == "circular"
    start = _circular_start(points, params["max_miss"], gap_square) if circular else 0
    for step in range(360):
        point = points[(start + step) % 360]
        if point["reason"] != "kept":
            if current is not None:
                missing += 1
                if missing > params["max_miss"]:
                    finish(current)
                    current, previous, missing = None, None, 0
            continue
        if current is None:
            current = _start(point)
        elif _connected(previous, point, gap_square):
            _add(current, point)
        else:
            finish(current)
            current = _start(point)
        previous, missing = point, 0
    finish(current)

    if not circular and len(accepted) >= 2:
        first, last = accepted[0], accepted[-1]
        if first["start_angle"] == 0 and last["end_angle"] == 359 and _connected(points[359], points[0], gap_square):
            total = first["point_count"] + last["point_count"]
            for axis in ("x", "y"):
                weighted = _f(_f(first["center_point"][axis] * first["point_count"]) + _f(last["center_point"][axis] * last["point_count"]))
                first["center_point"][axis] = _f(weighted / total)
                first["min_" + axis] = min(first["min_" + axis], last["min_" + axis])
                first["max_" + axis] = max(first["max_" + axis], last["max_" + axis])
            first["start_angle"] = last["start_angle"]
            first["point_count"] = total
            first["nearest_distance"] = min(first["nearest_distance"], last["nearest_distance"])
            first["width"] = _f(first["max_x"] - first["min_x"])
            first["height"] = _f(first["max_y"] - first["min_y"])
            first["angles"] = last["angles"] + first["angles"]
            accepted.pop()
    for identifier, cluster in enumerate(accepted):
        cluster["id"] = identifier
        for angle in cluster["angles"]:
            points[angle]["cluster_id"] = identifier
    for identifier, cluster in enumerate(rejected):
        cluster["id"] = identifier
    if any(cluster["reason"] == "capacity" for cluster in rejected):
        warnings.append("聚类结果已达容量上限；超出的候选在固件中被丢弃，工具以 capacity 标出。")
    return {"points": points, "clusters": accepted, "rejected": rejected, "warnings": warnings,
            "counts": counts, "scan_num": frame.get("scan_num", 0), "tick": frame.get("end_time", 0)}
