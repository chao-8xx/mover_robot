"""记录文件读写与 D6 字节解码；不操作串口、不改动固件。"""
from __future__ import annotations

import csv
import io
import json
import math
import struct
from collections import OrderedDict

FORMAT = "gongxun-lidar-v1"
MAX_FRAMES = 2000
MAX_BYTES = 16 * 1024 * 1024


class RecordingError(ValueError):
    pass


def number(value, name, low=None, high=None, integer=False):
    if isinstance(value, bool):
        raise RecordingError(f"{name} 需要数值")
    try:
        n = float(value)
    except (ValueError, TypeError):
        raise RecordingError(f"{name} 需要数值") from None
    if not math.isfinite(n) or (low is not None and n < low) or (high is not None and n > high):
        raise RecordingError(f"{name} 超出允许范围")
    if integer and n != int(n):
        raise RecordingError(f"{name} 需要整数")
    return int(n) if integer else n


def flag(value, name="valid"):
    if value in (0, "0", False, "false", "False"):
        return False
    if value in (1, "1", True, "true", "True"):
        return True
    raise RecordingError(f"{name} 只能是 0/1 或 true/false")


def normalize_pose(pose):
    if pose is None:
        return {"x": 0.0, "y": 0.0, "yaw_deg": 0.0, "valid": False}
    if not isinstance(pose, dict):
        raise RecordingError("pose 必须是对象")
    return {"x": number(pose.get("x", 0), "pose.x", -100000, 100000),
            "y": number(pose.get("y", 0), "pose.y", -100000, 100000),
            "yaw_deg": number(pose.get("yaw_deg", 0), "pose.yaw_deg", -360000, 360000),
            "valid": flag(pose.get("valid", False), "pose.valid")}


def normalize_frame(frame, index=0):
    if not isinstance(frame, dict):
        raise RecordingError(f"第 {index + 1} 圈需要对象")
    distances = [0] * 360
    valid = [0] * 360
    intensity = [0] * 360
    if "distance" in frame:
        raw = frame["distance"]
        if not isinstance(raw, list) or len(raw) != 360:
            raise RecordingError("distance 必须恰好包含 360 个整数角度距离")
        for angle, d in enumerate(raw):
            distances[angle] = number(d, "distance", 0, 65535, integer=True)
        v = frame.get("valid", [int(d > 0) for d in distances])
        it = frame.get("intensity", intensity)
        if not isinstance(v, list) or len(v) != 360 or not isinstance(it, list) or len(it) != 360:
            raise RecordingError("valid、intensity 必须各包含 360 项")
        valid = [int(flag(x)) for x in v]
        intensity = [number(x, "intensity", 0, 255, integer=True) for x in it]
    elif "points" in frame:
        if not isinstance(frame["points"], list) or len(frame["points"]) > 10000:
            raise RecordingError("points 需要数组，每圈最多 10000 个样点")
        for p in frame["points"]:
            if not isinstance(p, dict):
                raise RecordingError("points 中每项需要对象")
            a = int(number(p.get("angle"), "angle", 0, 359.999999))
            d = number(p.get("distance"), "distance", 0, 65535, integer=True)
            it = number(p.get("intensity", 0), "intensity", 0, 255, integer=True)
            if flag(p.get("valid", d > 0)) and (not valid[a] or d < distances[a]):
                distances[a], valid[a], intensity[a] = d, 1, it
    else:
        raise RecordingError("缺少 distance[360] 或 points；SCAN / WHEEL / INSTALL 统计日志不能恢复完整点云")
    result = {"distance": distances, "valid": valid, "intensity": intensity,
              "scan_num": number(frame.get("scan_num", index + 1), "scan_num", 1, 4294967295, integer=True),
              "start_time": number(frame.get("start_time", index), "start_time", 0, 4294967295, integer=True),
              "end_time": number(frame.get("end_time", frame.get("tick", index + 1)), "end_time", 0, 4294967295, integer=True),
              "pose": normalize_pose(frame.get("pose"))}
    for name in ("raw_count", "metadata", "label"):
        if name in frame:
            result[name] = frame[name]
    return result


def normalize_recording(data, filename="记录"):
    if isinstance(data, list):
        data = {"frames": data}
    if isinstance(data, dict) and "frames" not in data and ("points" in data or "distance" in data):
        data = {"frames": [data]}
    if not isinstance(data, dict) or not isinstance(data.get("frames"), list):
        raise RecordingError("文件需要 frames 数组，或逐行单圈 JSON")
    if data.get("format", FORMAT) != FORMAT:
        raise RecordingError(f"不支持的格式，请使用 {FORMAT}")
    if not 1 <= len(data["frames"]) <= MAX_FRAMES:
        raise RecordingError(f"记录需要 1～{MAX_FRAMES} 圈")
    frames = [normalize_frame(f, i) for i, f in enumerate(data["frames"])]
    if any(b["scan_num"] <= a["scan_num"] for a, b in zip(frames, frames[1:])):
        raise RecordingError("scan_num 必须严格递增；设备复位后的记录请拆成不同文件")
    for f in frames:
        if any(v and d == 0 for v, d in zip(f["valid"], f["distance"])):
            raise RecordingError("有效点不能是 0 mm；D6 固件将 0 距离视为无效")
    synthetic = flag(data.get("synthetic", False), "synthetic")
    result = {"format": FORMAT, "synthetic": synthetic,
            "data_kind": "synthetic" if synthetic else "imported",
            "label": str(data.get("label", filename))[:200], "frames": frames,
            "warnings": list(data.get("warnings", [])) if isinstance(data.get("warnings", []), list) else [],
            "metadata": data.get("metadata", {})}
    for key in ("params", "car_width_mm", "safety_mm", "source", "compare"):
        if key in data:
            result[key] = data[key]
    if "problem_frame_index" in data:
        result["problem_frame_index"] = number(data["problem_frame_index"], "problem_frame_index", 0, len(frames) - 1, integer=True)
    return result


def parse_csv(text):
    rows = csv.DictReader(io.StringIO(text.lstrip("\ufeff")))
    if not rows.fieldnames or not {"scan_num", "angle", "distance"}.issubset(rows.fieldnames):
        raise RecordingError("CSV 表头至少需要 scan_num,angle,distance")
    grouped = OrderedDict()
    synthetic_values = []
    for row in rows:
        if row.get("synthetic"):
            synthetic_values.append(flag(row["synthetic"], "synthetic"))
        key = number(row["scan_num"], "scan_num", 1, 4294967295, integer=True)
        if key not in grouped:
            grouped[key] = {"scan_num": key, "points": []}
        f = grouped[key]
        f["points"].append({"angle": row["angle"], "distance": row["distance"],
                            "valid": row.get("valid") or int(float(row["distance"]) > 0),
                            "intensity": row.get("intensity") or 0})
        if row.get("timestamp_ms"):
            tick = number(row["timestamp_ms"], "timestamp_ms", 0, 4294967295, integer=True)
            if "end_time" in f and f["end_time"] != tick:
                raise RecordingError("同一圈的 timestamp_ms 必须一致")
            f["end_time"] = tick
        if any(row.get(k) for k in ("pose_x", "pose_y", "pose_yaw", "pose_valid")):
            pose = normalize_pose({"x": row.get("pose_x") or 0, "y": row.get("pose_y") or 0,
                                   "yaw_deg": row.get("pose_yaw") or 0, "valid": row.get("pose_valid") or False})
            if "pose" in f and f["pose"] != pose:
                raise RecordingError("同一圈 pose 必须一致")
            f["pose"] = pose
        if len(grouped) > MAX_FRAMES:
            raise RecordingError(f"文件超过 {MAX_FRAMES} 圈")
    return {"frames": list(grouped.values()), "synthetic": bool(synthetic_values) and all(synthetic_values)}


def read_text_recording(text, filename):
    if not isinstance(text, str):
        raise RecordingError("缺少文件文本")
    if len(text.encode("utf-8")) > MAX_BYTES:
        raise RecordingError("文件超过 16 MB，请分段记录")
    try:
        if filename.lower().endswith(".csv"):
            obj = parse_csv(text)
        elif filename.lower().endswith((".jsonl", ".ndjson")):
            obj = {"frames": [json.loads(line) for line in text.splitlines() if line.strip()]}
            obj["synthetic"] = bool(obj["frames"]) and all(isinstance(f, dict) and isinstance(f.get("metadata"), dict) and f["metadata"].get("synthetic") is True for f in obj["frames"])
        else:
            obj = json.loads(text.lstrip("\ufeff"))
        return normalize_recording(obj, filename)
    except json.JSONDecodeError:
        raise RecordingError("无法读取 JSON。统计日志不能回放；请导入完整逐圈 JSON / JSONL / CSV 或 D6 .bin 字节记录") from None


def _f32(value):
    return struct.unpack("f", struct.pack("f", value))[0]


def decode_d6(data, filename="D6 原始记录"):
    """按当前 lidar.c 的状态机、XOR、向下取整、最近点发布规则解码。

    无时间戳的字节文件采用起始包序号作占位 tick，明确不代表实测时间。
    """
    if not isinstance(data, bytes) or not data or len(data) > MAX_BYTES:
        raise RecordingError("D6 字节文件需为 1 字节～16 MB")
    packet = bytearray()
    target = 0
    work = None
    frames = []
    stats = {"xor_ok": 0, "xor_error": 0, "protocol_rejected": 0,
             "oversized_packets": 0, "start_packets": 0, "prefix_samples": 0}
    for byte in data:
        if not packet:
            if byte == 0xAA:
                packet.append(byte)
            continue
        if len(packet) == 1:
            if byte == 0x55:
                packet.append(byte)
            elif byte != 0xAA:
                packet.clear()
                target = 0
            continue
        packet.append(byte)
        if len(packet) == 4:
            target = 10 + packet[3] * 3
            if target > 85:
                stats["oversized_packets"] += 1
                work = None
                packet.clear()
                target = 0
                continue
        if not target or len(packet) != target:
            continue
        p = bytes(packet)
        packet.clear()
        target = 0
        u16 = lambda offset: p[offset] | (p[offset + 1] << 8)
        checksum = u16(0) ^ u16(2) ^ u16(4) ^ u16(6)
        count = p[3]
        for i in range(count):
            checksum ^= u16(10 + 3 * i) ^ p[12 + 3 * i]
        if checksum != u16(8):
            stats["xor_error"] += 1
            work = None
            continue
        stats["xor_ok"] += 1
        fsa, lsa = u16(4), u16(6)
        start = bool(p[2] & 1)
        if count == 0:
            if start:
                work = None
                stats["protocol_rejected"] += 1
            continue
        if not (fsa & 1 and lsa & 1) or (fsa >> 1) >= 360 * 64 or (lsa >> 1) >= 360 * 64 or (start and count != 1):
            stats["protocol_rejected"] += 1
            work = None
            continue
        if start:
            stats["start_packets"] += 1
            tick = stats["start_packets"] - 1
            if work is not None:
                work["scan_num"] = len(frames) + 1
                work["end_time"] = tick
                frames.append(work)
                if len(frames) > MAX_FRAMES:
                    raise RecordingError(f"原始字节记录超过 {MAX_FRAMES} 圈，请分段")
            work = {"distance": [0] * 360, "valid": [0] * 360, "intensity": [0] * 360,
                    "raw_count": 0, "start_time": tick,
                    "metadata": {"timestamp_known": False, "time_basis": "scan_start_ordinal"}}
        if work is None:
            stats["prefix_samples"] += count
            continue
        a0, a1 = (fsa >> 1) / 64.0, (lsa >> 1) / 64.0
        if a1 < a0:
            a1 += 360
        gap = _f32((a1 - a0) / (count - 1)) if count > 1 else 0
        for i in range(count):
            angle = int(_f32(a0 + _f32(gap * i)) % 360)
            off = 10 + 3 * i
            distance = (p[off + 2] << 6) | (p[off + 1] >> 2)
            strength = ((p[off + 1] & 3) << 6) | (p[off] >> 2)
            work["raw_count"] = (work["raw_count"] + 1) & 0xFFFF
            if distance and (not work["valid"][angle] or distance < work["distance"][angle]):
                work["valid"][angle] = 1
                work["distance"][angle] = distance
                work["intensity"][angle] = strength
    if not frames:
        raise RecordingError("没有找到两个合法起始包，无法发布完整圈。请保留 D6 原始字节，至少记录两次整圈边界")
    warnings = ["原始字节没有时间戳；播放间隔由界面设定，tick 为圈序占位值。",
                "第一个合法起始包之前和最后一个未结束的圈已丢弃；原始记录默认没有有效位姿。"]
    if stats["xor_error"] or stats["protocol_rejected"] or stats["oversized_packets"]:
        warnings.append("检测到坏包或不支持的包；当前固件规则会丢弃正在收集的整圈，详见 metadata.decoder。")
    return normalize_recording({"frames": frames, "label": filename, "warnings": warnings,
                                "metadata": {"decoder": stats, "timestamp_known": False}}, filename)
