"""任务二：完整320x240黑色MASK、严格圆形过滤和同心环带确认。

本模块只生成圆形候选。数字2的低频确认和确认后的圆心跟踪由RingTask负责；
不包含椭圆拟合、1/2/3槽位或常态OCR。
"""

import math
import time as pytime

import cv2
import numpy as np

from vision_blob import empty_result


# 12个方向一次组成矩阵，避免在Python中沿半径逐像素循环。
_RAY_COS = np.asarray(
    [math.cos(index * math.pi / 6.0) for index in range(12)],
    dtype=np.float64,
).reshape((12, 1))
_RAY_SIN = np.asarray(
    [math.sin(index * math.pi / 6.0) for index in range(12)],
    dtype=np.float64,
).reshape((12, 1))

# 分组MASK只负责把断续圆弧和同心带连成一个物理候选；最终层数始终
# 从未粘连的原始MASK读取。
_GROUP_KERNEL_SIZE = 7
_GROUP_EXPAND = _GROUP_KERNEL_SIZE // 2
_GROUP_KERNEL = cv2.getStructuringElement(
    cv2.MORPH_RECT, (_GROUP_KERNEL_SIZE, _GROUP_KERNEL_SIZE)
)
_MIN_LAYER_PASS_DIRECTIONS = 6
_MIN_CONTOUR_AXIS_RATIO = 0.85
_PERF_COUNTER = getattr(pytime, "perf_counter", pytime.time)


def _normalize_contour(contour):
    value = contour
    if isinstance(value, (tuple, list)) and len(value) == 1:
        value = value[0]
    try:
        points = np.asarray(value, dtype=np.int32)
    except (TypeError, ValueError):
        return None
    if points.size < 6 or points.size % 2:
        return None
    return np.ascontiguousarray(points.reshape((-1, 1, 2)), dtype=np.int32)


def _find_external_contours(mask):
    found = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = found[0] if len(found) == 2 else found[1]
    result = []
    for contour in contours:
        normalized = _normalize_contour(contour)
        if normalized is not None:
            result.append(normalized)
    return result


def _median(values):
    if not values:
        return 0.0
    ordered = sorted(values)
    middle = len(ordered) // 2
    if len(ordered) % 2:
        return float(ordered[middle])
    return (float(ordered[middle - 1]) + float(ordered[middle])) * 0.5


def _contour_moment_shape(contour):
    """用二阶中心矩得到圆心和旋转不敏感的短/长轴比。"""
    moments = cv2.moments(contour)
    mass = float(moments.get("m00", 0.0))
    if abs(mass) <= 1.0:
        return None
    cx = float(moments["m10"]) / mass
    cy = float(moments["m01"]) / mass
    cov_xx = float(moments["mu20"]) / mass
    cov_yy = float(moments["mu02"]) / mass
    cov_xy = float(moments["mu11"]) / mass
    trace = cov_xx + cov_yy
    delta = math.sqrt(max(
        0.0,
        (cov_xx - cov_yy) * (cov_xx - cov_yy) +
        4.0 * cov_xy * cov_xy,
    ))
    major_value = (trace + delta) * 0.5
    minor_value = (trace - delta) * 0.5
    if major_value <= 1e-6 or minor_value <= 1e-6:
        return None
    return cx, cy, math.sqrt(minor_value / major_value)


def build_ring_mask(cv_bgr, ring_config):
    """完整320x240画面按灰度上限提取黑色。"""
    height, width = cv_bgr.shape[:2]
    gray = cv2.cvtColor(cv_bgr, cv2.COLOR_BGR2GRAY)
    _, mask = cv2.threshold(
        gray, int(ring_config["black_v_max"]), 255, cv2.THRESH_BINARY_INV
    )
    return mask, (0, 0, width, height)


def _circular_layer_profile(mask, cx, cy, radius):
    """沿正圆12方向批量统计黑带数量。"""
    start = max(1, int(float(radius) * 0.18))
    stop = max(start + 1, int(float(radius) * 1.03))
    distances = np.arange(start, stop, dtype=np.float64).reshape((1, -1))
    sample_x = np.rint(cx + _RAY_COS * distances).astype(np.int32)
    sample_y = np.rint(cy + _RAY_SIN * distances).astype(np.int32)

    height, width = mask.shape[:2]
    inside = (
        (sample_x >= 0) & (sample_x < width) &
        (sample_y >= 0) & (sample_y < height)
    )
    clipped_x = np.clip(sample_x, 0, width - 1)
    clipped_y = np.clip(sample_y, 0, height - 1)
    black = (mask[clipped_y, clipped_x] != 0) & inside

    counts = black[:, 0].astype(np.int32)
    if black.shape[1] > 1:
        run_starts = black[:, 1:] & (~black[:, :-1])
        counts = counts + np.count_nonzero(run_starts, axis=1)
    direction_counts = counts.tolist()
    return int(round(_median(direction_counts))), direction_counts


def _radial_layer_count(mask, cx, cy, radius):
    """兼容旧的圆形层数回归入口。"""
    layer_count, _ = _circular_layer_profile(mask, cx, cy, radius)
    return layer_count


def find_ring_candidates(mask, ring_config, roi_rect):
    """连接断续环得到外边界，再以正圆多方向黑带数确认同心结构。"""
    group_tick = _PERF_COUNTER()
    group_mask = cv2.dilate(mask, _GROUP_KERNEL, iterations=1)
    group_ms = (_PERF_COUNTER() - group_tick) * 1000.0

    contour_tick = _PERF_COUNTER()
    group_contours = _find_external_contours(group_mask)
    contour_ms = (_PERF_COUNTER() - contour_tick) * 1000.0

    offset_x, offset_y, _, _ = roi_rect
    min_layers = int(ring_config["min_layers"])
    min_diameter = float(ring_config["min_diameter"])
    max_diameter = float(ring_config["max_diameter"])
    min_area = float(ring_config["min_contour_area"])
    max_area = float(ring_config["max_contour_area"])
    min_circularity = float(ring_config["min_circularity"])
    center = ring_config["center"]
    grab_x = roi_rect[0] + int(center["x"])
    grab_y = roi_rect[1] + int(center["y"])
    candidates = []
    rejected_candidates = []
    reject_counts = {
        "AREA": 0,
        "SIZE": 0,
        "RATIO": 0,
        "CIRC": 0,
        "LAYER": 0,
    }
    radial_check_count = 0
    filter_tick = _PERF_COUNTER()

    for contour in group_contours:
        area = float(cv2.contourArea(contour))
        if area < min_area or area > max_area:
            reject_counts["AREA"] += 1
            continue

        x, y, w, h = cv2.boundingRect(contour)
        if w < 8 or h < 8:
            reject_counts["SIZE"] += 1
            continue
        diameter = max(1.0, float(max(w, h) - _GROUP_EXPAND * 2))
        if diameter < min_diameter or diameter > max_diameter:
            reject_counts["SIZE"] += 1
            continue

        moment_shape = _contour_moment_shape(contour)
        if moment_shape is None:
            reject_counts["RATIO"] += 1
            continue
        local_cx, local_cy, shape_roundness = moment_shape
        if shape_roundness < _MIN_CONTOUR_AXIS_RATIO:
            reject_counts["RATIO"] += 1
            continue

        perimeter = float(cv2.arcLength(contour, True))
        if perimeter <= 1.0:
            reject_counts["CIRC"] += 1
            continue
        circularity = 4.0 * math.pi * area / (perimeter * perimeter)
        if circularity < min_circularity:
            reject_counts["CIRC"] += 1
            continue

        radius = max(
            1.0,
            (float(w) + float(h)) * 0.25 - float(_GROUP_EXPAND),
        )
        radial_check_count += 1
        layer_count, direction_counts = _circular_layer_profile(
            mask, local_cx, local_cy, radius
        )
        layer_pass_count = int(np.count_nonzero(
            np.asarray(direction_counts, dtype=np.int32) >= min_layers
        ))

        cx = int(round(local_cx + offset_x))
        cy = int(round(local_cy + offset_y))
        draw_radius = max(1, int(round(radius)))
        common = {
            "cx": cx,
            "cy": cy,
            "radius": draw_radius,
            "diameter": int(round(diameter)),
            "layer_count": layer_count,
            "layer_pass_count": layer_pass_count,
            "direction_counts": direction_counts,
            "circularity": circularity,
            "shape_roundness": shape_roundness,
            "rect": (
                cx - draw_radius, cy - draw_radius,
                draw_radius * 2, draw_radius * 2,
            ),
        }
        if layer_pass_count < _MIN_LAYER_PASS_DIRECTIONS:
            reject_counts["LAYER"] += 1
            common["reason"] = "LAYER"
            rejected_candidates.append(common)
            continue

        common.update({
            "dx": cx - grab_x,
            "dy": cy - grab_y,
            "feature_count": layer_count,
        })
        candidates.append(common)

    candidates.sort(key=lambda item: item["cx"])
    rejected_candidates.sort(key=lambda item: item["cx"])
    for index, candidate in enumerate(candidates):
        candidate["candidate_index"] = index + 1
    filter_ms = (_PERF_COUNTER() - filter_tick) * 1000.0
    return (
        candidates,
        len(group_contours),
        radial_check_count,
        group_ms,
        contour_ms,
        filter_ms,
        rejected_candidates,
        reject_counts,
    )


def detect_ring_candidates(cv_bgr, ring_config):
    mask_tick = _PERF_COUNTER()
    mask, roi_rect = build_ring_mask(cv_bgr, ring_config)
    mask_ms = (_PERF_COUNTER() - mask_tick) * 1000.0
    (
        candidates,
        contour_count,
        feature_count,
        group_ms,
        contour_ms,
        filter_ms,
        rejected_candidates,
        reject_counts,
    ) = find_ring_candidates(mask, ring_config, roi_rect)
    checked_candidates = sorted(
        list(candidates) + list(rejected_candidates),
        key=lambda item: item["cx"],
    )
    result = empty_result()
    result.update({
        "candidate_count": len(candidates),
        "candidates": candidates,
        "contour_count": contour_count,
        "feature_count": feature_count,
        "geometry_count": feature_count,
        "layer_reject_count": int(reject_counts["LAYER"]),
        "layer_values": [
            int(item["layer_count"]) for item in checked_candidates
        ],
        "direction_pass_values": [
            int(item.get("layer_pass_count", 0))
            for item in checked_candidates
        ],
        "axis_ratio_values": [
            float(item.get("shape_roundness", 0.0))
            for item in checked_candidates
        ],
        "rejected_candidates": rejected_candidates,
        "reject_counts": reject_counts,
        "target_number": 2,
        "stage": "CANDIDATES_ONLY",
        "mask_ms": mask_ms,
        "group_ms": group_ms,
        "contour_ms": contour_ms,
        "filter_ms": filter_ms,
    })
    return result, mask, roi_rect


def make_ring_preview(cv_bgr, mask_roi, result, ring_config, view_mode="OVERLAY"):
    """生成任务二调试图；只在屏幕刷新时调用。"""
    roi = ring_config["roi"]
    x, y, w, h = roi["x"], roi["y"], roi["w"], roi["h"]
    if view_mode == "MASK" and mask_roi is not None:
        preview = np.zeros_like(cv_bgr)
        preview[y:y + h, x:x + w] = cv2.cvtColor(mask_roi, cv2.COLOR_GRAY2BGR)
    else:
        preview = cv_bgr.copy()

    cv2.rectangle(preview, (x, y), (x + w - 1, y + h - 1), (0, 220, 255), 1)
    center = ring_config["center"]
    center_x = x + int(center["x"])
    center_y = y + int(center["y"])
    cv2.drawMarker(
        preview, (center_x, center_y), (255, 255, 255),
        cv2.MARKER_CROSS, 18, 2,
    )

    if view_mode == "OVERLAY":
        for rejected in result.get("rejected_candidates", ()):
            cx, cy = rejected["cx"], rejected["cy"]
            radius = rejected["radius"]
            cv2.circle(preview, (cx, cy), radius, (0, 210, 255), 1)
            cv2.putText(
                preview,
                "X L{} P{}/12".format(
                    rejected.get("layer_count", 0),
                    rejected.get("layer_pass_count", 0),
                ),
                (max(0, cx - radius), max(14, cy - radius - 4)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.34,
                (0, 210, 255),
                1,
            )

        for candidate in result.get("candidates", ()):
            cx, cy = candidate["cx"], candidate["cy"]
            radius = candidate["radius"]
            selected = bool(candidate.get("selected", False))
            color = (0, 255, 80) if selected else (255, 190, 40)
            cv2.circle(preview, (cx, cy), radius, color, 2)
            cv2.circle(preview, (cx, cy), 3, (0, 0, 255), -1)
            cv2.putText(
                preview,
                "{} N{} L{} P{}".format(
                    "T" if selected else "C" + str(candidate["candidate_index"]),
                    2 if candidate.get("digit", 0) == 2 else "?",
                    candidate["layer_count"],
                    candidate.get("layer_pass_count", 0),
                ),
                (max(0, cx - radius), max(14, cy - radius - 4)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.38,
                color,
                1,
            )

        cv2.putText(
            preview,
            "{} TWO:{} F:{} C/G/L:{}/{}/{}".format(
                result.get("state", "SEARCH"),
                1 if result.get("is_two_confirmed", False) else 0,
                1 if result.get("found", False) else 0,
                result.get("candidate_count", 0),
                result.get("geometry_count", 0),
                result.get("layer_reject_count", 0),
            ),
            (x + 5, max(y + 18, y + h - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.40,
            (0, 220, 255),
            1,
        )
    return preview
