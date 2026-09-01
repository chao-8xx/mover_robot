"""任务二第一阶段：黑色MASK、圆形轮廓过滤和同心候选聚类。

本模块暂时不识别数字，也不把候选冒充为已经确认的目标圆心。
所有输入、输出坐标保持320x240原图契约。
"""

import math
import time as pytime

import cv2
import numpy as np

from vision_blob import empty_result


# 12个采样方向一次组成矩阵；每个候选只调用一次NumPy索引和转折统计，
# 避免在Python里沿半径逐像素循环。
_RAY_COS = np.asarray(
    [math.cos(index * math.pi / 6.0) for index in range(12)],
    dtype=np.float64,
).reshape((12, 1))
_RAY_SIN = np.asarray(
    [math.sin(index * math.pi / 6.0) for index in range(12)],
    dtype=np.float64,
).reshape((12, 1))

# 分组MASK可以比原始识别MASK更强：它只负责把断续圆弧和相邻同心带
# 连接成一个物理候选，最终黑带层数仍从未粘连的原始MASK读取。矩形核
# 通常走OpenCV的快速路径；单次膨胀也比闭运算的“膨胀+腐蚀”少一遍全图。
_GROUP_KERNEL_SIZE = 7
_GROUP_EXPAND = _GROUP_KERNEL_SIZE // 2
_GROUP_KERNEL = cv2.getStructuringElement(
    cv2.MORPH_RECT, (_GROUP_KERNEL_SIZE, _GROUP_KERNEL_SIZE)
)
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


def _normalized_contours(contours):
    result = []
    for contour in contours:
        normalized = _normalize_contour(contour)
        if normalized is not None:
            result.append(normalized)
    return result


def _find_external_contours(mask):
    result = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = result[0] if len(result) == 2 else result[1]
    return _normalized_contours(contours)


def _median(values):
    if not values:
        return 0.0
    ordered = sorted(values)
    middle = len(ordered) // 2
    if len(ordered) % 2:
        return float(ordered[middle])
    return (float(ordered[middle - 1]) + float(ordered[middle])) * 0.5


def build_ring_mask(cv_bgr, ring_config):
    """任务二固定在完整320x240画面内按灰度阈值提取黑色。"""
    height, width = cv_bgr.shape[:2]
    gray = cv2.cvtColor(cv_bgr, cv2.COLOR_BGR2GRAY)
    _, mask = cv2.threshold(
        gray, int(ring_config["black_v_max"]), 255, cv2.THRESH_BINARY_INV
    )
    # 六级同心圈在320x240画面中的间距可能只有数像素；闭运算容易把相邻圈
    # 粘成一片。第一阶段先保留原始MASK，依靠几何过滤排除小噪点。
    return mask, (0, 0, width, height)


def _radial_layer_count(mask, cx, cy, radius):
    """批量统计12个方向的黑带数量，跳过容易受数字污染的中心区。"""
    start = max(1, int(radius * 0.18))
    stop = max(start + 1, int(radius * 1.03))
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
    return int(round(_median(counts.tolist())))


def find_ring_candidates(mask, ring_config, roi_rect):
    """先连接断续环得到外边界，再以多方向黑带数确认同心结构。"""
    # 原始RETR_LIST轮廓和逐轮廓中心统计不参与候选是否通过的判断，且在
    # 电脑屏幕摩尔纹场景会制造大量Python循环，因此从正式热路径中移除。
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
    radial_check_count = 0
    filter_tick = _PERF_COUNTER()

    for contour in group_contours:
        area = float(cv2.contourArea(contour))
        if area < min_area or area > max_area:
            continue

        x, y, w, h = cv2.boundingRect(contour)
        if w < 8 or h < 8:
            continue
        # 单次膨胀会在两侧各扩张_GROUP_EXPAND像素；过滤、黑带采样和
        # 对外显示继续使用补偿后的原始圆环近似尺寸。
        diameter = max(1.0, float(max(w, h) - _GROUP_EXPAND * 2))
        if diameter < min_diameter or diameter > max_diameter:
            continue
        ratio = float(w) / float(h)
        if ratio < 0.55 or ratio > 1.80:
            continue

        perimeter = float(cv2.arcLength(contour, True))
        if perimeter <= 1.0:
            continue
        circularity = 4.0 * math.pi * area / (perimeter * perimeter)
        if circularity < min_circularity:
            continue

        local_cx = x + w * 0.5
        local_cy = y + h * 0.5
        radius = max(
            1.0,
            (float(w) + float(h)) * 0.25 - float(_GROUP_EXPAND),
        )
        radial_check_count += 1
        layer_count = _radial_layer_count(
            mask, local_cx, local_cy, radius
        )
        if layer_count < min_layers:
            continue

        cx = int(round(local_cx + offset_x))
        cy = int(round(local_cy + offset_y))
        draw_radius = max(1, int(round(radius)))
        candidates.append({
            "cx": cx,
            "cy": cy,
            "dx": cx - grab_x,
            "dy": cy - grab_y,
            "radius": draw_radius,
            "diameter": int(round(diameter)),
            "layer_count": layer_count,
            "feature_count": layer_count,
            "circularity": circularity,
            "rect": (cx - draw_radius, cy - draw_radius,
                     draw_radius * 2, draw_radius * 2),
        })

    candidates.sort(key=lambda item: item["cx"])
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
    ) = find_ring_candidates(mask, ring_config, roi_rect)
    result = empty_result()
    result.update({
        "candidate_count": len(candidates),
        "candidates": candidates,
        "contour_count": contour_count,
        "feature_count": feature_count,
        "target_number": int(ring_config["target_number"]),
        "stage": "CANDIDATES_ONLY",
        "mask_ms": mask_ms,
        "group_ms": group_ms,
        "contour_ms": contour_ms,
        "filter_ms": filter_ms,
    })
    return result, mask, roi_rect


def make_ring_preview(cv_bgr, mask_roi, result, ring_config, view_mode="OVERLAY"):
    """生成任务二调试图；只在屏幕刷新时调用，不进入高速检测热路径。"""
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
    cv2.drawMarker(preview, (center_x, center_y), (255, 255, 255),
                   cv2.MARKER_CROSS, 18, 2)

    if view_mode == "OVERLAY":
        for candidate in result.get("candidates", ()):
            cx, cy = candidate["cx"], candidate["cy"]
            radius = candidate["radius"]
            cv2.circle(preview, (cx, cy), radius, (0, 255, 80), 2)
            cv2.circle(preview, (cx, cy), 3, (0, 0, 255), -1)
            label = "C{} L{} D{}".format(
                candidate["candidate_index"],
                candidate["layer_count"],
                candidate["diameter"],
            )
            cv2.putText(preview, label, (max(0, cx - radius), max(14, cy - radius - 4)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 255, 80), 1)
    return preview
