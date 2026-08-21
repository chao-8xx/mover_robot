"""ROI 内最大色块检测。该模块不处理 UI、串口和参数文件。"""

import cv2
import numpy as np


# 形态学核固定不变，放在循环外创建，避免每帧重复分配 OpenCV 对象。
MORPH_KERNEL = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))


def _find_contours(mask):
    result = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = result[0] if len(result) == 2 else result[1]
    normalized = []
    for contour in contours:
        try:
            value = contour
            if isinstance(value, (tuple, list)) and len(value) == 1:
                value = value[0]
            points = np.asarray(value, dtype=np.int32)
            if points.size < 6 or points.size % 2:
                continue
            points = np.ascontiguousarray(points.reshape((-1, 1, 2)), dtype=np.int32)
            normalized.append(points)
        except (TypeError, ValueError):
            continue
    return normalized


def _hsv_mask(color_img, threshold):
    lower = threshold["lower"]
    upper = threshold["upper"]
    if lower[0] <= upper[0]:
        return cv2.inRange(
            color_img, np.array(lower, dtype=np.uint8), np.array(upper, dtype=np.uint8)
        )

    # H 起点大于终点表示跨越 179 -> 0，适合红色的两段色相。
    lower_high = np.array([lower[0], lower[1], lower[2]], dtype=np.uint8)
    upper_high = np.array([179, upper[1], upper[2]], dtype=np.uint8)
    lower_low = np.array([0, lower[1], lower[2]], dtype=np.uint8)
    upper_low = np.array([upper[0], upper[1], upper[2]], dtype=np.uint8)
    return cv2.bitwise_or(
        cv2.inRange(color_img, lower_high, upper_high),
        cv2.inRange(color_img, lower_low, upper_low),
    )


def build_mask(roi_bgr, config):
    mode = config["mode"]
    color_key = config["target_color"]
    threshold = config["thresholds"][mode][color_key]
    if mode == "HSV":
        converted = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)
        mask = _hsv_mask(converted, threshold)
    else:
        converted = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2LAB)
        mask = cv2.inRange(
            converted,
            np.array(threshold["lower"], dtype=np.uint8),
            np.array(threshold["upper"], dtype=np.uint8),
        )

    # 先开运算去除孤立噪点，再闭运算连接物料内部的小空洞。
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, MORPH_KERNEL, iterations=1)
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, MORPH_KERNEL, iterations=2)


def empty_result():
    return {
        "found": False,
        "cx": 0,
        "cy": 0,
        "dx": 0,
        "dy": 0,
        "area": 0,
        "rect": (0, 0, 0, 0),
        "contour": None,
    }


def detect_largest_blob(cv_bgr, config):
    """返回整图 mask 和 ROI 内面积最大的合格色块。"""
    roi = config["roi"]
    x, y, w, h = roi["x"], roi["y"], roi["w"], roi["h"]
    roi_bgr = cv_bgr[y:y + h, x:x + w]
    mask_roi = build_mask(roi_bgr, config)

    mask_full = np.zeros(cv_bgr.shape[:2], dtype=np.uint8)
    mask_full[y:y + h, x:x + w] = mask_roi

    min_area = config["blob"]["min_area"]
    max_area = config["blob"]["max_area"]
    best_contour = None
    best_area = 0.0
    for contour in _find_contours(mask_roi):
        area = float(cv2.contourArea(contour))
        if min_area <= area <= max_area and area > best_area:
            best_area = area
            best_contour = contour

    result = empty_result()
    if best_contour is None:
        return mask_full, result

    moments = cv2.moments(best_contour)
    if moments["m00"] == 0:
        return mask_full, result

    contour_full = best_contour.copy()
    contour_full[:, 0, 0] += x
    contour_full[:, 0, 1] += y
    cx = int(moments["m10"] / moments["m00"]) + x
    cy = int(moments["m01"] / moments["m00"]) + y
    center = config["center"]
    grab_x = x + center["x"]
    grab_y = y + center["y"]

    result.update({
        "found": True,
        "cx": cx,
        "cy": cy,
        "dx": cx - grab_x,
        "dy": cy - grab_y,
        "area": int(best_area),
        "rect": cv2.boundingRect(contour_full),
        "contour": contour_full,
    })
    return mask_full, result


def make_preview(cv_bgr, mask_full, result, config, view_mode="OVERLAY"):
    if view_mode == "MASK":
        preview = cv2.cvtColor(mask_full, cv2.COLOR_GRAY2BGR)
    else:
        preview = cv_bgr.copy()
        if view_mode == "OVERLAY":
            tint = preview.copy()
            tint[mask_full > 0] = (0, 220, 255)
            preview = cv2.addWeighted(preview, 0.72, tint, 0.28, 0)

    roi = config["roi"]
    x, y, w, h = roi["x"], roi["y"], roi["w"], roi["h"]
    cv2.rectangle(preview, (x, y), (x + w - 1, y + h - 1), (0, 220, 255), 2)

    center = config["center"]
    grab_x = x + center["x"]
    grab_y = y + center["y"]
    cv2.drawMarker(
        preview, (grab_x, grab_y), (255, 255, 255),
        markerType=cv2.MARKER_CROSS, markerSize=14, thickness=1
    )

    if result["found"]:
        cv2.drawContours(preview, [result["contour"]], -1, (0, 255, 0), 2)
        cv2.circle(preview, (result["cx"], result["cy"]), 4, (0, 0, 255), -1)
        cv2.line(
            preview, (grab_x, grab_y), (result["cx"], result["cy"]),
            (255, 100, 0), 1
        )
    return preview


def threshold_swatch_bgr(config, mode, color_key):
    """根据阈值中心值生成菜单色块，只用于视觉提示。"""
    threshold = config["thresholds"][mode][color_key]
    lower = threshold["lower"]
    upper = threshold["upper"]
    if mode == "HSV":
        if lower[0] <= upper[0]:
            hue = (lower[0] + upper[0]) // 2
        else:
            hue = int((lower[0] + ((upper[0] - lower[0]) % 180) / 2) % 180)
        pixel = np.uint8([[[hue, (lower[1] + upper[1]) // 2,
                            (lower[2] + upper[2]) // 2]]])
        return tuple(int(v) for v in cv2.cvtColor(pixel, cv2.COLOR_HSV2BGR)[0, 0])

    pixel = np.uint8([[[(lower[0] + upper[0]) // 2,
                        (lower[1] + upper[1]) // 2,
                        (lower[2] + upper[2]) // 2]]])
    return tuple(int(v) for v in cv2.cvtColor(pixel, cv2.COLOR_LAB2BGR)[0, 0])
