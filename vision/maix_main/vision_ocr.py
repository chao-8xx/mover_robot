"""任务二的低频二分类确认：当前圆环里的数字是不是2。

只在圆环稳定后调用；确认成功后RingTask进入TRACK，不再执行OCR。
1和3模板仅作为负样本，接口不会返回或跟踪1/3。
"""

import time as pytime

import cv2
import numpy as np


OCR_WIDTH = 32
OCR_HEIGHT = 48
OCR_CROP_RADIUS_RATIO = 0.50
OCR_MIN_SOURCE_HEIGHT = 6
OCR_MIN_CONTOUR_AREA = 8.0
OCR_MIN_ASPECT = 0.10
OCR_MAX_ASPECT = 1.05
OCR_MIN_FILL = 0.12
OCR_MAX_FILL = 0.82

# score是1-Dice，越小越好；margin是最佳负样本减2号分数，越大越可信。
OCR_MAX_TWO_SCORE = 0.42
OCR_MIN_TWO_MARGIN = 0.055

_PERF_COUNTER = getattr(pytime, "perf_counter", pytime.time)
_TEMPLATE_STACK = None
_TEMPLATE_LABELS = None


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


def _find_external_contours(binary):
    found = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = found[0] if len(found) == 2 else found[1]
    result = []
    for contour in contours:
        normalized = _normalize_contour(contour)
        if normalized is not None:
            result.append(normalized)
    return result


def _fit_binary_to_canvas(binary, rect):
    x, y, w, h = rect
    digit = binary[y:y + h, x:x + w]
    canvas = np.zeros((OCR_HEIGHT, OCR_WIDTH), dtype=np.uint8)
    scale = min(
        (OCR_WIDTH - 6) / float(w),
        (OCR_HEIGHT - 6) / float(h),
    )
    new_w = max(1, int(round(w * scale)))
    new_h = max(1, int(round(h * scale)))
    resized = cv2.resize(digit, (new_w, new_h), interpolation=cv2.INTER_NEAREST)
    px = (OCR_WIDTH - new_w) // 2
    py = (OCR_HEIGHT - new_h) // 2
    canvas[py:py + new_h, px:px + new_w] = resized
    return canvas


def normalize_digit_binary(binary, allow_edge=False):
    """提取最大的非边缘字符并归一化；失败时保持未知。"""
    if binary is None or binary.size == 0:
        return None, {"reason": "EMPTY"}

    height, width = binary.shape[:2]
    best = None
    best_area = 0.0
    for contour in _find_external_contours(binary):
        area = float(cv2.contourArea(contour))
        if area < OCR_MIN_CONTOUR_AREA:
            continue
        x, y, w, h = cv2.boundingRect(contour)
        touches_edge = x <= 0 or y <= 0 or x + w >= width or y + h >= height
        if touches_edge and not allow_edge:
            continue
        if area > best_area:
            best = (x, y, w, h)
            best_area = area

    if best is None:
        return None, {"reason": "NO_GLYPH"}

    x, y, w, h = best
    aspect = float(w) / float(max(1, h))
    fill = best_area / float(max(1, w * h))
    if h < OCR_MIN_SOURCE_HEIGHT:
        return None, {"reason": "TOO_SHORT", "height": h}
    if aspect < OCR_MIN_ASPECT or aspect > OCR_MAX_ASPECT:
        return None, {"reason": "ASPECT", "aspect": aspect}
    if fill < OCR_MIN_FILL or fill > OCR_MAX_FILL:
        return None, {"reason": "FILL", "fill": fill}

    normalized = _fit_binary_to_canvas(binary, best)
    return normalized, {
        "reason": "OK",
        "rect": best,
        "area": best_area,
        "aspect": aspect,
        "fill": fill,
    }


def _make_template(digit, font, thickness):
    canvas = np.zeros((96, 64), dtype=np.uint8)
    scale = 2.35
    (text_w, text_h), _ = cv2.getTextSize(str(digit), font, scale, thickness)
    x = (canvas.shape[1] - text_w) // 2
    y = (canvas.shape[0] + text_h) // 2
    cv2.putText(
        canvas, str(digit), (x, y), font, scale, 255, thickness, cv2.LINE_AA
    )
    _, binary = cv2.threshold(canvas, 96, 255, cv2.THRESH_BINARY)
    normalized, _ = normalize_digit_binary(binary, allow_edge=True)
    return normalized


def _ensure_templates():
    global _TEMPLATE_STACK, _TEMPLATE_LABELS
    if _TEMPLATE_STACK is not None:
        return

    templates = []
    labels = []
    variants = (
        (cv2.FONT_HERSHEY_SIMPLEX, 3),
        (cv2.FONT_HERSHEY_SIMPLEX, 5),
        (cv2.FONT_HERSHEY_DUPLEX, 3),
        (cv2.FONT_HERSHEY_DUPLEX, 5),
    )
    # 1/3只作为“不是2”的负样本，不对外输出分类结果。
    for digit in (1, 2, 3):
        for font, thickness in variants:
            template = _make_template(digit, font, thickness)
            if template is not None:
                templates.append(template != 0)
                labels.append(digit)
    _TEMPLATE_STACK = np.asarray(templates, dtype=np.bool_)
    _TEMPLATE_LABELS = np.asarray(labels, dtype=np.int32)


def warmup_two_templates():
    """启动阶段生成并缓存模板，避免第一次B3确认承担初始化耗时。"""
    tick = _PERF_COUNTER()
    _ensure_templates()
    return {
        "template_count": int(len(_TEMPLATE_LABELS)),
        "cost_ms": (_PERF_COUNTER() - tick) * 1000.0,
    }


def crop_digit_roi(cv_bgr, candidate):
    radius = max(1, int(candidate.get("radius", 1)))
    half = max(6, int(round(radius * OCR_CROP_RADIUS_RATIO)))
    cx = int(candidate["cx"])
    cy = int(candidate["cy"])
    height, width = cv_bgr.shape[:2]
    x1 = max(0, cx - half)
    y1 = max(0, cy - half)
    x2 = min(width, cx + half + 1)
    y2 = min(height, cy + half + 1)
    if x2 - x1 < 5 or y2 - y1 < 5:
        return None, (x1, y1, max(0, x2 - x1), max(0, y2 - y1))
    return cv_bgr[y1:y2, x1:x2], (x1, y1, x2 - x1, y2 - y1)


def confirm_digit_two(cv_bgr, candidate):
    """返回is_two；无法确认、空白、1或3一律返回False。"""
    tick = _PERF_COUNTER()
    roi, roi_rect = crop_digit_roi(cv_bgr, candidate)
    result = {
        "is_two": False,
        "digit": 0,
        "accepted": False,
        "score": 1.0,
        "margin": 0.0,
        "roi_rect": roi_rect,
        "normalized": None,
        "reason": "EMPTY",
        "ocr_ms": 0.0,
    }
    if roi is None or roi.size == 0:
        result["ocr_ms"] = (_PERF_COUNTER() - tick) * 1000.0
        return result

    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (3, 3), 0)
    _, binary = cv2.threshold(
        gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU
    )
    normalized, metadata = normalize_digit_binary(binary)
    result["reason"] = metadata.get("reason", "UNKNOWN")
    result["binary"] = binary
    if normalized is None:
        result["ocr_ms"] = (_PERF_COUNTER() - tick) * 1000.0
        return result

    _ensure_templates()
    glyph = normalized != 0
    intersection = np.count_nonzero(_TEMPLATE_STACK & glyph, axis=(1, 2))
    glyph_pixels = float(max(1, np.count_nonzero(glyph)))
    template_pixels = np.count_nonzero(
        _TEMPLATE_STACK, axis=(1, 2)
    ).astype(np.float32)
    dice = (2.0 * intersection.astype(np.float32)) / (
        template_pixels + glyph_pixels
    )
    scores = 1.0 - dice
    two_score = float(np.min(scores[_TEMPLATE_LABELS == 2]))
    other_score = float(np.min(scores[_TEMPLATE_LABELS != 2]))
    margin = other_score - two_score
    accepted = (
        two_score <= OCR_MAX_TWO_SCORE and
        margin >= OCR_MIN_TWO_MARGIN
    )
    if accepted:
        reason = "TWO"
    elif two_score > OCR_MAX_TWO_SCORE and margin < OCR_MIN_TWO_MARGIN:
        reason = "SCORE_MARGIN"
    elif two_score > OCR_MAX_TWO_SCORE:
        reason = "SCORE"
    else:
        reason = "MARGIN"

    result.update({
        "is_two": accepted,
        "digit": 2 if accepted else 0,
        "accepted": accepted,
        "score": two_score,
        "margin": margin,
        "normalized": normalized,
        "reason": reason,
        "metadata": metadata,
        "two_score": two_score,
        "other_score": other_score,
        "ocr_ms": (_PERF_COUNTER() - tick) * 1000.0,
    })
    return result
