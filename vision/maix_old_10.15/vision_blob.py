"""ROI 内最大色块检测。

职责边界：只处理图像、坐标和结果映射，不处理 UI、串口或参数文件。
对外结果统一使用 320x240 坐标，方便未来任务二、任务三共用主循环接口。
"""

import cv2
import numpy as np
from time import perf_counter

from vision_config import (
    MORPH_CLOSE_ONLY,
    MORPH_OPEN_CLOSE,
    MORPH_OPEN_ONLY,
    VALID_MORPH_MODES,
)


# 物料是大色块，3x3 开/闭各一次足够做第一轮去噪，比原来的 5x5 + 闭运算两次轻。
MORPH_KERNEL = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))

# 运行时只使用当前颜色的一组LAB阈值。缓存对应的 uint8 数组，
# 避免每帧重复把 Python list 转换成多个 NumPy 小数组。
_THRESHOLD_CACHE = {}
B2_PIPELINE_BUILD = "B2_BUFFERS_V1"


class ColorBuffers:
    """仅B2复用处理输出内存；不改变采样尺寸、阈值或形态学运算。"""

    def __init__(self, width, height):
        self.bgr = np.empty((height, width, 3), dtype=np.uint8)
        self.lab = np.empty_like(self.bgr)
        self.mask = np.empty((height, width), dtype=np.uint8)
        self.cleaned = np.empty_like(self.mask)


def _find_contours(mask):
    result = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return result[0] if len(result) == 2 else result[1]


def _normalize_contour(contour):
    """仅在旧版 OpenCV 绑定不能直接使用轮廓时执行兼容转换。"""
    value = contour
    if isinstance(value, (tuple, list)) and len(value) == 1:
        value = value[0]
    points = np.asarray(value, dtype=np.int32)
    if points.size < 6 or points.size % 2:
        return None
    return np.ascontiguousarray(points.reshape((-1, 1, 2)), dtype=np.int32)


def _threshold_arrays(threshold):
    """缓存阈值数组；触屏修改数值后自动使用新缓存项。"""
    lower = tuple(int(v) for v in threshold["lower"])
    upper = tuple(int(v) for v in threshold["upper"])
    cache_key = (lower, upper)
    if cache_key in _THRESHOLD_CACHE:
        return _THRESHOLD_CACHE[cache_key]

    arrays = (np.array(lower, dtype=np.uint8), np.array(upper, dtype=np.uint8))
    if len(_THRESHOLD_CACHE) >= 24:
        _THRESHOLD_CACHE.clear()
    _THRESHOLD_CACHE[cache_key] = arrays
    return arrays


def extract_roi(cv_bgr, config):
    """取得 ROI 图像视图和全图坐标，不在这里复制像素。"""
    roi = config["roi"]
    x, y, w, h = roi["x"], roi["y"], roi["w"], roi["h"]
    return cv_bgr[y:y + h, x:x + w], (x, y, w, h)


def make_run_geometry(config, source_width, source_height,
                      process_width, process_height):
    """预计算 RUN 缩放参数。

    config 中的 ROI、center、blob 面积均保持原图单位；返回值中的 config
    仅供小图识别使用，生命周期只覆盖一次 RUN，不会保存到 JSON。
    """
    roi = config["roi"]
    source_x = int(roi["x"])
    source_y = int(roi["y"])
    source_w = int(roi["w"])
    source_h = int(roi["h"])

    frame_scale_x = float(process_width) / float(source_width)
    frame_scale_y = float(process_height) / float(source_height)
    scaled_w = max(1, int(round(source_w * frame_scale_x)))
    scaled_h = max(1, int(round(source_h * frame_scale_y)))
    scale_x = float(scaled_w) / float(source_w)
    scale_y = float(scaled_h) / float(source_h)
    area_scale = scale_x * scale_y

    center = config["center"]
    scaled_center_x = min(scaled_w - 1, max(0, int(round(center["x"] * scale_x))))
    scaled_center_y = min(scaled_h - 1, max(0, int(round(center["y"] * scale_y))))

    blob = config["blob"]
    scaled_min_area = max(1, int(round(blob["min_area"] * area_scale)))
    scaled_max_area = max(
        scaled_min_area, int(round(blob["max_area"] * area_scale))
    )

    # 阈值表只读共享；ROI、中心点和面积阈值使用 RUN 专用的小图副本。
    process_config = {
        "target_color": config["target_color"],
        "morph_mode": config["morph_mode"],
        "thresholds": config["thresholds"],
        "roi": {"x": 0, "y": 0, "w": scaled_w, "h": scaled_h},
        "center": {
            "x": scaled_center_x,
            "y": scaled_center_y,
            "custom": bool(center["custom"]),
        },
        "blob": {"min_area": scaled_min_area, "max_area": scaled_max_area},
    }
    return {
        "source_roi": (source_x, source_y, source_w, source_h),
        "process_roi": (0, 0, scaled_w, scaled_h),
        "scale_x": scale_x,
        "scale_y": scale_y,
        "area_scale": area_scale,
        "config": process_config,
    }


def resize_run_roi(cv_bgr, geometry, dst=None):
    """裁出 320x240 基准 ROI，再按 RUN 比例缩小；小 ROI 不必缩放整幅图。"""
    x, y, w, h = geometry["source_roi"]
    scaled_w = geometry["process_roi"][2]
    scaled_h = geometry["process_roi"][3]
    source_roi = cv_bgr[y:y + h, x:x + w]
    if scaled_w == w and scaled_h == h:
        return source_roi
    # 大色块使用最近邻可避免边缘混色，通常也比高质量插值更省时。
    if dst is not None:
        return cv2.resize(source_roi, (scaled_w, scaled_h), dst=dst,
                          interpolation=cv2.INTER_NEAREST)
    return cv2.resize(
        source_roi, (scaled_w, scaled_h), interpolation=cv2.INTER_NEAREST
    )


def map_run_result_to_source(result, source_config, geometry):
    """把 RUN 小图结果恢复为 320x240 坐标、偏差和等效面积。

    返回字段契约：
    found 表示当前帧是否找到目标；cx/cy 是原图绝对坐标；
    dx/dy 是目标相对抓取中心的偏差；area 是 320x240 等效轮廓面积。
    """
    if not result["found"]:
        return empty_result()

    source_x, source_y, _, _ = geometry["source_roi"]
    scale_x = geometry["scale_x"]
    scale_y = geometry["scale_y"]
    area_scale = geometry["area_scale"]
    center = source_config["center"]
    grab_x = source_x + center["x"]
    grab_y = source_y + center["y"]

    # 先映射偏差再重建中心，确保小图中心对齐时回到 320x240 后仍为 dx=dy=0。
    dx = int(round(result["dx"] / scale_x))
    dy = int(round(result["dy"] / scale_y))
    cx = grab_x + dx
    cy = grab_y + dy

    rect_x, rect_y, rect_w, rect_h = result["rect"]
    mapped_rect = (
        source_x + int(round(rect_x / scale_x)),
        source_y + int(round(rect_y / scale_y)),
        max(1, int(round(rect_w / scale_x))),
        max(1, int(round(rect_h / scale_y))),
    )
    return {
        "found": True,
        "cx": cx,
        "cy": cy,
        "dx": dx,
        "dy": dy,
        "area": int(round(result["area"] / area_scale)),
        "rect": mapped_rect,
        "contour": None,
    }


def convert_color_space(roi_bgr, dst=None):
    """B2/B4固定使用LAB，每个处理帧只转换一次。"""
    if dst is not None:
        return cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2LAB, dst=dst)
    return cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2LAB)


def apply_color_threshold(converted, config, dst=None):
    """在已经转换的 ROI 上执行当前颜色阈值。"""
    color_key = config["target_color"]
    threshold = config["thresholds"]["LAB"][color_key]
    lower_array, upper_array = _threshold_arrays(threshold)
    if dst is not None:
        return cv2.inRange(converted, lower_array, upper_array, dst=dst)
    return cv2.inRange(converted, lower_array, upper_array)


def open_mask(mask, dst=None):
    """去除孤立噪点；独立函数便于评估这一遍处理是否值得保留。"""
    if dst is not None:
        return cv2.morphologyEx(mask, cv2.MORPH_OPEN, MORPH_KERNEL, dst=dst, iterations=1)
    return cv2.morphologyEx(mask, cv2.MORPH_OPEN, MORPH_KERNEL, iterations=1)


def close_mask(mask, dst=None):
    """连接物料内部的小空洞；独立函数便于在真机上单独计时。"""
    if dst is not None:
        return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, MORPH_KERNEL, dst=dst, iterations=1)
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, MORPH_KERNEL, iterations=1)


def clean_mask(mask, morph_mode=MORPH_OPEN_CLOSE, dst=None):
    """按照配置档位处理 mask；默认保持原有开运算加闭运算。"""
    if morph_mode not in VALID_MORPH_MODES:
        raise ValueError("unknown morphology mode: " + str(morph_mode))
    if morph_mode in (MORPH_OPEN_ONLY, MORPH_OPEN_CLOSE):
        mask = open_mask(mask) if dst is None else open_mask(mask, dst=dst)
    if morph_mode in (MORPH_CLOSE_ONLY, MORPH_OPEN_CLOSE):
        mask = close_mask(mask) if dst is None else close_mask(mask, dst=dst)
    return mask


def build_mask(roi_bgr, config):
    """兼容入口：颜色转换、阈值和配置中选定的形态学档位。"""
    converted = convert_color_space(roi_bgr)
    mask = apply_color_threshold(converted, config)
    return clean_mask(mask, config.get("morph_mode", MORPH_OPEN_CLOSE))


def detect_color(roi_bgr, config, roi_rect, need_mask=True, buffers=None, fast_filter=False):
    """只检测指定颜色；RUN不保留预览MASK，不创建三色结果集合。"""
    tick = perf_counter()
    converted = (convert_color_space(roi_bgr) if buffers is None else
                 convert_color_space(roi_bgr, dst=buffers.lab))
    conversion_ms = (perf_counter() - tick) * 1000.0
    tick = perf_counter()
    mask = (apply_color_threshold(converted, config) if buffers is None else
            apply_color_threshold(converted, config, dst=buffers.mask))
    threshold_ms = (perf_counter() - tick) * 1000.0
    tick = perf_counter()
    mode = config.get("morph_mode", MORPH_OPEN_CLOSE)
    mask = (clean_mask(mask, mode) if buffers is None else
            clean_mask(mask, mode, dst=buffers.cleaned))
    morph_ms = (perf_counter() - tick) * 1000.0
    tick = perf_counter()
    result = (find_largest_blob(mask, config, roi_rect, fast_filter=True) if fast_filter else
              find_largest_blob(mask, config, roi_rect))
    blob_ms = (perf_counter() - tick) * 1000.0
    return result, mask if need_mask else None, {
        "conversion_ms": conversion_ms,
        "threshold_ms": threshold_ms,
        "morph_ms": morph_ms,
        "blob_ms": blob_ms,
    }


def empty_result():
    """统一的未识别结果；未来其他任务也可以沿用相同基础字段。"""
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


def build_roi_mask(cv_bgr, config):
    """只生成 ROI 大小的掩膜；核心识别不再为调试界面创建整图 mask。"""
    roi_bgr, roi_rect = extract_roi(cv_bgr, config)
    return build_mask(roi_bgr, config), roi_rect


def find_largest_blob(mask_roi, config, roi_rect, fast_filter=False):
    """在已有 ROI 掩膜中只选择面积最大的合格色块并计算中心偏差。"""
    x, y, w, h = roi_rect
    min_area = config["blob"]["min_area"]
    max_area = config["blob"]["max_area"]
    best_contour = None
    best_area = 0.0
    for contour in _find_contours(mask_roi):
        if fast_filter:
            # 某些设备绑定返回tuple/list；直接规范化，避免每个轮廓先抛一次异常。
            if isinstance(contour, (tuple, list)):
                contour = _normalize_contour(contour)
                if contour is None:
                    continue
            _, _, rect_w, rect_h = cv2.boundingRect(contour)
            # contourArea不可能大于外接矩形面积。只跳过必然低于原min_area的噪点，
            # 合格轮廓仍以原轮廓面积/矩求结果，不改成像素面积、不改任何门槛。
            if (rect_w - 1) * (rect_h - 1) < min_area:
                continue
        try:
            area = float(cv2.contourArea(contour))
        except Exception:
            contour = _normalize_contour(contour)
            if contour is None:
                continue
            area = float(cv2.contourArea(contour))
        if min_area <= area <= max_area and area > best_area:
            best_area = area
            best_contour = contour

    result = empty_result()
    if best_contour is None:
        return result

    moments = cv2.moments(best_contour)
    if moments["m00"] == 0:
        return result

    cx = int(moments["m10"] / moments["m00"]) + x
    cy = int(moments["m01"] / moments["m00"]) + y
    center = config["center"]
    grab_x = x + center["x"]
    grab_y = y + center["y"]
    rect_x, rect_y, rect_w, rect_h = cv2.boundingRect(best_contour)

    result.update({
        "found": True,
        "cx": cx,
        "cy": cy,
        "dx": cx - grab_x,
        "dy": cy - grab_y,
        "area": int(best_area),
        "rect": (rect_x + x, rect_y + y, rect_w, rect_h),
        "contour": None,
    })
    return result


def expand_roi_mask(mask_roi, image_shape, roi_rect):
    """仅供 MASK/OVERLAY 调参画面使用，把 ROI mask 放回整图坐标。"""
    x, y, w, h = roi_rect
    mask_full = np.zeros(image_shape[:2], dtype=np.uint8)
    mask_full[y:y + h, x:x + w] = mask_roi
    return mask_full


def detect_largest_blob(cv_bgr, config, need_full_mask=True):
    """兼容入口；主循环使用拆开的三个函数以便分别计时。"""
    mask_roi, roi_rect = build_roi_mask(cv_bgr, config)
    result = find_largest_blob(mask_roi, config, roi_rect)
    mask_full = expand_roi_mask(mask_roi, cv_bgr.shape, roi_rect) if need_full_mask else None
    return mask_full, result


def make_preview(cv_bgr, mask_full, result, config, view_mode="OVERLAY"):
    if view_mode == "MASK":
        if mask_full is None:
            raise ValueError("MASK preview requires mask_full")
        preview = cv2.cvtColor(mask_full, cv2.COLOR_GRAY2BGR)
    else:
        preview = cv_bgr.copy()
        if view_mode == "OVERLAY":
            if mask_full is None:
                raise ValueError("OVERLAY preview requires mask_full")
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
        rect_x, rect_y, rect_w, rect_h = result["rect"]
        cv2.rectangle(preview, (rect_x, rect_y),
                      (rect_x + rect_w - 1, rect_y + rect_h - 1), (0, 255, 0), 2)
        cv2.circle(preview, (result["cx"], result["cy"]), 4, (0, 0, 255), -1)
        cv2.line(
            preview, (grab_x, grab_y), (result["cx"], result["cy"]),
            (255, 100, 0), 1
        )
    return preview


def threshold_swatch_bgr(config, color_key):
    """根据阈值中心值生成菜单色块，只用于视觉提示。"""
    threshold = config["thresholds"]["LAB"][color_key]
    lower = threshold["lower"]
    upper = threshold["upper"]
    pixel = np.uint8([[[(lower[0] + upper[0]) // 2,
                        (lower[1] + upper[1]) // 2,
                        (lower[2] + upper[2]) // 2]]])
    return tuple(int(v) for v in cv2.cvtColor(pixel, cv2.COLOR_LAB2BGR)[0, 0])
