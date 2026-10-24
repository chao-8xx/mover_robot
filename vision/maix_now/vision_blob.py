"""ROI 内最大色块检测。

职责边界：只处理图像、坐标和结果映射，不处理 UI、串口或参数文件。
对外结果统一使用 320x240 坐标，方便未来任务二、任务三共用主循环接口。
"""

import cv2
import numpy as np
from time import perf_counter

from vision_config import (
    IMAGE_WIDTH,
    IMAGE_HEIGHT,
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
B2_PIPELINE_BUILD = "B2_POLYGON_V5"
_POLYGON_MASK_CACHE = {}


def polygon_exclusion_mask(roi_rect, polygons, process_size=None):
    """只在几何参数变化时绘制；RUN最近邻采样同一份原图屏蔽形状。"""
    if not polygons:
        return None
    key = (tuple(roi_rect), process_size,
           tuple(tuple(tuple(p) for p in polygon) for polygon in polygons))
    if key in _POLYGON_MASK_CACHE:
        return _POLYGON_MASK_CACHE[key]
    x, y, w, h = roi_rect
    mask = np.zeros((IMAGE_HEIGHT, IMAGE_WIDTH), dtype=np.uint8)
    for polygon in polygons:
        points = np.asarray(polygon, dtype=np.int32)
        # 分别填充以取并集；一次传多个重叠轮廓会产生奇偶填充的空洞。
        cv2.fillPoly(mask, [points.reshape((-1, 1, 2))], 255)
    # 先在原图光栅化再裁剪，避免斜边在不同ROI边界裁切时产生像素偏移。
    mask = mask[y:y + h, x:x + w]
    if process_size is not None and process_size != (w, h):
        mask = cv2.resize(mask, process_size, interpolation=cv2.INTER_NEAREST)
    excluded = mask != 0
    if len(_POLYGON_MASK_CACHE) >= 4:
        _POLYGON_MASK_CACHE.clear()
    _POLYGON_MASK_CACHE[key] = excluded
    return excluded


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
    # B2的原图参考点允许落在裁剪区外，不能夹到ROI边缘而改变dx/dy原点。
    scaled_center_x = int(round(center["x"] * scale_x))
    scaled_center_y = int(round(center["y"] * scale_y))

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
    if "exclude_rois" in config:
        process_config["exclude_rois"] = []
        for rect in config["exclude_rois"]:
            # INTER_NEAREST采样原图floor(i*source/processed)，两端均向上取整。
            left = max(0, ((rect["x"] - source_x) * scaled_w + source_w - 1) // source_w)
            top = max(0, ((rect["y"] - source_y) * scaled_h + source_h - 1) // source_h)
            right = min(scaled_w, ((rect["x"] + rect["w"] - source_x) * scaled_w + source_w - 1) // source_w)
            bottom = min(scaled_h, ((rect["y"] + rect["h"] - source_y) * scaled_h + source_h - 1) // source_h)
            if right > left and bottom > top:
                process_config["exclude_rois"].append(dict(x=left, y=top, w=right-left, h=bottom-top))
    process_config["_exclude_polygon_mask"] = polygon_exclusion_mask(
        (source_x, source_y, source_w, source_h), config.get("exclude_polygons", []),
        (scaled_w, scaled_h))
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
    mapped = dict(result)
    mapped.update({
        "found": True,
        "cx": cx,
        "cy": cy,
        "dx": dx,
        "dy": dy,
        "area": int(round(result["area"] / area_scale)),
        "rect": mapped_rect,
        "contour": None,
    })
    return mapped


def scale_grasp_roi(grasp, geometry):
    """半开区间左上向下、右下向上取整，至少保留1个处理像素。

    使用整数比例避免边缘浮点误差；分母也是此处理图ROI的完整像素数。
    此框仅用于覆盖统计，绝不成为检测的搜索ROI。
    """
    roi = grasp["roi"]
    sx, sy, sw, sh = geometry["source_roi"]
    _, _, pw, ph = geometry["process_roi"]
    # 保留完整抓取框的投影，不能把超出搜索ROI的部分裁掉并缩小分母。
    left = (roi["x"] - sx) * pw // sw
    top = (roi["y"] - sy) * ph // sh
    right = ((roi["x"] + roi["w"] - sx) * pw + sw - 1) // sw
    bottom = ((roi["y"] + roi["h"] - sy) * ph + sh - 1) // sh
    return left, top, right - left, bottom - top


def selected_coverage(mask, contour, rect, roi_rect, grasp_roi, preview=None):
    """只计选中轮廓所属的8连通颜色区域，空洞和其他同色岛均不贡献。

    仅复制该候选外接框，种子取自现有轮廓（重心可能落在空洞中）。
    不填满外轮廓、不重做全图阈值，也不对全图执行连通域标记。
    """
    gx, gy, gw, gh = grasp_roi
    if preview is not None:
        # 仅LCD预览帧分配小块掩膜；它与计数使用同一个连通区域，不重做检测。
        preview.update(rect=grasp_roi, mask=np.zeros((gh, gw), dtype=np.uint8))
    gx -= roi_rect[0]
    gy -= roi_rect[1]
    x, y, w, h = rect
    left, top = max(x, gx), max(y, gy)
    right, bottom = min(x + w, gx + gw), min(y + h, gy + gh)
    if left >= right or top >= bottom:
        return 0, gw * gh
    contained = (left, top, right, bottom) == (x, y, x + w, y + h)
    # 部分重叠但交集无颜色时，贡献必然为0；整候选被包含时无需这次预检。
    if not contained and cv2.countNonZero(mask[top:bottom, left:right]) == 0:
        return 0, gw * gh
    component = mask[y:y + h, x:x + w].copy()
    point = np.asarray(contour).reshape(-1, 2)[0]
    seed = (int(point[0]) - x, int(point[1]) - y)
    filled, _, _, _ = cv2.floodFill(component, None, seed, 128, flags=8)
    if contained and preview is None:
        # 抓取框包含整个候选时，直接复用洪泛返回的连通像素数。
        return int(filled), gw * gh
    overlap = component[top - y:bottom - y, left - x:right - x]
    valid = overlap == 128
    if preview is not None:
        preview["mask"][top - gy:bottom - gy, left - gx:right - gx][valid] = 255
    return int(np.count_nonzero(valid)), gw * gh


def apply_grasp_status(result, grasp):
    """在320x240原图坐标中判中心；用整数交叉相乘保证恰好达标通过。"""
    if not result["found"]:
        return empty_result()
    roi = grasp["roi"]
    inside = (roi["x"] <= result["cx"] < roi["x"] + roi["w"] and
              roi["y"] <= result["cy"] < roi["y"] + roi["h"])
    total = result.get("coverage_total", 0)
    pixels = result.get("coverage_pixels", 0)
    enough = total > 0 and pixels * 100 >= total * grasp["min_coverage_pct"]
    result["ok"] = 2 if inside and enough else 1
    return result


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
    roi = config["roi"]
    rect = tuple(roi[k] for k in ("x", "y", "w", "h"))
    polygon_mask = polygon_exclusion_mask(rect, config.get("exclude_polygons", []))
    clear_exclusions(mask, rect, config.get("exclude_rois", []), polygon_mask)
    mask = clean_mask(mask, config.get("morph_mode", MORPH_OPEN_CLOSE))
    clear_exclusions(mask, rect, config.get("exclude_rois", []), polygon_mask)
    return mask


def clear_exclusions(mask, roi_rect, exclusions, polygon_mask=None):
    """只在二值掩膜置零，不涂原图，保证黑色目标也不会把屏蔽区当作目标。"""
    x, y, w, h = roi_rect
    for rect in exclusions:
        left, top = max(0, rect["x"] - x), max(0, rect["y"] - y)
        right = min(w, rect["x"] + rect["w"] - x)
        bottom = min(h, rect["y"] + rect["h"] - y)
        if right > left and bottom > top:
            mask[top:bottom, left:right] = 0
    if polygon_mask is not None:
        mask[polygon_mask] = 0


def detect_color(roi_bgr, config, roi_rect, need_mask=True, buffers=None, fast_filter=False,
                 grasp_roi=None, grasp_preview=False):
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
    exclusions = config.get("exclude_rois", [])
    polygon_mask = (config["_exclude_polygon_mask"] if "_exclude_polygon_mask" in config else
                    polygon_exclusion_mask(roi_rect, config.get("exclude_polygons", [])))
    clear_exclusions(mask, roi_rect, exclusions, polygon_mask)
    mode = config.get("morph_mode", MORPH_OPEN_CLOSE)
    mask = (clean_mask(mask, mode) if buffers is None else
            clean_mask(mask, mode, dst=buffers.cleaned))
    if mode != "NONE":
        clear_exclusions(mask, roi_rect, exclusions, polygon_mask)
    morph_ms = (perf_counter() - tick) * 1000.0
    tick = perf_counter()
    options = {"grasp_roi": grasp_roi} if grasp_roi is not None else {}
    if fast_filter:
        options["fast_filter"] = True
    if grasp_preview:
        options["grasp_preview"] = True
    result = find_largest_blob(mask, config, roi_rect, **options)
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
        "ok": 0,
        "coverage_pixels": 0,
        "coverage_total": 0,
        "coverage_ratio": 0.0,
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


def find_largest_blob(mask_roi, config, roi_rect, fast_filter=False, grasp_roi=None,
                      grasp_preview=False):
    """在已有 ROI 掩膜中只选择面积最大的合格色块并计算中心偏差。"""
    x, y, w, h = roi_rect
    min_area = config["blob"]["min_area"]
    max_area = config["blob"]["max_area"]
    best_contour = None
    best_area = 0.0
    best_rect = None
    for contour in _find_contours(mask_roi):
        contour_rect = None
        if fast_filter:
            # 某些设备绑定返回tuple/list；直接规范化，避免每个轮廓先抛一次异常。
            if isinstance(contour, (tuple, list)):
                contour = _normalize_contour(contour)
                if contour is None:
                    continue
            contour_rect = cv2.boundingRect(contour)
            _, _, rect_w, rect_h = contour_rect
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
            best_rect = contour_rect

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
    rect_x, rect_y, rect_w, rect_h = (best_rect if best_rect is not None else
                                     cv2.boundingRect(best_contour))

    result.update({
        "found": True,
        "ok": 1,
        "cx": cx,
        "cy": cy,
        "dx": cx - grab_x,
        "dy": cy - grab_y,
        "area": int(best_area),
        "rect": (rect_x + x, rect_y + y, rect_w, rect_h),
        "contour": None,
    })
    if grasp_roi is not None:
        tick = perf_counter()
        counted = {} if grasp_preview else None
        pixels, total = selected_coverage(
            mask_roi, best_contour, (rect_x, rect_y, rect_w, rect_h), roi_rect, grasp_roi,
            preview=counted)
        result.update(coverage_pixels=pixels, coverage_total=total,
                      coverage_ratio=float(pixels) / total,
                      coverage_ms=(perf_counter() - tick) * 1000.0)
        if counted is not None:
            counted["contour"] = (np.asarray(best_contour, dtype=np.int32).reshape(-1, 1, 2) +
                                   np.array((x, y), dtype=np.int32))
            result["grasp_preview"] = counted
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
    detection = config
    if "exclude_rois" in config:
        detection = dict(config)
        detection["center"] = dict(config["center"])
        detection["center"]["x"] -= roi_rect[0]
        detection["center"]["y"] -= roi_rect[1]
    result = find_largest_blob(mask_roi, detection, roi_rect)
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

    counted = result.get("grasp_preview")
    if "exclude_rois" in config:
        # 仅调试显示灰色排除范围；检测输入/相机图像保持原样。
        roi = config["roi"]
        x, y, w, h = (roi[k] for k in ("x", "y", "w", "h"))
        preview[:y] //= 3
        preview[y + h:] //= 3
        preview[y:y + h, :x] //= 3
        preview[y:y + h, x + w:] //= 3
        for rect in config["exclude_rois"]:
            ex, ey, ew, eh = (rect[k] for k in ("x", "y", "w", "h"))
            preview[ey:ey + eh, ex:ex + ew] //= 3
            cv2.rectangle(preview, (ex, ey), (ex + ew - 1, ey + eh - 1), (0, 140, 255), 1)
        polygons = config.get("exclude_polygons", [])
        polygon_mask = polygon_exclusion_mask((0, 0, preview.shape[1], preview.shape[0]), polygons)
        if polygon_mask is not None:
            preview[polygon_mask] //= 3
            for polygon in polygons:
                cv2.polylines(preview, [np.asarray(polygon, dtype=np.int32).reshape((-1, 1, 2))],
                              True, (0, 140, 255), 1)
    if counted is not None:
        # 青色仅覆盖真正计入分子的像素；原图中的亮面/漏分割区域仍保持原色。
        gx, gy, gw, gh = counted["rect"]
        patch = preview[gy:gy + gh, gx:gx + gw]
        tint = np.empty_like(patch)
        tint[:] = (255, 255, 0)
        tint = cv2.addWeighted(patch, 0.35, tint, 0.65, 0)
        valid = counted["mask"] != 0
        patch[valid] = tint[valid]

    roi = config["roi"]
    x, y, w, h = roi["x"], roi["y"], roi["w"], roi["h"]
    cv2.rectangle(preview, (x, y), (x + w - 1, y + h - 1), (0, 220, 255), 2)

    center = config["center"]
    grab_x = center["x"] if "exclude_rois" in config else x + center["x"]
    grab_y = center["y"] if "exclude_rois" in config else y + center["y"]
    cv2.drawMarker(
        preview, (grab_x, grab_y), (255, 255, 255),
        markerType=cv2.MARKER_CROSS, markerSize=14, thickness=1
    )

    if result["found"]:
        rect_x, rect_y, rect_w, rect_h = result["rect"]
        if counted is not None:
            # B2展示真实选中轮廓，避免外接矩形让人误以为矩形内部全部有效。
            cv2.drawContours(preview, [counted["contour"]], -1, (0, 255, 0), 1)
        else:
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
