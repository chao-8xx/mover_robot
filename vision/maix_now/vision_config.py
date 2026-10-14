"""三任务视觉参数的默认值、校验和单套 JSON 持久化。

坐标契约：本文件保存的 ROI、中心点和面积范围一律属于 320x240 原图；
RUN_PROCESS_WIDTH/HEIGHT 只是运行时优化尺寸，不能反向写入用户参数。
"""

import copy
import json
import os


SCHEMA_VERSION = 9
IMAGE_WIDTH = 320
IMAGE_HEIGHT = 240

# 调参和参数存储始终使用 320x240；RUN 只在内部按此尺寸处理。
RUN_PROCESS_WIDTH = 240
RUN_PROCESS_HEIGHT = 180

MORPH_NONE = "NONE"
MORPH_OPEN_ONLY = "OPEN_ONLY"
MORPH_CLOSE_ONLY = "CLOSE_ONLY"
MORPH_OPEN_CLOSE = "OPEN_CLOSE"
VALID_MORPH_MODES = (
    MORPH_NONE,
    MORPH_OPEN_ONLY,
    MORPH_CLOSE_ONLY,
    MORPH_OPEN_CLOSE,
)

COLOR_ORDER = ("red", "yellow", "blue", "green", "black", "light_blue")
COLOR_LABELS = {
    "red": "红色",
    "yellow": "黄色",
    "blue": "蓝色",
    "green": "绿色",
    "black": "黑色",
    "light_blue": "浅蓝",
}
COLOR_NUMBERS = {
    "red": 1,
    "yellow": 2,
    "blue": 3,
    "green": 4,
    "black": 5,
    "light_blue": 6,
}

# 只用于识别旧NOW默认值的单次迁移，不是第二套现场可调参数。
# 与这些值不同的已保存LAB阈值保持原样；保存为版本9后不再替换。
_LEGACY_NOW_LAB = {
    "red": ([110, 170, 130], [210, 220, 170]),
    "yellow": ([216, 100, 195], [255, 130, 220]),
    "blue": ([85, 138, 50], [138, 175, 80]),
    "green": ([175, 70, 145], [205, 100, 170]),
    "black": ([55, 105, 110], [155, 140, 140]),
    "light_blue": ([145, 105, 75], [195, 135, 100]),
}


DEFAULT_CONFIG = {
    "schema_version": SCHEMA_VERSION,
    # B2/B4每帧只检测指定颜色；固定LAB，参数中不再保存模式开关。
    "target_color": "light_blue",
    # 当前真机性能基线使用 OPEN_ONLY；仍保留另外三档供现场对比。
    "morph_mode": MORPH_OPEN_ONLY,
    # B2保留完整视野，固定全图ROI；RUN仍缩小整幅图到240x180。
    "roi": {"x": 0, "y": 0, "w": IMAGE_WIDTH, "h": IMAGE_HEIGHT},
    # center.x/y 是 ROI 内相对坐标；custom=False 表示使用 ROI 几何中心。
    "center": {
        # 旧main的ROI(110,25,135,135)几何中心为原图(177,92)。
        # NOW保留全图视野，因此明确保存为自定义绝对对准点。
        "x": 177,
        "y": 92,
        "custom": True,
    },
    # 面积范围同样按 320x240 等效面积保存，RUN 内部会自动按像素比例换算。
    "blob": {"min_area": 1000, "max_area": 14000},
    # 任务三复用颜色阈值和ColorTracker，但保留独立ROI、对准中心和面积范围。
    "stack": {
        "roi": {"x": 110, "y": 25, "w": 135, "h": 135},
        "center": {
            "x": IMAGE_WIDTH // 2,
            "y": IMAGE_HEIGHT // 2,
            "custom": False,
        },
        "blob": {"min_area": 1000, "max_area": 12000},
    },
    "thresholds": {
        # 按用户提供的maix_main现场配置迁入六色LAB；B2/B4共享此表。
        "LAB": {
            "red": {"lower": [110, 170, 130], "upper": [210, 220, 170]},
            "yellow": {"lower": [190, 100, 185], "upper": [230, 130, 220]},
            "blue": {"lower": [50, 135, 50], "upper": [138, 175, 80]},
            "green": {"lower": [150, 70, 128], "upper": [205, 120, 170]},
            "black": {"lower": [55, 105, 110], "upper": [155, 140, 140]},
            "light_blue": {"lower": [145, 105, 75], "upper": [195, 135, 100]},
        },
    },
    # F407先到2号附近，B3只定位几何圆心，不识别数字。
    "ring": {
        # 任务位置约定为2，但这不是OCR识别结果。
        "target_number": 2,
        "roi": {"x": 0, "y": 0, "w": IMAGE_WIDTH, "h": IMAGE_HEIGHT},
        "center": {
            "x": IMAGE_WIDTH // 2,
            "y": IMAGE_HEIGHT // 2,
            "custom": False,
        },
        # 灰度值不高于该值的像素作为黑色候选。
        "black_v_max": 125,
        # 普通页面只调节最终圆环组的外径范围，单位为320x240原图像素。
        "min_diameter": 45,
        "max_diameter": 120,
        # 以下为同心候选内部默认值，后续有真机样本后再决定是否开放高级页面。
        "min_contour_area": 2000,
        "max_contour_area": 12000,
        "min_circularity": 0.20,
        "min_layers": 3,
        "center_tolerance": 8,
        # 跟踪窗口在圆环外径外再留48px移动余量；层数每5帧复核一次。
        "track_padding": 48,
        "layer_check_interval": 5,
    },
}


def clone_config(config):
    return copy.deepcopy(config)


def _clamp(value, low, high):
    try:
        value = int(value)
    except (TypeError, ValueError):
        value = low
    return max(low, min(high, value))


def _clamp_float(value, low, high):
    try:
        value = float(value)
    except (TypeError, ValueError):
        value = low
    return max(low, min(high, value))


def _validated_triplet(value, fallback, limits):
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        value = fallback
    return [_clamp(value[i], limits[i][0], limits[i][1]) for i in range(3)]


def _validated_threshold(value, fallback):
    if not isinstance(value, dict):
        value = {}
    limits = ((0, 255), (0, 255), (0, 255))
    lower = _validated_triplet(value.get("lower"), fallback["lower"], limits)
    upper = _validated_triplet(value.get("upper"), fallback["upper"], limits)

    for index in range(3):
        if lower[index] > upper[index]:
            lower[index], upper[index] = upper[index], lower[index]
    return {"lower": lower, "upper": upper}


def validate_config(value):
    """只接受已知字段，并将所有坐标和阈值限制在安全范围内。"""
    source = value if isinstance(value, dict) else {}
    result = clone_config(DEFAULT_CONFIG)
    migrate_defaults = _clamp(source.get("schema_version", 0), 0, 9999) < 9

    target = source.get("target_color", result["target_color"])
    if target in COLOR_ORDER:
        result["target_color"] = target

    morph_mode = str(source.get("morph_mode", result["morph_mode"])).upper()
    if morph_mode in VALID_MORPH_MODES:
        result["morph_mode"] = morph_mode

    roi_source = source.get("roi", {})
    if not isinstance(roi_source, dict):
        roi_source = {}
    w = _clamp(roi_source.get("w", IMAGE_WIDTH), 20, IMAGE_WIDTH)
    h = _clamp(roi_source.get("h", IMAGE_HEIGHT), 20, IMAGE_HEIGHT)
    x = _clamp(roi_source.get("x", 0), 0, IMAGE_WIDTH - w)
    y = _clamp(roi_source.get("y", 0), 0, IMAGE_HEIGHT - h)
    result["roi"] = {"x": 0, "y": 0, "w": IMAGE_WIDTH, "h": IMAGE_HEIGHT}

    center_source = source.get("center", {})
    if not isinstance(center_source, dict):
        center_source = {}
    small_roi = (x, y, w, h) != (0, 0, IMAGE_WIDTH, IMAGE_HEIGHT)
    center_custom = bool(center_source.get("custom", False if small_roi else result["center"]["custom"]))
    fallback_x = w // 2 if small_roi else result["center"]["x"]
    fallback_y = h // 2 if small_roi else result["center"]["y"]
    result["center"] = {
        # 从旧小ROI迁移时保留对准点的原图绝对位置。
        "x": _clamp(x + _clamp(center_source.get("x", fallback_x), 0, w - 1),
                    0, IMAGE_WIDTH - 1),
        "y": _clamp(y + _clamp(center_source.get("y", fallback_y), 0, h - 1),
                    0, IMAGE_HEIGHT - 1),
        "custom": center_custom,
    }
    if not result["center"]["custom"]:
        if small_roi:
            # 原来使用小ROI几何中心时，迁到全图后同样保留原图对准位置。
            result["center"] = {"x": x + w // 2, "y": y + h // 2, "custom": True}
        elif migrate_defaults:
            result["center"] = clone_config(DEFAULT_CONFIG["center"])
        else:
            # 版本9后用户主动选择全图几何中心，仍允许保存(160,120)。
            result["center"] = {"x": IMAGE_WIDTH // 2, "y": IMAGE_HEIGHT // 2, "custom": False}

    blob_source = source.get("blob", {})
    if not isinstance(blob_source, dict):
        blob_source = {}
    min_area = _clamp(blob_source.get("min_area", result["blob"]["min_area"]),
                      1, IMAGE_WIDTH * IMAGE_HEIGHT)
    max_area = _clamp(
        blob_source.get("max_area", result["blob"]["max_area"]),
        min_area, IMAGE_WIDTH * IMAGE_HEIGHT
    )
    result["blob"] = {"min_area": min_area, "max_area": max_area}
    if migrate_defaults and result["blob"] == {"min_area": 1800, "max_area": 12000}:
        result["blob"] = clone_config(DEFAULT_CONFIG["blob"])

    stack_source = source.get("stack", {})
    if not isinstance(stack_source, dict):
        stack_source = {}
    stack_result = result["stack"]
    # 旧配置缺少某个ROI字段时沿用本应用标定值，不退回旧全图默认值。
    stack_default_roi = stack_result["roi"]
    stack_roi_source = stack_source.get("roi", {})
    if not isinstance(stack_roi_source, dict):
        stack_roi_source = {}
    stack_w = _clamp(
        stack_roi_source.get("w", stack_default_roi["w"]), 20, IMAGE_WIDTH
    )
    stack_h = _clamp(
        stack_roi_source.get("h", stack_default_roi["h"]), 20, IMAGE_HEIGHT
    )
    stack_x = _clamp(
        stack_roi_source.get("x", stack_default_roi["x"]), 0, IMAGE_WIDTH - stack_w
    )
    stack_y = _clamp(
        stack_roi_source.get("y", stack_default_roi["y"]), 0, IMAGE_HEIGHT - stack_h
    )
    stack_result["roi"] = {
        "x": stack_x, "y": stack_y, "w": stack_w, "h": stack_h
    }

    stack_center_source = stack_source.get("center", {})
    if not isinstance(stack_center_source, dict):
        stack_center_source = {}
    stack_result["center"] = {
        "x": _clamp(
            stack_center_source.get("x", stack_w // 2), 0, stack_w - 1
        ),
        "y": _clamp(
            stack_center_source.get("y", stack_h // 2), 0, stack_h - 1
        ),
        "custom": bool(stack_center_source.get("custom", False)),
    }
    if not stack_result["center"]["custom"]:
        stack_result["center"]["x"] = stack_w // 2
        stack_result["center"]["y"] = stack_h // 2

    stack_blob_source = stack_source.get("blob", {})
    if not isinstance(stack_blob_source, dict):
        stack_blob_source = {}
    stack_min_area = _clamp(
        stack_blob_source.get("min_area", stack_result["blob"]["min_area"]),
        1,
        IMAGE_WIDTH * IMAGE_HEIGHT,
    )
    stack_max_area = _clamp(
        stack_blob_source.get("max_area", stack_result["blob"]["max_area"]),
        stack_min_area,
        IMAGE_WIDTH * IMAGE_HEIGHT,
    )
    stack_result["blob"] = {
        "min_area": stack_min_area, "max_area": stack_max_area
    }

    thresholds_source = source.get("thresholds", {})
    if not isinstance(thresholds_source, dict):
        thresholds_source = {}
    # 旧配置只取明确标记的LAB分支；HSV值绝不能解释为LAB。
    lab_source = thresholds_source.get("LAB", {})
    if not isinstance(lab_source, dict):
        lab_source = {}
    for color_key in COLOR_ORDER:
        fallback = DEFAULT_CONFIG["thresholds"]["LAB"][color_key]
        result["thresholds"]["LAB"][color_key] = _validated_threshold(
            lab_source.get(color_key), fallback
        )
        lower, upper = _LEGACY_NOW_LAB[color_key]
        if migrate_defaults and result["thresholds"]["LAB"][color_key] == {"lower": lower, "upper": upper}:
            result["thresholds"]["LAB"][color_key] = clone_config(fallback)

    ring_source = source.get("ring", {})
    if not isinstance(ring_source, dict):
        ring_source = {}
    ring_result = result["ring"]
    # 2是F407约定的任务位置；视觉端不再通过OCR确认身份。
    ring_result["target_number"] = 2

    # 圆环ROI坐标属于完整320x240画面；center是ROI内相对坐标。
    ring_roi_source = ring_source.get("roi", {})
    if not isinstance(ring_roi_source, dict):
        ring_roi_source = {}
    ring_w = _clamp(
        ring_roi_source.get("w", IMAGE_WIDTH), 20, IMAGE_WIDTH
    )
    ring_h = _clamp(
        ring_roi_source.get("h", IMAGE_HEIGHT), 20, IMAGE_HEIGHT
    )
    ring_x = _clamp(
        ring_roi_source.get("x", 0), 0, IMAGE_WIDTH - ring_w
    )
    ring_y = _clamp(
        ring_roi_source.get("y", 0), 0, IMAGE_HEIGHT - ring_h
    )
    ring_result["roi"] = {
        "x": ring_x, "y": ring_y, "w": ring_w, "h": ring_h
    }

    ring_center_source = ring_source.get("center", {})
    if not isinstance(ring_center_source, dict):
        ring_center_source = {}
    center_custom = bool(ring_center_source.get("custom", False))
    ring_result["center"] = {
        "x": _clamp(
            ring_center_source.get("x", ring_w // 2), 0, ring_w - 1
        ),
        "y": _clamp(
            ring_center_source.get("y", ring_h // 2), 0, ring_h - 1
        ),
        "custom": center_custom,
    }
    if not ring_result["center"]["custom"]:
        ring_result["center"]["x"] = ring_w // 2
        ring_result["center"]["y"] = ring_h // 2

    ring_result["black_v_max"] = _clamp(
        ring_source.get("black_v_max", ring_result["black_v_max"]), 0, 255
    )
    ring_min_diameter = _clamp(
        ring_source.get("min_diameter", ring_result["min_diameter"]), 8, IMAGE_WIDTH
    )
    ring_max_diameter = _clamp(
        ring_source.get("max_diameter", ring_result["max_diameter"]),
        ring_min_diameter,
        IMAGE_WIDTH,
    )
    ring_result["min_diameter"] = ring_min_diameter
    ring_result["max_diameter"] = ring_max_diameter
    ring_result["min_contour_area"] = _clamp(
        ring_source.get("min_contour_area", ring_result["min_contour_area"]),
        5,
        IMAGE_WIDTH * IMAGE_HEIGHT,
    )
    ring_result["max_contour_area"] = _clamp(
        ring_source.get("max_contour_area", ring_result["max_contour_area"]),
        ring_result["min_contour_area"],
        IMAGE_WIDTH * IMAGE_HEIGHT,
    )
    ring_result["min_circularity"] = _clamp_float(
        ring_source.get("min_circularity", ring_result["min_circularity"]),
        0.0,
        1.0,
    )
    ring_result["min_layers"] = _clamp(
        ring_source.get("min_layers", ring_result["min_layers"]), 2, 12
    )
    ring_result["center_tolerance"] = _clamp(
        ring_source.get("center_tolerance", ring_result["center_tolerance"]), 2, 40
    )
    ring_result["track_padding"] = _clamp(
        ring_source.get("track_padding", ring_result["track_padding"]), 12, 120
    )
    ring_result["layer_check_interval"] = _clamp(
        ring_source.get("layer_check_interval", ring_result["layer_check_interval"]),
        1, 10,
    )

    result["schema_version"] = SCHEMA_VERSION
    return result


class VisionConfigStore:
    def __init__(self, data_dir, filename="vision_now_config.json"):
        self.data_dir = data_dir or "./data"
        self.path = os.path.join(self.data_dir, filename)

    def load(self):
        if not os.path.exists(self.path):
            return validate_config(DEFAULT_CONFIG), "使用默认参数"
        try:
            with open(self.path, "r", encoding="utf-8") as config_file:
                raw = json.load(config_file)
            if str(raw.get("mode", "")).upper() == "HSV":
                return validate_config(raw), "已迁移LAB，请检查颜色阈值"
            if raw.get("schema_version") != SCHEMA_VERSION:
                return validate_config(raw), "参数已迁移，请检查B2标定"
            return validate_config(raw), "参数加载成功"
        except Exception as exc:
            print("[VISION CONFIG] load failed:", exc)
            return validate_config(DEFAULT_CONFIG), "参数损坏，已用默认值"

    def save(self, config):
        value = validate_config(config)
        temp_path = self.path + ".tmp"
        try:
            if not os.path.exists(self.data_dir):
                os.makedirs(self.data_dir)
            with open(temp_path, "w", encoding="utf-8") as config_file:
                json.dump(value, config_file, ensure_ascii=False, indent=2)
                config_file.flush()
            try:
                os.replace(temp_path, self.path)
            except AttributeError:
                if os.path.exists(self.path):
                    os.remove(self.path)
                os.rename(temp_path, self.path)
            return value, "保存成功"
        except Exception as exc:
            print("[VISION CONFIG] save failed:", exc)
            try:
                if os.path.exists(temp_path):
                    os.remove(temp_path)
            except Exception:
                pass
            return None, "保存失败"


def default_threshold(color_key):
    return clone_config(DEFAULT_CONFIG["thresholds"]["LAB"][color_key])
