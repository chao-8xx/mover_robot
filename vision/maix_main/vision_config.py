"""物料识别参数的默认值、校验和单套 JSON 持久化。"""

import copy
import json
import os


SCHEMA_VERSION = 1
IMAGE_WIDTH = 320
IMAGE_HEIGHT = 240

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


DEFAULT_CONFIG = {
    "schema_version": SCHEMA_VERSION,
    "mode": "LAB",
    "target_color": "red",
    "roi": {"x": 0, "y": 0, "w": IMAGE_WIDTH, "h": IMAGE_HEIGHT},
    # x/y 是 ROI 内的相对坐标。custom=False 表示使用 ROI 几何中心。
    "center": {
        "x": IMAGE_WIDTH // 2,
        "y": IMAGE_HEIGHT // 2,
        "custom": False,
    },
    "blob": {"min_area": 300, "max_area": 60000},
    "thresholds": {
        "HSV": {
            # 红色 H 起点大于终点表示跨越 179 -> 0，检测时自动拆为两段。
            "red": {"lower": [160, 80, 50], "upper": [10, 255, 255]},
            "yellow": {"lower": [18, 80, 60], "upper": [35, 255, 255]},
            "blue": {"lower": [100, 70, 40], "upper": [130, 255, 255]},
            "green": {"lower": [35, 50, 40], "upper": [85, 255, 255]},
            "black": {"lower": [0, 0, 0], "upper": [179, 255, 60]},
            "light_blue": {"lower": [85, 40, 60], "upper": [105, 255, 255]},
        },
        "LAB": {
            "red": {"lower": [20, 145, 130], "upper": [255, 210, 210]},
            "yellow": {"lower": [80, 115, 145], "upper": [255, 160, 230]},
            "blue": {"lower": [20, 110, 60], "upper": [220, 170, 130]},
            "green": {"lower": [20, 60, 80], "upper": [220, 125, 170]},
            "black": {"lower": [0, 0, 0], "upper": [70, 255, 255]},
            "light_blue": {"lower": [80, 90, 70], "upper": [255, 150, 135]},
        },
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


def _validated_triplet(value, fallback, limits):
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        value = fallback
    return [_clamp(value[i], limits[i][0], limits[i][1]) for i in range(3)]


def _validated_threshold(mode, value, fallback):
    if not isinstance(value, dict):
        value = {}
    limits = ((0, 179), (0, 255), (0, 255)) if mode == "HSV" else (
        (0, 255), (0, 255), (0, 255)
    )
    lower = _validated_triplet(value.get("lower"), fallback["lower"], limits)
    upper = _validated_triplet(value.get("upper"), fallback["upper"], limits)

    # HSV 的 H 允许 lower > upper，用来表达红色跨零区间；其他通道保持 min <= max。
    start_index = 1 if mode == "HSV" else 0
    for index in range(start_index, 3):
        if lower[index] > upper[index]:
            lower[index], upper[index] = upper[index], lower[index]
    return {"lower": lower, "upper": upper}


def validate_config(value):
    """只接受已知字段，并将所有坐标和阈值限制在安全范围内。"""
    source = value if isinstance(value, dict) else {}
    result = clone_config(DEFAULT_CONFIG)

    mode = str(source.get("mode", result["mode"])).upper()
    result["mode"] = mode if mode in ("HSV", "LAB") else result["mode"]

    target = source.get("target_color", result["target_color"])
    if target in COLOR_ORDER:
        result["target_color"] = target

    roi_source = source.get("roi", {})
    if not isinstance(roi_source, dict):
        roi_source = {}
    w = _clamp(roi_source.get("w", IMAGE_WIDTH), 20, IMAGE_WIDTH)
    h = _clamp(roi_source.get("h", IMAGE_HEIGHT), 20, IMAGE_HEIGHT)
    x = _clamp(roi_source.get("x", 0), 0, IMAGE_WIDTH - w)
    y = _clamp(roi_source.get("y", 0), 0, IMAGE_HEIGHT - h)
    result["roi"] = {"x": x, "y": y, "w": w, "h": h}

    center_source = source.get("center", {})
    if not isinstance(center_source, dict):
        center_source = {}
    result["center"] = {
        "x": _clamp(center_source.get("x", w // 2), 0, w - 1),
        "y": _clamp(center_source.get("y", h // 2), 0, h - 1),
        "custom": bool(center_source.get("custom", False)),
    }
    if not result["center"]["custom"]:
        result["center"]["x"] = w // 2
        result["center"]["y"] = h // 2

    blob_source = source.get("blob", {})
    if not isinstance(blob_source, dict):
        blob_source = {}
    min_area = _clamp(blob_source.get("min_area", 300), 1, IMAGE_WIDTH * IMAGE_HEIGHT)
    max_area = _clamp(
        blob_source.get("max_area", 60000), min_area, IMAGE_WIDTH * IMAGE_HEIGHT
    )
    result["blob"] = {"min_area": min_area, "max_area": max_area}

    thresholds_source = source.get("thresholds", {})
    if not isinstance(thresholds_source, dict):
        thresholds_source = {}
    for threshold_mode in ("HSV", "LAB"):
        mode_source = thresholds_source.get(threshold_mode, {})
        if not isinstance(mode_source, dict):
            mode_source = {}
        for color_key in COLOR_ORDER:
            fallback = DEFAULT_CONFIG["thresholds"][threshold_mode][color_key]
            result["thresholds"][threshold_mode][color_key] = _validated_threshold(
                threshold_mode, mode_source.get(color_key), fallback
            )

    result["schema_version"] = SCHEMA_VERSION
    return result


class VisionConfigStore:
    def __init__(self, data_dir, filename="vision_config.json"):
        self.data_dir = data_dir or "./data"
        self.path = os.path.join(self.data_dir, filename)

    def load(self):
        if not os.path.exists(self.path):
            return clone_config(DEFAULT_CONFIG), "使用默认参数"
        try:
            with open(self.path, "r", encoding="utf-8") as config_file:
                raw = json.load(config_file)
            if raw.get("schema_version") != SCHEMA_VERSION:
                return validate_config(raw), "参数版本已兼容"
            return validate_config(raw), "参数加载成功"
        except Exception as exc:
            print("[VISION CONFIG] load failed:", exc)
            return clone_config(DEFAULT_CONFIG), "参数损坏，已用默认值"

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


def default_threshold(mode, color_key):
    return clone_config(DEFAULT_CONFIG["thresholds"][mode][color_key])
