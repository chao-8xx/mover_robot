"""三任务视觉参数的默认值、校验和单套 JSON 持久化。

坐标契约：本文件保存的 ROI、中心点和面积范围一律属于 320x240 原图；
RUN_PROCESS_WIDTH/HEIGHT 只是运行时优化尺寸，不能反向写入用户参数。
"""

import copy
import json
import os


SCHEMA_VERSION = 13
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

DEFAULT_CONFIG = {
    "schema_version": SCHEMA_VERSION,
    # B2/B4每帧只检测指定颜色；固定LAB，参数中不再保存模式开关。
    "target_color": "yellow",
    # 当前真机性能基线使用 OPEN_ONLY；仍保留另外三档供现场对比。
    "morph_mode": MORPH_OPEN_ONLY,
    # B2黑色/浅蓝每秒采样一次掩膜及候选诊断；不修改检测阈值或候选排序。
    "color_diagnostics": True,
    # B2外层检测ROI：保留左右/上边界，底边由230收至215，排除盘下背景。
    # RUN保持约0.75的采样比例，只缩小此ROI，不将裁剪区拉伸成整幅240x180。
    "roi": {"x": 65, "y": 30, "w": 220, "h": 185},
    # B2内部屏蔽区：320x240原图坐标，只清检测掩膜，不把原图涂黑。
    # 按本轮实拍移至盘内的夹爪暗面；须检查全行程，目标不能被屏蔽切掉。[]可关闭。
    "exclude_rois": [
        {"x": 0, "y": 0, "w": 0, "h": 0},   # 左夹爪暗面
        {"x": 0, "y": 0, "w": 0, "h": 0},  # 右夹爪暗面
    ],
    # B2多边形屏蔽：每个列表是一块封闭区域，顶点沿边界顺序填写[x,y]。
    # 圆弧用多个顶点近似：沿外弧走，再沿内弧反向返回，形成有厚度的弧带。
    # 沿现有外沿拆成四个独立区域；物料高度余量仍须按实拍确认。[]可关闭。
    "exclude_polygons": [
        [[65, 115], [65, 30], [124, 30], [66, 115]],    # 左上屏蔽区
        [[236, 30], [285, 30], [285, 115], [284, 115]], # 右上屏蔽区
        [[65, 115], [124, 230], [65, 230]],            # 左下屏蔽区
        [[284, 115], [285, 230], [211, 230]],          # 右下屏蔽区
    ],
    # B2 center.x/y为原图绝对坐标，裁剪外层ROI不能移动此参考点。
    # custom=False使用原图中心(160,120)；stack/ring的center仍为各自ROI内相对坐标。
    "center": {
        # 旧main的ROI(110,25,135,135)几何中心为原图(177,92)。
        # NOW保留全图视野，因此明确保存为自定义绝对对准点。
        "x": 163,
        "y": 116,
        "custom": True,
    },
    # 面积范围同样按 320x240 等效面积保存，RUN 内部会自动按像素比例换算。
    "blob": {"min_area": 500, "max_area": 1800},
    # B2抓取判定区：只判定覆盖，不裁剪全图搜索、不改变dx/dy参考点。
    # 320x240原图坐标；40x40/60%只是初试值，新镜头/夹爪必须实物标定。
    "grasp": {
        "roi": {"x": 121, "y": 74, "w": 105, "h": 80},
        "min_coverage_pct": 40,
    },
    # 任务三复用颜色阈值和ColorTracker，但保留独立ROI、对准中心和面积范围。
    "stack": {
        "roi": {"x": 121, "y": 74, "w": 105, "h": 80},
        "center": {
            "x": 163,
            "y": 116,
            "custom": False,
        },
        "blob": {"min_area": 300, "max_area": 800},
    },
    "thresholds": {
        # 按用户提供的maix_main现场配置迁入六色LAB；B2/B4共享此表。
        "LAB": {
            "red": {"lower": [75, 170, 130], "upper": [210, 220, 170]},
            "yellow": {"lower": [150, 100, 155], "upper": [245, 130, 220]},
            "blue": {"lower": [35, 135, 50], "upper": [138, 185, 110]},
            "green": {"lower": [102, 70, 128], "upper": [205, 120, 170]},
            "black": {"lower": [0, 118, 116], "upper": [135, 140, 140]},
            "light_blue": {"lower": [85, 100, 70], "upper": [255, 130, 125]},
        },
    },
    # F407先到2号附近，B3只定位几何圆心，不识别数字。
    "ring": {
        # 任务位置约定为2，但这不是OCR识别结果。
        "target_number": 2,
        "roi": {"x": 121, "y": 74, "w": 105, "h": 80},
        "center": {
            "x": 163,
            "y": 116,
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

# 只迁移这一份已知的旧连画点列；其他现场多边形不替换。
_LEGACY_JOINED_POLYGON = [
    [65, 115], [65, 30], [78, 30], [285, 30], [285, 115],
    [280, 115], [230, 30], [130, 30], [68, 115], [65, 115],
    [65, 230], [130, 230], [215, 230], [280, 115], [285, 230], [130, 230],
]


def clone_config(config):
    return copy.deepcopy(config)


def _clamp(value, low, high):
    try:
        value = int(value)
    except (TypeError, ValueError, OverflowError):
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

    target = source.get("target_color", result["target_color"])
    if target in COLOR_ORDER:
        result["target_color"] = target

    morph_mode = str(source.get("morph_mode", result["morph_mode"])).upper()
    if morph_mode in VALID_MORPH_MODES:
        result["morph_mode"] = morph_mode

    result["color_diagnostics"] = bool(source.get("color_diagnostics", result["color_diagnostics"]))

    roi_source = source.get("roi", {})
    if not isinstance(roi_source, dict):
        roi_source = {}
    fallback = DEFAULT_CONFIG["roi"]
    w = _clamp(roi_source.get("w", fallback["w"]), 20, IMAGE_WIDTH)
    h = _clamp(roi_source.get("h", fallback["h"]), 20, IMAGE_HEIGHT)
    x = _clamp(roi_source.get("x", fallback["x"]), 0, IMAGE_WIDTH - w)
    y = _clamp(roi_source.get("y", fallback["y"]), 0, IMAGE_HEIGHT - h)
    result["roi"] = {"x": x, "y": y, "w": w, "h": h}

    center_source = source.get("center", {})
    if not isinstance(center_source, dict):
        center_source = {}
    small_roi = (x, y, w, h) != (0, 0, IMAGE_WIDTH, IMAGE_HEIGHT)
    legacy_local = small_roi and _clamp(source.get("schema_version", 0), 0, 9999) < 11
    custom = bool(center_source.get("custom", False if legacy_local else result["center"]["custom"]))
    if legacy_local:
        # v10及以前小ROI中心是局部坐标，仅迁移一次，保持原图位置。
        cx = x + (_clamp(center_source.get("x", w // 2), 0, w - 1) if custom else w // 2)
        cy = y + (_clamp(center_source.get("y", h // 2), 0, h - 1) if custom else h // 2)
        result["center"] = {"x": cx, "y": cy, "custom": True}
    else:
        result["center"] = {
            "x": _clamp(center_source.get("x", result["center"]["x"]), 0, IMAGE_WIDTH - 1) if custom else IMAGE_WIDTH // 2,
            "y": _clamp(center_source.get("y", result["center"]["y"]), 0, IMAGE_HEIGHT - 1) if custom else IMAGE_HEIGHT // 2,
            "custom": custom,
        }

    exclusions = source.get("exclude_rois", DEFAULT_CONFIG["exclude_rois"])
    result["exclude_rois"] = []
    for rect in exclusions if isinstance(exclusions, (list, tuple)) else []:
        if not isinstance(rect, dict):
            continue
        left = _clamp(rect.get("x", 0), -IMAGE_WIDTH, IMAGE_WIDTH)
        top = _clamp(rect.get("y", 0), -IMAGE_HEIGHT, IMAGE_HEIGHT)
        right = min(IMAGE_WIDTH, left + _clamp(rect.get("w", 0), 0, IMAGE_WIDTH))
        bottom = min(IMAGE_HEIGHT, top + _clamp(rect.get("h", 0), 0, IMAGE_HEIGHT))
        left, top = max(0, left), max(0, top)
        if right > left and bottom > top:
            result["exclude_rois"].append(dict(x=left, y=top, w=right-left, h=bottom-top))

    polygons = source.get("exclude_polygons", DEFAULT_CONFIG["exclude_polygons"])
    if source.get("schema_version", 0) != SCHEMA_VERSION and polygons == [_LEGACY_JOINED_POLYGON]:
        polygons = DEFAULT_CONFIG["exclude_polygons"]
    result["exclude_polygons"] = []
    for polygon in polygons if isinstance(polygons, (list, tuple)) else []:
        if not isinstance(polygon, (list, tuple)) or len(polygon) < 3:
            continue
        points = []
        try:
            for point in polygon:
                if not isinstance(point, (list, tuple)) or len(point) != 2:
                    raise ValueError("invalid polygon point")
                # 保留画面外顶点，光栅化时裁剪；逐点夹到边界会改变原多边形形状。
                px, py = int(point[0]), int(point[1])
                points.append([max(-IMAGE_WIDTH, min(2 * IMAGE_WIDTH, px)),
                               max(-IMAGE_HEIGHT, min(2 * IMAGE_HEIGHT, py))])
        except (TypeError, ValueError, OverflowError):
            continue
        area2 = sum(points[i][0] * points[(i + 1) % len(points)][1] -
                    points[(i + 1) % len(points)][0] * points[i][1]
                    for i in range(len(points)))
        if area2 and len(set(map(tuple, points))) >= 3:
            result["exclude_polygons"].append(points)

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

    # 旧JSON仅补充新字段，绝不按“看起来像旧默认值”覆盖现场标定。
    grasp_source = source.get("grasp", {})
    if not isinstance(grasp_source, dict):
        grasp_source = {}
    grasp_roi = grasp_source.get("roi", {})
    if not isinstance(grasp_roi, dict):
        grasp_roi = {}
    defaults = DEFAULT_CONFIG["grasp"]
    gw = _clamp(grasp_roi.get("w", defaults["roi"]["w"]), 1, IMAGE_WIDTH)
    gh = _clamp(grasp_roi.get("h", defaults["roi"]["h"]), 1, IMAGE_HEIGHT)
    # 旧配置缺少抓取框时围绕其实际对准点初始化，已有抓取框则独立保存。
    gx = _clamp(grasp_roi.get("x", result["center"]["x"] - gw // 2), 0, IMAGE_WIDTH - gw)
    gy = _clamp(grasp_roi.get("y", result["center"]["y"] - gh // 2), 0, IMAGE_HEIGHT - gh)
    result["grasp"] = {
        "roi": {"x": gx, "y": gy, "w": gw, "h": gh},
        "min_coverage_pct": _clamp(
            grasp_source.get("min_coverage_pct", defaults["min_coverage_pct"]), 0, 100),
    }

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
