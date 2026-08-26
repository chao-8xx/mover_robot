# 当前阶段正式入口：三任务首页、任务调度、B2物料识别和F407任务控制。
# B3圆环数字和B4码垛只接入安全占位页，本轮不运行旧原型算法。
# 协议: AA 55 CMD LEN DATA SUM
# 返回坐标包 DATA: FRAME_ID RAW_OK dxL dxH dyL dyH

from maix import camera, display, app, time, image, touchscreen, uart
import cv2
import numpy as np
import math

from vision_config import (
    COLOR_NUMBERS,
    IMAGE_HEIGHT,
    IMAGE_WIDTH,
    RUN_PROCESS_HEIGHT,
    RUN_PROCESS_WIDTH,
    VisionConfigStore,
    clone_config,
)
from vision_blob import (
    build_mask,
    expand_roi_mask,
    find_largest_blob,
    make_preview,
    make_run_geometry,
    map_run_result_to_source,
    resize_run_roi,
)
from vision_ui import TouchInput, VisionUI
from vision_tasks import (
    TASK_LABELS,
    TASK_MATERIAL,
    TASK_NONE,
    TASK_RING,
    TASK_STACK,
    TaskManager,
)
from vision_ring import make_ring_preview

# 通信协议
HEAD1 = 0XAA
HEAD2 = 0X55

CMD_NONE = 0XB0
# CMD_QR   = 0XB1
CMD_BLOB = 0XB2         # 物料 色块
CMD_RING = 0XB3         # 圆环 数字
CMD_ONUP = 0XB4         # 堆放物料顶部

# 颜色指令码
COLOR_NONE       = 0xC0
COLOR_RED        = 0xC1
COLOR_YELLOW     = 0xC2
COLOR_BLUE       = 0xC3
COLOR_GREEN      = 0xC4
COLOR_BLACK      = 0xC5
COLOR_LIGHTBLUE  = 0xC6

# 圆环数字指令码
SUM_NONE = 0xD0
SUM_1    = 0xD1
SUM_2    = 0xD2
SUM_3    = 0xD3

# 全局变量
image_width = IMAGE_WIDTH
image_height = IMAGE_HEIGHT
center_x = image_width // 2
center_y = image_height // 2

# 识别参数
BLOB_WAY = "LAB"        # "HSV" / "LAB"

MIN_BLOB_AREA = 300     # 识别物料面积阈值
MAX_BLOB_AREA = 60000

BLACK_V_MAX = 85        # HSV中黑色的V通道最大值 (亮度)
MIN_RING_AREA = 500
MAX_RING_AREA = 50000   # 圆环面积阈值
MIN_RATIO = 0.45
MAX_RATIO = 2.20        # 圆环宽高比阈值
MIN_CIRCULARITY = 0.18
MIN_HOLE_AREA = 80

OCR_W = 32
OCR_H = 48

USE_SEARCH_ROI = False
SEARCH_X1 = 0
SEARCH_Y1 = 0
SEARCH_X2 = image_width
SEARCH_Y2 = image_height

SEND_TIME_MS = 50   # 与已完成的 B2 测试一致：平均 20Hz 发送最新单帧结果
TX_DEBUG_TIME_MS = 1000    # 调试打印发送数据包间隔，避免串口打印过多影响识别速度 在maix端打印
DISPLAY_TIME_MS = 100      # 调试画面最多 10Hz，不拖慢识别主循环
RING_PERF_TIME_MS = 1000   # 任务二每秒输出一次采集/检测/显示耗时
MAX_RX_DATA_LEN = 32

# 初始化
cam = camera.Camera(image_width, image_height)
disp = display.Display()
ts = touchscreen.TouchScreen()

try:
    screen_w = int(disp.width())
    screen_h = int(disp.height())
except Exception:
    screen_w = 480
    screen_h = 640

try:
    data_dir = app.get_app_data_path()
except Exception:
    data_dir = "./data"

config_store = VisionConfigStore(data_dir)
vision_config, config_status = config_store.load()
ui = VisionUI(screen_w, screen_h, vision_config, config_status)
touch_input = TouchInput(ts)
task_manager = TaskManager()

device = "/dev/serial0"
try:
    serial = uart.UART(device, 115200)
    serial_status = "ready"
except Exception as error:
    serial = None
    serial_status = "disabled: {}".format(error)

# pinmap.set_pin_function("A18", "UART1_RX")
# pinmap.set_pin_function("A19", "UART1_TX")    # serial1 配置方法
# device = "/dev/ttyS1"
# serial = uart.UART(device, 115200)

mode = CMD_NONE
target = COLOR_NONE
rx_buf = bytearray()
# 发送截止时间必须从当前时刻起算，不能从0开始追赶系统运行时间。
next_send_time = time.ticks_ms()
last_tx_debug_time = 0
last_display_time = 0
vision_frame_id = 0
uart_control_active = False
blob_runtime_target = None
blob_runtime_config = None
blob_run_geometry = None

print("[VISION CONFIG]", config_status)
print("[VISION UART] {} @ 115200: {}".format(device, serial_status))
print("[VISION UI] display: {}x{}".format(screen_w, screen_h))
print("[VISION UI] touchscreen available:", hasattr(ts, "available"))
print("[VISION BUILD] B3_DILATE7_RECT_PROFILE_B2_RUN240_V4")


###################################### 通信函数 ############################
# 数据包校验和
def checksum(cmd, length, data):
    total = cmd + length
    for byte in data:
        total += byte
    return total & 0xFF

# 构建并发送数据包：AA 55 CMD LEN DATA SUM。
def send_frame(cmd, data):
    length = len(data)       # 数据长度

    frame = bytearray([HEAD1, HEAD2, cmd, length])  # 帧头 + 命令字 + 长度
    frame.extend(data)  # 添加数据
    frame.append(checksum(cmd, length, data))  # 添加校验和

    if serial is not None:
        serial.write(bytes(frame))
    return bytes(frame)

# 发送当前单帧原始坐标。DATA 与 F407 camera.c 保持一致：
# FRAME_ID RAW_OK dxL dxH dyL dyH
def send_xy_result(cmd, frame_id, raw_ok, dx, dy):
    global last_tx_debug_time

    data = bytearray()
    data.append(int(frame_id) & 0xFF)
    data.append(1 if raw_ok else 0)
    append_i16_le(data, dx)
    append_i16_le(data, dy)
    frame = send_frame(cmd, data)
    now = time.ticks_ms()

    # 调试 打印在maix端的串口输出
    if (now - last_tx_debug_time) >= TX_DEBUG_TIME_MS:
        print("Maix TX:", " ".join("%02X" % b for b in frame))
        last_tx_debug_time = now
    return frame


def append_i16_le(data, value):
    value = max(-32768, min(32767, int(value)))
    if value < 0:
        value += 0x10000
    data.append(value & 0xFF)
    data.append((value >> 8) & 0xFF)

# 处理串口接收数据
def rx_handle():
    global rx_buf, uart_control_active

    # 这一赋值是“正式模式”的开关：只要主循环调用本函数，就必须等待 F407。
    # 调试 B2 时只需注释主循环中的 rx_handle()，无需改 mode/target。
    uart_control_active = True

    if serial is None:
        return

    # # 非阻塞读取串口数据
    # buf = serial.read(-1, 2)
    # if not buf:
    #     return

    # 非阻塞读取串口数据。
    buf = serial.read()
    if not buf:
        return

    rx_buf.extend(buf)

    while len(rx_buf) >= 5:
        # 找帧头
        if rx_buf[0] != HEAD1 or rx_buf[1] != HEAD2:
            rx_buf.pop(0)
            continue

        # 命令字和帧长
        cmd = rx_buf[2]
        length = rx_buf[3]
        if length > MAX_RX_DATA_LEN:
            del rx_buf[0]
            continue
        frame_len = 2 + 1 + 1 + length + 1

        if len(rx_buf) < frame_len:
            return

        # 数据和校验位
        data = rx_buf[4:4 + length]
        rx_checksum = rx_buf[4 + length]

        if rx_checksum == checksum(cmd, length, data):
            handle_cmd(cmd, data)
            del rx_buf[:frame_len]
        else:
            # 校验失败只丢一个字节，下一轮重新搜索 AA 55，避免吞掉后续好帧。
            del rx_buf[0]

# 命令处理
def handle_cmd(cmd, data):
    global mode, target, vision_frame_id, next_send_time
    global blob_runtime_target, blob_runtime_config, blob_run_geometry

    next_mode = mode
    next_target = target

    if cmd == CMD_NONE and len(data) == 0:
        next_mode = CMD_NONE
        next_target = COLOR_NONE

    elif cmd == CMD_BLOB and len(data) == 1:
        color = clear_color_target(data[0])
        if color != COLOR_NONE:
            next_mode = CMD_BLOB
            next_target = color

    elif cmd == CMD_RING and len(data) == 1:
        ring = clear_ring_target(data[0])
        if ring != SUM_NONE:
            next_mode = CMD_RING
            next_target = ring

    elif cmd == CMD_ONUP and len(data) == 1:
        color = clear_color_target(data[0])
        if color != COLOR_NONE:
            next_mode = CMD_ONUP
            next_target = color
    else:
        return

    if next_mode == mode and next_target == target:
        return

    mode = next_mode
    target = next_target
    vision_frame_id = 0
    # 新任务可以立即发送首个识别结果，之后按50ms累计截止时间调度。
    next_send_time = time.ticks_ms()
    blob_runtime_target = None
    blob_runtime_config = None
    blob_run_geometry = None
    print("[VISION CMD] mode=0x%02X target=0x%02X" % (mode, target))


def active_task():
    """正式模式返回F407状态；本机模式由三任务首页决定当前任务。"""
    if uart_control_active:
        return mode, target
    local_task = ui.current_task()
    if local_task == TASK_MATERIAL:
        color_number = COLOR_NUMBERS.get(ui.detection_config()["target_color"], 1)
        return CMD_BLOB, clear_color_target(color_number)
    if local_task == TASK_RING:
        ring_number = ui.detection_config()["ring"]["target_number"]
        return CMD_RING, clear_ring_target(ring_number)
    if local_task == TASK_STACK:
        return CMD_ONUP, COLOR_NONE
    return CMD_NONE, COLOR_NONE


def mode_to_task_id(active_mode):
    if active_mode == CMD_BLOB:
        return TASK_MATERIAL
    if active_mode == CMD_RING:
        return TASK_RING
    if active_mode == CMD_ONUP:
        return TASK_STACK
    return TASK_NONE


################################## 辅助函数 ##################################
# OpenCV版本兼容函数，适配不同版本的cv2.findContours返回值
def cv_normalize_contour(contour):
    """Convert contour variants to contiguous N x 1 x 2 int32 arrays."""
    try:
        value = contour
        if isinstance(value, (tuple, list)) and len(value) == 1:
            value = value[0]
        points = np.asarray(value, dtype=np.int32)
    except (TypeError, ValueError):
        return None

    if points.size < 6 or points.size % 2 != 0:
        return None

    points = points.reshape((-1, 1, 2))
    return np.ascontiguousarray(points, dtype=np.int32)


# 轮廓匹配 最外层匹配
def cv_find_contours_external(mask):
    result = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if len(result) == 2:
        contours, hierarchy = result
    else:
        _, contours, hierarchy = result
    normalized = []
    for contour in contours:
        contour = cv_normalize_contour(contour)
        if contour is not None:
            normalized.append(contour)
    return normalized

# 轮廓匹配 树形结构
def cv_find_contours_tree(mask):
    result = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if len(result) == 2:
        contours, hierarchy = result
    else:
        _, contours, hierarchy = result
    contours = [cv_normalize_contour(contour) for contour in contours]
    return contours, hierarchy

# 兼容下位机发 1~6 或直接发 0xC1~0xC6
def clear_color_target(v):
    table = {
        1: COLOR_RED,
        2: COLOR_YELLOW,
        3: COLOR_BLUE,
        4: COLOR_GREEN,
        5: COLOR_BLACK,
        6: COLOR_LIGHTBLUE,
        COLOR_RED: COLOR_RED,
        COLOR_YELLOW: COLOR_YELLOW,
        COLOR_BLUE: COLOR_BLUE,
        COLOR_GREEN: COLOR_GREEN,
        COLOR_BLACK: COLOR_BLACK,
        COLOR_LIGHTBLUE: COLOR_LIGHTBLUE,
    }
    return table.get(v, COLOR_NONE)

# 兼容下位机发 1~3 或直接发 0xD1~0xD3
def clear_ring_target(v):
    table = {
        1: SUM_1,
        2: SUM_2,
        3: SUM_3,
        SUM_1: SUM_1,
        SUM_2: SUM_2,
        SUM_3: SUM_3,
    }
    return table.get(v, SUM_NONE)

# 兼容下位机发 1~3 或直接发 0xD1~0xD3
def target_to_num(target):
    if target == SUM_1 or target == 1:
        return 1
    elif target == SUM_2 or target == 2:
        return 2
    elif target == SUM_3 or target == 3:
        return 3
    else:
        return 0

# 显示颜色字符串  为了显示调试
def color_name(color):
    if color == COLOR_RED:
        return "RED"
    elif color == COLOR_YELLOW:
        return "YELLOW"
    elif color == COLOR_BLUE:
        return "BLUE"
    elif color == COLOR_GREEN:
        return "REGREEND"
    elif color == COLOR_BLACK:
        return "BLACK"
    elif color == COLOR_LIGHTBLUE:
        return "LIGHTBLUE"

# 在图像中心画一个小十字
def draw_center_cross(cv_img):
    cv2.line(cv_img, (center_x - 8, center_y), (center_x + 8, center_y), (255, 255, 255), 1)
    cv2.line(cv_img, (center_x, center_y - 8), (center_x, center_y + 8), (255, 255, 255), 1)



##################################### 物料识别 ################################
# 物料识别的调试信息
def draw_blob_debug(cv_img, ok, target_color, cx, cy, dx, dy, area, rect, way):
    cv2.drawMarker(cv_img, (center_x, center_y), (255, 255, 255),
                   markerType=cv2.MARKER_CROSS, thickness=1)    # 白色十字中心

    # 左上角显示当前模式WAY和目标颜色
    text1 = "WAY:{} COLOR:{}".format(way, color_name(target_color))
    cv2.putText(cv_img, text1, (5, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)

    if ok:
        x, y, w, h = rect
        cv2.rectangle(cv_img, (x, y), (x + w, y + h), (0, 255, 0), 2)   # 物料最大外接框 绿色
        cv2.circle(cv_img, (cx, cy), 5, (0, 0, 255), -1)   # 物料中心点 红色
        cv2.line(cv_img, (center_x, center_y), (cx, cy), (255, 0, 0), 1)  # 屏幕中心到物料中心连线 蓝色

        # 左上角显示坐标和面积信息
        text2 = "cx:{} cy:{} dx:{} dy:{}".format(cx, cy, dx, dy)
        text3 = "area:{}".format(int(area))
        cv2.putText(cv_img, text2, (5, 38), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
        cv2.putText(cv_img, text3, (5, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
    else:
        cv2.putText(cv_img, "NO TARGET", (5, 42), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)

# 从mask中找最大色块 返回:ok, dx, dy, debug_img
# 此函数为HSV和LAB两种方法的公共部分，输入mask，输出坐标和调试图像
def find_blob_from_mask(cv_img, mask, target_color, way):
    # 形态学处理  开闭运算
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

    contours = cv_find_contours_external(mask)

    max_area = 0
    best_contour = None
    # 遍历轮廓 找最大轮廓
    for c in contours:
        area = cv2.contourArea(c)
        if area < MIN_BLOB_AREA or area > MAX_BLOB_AREA:
            continue
        if area > max_area:
            max_area = area
            best_contour = c

    if best_contour is None:
        draw_blob_debug(cv_img, 0, target_color, 0, 0, 0, 0, 0, (0, 0, 0, 0), way)
        return 0, 0, 0, image.cv2image(cv_img)

    # 计算最大轮廓空间矩  (中心点坐标cx, cy，距中心点偏移dx, dy，计算外接矩形)
    M = cv2.moments(best_contour)

    if M["m00"] == 0:
        draw_blob_debug(cv_img, 0, target_color, 0, 0, 0, 0, 0, (0, 0, 0, 0), way)
        return 0, 0, 0, image.cv2image(cv_img)

    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])
    dx = cx - center_x
    dy = cy - center_y
    rect = cv2.boundingRect(best_contour)

    # 画物料实际轮廓边界 绿色
    cv2.drawContours(cv_img, [best_contour], -1, (0, 255, 0), 2)
    draw_blob_debug(cv_img, 1, target_color, cx, cy, dx, dy, max_area, rect, way)

    return 1, dx, dy, image.cv2image(cv_img)

# HSV方法的物料识别
def find_blobs_hsv(img, target_color):
    cv_img = image.image2cv(img, ensure_bgr=True, copy=True)
    hsv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)

    # HSV颜色范围表 待调
    hsv_ranges = {
        COLOR_RED: [
            (np.array([0, 80, 50]), np.array([10, 255, 255])),
            (np.array([160, 80, 50]), np.array([180, 255, 255])),
        ],
        COLOR_YELLOW: [(np.array([18, 80, 60]), np.array([35, 255, 255]))],
        COLOR_BLUE: [(np.array([100, 70, 40]), np.array([130, 255, 255]))],
        COLOR_GREEN: [(np.array([35, 50, 40]), np.array([85, 255, 255]))],
        COLOR_BLACK: [(np.array([0, 0, 0]), np.array([180, 255, 60]))],
        COLOR_LIGHTBLUE: [(np.array([85, 40, 60]), np.array([105, 255, 255]))],
    }

    # 目标颜色检测
    if target_color not in hsv_ranges:
        draw_blob_debug(cv_img, 0, target_color, 0, 0, 0, 0, 0, (0, 0, 0, 0), "HSV")
        return 0, 0, 0, image.cv2image(cv_img)
    mask = None

    # 多段颜色范围时，合并mask  (红色)
    for lower, upper in hsv_ranges[target_color]:
        temp_mask = cv2.inRange(hsv_img, lower, upper)

        if mask is None:
            mask = temp_mask
        else:
            mask = cv2.bitwise_or(mask, temp_mask)

    return find_blob_from_mask(cv_img, mask, target_color, "HSV")

# LAB方法的物料识别
def find_blobs_lab(img, target_color):
    cv_img = image.image2cv(img, ensure_bgr=True, copy=True)
    lab_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2LAB)

    # LAB颜色范围表 待调
    lab_ranges = {
        COLOR_RED: [(np.array([20, 145, 130]), np.array([255, 210, 210]))],
        COLOR_YELLOW: [(np.array([80, 115, 145]), np.array([255, 160, 230]))],
        COLOR_BLUE: [(np.array([20, 110, 60]), np.array([220, 170, 130]))],
        COLOR_GREEN: [(np.array([20, 60, 80]), np.array([220, 125, 170]))],
        COLOR_BLACK: [(np.array([0, 0, 0]), np.array([70, 255, 255]))],
        COLOR_LIGHTBLUE: [(np.array([80, 90, 70]), np.array([255, 150, 135]))],
    }

    # 目标颜色检测
    if target_color not in lab_ranges:
        draw_blob_debug(cv_img, 0, target_color, 0, 0, 0, 0, 0, (0, 0, 0, 0), "LAB")
        return 0, 0, 0, image.cv2image(cv_img)
    mask = None

    # 多段颜色范围时，合并mask
    for lower, upper in lab_ranges[target_color]:
        lower = lower.astype(np.uint8)
        upper = upper.astype(np.uint8)

        temp_mask = cv2.inRange(lab_img, lower, upper)

        if mask is None:
            mask = temp_mask
        else:
            mask = cv2.bitwise_or(mask, temp_mask)

    return find_blob_from_mask(cv_img, mask, target_color, "LAB")


####################################### 圆环内数字OCR检测 ################################
# 生成一个数字模版  白底黑字二值图
def make_one_template(num, scale, thickness, dx, dy):
    canvas = np.ones((OCR_H, OCR_W), dtype=np.uint8) * 255
    font = cv2.FONT_HERSHEY_SIMPLEX
    text = str(num)

    try:
        text_size, baseline = cv2.getTextSize(text, font, scale, thickness)
        tw, th = text_size
    except Exception:
        tw = int(16 * scale)
        th = int(32 * scale)

    x = int((OCR_W - tw) / 2) + dx
    y = int((OCR_H + th) / 2) + dy
    cv2.putText(canvas, text, (x, y), font, scale, 0, thickness)
    _, binary = cv2.threshold(canvas, 128, 255, cv2.THRESH_BINARY_INV)
    return binary


def build_digit_templates():
    templates = {1: [], 2: [], 3: []}
    scales = [1.10, 1.20, 1.30, 1.40]
    thicknesses = [2, 3]
    offsets = [-2, 0, 2]

    for num in [1, 2, 3]:
        for scale in scales:
            for thick in thicknesses:
                for dx in offsets:
                    for dy in offsets:
                        templates[num].append(make_one_template(num, scale, thick, dx, dy))
    return templates


DIGIT_TEMPLATES = None


def normalize_digit_image(binary):
    contours = cv_find_contours_external(binary)
    if not contours:
        return None

    best = None
    best_area = 0
    for c in contours:
        area = cv2.contourArea(c)
        if area > best_area:
            best_area = area
            best = c

    if best is None or best_area < 10:
        return None

    x, y, w, h = cv2.boundingRect(best)
    if w < 2 or h < 5:
        return None

    digit = binary[y:y + h, x:x + w]
    canvas = np.zeros((OCR_H, OCR_W), dtype=np.uint8)

    scale = min((OCR_W - 6) / float(w), (OCR_H - 6) / float(h))
    new_w = int(w * scale)
    new_h = int(h * scale)

    if new_w <= 0 or new_h <= 0:
        return None

    digit_resized = cv2.resize(digit, (new_w, new_h))
    px = (OCR_W - new_w) // 2
    py = (OCR_H - new_h) // 2
    canvas[py:py + new_h, px:px + new_w] = digit_resized

    return canvas


def recognize_digit_bruteforce(roi_bgr):
    global DIGIT_TEMPLATES

    if roi_bgr is None or roi_bgr.size == 0:
        return 0, 999999, None

    # B2 启动时不预生成数百张 OCR 模板；第一次真正进入 B3 时再创建。
    if DIGIT_TEMPLATES is None:
        DIGIT_TEMPLATES = build_digit_templates()

    gray = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (3, 3), 0)
    _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (2, 2))
    binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel, iterations=1)

    norm = normalize_digit_image(binary)
    if norm is None:
        return 0, 999999, binary

    best_num = 0
    best_score = 999999

    for num in [1, 2, 3]:
        for tmpl in DIGIT_TEMPLATES[num]:
            diff = cv2.absdiff(norm, tmpl)
            score = cv2.countNonZero(diff)
            if score < best_score:
                best_score = score
                best_num = num

    return best_num, best_score, norm

####################################### 圆环检测 ################################
def scan_ring(img):
    # 灰度化 -> 二值化 -> 找黑色圆环 -> 识别数字 -> 返回圆心偏差
    cv_img = image.image2cv(img, ensure_bgr=True, copy=True)

    # 如果使用搜索ROI，就在指定区域内找圆环；否则在全图找
    if USE_SEARCH_ROI:
        ring_roi = cv_img[SEARCH_Y1:SEARCH_Y2, SEARCH_X1:SEARCH_X2]
        offset_x = SEARCH_X1
        offset_y = SEARCH_Y1
    else:
        ring_roi = cv_img
        offset_x = 0
        offset_y = 0

    hsv = cv2.cvtColor(ring_roi, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array([0, 0, 0]), np.array([180, 255, BLACK_V_MAX]))

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)

    contours, hierarchy = cv_find_contours_tree(mask)
    candidates = []

    if hierarchy is None or len(contours) == 0:
        return candidates, cv_img

    hierarchy = hierarchy[0]

    for i, c in enumerate(contours):
        if c is None:
            continue
        area = cv2.contourArea(c)
        if area < MIN_RING_AREA or area > MAX_RING_AREA:
            continue

        x, y, w, h = cv2.boundingRect(c)
        if h == 0:
            continue

        ratio = w / float(h)
        if ratio < MIN_RATIO or ratio > MAX_RATIO:
            continue

        peri = cv2.arcLength(c, True)
        if peri <= 1:
            continue

        circularity = 4.0 * math.pi * area / (peri * peri)
        if circularity < MIN_CIRCULARITY:
            continue

        child_idx = hierarchy[i][2]
        if child_idx < 0:
            continue

        child_contour = contours[child_idx]
        if child_contour is None:
            continue
        child_area = cv2.contourArea(child_contour)
        if child_area < MIN_HOLE_AREA:
            continue

        x += offset_x
        y += offset_y
        cx = x + w // 2
        cy = y + h // 2
        dx = cx - center_x
        dy = cy - center_y

        candidates.append({
            "x": x,
            "y": y,
            "w": w,
            "h": h,
            "cx": cx,
            "cy": cy,
            "dx": dx,
            "dy": dy,
            "area": area,
            "circularity": circularity,
            "rect": (x, y, w, h),
        })

    return candidates, cv_img


def crop_digit_roi(cv_img, ring):
    x = ring["x"]
    y = ring["y"]
    w = ring["w"]
    h = ring["h"]

    roi_x = x + int(w * 0.21)
    roi_y = y + int(h * 0.21)
    roi_w = int(w * 0.58)
    roi_h = int(h * 0.58)

    roi_x = max(0, roi_x)
    roi_y = max(0, roi_y)
    roi_w = min(image_width - roi_x, roi_w)
    roi_h = min(image_height - roi_y, roi_h)

    if roi_w <= 0 or roi_h <= 0:
        return None, (roi_x, roi_y, roi_w, roi_h)

    roi = cv_img[roi_y:roi_y + roi_h, roi_x:roi_x + roi_w]
    return roi, (roi_x, roi_y, roi_w, roi_h)


def find_ring(img, target_code):
    target_code = clear_ring_target(target_code)
    target_num = target_to_num(target_code)
    candidates, debug_img = scan_ring(img)
    draw_center_cross(debug_img)
    best_target = None

    for ring in candidates:
        roi, roi_rect = crop_digit_roi(debug_img, ring)
        detected_num, score, norm_img = recognize_digit_bruteforce(roi)

        ring["num"] = detected_num
        ring["score"] = score
        ring["roi_rect"] = roi_rect

        x = ring["x"]
        y = ring["y"]
        w = ring["w"]
        h = ring["h"]
        cx = ring["cx"]
        cy = ring["cy"]

        cv2.rectangle(debug_img, (x, y), (x + w, y + h), (255, 0, 0), 1)
        rx, ry, rw, rh = roi_rect
        cv2.rectangle(debug_img, (rx, ry), (rx + rw, ry + rh), (0, 255, 255), 1)
        cv2.circle(debug_img, (cx, cy), 4, (0, 0, 255), -1)

        text = "N:{} S:{}".format(detected_num, score)
        cv2.putText(debug_img, text, (x, max(15, y - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)

        if target_num != 0 and detected_num == target_num:
            if best_target is None or score < best_target["score"]:
                best_target = ring

    if best_target is not None:
        x = best_target["x"]
        y = best_target["y"]
        w = best_target["w"]
        h = best_target["h"]
        cx = best_target["cx"]
        cy = best_target["cy"]
        dx = best_target["dx"]
        dy = best_target["dy"]
        area = best_target["area"]
        num = best_target["num"]
        score = best_target["score"]

        cv2.rectangle(debug_img, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.circle(debug_img, (cx, cy), 5, (0, 0, 255), -1)
        cv2.line(debug_img, (center_x, center_y), (cx, cy), (255, 255, 255), 1)

        cv2.putText(debug_img, "TARGET:{} FOUND:{}".format(target_num, num), (5, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        cv2.putText(debug_img, "dx:{} dy:{} area:{}".format(dx, dy, int(area)), (5, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        cv2.putText(debug_img, "score:{}".format(score), (5, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        return 1, dx, dy, image.cv2image(debug_img), num

    cv2.putText(debug_img, "TARGET:{} NOT FOUND".format(target_num), (5, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)
    cv2.putText(debug_img, "candidates:{}".format(len(candidates)), (5, 45),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

    return 0, 0, 0, image.cv2image(debug_img), 0



######################################## 视觉主处理 ################################
COLOR_CODE_TO_KEY = {
    COLOR_RED: "red",
    COLOR_YELLOW: "yellow",
    COLOR_BLUE: "blue",
    COLOR_GREEN: "green",
    COLOR_BLACK: "black",
    COLOR_LIGHTBLUE: "light_blue",
}


def prepare_blob_runtime(target_color):
    """只在 B2/B4 目标变化时重建 240x180 RUN 几何参数。"""
    global blob_runtime_target, blob_runtime_config, blob_run_geometry

    normalized_target = clear_color_target(target_color)
    color_key = COLOR_CODE_TO_KEY.get(normalized_target)
    if color_key is None:
        return False
    if blob_runtime_target == normalized_target:
        return True

    blob_runtime_config = clone_config(vision_config)
    blob_runtime_config["target_color"] = color_key
    blob_run_geometry = make_run_geometry(
        blob_runtime_config,
        IMAGE_WIDTH,
        IMAGE_HEIGHT,
        RUN_PROCESS_WIDTH,
        RUN_PROCESS_HEIGHT,
    )
    blob_runtime_target = normalized_target
    print("[VISION BLOB] target={} process={}x{}".format(
        color_key, RUN_PROCESS_WIDTH, RUN_PROCESS_HEIGHT
    ))
    return True


def find_blobs(img, target_color, need_debug=False):
    """使用已完成的 B2 路径：RUN 240x180，结果恢复为 320x240。"""
    if not prepare_blob_runtime(target_color):
        return 0, 0, 0, img

    cv_bgr = image.image2cv(img, ensure_bgr=True, copy=True)
    process_bgr = resize_run_roi(cv_bgr, blob_run_geometry)
    process_config = blob_run_geometry["config"]
    mask = build_mask(process_bgr, process_config)
    process_result = find_largest_blob(
        mask, process_config, blob_run_geometry["process_roi"]
    )
    result = map_run_result_to_source(
        process_result, blob_runtime_config, blob_run_geometry
    )

    debug_img = img
    if need_debug:
        # 调试预览只画 ROI、抓取中心和目标框，不生成整图 mask。
        preview = make_preview(
            cv_bgr, None, result, blob_runtime_config, view_mode="RAW"
        )
        cv2.putText(
            preview,
            "B2 {} OK:{} DX:{} DY:{}".format(
                blob_runtime_config["target_color"],
                1 if result["found"] else 0,
                result["dx"],
                result["dy"],
            ),
            (5, 18),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.43,
            (255, 255, 255),
            1,
        )
        debug_img = image.cv2image(preview)

    return (
        1 if result["found"] else 0,
        result["dx"],
        result["dy"],
        debug_img,
    )


# def find_ring(img, pos):
#     """
#     B3: 找指定编号黑色圆环
#     输入 pos = 1/2/3
#     返回 ok, dx, dy
#
#     注意：
#     今年圆环不是红绿蓝色环，而是黑色圆环。
#     pos 表示要找 1号/2号/3号圆环。
#     后面这里写：
#     灰度化 -> 二值化 -> 找黑色圆环 -> 识别数字 -> 筛选目标pos -> 返回圆心偏差
#     """
#     if pos == 1:
#         return 1, 10, -5
#     elif pos == 2:
#         return 1, 20, -10
#     elif pos == 3:
#         return 1, -15, 8
#     else:
#         return 0, 0, 0


def find_stack(img, target_color, need_debug=False):
    # 暂时和物料识别用同一个函数；后面如果要专门找顶部，再单独替换这里
    return find_blobs(img, target_color, need_debug)


# # 数据打包
# def pack_xy_result(color, ok, dx, dy):
#     data = bytearray()
#     data.append(color)
#     data.append(ok)
#     data.extend(pack("<h", int(dx)))
#     data.extend(pack("<h", int(dy)))
#     return data


# def pack_ring_result(color, ok, dx, dy, num):
#     data = bytearray()
#     data.append(color)
#     data.append(ok)
#     data.extend(pack("<h", int(dx)))
#     data.extend(pack("<h", int(dy)))
#     data.append(num)
#     return data


############################### B2 UI + F407 主循环 ###########################
# HOME/调参页面低频刷新；RUN 页面只显示一次静态页面，识别保持高速。
vision_fps_value = 0.0
display_fps_value = 0.0
vision_fps_frames = 0
display_fps_frames = 0
fps_tick = time.ticks_ms()
last_display_time = fps_tick - DISPLAY_TIME_MS
force_display = True
run_screen_shown = False
previous_run_mode = False
ring_perf_tick = fps_tick
ring_perf_capture_ms = 0
ring_perf_detect_ms = 0
ring_perf_display_ms = 0
ring_perf_mask_ms = 0
ring_perf_group_ms = 0
ring_perf_contour_ms = 0
ring_perf_filter_ms = 0
ring_perf_frames = 0
ring_perf_displays = 0


while not app.need_exit():
    loop_tick = time.ticks_ms()

    # 正式运行保留下一行：F407的B0/B2/B3/B4选择当前视觉任务。
    # 本地调试只注释下一行：由三任务首页和任务内按钮控制。
    # rx_handle()

    active_mode, active_target = active_task()
    active_task_id = mode_to_task_id(active_mode)
    if uart_control_active:
        if active_task_id == TASK_MATERIAL:
            runtime_color = COLOR_CODE_TO_KEY.get(clear_color_target(active_target))
            if runtime_color is not None:
                ui.set_runtime_target(runtime_color)
        elif active_task_id == TASK_RING:
            ring_number = target_to_num(active_target)
            if ring_number:
                ui.set_runtime_ring_target(ring_number)
        ui.set_external_task(active_task_id)
        run_mode = active_task_id in (TASK_MATERIAL, TASK_RING)
    else:
        # 注释 rx_handle() 后，由三任务首页选择任务；任务一和任务二都有高速RUN。
        run_mode = ui.is_run_mode()

    task_changed = task_manager.switch(active_task_id, active_target)
    if task_changed or run_mode != previous_run_mode:
        previous_run_mode = run_mode
        run_screen_shown = False
        force_display = not run_mode
        vision_frame_id = 0
        next_send_time = loop_tick
        vision_fps_value = 0.0
        display_fps_value = 0.0
        vision_fps_frames = 0
        display_fps_frames = 0
        fps_tick = loop_tick
        ring_perf_tick = loop_tick
        ring_perf_capture_ms = 0
        ring_perf_detect_ms = 0
        ring_perf_display_ms = 0
        ring_perf_mask_ms = 0
        ring_perf_group_ms = 0
        ring_perf_contour_ms = 0
        ring_perf_filter_ms = 0
        ring_perf_frames = 0
        ring_perf_displays = 0
        print("[VISION TASK]", TASK_LABELS.get(active_task_id, "未知任务"),
              "target=0x%02X" % int(active_target),
              "RUN" if run_mode else "UI/PLACEHOLDER")

    display_due = (
        not run_mode and
        (force_display or loop_tick - last_display_time >= DISPLAY_TIME_MS)
    )

    # UI 页面需要实时画面；正式 B2/B3 RUN 使用相同相机，只关闭周期预览刷新。
    capture_tick = time.ticks_ms()
    camera_image = cam.read()
    cv_bgr = image.image2cv(camera_image, ensure_bgr=True, copy=True)
    capture_ms = time.ticks_ms() - capture_tick
    active_config = ui.detection_config()
    detect_tick = time.ticks_ms()
    task_frame = task_manager.process(cv_bgr, active_config, run_mode)
    detect_ms = time.ticks_ms() - detect_tick
    blob_result = task_frame["result"]
    mask_roi = task_frame["mask_roi"]
    roi_rect = task_frame["roi_rect"]

    # 当前只有已经验收的B2结果允许发送；B3候选阶段不能冒充最终圆心结果。
    if run_mode and active_task_id == TASK_MATERIAL:
        vision_frame_id = (vision_frame_id + 1) & 0xFF
        result_tick = time.ticks_ms()
        if result_tick >= next_send_time:
            send_xy_result(
                CMD_BLOB,
                vision_frame_id,
                blob_result["found"],
                blob_result["dx"],
                blob_result["dy"],
            )
            while next_send_time <= result_tick:
                next_send_time += SEND_TIME_MS

    # RUN 只生成一次静态页面；HOME 和各调参页保持原来的实时菜单预览。
    screen_was_shown = False
    display_ms = 0
    if run_mode and not run_screen_shown:
        display_tick = time.ticks_ms()
        screen_image = ui.render(cv_bgr, blob_result, 0.0, 0.0)
        disp.show(screen_image)
        display_ms = time.ticks_ms() - display_tick
        run_screen_shown = True
        screen_was_shown = True
        print("[VISION RUN] static page ready; live display refresh disabled")
    elif display_due:
        display_tick = time.ticks_ms()
        mask_full = None
        if active_task_id == TASK_MATERIAL and ui.needs_mask_for_display():
            mask_full = expand_roi_mask(mask_roi, cv_bgr.shape, roi_rect)
        if active_task_id == TASK_MATERIAL:
            preview_bgr = make_preview(
                cv_bgr,
                mask_full,
                blob_result,
                active_config,
                ui.current_view_mode(),
            )
        elif active_task_id == TASK_RING:
            preview_bgr = make_ring_preview(
                cv_bgr,
                mask_roi,
                blob_result,
                active_config["ring"],
                ui.current_view_mode(),
            )
        else:
            preview_bgr = cv_bgr
        screen_image = ui.render(
            preview_bgr, blob_result, vision_fps_value, display_fps_value
        )
        disp.show(screen_image)
        display_ms = time.ticks_ms() - display_tick
        last_display_time = time.ticks_ms()
        force_display = False
        screen_was_shown = True
        display_fps_frames += 1

    vision_fps_frames += 1
    fps_now = time.ticks_ms()
    fps_elapsed = fps_now - fps_tick
    if fps_elapsed >= 500:
        vision_fps_value = vision_fps_frames * 1000.0 / fps_elapsed
        display_fps_value = display_fps_frames * 1000.0 / fps_elapsed
        vision_fps_frames = 0
        display_fps_frames = 0
        fps_tick = fps_now

    if active_task_id == TASK_RING:
        ring_perf_capture_ms += capture_ms
        ring_perf_detect_ms += detect_ms
        ring_perf_mask_ms += blob_result.get("mask_ms", 0.0)
        ring_perf_group_ms += blob_result.get("group_ms", 0.0)
        ring_perf_contour_ms += blob_result.get("contour_ms", 0.0)
        ring_perf_filter_ms += blob_result.get("filter_ms", 0.0)
        ring_perf_frames += 1
        if screen_was_shown:
            ring_perf_display_ms += display_ms
            ring_perf_displays += 1
        ring_perf_elapsed = fps_now - ring_perf_tick
        if ring_perf_elapsed >= RING_PERF_TIME_MS and ring_perf_frames > 0:
            print(
                "[RING PERF] {} FPS:{:.1f} CAP:{:.1f}ms DET:{:.1f}ms "
                "DSP:{:.1f}ms MASK:{:.1f} GRP:{:.1f} CTR:{:.1f} FLT:{:.1f} "
                "CAND:{} CNT:{} RAY:{}".format(
                    "RUN" if run_mode else "UI",
                    ring_perf_frames * 1000.0 / ring_perf_elapsed,
                    ring_perf_capture_ms / float(ring_perf_frames),
                    ring_perf_detect_ms / float(ring_perf_frames),
                    (ring_perf_display_ms / float(ring_perf_displays)
                     if ring_perf_displays else 0.0),
                    ring_perf_mask_ms / float(ring_perf_frames),
                    ring_perf_group_ms / float(ring_perf_frames),
                    ring_perf_contour_ms / float(ring_perf_frames),
                    ring_perf_filter_ms / float(ring_perf_frames),
                    blob_result.get("candidate_count", 0),
                    blob_result.get("contour_count", 0),
                    blob_result.get("feature_count", 0),
                )
            )
            ring_perf_tick = fps_now
            ring_perf_capture_ms = 0
            ring_perf_detect_ms = 0
            ring_perf_display_ms = 0
            ring_perf_mask_ms = 0
            ring_perf_group_ms = 0
            ring_perf_contour_ms = 0
            ring_perf_filter_ms = 0
            ring_perf_frames = 0
            ring_perf_displays = 0

    # RUN 期间也持续读触摸；本地调试可点“返回”，正式 F407 B2 会保持 RUN。
    touch_event = touch_input.poll()
    if touch_event is not None:
        action = ui.handle_touch(touch_event[0], touch_event[1], touch_event[2])
        if not touch_event[2] and not run_mode:
            force_display = True
        if action is not None and action.get("type") == "save":
            saved_config, save_status = config_store.save(action["config"])
            if saved_config is None:
                ui.save_failed(save_status)
            elif action.get("stay", False):
                ui.set_saved_config(saved_config, save_status)
            else:
                ui.save_succeeded(saved_config, save_status)
            # 下次进入任务一RUN时重新按最新ROI、中心点和阈值生成缩放参数。
            task_manager.invalidate_material_geometry()
            force_display = True

    time.sleep_ms(1)




# # ################################ 物料识别触摸调参程序 #########################################################################
# from maix import camera, display, app, time, image, touchscreen, uart
#
# from vision_config import (
#     IMAGE_WIDTH,
#     IMAGE_HEIGHT,
#     MORPH_CLOSE_ONLY,
#     MORPH_OPEN_CLOSE,
#     MORPH_OPEN_ONLY,
#     RUN_PROCESS_HEIGHT,
#     RUN_PROCESS_WIDTH,
#     VisionConfigStore,
# )
# from vision_blob import (
#     apply_color_threshold,
#     close_mask,
#     convert_color_space,
#     expand_roi_mask,
#     extract_roi,
#     find_largest_blob,
#     make_run_geometry,
#     make_preview,
#     map_run_result_to_source,
#     open_mask,
#     resize_run_roi,
# )
# from vision_ui import TouchInput, VisionUI
#
#
# # 当前阶段只发送 B2 单帧原始结果，仍由触摸 RUN 控制是否运行。
# # 后续接入 F407 状态机时，再增加 B0/B2/B3/B4 的接收与任务切换。
# FRAME_HEAD_1 = 0xAA
# FRAME_HEAD_2 = 0x55
# CMD_BLOB = 0xB2
# RESULT_DATA_LEN = 6
# RESULT_SEND_INTERVAL_MS = 50
# TX_DEBUG_INTERVAL_MS = 1000
# SERIAL_DEVICE = "/dev/serial0"
#
# def _result_checksum(cmd, data):
#     """计算 AA 55 CMD LEN DATA SUM 协议中的单字节和校验。"""
#     total = int(cmd) + len(data)
#     for value in data:
#         total += int(value)
#     return total & 0xFF
#
#
# def _append_i16_le(data, value):
#     """把有符号坐标限制在 int16 范围并按小端序追加。"""
#     value = max(-32768, min(32767, int(value)))
#     if value < 0:
#         value += 0x10000
#     data.append(value & 0xFF)
#     data.append((value >> 8) & 0xFF)
#
#
# def build_blob_result_frame(frame_id, raw_ok, dx, dy):
#     """构造 11 字节B2原始帧；dx/dy始终属于320x240坐标系。"""
#     data = bytearray()
#     data.append(int(frame_id) & 0xFF)
#     data.append(1 if raw_ok else 0)
#     _append_i16_le(data, dx)
#     _append_i16_le(data, dy)
#
#     frame = bytearray([FRAME_HEAD_1, FRAME_HEAD_2, CMD_BLOB, RESULT_DATA_LEN])
#     frame.extend(data)
#     frame.append(_result_checksum(CMD_BLOB, data))
#     return frame
#
#
# def send_blob_result_if_due(now_tick, frame_id, raw_ok, dx, dy):
#     """按 50 ms 平均节拍发送最新快照，不阻塞或降低视觉识别帧率。"""
#     global next_result_send_tick, last_tx_debug_tick, result_tx_count
#
#     if vision_serial is None:
#         return
#     if now_tick < next_result_send_tick:
#         return
#
#     frame = build_blob_result_frame(frame_id, raw_ok, dx, dy)
#     vision_serial.write(bytes(frame))
#     result_tx_count += 1
#
#     # 使用累计截止时间而不是 now + 50，避免 30 FPS 时退化为约 15 Hz。
#     # 单线程只能在某一识别帧结束后发送，所以单次间隔会有一帧以内的抖动。
#     while next_result_send_tick <= now_tick:
#         next_result_send_tick += RESULT_SEND_INTERVAL_MS
#
#     # 限频打印只用于首轮联调；比赛版可整体关闭。
#     if now_tick - last_tx_debug_tick >= TX_DEBUG_INTERVAL_MS:
#         debug_elapsed = max(1, now_tick - last_tx_debug_tick)
#         tx_hz = result_tx_count * 1000.0 / debug_elapsed
#         print("[VISION TX] TXHZ:{:.1f}".format(tx_hz),
#               " ".join("%02X" % value for value in frame))
#         last_tx_debug_tick = now_tick
#         result_tx_count = 0
#
#
# cam = camera.Camera(IMAGE_WIDTH, IMAGE_HEIGHT)
# disp = display.Display()
# ts = touchscreen.TouchScreen()
#
# try:
#     vision_serial = uart.UART(SERIAL_DEVICE, 115200)
#     serial_status = "ready"
# except Exception as error:
#     vision_serial = None
#     serial_status = "disabled: {}".format(error)
#
# try:
#     screen_w = int(disp.width())
#     screen_h = int(disp.height())
# except Exception:
#     screen_w = 480
#     screen_h = 640
#
# try:
#     data_dir = app.get_app_data_path()
# except Exception:
#     data_dir = "./data"
#
# config_store = VisionConfigStore(data_dir)
# vision_config, config_status = config_store.load()
# ui = VisionUI(screen_w, screen_h, vision_config, config_status)
# touch_input = TouchInput(ts)
#
# print("[VISION UI] display: {}x{}".format(screen_w, screen_h))
# print("[VISION UI] config:", config_status)
# print("[VISION CONFIG] morphology:", vision_config["morph_mode"])
# print("[VISION RUN] process: {}x{}; config coordinates: {}x{}".format(
#     RUN_PROCESS_WIDTH, RUN_PROCESS_HEIGHT, IMAGE_WIDTH, IMAGE_HEIGHT
# ))
# print("[VISION UART] {} @ 115200: {}".format(SERIAL_DEVICE, serial_status))
# print("[VISION BUILD] B2_RAW6_TX_50MS_V2")
# print("[VISION UI] touchscreen available:", hasattr(ts, "available"))
#
# # HOME/调参页低频刷新屏幕；RUN 页进入时只显示一次静态状态页。
# DISPLAY_INTERVAL_MS = 100
# PERF_WARMUP_MS = 5000
# vision_fps_value = 0.0
# display_fps_value = 0.0
# vision_fps_frames = 0
# display_fps_frames = 0
# fps_tick = time.ticks_ms()
# last_display_tick = fps_tick - DISPLAY_INTERVAL_MS
# force_display = True
# run_screen_shown = False
# previous_run_mode = ui.is_run_mode()
# run_geometry = None
# vision_frame_id = 0
# next_result_send_tick = fps_tick
# last_tx_debug_tick = fps_tick - TX_DEBUG_INTERVAL_MS
# result_tx_count = 0
#
# # 每次切换 RUN/UI 路径后重新预热，避免首次字体、缓存和 OpenCV 开销污染统计。
# warmup_tick = fps_tick
# perf_ready = False
# perf_tick = fps_tick
# perf_vision_frames = 0
# perf_screen_frames = 0
# perf_camera_ms = 0
# perf_resize_ms = 0
# perf_mask_ms = 0
# perf_convert_ms = 0
# perf_threshold_ms = 0
# perf_open_ms = 0
# perf_close_ms = 0
# perf_blob_ms = 0
# perf_preview_ms = 0
# perf_ui_ms = 0
# perf_display_ms = 0
#
# while not app.need_exit():
#     loop_tick = time.ticks_ms()
#     run_mode = ui.is_run_mode()
#
#     # 运行与调参是两条真实执行路径。切换后重置计数器，分别观察纯识别和带 UI 的性能。
#     if run_mode != previous_run_mode:
#         previous_run_mode = run_mode
#         run_screen_shown = False
#         force_display = not run_mode
#         vision_fps_value = 0.0
#         display_fps_value = 0.0
#         vision_fps_frames = 0
#         display_fps_frames = 0
#         fps_tick = loop_tick
#         warmup_tick = loop_tick
#         perf_ready = False
#         perf_tick = loop_tick
#         perf_vision_frames = 0
#         perf_screen_frames = 0
#         perf_camera_ms = 0
#         perf_resize_ms = 0
#         perf_mask_ms = 0
#         perf_convert_ms = 0
#         perf_threshold_ms = 0
#         perf_open_ms = 0
#         perf_close_ms = 0
#         perf_blob_ms = 0
#         perf_preview_ms = 0
#         perf_ui_ms = 0
#         perf_display_ms = 0
#         # RUN 期间保存参数不会变化，因此缩放坐标和面积阈值只在进入时计算一次。
#         if run_mode:
#             run_geometry = make_run_geometry(
#                 ui.detection_config(), IMAGE_WIDTH, IMAGE_HEIGHT,
#                 RUN_PROCESS_WIDTH, RUN_PROCESS_HEIGHT
#             )
#             vision_frame_id = 0
#             # 进入 RUN 后，第一帧识别完成即可立即发送，不额外等待 50 ms。
#             next_result_send_tick = loop_tick
#         else:
#             run_geometry = None
#         print("[VISION MODE]", "RUN FAST" if run_mode else "UI/CALIBRATE")
#
#     # RUN 不使用周期刷新；非 RUN 页面才根据时间决定是否生成预览和显示。
#     display_due = (
#         not run_mode and
#         (force_display or loop_tick - last_display_tick >= DISPLAY_INTERVAL_MS)
#     )
#
#     # 1. 获取 320x240 相机画面并转换为 OpenCV BGR。
#     camera_image = cam.read()
#     cv_bgr = image.image2cv(camera_image, ensure_bgr=True, copy=True)
#     camera_done_tick = time.ticks_ms()
#
#     # 2. 调参保持 320x240；RUN 只缩放 ROI，并用专用配置在小图坐标系识别。
#     active_config = ui.detection_config()
#     if run_mode:
#         if run_geometry is None:
#             run_geometry = make_run_geometry(
#                 active_config, IMAGE_WIDTH, IMAGE_HEIGHT,
#                 RUN_PROCESS_WIDTH, RUN_PROCESS_HEIGHT
#             )
#         process_config = run_geometry["config"]
#         roi_bgr = resize_run_roi(cv_bgr, run_geometry)
#         roi_rect = run_geometry["process_roi"]
#     else:
#         process_config = active_config
#         roi_bgr, roi_rect = extract_roi(cv_bgr, active_config)
#     resize_done_tick = time.ticks_ms()
#
#     # 颜色转换、阈值、形态学继续分段计时，缩放耗时独立记入 RSZ。
#     converted_roi = convert_color_space(roi_bgr, process_config["mode"])
#     convert_done_tick = time.ticks_ms()
#     raw_mask_roi = apply_color_threshold(converted_roi, process_config)
#     threshold_done_tick = time.ticks_ms()
#     morph_mode = process_config.get("morph_mode", MORPH_OPEN_CLOSE)
#
#     # 四个实验档位必须在主识别链路中真正分流；跳过的阶段计时固定为 0 ms。
#     if morph_mode in (MORPH_OPEN_ONLY, MORPH_OPEN_CLOSE):
#         opened_mask_roi = open_mask(raw_mask_roi)
#         open_done_tick = time.ticks_ms()
#     else:
#         opened_mask_roi = raw_mask_roi
#         open_done_tick = threshold_done_tick
#
#     if morph_mode in (MORPH_CLOSE_ONLY, MORPH_OPEN_CLOSE):
#         mask_roi = close_mask(opened_mask_roi)
#         mask_done_tick = time.ticks_ms()
#     else:
#         mask_roi = opened_mask_roi
#         mask_done_tick = open_done_tick
#     process_result = find_largest_blob(mask_roi, process_config, roi_rect)
#     if run_mode:
#         blob_result = map_run_result_to_source(
#             process_result, active_config, run_geometry
#         )
#     else:
#         blob_result = process_result
#     blob_done_tick = time.ticks_ms()
#
#     # 每个 RUN 识别帧只更新一次 FRAME_ID；50 ms 发送器读取当时的最新结果。
#     # F407 仍需按 FRAME_ID 去重，为以后独立发送调度或重复发送保留兼容性。
#     if run_mode:
#         vision_frame_id = (vision_frame_id + 1) & 0xFF
#         send_blob_result_if_due(
#             blob_done_tick,
#             vision_frame_id,
#             blob_result["found"],
#             blob_result["dx"],
#             blob_result["dy"],
#         )
#
#     # 3. RUN 页面只在进入时绘制一次；高速循环不创建 preview/mask_full，也不调用 UI/show。
#     preview_cost = 0
#     ui_cost = 0
#     display_cost = 0
#     screen_was_shown = False
#     if run_mode and not run_screen_shown:
#         run_render_tick = time.ticks_ms()
#         screen_image = ui.render(cv_bgr, blob_result, 0.0, 0.0)
#         run_ui_done_tick = time.ticks_ms()
#         disp.show(screen_image)
#         display_done_tick = time.ticks_ms()
#         ui_cost = run_ui_done_tick - run_render_tick
#         display_cost = display_done_tick - run_ui_done_tick
#         run_screen_shown = True
#         screen_was_shown = True
#         print("[VISION RUN] static screen ready; preview and display refresh disabled")
#     elif display_due:
#         mask_full = None
#         if ui.needs_mask_for_display():
#             mask_full = expand_roi_mask(mask_roi, cv_bgr.shape, roi_rect)
#         preview_bgr = make_preview(
#             cv_bgr, mask_full, blob_result, active_config, ui.current_view_mode()
#         )
#         preview_done_tick = time.ticks_ms()
#         screen_image = ui.render(
#             preview_bgr, blob_result, vision_fps_value, display_fps_value
#         )
#         ui_done_tick = time.ticks_ms()
#         # screen_image 已经与屏幕同尺寸，先测试默认 show，避免额外 FIT_FILL 路径。
#         disp.show(screen_image)
#         display_done_tick = time.ticks_ms()
#         preview_cost = preview_done_tick - blob_done_tick
#         ui_cost = ui_done_tick - preview_done_tick
#         display_cost = display_done_tick - ui_done_tick
#         # 从 show 完成时开始计下一次间隔，避免显示本身耗时导致下一轮立刻再次到期。
#         last_display_tick = display_done_tick
#         force_display = False
#         screen_was_shown = True
#         display_fps_frames += 1
#     else:
#         display_done_tick = blob_done_tick
#
#     vision_fps_frames += 1
#     fps_elapsed = display_done_tick - fps_tick
#     if fps_elapsed >= 500:
#         vision_fps_value = vision_fps_frames * 1000.0 / fps_elapsed
#         display_fps_value = display_fps_frames * 1000.0 / fps_elapsed
#         vision_fps_frames = 0
#         display_fps_frames = 0
#         fps_tick = display_done_tick
#
#     if not perf_ready:
#         if display_done_tick - warmup_tick >= PERF_WARMUP_MS:
#             perf_ready = True
#             perf_tick = display_done_tick
#     else:
#         perf_vision_frames += 1
#         perf_camera_ms += camera_done_tick - loop_tick
#         perf_resize_ms += resize_done_tick - camera_done_tick
#         perf_mask_ms += mask_done_tick - resize_done_tick
#         perf_convert_ms += convert_done_tick - resize_done_tick
#         perf_threshold_ms += threshold_done_tick - convert_done_tick
#         perf_open_ms += open_done_tick - threshold_done_tick
#         perf_close_ms += mask_done_tick - open_done_tick
#         perf_blob_ms += blob_done_tick - mask_done_tick
#         if screen_was_shown:
#             perf_screen_frames += 1
#             perf_preview_ms += preview_cost
#             perf_ui_ms += ui_cost
#             perf_display_ms += display_cost
#
#         if display_done_tick - perf_tick >= 2000:
#             vision_count = max(1, perf_vision_frames)
#             base_roi = active_config["roi"]
#             base_roi_width, base_roi_height = base_roi["w"], base_roi["h"]
#             process_roi_width, process_roi_height = roi_rect[2], roi_rect[3]
#             mode_name = active_config["mode"]
#             color_name = active_config["target_color"]
#             if run_mode:
#                 print(("[VISION RUN PERF] RFPS:{:.1f} {}/{} ROI:{}x{}->{}x{} MORPH:{} "
#                        "CAM:{:.1f}ms RSZ:{:.1f}ms "
#                        "MASK:{:.1f}ms(CVT:{:.1f} THR:{:.1f} OPN:{:.1f} CLS:{:.1f}) "
#                        "BLOB:{:.1f}ms OK:{} XY:{},{} DXY:{},{} AREA:{}").format(
#                     vision_fps_value,
#                     mode_name,
#                     color_name,
#                     base_roi_width,
#                     base_roi_height,
#                     process_roi_width,
#                     process_roi_height,
#                     morph_mode,
#                     perf_camera_ms / vision_count,
#                     perf_resize_ms / vision_count,
#                     perf_mask_ms / vision_count,
#                     perf_convert_ms / vision_count,
#                     perf_threshold_ms / vision_count,
#                     perf_open_ms / vision_count,
#                     perf_close_ms / vision_count,
#                     perf_blob_ms / vision_count,
#                     1 if blob_result["found"] else 0,
#                     blob_result["cx"],
#                     blob_result["cy"],
#                     blob_result["dx"],
#                     blob_result["dy"],
#                     blob_result["area"],
#                 ))
#             else:
#                 screen_count = max(1, perf_screen_frames)
#                 print(("[VISION UI PERF] VFPS:{:.1f} SFPS:{:.1f} {}/{} ROI:{}x{} MORPH:{} "
#                        "CAM:{:.1f}ms "
#                        "MASK:{:.1f}ms(CVT:{:.1f} THR:{:.1f} OPN:{:.1f} CLS:{:.1f}) "
#                        "BLOB:{:.1f}ms PREV:{:.1f}ms "
#                        "UI:{:.1f}ms DISP:{:.1f}ms").format(
#                     vision_fps_value,
#                     display_fps_value,
#                     mode_name,
#                     color_name,
#                     base_roi_width,
#                     base_roi_height,
#                     morph_mode,
#                     perf_camera_ms / vision_count,
#                     perf_mask_ms / vision_count,
#                     perf_convert_ms / vision_count,
#                     perf_threshold_ms / vision_count,
#                     perf_open_ms / vision_count,
#                     perf_close_ms / vision_count,
#                     perf_blob_ms / vision_count,
#                     perf_preview_ms / screen_count,
#                     perf_ui_ms / screen_count,
#                     perf_display_ms / screen_count,
#                 ))
#             perf_tick = display_done_tick
#             perf_vision_frames = 0
#             perf_screen_frames = 0
#             perf_camera_ms = 0
#             perf_resize_ms = 0
#             perf_mask_ms = 0
#             perf_convert_ms = 0
#             perf_threshold_ms = 0
#             perf_open_ms = 0
#             perf_close_ms = 0
#             perf_blob_ms = 0
#             perf_preview_ms = 0
#             perf_ui_ms = 0
#             perf_display_ms = 0
#
#     # 4. RUN 虽不刷新屏幕，仍持续轮询触摸，因此静态“返回”按钮可以正常退出。
#     touch_event = touch_input.poll()
#     if touch_event is not None:
#         action = ui.handle_touch(touch_event[0], touch_event[1], touch_event[2])
#         if not touch_event[2] and not run_mode:
#             force_display = True
#         if action is not None and action.get("type") == "save":
#             saved_config, save_status = config_store.save(action["config"])
#             if saved_config is None:
#                 ui.save_failed(save_status)
#             elif action.get("stay", False):
#                 ui.set_saved_config(saved_config, save_status)
#             else:
#                 ui.save_succeeded(saved_config, save_status)
#             force_display = True
#
#     time.sleep_ms(1)
#
# ############################ 圆环检测测试 ############################################################################

# from maix import camera, display, app, time, image
# import cv2
# import numpy as np
# import math


# # ================== 图像参数 ==================
# IMG_W = 320
# IMG_H = 240
# CENTER_X = IMG_W // 2
# CENTER_Y = IMG_H // 2


# # ================== 圆环数字目标码 ==================
# SUM_NONE = 0xD0
# SUM_1    = 0xD1
# SUM_2    = 0xD2
# SUM_3    = 0xD3


# # ================== 测试目标 ==================
# # 这个只是测试时用，表示你想找哪个数字的圆环
# TEST_TARGET = SUM_1
# # TEST_TARGET = SUM_2
# # TEST_TARGET = SUM_3


# # ================== 圆环识别参数 ==================
# BLACK_V_MAX = 85          # 黑色阈值，越大识别黑色越多
# MIN_RING_AREA = 500       # 圆环最小面积
# MAX_RING_AREA = 50000     # 圆环最大面积
# MIN_RATIO = 0.45          # 宽高比下限
# MAX_RATIO = 2.20          # 宽高比上限
# MIN_CIRCULARITY = 0.18    # 圆形度，不要太严
# MIN_HOLE_AREA = 80        # 内孔最小面积


# # ================== OCR参数 ==================
# OCR_W = 32
# OCR_H = 48

# # 是否只取画面中间区域，先设 False，调稳定后可以改 True
# USE_SEARCH_ROI = False

# # 搜索区域，只有 USE_SEARCH_ROI=True 才生效
# SEARCH_X1 = 0
# SEARCH_Y1 = 0
# SEARCH_X2 = IMG_W
# SEARCH_Y2 = IMG_H


# cam = camera.Camera(IMG_W, IMG_H)
# disp = display.Display()


# # ================== 工具函数 ==================
# def target_to_num(target):
#     if target == SUM_1 or target == 1:
#         return 1
#     elif target == SUM_2 or target == 2:
#         return 2
#     elif target == SUM_3 or target == 3:
#         return 3
#     else:
#         return 0


# def num_to_target(num):
#     if num == 1:
#         return SUM_1
#     elif num == 2:
#         return SUM_2
#     elif num == 3:
#         return SUM_3
#     else:
#         return SUM_NONE


# def cv_find_contours_tree(mask):
#     result = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

#     if len(result) == 2:
#         contours, hierarchy = result
#     else:
#         _, contours, hierarchy = result

#     return contours, hierarchy


# def cv_find_contours_external(mask):
#     result = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

#     if len(result) == 2:
#         contours, hierarchy = result
#     else:
#         _, contours, hierarchy = result

#     return contours


# def draw_center_cross(cv_img):
#     cv2.line(cv_img, (CENTER_X - 8, CENTER_Y), (CENTER_X + 8, CENTER_Y), (255, 255, 255), 1)
#     cv2.line(cv_img, (CENTER_X, CENTER_Y - 8), (CENTER_X, CENTER_Y + 8), (255, 255, 255), 1)


# # ================== 生成 OCR 暴力模板 ==================
# def make_one_template(num, scale, thickness, dx, dy):
#     """
#     生成一个数字模板。
#     模板规则：
#     白底黑字，最后转成黑字为255、背景为0。
#     """

#     canvas = np.ones((OCR_H, OCR_W), dtype=np.uint8) * 255

#     font = cv2.FONT_HERSHEY_SIMPLEX
#     text = str(num)

#     # 估计文字大小
#     try:
#         text_size, baseline = cv2.getTextSize(text, font, scale, thickness)
#         tw, th = text_size
#     except Exception:
#         tw = int(16 * scale)
#         th = int(32 * scale)

#     x = int((OCR_W - tw) / 2) + dx
#     y = int((OCR_H + th) / 2) + dy

#     cv2.putText(canvas, text, (x, y), font, scale, (0), thickness)

#     # 黑字变成255，白底变成0
#     _, binary = cv2.threshold(canvas, 128, 255, cv2.THRESH_BINARY_INV)

#     return binary


# def build_digit_templates():
#     """
#     给 1/2/3 各生成多套模板。
#     这样比单一模板鲁棒：字体稍微偏大、偏小、偏移，都能匹配。
#     """

#     templates = {
#         1: [],
#         2: [],
#         3: []
#     }

#     scales = [1.10, 1.20, 1.30, 1.40]
#     thicknesses = [2, 3]
#     offsets = [-2, 0, 2]

#     for num in [1, 2, 3]:
#         for scale in scales:
#             for thick in thicknesses:
#                 for dx in offsets:
#                     for dy in offsets:
#                         tmpl = make_one_template(num, scale, thick, dx, dy)
#                         templates[num].append(tmpl)

#     return templates


# DIGIT_TEMPLATES = build_digit_templates()


# # ================== OCR预处理 ==================
# def normalize_digit_image(binary):
#     """
#     把 ROI 里的数字抠出来并归一化到 32x48。
#     输入 binary 要求：数字黑色区域为255，背景为0。
#     """

#     contours = cv_find_contours_external(binary)

#     if not contours:
#         return None

#     # 找最大数字轮廓
#     best = None
#     best_area = 0

#     for c in contours:
#         area = cv2.contourArea(c)
#         if area > best_area:
#             best_area = area
#             best = c

#     if best is None or best_area < 10:
#         return None

#     x, y, w, h = cv2.boundingRect(best)

#     # 过滤太奇怪的轮廓
#     if w < 2 or h < 5:
#         return None

#     digit = binary[y:y+h, x:x+w]

#     # 保持比例缩放到模板画布
#     canvas = np.zeros((OCR_H, OCR_W), dtype=np.uint8)

#     scale = min((OCR_W - 6) / float(w), (OCR_H - 6) / float(h))
#     new_w = int(w * scale)
#     new_h = int(h * scale)

#     if new_w <= 0 or new_h <= 0:
#         return None

#     digit_resized = cv2.resize(digit, (new_w, new_h))

#     px = (OCR_W - new_w) // 2
#     py = (OCR_H - new_h) // 2

#     canvas[py:py+new_h, px:px+new_w] = digit_resized

#     return canvas


# def recognize_digit_bruteforce(roi_bgr):
#     """
#     暴力 OCR：
#     1. ROI 灰度化
#     2. 二值化成黑字=255、背景=0
#     3. 抠最大数字轮廓
#     4. 和 1/2/3 的多套模板逐个 absdiff
#     5. 分数最小者为识别结果
#     """

#     if roi_bgr is None or roi_bgr.size == 0:
#         return 0, 999999, None

#     gray = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2GRAY)

#     # 轻微模糊，减少噪声
#     gray = cv2.GaussianBlur(gray, (3, 3), 0)

#     # OTSU 二值化
#     # THRESH_BINARY_INV：黑色数字变成255，白色背景变成0
#     _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

#     # 去噪
#     kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (2, 2))
#     binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel, iterations=1)

#     norm = normalize_digit_image(binary)

#     if norm is None:
#         return 0, 999999, binary

#     best_num = 0
#     best_score = 999999

#     for num in [1, 2, 3]:
#         for tmpl in DIGIT_TEMPLATES[num]:
#             diff = cv2.absdiff(norm, tmpl)

#             # 越小越像
#             score = cv2.countNonZero(diff)

#             if score < best_score:
#                 best_score = score
#                 best_num = num

#     return best_num, best_score, norm


# # ================== 找黑色圆环候选 ==================
# def find_ring_candidates(img):
#     """
#     返回所有圆环候选:
#     [
#         {
#             "x": x, "y": y, "w": w, "h": h,
#             "cx": cx, "cy": cy,
#             "dx": dx, "dy": dy,
#             "area": area,
#             "circularity": circularity,
#             "rect": (x,y,w,h)
#         }
#     ],
#     debug_img
#     """

#     cv_img = image.image2cv(img, ensure_bgr=True, copy=True)

#     if USE_SEARCH_ROI:
#         search_img = cv_img[SEARCH_Y1:SEARCH_Y2, SEARCH_X1:SEARCH_X2]
#         offset_x = SEARCH_X1
#         offset_y = SEARCH_Y1
#     else:
#         search_img = cv_img
#         offset_x = 0
#         offset_y = 0

#     hsv = cv2.cvtColor(search_img, cv2.COLOR_BGR2HSV)

#     # 黑色区域：V低
#     mask = cv2.inRange(hsv, np.array([0, 0, 0]), np.array([180, 255, BLACK_V_MAX]))

#     kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))

#     # 先闭运算补断裂，再开运算去噪
#     mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
#     mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)

#     contours, hierarchy = cv_find_contours_tree(mask)

#     candidates = []

#     if hierarchy is None or len(contours) == 0:
#         return candidates, cv_img

#     hierarchy = hierarchy[0]

#     for i, c in enumerate(contours):
#         area = cv2.contourArea(c)

#         if area < MIN_RING_AREA or area > MAX_RING_AREA:
#             continue

#         x, y, w, h = cv2.boundingRect(c)

#         if h == 0:
#             continue

#         ratio = w / float(h)

#         if ratio < MIN_RATIO or ratio > MAX_RATIO:
#             continue

#         peri = cv2.arcLength(c, True)
#         if peri <= 1:
#             continue

#         circularity = 4.0 * math.pi * area / (peri * peri)

#         if circularity < MIN_CIRCULARITY:
#             continue

#         # 判断是否有内孔：child index
#         child_idx = hierarchy[i][2]
#         if child_idx < 0:
#             continue

#         child_area = cv2.contourArea(contours[child_idx])
#         if child_area < MIN_HOLE_AREA:
#             continue

#         # 转回全图坐标
#         x += offset_x
#         y += offset_y

#         cx = x + w // 2
#         cy = y + h // 2
#         dx = cx - CENTER_X
#         dy = cy - CENTER_Y

#         candidates.append({
#             "x": x,
#             "y": y,
#             "w": w,
#             "h": h,
#             "cx": cx,
#             "cy": cy,
#             "dx": dx,
#             "dy": dy,
#             "area": area,
#             "circularity": circularity,
#             "rect": (x, y, w, h)
#         })

#     return candidates, cv_img


# def crop_digit_roi(cv_img, ring):
#     """
#     从圆环中间裁剪数字 ROI。
#     注意不要裁到黑色圆环边缘，否则会干扰数字。
#     """

#     x = ring["x"]
#     y = ring["y"]
#     w = ring["w"]
#     h = ring["h"]

#     # 取中间 58% 区域
#     roi_x = x + int(w * 0.21)
#     roi_y = y + int(h * 0.21)
#     roi_w = int(w * 0.58)
#     roi_h = int(h * 0.58)

#     # 防止越界
#     roi_x = max(0, roi_x)
#     roi_y = max(0, roi_y)
#     roi_w = min(IMG_W - roi_x, roi_w)
#     roi_h = min(IMG_H - roi_y, roi_h)

#     if roi_w <= 0 or roi_h <= 0:
#         return None, (roi_x, roi_y, roi_w, roi_h)

#     roi = cv_img[roi_y:roi_y+roi_h, roi_x:roi_x+roi_w]

#     return roi, (roi_x, roi_y, roi_w, roi_h)


# # ================== find_ring 主函数 ==================
# def find_ring(img, target):
#     """
#     找指定数字圆环。

#     输入:
#     target = SUM_1 / SUM_2 / SUM_3
#     或者 target = 1 / 2 / 3

#     返回:
#     ok, dx, dy, debug_img, detected_num
#     """

#     target_num = target_to_num(target)

#     candidates, debug_img = find_ring_candidates(img)

#     draw_center_cross(debug_img)

#     best_target = None
#     all_results = []

#     for ring in candidates:
#         roi, roi_rect = crop_digit_roi(debug_img, ring)

#         detected_num, score, norm_img = recognize_digit_bruteforce(roi)

#         ring["num"] = detected_num
#         ring["score"] = score
#         ring["roi_rect"] = roi_rect

#         all_results.append(ring)

#         x = ring["x"]
#         y = ring["y"]
#         w = ring["w"]
#         h = ring["h"]
#         cx = ring["cx"]
#         cy = ring["cy"]

#         # 所有候选画蓝框
#         cv2.rectangle(debug_img, (x, y), (x + w, y + h), (255, 0, 0), 1)

#         # ROI画黄框
#         rx, ry, rw, rh = roi_rect
#         cv2.rectangle(debug_img, (rx, ry), (rx + rw, ry + rh), (0, 255, 255), 1)

#         cv2.circle(debug_img, (cx, cy), 4, (0, 0, 255), -1)

#         text = "N:{} S:{}".format(detected_num, score)
#         cv2.putText(debug_img, text, (x, max(15, y - 5)),
#                     cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)

#         # 如果是目标数字，候选进入目标列表
#         if target_num != 0 and detected_num == target_num:
#             if best_target is None:
#                 best_target = ring
#             else:
#                 # 优先选择匹配分数更低的；如果接近，则选面积更大的
#                 if score < best_target["score"]:
#                     best_target = ring

#     if best_target is not None:
#         x = best_target["x"]
#         y = best_target["y"]
#         w = best_target["w"]
#         h = best_target["h"]
#         cx = best_target["cx"]
#         cy = best_target["cy"]
#         dx = best_target["dx"]
#         dy = best_target["dy"]
#         area = best_target["area"]
#         num = best_target["num"]
#         score = best_target["score"]

#         # 目标圆环画绿粗框
#         cv2.rectangle(debug_img, (x, y), (x + w, y + h), (0, 255, 0), 2)
#         cv2.circle(debug_img, (cx, cy), 5, (0, 0, 255), -1)
#         cv2.line(debug_img, (CENTER_X, CENTER_Y), (cx, cy), (255, 255, 255), 1)

#         info1 = "TARGET:{} FOUND:{}".format(target_num, num)
#         info2 = "dx:{} dy:{} area:{}".format(dx, dy, int(area))
#         info3 = "score:{}".format(score)

#         cv2.putText(debug_img, info1, (5, 20),
#                     cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
#         cv2.putText(debug_img, info2, (5, 40),
#                     cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
#         cv2.putText(debug_img, info3, (5, 60),
#                     cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

#         return 1, dx, dy, image.cv2image(debug_img), num

#     else:
#         info1 = "TARGET:{} NOT FOUND".format(target_num)
#         info2 = "candidates:{}".format(len(candidates))

#         cv2.putText(debug_img, info1, (5, 20),
#                     cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)
#         cv2.putText(debug_img, info2, (5, 45),
#                     cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

#         return 0, 0, 0, image.cv2image(debug_img), 0


# # ================== 主循环：纯测试，不通信 ==================
# while not app.need_exit():
#     img = cam.read()

#     ok, dx, dy, debug_img, num = find_ring(img, TEST_TARGET)

#     disp.show(debug_img)

#     print("ok:", ok,
#           "target:", target_to_num(TEST_TARGET),
#           "num:", num,
#           "dx:", dx,
#           "dy:", dy)

#     time.sleep_ms(30)


# ####################################### 通信测试 ##########################
# from maix import uart, pinmap, app, time


# # UART1
# pinmap.set_pin_function("A18", "UART1_RX")
# pinmap.set_pin_function("A19", "UART1_TX")


# serial = uart.UART(
#     "/dev/ttyS0",
#     115200
# )


# print("UART TEST START")


# while not app.need_exit():

#     # serial.write(b"hello\r\n")

#     data = serial.read()

#     if data:

#         print("RX:",
#               " ".join("%02X" % b for b in data))

#         print(data)


#         # 原样返回
#         serial.write(data)


#     time.sleep_ms(10)
