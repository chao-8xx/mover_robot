 MaixCam 主要任务: 识别物料色块、圆环数字以及圆环中心、堆放物料顶部
# 协议: AA 55 CMD LEN DATA SUM
# 返回坐标包 DATA: TARGET OK dxL dxH dyL dyH

from maix import camera, display, app, time, image, uart, pinmap
import cv2
import numpy as np
import math
from struct import pack

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
image_width = 320
image_height = 240
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

SEND_TIME_MS = 20   # 数据帧发送间隔 20ms 也就是50Hz 发送一次
TX_DEBUG_TIME_MS = 1000    # 调试打印发送数据包间隔，避免串口打印过多影响识别速度 在maix端打印

# 初始化
cam = camera.Camera(image_width, image_height)
disp = display.Display()

device = "/dev/serial0"         # 教程里写的是serial1  但是实测是serial0 真他妈草了
serial = uart.UART(device, 115200)

# pinmap.set_pin_function("A18", "UART1_RX")
# pinmap.set_pin_function("A19", "UART1_TX")    # serial1 配置方法
# device = "/dev/ttyS1"
# serial = uart.UART(device, 115200)

mode = CMD_NONE
target = COLOR_NONE
rx_buf = bytearray()
last_send_time = 0
last_tx_debug_time = 0


###################################### 通信函数 ############################
# 数据包校验和
def checksum(cmd,length,data):
    sum = cmd + length
    for byte in data:
        sum += byte
    return sum & 0xFF

# 构建并发送数据包   格式: AA 55 CMD 06 TARGET OK dxL dxH dyL dyH SUM
def send_frame(cmd,data):
    length = len(data)       # 数据长度

    frame = bytearray([HEAD1, HEAD2, cmd, length])  # 帧头 + 命令字 + 长度
    frame.extend(data)  # 添加数据
    frame.append(checksum(cmd, length, data))  # 添加校验和

    serial.write(bytes(frame))  # 发送数据包  注: 这里的serial.write(bytes(frame))只能返回int型 不能返回数组
    return bytes(frame)

# 发送坐标
def send_xy_result(cmd, target, ok, dx, dy):
    global last_tx_debug_time

    data = bytearray()
    data.append(target & 0xFF)
    data.append(1 if ok else 0)     # 是否找到的标志位
    data.extend(pack("<h", int(dx)))    # 拼接坐标
    data.extend(pack("<h", int(dy)))    # 2字节 小端序
    frame = send_frame(cmd, data)
    now = time.ticks_ms()

    # 调试 打印在maix端的串口输出
    if (now - last_tx_debug_time) >= TX_DEBUG_TIME_MS:
        print("Maix TX:", " ".join("%02X" % b for b in frame))
        last_tx_debug_time = now

# 处理串口接收数据
def rx_handle():
    global rx_buf, mode

    # # 非阻塞读取串口数据
    # buf = serial.read(-1, 2)
    # if not buf:
    #     return

    # 非阻塞读取串口数据 打印接收到的内容
    buf = serial.read()
    if buf:
        print("UART1 RX:", " ".join("%02X" % b for b in buf))

    else:
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
        frame_len = 2 + 1 + 1 + length + 1

        if len(rx_buf) < frame_len:
            return

        # 数据和校验位
        data = rx_buf[4:4 + length]
        rx_checksum = rx_buf[4 + length]

        if rx_checksum == checksum(cmd, length, data):
            handle_cmd(cmd, data)

        # 删除已处理帧
        del rx_buf[:frame_len]

# 命令处理
def handle_cmd(cmd, data):
    global mode, target

    if cmd == CMD_NONE:
        mode = CMD_NONE
        target = COLOR_NONE
        print("Maix停止识别")

    elif cmd == CMD_BLOB:
        # print("ENTER CMD_BLOB SET")
        if len(data) >= 1:
            mode = CMD_BLOB
            target = data[0]    # 颜色编号 1~6
            # print("mode=",mode)
            # print("target=",target)
            # print("Maix物料识别,物料颜色为:", target)

    elif cmd == CMD_RING:
        if len(data) >= 1:
            mode = CMD_RING
            target = data[0]    # 圆环编号 1~3
        print("Maix圆环识别,圆环编号为:", target)

    elif cmd == CMD_ONUP:
        if len(data) >= 1:
            mode = CMD_ONUP
            target = data[0]    # 颜色编号 1~6
        print("Maix码垛识别,码垛颜色为:", target)


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

# 同上
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
    cv2.line(cv_img, (center_x - 8, center_x), (center_x + 8, center_y), (255, 255, 255), 1)
    cv2.line(cv_img, (center_x, center_x - 8), (center_x, center_y + 8), (255, 255, 255), 1)



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


DIGIT_TEMPLATES = build_digit_templates()


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
    if roi_bgr is None or roi_bgr.size == 0:
        return 0, 999999, None

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
#物料识别  找色块 核心函数
def find_blobs(img, target_color):
    target_color = clear_color_target(target_color)
    if BLOB_WAY == "LAB":
        return find_blobs_lab(img, target_color)
    else:
        return find_blobs_hsv(img, target_color)


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


# def find_stack(img, target_color):
#     # 暂时和物料识别用同一个函数；后面如果要专门找顶部，再单独替换这里
#     return find_blobs(img, target_color)


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

############################### 主循环：通信调度 ###########################
while not app.need_exit():
    rx_handle()

    img = cam.read()
    debug_img = img

    now = time.ticks_ms()
    can_send = (now - last_send_time) >= SEND_TIME_MS

    if mode == CMD_NONE:
        disp.show(img)
        time.sleep_ms(10)
        continue

    if mode == CMD_BLOB:
        ok, dx, dy, debug_img = find_blobs(img,target)
        if can_send:
            send_xy_result(CMD_BLOB, target, ok, dx, dy)
            last_send_time = now

    elif mode == CMD_RING:
        ok, dx, dy, debug_img, num = find_ring(img, target)
        if can_send:
            send_xy_result(CMD_RING, target, ok, dx, dy)
            last_send_time = now

    # elif mode == CMD_ONUP:
    #     ok, dx, dy, debug_img = find_stack(img, target)
    #     if can_send:
    #         send_xy_result(CMD_ONUP, target, ok, dx, dy)
    #         last_send_time = now

    disp.show(debug_img)
    time.sleep_ms(5)




# ################################ 物料识别测试 #################################################################################
# from maix import camera, display, app, time, image
# import cv2
# import numpy as np
#
# # ================== 图像参数 ==================
# IMG_W = 320
# IMG_H = 240
# CENTER_X = IMG_W // 2
# CENTER_Y = IMG_H // 2
#
# # ================== 颜色编号 ==================
# COLOR_RED = 0xC1
# COLOR_YELLOW = 0xC2
# COLOR_BLUE = 0xC3
# COLOR_GREEN = 0xC4
# COLOR_BLACK = 0xC5
# COLOR_LIGHTBLUE = 0xC6
#
# # ================== 测试选择 ==================
# # 这里切换 HSV / LAB
# TEST_METHOD = "HSV"
# # TEST_METHOD = "LAB"
#
# # 这里切换要识别的颜色
# # TEST_COLOR = COLOR_RED
# # TEST_COLOR = COLOR_YELLOW
# # TEST_COLOR = COLOR_BLUE
# TEST_COLOR = COLOR_GREEN
# # TEST_COLOR = COLOR_BLACK
# # TEST_COLOR = COLOR_LIGHTBLUE
#
#
# # ================== 面积过滤 ==================
# MIN_BLOB_AREA = 300
# MAX_BLOB_AREA = 60000
#
# # ================== 初始化 ==================
# cam = camera.Camera(IMG_W, IMG_H)
# disp = display.Display()
#
# # =============== 辅助函数 ===============
# # 显示颜色字符串  将颜色编号转换成可读的字符串名称，用于串口打印和屏幕显示
# def color_name(color):
#     if color == COLOR_RED:
#         return "RED"
#     elif color == COLOR_YELLOW:
#         return "YELLOW"
#     elif color == COLOR_BLUE:
#         return "BLUE"
#     elif color == COLOR_GREEN:
#         return "GREEN"
#     elif color == COLOR_BLACK:
#         return "BLACK"
#     elif color == COLOR_LIGHTBLUE:
#         return "LIGHTBLUE"
#     else:
#         return "NONE"
#
# # 兼容cv库里的 cv2.findContours 查找轮廓
# def cv_find_contours(mask):
#     result = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#
#     if len(result) == 2:
#         contours, hierarchy = result
#     else:
#         _, contours, hierarchy = result
#     return contours
#
# # 在图像上画调试信息   (屏幕中心十字、最大物料框、物料中心点、屏幕中心到物料中心连线、坐标和面积信息)
# def draw_debug(cv_img, ok, target_color, cx, cy, dx, dy, area, rect, method, mask=None):
#     # 画屏幕中心点 (白色十字)
#     cv2.drawMarker(cv_img,(CENTER_X, CENTER_Y),(255, 255, 255),
#         markerType=cv2.MARKER_CROSS,thickness=1)
#
#     # 左上角显示当前模式METHOD和目标颜色
#     info1 = "METHOD:{}  COLOR:{}".format(method, color_name(target_color))
#     cv2.putText(cv_img, info1, (5, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
#
#     if ok:
#         x, y, w, h = rect
#
#         # 最大物料外接框
#         cv2.rectangle(cv_img, (x, y), (x + w, y + h), (0, 255, 0), 2)
#
#         # 物料中心点
#         cv2.circle(cv_img, (cx, cy), 5, (0, 0, 255), -1)
#
#         # 屏幕中心到物料中心连线
#         cv2.line(cv_img, (CENTER_X, CENTER_Y), (cx, cy), (255, 0, 0), 1)
#
#         info2 = "cx:{} cy:{} dx:{} dy:{}".format(cx, cy, dx, dy)
#         info3 = "area:{}".format(int(area))
#
#         cv2.putText(cv_img, info2, (5, 38), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)
#         cv2.putText(cv_img, info3, (5, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)
#
#     else:
#         cv2.putText(cv_img, "NO TARGET", (5, 42), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)
#
#     return cv_img
#
# # 找最大色块  返回: ok, dx, dy, debug_img
# def find_blob_from_mask(cv_img, mask, target_color, method):
#     # 形态学处理
#     kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
#
#     # 开闭运算  找轮廓
#     mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
#     mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
#
#     contours = cv_find_contours(mask)
#
#     max_area = 0
#     best_contour = None
#
#     # 遍历轮廓 找最大轮廓
#     for c in contours:
#         area = cv2.contourArea(c)
#
#         # 面积限制
#         if (area < MIN_BLOB_AREA) or (area > MAX_BLOB_AREA):
#             continue
#
#         if area > max_area:
#             max_area = area
#             best_contour = c
#
#     if best_contour is None:
#         draw_debug(cv_img, 0, target_color, 0, 0, 0, 0, 0, (0, 0, 0, 0), method)
#         return 0, 0, 0, image.cv2image(cv_img)
#
#     M = cv2.moments(best_contour)
#
#     if M["m00"] == 0:
#         draw_debug(cv_img, 0, target_color, 0, 0, 0, 0, 0, (0, 0, 0, 0), method)
#         return 0, 0, 0, image.cv2image(cv_img)
#
#     cx = int(M["m10"] / M["m00"])
#     cy = int(M["m01"] / M["m00"])
#
#     dx = cx - CENTER_X
#     dy = cy - CENTER_Y
#
#     rect = cv2.boundingRect(best_contour)
#
#     # 画最大轮廓
#     cv2.drawContours(cv_img, [best_contour], -1, (0, 255, 0), 2)
#
#     draw_debug(cv_img, 1, target_color, cx, cy, dx, dy, max_area, rect, method)
#
#     return 1, dx, dy, image.cv2image(cv_img)
#
#
# def find_blobs_hsv(img, target_color):
#     """
#     HSV 版本物料识别。
#     """
#
#     cv_img = image.image2cv(img, ensure_bgr=True, copy=True)
#     hsv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
#
#     hsv_ranges = {
#         # 红色需要两段
#         COLOR_RED: [
#             (np.array([0, 80, 50]), np.array([10, 255, 255])),
#             (np.array([160, 80, 50]), np.array([180, 255, 255])),
#         ],
#
#         COLOR_YELLOW: [
#             (np.array([18, 80, 60]), np.array([35, 255, 255])),
#         ],
#
#         COLOR_BLUE: [
#             (np.array([100, 70, 40]), np.array([130, 255, 255])),
#         ],
#
#         COLOR_GREEN: [
#             (np.array([35, 50, 40]), np.array([85, 255, 255])),
#         ],
#
#         COLOR_BLACK: [
#             (np.array([0, 0, 0]), np.array([180, 255, 60])),
#         ],
#
#         COLOR_LIGHTBLUE: [
#             (np.array([85, 40, 60]), np.array([105, 255, 255])),
#         ],
#     }
#
#     if target_color not in hsv_ranges:
#         draw_debug(cv_img, 0, target_color, 0, 0, 0, 0, 0, (0, 0, 0, 0), "HSV")
#         return 0, 0, 0, image.cv2image(cv_img)
#
#     mask = None
#
#     for lower, upper in hsv_ranges[target_color]:
#         temp_mask = cv2.inRange(hsv_img, lower, upper)
#
#         if mask is None:
#             mask = temp_mask
#         else:
#             mask = cv2.bitwise_or(mask, temp_mask)
#
#     return find_blob_from_mask(cv_img, mask, target_color, "HSV")
#
#
# def find_blobs_lab(img, target_color):
#     """
#     LAB 版本物料识别。
#     """
#
#     cv_img = image.image2cv(img, ensure_bgr=True, copy=True)
#     lab_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2LAB)
#
#     lab_ranges = {
#         COLOR_RED: [
#             (np.array([20, 145, 130]), np.array([255, 210, 210])),
#         ],
#
#         COLOR_YELLOW: [
#             (np.array([80, 115, 145]), np.array([255, 160, 230])),
#         ],
#
#         COLOR_BLUE: [
#             (np.array([20, 110, 60]), np.array([220, 170, 130])),
#         ],
#
#         COLOR_GREEN: [
#             (np.array([20, 60, 80]), np.array([220, 125, 170])),
#         ],
#
#         COLOR_BLACK: [
#             (np.array([0, 0, 0]), np.array([70, 255, 255])),
#         ],
#
#         COLOR_LIGHTBLUE: [
#             (np.array([80, 90, 70]), np.array([255, 150, 135])),
#         ],
#     }
#
#     if target_color not in lab_ranges:
#         draw_debug(cv_img, 0, target_color, 0, 0, 0, 0, 0, (0, 0, 0, 0), "LAB")
#         return 0, 0, 0, image.cv2image(cv_img)
#
#     mask = None
#
#     for lower, upper in lab_ranges[target_color]:
#         lower = lower.astype(np.uint8)
#         upper = upper.astype(np.uint8)
#
#         temp_mask = cv2.inRange(lab_img, lower, upper)
#
#         if mask is None:
#             mask = temp_mask
#         else:
#             mask = cv2.bitwise_or(mask, temp_mask)
#
#     return find_blob_from_mask(cv_img, mask, target_color, "LAB")
#
#
# def find_blobs_test(img, target_color):
#     if TEST_METHOD == "HSV":
#         return find_blobs_hsv(img, target_color)
#     elif TEST_METHOD == "LAB":
#         return find_blobs_lab(img, target_color)
#     else:
#         return find_blobs_hsv(img, target_color)
#
#
# while not app.need_exit():
#     img = cam.read()
#
#     ok, dx, dy, debug_img = find_blobs_test(img, TEST_COLOR)
#
#     disp.show(debug_img)
#
#     # 串口监视器里也打印一下，方便看坐标稳定性
#     print("method:", TEST_METHOD,
#           "color:", color_name(TEST_COLOR),
#           "ok:", ok,
#           "dx:", dx,
#           "dy:", dy)
#
#     time.sleep_ms(30)

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
