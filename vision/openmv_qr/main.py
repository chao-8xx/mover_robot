# OpenMV 只负责赛题任务码扫码
# 控制帧：AA 55 B1 01 00/01 SUM
# 结果帧：AA 55 BB 0F 15字节ASCII任务码 SUM，总长度固定为20字节

import sensor
import time
from pyb import UART


# ============================== 通信常量 ==============================
HEAD_1 = 0xAA               # 帧头固定为 AA 55
HEAD_2 = 0x55
CMD_SCAN = 0xB1      # 控制CMD, 启动后扫描一次有效任务码后自动停止，需再次发送启动命令才能继续扫描
CMD_QR = 0xBB        # 结果CMD, 有效任务码发送一次后自动停止扫描，需再次发送启动命令才能继续扫描

QR_PAYLOAD_LEN = 15         # 任务码固定15字节
RX_MAX_DATA_LEN = 16        # 接收的最大数据长度


# ============================== 摄像头与串口 ==============================
sensor.reset()                           # 初始化
#sensor.set_pixformat(sensor.RGB565)     # 彩色图
sensor.set_pixformat(sensor.GRAYSCALE)   # 灰度图，省内存
sensor.set_framesize(sensor.QVGA)        # 分辨率 320x240
sensor.skip_frames(time=2000)            # 跳过前两秒内容 等待摄像头曝光、白平衡等参数稳定
sensor.set_auto_gain(True)               # 自动增益 看情况开不开 对灰度图影响不大
sensor.set_auto_whitebal(True)           # 白平衡 同上


uart = UART(3, 115200)          # 初始化串口3
clock = time.clock()            # 注: P4接RX P5接TX 注意是接 直接接F407即可

rx_buffer = bytearray()         # 持久化接收缓冲区，拆帧时可能会保留半帧数据
scan_flag = False               # 扫码使能标志，收到控制帧后置位，发送一次有效任务码后清零
send_flag = False  # 当前启动周期是否已发送过有效任务码，收到控制帧后清零
last_payload = None     # 上一次无效任务码，避免重复打印


# 数据包校验和
def checksum(cmd, length, data):
    all = int(cmd) + int(length)
    for value in data:
        all += int(value)
    return all & 0xFF


# 15字符任务码校验  例如 156+123+516+231
# 任务码长度固定15字节，且第4、8、12位必须为加号，其余位置为数字
def check_TaskCode(payload):    # payload: 任务码字符串

    if payload is None or len(payload) != QR_PAYLOAD_LEN:
        return False
    if payload[3] != "+" or payload[7] != "+" or payload[11] != "+":
        return False

    # 第1、3组是物料颜色编号1~6；第2、4组是放置位置编号1~3
    for index in (0, 1, 2, 8, 9, 10):
        if payload[index] < "1" or payload[index] > "6":
            return False
    for index in (4, 5, 6, 12, 13, 14):
        if payload[index] < "1" or payload[index] > "3":
            return False
    return True

# 构造固定20字节结果帧
def build_qr_result_frame(payload):

    data = payload.encode() # 二维码内容转为二进制字节
    frame = bytearray([HEAD_1, HEAD_2, CMD_QR, QR_PAYLOAD_LEN])  # 帧头 cmd len
    frame.extend(data)  # 添加数据
    frame.append(checksum(CMD_QR, QR_PAYLOAD_LEN, data)) # 添加校验和
    return frame

# 发送一次有效任务码
def send_qr_once(payload):

    frame = build_qr_result_frame(payload)
    uart.write(frame)  # 发送数据包

    # 打印发送的内容和帧的十六进制表示，方便调试
    print("[OPENMV QR TX]", payload)
    # print("[OPENMV QR FRAME]", " ".join("%02X" % value for value in frame))

# 处理控制帧   B1/LEN=1
def handle_control_frame(cmd, data):

    global scan_flag, send_flag, last_payload

    # 只处理 B1/LEN=1 控制帧，其他帧忽略
    if cmd != CMD_SCAN or len(data) != 1:
        return

    control = data[0]

    # 0 停止扫描，1 启动扫描
    if control == 0:
        scan_flag = False
        send_flag = False
        last_payload = None
        print("[OPENMV QR] 停止扫描任务码")
    elif control == 1:
        if send_flag:
            print("[OPENMV QR] 已发送过有效任务码，需再次发送启动命令才能继续扫描")
        elif not scan_flag:
            scan_flag = True
            last_payload = None
            print("[OPENMV QR] 开始扫描任务码")

# 持续拆帧，可处理半帧、粘包、噪声和错误校验
def rx_buffer_handle():

    global rx_buffer

    # 循环处理缓冲区
    while True:
        while len(rx_buffer) >= 2:
            # 帧头同步，丢弃噪声数据
            if rx_buffer[0] == HEAD_1 and rx_buffer[1] == HEAD_2:
                break
            # 使用切片重建缓冲区
            rx_buffer = rx_buffer[1:]

        if len(rx_buffer) < 4:  # 帧头2字节+CMD+长度  为4
            return

        cmd = rx_buffer[2]
        length = rx_buffer[3]
        if length > RX_MAX_DATA_LEN:    # 长度大于16字节 认为字节明显损坏  此时只丢一个字节，让后面的 AA 55 仍有机会重新同步

            rx_buffer = rx_buffer[1:]
            continue

        frame_len = 5 + length      # 完整帧长度 = 帧头2字节 + CMD + 长度 + 数据 + 校验和
        if len(rx_buffer) < frame_len:
            return

        data = rx_buffer[4:4 + length]
        received_sum = rx_buffer[4 + length]
        # 丢弃已经解析完成的整帧，保留后续粘在一起的其他帧
        rx_buffer = rx_buffer[frame_len:]

        if received_sum != checksum(cmd, length, data): # 校验和错误
            print("[OPENMV RX] 校验和错误")
            continue

        handle_control_frame(cmd, data)

# 非阻塞读取当前所有字节，并交给持久化接收缓冲区
def poll_uart():

    available = uart.any()
    if available <= 0:
        return

    # 查询串口是否有可用字节
    chunk = uart.read(available)
    if chunk:
        rx_buffer.extend(chunk)
        rx_buffer_handle()

# 打印版本号和初始状态，确认 IDE 已运行 bytearray 兼容修复版。
print("[OPENMV BUILD] QR_BYTEARRAY_FIX_V2")
print("[OPENMV MODE] STOP")

while True:
    clock.tick()
    poll_uart()

    # 不扫码 休眠
    if not scan_flag:
        time.sleep_ms(20)
        continue

    # # 看图像并遍历查找其中的二维码
    img = sensor.snapshot()
    for qr in img.find_qrcodes():
        # 提取二维码的 payload 字符串并去除首尾空白（如换行符）
        payload = qr.payload().strip()

        if not check_TaskCode(payload):     # 校验任务码格式，若非法：若与上次无效内容不同则打印警告，记录无效内容，然后跳过该二维码
            if payload != last_payload:
                print("[OPENMV QR] 非法", payload)
                last_payload = payload
            continue

        send_qr_once(payload)
        send_flag = True
        scan_flag = False
        print("[OPENMV QR] 完成 fps:{:.1f}".format(clock.fps()))
        break
