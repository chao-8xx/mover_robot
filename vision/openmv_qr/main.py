# OpenMV 扫码
import sensor
import image
import time
import re
from pyb import UART

# 摄像头初始化
sensor.reset()
#sensor.set_pixformat(sensor.RGB565)     # 彩色图
sensor.set_framesize(sensor.QVGA)       # 分辨率 320*240
sensor.set_pixformat(sensor.GRAYSCALE)  # 灰度图 省内存
sensor.skip_frames(time=2000)           # 跳过前两秒内容 等待摄像头曝光、白平衡等参数稳定
sensor.set_auto_gain(True)              # 自动增益 看情况开不开 对灰度图影响不大
sensor.set_auto_whitebal(True)          # 白平衡 同上

uart = UART(3, 115200)                  # 初始化串口3

clock = time.clock()


flag = [0]                   # 开启命令
last_payload = None          # 上一次二维码内容

# 数据包校验和
def checksum(cmd,length,data):
    sum = cmd + length
    for byte in data:
        sum += byte
    return sum & 0xFF

# 构建并发送数据包
def send_qr(payload):

    data = payload.encode()  # 二维码内容转为二进制字节
    length = len(data)       # 数据长度
    cmd = 0x18               # 命令字

    frame = bytearray([0xAA, 0x55, cmd, length])  # 帧头 + 命令字 + 长度
    frame.extend(data)  # 添加数据
    frame.append(checksum(cmd, length, data))  # 添加校验和

    uart.write(frame)  # 发送数据包

# 处理串口接收数据
def rx_handle():
    global last_payload

    # 检查串口数据，至少需要6字节（帧头2字节 + 命令字1字节 + 长度1字节 + 数据至少1字节 + 校验和1字节）
    if uart.any() >= 6:
        buf = uart.read()
        if not buf:
            return

        # 解析数据
        for i in range(len(buf) - 5):
            if buf[i] == 0xAA and buf[i+1] == 0x55:
                cmd = buf[i+2]
                length = buf[i+3]
                data = buf[i+4:i+4+length]
                recv_checksum = buf[i+4+length]

                # 校验数据
                if recv_checksum == checksum(cmd, length, data):
                    # 数据包正确格式: 命令字0x01 长度1 数据为1 开启扫码
                    if cmd == 0x01 and length == 1 and data[0] == 0:
                        flag[0] = 0
                        last_payload = None
                        print("QR scan stop")
                    elif cmd == 0x01 and length == 1 and data[0] == 1:
                        flag[0] = 1
                        print("扫码开启")
                        last_payload = None  # 重置上一次内容
                    return

while True:
    clock.tick()
    rx_handle()

    if flag[0] == 1:

        # 看图像并查找其中的二维码
        img = sensor.snapshot()
        qrcodes = img.find_qrcodes()

        for qr in qrcodes:

            # 调试使用
            # corners = qr.corners()

            # # 找二维码四角点并画红框
            # for i in range(4):
            #     x1, y1 = corners[i]
            #     x2, y2 = corners[(i + 1) % 4]
            #     img.draw_line(x1, y1, x2, y2, color=255, thickness=2)
            #     # img.draw_line(x1, y1, x2, y2, color=(255, 0, 0), thickness=2)

            # # 显示二维码解析的字符串内容
            payload = qr.payload()
            # img.draw_string(qr.x(), max(0, qr.y() - 15), payload, color=255, scale=1)

            # 只在二维码内容变化时处理一次 减少发送次数 省资源
            if payload != last_payload:
                send_qr(payload)
                # 串口打印内容 帧率
                print("QR:", payload)
                print("fps:", clock.fps())

                last_payload = payload
                flag[0] = 0
                break
    else:
        # 不扫码 休眠
        time.sleep_ms(20)
