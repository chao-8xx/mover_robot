# OpenMV 扫码
import sensor
import image
import time
import re

# 是否开启
flag = [1]

# 摄像头初始化
sensor.reset()
#sensor.set_pixformat(sensor.RGB565)     # 彩色图
sensor.set_framesize(sensor.QVGA)       # 分辨率 320*240
sensor.set_pixformat(sensor.GRAYSCALE)  # 灰度图 省内存
sensor.skip_frames(time=2000)           # 跳过前两秒内容 等待摄像头曝光、白平衡等参数稳定
sensor.set_auto_gain(True)              # 自动增益 看情况开不开 对灰度图影响不大
sensor.set_auto_whitebal(True)          # 白平衡 同上

clock = time.clock()
last_payload = None             # 上一次二维码内容

while True:
    clock.tick()

    if flag[0] == 1:

        # 拍摄图像并查找其中的二维码
        img = sensor.snapshot()
        qrcodes = img.find_qrcodes()

        for qr in qrcodes:
            corners = qr.corners()

            # 找二维码四角点并画红框
            for i in range(4):
                x1, y1 = corners[i]
                x2, y2 = corners[(i + 1) % 4]
                img.draw_line(x1, y1, x2, y2, color=255, thickness=2)
                # img.draw_line(x1, y1, x2, y2, color=(255, 0, 0), thickness=2)

            # # 显示二维码解析的字符串内容
            payload = qr.payload()
            # img.draw_string(qr.x(), max(0, qr.y() - 15), payload, color=255, scale=1)

            # 二维码内容变化时处理一次 减少发送次数 省资源
            if payload != last_payload:
                # 串口打印内容 帧率
                print("QR:", payload)
                print("fps:", clock.fps())

                last_payload = payload
    else:
        # 不扫码 休眠
        time.sleep_ms(20)
