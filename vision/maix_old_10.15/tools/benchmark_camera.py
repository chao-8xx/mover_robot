"""可选真机采集基线：与main.py放同目录后运行，不显示画面、不发送结果。"""
from time import perf_counter
from maix import app, image
from main import open_camera, uart_tx_release

uart_tx_release()
cam, native_bgr = open_camera()
copy = not native_bgr
try:
    cam.skip_frames(10)
    started = perf_counter()
    window = started
    frames = 0
    read_total = convert_total = 0.0
    while not app.need_exit() and perf_counter() - started < 15:
        tick = perf_counter()
        frame = cam.read()
        read_done = perf_counter()
        try:
            bgr = image.image2cv(frame, ensure_bgr=True, copy=copy)
        except TypeError:
            copy = True
            bgr = image.image2cv(frame, ensure_bgr=True, copy=True)
        finished = perf_counter()
        read_total += (read_done - tick) * 1000
        convert_total += (finished - read_done) * 1000
        frames += 1
        if finished - window >= 1:
            print("[CAM BASELINE] READ_FPS:{:.1f} READ:{:.1f}ms CV:{:.1f}ms COPY:{}".format(
                frames / (finished - window), read_total / frames, convert_total / frames, copy))
            window = finished
            frames = 0
            read_total = convert_total = 0.0
finally:
    cam.close()
    uart_tx_release()
