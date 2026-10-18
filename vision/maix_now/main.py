# 校赛后独立入口：B2单色LAB/可裁剪与屏蔽 / B3纯圆心 / B4单色码垛。
# 参数只在vision_config.py或触屏菜单修改；没有第二套旧阈值。
# 协议不变：AA 55 CMD 06 FRAME_ID RAW_OK dxL dxH dyL dyH SUM。

from time import perf_counter
from maix import camera, display, app, time, image, touchscreen, uart, pinmap, gpio, err
import vision_config as config_module
import vision_blob as blob_module
import vision_tasks as tasks_module
import vision_ui as ui_module
import vision_runtime as runtime_module

from vision_config import COLOR_NUMBERS, IMAGE_WIDTH, IMAGE_HEIGHT, VisionConfigStore
from vision_blob import make_preview, expand_roi_mask
from vision_ring import make_ring_preview
from vision_tasks import (
    TASK_NONE, TASK_MATERIAL, TASK_RING, TASK_STACK, TASK_LABELS, TaskManager,
)
from vision_ui import TouchInput, VisionUI
from vision_runtime import B2_SEND_TIME_MS, SEND_TIME_MS, B2DebugLog, ResultSender, PerfStats, pack_xy_frame, send_interval_ms

BUILD = "NOW_B2_ROI_EXCLUDE_V11"
CMD_NONE, CMD_BLOB, CMD_RING, CMD_ONUP = 0xB0, 0xB2, 0xB3, 0xB4
COLOR_NONE, SUM_2 = 0xC0, 0xD2
COLOR_CODE_TO_KEY = {0xC0 + number: key for key, number in COLOR_NUMBERS.items()}
MAX_RX_DATA_LEN = 32
DISPLAY_TIME_MS = 100
SERIAL_DEVICE, SERIAL_BAUD = "/dev/serial0", 115200
UART_TX_PIN, UART_TX_GPIO, UART_TX_FUNCTION = "A16", "GPIOA16", "UART0_TX"

uart_tx_gpio = None
uart_tx_owned = False
serial = None
mode, target = CMD_NONE, COLOR_NONE
uart_control_active = False
rx_buf = bytearray()
control_revision = 0


def now_ms():
    return perf_counter() * 1000.0


def check_b2_modules():
    """检查实际导入的依赖，避免新抓取界面/检测混用会压掉OK=2的旧模块。"""
    modules = (("vision_blob", blob_module, "B2_PIPELINE_BUILD", "B2_EXCLUDE_V4"),
               ("vision_tasks", tasks_module, "B2_TASK_BUILD", "B2_EXCLUDE_V4"),
               ("vision_ui", ui_module, "B2_UI_BUILD", "B2_EXCLUDE_UI_V3"),
               ("vision_runtime", runtime_module, "B2_RUNTIME_BUILD", "B2_OK2_PIXELS_V3"))
    mismatch = []
    for name, module, attr, expected in modules:
        actual = getattr(module, attr, "OLD")
        print("[VISION B2 MODULE]", name, getattr(module, "__file__", "?"), actual)
        if actual != expected:
            mismatch.append(name)
    colors = tuple(getattr(config_module, "COLOR_ORDER", ()))
    schema = getattr(config_module, "SCHEMA_VERSION", 0)
    print("[VISION B2 MODULE] vision_config", getattr(config_module, "__file__", "?"), colors, "schema=", schema)
    if colors != ("red", "yellow", "blue", "green", "black", "light_blue") or schema != 11:
        mismatch.append("vision_config")
    if mismatch:
        raise RuntimeError("B2 module mismatch: " + ",".join(mismatch) +
                           "; upload ALL app.yaml Python files together and restart")


def uart_tx_release():
    """共享RX总线时，空闲Maix TX切为GPIO输入高阻；不能重复申请GPIO。"""
    global uart_tx_gpio, uart_tx_owned
    if uart_tx_owned:
        time.sleep_ms(3)  # 115200下11字节约1ms，留出发送完成余量。
    err.check_raise(pinmap.set_pin_function(UART_TX_PIN, UART_TX_GPIO),
                    "release A16 failed")
    if uart_tx_gpio is None:
        uart_tx_gpio = gpio.GPIO(UART_TX_GPIO, gpio.Mode.IN)
    uart_tx_owned = False


def uart_tx_acquire():
    global uart_tx_gpio, uart_tx_owned
    if uart_tx_owned:
        return
    uart_tx_gpio = None
    err.check_raise(pinmap.set_pin_function(UART_TX_PIN, UART_TX_FUNCTION),
                    "acquire A16 failed")
    time.sleep_ms(1)
    uart_tx_owned = True


def checksum(cmd, data):
    return (cmd + len(data) + sum(data)) & 0xFF


def clear_color_target(value):
    if 1 <= value <= 6:
        return 0xC0 + value
    return value if value in COLOR_CODE_TO_KEY else COLOR_NONE


def handle_cmd(cmd, data):
    """B3只接受2/D2。纯几何版本不能承诺识别数字1或3。"""
    global mode, target, control_revision
    if cmd == CMD_NONE and len(data) == 0:
        next_mode, next_target = CMD_NONE, COLOR_NONE
    elif cmd in (CMD_BLOB, CMD_ONUP) and len(data) == 1:
        next_target = clear_color_target(data[0])
        if next_target == COLOR_NONE:
            return
        next_mode = cmd
    elif cmd == CMD_RING and len(data) == 1 and data[0] in (2, SUM_2):
        next_mode, next_target = CMD_RING, SUM_2
    else:
        return
    # 所有权切换失败就不进入新模式；停止也会立即禁止后续坐标发送。
    try:
        if next_mode == CMD_NONE:
            mode, target = next_mode, next_target
            control_revision += 1
            uart_tx_release()
        else:
            uart_tx_acquire()
            if (next_mode, next_target) == (mode, target):
                return
            mode, target = next_mode, next_target
            control_revision += 1
        print("[VISION CMD] mode=0x%02X target=0x%02X TX=%s" %
              (mode, target, "UART0" if uart_tx_owned else "HI-Z"))
    except Exception as error:
        print("[VISION BUS] switch failed:", error)


def rx_handle():
    global uart_control_active
    # 正式模式开关：注释主循环标记处的一行即可本地触屏调试。
    uart_control_active = True
    if serial is None:
        return
    buf = serial.read()
    if buf:
        rx_buf.extend(buf)
    while len(rx_buf) >= 5:
        if rx_buf[0:2] != b"\xAA\x55":
            del rx_buf[0]
            continue
        cmd, length = rx_buf[2], rx_buf[3]
        if length > MAX_RX_DATA_LEN:
            del rx_buf[0]
            continue
        frame_len = length + 5
        if len(rx_buf) < frame_len:
            break
        data = rx_buf[4:4 + length]
        if rx_buf[4 + length] != checksum(cmd, data):
            del rx_buf[0]
            continue
        del rx_buf[:frame_len]
        handle_cmd(cmd, data)


def active_task(ui):
    if uart_control_active:
        return mode, target
    task_id = ui.current_task()
    if task_id == TASK_RING:
        return CMD_RING, SUM_2
    if task_id in (TASK_MATERIAL, TASK_STACK):
        code = 0xC0 + COLOR_NUMBERS[ui.detection_config()["target_color"]]
        return (CMD_BLOB if task_id == TASK_MATERIAL else CMD_ONUP), code
    return CMD_NONE, COLOR_NONE


def mode_to_task_id(value):
    return {CMD_BLOB: TASK_MATERIAL, CMD_RING: TASK_RING,
            CMD_ONUP: TASK_STACK}.get(value, TASK_NONE)


def open_camera():
    """优先原生BGR以减少整图转换；fps=60只是请求，实际帧率由日志验证。"""
    bgr_format = getattr(image.Format, "FMT_BGR888", None)
    if bgr_format is not None:
        try:
            value = camera.Camera(IMAGE_WIDTH, IMAGE_HEIGHT, bgr_format, fps=60)
            print("[VISION CAMERA] BGR888 320x240 requested FPS=60")
            return value, True
        except Exception as error:
            print("[VISION CAMERA] FPS=60 unavailable:", error)
        try:
            value = camera.Camera(IMAGE_WIDTH, IMAGE_HEIGHT, bgr_format)
            print("[VISION CAMERA] BGR888 default FPS (verify actual rate)")
            return value, True
        except Exception as error:
            print("[VISION CAMERA] native BGR unavailable:", error)
    print("[VISION CAMERA] legacy format/default FPS (verify actual rate)")
    return camera.Camera(IMAGE_WIDTH, IMAGE_HEIGHT), False


def send_result(cmd, tx):
    if serial is None or not uart_tx_owned:
        return False
    frame = pack_xy_frame(cmd, tx["frame_id"], tx["raw_ok"], tx["dx"], tx["dy"])
    try:
        written = serial.write(frame)
        if written is not None and written != len(frame):
            print("[VISION UART] incomplete write:", written)
            return False
        return True
    except Exception as error:
        print("[VISION UART] write failed:", error)
        return False


def run():
    global serial
    # 尽量先释放TX；Python启动前的系统UART日志不受此程序控制。
    uart_tx_release()
    cam, native_bgr = open_camera()
    disp = display.Display()
    ts = touchscreen.TouchScreen()
    try:
        screen_w, screen_h = int(disp.width()), int(disp.height())
    except Exception:
        screen_w, screen_h = 480, 640
    try:
        data_dir = app.get_app_data_path()
    except Exception:
        data_dir = "./data"
    # 新app id和新文件名隔离参数，避免覆盖校赛版本的现场配置。
    store = VisionConfigStore(data_dir, filename="vision_now_config.json")
    config, status = store.load()
    ui = VisionUI(screen_w, screen_h, config, status)
    touch = TouchInput(ts)
    tasks = TaskManager()
    sender = ResultSender()
    b2_debug = B2DebugLog()
    stats = PerfStats(now_ms())
    try:
        serial = uart.UART(SERIAL_DEVICE, SERIAL_BAUD)
        uart_tx_release()
        serial_status = "ready, TX Hi-Z"
    except Exception as error:
        serial = None
        serial_status = "disabled: " + str(error)
    print("[VISION BUILD]", BUILD)
    print("[VISION CONFIG]", status, store.path)
    print("[VISION UART]", SERIAL_DEVICE, SERIAL_BAUD, serial_status)
    print("[VISION UI] display: {}x{}".format(screen_w, screen_h))
    print("[VISION RATE] B2={}ms ({:.1f}Hz); B3/B4={}ms ({:.1f}Hz); verify FPS/TXHZ separately".format(
        B2_SEND_TIME_MS, 1000.0 / B2_SEND_TIME_MS, SEND_TIME_MS, 1000.0 / SEND_TIME_MS))
    print("[VISION RING] geometric center only; F407 must approach slot 2 first")

    previous_signature = None
    previous_revision = -1
    last_display = 0
    fps = 0.0
    tx_hz = display_fps = 0.0
    # copy=False会忽略ensure_bgr，只有原生BGR才允许借用内存。
    # 回退到默认RGB相机时必须copy=True，否则红蓝会互换。
    zero_copy = native_bgr
    try:
        while not app.need_exit():
            loop_start = now_ms()
            # 本地触屏调试只注释下一行；下面第二次接收受此开关控制。
            # rx_handle()

            event = touch.poll()
            if event is not None:
                action = ui.handle_touch(*event)
                if action is not None and action.get("type") == "save":
                    value, save_status = store.save(action["config"])
                    if value is None:
                        ui.save_failed(save_status)
                    elif action.get("stay", False):
                        ui.set_saved_config(value, save_status)
                    else:
                        ui.save_succeeded(value, save_status)
                    tasks.reset_current()
                    sender.reset(now_ms())
                    b2_debug.reset()

            active_mode, active_target = active_task(ui)
            task_id = mode_to_task_id(active_mode)
            if uart_control_active:
                if task_id in (TASK_MATERIAL, TASK_STACK):
                    ui.set_runtime_target(COLOR_CODE_TO_KEY[active_target])
                ui.set_external_task(task_id)
            run_mode = task_id != TASK_NONE and ui.is_run_mode()
            signature = (task_id, active_target, run_mode, control_revision)
            changed = signature != previous_signature
            if changed and task_id == TASK_MATERIAL:
                check_b2_modules()
                geometry_config = ui.detection_config()
                print("[VISION B2 GEOMETRY] search=", geometry_config["roi"],
                      "exclude=", geometry_config["exclude_rois"],
                      "reference=", geometry_config["center"],
                      "grasp=", geometry_config["grasp"]["roi"])
            tasks.switch(task_id, active_target)
            if changed:
                tasks.reset_current()
                # 切任务先选择本任务周期，再清空旧调度，不让B2的28ms带到B3/B4。
                sender.interval_ms = send_interval_ms(task_id)
                sender.reset(now_ms())
                b2_debug.reset()
                stats.reset(now_ms())
                fps = tx_hz = display_fps = 0.0
                previous_signature = signature
                print("[VISION TASK]", TASK_LABELS[task_id],
                      "target=0x%02X" % active_target, "RUN" if run_mode else "UI")

            display_due = (changed or ui.ui_revision != previous_revision or
                           (not run_mode and now_ms() - last_display >= DISPLAY_TIME_MS))
            display_ms = 0.0
            if run_mode and display_due:
                # 慢速LCD刷新放在采集之前，而且RUN仅在页面改变时显示一次。
                display_start = now_ms()
                disp.show(ui.render(None, {}, fps, 0.0))
                display_ms += now_ms() - display_start
                previous_revision = ui.ui_revision
                last_display = now_ms()
                if changed and hasattr(cam, "clear_buff"):
                    # 换任务/首次RUN页面绘制很慢，丢掉这段时间的排队旧图。
                    cam.clear_buff()

            capture_start = now_ms()
            camera_image = cam.read()
            # 相机对象在本轮处理结束前始终保留，不跨线程借用这块内存。
            try:
                cv_bgr = image.image2cv(camera_image, ensure_bgr=True, copy=not zero_copy)
            except TypeError:
                if not zero_copy:
                    raise
                zero_copy = False
                print("[VISION CAMERA] image2cv copy=False unsupported; using copy")
                cv_bgr = image.image2cv(camera_image, ensure_bgr=True, copy=True)
            capture_ms = now_ms() - capture_start
            config = ui.detection_config()
            detect_start = now_ms()
            frame = tasks.process(cv_bgr, config, run_mode,
                                  need_mask=display_due and not run_mode and
                                  (task_id == TASK_MATERIAL or ui.needs_mask_for_display()))
            result = frame["result"]
            detect_ms = now_ms() - detect_start

            # 检测过程中到达的B0/切任务指令优先处理，丢弃旧任务这一帧。
            revision_before_rx = control_revision
            if uart_control_active:
                rx_handle()
            if revision_before_rx != control_revision:
                continue
            sent = None
            if run_mode and uart_control_active and uart_tx_owned:
                tx = sender.take(result, capture_start, now_ms(),
                                 require_track=task_id == TASK_RING,
                                 allow_grasp=task_id == TASK_MATERIAL)
                if tx is not None and send_result(active_mode, tx):
                    sent = tx

            if task_id == TASK_MATERIAL:
                debug_line = b2_debug.line(
                    result, config["grasp"], now_ms(), run_mode, sent,
                    tx_active=run_mode and uart_control_active and uart_tx_owned,
                    fps=fps, tx_hz=tx_hz)
                if debug_line is not None:
                    print(debug_line)

            if display_due and not run_mode:
                display_start = now_ms()
                preview_config = frame["config"]
                if task_id in (TASK_MATERIAL, TASK_STACK):
                    full_mask = None
                    if frame["mask_roi"] is not None and ui.current_view_mode() in ("MASK", "OVERLAY"):
                        full_mask = expand_roi_mask(
                            frame["mask_roi"], cv_bgr.shape, frame["roi_rect"])
                    preview = make_preview(cv_bgr, full_mask, result,
                                           preview_config, ui.current_view_mode())
                elif task_id == TASK_RING:
                    preview = make_ring_preview(cv_bgr, frame["mask_roi"], result,
                                                config["ring"], ui.current_view_mode())
                else:
                    preview = cv_bgr
                disp.show(ui.render(preview, result, fps, display_fps))
                display_ms += now_ms() - display_start
                previous_revision = ui.ui_revision
                last_display = now_ms()

            if task_id == TASK_NONE:
                time.sleep_ms(5)
            loop_end = now_ms()
            stats.add(loop_end - loop_start, capture_ms, detect_ms, display_ms, result, sent)
            if loop_end - stats.start_ms >= 1000:
                fps = stats.frames * 1000.0 / (loop_end - stats.start_ms)
                tx_hz = stats.tx_count * 1000.0 / (loop_end - stats.start_ms)
                display_fps = stats.display_count * 1000.0 / (loop_end - stats.start_ms)
                if task_id != TASK_NONE:
                    print(stats.line(loop_end, task_id, run_mode, result))
                stats.reset(loop_end)
    finally:
        uart_tx_release()
        if hasattr(cam, "close"):
            cam.close()


if __name__ == "__main__":
    run()
