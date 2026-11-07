"""纯Python发送调度和性能统计，可在电脑上测试，无Maix硬件依赖。"""

from vision_tasks import TASK_MATERIAL, safe_xy_result


SEND_TIME_MS = 25
B2_SEND_TIME_MS = 28
MAX_RESULT_AGE_MS = 50
B2_RUNTIME_BUILD = "B2_COLOR_DIAG_V4"


def send_interval_ms(task_id):
    """仅任务一改为28ms；圆环和码垛继续使用25ms。"""
    return B2_SEND_TIME_MS if task_id == TASK_MATERIAL else SEND_TIME_MS


class ResultSender:
    """每个新帧至多发送一次，错过截止时间不突发补发旧结果。"""

    def __init__(self, interval_ms=SEND_TIME_MS, max_age_ms=MAX_RESULT_AGE_MS):
        self.interval_ms = max(1, int(interval_ms))
        self.max_age_ms = max(1, int(max_age_ms))
        self.reset(0)

    def reset(self, now_ms):
        self.next_send_ms = now_ms
        self.frame_id = 0

    def take(self, result, capture_ms, now_ms, require_track=False, allow_grasp=False):
        self.frame_id = (self.frame_id + 1) & 0xFF
        if now_ms < self.next_send_ms:
            return None
        missed = int((now_ms - self.next_send_ms) // self.interval_ms)
        self.next_send_ms += (missed + 1) * self.interval_ms
        valid, dx, dy = safe_xy_result(result, require_track, allow_grasp)
        age_ms = max(0, now_ms - capture_ms)
        fresh = age_ms <= self.max_age_ms
        if not fresh:
            valid, dx, dy = 0, 0, 0
        return {"frame_id": self.frame_id, "raw_ok": valid,
                "dx": dx, "dy": dy, "age_ms": age_ms, "fresh": fresh}


def pack_xy_frame(cmd, frame_id, raw_ok, dx, dy):
    """AA55 CMD 06 FRAME_ID RAW_OK int16(dx) int16(dy) SUM。"""
    ok = 2 if cmd == 0xB2 and raw_ok == 2 else 1 if raw_ok in (1, 2) else 0
    if not ok:
        dx = dy = 0
    dx = max(-32768, min(32767, int(dx))) & 0xFFFF
    dy = max(-32768, min(32767, int(dy))) & 0xFFFF
    data = [int(frame_id) & 0xFF, ok,
            dx & 0xFF, dx >> 8, dy & 0xFF, dy >> 8]
    return bytes([0xAA, 0x55, cmd, 6] + data + [(cmd + 6 + sum(data)) & 0xFF])


def color_diagnostic_line(result, config, run_mode=False):
    """采样帧复用检测统计；像素数、候选面积和筛选范围均为实际处理图单位。"""
    info = result.get("diagnostic")
    if info is None:
        return None
    raw, kept, clean = (info[k] for k in ("raw_pixels", "kept_pixels", "clean_pixels"))
    reason = ("THRESHOLD_EMPTY" if not raw else "MASK_EMPTY" if not kept else
              "MORPH_EMPTY" if not clean else "AREA_REJECTED" if not info["eligible"] else
              "SELECTED" if result.get("found") else "NO_CENTER")
    top = []
    for area in info["top"]:
        state = "SMALL" if area < info["min_area"] else "LARGE" if area > info["max_area"] else "PASS"
        top.append("{:.1f}:{}".format(area, state))
    threshold = config["thresholds"]["LAB"][config["target_color"]]
    return ("[VISION B2 DIAG] {} TARGET={} WHY={} SIZE={}x{} "
            "MASK_PIX={}/{}/{} CUT={} TOP_AREA_PROC=[{}] LIMIT_PROC={}..{} "
            "CAND={} SMALL={} LARGE={} PASS={} LAB={}..{}").format(
        "RUN" if run_mode else "UI", config["target_color"], reason, *info["size"],
        raw, kept, clean, raw-kept, ",".join(top),
        info["min_area"], info["max_area"],
        info["contours"], info["small"], info["large"], info["eligible"],
        threshold["lower"], threshold["upper"])


class B2DebugLog:
    """只打印，不获取TX所有权；稳定状态1Hz，变化最多5Hz，避免逐帧打印。

    PREVIEW是本帧检测结果的模拟打包；TX仅采样已成功发送的同一帧。
    SEEN记录本日志窗口出现过的检测状态，防止短暂的2被限频完全隐藏。
    """

    def __init__(self):
        self.reset()

    def reset(self):
        self.last_ms = None
        self.last_key = None
        self.seen = 0
        self.preview_id = 0

    def line(self, result, grasp, now_ms, run_mode=False, sent=None, tx_active=False,
             fps=0.0, tx_hz=0.0):
        self.preview_id = (self.preview_id + 1) & 255
        det_ok, dx, dy = safe_xy_result(result, allow_grasp=True)
        self.seen |= 1 << det_ok
        # 没发出的帧不能标成TX；正式发送失败/未到发送时刻也不伪装成PREVIEW。
        if tx_active and sent is None:
            return None
        roi = grasp["roi"]
        cx, cy = int(result.get("cx", 0)), int(result.get("cy", 0))
        inside = (bool(result.get("found")) and
                  roi["x"] <= cx < roi["x"] + roi["w"] and
                  roi["y"] <= cy < roi["y"] + roi["h"])
        reason = ("NO_TARGET" if not det_ok else "CENTER_OUT" if not inside else
                  "ALIGNED" if det_ok == 2 else "LOW_COVER")
        tx = sent if sent is not None else {
            "frame_id": self.preview_id, "raw_ok": det_ok, "dx": dx, "dy": dy}
        if sent is not None and not sent["fresh"]:
            reason = "STALE"
        key = (tx["raw_ok"], det_ok, reason)
        changed = key != self.last_key or bool(self.seen & (self.seen - 1))
        if self.last_ms is not None:
            gap = now_ms - self.last_ms
            if gap < 200 or (not changed and gap < 1000):
                return None
        packet = pack_xy_frame(0xB2, tx["frame_id"], tx["raw_ok"], tx["dx"], tx["dy"])
        states = ",".join(str(i) for i in range(3) if self.seen & (1 << i))
        text = ("[VISION B2 {}] {} FPS={:.1f} TXHZ={:.1f} OK={} DET_OK={} WHY={} IN={} "
                "COV={:.1f}%/{}% PIX={}/{} C=({},{}) ROI=({},{},{},{}) D=({},{}) SEEN={} HEX={}").format(
            "TX" if sent is not None else "PREVIEW", "RUN" if run_mode else "UI",
            fps, tx_hz, packet[5], det_ok, reason, int(inside), result.get("coverage_ratio", 0.0) * 100,
            grasp["min_coverage_pct"], result.get("coverage_pixels", 0),
            result.get("coverage_total", 0), cx, cy,
            roi["x"], roi["y"], roi["w"], roi["h"], tx["dx"], tx["dy"], states,
            " ".join("{:02X}".format(value) for value in packet))
        self.last_ms, self.last_key, self.seen = now_ms, key, 0
        return text


class PerfStats:
    def __init__(self, now_ms):
        self.reset(now_ms)

    def reset(self, now_ms):
        self.start_ms = now_ms
        self.frames = 0
        self.tx_count = 0
        self.stale_count = 0
        self.loop_times = []
        self.max_age_ms = 0.0
        self.totals = {}
        self.display_count = 0

    def add(self, loop_ms, capture_ms, detect_ms, display_ms, result, tx):
        self.frames += 1
        self.loop_times.append(loop_ms)
        for key, value in (("CAP", capture_ms), ("DET", detect_ms), ("DSP", display_ms)):
            self.totals[key] = self.totals.get(key, 0.0) + value
        if display_ms > 0:
            self.display_count += 1
        for key in ("conversion_ms", "threshold_ms", "morph_ms", "blob_ms",
                    "mask_ms", "group_ms", "contour_ms", "filter_ms", "resize_ms", "coverage_ms", "ocr_ms"):
            self.totals[key] = self.totals.get(key, 0.0) + result.get(key, 0.0)
        if tx is not None:
            self.tx_count += 1
            self.max_age_ms = max(self.max_age_ms, tx["age_ms"])
            if not tx["fresh"]:
                self.stale_count += 1

    def line(self, now_ms, task_id, run_mode, result):
        elapsed = max(1, now_ms - self.start_ms)
        n = max(1, self.frames)
        ordered = sorted(self.loop_times)
        p95 = ordered[min(len(ordered) - 1, int(len(ordered) * 0.95))] if ordered else 0
        maximum = ordered[-1] if ordered else 0
        avg = lambda key: self.totals.get(key, 0.0) / n
        text = (
            "[VISION PERF] B{} {} FPS:{:.1f} TXHZ:{:.1f} "
            "CAP:{:.1f} DET:{:.1f} DSP:{:.1f} LOOP95:{:.1f} MAX:{:.1f} "
            "AGE_MAX:{:.1f} STALE:{}"
        ).format(task_id + 1, "RUN" if run_mode else "UI",
                 self.frames * 1000.0 / elapsed, self.tx_count * 1000.0 / elapsed,
                 avg("CAP"), avg("DET"), avg("DSP"), p95, maximum,
                 self.max_age_ms, self.stale_count)
        if task_id == 2:
            roi = result.get("process_roi", (0, 0, 0, 0))
            text += (" MASK:{:.1f} GRP:{:.1f} CTR:{:.1f} FLT:{:.1f} "
                     "ROI:{}x{} LAYER:{} CAND:{} LOCK:{} LOST:{} STATE:{} "
                     "OCR:{:.1f} DIG:{} OCR_N:{} WHY:{}").format(
                avg("mask_ms"), avg("group_ms"), avg("contour_ms"), avg("filter_ms"),
                roi[2], roi[3], int(result.get("layers_checked", False)),
                result.get("candidate_count", 0), int(result.get("target_confirmed", False)),
                result.get("lost_frames", 0), result.get("state", "SEARCH"),
                avg("ocr_ms"), result.get("recognized_digit", 0),
                result.get("ocr_calls", 0), result.get("ocr_reason", "NONE"))
        else:
            text += " CVT:{:.1f} THR:{:.1f} MORPH:{:.1f} BLOB:{:.1f} COLORS:{}".format(
                avg("conversion_ms"), avg("threshold_ms"), avg("morph_ms"), avg("blob_ms"),
                result.get("color_count", 0))
            text += " TARGET:{} FOUND:{}".format(
                result.get("target_color", "-"), int(result.get("found", False)))
            if task_id == 1:
                text += " RESIZE:{:.1f} BUF:{}".format(
                    avg("resize_ms"), int(result.get("buffer_reuse", False)))
                text += " OK:{} COV:{:.1f}%".format(
                    result.get("ok", 0), result.get("coverage_ratio", 0.0) * 100)
                text += " COVER:{:.1f}".format(avg("coverage_ms"))
        return text
