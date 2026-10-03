"""纯Python发送调度和性能统计，可在电脑上测试，无Maix硬件依赖。"""

from vision_tasks import safe_xy_result


SEND_TIME_MS = 25
MAX_RESULT_AGE_MS = 50


class ResultSender:
    """每个新帧至多发送一次，错过截止时间不突发补发旧结果。"""

    def __init__(self, interval_ms=SEND_TIME_MS, max_age_ms=MAX_RESULT_AGE_MS):
        self.interval_ms = max(1, int(interval_ms))
        self.max_age_ms = max(1, int(max_age_ms))
        self.reset(0)

    def reset(self, now_ms):
        self.next_send_ms = now_ms
        self.frame_id = 0

    def take(self, result, capture_ms, now_ms, require_track=False):
        self.frame_id = (self.frame_id + 1) & 0xFF
        if now_ms < self.next_send_ms:
            return None
        missed = int((now_ms - self.next_send_ms) // self.interval_ms)
        self.next_send_ms += (missed + 1) * self.interval_ms
        valid, dx, dy = safe_xy_result(result, require_track)
        age_ms = max(0, now_ms - capture_ms)
        fresh = age_ms <= self.max_age_ms
        if not fresh:
            valid, dx, dy = False, 0, 0
        return {"frame_id": self.frame_id, "raw_ok": valid,
                "dx": dx, "dy": dy, "age_ms": age_ms, "fresh": fresh}


def pack_xy_frame(cmd, frame_id, raw_ok, dx, dy):
    """AA55 CMD 06 FRAME_ID RAW_OK int16(dx) int16(dy) SUM。"""
    if not raw_ok:
        dx = dy = 0
    dx = max(-32768, min(32767, int(dx))) & 0xFFFF
    dy = max(-32768, min(32767, int(dy))) & 0xFFFF
    data = [int(frame_id) & 0xFF, int(bool(raw_ok)),
            dx & 0xFF, dx >> 8, dy & 0xFF, dy >> 8]
    return bytes([0xAA, 0x55, cmd, 6] + data + [(cmd + 6 + sum(data)) & 0xFF])


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
                    "mask_ms", "group_ms", "contour_ms", "filter_ms"):
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
                     "ROI:{}x{} LAYER:{} CAND:{} LOCK:{} LOST:{} STATE:{}").format(
                avg("mask_ms"), avg("group_ms"), avg("contour_ms"), avg("filter_ms"),
                roi[2], roi[3], int(result.get("layers_checked", False)),
                result.get("candidate_count", 0), int(result.get("target_confirmed", False)),
                result.get("lost_frames", 0), result.get("state", "SEARCH"))
        else:
            text += " CVT:{:.1f} THR:{:.1f} MORPH:{:.1f} BLOB:{:.1f} COLORS:{}".format(
                avg("conversion_ms"), avg("threshold_ms"), avg("morph_ms"), avg("blob_ms"),
                result.get("color_count", 0))
            if "by_color" in result:
                text += " FOUND:" + "/".join(
                    str(int(result["by_color"][key]["found"]))
                    for key in ("red", "yellow", "blue"))
        return text
