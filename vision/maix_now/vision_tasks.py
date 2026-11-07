"""校赛后视觉：B2单色、B3确认数字2后跟踪圆心、B4复用单色LAB。"""

import cv2
from time import perf_counter

from vision_blob import (
    ColorBuffers, apply_grasp_status, detect_color, empty_result, extract_roi, make_run_geometry,
    map_run_result_to_source, resize_run_roi, scale_grasp_roi,
)
from vision_config import (
    IMAGE_HEIGHT, IMAGE_WIDTH,
    RUN_PROCESS_HEIGHT, RUN_PROCESS_WIDTH,
)
from vision_ring import detect_ring_candidates
from vision_ocr import confirm_digit_two

B3_TASK_BUILD = "B3_DIGIT_TWO_V1"


TASK_NONE = 0
B2_TASK_BUILD = "B2_COLOR_DIAG_V6"
TASK_MATERIAL = 1
TASK_RING = 2
TASK_STACK = 3
TASK_LABELS = {
    TASK_NONE: "待机",
    TASK_MATERIAL: "任务一 单色物料定位",
    TASK_RING: "任务二 圆环中心定位",
    TASK_STACK: "任务三 码垛堆放",
}


def safe_xy_result(result, require_track=False, allow_grasp=False):
    """无效或未锁定的帧绝不携带旧坐标。"""
    valid = bool(result.get("found", False))
    if require_track:
        valid = valid and result.get("state") == "TRACK"
    if not valid:
        return 0, 0, 0
    ok = 2 if allow_grasp and result.get("ok") == 2 else 1
    return ok, int(result.get("dx", 0)), int(result.get("dy", 0))


class ColorTracker:
    """B2/B4共享单色LAB处理，标定参数由各任务独立提供。"""

    def __init__(self):
        self.b2_buffers_supported = True
        self.reset()

    def enter(self, target):
        self.target = target

    def reset(self):
        self.target = None
        self.invalidate_geometry()

    def exit(self):
        self.reset()

    def invalidate_geometry(self):
        self.run_geometry = None
        self.run_geometry_key = None
        self.b2_buffers = None

    def _ensure_b2_buffers(self, width, height):
        if self.b2_buffers_supported and (self.b2_buffers is None or
                                         self.b2_buffers.mask.shape != (height, width)):
            self.b2_buffers = ColorBuffers(width, height)

    def process(self, cv_bgr, config, run_mode=False, need_mask=True, reuse_buffers=False,
                grasp=None, diagnostic=False):
        resize_tick = perf_counter()
        if run_mode:
            roi, center, blob = config["roi"], config["center"], config["blob"]
            key = (tuple(roi[k] for k in ("x", "y", "w", "h")),
                   center["x"], center["y"], center["custom"],
                   blob["min_area"], blob["max_area"],
                   config["morph_mode"], config["target_color"])
            key += (tuple(tuple(r[k] for k in ("x", "y", "w", "h"))
                          for r in config.get("exclude_rois", [])),)
            key += (tuple(tuple(tuple(p) for p in polygon)
                          for polygon in config.get("exclude_polygons", [])),)
            if self.run_geometry is None or key != self.run_geometry_key:
                self.run_geometry = make_run_geometry(
                    config, IMAGE_WIDTH, IMAGE_HEIGHT,
                    RUN_PROCESS_WIDTH, RUN_PROCESS_HEIGHT,
                )
                self.run_geometry_key = key
            process_config = self.run_geometry["config"]
            process_config["thresholds"] = config["thresholds"]
            process_roi = self.run_geometry["process_roi"]
            if reuse_buffers:
                self._ensure_b2_buffers(process_roi[2], process_roi[3])
            try:
                dst = self.b2_buffers.bgr if reuse_buffers and self.b2_buffers is not None else None
                roi_bgr = resize_run_roi(cv_bgr, self.run_geometry, dst=dst)
            except (TypeError, cv2.error) as error:
                if dst is None:
                    raise
                self._disable_b2_buffers(error)
                roi_bgr = resize_run_roi(cv_bgr, self.run_geometry)
        else:
            process_config = config
            roi_bgr, process_roi = extract_roi(cv_bgr, config)
            if reuse_buffers:
                self._ensure_b2_buffers(process_roi[2], process_roi[3])

        target_key = config["target_color"]
        grasp_roi = None
        if grasp is not None:
            grasp_roi = (scale_grasp_roi(grasp, self.run_geometry) if run_mode else
                         tuple(grasp["roi"][k] for k in ("x", "y", "w", "h")))
        resize_ms = (perf_counter() - resize_tick) * 1000.0
        # B2调参也复用LAB/MASK输出，仍按320x240处理；B4沿用原路径。
        buffers = self.b2_buffers if reuse_buffers else None
        diagnostics = {} if diagnostic else None
        try:
            result, mask_roi, perf = detect_color(
                roi_bgr, process_config, process_roi, need_mask, buffers=buffers,
                fast_filter=reuse_buffers,
                grasp_roi=grasp_roi,
                grasp_preview=need_mask and not run_mode and grasp is not None,
                diagnostics=diagnostics,
            )
        except (TypeError, cv2.error) as error:
            if buffers is None:
                raise
            self._disable_b2_buffers(error)
            result, mask_roi, perf = detect_color(
                roi_bgr, process_config, process_roi, need_mask, fast_filter=reuse_buffers,
                grasp_roi=grasp_roi,
                grasp_preview=need_mask and not run_mode and grasp is not None,
                diagnostics=diagnostics)
        if reuse_buffers:
            perf["resize_ms"] = resize_ms
            perf["buffer_reuse"] = buffers is not None and self.b2_buffers_supported
        if run_mode:
            result = map_run_result_to_source(result, config, self.run_geometry)
            if mask_roi is not None:
                roi = config["roi"]
                mask_roi = cv2.resize(mask_roi, (roi["w"], roi["h"]),
                                      interpolation=cv2.INTER_NEAREST)
        if grasp is not None:
            result = apply_grasp_status(result, grasp)
        if diagnostics is not None:
            diagnostics["size"] = process_roi[2:]
            result["diagnostic"] = diagnostics
        result.update(perf)
        result["target_color"] = target_key
        result["color_count"] = 1
        roi = config["roi"]
        return {"result": result, "mask_roi": mask_roi,
                "roi_rect": tuple(roi[k] for k in ("x", "y", "w", "h")),
                "config": config}

    def _disable_b2_buffers(self, error):
        # 旧固件不支持dst时只提示一次，以原路径处理当前新图，不反复试错。
        self.b2_buffers_supported = False
        self.b2_buffers = None
        print("[VISION B2] dst unsupported, using original path:", error)


MaterialTask = ColorTracker


class RingTask:
    """稳定圆环中确认数字2；确认后沿用动态窗口、圆心跟踪和丢失恢复。"""

    STABLE_FRAMES_REQUIRED = 3
    OCR_RETRY_COOLDOWN_FRAMES = 8
    REACQUIRE_FRAME_LIMIT = 15
    REACQUIRE_DISTANCE = 48

    def __init__(self):
        self.target = None
        self._clear_runtime()

    def enter(self, target):
        self.target = target
        self._clear_runtime()

    def reset(self):
        self.target = None
        self._clear_runtime()

    def exit(self):
        self.reset()

    def _clear_runtime(self):
        self.state = "SEARCH"
        self.target_confirmed = False
        self.stable_frames = 0
        self.pending_center = None
        self.target_center = None
        self.target_radius = 0
        self.lost_frames = 0
        self.frames_since_layers = 0
        self.ocr_retry_cooldown = 0
        self.last_ocr_reason = "NONE"
        self.last_ocr_score = 1.0
        self.last_ocr_margin = 0.0

    @staticmethod
    def _distance2(left, right):
        return (int(left[0]) - int(right[0])) ** 2 + (int(left[1]) - int(right[1])) ** 2

    @classmethod
    def _nearest_candidate(cls, candidates, center, max_distance=None):
        if not candidates:
            return None
        item = min(candidates,
                   key=lambda c: cls._distance2((c["cx"], c["cy"]), center))
        if (max_distance is not None and
                cls._distance2((item["cx"], item["cy"]), center) > max_distance ** 2):
            return None
        return item

    def _tracking_config(self, ring):
        """窗口随圆心移动，dx/dy仍相对于原始标定中心。"""
        roi = ring["roi"]
        padding = int(ring.get("track_padding", 48))
        half = self.target_radius + padding + 8
        cx, cy = self.target_center
        x1, y1 = max(roi["x"], cx - half), max(roi["y"], cy - half)
        x2 = min(roi["x"] + roi["w"], cx + half + 1)
        y2 = min(roi["y"] + roi["h"], cy + half + 1)
        tracking = dict(ring)
        tracking["roi"] = {"x": x1, "y": y1, "w": x2 - x1, "h": y2 - y1}
        tracking["center"] = {
            "x": roi["x"] + ring["center"]["x"] - x1,
            "y": roi["y"] + ring["center"]["y"] - y1,
            "custom": True,
        }
        return tracking

    def process(self, cv_bgr, config, run_mode=False, need_mask=True):
        ring = config["ring"]
        ocr_ms, ocr_calls = 0.0, 0
        check_layers = True
        process_ring = ring
        if self.target_confirmed and self.state == "TRACK":
            process_ring = self._tracking_config(ring)
            self.frames_since_layers += 1
            interval = int(ring.get("layer_check_interval", 5))
            check_layers = self.frames_since_layers >= interval
            if check_layers:
                self.frames_since_layers = 0
        result, mask, roi_rect = detect_ring_candidates(
            cv_bgr, process_ring, check_layers=check_layers,
        )
        candidates = result["candidates"]
        tolerance = max(2, int(ring.get("center_tolerance", 8)))
        if self.target_confirmed:
            selected = self._nearest_candidate(
                candidates, self.target_center, self.REACQUIRE_DISTANCE,
            )
            if selected is None:
                self.lost_frames += 1
                self.state = "LOST"
                if self.lost_frames >= self.REACQUIRE_FRAME_LIMIT:
                    self._clear_runtime()
            else:
                self.state = "TRACK"
                self.lost_frames = 0
        else:
            roi, center = ring["roi"], ring["center"]
            selected = self._nearest_candidate(
                candidates, (roi["x"] + center["x"], roi["y"] + center["y"]),
            )
            if selected is None:
                self.state = "SEARCH"
                self.stable_frames = 0
                self.pending_center = None
                self.ocr_retry_cooldown = 0
            else:
                position = (selected["cx"], selected["cy"])
                if (self.pending_center is not None and
                        self._distance2(position, self.pending_center) <= tolerance ** 2):
                    self.stable_frames += 1
                else:
                    self.stable_frames = 1
                    self.ocr_retry_cooldown = 0
                self.pending_center = position
                self.state = "CONFIRM"
                if self.ocr_retry_cooldown > 0:
                    self.ocr_retry_cooldown -= 1
                elif self.stable_frames >= self.STABLE_FRAMES_REQUIRED:
                    # 沿用旧版按参考点远近确认候选的流程，避免最近的1/3挡住真正的2。
                    matched = None
                    reference = (roi["x"] + center["x"], roi["y"] + center["y"])
                    for candidate in sorted(candidates, key=lambda c: self._distance2(
                            (c["cx"], c["cy"]), reference)):
                        decision = confirm_digit_two(cv_bgr, candidate)
                        ocr_ms += float(decision.get("ocr_ms", 0.0))
                        ocr_calls += 1
                        self.last_ocr_reason = decision.get("reason", "UNKNOWN")
                        self.last_ocr_score = float(decision.get("score", 1.0))
                        self.last_ocr_margin = float(decision.get("margin", 0.0))
                        candidate["digit"] = 2 if decision.get("is_two", False) else 0
                        if candidate["digit"] == 2:
                            matched = candidate
                            break
                    if matched is not None:
                        selected = matched
                        self.target_confirmed = True
                        self.state = "TRACK"
                    else:
                        self.ocr_retry_cooldown = self.OCR_RETRY_COOLDOWN_FRAMES

        if selected is not None and self.state == "TRACK":
            self.target_center = (selected["cx"], selected["cy"])
            self.target_radius = selected["radius"]
            result.update({
                "found": True, "cx": selected["cx"], "cy": selected["cy"],
                "dx": selected["dx"], "dy": selected["dy"],
                "area": int(round(3.1415926 * selected["radius"] ** 2)),
                "rect": selected["rect"], "quality": selected["shape_roundness"],
            })
        for candidate in candidates:
            candidate["selected"] = candidate is selected
            if candidate is selected and self.target_confirmed:
                candidate["digit"] = 2
        result.update({
            "state": self.state, "stage": self.state, "target_number": 2,
            "target_confirmed": self.target_confirmed,
            "stable_frames": self.stable_frames, "lost_frames": self.lost_frames,
            "selected_present": selected is not None, "process_roi": roi_rect,
            "layers_checked": check_layers,
            "recognized_digit": 2 if self.target_confirmed else 0,
            "is_two_confirmed": self.target_confirmed,
            "ocr_ms": ocr_ms, "ocr_calls": ocr_calls,
            "ocr_reason": self.last_ocr_reason, "ocr_score": self.last_ocr_score,
            "ocr_margin": self.last_ocr_margin,
        })
        return {"result": result, "mask_roi": mask if need_mask else None,
                "roi_rect": roi_rect, "config": config}


class TaskManager:
    """每帧只运行一个任务；B2/B4共享ColorTracker。"""

    def __init__(self):
        self.color_tracker = ColorTracker()
        self.tasks = {TASK_MATERIAL: self.color_tracker,
                      TASK_RING: RingTask(), TASK_STACK: self.color_tracker}
        self.task_id = TASK_NONE
        self.target = None
        self.current = None

    def switch(self, task_id, target=None):
        if task_id not in self.tasks:
            task_id, target = TASK_NONE, None
        if task_id == self.task_id and target == self.target:
            return False
        if self.current is not None:
            self.current.exit()
        self.task_id, self.target = task_id, target
        self.current = self.tasks.get(task_id)
        if self.current is not None:
            self.current.enter(target)
        return True

    def reset_current(self):
        if self.current is not None:
            self.current.reset()
            self.current.enter(self.target)

    def invalidate_material_geometry(self):
        self.color_tracker.invalidate_geometry()

    def process(self, cv_bgr, config, run_mode=False, need_mask=True, diagnostic=False):
        if self.current is None:
            return {"result": empty_result(), "mask_roi": None,
                    "roi_rect": (0, 0, IMAGE_WIDTH, IMAGE_HEIGHT), "config": config}
        if self.task_id == TASK_RING:
            return self.current.process(cv_bgr, config, run_mode, need_mask)
        task_config = dict(config)
        if self.task_id == TASK_STACK:
            task_config.pop("exclude_rois", None)
            task_config.pop("exclude_polygons", None)
            for key in ("roi", "center", "blob"):
                task_config[key] = config["stack"][key]
        else:
            # 持久化B2参考点为原图绝对坐标，检测器内部仍接收ROI内相对参考点。
            task_config["center"] = dict(config["center"])
            task_config["center"]["x"] -= config["roi"]["x"]
            task_config["center"]["y"] -= config["roi"]["y"]
        frame = self.current.process(
            cv_bgr, task_config, run_mode, need_mask=need_mask,
            reuse_buffers=self.task_id == TASK_MATERIAL,
            grasp=config["grasp"] if self.task_id == TASK_MATERIAL else None,
            diagnostic=diagnostic and self.task_id == TASK_MATERIAL,
        )
        if self.task_id == TASK_MATERIAL:
            frame["config"] = config  # 预览仍用原图绝对参考点，内部局部坐标不外泄。
        return frame
