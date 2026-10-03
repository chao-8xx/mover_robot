"""校赛后视觉：B2全图三色、B3纯圆心、B4复用颜色识别。"""

import cv2

from vision_blob import (
    detect_color_set, empty_result, extract_roi, make_run_geometry,
    map_run_result_to_source, resize_run_roi,
)
from vision_config import (
    IMAGE_HEIGHT, IMAGE_WIDTH, MATERIAL_COLORS,
    RUN_PROCESS_HEIGHT, RUN_PROCESS_WIDTH,
)
from vision_ring import detect_ring_candidates


TASK_NONE = 0
TASK_MATERIAL = 1
TASK_RING = 2
TASK_STACK = 3
TASK_LABELS = {
    TASK_NONE: "待机",
    TASK_MATERIAL: "任务一 全图三色物料",
    TASK_RING: "任务二 圆环中心定位",
    TASK_STACK: "任务三 码垛堆放",
}


def safe_xy_result(result, require_track=False):
    """无效或未锁定的帧绝不携带旧坐标。"""
    valid = bool(result.get("found", False))
    if require_track:
        valid = valid and result.get("state") == "TRACK"
    if not valid:
        return False, 0, 0
    return True, int(result.get("dx", 0)), int(result.get("dy", 0))


class ColorTracker:
    """一次颜色转换供多个MASK共用；B4只处理指定颜色。"""

    def __init__(self):
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

    def process(self, cv_bgr, config, run_mode=False,
                multi_color=False, need_mask=True):
        if run_mode:
            roi, center, blob = config["roi"], config["center"], config["blob"]
            key = (tuple(roi[k] for k in ("x", "y", "w", "h")),
                   center["x"], center["y"], center["custom"],
                   blob["min_area"], blob["max_area"],
                   config["mode"], config["morph_mode"], config["target_color"])
            if self.run_geometry is None or key != self.run_geometry_key:
                self.run_geometry = make_run_geometry(
                    config, IMAGE_WIDTH, IMAGE_HEIGHT,
                    RUN_PROCESS_WIDTH, RUN_PROCESS_HEIGHT,
                )
                self.run_geometry_key = key
            process_config = self.run_geometry["config"]
            process_config["thresholds"] = config["thresholds"]
            roi_bgr = resize_run_roi(cv_bgr, self.run_geometry)
            process_roi = self.run_geometry["process_roi"]
        else:
            process_config = config
            roi_bgr, process_roi = extract_roi(cv_bgr, config)

        target_key = config["target_color"]
        color_keys = MATERIAL_COLORS if multi_color else (target_key,)
        # 保持旧B2对绿/黑/浅蓝调参和指令的兼容性。
        if target_key not in color_keys:
            color_keys = color_keys + (target_key,)
        detected, mask_roi, perf = detect_color_set(
            roi_bgr, process_config, process_roi, color_keys, need_mask,
        )
        if run_mode:
            detected = {
                color: map_run_result_to_source(value, config, self.run_geometry)
                for color, value in detected.items()
            }
            if mask_roi is not None:
                roi = config["roi"]
                mask_roi = cv2.resize(mask_roi, (roi["w"], roi["h"]),
                                      interpolation=cv2.INTER_NEAREST)
        result = dict(detected[target_key])
        result.update(perf)
        result["target_color"] = target_key
        result["color_count"] = len(color_keys)
        if multi_color:
            result["by_color"] = detected
        roi = config["roi"]
        return {"result": result, "mask_roi": mask_roi,
                "roi_rect": tuple(roi[k] for k in ("x", "y", "w", "h")),
                "config": config}


MaterialTask = ColorTracker


class RingTask:
    """已到2号附近后，以几何确认圆环；不加载或执行OCR。"""

    STABLE_FRAMES_REQUIRED = 3
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
            else:
                position = (selected["cx"], selected["cy"])
                if (self.pending_center is not None and
                        self._distance2(position, self.pending_center) <= tolerance ** 2):
                    self.stable_frames += 1
                else:
                    self.stable_frames = 1
                self.pending_center = position
                self.state = "CONFIRM"
                if self.stable_frames >= self.STABLE_FRAMES_REQUIRED:
                    self.target_confirmed = True
                    self.state = "TRACK"

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
            candidate["digit"] = 0
        result.update({
            "state": self.state, "stage": self.state, "target_number": 2,
            "recognized_digit": 0, "ocr_ms": 0.0, "ocr_calls": 0,
            "target_confirmed": self.target_confirmed,
            "stable_frames": self.stable_frames, "lost_frames": self.lost_frames,
            "selected_present": selected is not None, "process_roi": roi_rect,
            "layers_checked": check_layers,
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

    def process(self, cv_bgr, config, run_mode=False, need_mask=True):
        if self.current is None:
            return {"result": empty_result(), "mask_roi": None,
                    "roi_rect": (0, 0, IMAGE_WIDTH, IMAGE_HEIGHT), "config": config}
        if self.task_id == TASK_RING:
            return self.current.process(cv_bgr, config, run_mode, need_mask)
        task_config = dict(config)
        if self.task_id == TASK_STACK:
            for key in ("roi", "center", "blob"):
                task_config[key] = config["stack"][key]
        else:
            task_config["roi"] = {"x": 0, "y": 0, "w": IMAGE_WIDTH, "h": IMAGE_HEIGHT}
        return self.current.process(
            cv_bgr, task_config, run_mode,
            multi_color=self.task_id == TASK_MATERIAL, need_mask=need_mask,
        )
