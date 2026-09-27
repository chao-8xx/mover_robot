"""轻量视觉任务调度。

主程序只负责相机、UI、串口和显示；每轮只把当前帧交给一个任务。
B2物料和B4码垛共享同一个颜色识别器；B3低频确认指定的1/2/3，
确认成功后停止OCR并持续跟踪圆心。
"""

from vision_blob import (
    build_mask,
    empty_result,
    extract_roi,
    find_largest_blob,
    make_run_geometry,
    map_run_result_to_source,
    resize_run_roi,
)
from vision_config import (
    IMAGE_HEIGHT,
    IMAGE_WIDTH,
    RUN_PROCESS_HEIGHT,
    RUN_PROCESS_WIDTH,
)
from vision_ocr import recognize_digit
from vision_ring import detect_ring_candidates


TASK_NONE = 0
TASK_MATERIAL = 1
TASK_RING = 2
TASK_STACK = 3

TASK_LABELS = {
    TASK_NONE: "待机",
    TASK_MATERIAL: "任务一 物料识别",
    TASK_RING: "任务二 数字圆环定位",
    TASK_STACK: "任务三 码垛堆放",
}


def safe_xy_result(result, require_track=False):
    """把检测结果转换为串口安全值；无效帧绝不携带旧坐标。"""
    valid = bool(result.get("found", False))
    if require_track:
        valid = valid and result.get("state") == "TRACK"
    if not valid:
        return False, 0, 0
    return True, int(result.get("dx", 0)), int(result.get("dy", 0))


class ColorTracker:
    """B2/B4共享的颜色识别器，保持已经验收的坐标和缩放契约。"""

    def __init__(self):
        self.target = None
        self.run_geometry = None
        self.run_geometry_target = None

    def enter(self, target):
        self.target = target

    def reset(self):
        self.target = None
        self.run_geometry = None
        self.run_geometry_target = None

    def exit(self):
        self.reset()

    def invalidate_geometry(self):
        """保存新ROI、中心或阈值后，下次RUN重新生成缩放参数。"""
        self.run_geometry = None
        self.run_geometry_target = None

    def process(self, cv_bgr, config, run_mode=False):
        if run_mode:
            target_key = config["target_color"]
            if self.run_geometry is None or self.run_geometry_target != target_key:
                self.run_geometry = make_run_geometry(
                    config,
                    IMAGE_WIDTH,
                    IMAGE_HEIGHT,
                    RUN_PROCESS_WIDTH,
                    RUN_PROCESS_HEIGHT,
                )
                self.run_geometry_target = target_key
            process_config = self.run_geometry["config"]
            roi_bgr = resize_run_roi(cv_bgr, self.run_geometry)
            roi_rect = self.run_geometry["process_roi"]
        else:
            process_config = config
            roi_bgr, roi_rect = extract_roi(cv_bgr, config)

        mask_roi = build_mask(roi_bgr, process_config)
        process_result = find_largest_blob(mask_roi, process_config, roi_rect)
        if run_mode:
            result = map_run_result_to_source(
                process_result, config, self.run_geometry
            )
        else:
            result = process_result

        return {
            "result": result,
            "mask_roi": mask_roi,
            "roi_rect": roi_rect,
            "config": config,
        }


# 保留旧名称，避免已有本机测试或外部脚本因重构失效。
MaterialTask = ColorTracker


class RingTask:
    """B3：低频确认指定的1/2/3，确认后只跟踪该圆环中心。"""

    STABLE_FRAMES_REQUIRED = 3
    OCR_RETRY_COOLDOWN_FRAMES = 8
    REACQUIRE_FRAME_LIMIT = 15
    REACQUIRE_DISTANCE = 48

    def __init__(self):
        self.target = None
        self.target_number = 2
        self._clear_runtime()

    def enter(self, target):
        self.target = target
        self.target_number = self._target_to_number(target)
        self._clear_runtime()

    def reset(self):
        self.target = None
        self.target_number = 2
        self._clear_runtime()

    def exit(self):
        self.reset()

    def _clear_runtime(self):
        self.state = "SEARCH"
        self.target_confirmed = False
        self.stable_frames = 0
        self.pending_center = None
        self.target_center = None
        self.lost_frames = 0
        self.ocr_retry_cooldown = 0
        self.ocr_attempts = 0
        self.ocr_best_score = 1.0
        self.ocr_preview = None
        self.last_ocr_score = 1.0
        self.last_ocr_margin = 0.0
        self.last_ocr_reason = "NONE"

    @staticmethod
    def _target_to_number(target):
        if target in (1, 0xD1):
            return 1
        if target in (3, 0xD3):
            return 3
        return 2

    @staticmethod
    def _distance2(left, right):
        dx = int(left[0]) - int(right[0])
        dy = int(left[1]) - int(right[1])
        return dx * dx + dy * dy

    @classmethod
    def _nearest_candidate(cls, candidates, center, max_distance=None):
        if not candidates:
            return None
        candidate = min(
            candidates,
            key=lambda item: cls._distance2(
                (item["cx"], item["cy"]), center
            ),
        )
        if max_distance is not None:
            if cls._distance2(
                    (candidate["cx"], candidate["cy"]), center
            ) > max_distance * max_distance:
                return None
        return candidate

    def _set_target_result(self, result, candidate):
        radius = int(candidate.get("radius", 0))
        result.update({
            "found": True,
            "cx": int(candidate["cx"]),
            "cy": int(candidate["cy"]),
            "dx": int(candidate["dx"]),
            "dy": int(candidate["dy"]),
            "area": int(round(3.1415926 * radius * radius)),
            "rect": candidate.get("rect", (0, 0, 0, 0)),
            "quality": max(0.0, 1.0 - float(self.ocr_best_score)),
            "recognized_digit": self.target_number,
        })

    def _update_pending(self, candidate, tolerance):
        center = (int(candidate["cx"]), int(candidate["cy"]))
        if (self.pending_center is not None and
                self._distance2(center, self.pending_center) <= tolerance * tolerance):
            self.stable_frames += 1
        else:
            self.stable_frames = 1
            self.ocr_retry_cooldown = 0
            self.ocr_attempts = 0
            self.ocr_best_score = 1.0
            self.ocr_preview = None
            self.last_ocr_score = 1.0
            self.last_ocr_margin = 0.0
            self.last_ocr_reason = "NONE"
        self.pending_center = center

    def _annotate_result(self, result, selected, ocr_ms, ocr_calls):
        for candidate in result.get("candidates", ()):
            is_selected = candidate is selected
            candidate["selected"] = is_selected
            if is_selected and self.target_confirmed:
                candidate["digit"] = self.target_number
            elif "digit" not in candidate:
                candidate["digit"] = 0

        result.update({
            "state": self.state,
            "stage": self.state,
            "target_number": self.target_number,
            "recognized_digit": (
                self.target_number if self.target_confirmed else 0
            ),
            "stable_frames": int(self.stable_frames),
            "selected_present": selected is not None,
            "target_confirmed": bool(self.target_confirmed),
            "is_two_confirmed": bool(
                self.target_confirmed and self.target_number == 2
            ),
            "lost_frames": int(self.lost_frames),
            "ocr_attempts": int(self.ocr_attempts),
            "ocr_best_score": float(self.ocr_best_score),
            "ocr_score": float(self.last_ocr_score),
            "ocr_margin": float(self.last_ocr_margin),
            "ocr_reason": self.last_ocr_reason,
            "ocr_preview": self.ocr_preview,
            "ocr_ms": float(ocr_ms),
            "ocr_calls": int(ocr_calls),
            "ocr_retry_cooldown": int(self.ocr_retry_cooldown),
        })

    def process(self, cv_bgr, config, run_mode=False):
        result, mask_roi, roi_rect = detect_ring_candidates(
            cv_bgr, config["ring"]
        )
        candidates = result.get("candidates", [])
        tolerance = max(2, int(config["ring"].get("center_tolerance", 8)))
        ocr_ms = 0.0
        ocr_calls = 0
        selected = None

        if self.target_confirmed:
            track_tolerance = max(self.REACQUIRE_DISTANCE, tolerance * 6)
            selected = self._nearest_candidate(
                candidates, self.target_center, track_tolerance
            )
            if selected is None:
                self.lost_frames += 1
                self.state = "LOST"
                if self.lost_frames >= self.REACQUIRE_FRAME_LIMIT:
                    self._clear_runtime()
            else:
                self.state = "TRACK"
                self.lost_frames = 0
                self.target_center = (
                    int(selected["cx"]), int(selected["cy"])
                )
                self.pending_center = self.target_center
                self._set_target_result(result, selected)
        else:
            ring_center = config["ring"]["center"]
            ring_roi = config["ring"]["roi"]
            grab_center = (
                int(ring_roi["x"]) + int(ring_center["x"]),
                int(ring_roi["y"]) + int(ring_center["y"]),
            )
            selected = self._nearest_candidate(candidates, grab_center)
            if selected is None:
                self.state = "SEARCH"
                self.stable_frames = 0
                self.pending_center = None
                if self.ocr_retry_cooldown > 0:
                    self.ocr_retry_cooldown -= 1
            else:
                self._update_pending(selected, tolerance)
                self.state = (
                    "CONFIRM"
                    if self.stable_frames >= self.STABLE_FRAMES_REQUIRED
                    else "SEARCH"
                )
                if self.ocr_retry_cooldown > 0:
                    self.ocr_retry_cooldown -= 1
                elif self.state == "CONFIRM":
                    ordered_candidates = sorted(
                        candidates,
                        key=lambda item: self._distance2(
                            (item["cx"], item["cy"]), grab_center
                        ),
                    )
                    matched = None
                    for candidate in ordered_candidates:
                        ocr_result = recognize_digit(cv_bgr, candidate)
                        ocr_ms += float(ocr_result.get("ocr_ms", 0.0))
                        ocr_calls += 1
                        self.ocr_attempts += 1
                        self.last_ocr_score = float(
                            ocr_result.get("score", 1.0)
                        )
                        self.last_ocr_margin = float(
                            ocr_result.get("margin", 0.0)
                        )
                        self.last_ocr_reason = str(
                            ocr_result.get("reason", "UNKNOWN")
                        )
                        self.ocr_best_score = min(
                            self.ocr_best_score,
                            self.last_ocr_score,
                        )
                        preview = ocr_result.get("normalized")
                        if preview is None:
                            preview = ocr_result.get("binary")
                        self.ocr_preview = preview
                        candidate["digit"] = int(
                            ocr_result.get("digit", 0)
                        )
                        if candidate["digit"] == self.target_number:
                            matched = candidate
                            self.ocr_best_score = self.last_ocr_score
                            break

                    if matched is not None:
                        selected = matched
                        self.target_confirmed = True
                        self.state = "TRACK"
                        self.target_center = (
                            int(selected["cx"]), int(selected["cy"])
                        )
                        self.pending_center = self.target_center
                        self.lost_frames = 0
                        self._set_target_result(result, selected)
                    else:
                        # 未找到指定数字时保持未知，间隔若干帧再确认。
                        self.ocr_retry_cooldown = self.OCR_RETRY_COOLDOWN_FRAMES

        self._annotate_result(result, selected, ocr_ms, ocr_calls)
        if self.state == "TRACK" and selected is not None:
            self._set_target_result(result, selected)
        return {
            "result": result,
            "mask_roi": mask_roi,
            "roi_rect": roi_rect,
            "config": config,
        }


class TaskManager:
    """任意时刻只有一个任务拥有当前帧；B2/B4共享颜色识别实例。"""

    def __init__(self):
        self.color_tracker = ColorTracker()
        self.tasks = {
            TASK_MATERIAL: self.color_tracker,
            TASK_RING: RingTask(),
            TASK_STACK: self.color_tracker,
        }
        self.task_id = TASK_NONE
        self.target = None
        self.current = None

    def switch(self, task_id, target=None):
        if task_id not in self.tasks:
            task_id = TASK_NONE
            target = None
        if task_id == self.task_id and target == self.target:
            return False

        if self.current is not None:
            self.current.exit()

        self.task_id = task_id
        self.target = target
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

    def process(self, cv_bgr, config, run_mode=False):
        if self.current is None:
            return {
                "result": empty_result(),
                "mask_roi": None,
                "roi_rect": (0, 0, IMAGE_WIDTH, IMAGE_HEIGHT),
                "config": config,
            }
        task_config = config
        if self.task_id == TASK_STACK:
            # 阈值、模式和形态学继续共享；几何与面积使用任务三独立参数。
            task_config = dict(config)
            stack_config = config["stack"]
            task_config["roi"] = stack_config["roi"]
            task_config["center"] = stack_config["center"]
            task_config["blob"] = stack_config["blob"]
        return self.current.process(cv_bgr, task_config, run_mode)
