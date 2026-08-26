"""轻量视觉任务调度。

主程序只负责相机、UI、串口和显示；每个任务每次只处理一帧。
当前阶段封装已经真机验收的任务一和任务二候选检测；任务三保留安全占位结果。
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
from vision_ring import detect_ring_candidates


TASK_NONE = 0
TASK_MATERIAL = 1
TASK_RING = 2
TASK_STACK = 3

TASK_LABELS = {
    TASK_NONE: "待机",
    TASK_MATERIAL: "任务一 物料识别",
    TASK_RING: "任务二 数字与圆环定位",
    TASK_STACK: "任务三 码垛堆放",
}


class MaterialTask:
    """任务一：封装现有B2识别路径，不改变算法和坐标契约。"""

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


class PlaceholderTask:
    """未实现任务的安全占位：不运行旧原型，也不产生有效坐标。"""

    def __init__(self, task_id):
        self.task_id = task_id
        self.target = None

    def enter(self, target):
        self.target = target

    def reset(self):
        self.target = None

    def exit(self):
        self.reset()

    def process(self, cv_bgr, config, run_mode=False):
        return {
            "result": empty_result(),
            "mask_roi": None,
            "roi_rect": (0, 0, IMAGE_WIDTH, IMAGE_HEIGHT),
            "config": config,
        }


class RingTask:
    """任务二第一阶段：只生成黑色MASK和同心圆候选，不运行OCR。"""

    def __init__(self):
        self.target = None

    def enter(self, target):
        self.target = target

    def reset(self):
        self.target = None

    def exit(self):
        self.reset()

    def process(self, cv_bgr, config, run_mode=False):
        result, mask_roi, roi_rect = detect_ring_candidates(
            cv_bgr, config["ring"]
        )
        return {
            "result": result,
            "mask_roi": mask_roi,
            "roi_rect": roi_rect,
            "config": config,
        }


class TaskManager:
    """保证任意时刻只有一个任务拥有当前帧，并在切换时清空旧状态。"""

    def __init__(self):
        self.tasks = {
            TASK_MATERIAL: MaterialTask(),
            TASK_RING: RingTask(),
            TASK_STACK: PlaceholderTask(TASK_STACK),
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
        self.tasks[TASK_MATERIAL].invalidate_geometry()

    def process(self, cv_bgr, config, run_mode=False):
        if self.current is None:
            return {
                "result": empty_result(),
                "mask_roi": None,
                "roi_rect": (0, 0, IMAGE_WIDTH, IMAGE_HEIGHT),
                "config": config,
            }
        return self.current.process(cv_bgr, config, run_mode)
