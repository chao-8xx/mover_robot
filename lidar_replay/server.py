"""本地文件回放服务器，仅绑定 127.0.0.1，无第三方依赖。"""
from __future__ import annotations

import argparse
import base64
import copy
import hashlib
import json
import os
import re
import socket
import subprocess
import threading
import uuid
import webbrowser
from collections import OrderedDict
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse
from urllib.request import urlopen

from demo import make_demo
from map_engine import MapReplay, map_layout
from recording import (MAX_BYTES, MAX_FRAMES, RecordingError, decode_d6, flag,
                       normalize_pose, normalize_recording, number, read_text_recording)
from replay_engine import replay_frame, source_defaults

TOOL_DIR = Path(__file__).resolve().parent
REPO = TOOL_DIR.parent
SOURCE_FILES = ["gongxun_1/BSP/Src/lidar.c", "gongxun_1/BSP/Inc/lidar.h",
                "gongxun_1/APP/Src/obstacle_find.c", "gongxun_1/APP/Inc/obstacle_find.h",
                "gongxun_1/APP/Src/tf2d.c", "gongxun_1/APP/Inc/tf2d.h",
                "gongxun_1/APP/Src/track_map.c", "gongxun_1/APP/Inc/track_map.h",
                "gongxun_1/APP/Src/map_data.c", "gongxun_1/APP/Inc/map_data.h",
                "gongxun_1/APP/Src/map_update.c", "gongxun_1/APP/Inc/map_update.h",
                "gongxun_1/APP/Src/map_lidar.c", "gongxun_1/Control/Src/task.c"]


def source_snapshot():
    def git(*args):
        try:
            return subprocess.check_output(["git", "-c", f"safe.directory={REPO.as_posix()}",
                                            "-C", str(REPO), *args], text=True,
                                           encoding="utf-8", timeout=8, stderr=subprocess.DEVNULL).strip()
        except (OSError, subprocess.SubprocessError):
            return "unknown"
    files = {name: hashlib.sha256((REPO / name).read_bytes()).hexdigest() for name in SOURCE_FILES}
    return {"branch": git("branch", "--show-current"), "commit": git("rev-parse", "HEAD"),
            "dirty": bool(git("status", "--porcelain", "--", "gongxun_1")), "files": files,
            "coordinate_rule": "x=-d*sin(angle), y=d*cos(angle); standard CCW rotation for TF",
            "note": "来源是当前工作区（包含未提交内容）；Python 回放副本的源码对照验证见 VALIDATION.md。"}


def validate_params(params, defaults):
    if not isinstance(params, dict):
        raise RecordingError("参数需要对象")
    p = copy.deepcopy(defaults)
    allowed = {"min_distance", "max_distance", "gap_mm", "min_points", "max_miss",
               "max_clusters", "wheel_ranges", "mask_enabled", "tf", "cluster_mode"}
    if set(params) - allowed:
        raise RecordingError("存在未知回放参数")
    p.update(copy.deepcopy(params))
    for key in ("min_distance", "max_distance"):
        p[key] = number(p[key], key, 0, 65535, integer=True)
    if p["min_distance"] > p["max_distance"]:
        raise RecordingError("最小距离不能大于最大距离")
    p["gap_mm"] = number(p["gap_mm"], "连接距离", 0, 20000)
    for key, lo, hi in (("min_points", 1, 360), ("max_miss", 0, 254), ("max_clusters", 1, 20)):
        p[key] = number(p[key], key, lo, hi, integer=True)
    ranges = p["wheel_ranges"]
    if not isinstance(ranges, list) or len(ranges) != 4:
        raise RecordingError("四轮遮挡需要四组角度区间")
    for r in ranges:
        if not isinstance(r, list) or len(r) != 2:
            raise RecordingError("遮挡区间需要 [起始角度, 结束角度]")
    p["wheel_ranges"] = [[number(a, "遮挡角度", 0, 359, integer=True) for a in r] for r in ranges]
    p["mask_enabled"] = flag(p["mask_enabled"], "mask_enabled")
    if p.get("cluster_mode") not in ("circular", "linear_merge"):
        raise RecordingError("不支持的聚类遍历模式")
    tf = p["tf"]
    if not isinstance(tf, dict) or set(tf) - {"tx", "ty", "yaw_deg", "mirror_x"}:
        raise RecordingError("安装变换参数无效")
    p["tf"] = {"tx": number(tf.get("tx", 0), "tf.tx", -100000, 100000),
               "ty": number(tf.get("ty", 0), "tf.ty", -100000, 100000),
               "yaw_deg": number(tf.get("yaw_deg", 0), "tf.yaw_deg", -360000, 360000),
               "mirror_x": flag(tf.get("mirror_x", False), "mirror_x")}
    return p


class ReplayService:
    def __init__(self):
        self.defaults = source_defaults(REPO)
        self.layout = map_layout(REPO)
        self.source = source_snapshot()
        self.warnings = []
        try:
            verified = json.loads((TOOL_DIR / "source_snapshot.json").read_text(encoding="utf-8"))
            self.source["validated"] = verified.get("files") == self.source["files"]
        except (OSError, ValueError):
            self.source["validated"] = False
        if not self.source["validated"]:
            self.warnings.append("当前固件摘要与工具上次验证版本不同；请重跑自检并核对回放算法。")
        self.datasets = OrderedDict()
        self.states = OrderedDict()
        self.lock = threading.Lock()

    def register(self, recording):
        with self.lock:
            key = uuid.uuid4().hex
            self.datasets[key] = recording
            while len(self.datasets) > 6:
                expired, _ = self.datasets.popitem(last=False)
                self.states = OrderedDict((k, v) for k, v in self.states.items() if k[0] != expired)
            return {**recording, "dataset_id": key}

    def info(self):
        return {"defaults": self.defaults, "layout": self.layout, "source": self.source,
                "limits": {"max_frames": MAX_FRAMES, "max_bytes": MAX_BYTES},
                "warnings": self.warnings + ["已确认障碍与封锁道路不因漏扫自动清除。",
                             "没有有效场地位姿时，仅查看局部点云；地图累计停止。"]}

    def replay(self, request):
        with self.lock:
            dataset_id = request.get("dataset_id")
            if dataset_id:
                recording = self.datasets.get(dataset_id)
                if recording is None:
                    raise RecordingError("记录缓存已过期，请重新导入")
                self.datasets.move_to_end(dataset_id)
            else:
                recording = normalize_recording({"frames": request.get("frames")})
                # 支持脚本调用；界面使用 dataset_id，避免反复传输整份文件。
                dataset_id = hashlib.sha256(json.dumps(recording, sort_keys=True).encode()).hexdigest()
            frames = recording["frames"]
            index = number(request.get("index", 0), "index", 0, len(frames) - 1, integer=True)
            params = validate_params(request.get("params", {}), self.defaults)
            width = number(request.get("car_width_mm", 450), "车体外接正方形边长", 0.001, self.layout["width"])
            safety = number(request.get("safety_mm", 50), "安全余量", 0, self.layout["width"])
            override = normalize_pose(request["pose_override"]) if request.get("pose_override") is not None else None
            def run(profile):
                settings = json.dumps([profile, width, safety, override], sort_keys=True)
                key = (dataset_id, settings)
                previous = self.states.get(key)
                if previous is not None and previous["index"] == index:
                    self.states.move_to_end(key)
                    return copy.deepcopy(previous["result"])
                if previous is None or index < previous["index"]:
                    mapper = MapReplay(width, safety, layout=self.layout)
                    start = 0
                else:
                    mapper = previous["mapper"]
                    start = previous["index"] + 1
                result = None
                for i in range(start, index + 1):
                    frame = dict(frames[i])
                    if override is not None:
                        frame["pose"] = override
                    result = replay_frame(frame, profile)
                    result["pose"] = frame["pose"]
                    result["label"] = frame.get("label", "")
                    result["map_state"] = mapper.apply(result["clusters"], frame["scan_num"],
                                                       frame["end_time"], pose_valid=frame["pose"]["valid"])
                result["warnings"] = result.get("warnings", []) + recording.get("warnings", []) + self.warnings
                self.states[key] = {"index": index, "mapper": mapper, "result": result}
                self.states.move_to_end(key)
                while len(self.states) > 12:
                    self.states.popitem(last=False)
                return copy.deepcopy(result)
            result = run(params)
            baseline = run(self.defaults) if flag(request.get("compare", False), "compare") else None
            return {"frame": result, "baseline": baseline, "index": index, "total": len(frames)}


class Handler(BaseHTTPRequestHandler):
    server_version = "LidarReplay/1"

    def log_message(self, fmt, *args):
        if len(args) > 1 and str(args[1]) not in ("200", "304"):
            super().log_message(fmt, *args)

    def send_json(self, value, status=200):
        body = json.dumps(value, ensure_ascii=False, allow_nan=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.end_headers()
        self.wfile.write(body)

    def local_request(self):
        host = self.headers.get("Host", "")
        allowed = {f"127.0.0.1:{self.server.server_port}", f"localhost:{self.server.server_port}"}
        origin = self.headers.get("Origin")
        return host in allowed and (origin is None or origin in {f"http://{x}" for x in allowed})

    def do_GET(self):
        if not self.local_request():
            return self.send_json({"error": "仅接受本地页面请求"}, 403)
        path = urlparse(self.path).path
        if path == "/api/info":
            return self.send_json(self.server.service.info())
        if path == "/api/demo":
            return self.send_json(self.server.service.register(make_demo()))
        if re.fullmatch(r"/exports/lidar-scan-\d+-[a-f0-9]{12}\.json", path):
            export_dir = getattr(self.server, "export_dir", TOOL_DIR / "exports")
            file = export_dir / path.rsplit("/", 1)[1]
            if not file.is_file():
                return self.send_json({"error": "导出文件不存在"}, 404)
            body = file.read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Disposition", f'attachment; filename="{file.name}"')
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            return self.wfile.write(body)
        files = {"/": "index.html", "/index.html": "index.html", "/app.js": "app.js", "/style.css": "style.css"}
        if path not in files:
            return self.send_json({"error": "页面不存在"}, 404)
        file = TOOL_DIR / "web" / files[path]
        if not file.is_file():
            return self.send_json({"error": "界面文件缺失"}, 404)
        body = file.read_bytes()
        self.send_response(200)
        self.send_header("Content-Type", {".html": "text/html", ".js": "text/javascript", ".css": "text/css"}[file.suffix] + "; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.end_headers()
        self.wfile.write(body)

    def do_POST(self):
        if not self.local_request():
            return self.send_json({"error": "仅接受本地页面请求"}, 403)
        try:
            length = int(self.headers.get("Content-Length", 0))
            if length <= 0 or length > MAX_BYTES * 2:
                raise RecordingError("请求过大或缺少内容；文件上限 16 MB")
            request = json.loads(self.rfile.read(length))
            if not isinstance(request, dict):
                raise RecordingError("请求需要对象")
            if self.path == "/api/import":
                filename = str(request.get("filename", "记录"))[:200]
                if "base64" in request:
                    try:
                        data = base64.b64decode(request["base64"], validate=True)
                    except (ValueError, TypeError):
                        raise RecordingError("无法解码原始字节文件") from None
                    recording = decode_d6(data, filename)
                else:
                    recording = read_text_recording(request.get("text"), filename)
                if "params" in recording:
                    recording["params"] = validate_params(recording["params"], self.server.service.defaults)
                if "car_width_mm" in recording:
                    recording["car_width_mm"] = number(recording["car_width_mm"], "车体宽度", 0.001, self.server.service.layout["width"])
                if "safety_mm" in recording:
                    recording["safety_mm"] = number(recording["safety_mm"], "安全余量", 0, self.server.service.layout["width"])
                if "compare" in recording:
                    recording["compare"] = flag(recording["compare"], "compare")
                old_source = recording.get("source", {})
                if isinstance(old_source, dict) and old_source.get("files") and old_source["files"] != self.server.service.source["files"]:
                    recording["warnings"].append("问题帧导出时的固件与当前版本不同；当前重放结果可能改变。")
                if "pose" in request:
                    pose = normalize_pose(request["pose"])
                    for frame in recording["frames"]:
                        frame["pose"] = pose.copy()
                self.send_json(self.server.service.register(recording))
            elif self.path == "/api/replay":
                self.send_json(self.server.service.replay(request))
            elif self.path == "/api/export":
                output = request.get("recording")
                normalized = normalize_recording(output)
                if "params" in normalized:
                    validate_params(normalized["params"], self.server.service.defaults)
                body = json.dumps(output, ensure_ascii=False, allow_nan=False, separators=(",", ":")).encode("utf-8")
                if len(body) > MAX_BYTES:
                    raise RecordingError("问题记录超过 16 MB，请缩短历史记录后导出")
                index = normalized.get("problem_frame_index", len(normalized["frames"]) - 1)
                scan = normalized["frames"][index]["scan_num"]
                export_dir = getattr(self.server, "export_dir", TOOL_DIR / "exports")
                export_dir.mkdir(exist_ok=True)
                filename = f"lidar-scan-{scan}-{uuid.uuid4().hex[:12]}.json"
                file = export_dir / filename
                file.write_bytes(body)
                self.send_json({"filename": filename, "path": str(file), "url": f"/exports/{filename}"})
            else:
                self.send_json({"error": "接口不存在"}, 404)
        except (ValueError, TypeError, KeyError, OverflowError) as exc:
            self.send_json({"error": str(exc)}, 400)
        except (BrokenPipeError, ConnectionResetError):
            pass


class LocalHTTPServer(ThreadingHTTPServer):
    # Windows 的 SO_REUSEADDR 允许多个监听器同占端口，会把请求分给不同缓存。
    allow_reuse_address = False

    def server_bind(self):
        if os.name == "nt":
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
        super().server_bind()


def main():
    parser = argparse.ArgumentParser(description="雷达文件回放工具（本地运行，无外部依赖）")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--no-browser", action="store_true")
    args = parser.parse_args()
    service = ReplayService()
    try:
        server = LocalHTTPServer(("127.0.0.1", args.port), Handler)
    except OSError:
        # 用户再次双击启动时，打开已经运行的工具，不重复占用端口。
        url = f"http://127.0.0.1:{args.port}"
        try:
            with urlopen(url + "/api/info", timeout=2) as response:
                existing = json.load(response)
            if not all(key in existing for key in ("defaults", "layout", "source")):
                raise ValueError("unrelated service")
        except (OSError, ValueError):
            parser.exit(2, f"Port {args.port} is in use. Try --port {args.port + 1}.\n")
        print(f"LiDAR replay already running: {url}", flush=True)
        if not args.no_browser:
            webbrowser.open(url)
        return
    server.service = service
    url = f"http://127.0.0.1:{server.server_port}"
    print(f"LiDAR replay: {url}\nStop: Ctrl+C", flush=True)
    if not args.no_browser:
        threading.Timer(0.4, lambda: webbrowser.open(url)).start()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
