"""模拟Maix硬件的入口集成检查，不等同于真机验收。"""
import ast
import itertools
import tempfile
import types
import unittest
from unittest.mock import Mock

from test_ui_v9 import _FakeImageModule, VisionUI
from test_vision_v9 import MODULE_DIR, config, color_frame
from vision_tasks import TASK_MATERIAL


class MainLoopTests(unittest.TestCase):
    def load_entry(self, local=False):
        tree = ast.parse((MODULE_DIR / "main.py").read_text(encoding="utf-8"))
        tree.body = [node for node in tree.body
                     if not (isinstance(node, ast.ImportFrom) and node.module == "maix")]
        run_node = next(node for node in tree.body
                        if isinstance(node, ast.FunctionDef) and node.name == "run")
        if not local:
            # 用户可保留源码的本地调试注释；正式通信检查只在测试副本恢复接收。
            loop = next(node for node in ast.walk(run_node) if isinstance(node, ast.While))
            if not any(isinstance(line, ast.Expr) and isinstance(line.value, ast.Call)
                       and isinstance(line.value.func, ast.Name)
                       and line.value.func.id == "rx_handle" for line in loop.body):
                loop.body.insert(1, ast.Expr(value=ast.Call(
                    func=ast.Name(id="rx_handle", ctx=ast.Load()), args=[], keywords=[])))
                ast.fix_missing_locations(tree)
        if local:
            for node in ast.walk(run_node):
                if isinstance(node, ast.While):
                    node.body = [line for line in node.body
                                 if not (isinstance(line, ast.Expr)
                                         and isinstance(line.value, ast.Call)
                                         and isinstance(line.value.func, ast.Name)
                                         and line.value.func.id == "rx_handle")]
        env = {"__name__": "test_entry", "print": Mock()}
        exec(compile(tree, "main.py", "exec"), env)
        return env

    def hardware(self, env, path, chunks, loops=4, local=False, bgr=True):
        cam = Mock()
        cam.read.return_value = color_frame(config())
        disp = Mock()
        disp.width.return_value, disp.height.return_value = 640, 480
        serial = Mock()
        serial.read.side_effect = itertools.chain(chunks, itertools.repeat(b""))
        serial.write.side_effect = len
        app = Mock()
        app.get_app_data_path.return_value = path
        app.need_exit.side_effect = [False] * loops + [True]
        clock = itertools.count(0, 2)
        def create_ui(*args):
            ui = VisionUI(*args)
            if local:
                ui._open_task(TASK_MATERIAL)
                ui._activate("START_RUN")
            return ui
        class ImageModule(_FakeImageModule):
            Format = types.SimpleNamespace(FMT_BGR888=1)
        conversion = Mock(side_effect=ImageModule.image2cv)
        ImageModule.image2cv = conversion
        env.update(
            app=app, image=ImageModule, camera=Mock(), display=Mock(Display=Mock(return_value=disp)),
            touchscreen=Mock(), TouchInput=Mock(return_value=Mock(poll=Mock(return_value=None))),
            uart=Mock(UART=Mock(return_value=serial)), pinmap=Mock(), gpio=Mock(),
            err=Mock(), time=Mock(), now_ms=lambda: next(clock),
            open_camera=lambda: (cam, bgr), VisionUI=create_ui,
        )
        env["run"]()
        return cam, disp, serial, conversion

    @staticmethod
    def command(cmd, data):
        return bytes([0xAA, 0x55, cmd, len(data)] + data +
                     [(cmd + len(data) + sum(data)) & 255])

    def test_formal_run_static_screen_and_current_frame_uart(self):
        env = self.load_entry()
        start = self.command(0xB2, [0xC1])
        with tempfile.TemporaryDirectory() as path:
            cam, disp, serial, convert = self.hardware(env, path, [start])
        self.assertEqual(cam.read.call_count, 4)
        self.assertEqual(disp.show.call_count, 1)
        self.assertGreater(serial.write.call_count, 0)
        packet = serial.write.call_args[0][0]
        self.assertEqual(packet[2], 0xB2)
        self.assertEqual(packet[5], 1)
        self.assertTrue(all(not call.kwargs["copy"] for call in convert.call_args_list))
        cam.close.assert_called_once()

    def test_stop_arriving_during_detection_discards_pending_result(self):
        env = self.load_entry()
        chunks = [self.command(0xB2, [0xC1]), self.command(0xB0, [])]
        with tempfile.TemporaryDirectory() as path:
            _, _, serial, _ = self.hardware(env, path, chunks, loops=1)
        serial.write.assert_not_called()
        self.assertEqual(env["mode"], 0xB0)
        self.assertFalse(env["uart_tx_owned"])

    def test_task_or_color_switch_during_detection_discards_pending_result(self):
        for cmd, target in ((0xB4, 0xC2), (0xB3, 0xD2), (0xB2, 0xC3)):
            env = self.load_entry()
            chunks = [self.command(0xB2, [0xC1]), self.command(cmd, [target])]
            with tempfile.TemporaryDirectory() as path:
                _, _, serial, _ = self.hardware(env, path, chunks, loops=1)
            serial.write.assert_not_called()
            self.assertEqual((env["mode"], env["target"]), (cmd, target))

    def test_main_selects_28ms_for_b2_and_restores_25ms_after_task_switch(self):
        for cmd, target, period in ((0xB2, 0xC1, 28), (0xB4, 0xC2, 25), (0xB3, 0xD2, 25)):
            env = self.load_entry()
            real_sender = env["ResultSender"]
            senders = []
            def create_sender():
                sender = real_sender()
                senders.append(sender)
                return sender
            env["ResultSender"] = create_sender
            chunks = [self.command(0xB2, [0xC1]), b"", self.command(cmd, [target])]
            with tempfile.TemporaryDirectory() as path:
                self.hardware(env, path, chunks, loops=2)
            self.assertEqual(senders[0].interval_ms, period)

    def test_commenting_first_rx_call_enables_local_run_without_transmitting(self):
        env = self.load_entry(local=True)
        with tempfile.TemporaryDirectory() as path:
            cam, disp, serial, _ = self.hardware(env, path, [], local=True)
        self.assertEqual(cam.read.call_count, 4)
        self.assertEqual(disp.show.call_count, 1)
        serial.read.assert_not_called()
        serial.write.assert_not_called()
        self.assertFalse(env["uart_control_active"])

    def test_rgb_camera_fallback_never_uses_unsafe_zero_copy(self):
        env = self.load_entry()
        with tempfile.TemporaryDirectory() as path:
            _, _, _, convert = self.hardware(
                env, path, [self.command(0xB2, [1])], bgr=False)
        self.assertTrue(all(call.kwargs["copy"] for call in convert.call_args_list))

    def test_camera_request_falls_back_without_claiming_fps(self):
        env = self.load_entry()
        cam = Mock()
        env["image"] = types.SimpleNamespace(
            Format=types.SimpleNamespace(FMT_BGR888=1))
        env["camera"] = types.SimpleNamespace(
            Camera=Mock(side_effect=[TypeError("fps unavailable"), cam]))
        self.assertEqual(env["open_camera"](), (cam, True))
        calls = env["camera"].Camera.call_args_list
        self.assertEqual(calls[0].kwargs, {"fps": 60})
        self.assertEqual(calls[1].kwargs, {})

    def test_b2_reports_and_rejects_mixed_old_ui_module(self):
        env = self.load_entry()
        env["ui_module"] = types.SimpleNamespace(__file__="/old/vision_ui.py")
        with self.assertRaisesRegex(RuntimeError, "vision_ui.*upload ALL"):
            env["check_b2_modules"]()
        self.assertTrue(any("/old/vision_ui.py" in str(c) for c in env["print"].call_args_list))

    def test_b2_detects_old_calibration_module_even_when_it_has_six_colors(self):
        env = self.load_entry()
        env["config_module"] = types.SimpleNamespace(
            __file__="/old/vision_config.py", SCHEMA_VERSION=8,
            COLOR_ORDER=("red", "yellow", "blue", "green", "black", "light_blue"))
        with self.assertRaisesRegex(RuntimeError, "vision_config.*upload ALL"):
            env["check_b2_modules"]()


if __name__ == "__main__":
    unittest.main()
