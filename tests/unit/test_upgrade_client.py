"""最小客户端对生产Loader内核的离线端到端验证；绝不加载硬件SDK。"""

from __future__ import annotations

import ctypes
import hashlib
import json
import shutil
import struct
import subprocess
import sys
import tempfile
import unittest
import zlib
from pathlib import Path
from queue import SimpleQueue
from threading import Event
from unittest.mock import MagicMock, patch

from host_app.upgrade_client import (
    UpgradeClient,
    UpgradePackage,
    crc8,
    crc16,
    decode_reply,
    encode_frames,
)
from loader.tools.factory_identity import encode_record
from tools.harness.common import zig_path
from tools.project_paths import NATIVE_INCLUDE_FLAGS

ROOT = Path(__file__).resolve().parents[2]


class NativeDevice:
    """DLL是生产Loader+内存Flash，CAN端口只转交帧；可丢弃指定回复。"""

    def __init__(self, dll: ctypes.CDLL) -> None:
        self.dll = dll
        self.queue: list[tuple[int, bytes]] = []
        self.drop = 0
        self.drop_kind: dict[int, int] = {}
        self.writes = 0
        dll.Device_Reset()

    def send(self, identifier: int, data: bytes) -> None:
        """调用真实接收路径，记录WRITE帧以检查拒绝前不曾擦写。"""
        self.writes += struct.unpack_from("<H", data, 6)[0] == 51
        output = (ctypes.c_uint8 * 64)()
        reply_id = ctypes.c_uint32()
        raw = (ctypes.c_uint8 * len(data)).from_buffer_copy(data)
        count = self.dll.Device_Exchange(identifier, raw, len(data), ctypes.byref(reply_id), output)
        if count:
            kind = struct.unpack_from("<H", data, 6)[0]
            if self.drop_kind.get(kind, 0):
                self.drop_kind[kind] -= 1
            elif self.drop:
                self.drop -= 1
            else:
                self.queue.append((reply_id.value, bytes(output[:count])))

    def receive(self, timeout_s: float) -> tuple[int, bytes] | None:
        """内存队列无需等待；无回复直接模拟超时。"""
        return self.queue.pop(0) if self.queue else None


class UpgradeClientTests(unittest.TestCase):
    """C/Python三端包与公司帧互验，覆盖故障恢复和身份边界。"""

    @classmethod
    def setUpClass(cls) -> None:
        """把硬件无关设备模型编译为DLL，所有生成物在outputs。"""
        compiler = zig_path()
        if compiler is None:
            raise unittest.SkipTest("原生编译器不可用；PR profile必须提供Zig")
        output = ROOT / "outputs/tests/upgrade_client"
        output.mkdir(parents=True, exist_ok=True)
        sources = [
            "tests/unit/loader_client_device.c",
            "loader/core/upgrade.c",
            "shared/image/image_transfer.c",
            "shared/boot/image_mailbox.c",
            "shared/boot/device_identity.c",
            "shared/checksum/crc32.c",
            "shared/protocol/yg_protocol_upgrade_wire.c",
            "shared/protocol/yg_protocol_can_id.c",
            "shared/protocol/yg_protocol_frame_codec.c",
            "shared/protocol/yg_protocol_crc.c",
            "shared/protocol/yg_protocol_fragment.c",
        ]
        binary = output / "device.dll"
        command = [str(compiler)] + (["cc"] if compiler.stem.lower() == "zig" else [])
        command += NATIVE_INCLUDE_FLAGS + [
            "-I",
            str(ROOT / "loader/core"),
            "-I",
            str(ROOT / "loader/ports/include"),
            "-I",
            str(ROOT / "loader/product"),
            "-I",
            str(ROOT / "shared/targets/vector_mini_st_g431"),
            "-shared",
            "-std=c99",
            "-Wall",
            "-Wextra",
            "-Werror",
            *[str(ROOT / source) for source in sources],
            "-o",
            str(binary),
        ]
        result = subprocess.run(command, capture_output=True, check=False)
        (output / "compile.log").write_bytes(result.stdout + result.stderr)
        if result.returncode:
            raise RuntimeError(result.stderr.decode(errors="replace"))
        cls.dll = ctypes.CDLL(str(binary))
        cls.dll.Device_Exchange.argtypes = [
            ctypes.c_uint32,
            ctypes.c_void_p,
            ctypes.c_uint32,
            ctypes.c_void_p,
            ctypes.c_void_p,
        ]
        cls.dll.Device_Exchange.restype = ctypes.c_uint32
        vector = json.loads(
            (ROOT / "tests/fixtures/loader_image_v3.json").read_text(encoding="utf-8")
        )
        cls.temp = tempfile.TemporaryDirectory(dir=ROOT / "outputs")
        cls.directory = Path(cls.temp.name)
        raw = cls.directory / "source.bin"
        image = bytearray.fromhex(vector["source_hex"])
        target = json.loads(
            (ROOT / "shared/targets/vector_mini_st_g431.json").read_text(encoding="utf-8")
        )["constants"]
        descriptor = int(target["TARGET_DESCRIPTOR_OFFSET"].rstrip("U"), 0)
        app_base = int(target["TARGET_APP_BASE"].rstrip("U"), 0)
        struct.pack_into("<I", image, 4, app_base + descriptor + 96 + 1)
        struct.pack_into(">I", image, descriptor + 12, int(target["TARGET_LAYOUT"].rstrip("U"), 0))
        struct.pack_into(
            ">I", image, descriptor + 28, int(target["TARGET_APP_VERSION"].rstrip("U"), 0)
        )
        struct.pack_into(
            ">I", image, descriptor + 36, int(target["TARGET_PARAM_ABI"].rstrip("U"), 0)
        )
        schema = int(target["TARGET_SCHEMA"].rstrip("U"), 0)
        struct.pack_into(">HH", image, descriptor + 40, schema, schema)
        raw.write_bytes(image)
        subprocess.run(
            [
                sys.executable,
                str(ROOT / "loader/tools/package_image.py"),
                "--app-bin",
                str(raw),
                "--token",
                "7",
                "--output",
                str(cls.directory / "package"),
            ],
            check=True,
            capture_output=True,
        )
        cls.package = UpgradePackage.load(cls.directory / "package")

    @classmethod
    def tearDownClass(cls) -> None:
        """释放临时包，不清理持久测试日志。"""
        cls.temp.cleanup()

    def test_real_loader_transfer_and_ack_loss(self) -> None:
        """真实C内核完成下载、激活和试运行；首个查询丢失不能中止重试。"""
        device = NativeDevice(self.dll)
        device.drop = 1
        client = UpgradeClient(device, 3, b"\x01" * 12)
        with patch("host_app.upgrade_client.time.sleep"):
            result = client.run(self.package)
        self.assertTrue(result["success"])
        self.assertGreater(device.writes, 0)
        self.assertTrue(any(item["attempt"] == 1 for item in client.trace))
        self.assertTrue(any(item["type"] == 57 for item in client.trace))

    def desktop_rounds(self, uid, stop_stage=None, incoming=(), targets=None, transport=None):
        """用生产 Loader C 内核运行桌面升级工作进程；仅替换适配器和测试包。"""
        with patch.object(sys, "path", [str(ROOT / "tools/bench"), *sys.path]):
            import can_upgrade_stress as stress

        device = transport
        if device is None:
            device = NativeDevice(self.dll)
            device.queue.extend(incoming)
            self.dll.Device_StartApp()
        stop, events = Event(), SimpleQueue()

        class Events:
            def put(self, msg):
                events.put(msg)
                if msg[0] == "stage" and stop_stage and stop_stage in msg[1]:
                    stop.set()

        with tempfile.TemporaryDirectory(dir=self.directory) as directory:
            out = Path(directory)
            bus = MagicMock()
            bus.__enter__.return_value = device
            settings = {
                "profile": "can_upgrade",
                "node": 3,
                "rounds": 3,
                "uid": uid.hex(),
                "bus": {},
                "package": str(out),
            }
            if targets is not None:
                settings["targets"] = targets
            with (
                patch.object(stress.UpgradePackage, "load", return_value=self.package),
                patch.object(stress, "controlcan_dll_path", return_value=out / "test.dll"),
                patch.object(stress.os, "add_dll_directory", return_value=MagicMock(), create=True),
                patch.object(stress, "ControlCanFd", return_value=bus),
                patch("time.sleep"),
            ):
                stress.run_upgrade_test(settings, out, Events(), stop)
            log = out / "rounds.jsonl"
            rows = (
                [json.loads(line) for line in log.read_text(encoding="utf-8").splitlines()]
                if log.exists()
                else []
            )
            return json.loads((out / "summary.json").read_text(encoding="utf-8")), rows, device

    def test_batch_confirms_both_devices_before_starting_next_round(self):
        binary = Path(self.dll._name)
        second = binary.with_name("batch-node7.dll")
        shutil.copyfile(binary, second)
        dll = ctypes.CDLL(str(second))
        dll.Device_Exchange.argtypes = self.dll.Device_Exchange.argtypes
        dll.Device_Exchange.restype = self.dll.Device_Exchange.restype
        devices = {3: NativeDevice(self.dll), 7: NativeDevice(dll)}
        dll.Device_SetNode(7)
        for device in devices.values():
            device.dll.Device_StartApp()
        bus = MagicMock()
        bus.send.side_effect = lambda identifier, data: devices[(identifier >> 8) & 255].send(
            identifier, data
        )
        bus.receive.side_effect = lambda _: next(
            (d.queue.pop(0) for d in devices.values() if d.queue), None
        )
        targets = [{"node": n, "uid": bytes([n - 2]).hex() * 12} for n in (7, 3)]
        report, rows, _ = self.desktop_rounds(b"", targets=targets, transport=bus)
        self.assertEqual(
            (report["planned"], report["passed"], report["status"]), (6, 6, "通过"), report
        )
        self.assertEqual(
            [(r["round"], r["node"]) for r in rows], [(n, k) for n in (1, 2, 3) for k in (3, 7)]
        )
        self.assertTrue(
            all(r["success"] and r["bytes_written"] == len(self.package.image) for r in rows)
        )
        self.assertTrue(all(r["timing"]["by_type"]["57"]["requests"] for r in rows))

    def test_batch_stops_on_missing_second_motor_and_retains_first_result(self):
        targets = [{"node": 3, "uid": "01" * 12}, {"node": 7, "uid": "05" * 12}]
        report, rows, _ = self.desktop_rounds(b"", targets=targets)
        self.assertEqual([r["node"] for r in rows], [3, 7])
        self.assertEqual(report["motors"]["3"]["passed"], 1)
        self.assertTrue(report["motors"]["7"]["identity_changed_or_missing"])
        self.assertEqual(rows[1]["bytes_written"], 0)
        report, rows, _ = self.desktop_rounds(b"", stop_stage="下载 APP", targets=targets)
        self.assertEqual(report["status"], "已停止")
        self.assertEqual([(r["node"], r["success"]) for r in rows], [(3, True)])
        self.assertEqual(report["motors"]["7"]["passed"], 0)

    def test_desktop_keeps_scan_and_selection_across_jobs(self):
        try:
            import tkinter as tk
        except ImportError:
            self.skipTest("Tk 未安装")
        with patch.object(sys, "path", [str(ROOT / "tools/bench"), *sys.path]):
            from canfd_desktop import CanTestWindow
        try:
            window = tk.Tk()
        except tk.TclError as exc:
            self.skipTest(str(exc))
        window.withdraw()
        self.addCleanup(window.destroy)
        ui = CanTestWindow(window)
        ui.tabs.select(3)
        motors = [
            dict(
                node=n,
                uid=bytes([n]).hex() * 12,
                role=1,
                state=5,
                faults=0,
                app_version=1538,
                loader_version=131584,
            )
            for n in (6, 7)
        ]
        ui.display_report(dict(profile="can_scan", motors=motors))
        ui.update_controls()
        for n in (6, 7):
            self.assertIn("在线", ui.upgrade_lights[n].cget("text"))
            ui.upgrade_checks[n].invoke()
        report = dict(
            profile="can_upgrade",
            status="通过",
            planned=2,
            rounds_per_motor=1,
            passed=2,
            failed=0,
            elapsed_s=1,
            version=1538,
            motors={str(m["node"]): dict(uid=m["uid"], passed=1, failed=0) for m in motors},
        )
        ui.display_report(report)
        ui.update_controls()
        self.assertEqual(set(ui.upgrade_targets), {6, 7})
        self.assertTrue(all(ui.upgrade_selected[n].get() for n in (6, 7)))
        self.assertFalse(ui.start_button.instate(["disabled"]))
        self.assertIn("1/1", ui.upgrade_results[6].cget("text"))
        report["motors"]["7"].update(passed=0, failed=1, identity_changed_or_missing=True)
        ui.display_report(report)
        ui.update_controls()
        self.assertEqual(set(ui.upgrade_targets), {6})
        self.assertTrue(ui.upgrade_checks[7].instate(["disabled"]))
        self.assertTrue(ui.upgrade_selected[6].get())
        ui.channel.set("2")
        self.assertFalse(ui.upgrade_targets)
        self.assertTrue(ui.start_button.instate(["disabled"]))

    def test_batch_rejects_duplicate_target_before_handoff(self):
        targets = [{"node": 3, "uid": "01" * 12}, {"node": 7, "uid": "01" * 12}]
        report, rows, device = self.desktop_rounds(b"", targets=targets)
        self.assertIn("重复", report["fatal"])
        self.assertEqual((report["attempted"], rows, device.writes), (0, [], 0))

    def test_desktop_rewrites_and_confirms_same_image_each_round(self):
        result, rows, _ = self.desktop_rounds(b"\x01" * 12)
        self.assertEqual(
            (result["status"], result["passed"], result["failed"]), ("通过", 3, 0), result
        )
        self.assertEqual([r["round"] for r in rows], [1, 2, 3])
        for row in rows:
            self.assertEqual(row["bytes_written"], len(self.package.image))
            self.assertTrue(row["success"])
            self.assertIn("57", row["timing"]["by_type"])

    def test_desktop_detects_polling_on_another_axis_before_handoff(self):
        incoming = encode_frames(7, 108, b"", 9)
        result, rows, device = self.desktop_rounds(b"\x01" * 12, incoming=incoming)
        self.assertIn("节点 7 发送 Type108", result["fatal"])
        self.assertEqual(rows[0]["trace"], [])
        self.assertEqual(rows[0]["bytes_written"], 0)
        self.assertEqual(device.writes, 0)

    def test_desktop_ignores_responses_during_passive_check(self):
        identifier, request = encode_frames(7, 108, b"", 9)[0]
        response_id = (identifier & ~0xFFFF) | (2 << 8) | 7
        result, _, _ = self.desktop_rounds(b"\x01" * 12, incoming=[(response_id, request)])
        self.assertEqual(result["passed"], 3)

    def test_desktop_stop_finishes_current_image(self):
        result, rows, _ = self.desktop_rounds(b"\x01" * 12, "下载 APP")
        self.assertEqual((result["status"], result["passed"], result["failed"]), ("已停止", 1, 0))
        self.assertEqual(len(rows), 1)
        self.assertTrue(rows[0]["success"])

    def test_scan_identifies_loader_without_starting_upgrade(self):
        with patch.object(sys, "path", [str(ROOT / "tools/bench"), *sys.path]):
            import can_upgrade_stress as stress

        device = NativeDevice(self.dll)
        with patch.object(device, "send", wraps=device.send) as send:
            motors, issues = stress.scan_upgrade_motors(device, lambda: False, lambda _: None)
        self.assertEqual(issues, [])
        self.assertEqual([(m["node"], m["uid"], m["role"]) for m in motors], [(3, "01" * 12, 2)])
        self.assertNotIn("state", motors[0])
        self.assertEqual(
            {struct.unpack_from("<H", c.args[1], 6)[0] for c in send.call_args_list}, {1}
        )

    def test_duplicate_node_uids_excluded_from_scan_and_upgrade(self):
        with patch.object(sys, "path", [str(ROOT / "tools/bench"), *sys.path]):
            import can_upgrade_stress as stress

        device = NativeDevice(self.dll)
        original_send = device.send

        def duplicate_reply(identifier, data):
            original_send(identifier, data)
            if device.queue:
                reply_id, response = device.queue[-1]
                altered = bytearray(response)
                altered[16] ^= 0x80
                struct.pack_into("<H", altered, 48, crc16(altered[:48]))
                device.queue.append((reply_id, bytes(altered)))

        with patch.object(device, "send", side_effect=duplicate_reply):
            motors, issues = stress.scan_upgrade_motors(device, lambda: False, lambda _: None)
        self.assertFalse(motors)
        self.assertEqual(len(issues), 1)
        self.assertIn("多个 UID", issues[0])
        with patch.object(
            stress, "read_node_uids", return_value={bytes([1]) * 12, bytes([3]) * 12}
        ):
            report, rows, _ = self.desktop_rounds(bytes([1]) * 12)
        self.assertIn("地址冲突", report["fatal"])
        self.assertEqual(rows[0]["bytes_written"], 0)

    def test_desktop_identity_mismatch_stops_batch(self):
        result, rows, device = self.desktop_rounds(b"\x02" * 12)
        self.assertEqual(
            (result["status"], result["attempted"], result["passed"], result["failed"]),
            ("失败", 1, 0, 1),
        )
        self.assertFalse(rows[0]["success"])
        self.assertEqual(device.writes, 0)

    def test_cli_requires_exclusive_bench_without_power_isolation_flag(self) -> None:
        """独占参数仍为必填；带电台架入口可进入SDK打开前的失败路径。"""
        output = self.directory / "cli_live_bench"
        command = [
            sys.executable,
            str(ROOT / "host_app/upgrade_client.py"),
            "upgrade",
            "--package",
            str(self.directory / "package"),
            "--sdk",
            str(self.directory / "missing-sdk.dll"),
            "--node",
            "3",
            "--uid",
            "01" * 12,
            "--output",
            str(output),
        ]
        without_exclusive = subprocess.run(command, capture_output=True, check=False)
        self.assertEqual(without_exclusive.returncode, 2)
        self.assertFalse(output.exists())
        with_exclusive = subprocess.run(
            command + ["--exclusive-bench"], capture_output=True, check=False
        )
        self.assertEqual(with_exclusive.returncode, 1)
        self.assertTrue((output / "summary.json").exists())
        self.assertFalse(
            json.loads((output / "summary.json").read_text(encoding="utf-8"))["success"]
        )

    def test_wrong_uid_never_writes(self) -> None:
        """错误板卡身份必须在BEGIN之前拒绝。"""
        device = NativeDevice(self.dll)
        with self.assertRaisesRegex(ValueError, "UID"):
            UpgradeClient(device, 3, bytes(12)).run(self.package)
        self.assertEqual(device.writes, 0)

    def test_portable_inspect_without_repository(self) -> None:
        """独立客户端只用随包契约；缺失契约在打开任何设备前清晰拒绝。"""
        with tempfile.TemporaryDirectory() as directory:
            portable = Path(directory) / "host"
            contracts = portable / "contracts"
            contracts.mkdir(parents=True)
            client = portable / "upgrade_client.py"
            shutil.copy2(ROOT / "host_app/upgrade_client.py", client)
            shutil.copy2(
                ROOT / "shared/targets/target_contract.py", portable / "target_contract.py"
            )
            target = contracts / "vector_mini_st_g431.json"
            shutil.copy2(ROOT / "shared/targets/vector_mini_st_g431.json", target)
            command = [
                sys.executable,
                "-I",
                str(client),
                "inspect",
                "--package",
                str(self.directory / "package"),
            ]
            result = subprocess.run(command, cwd=portable, capture_output=True, check=False)
            self.assertEqual(result.returncode, 0, result.stderr.decode(errors="replace"))
            self.assertEqual(json.loads(result.stdout)["bytes"], len(self.package.image))
            target.unlink()
            missing = subprocess.run(command, cwd=portable, capture_output=True, check=False)
            self.assertEqual(missing.returncode, 2)
            self.assertNotIn(b"Traceback", missing.stderr)

    def test_write_ack_loss_and_progress(self) -> None:
        """已编程块的回复丢失后只重发同一块，进度仍按持久偏移递增。"""
        device = NativeDevice(self.dll)
        device.drop_kind[51] = 1
        progress: list[tuple[int, int]] = []
        client = UpgradeClient(
            device, 3, b"\x01" * 12, lambda n, total: progress.append((n, total))
        )
        with patch("host_app.upgrade_client.time.sleep"):
            self.assertTrue(client.run(self.package)["success"])
        self.assertTrue(any(item["type"] == 51 and item["attempt"] == 1 for item in client.trace))
        self.assertEqual(progress[-1], (len(self.package.image), len(self.package.image)))
        self.assertEqual(progress, sorted(set(progress)))
        timing = client.timing_summary()["by_type"]["51"]
        self.assertEqual(timing["retries"], 1)
        self.assertEqual(timing["failed_requests"], 0)
        self.assertGreater(timing["transport_send_s"], 0)

    def test_fast_write_discards_delayed_ack_after_fragment_sequence_reuse(self):
        device = NativeDevice(self.dll)
        original_send = device.send
        previous_ack = None

        def send_with_delayed_ack(identifier, data):
            nonlocal previous_ack
            original_send(identifier, data)
            if data[6:8] == b"\x33\x00" and data[3] == 0xE0 and device.queue:
                current_ack = device.queue[-1]
                if previous_ack is not None:
                    device.queue.insert(0, previous_ack)
                previous_ack = current_ack

        client = UpgradeClient(device, 3, b"\x01" * 12, write_block_limit=72)
        with patch.object(device, "send", side_effect=send_with_delayed_ack), patch("time.sleep"):
            self.assertTrue(client.run(self.package)["success"])
        writes = client.timing_summary()["by_type"]["51"]
        expected = (len(self.package.image) + 71) // 72
        self.assertEqual(writes["requests"], expected)
        self.assertLessEqual(writes["frames_sent"], expected * 2)
        self.assertEqual(writes["retries"], 0)

    def test_compatibility_write_uses_one_frame_per_ack(self):
        client = UpgradeClient(NativeDevice(self.dll), 3, b"\x01" * 12, single_frame_writes=True)
        with patch("time.sleep"):
            self.assertTrue(client.run(self.package)["success"])
        writes = client.timing_summary()["by_type"]["51"]
        self.assertEqual(writes["requests"], (len(self.package.image) + 23) // 24)
        self.assertEqual(writes["requests"], writes["frames_sent"])

    def test_request_timing_separates_adapter_time(self) -> None:
        """注入时钟核对发送占用与回复等待分开，避免把调试器开销算成CAN速度。"""
        device = NativeDevice(self.dll)
        client = UpgradeClient(device, 3, b"\x01" * 12)
        with patch("host_app.upgrade_client.time.perf_counter", side_effect=[1, 1.1, 1.3, 1.6]):
            client.request(1, struct.pack("<IIHH", 0, 0, 0, 0))
        timing = client.timing_summary()["by_type"]["1"]
        self.assertAlmostEqual(timing["elapsed_s"], 0.6)
        self.assertAlmostEqual(timing["transport_send_s"], 0.2)
        self.assertAlmostEqual(timing["reply_wait_s"], 0.4)
        self.assertEqual(timing["frames_sent"], 1)

    def test_failed_request_is_counted_in_timing(self) -> None:
        """最终超时也计入总耗时与重试统计，不能只统计成功请求。"""
        device = NativeDevice(self.dll)
        device.drop = 4
        client = UpgradeClient(device, 3, b"\x01" * 12)
        with self.assertRaises(TimeoutError):
            client.request(1, struct.pack("<IIHH", 0, 0, 0, 0))
        timing = client.timing_summary()["by_type"]["1"]
        self.assertEqual(timing["failed_requests"], 1)
        self.assertEqual(timing["retries"], 3)
        self.assertEqual(timing["frames_sent"], 4)

    def test_lost_activation_ack_requires_new_app_confirmation(self) -> None:
        """ACTIVATE 回复已发却被主机丢弃，必须凭新 APP 版本/CRC/持久确认恢复。"""
        device = NativeDevice(self.dll)
        device.drop_kind[54] = 1
        client = UpgradeClient(device, 3, b"\x01" * 12)
        with patch("host_app.upgrade_client.time.sleep"):
            self.assertTrue(client.run(self.package)["success"])
        self.assertFalse(any(item["type"] == 54 for item in client.trace))
        self.assertTrue(any(item["type"] == 57 for item in client.trace))

    def test_lost_enter_ack_recovers_only_through_loader_identity(self) -> None:
        """旧APP交接桩的READY被丢弃，必须重新读到相同UID的配套Loader才继续。"""
        device = NativeDevice(self.dll)
        self.dll.Device_StartApp()
        device.drop_kind[56] = 1
        client = UpgradeClient(device, 3, b"\x01" * 12)
        with patch("host_app.upgrade_client.time.sleep"):
            self.assertTrue(client.run(self.package)["success"])
        self.assertFalse(any(item["type"] == 56 for item in client.trace))
        self.assertEqual(client.trace[0]["type"], 1)

    def test_rescue_hold_never_writes_before_identity_check(self) -> None:
        """救援捕获仅查询 STATUS，错误 UID 在驻留后仍不能开始擦写。"""
        device = NativeDevice(self.dll)
        client = UpgradeClient(device, 3, bytes(12))
        client.hold_rescue_window()
        self.assertEqual(device.writes, 0)
        with self.assertRaisesRegex(ValueError, "UID"):
            client.run(self.package)
        self.assertEqual(device.writes, 0)

    def test_compatible_new_app_version_does_not_require_new_host(self) -> None:
        """目标布局/ABI不变的新APP包按自身版本核对，不绑客户端构建版本。"""
        with tempfile.TemporaryDirectory(dir=self.directory) as directory:
            destination = Path(directory)
            image = bytearray(self.package.image)
            manifest = bytearray(self.package.manifest)
            version = self.package.version + 1
            struct.pack_into(">I", image, 0x200 + 28, version)
            struct.pack_into(">I", image, 0x200 + 48, 0)
            crc = zlib.crc32(image)
            struct.pack_into(">I", image, 0x200 + 48, crc)
            struct.pack_into(">I", manifest, 24, version)
            struct.pack_into(">I", manifest, 48, crc)
            metadata = json.loads((self.directory / "package/package.json").read_text())
            metadata.update(
                image_version=version,
                image_crc32=f"{crc:08x}",
                image_sha256=hashlib.sha256(image).hexdigest(),
                manifest_sha256=hashlib.sha256(manifest).hexdigest(),
            )
            (destination / "image.bin").write_bytes(image)
            (destination / "manifest.bin").write_bytes(manifest)
            (destination / "package.json").write_text(json.dumps(metadata))
            package = UpgradePackage.load(destination)
            self.assertEqual(package.version, version)
            self.assertEqual(package.crc32, crc)
            struct.pack_into(">I", manifest, 20, 999)
            (destination / "manifest.bin").write_bytes(manifest)
            with self.assertRaisesRegex(ValueError, "目标"):
                UpgradePackage.load(destination)

    def test_tampered_package_is_rejected(self) -> None:
        """包文件被改写不能靠JSON元数据通过检查。"""
        image_file = self.directory / "package/image.bin"
        original = image_file.read_bytes()
        changed = bytearray(original)
        changed[100] ^= 1
        image_file.write_bytes(changed)
        try:
            with self.assertRaisesRegex(ValueError, "CRC"):
                UpgradePackage.load(image_file.parent)
        finally:
            image_file.write_bytes(original)

    def test_reply_crc_and_padding(self) -> None:
        """回复必须匹配完整帧身份，正确CRC不能掩盖尾部非零或错误CAN ID。"""
        device = NativeDevice(self.dll)
        client = UpgradeClient(device, 3, b"\x01" * 12)
        frame = encode_frames(3, 53, b"", 12)[0]
        device.send(*frame)
        identifier, data = device.receive(0)
        self.assertEqual(len(decode_reply(identifier, data, 3, 53, 12)), 40)
        for changed_id, changed in (
            (identifier ^ 1, data),
            (identifier, data[:-1] + b"\x01"),
            (identifier, data[:15] + b"\x00" + data[16:]),
        ):
            with self.assertRaises(ValueError):
                decode_reply(changed_id, changed, 3, 53, 12)
        self.assertEqual(client.uid, b"\x01" * 12)

    def test_factory_record_golden(self) -> None:
        """生产工装记录与只读页面使用同一固定ASCII/CRC契约。"""
        record = encode_record("YG2026-ESC-001", 7)
        golden = json.loads(
            (ROOT / "tests/fixtures/device_identity_v1.json").read_text(encoding="utf-8")
        )
        self.assertEqual(record.hex(), golden["record_hex"])
        self.assertEqual(len(record), 64)
        self.assertEqual(record[:8].hex(), "3153475901000000")
        self.assertEqual(record[8:24], b"YG2026-ESC-001\x00\x00")
        self.assertEqual(record[24:28], struct.pack("<I", 7))
        for sn, kind in (("中文", 1), ("a" * 17, 1), ("", 1), ("OK", 0)):
            with self.assertRaises(ValueError):
                encode_record(sn, kind)

    def test_frame_golden(self) -> None:
        """帧CRC的标准字节计算与既有公司帧黄金向量保持一致。"""
        self.assertEqual(crc8(b"123456789"), 0xEA)
        self.assertEqual(crc16(b"123456789"), 0x932D)


if __name__ == "__main__":
    unittest.main()
