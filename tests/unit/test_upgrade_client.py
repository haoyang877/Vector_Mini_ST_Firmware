"""最小客户端对生产Loader内核的离线端到端验证；绝不加载硬件SDK。"""

from __future__ import annotations

import ctypes
import json
import struct
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

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
            str(ROOT / "loader/ports/stm32g4"),
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
        raw.write_bytes(bytes.fromhex(vector["source_hex"]))
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

    def test_wrong_uid_never_writes(self) -> None:
        """错误板卡身份必须在BEGIN之前拒绝。"""
        device = NativeDevice(self.dll)
        with self.assertRaisesRegex(ValueError, "UID"):
            UpgradeClient(device, 3, bytes(12)).run(self.package)
        self.assertEqual(device.writes, 0)

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
