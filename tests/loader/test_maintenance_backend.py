"""维护工具离线验证：真实参数ABI、损坏记录、目标变化及升级保护，不打开硬件。"""

import argparse
import hashlib
import json
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch
from types import SimpleNamespace
import zlib

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "loader/tools"))
import maintenance_backend as backend  # noqa: E402


def parameter_fixture() -> bytes:
    """生成明确schema10黄金字段，尾记录CRC使用独立zlib实现。"""
    data = bytearray(0x4000)
    struct.pack_into("<9f", data, 0, 4, 2001, 2002, 2003, 21, 2.2, 0.01, 0.02, 0.03)
    struct.pack_into("<HHBB", data, 36, 32768, 16384, 7, 0)
    struct.pack_into("<1024h", data, 44, *range(-512, 512))
    struct.pack_into("<II", data, 2160, 10, 0x454E4332)
    struct.pack_into("<I", data, 2172, 6)
    struct.pack_into("<5f", data, 2188, 3.1, 3.2, 0.03, 0.04, 0)
    struct.pack_into("<I", data, 2204, 1)
    struct.pack_into("<2f", data, 2144, 5, 8)
    for offset, prefix in (
        (2208, struct.pack("<II8sfff", 0x31535841, 1, b"pitch", -0.3, 0.9, 1.5)),
        (2240, struct.pack("<III", 0x31504D44, 1, 3)),
        (2256, struct.pack("<III", 0x31465650, 1, 1)),
        (2272, struct.pack("<IIff", 0x31505350, 1, 0.06, 0.6)),
    ):
        data[offset : offset + len(prefix) + 4] = prefix + struct.pack(
            "<I", zlib.crc32(prefix)
        )
    return bytes(data)


def upgrade_fixture(root: Path) -> Path:
    """独立生成有效的小APP包，不依赖旧outputs产物。"""
    image = struct.pack("<II", 0x20007000, 0x08004011) + bytes(range(56))
    manifest = {
        "app_base": 0x08004000,
        "layout_id": 0x4F564C31,
        "hardware_id": 1,
        "param_abi": 0x4F563130,
        "protocol_version": 2,
        "app_version": 0x00050000,
        "image_bytes": len(image),
        "image_crc32": zlib.crc32(image),
        "image_sha256": hashlib.sha256(image).hexdigest(),
    }
    (root / "app.bin").write_bytes(image)
    path = root / "manifest.json"
    path.write_text(json.dumps(manifest), encoding="utf-8")
    return path


class MaintenanceTests(unittest.TestCase):
    """纯计算及IO替身验证，全部测试保持hardware_accessed=false。"""

    def test_calibration_units_and_independent_records(self):
        report = backend.decode_calibration(parameter_fixture())
        rows = {name: (value, unit) for _, name, value, unit in report["rows"]}
        self.assertEqual(rows["A相零偏"], (2001, "ADC count"))
        self.assertEqual(rows["电零位"], (180, "°"))
        self.assertEqual(rows["机械零位"], (90, "°"))
        self.assertEqual(rows["位置速度滤波"], (5, "Hz"))
        self.assertEqual(rows["摩擦前馈"][0], "开启")
        self.assertEqual(rows["电机类型"][0], "pitch")
        self.assertEqual(len(report["lut_deg"]), 1024)
        self.assertAlmostEqual(rows["位置内环速度 Ki"][0], 0.6)
        self.assertAlmostEqual(rows["正向粘性摩擦"][0], 0.03 * 3.141592653589793 / 180)

    def test_unknown_schema_blank_and_corrupt_tail(self):
        self.assertEqual(
            backend.decode_calibration(b"\xff" * 0x4000)["state"], "未配置"
        )
        data = bytearray(parameter_fixture())
        data[2264] ^= 1
        report = backend.decode_calibration(bytes(data))
        rows = {name: value for _, name, value, _ in report["rows"]}
        self.assertEqual(rows["速度滤波记录"], "格式 / CRC无效")
        self.assertNotIn("位置速度滤波", rows)
        struct.pack_into("<I", data, 2160, 9)
        with self.assertRaises(ValueError):
            backend.decode_calibration(bytes(data))

    def test_bad_flags_and_nan_are_not_presented_as_valid(self):
        for offset, raw in ((41, b"\x02"), (4, struct.pack("<f", float("nan")))):
            data = bytearray(parameter_fixture())
            data[offset : offset + len(raw)] = raw
            with self.assertRaises(ValueError):
                backend.decode_calibration(bytes(data))

    def test_manifest_hash_and_version_rejected_before_open(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            path = upgrade_fixture(root)
            _, _, digest = backend.load_upgrade(path)
            (root / "app.bin").write_bytes(bytes(64))
            with patch.object(backend, "LoaderBus") as bus:
                with self.assertRaises(ValueError):
                    backend.run_can_upgrade({}, {}, path, digest, root / "job")
                bus.assert_not_called()

    def test_query_protocol_and_loader_are_read_only(self):
        with (
            patch.object(backend, "app_uid", return_value=bytes(12)),
            patch.object(backend, "app_value", return_value=1) as value,
        ):
            with self.assertRaises(ValueError):
                backend.query_device(None, 0, 4)
            self.assertEqual(value.call_args.args[-1], 0x67)
        with (
            patch.object(backend, "app_uid", side_effect=TimeoutError),
            patch.object(backend, "Client") as client_type,
        ):
            client = client_type.return_value
            client.query.side_effect = [
                bytes(12),
                struct.pack(">BBH4I", 0, 0, 0, 0, 64, 123, 0),
            ]
            client.check_target.return_value = 0x01000000
            client.parameter_crc = 123
            report = backend.query_device(None, 0, 4)
            self.assertEqual(report["runtime"], "Loader")
            client.request.assert_not_called()

    def test_flash_snapshot_preserves_cpu_and_rejects_unstable_read(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            data = bytes(0x1C000) + parameter_fixture()
            for halted, unstable in ((False, False), (True, False), (False, True)):
                commands = []
                reads = [data, bytes(len(data)) if unstable else data]

                class Probe:
                    def open(self, **_):
                        pass

                    def close(self):
                        pass

                    def exec_command(self, command):
                        commands.append(command)

                    def set_tif(self, *_):
                        pass

                    def connect(self, *_, **__):
                        pass

                    def halted(self):
                        return halted

                    def memory_read16(self, *_):
                        return [128]

                    def memory_read32(self, address, _):
                        return {
                            0xE0042000: [0x468],
                            0x1FFF7590: [1, 2, 3],
                            0xE000ED08: [0x08004000],
                            0xE000ED28: [0, 0],
                        }[address]

                    def memory_read8(self, *_):
                        return reads.pop(0)

                fake = SimpleNamespace(
                    JLink=lambda **_: Probe(),
                    library=SimpleNamespace(Library=lambda **_: None),
                    enums=SimpleNamespace(JLinkInterfaces=SimpleNamespace(SWD=1)),
                )
                out = root / f"{halted}_{unstable}"
                with patch.dict(sys.modules, {"pylink": fake}):
                    if unstable:
                        with self.assertRaises(RuntimeError):
                            backend.flash_snapshot(Path("unused.dll"), 1, 1000, out)
                    else:
                        report = backend.flash_snapshot(
                            Path("unused.dll"), 1, 1000, out
                        )
                        self.assertEqual(report["cpu_running"], not halted)
                self.assertEqual(
                    commands[-1], "SetRestartOnClose = " + ("0" if halted else "1")
                )
                report = json.loads((out / "summary.json").read_text())
                self.assertEqual(report["status"], "FAIL" if unstable else "PASS")

    def run_upgrade(
        self,
        runtime="APP",
        mode=0,
        changed_uid=False,
        after_crc=123,
        loader_crc=123,
        session=0,
        close_failure=False,
        progress=False,
    ):
        """显式注入协议边界结果，不模拟或接触真实硬件。"""
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            path = upgrade_fixture(root)
            _, _, digest = backend.load_upgrade(path)
            binding = {
                "uid": bytes(range(12)).hex(),
                "adapter": {"serial": "TEST_ONLY"},
            }
            current = {
                "uid": "ab" * 12 if changed_uid else binding["uid"],
                "runtime": runtime,
                "mode": mode,
                "parameter_crc": 123,
                "session": session,
                "loader_state": 0,
            }
            settings = {"dll": "unused.dll", "channel": 0, "index": 0, "node": 4}
            with (
                patch.object(backend, "LoaderBus") as constructor,
                patch.object(backend, "query_device", return_value=current),
                patch.object(backend, "Client") as client_type,
                patch.object(backend, "app_query") as app_request,
                patch.object(backend, "app_ready"),
                patch.object(backend, "app_parameter_crc", return_value=after_crc),
                patch.object(backend, "app_value", return_value=0),
            ):
                client = client_type.return_value
                client.parameter_crc = loader_crc
                client.trace = []
                client.update.return_value = {"activation_acknowledged": True}
                if close_failure:
                    constructor.return_value.close.side_effect = OSError("close failed")
                failure = None
                try:
                    backend.run_can_upgrade(
                        settings,
                        binding,
                        path,
                        digest,
                        root / "job",
                        (lambda *_: None) if progress else None,
                    )
                except (RuntimeError, OSError) as exc:
                    failure = str(exc)
                report = json.loads((root / "job/summary.json").read_text())
                return report, failure, client.update.call_count, app_request.call_count

    def test_upgrade_success_and_parameter_change_failure(self):
        report, failure, count, _ = self.run_upgrade(progress=True)
        self.assertEqual((report["status"], failure, count), ("PASS", None, 1))
        report, failure, count, _ = self.run_upgrade(after_crc=456)
        self.assertEqual(report["status"], "FAIL")
        self.assertIn("参数CRC改变", failure)
        self.assertEqual(count, 1)

    def test_identity_moving_and_active_loader_reject_before_erase(self):
        for kwargs in (
            {"changed_uid": True},
            {"mode": 3},
            {"runtime": "Loader", "session": 1},
        ):
            report, failure, count, requests = self.run_upgrade(**kwargs)
            self.assertEqual(report["status"], "FAIL")
            self.assertIsNotNone(failure)
            self.assertEqual((count, requests), (0, 0))

    def test_loader_parameter_change_and_close_failure_are_failures(self):
        report, _, count, _ = self.run_upgrade(loader_crc=456)
        self.assertEqual((report["status"], count), ("FAIL", 0))
        report, failure, _, _ = self.run_upgrade(close_failure=True)
        self.assertEqual(report["status"], "FAIL")
        self.assertIn("close failed", failure)


def check_c_abi(cc: str, out: Path):
    """使用真实参数头文件核对读取偏移，不复制C结构定义。"""
    out.mkdir(parents=True, exist_ok=True)
    (out / "main.h").write_text(
        "#include <stdint.h>\n#include <stdbool.h>\n", encoding="utf-8"
    )
    (out / "encoder.h").write_text(
        "#define ENCODER_OFFSET_LUT_SIZE 1024U\n", encoding="utf-8"
    )
    (out / "hw_conf.h").write_text("", encoding="utf-8")
    expected = {
        "node_id": 0,
        "currentoffset_a": 4,
        "currentoffset_b": 8,
        "currentoffset_c": 12,
        "motor_pole_pairs": 16,
        "motor_phase_resistance": 20,
        "motor_d_inductance": 24,
        "motor_q_inductance": 28,
        "motor_flux": 32,
        "encoder_electrical_zero_q15": 36,
        "encoder_mechanical_zero_q15": 38,
        "encoder_calib_flag": 40,
        "encoder_reverse": 41,
        "encoder_linearization_lut_q15": 44,
        "id_kp": 2092,
        "id_ki": 2096,
        "iq_kp": 2100,
        "iq_ki": 2104,
        "speed_kp": 2116,
        "speed_ki": 2120,
        "pos_kp": 2136,
        "pos_kd": 2140,
        "calib_current": 2144,
        "current_limit": 2148,
        "schema_version": 2160,
        "magic_word": 2164,
        "pos_ki": 2168,
        "current_sense_shunt_milliohm": 2172,
        "friction_coulomb_pos_a": 2188,
        "friction_coulomb_neg_a": 2192,
        "friction_viscous_pos_a_per_rad_s": 2196,
        "friction_viscous_neg_a_per_rad_s": 2200,
        "friction_model_valid": 2204,
        "axis_profile": 2208,
        "load_profile": 2240,
        "position_filter": 2256,
        "position_speed_pi": 2272,
    }
    lines = [
        "#include <stddef.h>",
        "#include <stdio.h>",
        '#include "foc_param.h"',
        "int main(void) {",
        'puts("{");',
    ]
    for index, field in enumerate(expected):
        comma = "," if index + 1 < len(expected) else ""
        lines.append(
            f'printf("\\"{field}\\":%zu{comma}\\n", offsetof(InterfaceParam_TypeDef, {field}));'
        )
    lines.extend(['puts("}");', "return 0;", "}"])
    source = out / "abi.c"
    source.write_text("\n".join(lines), encoding="utf-8")
    executable = out / "abi.exe"
    command = [cc] + (["cc"] if Path(cc).stem == "zig" else [])
    command += [
        "-std=c99",
        "-Wall",
        "-Wextra",
        "-Werror",
        "-I",
        str(out),
        "-I",
        "firmware/services/parameters",
        "-I",
        "firmware/motor/position",
        "-I",
        "firmware/motor",
        "-I",
        "firmware/motor/protection",
        str(source),
        "-o",
        str(executable),
    ]
    subprocess.run(command, cwd=ROOT, check=True, capture_output=True)
    actual = json.loads(subprocess.check_output([str(executable)], text=True))
    if actual != expected:
        raise RuntimeError("当前C参数ABI与维护工具读取偏移不一致")
    (out / "offsets.json").write_text(json.dumps(actual, indent=2), encoding="utf-8")
    print("PASS calibration offsets match actual C parameter header")


def main():
    """默认仅离线；可选本地编译器核对实际C参数布局。"""
    parser = argparse.ArgumentParser()
    parser.add_argument("--cc")
    parser.add_argument(
        "--out", type=Path, default=ROOT / "outputs/tests/maintenance_abi"
    )
    args = parser.parse_args()
    if args.cc:
        check_c_abi(args.cc, args.out)
    result = unittest.TextTestRunner().run(
        unittest.defaultTestLoader.loadTestsFromTestCase(MaintenanceTests)
    )
    return 0 if result.wasSuccessful() else 1


if __name__ == "__main__":
    raise SystemExit(main())
