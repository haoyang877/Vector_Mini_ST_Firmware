"""HEX选择入口的离线检查；坏输入不允许打开探针，真实初装写入仍复用原函数。"""

import json
from pathlib import Path
import struct
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "loader/tools"))
import loader_factory as factory  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_factory_package import LOADER, MANIFEST, PARTS, Probe, UID  # noqa: E402


class HexTests(unittest.TestCase):
    """保证HEX到三段受保护写入模型的边界和记录一致。"""

    def test_valid_hex_matches_package_and_programs_only_unprotected_pages(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "combined.hex"
            path.write_text(factory.combined_hex(PARTS), encoding="ascii")
            parts, manifest = factory.load_hex(path)
            self.assertEqual(parts, PARTS)
            self.assertEqual(manifest["app"], MANIFEST)
            probe = Probe()
            before = bytes(probe.data)
            report = factory.program_probe(probe, parts, UID, Path(directory))
            self.assertEqual(report["status"], "PASS")
            self.assertEqual(probe.data[0x3800:0x4000], before[0x3800:0x4000])
            self.assertEqual(probe.data[0x1C000:], before[0x1C000:])

    def test_rejects_checksum_truncation_overlap_and_eof_errors(self):
        good = factory.combined_hex(PARTS)
        lines = good.splitlines()
        bad = (
            good.replace(lines[1][-2:], "00", 1),
            "\n".join(lines[:-1]),
            "\n".join(lines[:2] + [lines[1]] + lines[2:]),
            "\n".join(lines[:2] + lines[3:]),
            good + lines[1] + "\n",
            good.replace(lines[1], lines[1][:-2], 1),
        )
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "bad.hex"
            for text in bad:
                path.write_text(text, encoding="ascii")
                with self.assertRaises(ValueError):
                    factory.load_hex(path)

    def test_rejects_protected_bytes_wrong_record_crc_vectors_and_padding(self):
        app_crc = bytearray(PARTS[2][1])
        app_crc[8] ^= 1
        padding = bytearray(PARTS[1][1])
        padding[49] = 0
        bad_loader = struct.pack("<2I", 0x20008000, factory.APP_BASE + 9) + LOADER[8:]
        bad_sets = (
            PARTS + ((factory.PARAM_BASE, b"\xff"),),
            PARTS + ((factory.RECORD_BASE + factory.PAGE_BYTES, b"\xff"),),
            (PARTS[0], PARTS[1], (factory.APP_BASE, bytes(app_crc))),
            (PARTS[0], (factory.RECORD_BASE, bytes(padding)), PARTS[2]),
            (
                (factory.FLASH_BASE, bad_loader.ljust(factory.LOADER_BYTES, b"\xff")),
                PARTS[1],
                PARTS[2],
            ),
        )
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "bad.hex"
            for parts in bad_sets:
                path.write_text(factory.combined_hex(parts), encoding="ascii")
                with self.assertRaises(ValueError):
                    factory.load_hex(path)

    def test_bad_input_or_changed_file_never_imports_probe_library(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "bad.hex"
            args = SimpleNamespace(hex=path, package=None, expected_hex_sha256="0" * 64)
            for text in (":00000001FF\n", factory.combined_hex(PARTS)):
                path.write_text(text, encoding="ascii")
                with (
                    patch.dict(sys.modules, {"pylink": None}),
                    self.assertRaises(ValueError),
                ):
                    factory.flash_package(args)

    def test_hex_flash_entry_preserves_run_policy_and_records_exact_source(self):
        probe = Probe()
        commands = []
        probe.open = lambda serial_no: None
        probe.exec_command = commands.append
        probe.set_tif = lambda interface: None
        probe.connect = lambda device, speed: None
        probe.close = lambda: None
        module = SimpleNamespace(
            JLink=lambda lib: probe,
            enums=SimpleNamespace(JLinkInterfaces=SimpleNamespace(SWD=1)),
        )
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "firmware.hex"
            path.write_text(factory.combined_hex(PARTS), encoding="ascii")
            args = SimpleNamespace(
                hex=path,
                package=None,
                uid=UID.hex(),
                jlink_serial=123,
                speed_khz=100,
                out=Path(directory) / "flash",
                pylink_dir=None,
                jlink_dll=None,
                run=True,
            )
            with (
                patch.dict(sys.modules, {"pylink": module}),
                patch.object(factory.time, "sleep"),
            ):
                report = factory.flash_package(args)
            self.assertEqual(
                report["firmware_source_sha256"], factory.sha256(path.read_bytes())
            )
            self.assertEqual(commands[-1], "SetRestartOnClose = 1")
            self.assertEqual(
                json.loads((args.out / "summary.json").read_text())["status"], "PASS"
            )


if __name__ == "__main__":
    unittest.main()
