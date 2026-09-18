"""主机侧失败路径测试：不打开探针、不访问硬件。"""

import sys as _sys
from pathlib import Path as _Path

_sys.path.insert(0, str(_Path(__file__).resolve().parents[2] / "tools"))
_sys.path.insert(0, str(_Path(__file__).resolve().parents[2] / "tools" / "bench"))
import hashlib
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

import servo_hil_run as runner
from project_paths import ROOT
from servo_hil_emergency import disable_outputs


class OutputProbe:
    def __init__(self, stuck=False):
        self.registers = {0x40012C20: 0x1555, 0x40012C44: 0x8000}
        self.stuck = stuck
        self.is_halted = False

    def memory_read32(self, address, count):
        return [self.registers[address]]

    def memory_write32(self, address, values):
        if not self.stuck:
            self.registers[address] = values[0]

    def halt(self):
        # Stopping the CPU must not leave an energized static PWM vector.
        assert self.registers[0x40012C20] & 0x555 == 0
        assert self.registers[0x40012C44] & 0x8000 == 0
        self.is_halted = True

    def halted(self):
        return self.is_halted


class RecoveryTests(unittest.TestCase):
    def test_rtt_wait_handles_delayed_discovery_without_arming(self):
        probe = MagicMock()
        probe.rtt_get_num_up_buffers.side_effect = [RuntimeError("not yet found"), 1, 2]
        with patch.object(runner.time, "sleep"):
            runner.wait_for_rtt(probe)
        self.assertEqual(probe.rtt_get_num_up_buffers.call_count, 3)
        probe.memory_write32.assert_not_called()

    def test_rtt_wait_is_bounded(self):
        probe = MagicMock()
        probe.rtt_get_num_up_buffers.return_value = 0
        with (
            patch.object(runner.time, "monotonic", side_effect=[0, 0.1, 2.1]),
            patch.object(runner.time, "sleep"),
        ):
            with self.assertRaisesRegex(RuntimeError, "before ARM"):
                runner.wait_for_rtt(probe)

    def test_shutdown_disables_before_halt(self):
        result = disable_outputs(OutputProbe())
        self.assertTrue(result["cpu_halted"])
        self.assertEqual(result["ccer"], 0x1000)  # Preserve ADC trigger channel.

    def test_failed_disable_does_not_halt_energized_cpu(self):
        probe = OutputProbe(stuck=True)
        with self.assertRaises(RuntimeError):
            disable_outputs(probe)
        self.assertFalse(probe.halted())

    def test_disconnected_probe_during_stop_still_saves_trial(self):
        with tempfile.TemporaryDirectory() as folder:
            out = Path(folder)
            (out / "symbols.json").write_text(
                json.dumps({"servo_hil_mailbox": 0x20001000, "_SEGGER_RTT": 0x20002000})
            )
            image = out / "image"
            image.mkdir()
            axf = image / "Vector_Mini_ST.axf"
            axf.write_bytes(b"test-image")
            digest = hashlib.sha256(axf.read_bytes()).hexdigest()
            (out / "active_image.json").write_text(
                json.dumps({"directory": "image", "axf_sha256": digest})
            )
            profile = ROOT / "tests/hil/profiles/mode3/roll.json"
            with (
                patch.object(runner, "OUT", out),
                patch.object(
                    _sys,
                    "argv",
                    [
                        "runner",
                        "--name",
                        "failed",
                        "--session-dir",
                        str(out),
                        "--axis-profile",
                        str(profile),
                        "--bench-id",
                        "test-bench",
                        "--scenario",
                        "motion",
                        "--operator-confirmation",
                        "POWER_LIMITS_VERIFIED",
                        "--expected-firmware-sha256",
                        digest,
                        "--jlink-dll",
                        str(Path(__file__)),
                        "--probe-serial",
                        "1",
                    ],
                ),
                patch.object(runner, "pylink", MagicMock()) as library,
            ):
                probe = library.JLink.return_value
                probe.connect.side_effect = RuntimeError("probe disconnected")
                probe.opened.return_value = True
                probe.memory_write32.side_effect = RuntimeError("probe disconnected")
                probe.memory_read32.side_effect = RuntimeError("probe disconnected")
                with self.assertRaisesRegex(RuntimeError, "probe disconnected"):
                    runner.main()
            trial = json.loads((out / "failed/trial.json").read_text())
            self.assertIn("probe disconnected", trial["failure"])
            self.assertIn("probe disconnected", trial["shutdown_error"])
            self.assertFalse(trial["shutdown_verified"])
            self.assertTrue((out / "failed/capture.bin").exists())

    def test_wrong_image_hash_fails_before_probe_open(self):
        with tempfile.TemporaryDirectory() as folder:
            out = Path(folder)
            image = out / "image"
            image.mkdir()
            (image / "Vector_Mini_ST.axf").write_bytes(b"test-image")
            digest = hashlib.sha256((image / "Vector_Mini_ST.axf").read_bytes()).hexdigest()
            (out / "active_image.json").write_text(
                json.dumps({"directory": "image", "axf_sha256": digest})
            )
            (out / "symbols.json").write_text("{}")
            library = MagicMock()
            with (
                patch.object(
                    _sys,
                    "argv",
                    [
                        "runner",
                        "--name",
                        "bad-hash",
                        "--session-dir",
                        str(out),
                        "--axis-profile",
                        str(ROOT / "tests/hil/profiles/mode3/roll.json"),
                        "--bench-id",
                        "test-bench",
                        "--scenario",
                        "motion",
                        "--operator-confirmation",
                        "POWER_LIMITS_VERIFIED",
                        "--expected-firmware-sha256",
                        "0" * 64,
                        "--jlink-dll",
                        str(Path(__file__)),
                        "--probe-serial",
                        "1",
                    ],
                ),
                patch.object(runner, "pylink", library),
            ):
                with self.assertRaisesRegex(RuntimeError, "explicit firmware authorization"):
                    runner.main()
            library.JLink.assert_not_called()
            failure = json.loads((out / "bad-hash/preflight_failure.json").read_text())
            self.assertFalse(failure["hardware_contacted"])


if __name__ == "__main__":
    unittest.main()
