"""Host failure-path tests; no probe is opened and no hardware is accessed."""
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch, MagicMock

import servo_hil_run as runner
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
    def test_shutdown_disables_before_halt(self):
        result = disable_outputs(OutputProbe())
        self.assertTrue(result['cpu_halted'])
        self.assertEqual(result['ccer'], 0x1000)  # Preserve ADC trigger channel.

    def test_failed_disable_does_not_halt_energized_cpu(self):
        probe = OutputProbe(stuck=True)
        with self.assertRaises(AssertionError):
            disable_outputs(probe)
        self.assertFalse(probe.halted())

    def test_disconnected_probe_during_stop_still_saves_trial(self):
        with tempfile.TemporaryDirectory() as folder:
            out = Path(folder)
            (out/'symbols.json').write_text(json.dumps({'servo_hil_mailbox': 0x20001000,
                                                       '_SEGGER_RTT': 0x20002000}))
            with patch.object(runner, 'OUT', out), \
                 patch.object(runner.sys, 'argv', ['runner', '--name', 'failed']), \
                 patch.object(runner, 'pylink', MagicMock()) as library:
                probe = library.JLink.return_value
                probe.connect.side_effect = RuntimeError('probe disconnected')
                probe.opened.return_value = True
                probe.memory_write32.side_effect = RuntimeError('probe disconnected')
                probe.memory_read32.side_effect = RuntimeError('probe disconnected')
                with self.assertRaisesRegex(RuntimeError, 'probe disconnected'):
                    runner.main()
            trial = json.loads((out/'failed/trial.json').read_text())
            self.assertIn('probe disconnected', trial['failure'])
            self.assertIn('probe disconnected', trial['shutdown_error'])
            self.assertFalse(trial['shutdown_verified'])
            self.assertTrue((out/'failed/capture.bin').exists())


if __name__ == '__main__':
    unittest.main()
