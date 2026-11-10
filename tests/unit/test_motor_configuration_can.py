"""使用真实参数实现和替身 Flash 验证 CAN 型号配置事务，不访问硬件。"""

import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
from harness.common import zig_path
from project_paths import NATIVE_INCLUDE_FLAGS, ROOT


class MotorConfigurationCanTest(unittest.TestCase):
    def test_configuration_transaction_on_both_boards(self) -> None:
        compiler = zig_path()
        self.assertIsNotNone(compiler, "需要仓库锁定的 Zig 工具链")
        sources = [
            "tests/unit/native/motor_configuration_can_test.c",
            "firmware/app/app_motor_configuration.c",
            "firmware/parameters/parameter_record.c",
            "firmware/parameters/motor_axis_profile.c",
            "shared/checksum/crc32.c",
            "firmware/motor/protection/motor_thermal.c",
            "firmware/motor/protection/motor_thermal_config.c",
            "firmware/motor/math/control_math.c",
            "firmware/motor/control/speed_feedforward.c",
            "firmware/parameters/parameter_values.c",
        ]
        with tempfile.TemporaryDirectory() as directory:
            for board in (1, 2):
                with self.subTest(board=board):
                    output = Path(directory) / f"configuration-{board}.exe"
                    build = subprocess.run(
                        [
                            str(compiler),
                            "cc",
                            *NATIVE_INCLUDE_FLAGS,
                            f"-DBSP_BOARD={board}",
                            "-std=c11",
                            "-O2",
                            "-UNDEBUG",
                            "-Wall",
                            "-Wextra",
                            "-Werror",
                            *sources,
                            "-o",
                            str(output),
                        ],
                        cwd=ROOT,
                        check=False,
                        capture_output=True,
                    )
                    self.assertEqual(
                        build.returncode, 0, build.stderr.decode("utf-8", errors="replace")
                    )
                    result = subprocess.run(
                        [str(output)], check=True, capture_output=True, text=True
                    )
                    self.assertIn("PASS motor configuration", result.stdout)


if __name__ == "__main__":
    unittest.main()
