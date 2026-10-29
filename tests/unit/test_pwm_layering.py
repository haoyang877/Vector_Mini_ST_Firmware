"""验证PWM输出、目标许可和板级装配具有唯一所有者。"""

import re
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
PLATFORM = ROOT / "firmware/platform"


class PwmLayeringTests(unittest.TestCase):
    def test_each_target_has_one_pwm_backend_and_inline_permit(self):
        """同一接口不允许混入另一块板的实现或遗留的输出端口。"""
        for project in (ROOT / "projects").rglob("*.uvprojx"):
            files = [
                (project.parent / node.text.replace("\\", "/")).resolve()
                for node in ET.parse(project).findall(".//FilePath")
            ]
            board = "at32f456" if "at32f456" in project.parts else "stm32g4"
            for path in (
                PLATFORM / "bsp/bsp_pwm.c",
                PLATFORM / "bsp/mcu" / board / "pwm.c",
            ):
                self.assertEqual([p for p in files if p.name == path.name], [path])
            self.assertEqual(
                [p for p in files if p.name == "pwm_output_permit.c"],
                [],
            )
            self.assertFalse(any(p.stem.startswith(("motor_pwm_", "power_stage_")) for p in files))

    def test_only_one_public_pwm_contract(self):
        """业务代码统一调用公共BSP；许可状态不进入控制算法。"""
        self.assertTrue((PLATFORM / "bsp/include/bsp_pwm.h").is_file())
        for name in ("motor_hw.h", "power_stage_hw.h"):
            self.assertFalse((PLATFORM / "bsp/include" / name).exists())
        for directory in ("app", "motor", "parameters", "communication"):
            for path in (ROOT / "firmware" / directory).rglob("*"):
                if path.suffix not in (".c", ".h"):
                    continue
                source = path.read_text(encoding="utf-8")
                if directory != "app":
                    self.assertNotIn("pwm_power_window.h", source, str(path))
                self.assertNotIn("pwm_output_permit.h", source, str(path))

    def test_permit_module_has_no_hardware_access(self):
        """目标许可只引用私有状态与时间契约，不控制寄存器。"""
        for path in (PLATFORM / "bsp").glob("pwm_*.[ch]"):
            source = path.read_text(encoding="utf-8")
            self.assertIsNone(re.search(r"\b(?:TMR\d|TIM\d|HAL_\w+|tmr_\w+)\b", source))
        startup = (PLATFORM / "bsp/mcu/stm32g4/system.c").read_text(encoding="utf-8")
        self.assertNotIn("HAL_TIM_PWM_Start", startup)
        self.assertNotIn("HAL_TIMEx_OCN_Start", startup)
        common = (PLATFORM / "bsp/bsp_pwm.c").read_text(encoding="utf8")
        self.assertIn("McuPwm_CommitThree(a, b, c)", common)
        self.assertIn("McuPwm_Fatal()", common)
        self.assertNotIn("PwmPowerWindow", common)
        self.assertNotIn("pwm_output_config.h", common)
        self.assertNotIn("BSP_PWM_PERMIT_MODE", common)
        self.assertIn("BSP_PWM_HARD_LOCK", common)
        for board in ("at32f456", "stm32g4"):
            self.assertNotIn(
                "PwmOutputPermit_TryEnable",
                (PLATFORM / "bsp/mcu" / board / "pwm.c").read_text(encoding="utf8"),
            )

    def test_board_inputs_do_not_own_application_current_limits(self):
        """电气量程归板配置，命令/标定上限和软件跳闸值归应用策略。"""
        for path in (PLATFORM / "config/boards").glob("*.h"):
            source = path.read_text(encoding="utf8")
            for policy in (
                "CURRENT_SENSE_PROFILE_COMMAND_LIMIT_MAX_A",
                "CURRENT_SENSE_PROFILE_CALIB_LIMIT_MAX_A",
                "CURRENT_SENSE_PROFILE_OVERCURRENT_TRIP_A",
            ):
                self.assertNotIn(policy, source, str(path))


if __name__ == "__main__":
    unittest.main()
