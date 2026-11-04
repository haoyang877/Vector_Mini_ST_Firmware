"""核对统一BSP声明、MCU实现和应用的依赖边界。"""

import re
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
PLATFORM = ROOT / "firmware/platform"


class PlatformLayoutTests(unittest.TestCase):
    def test_platform_source_has_only_hardware_and_vendor_roots(self):
        roots = {
            p.relative_to(PLATFORM).parts[0]
            for p in PLATFORM.rglob("*")
            if p.is_file() and len(p.relative_to(PLATFORM).parts) > 1
        }
        self.assertEqual(roots, {"bsp", "sdk"})

    def test_board_config_does_not_own_application_requirements(self):
        config = ROOT / "firmware/config"
        self.assertEqual(
            {p.name for p in config.glob("*.h")},
            {
                "config.h",
                "board_config.h",
                "motor_profile_config.h",
                "application_config.h",
                "SEGGER_RTT_Conf.h",
            },
        )
        resource = (config / "board_config.h").read_text(encoding="utf8")
        self.assertNotRegex(
            resource, r"BSP_REQUIRED_|AT32_APP_(?:PHASE_LOCK|BOUNDED_TEST)|PARAM_MOTOR_THERMAL_"
        )
        app = (config / "application_config.h").read_text(encoding="utf8")
        self.assertIn("CURRENT_SENSE_PROFILE_OVERCURRENT_TRIP_A", app)

    def test_motor_and_board_inputs_are_separate(self):
        config = ROOT / "firmware/config"
        board = (config / "board_config.h").read_text(encoding="utf8")
        motor = (config / "motor_profile_config.h").read_text(encoding="utf8")
        self.assertNotIn("FOC_ACTIVE_MOTOR_PROFILE", board)
        self.assertNotIn("PARAM_MOTOR_", board)
        self.assertNotIn("BSP_SUPPORTED_", motor)

    def test_each_project_has_its_own_entry_outside_bsp(self):
        for project in (ROOT / "projects").rglob("*.uvprojx"):
            sources = [
                (project.parent / n.text.replace("\\", "/")).resolve()
                for n in ET.parse(project).findall(".//FilePath")
            ]
            entries = [
                p
                for p in sources
                if p.suffix == ".c"
                and re.search(r"(?m)^int main\(void\)", p.read_text(encoding="utf8"))
            ]
            self.assertEqual(len(entries), 1, project.name)
            self.assertFalse(entries[0].is_relative_to(PLATFORM))
            board = "at32f456" if "at32f456" in project.parts else "stm32g4"
            for kind in ("adc", "pwm", "encoder_spi", "can"):
                self.assertEqual(
                    [
                        path
                        for path in sources
                        if path.name == f"{kind}.c" and "generated" not in path.parts
                    ],
                    [PLATFORM / "bsp/mcu" / board / f"{kind}.c"],
                )

    def test_common_bsp_does_not_know_board_or_sdk(self):
        for p in (PLATFORM / "bsp").glob("bsp_*.c"):
            self.assertNotRegex(
                p.read_text(encoding="utf8"), r"at32f|stm32g|\b(?:TMR1|TIM1|ADC1|HAL_|R40_)"
            )
        self.assertEqual(list((PLATFORM / "bsp").glob("*.c")), [])

    def test_one_private_header_without_implementation(self):
        """私有声明集中，应用不依赖；无退役器件目录残留。"""
        bsp = PLATFORM / "bsp"
        self.assertFalse((bsp / "internal").exists())
        header = (bsp / "bsp_internal.h").read_text(encoding="utf8")
        self.assertNotRegex(header, r"\)\s*\{")
        self.assertNotRegex(header, r"(?m)^\s*(?:static|extern)\s+")
        self.assertNotRegex(header, r"\b(?:HAL_|TMR1|TIM1|ADC1|at32f45x|stm32g4xx)")
        self.assertNotIn("EncoderSpiRequest", header)
        self.assertFalse((PLATFORM / "devices").exists())
        self.assertEqual(list((bsp / "devices").rglob("*.[ch]")), [])
        for directory in ("app", "motor", "parameters", "communication"):
            for path in (ROOT / "firmware" / directory).rglob("*.[ch]"):
                self.assertNotIn('"bsp_internal.h"', path.read_text(encoding="utf8"), str(path))

    def test_direct_queries_have_no_common_forwarder(self):
        """查询仍由原硬件所有者实现，不重复公共状态和保护逻辑。"""
        self.assertFalse((PLATFORM / "bsp/bsp_adc.c").exists())
        self.assertFalse((PLATFORM / "bsp/bsp_pwm.c").exists())
        for board in ("at32f456", "stm32g4"):
            mcu = PLATFORM / "bsp/mcu" / board
            adc = (mcu / "adc.c").read_text(encoding="utf8")
            pwm = (mcu / "pwm.c").read_text(encoding="utf8")
            self.assertIn("float BspAdc_ReadChannel(", adc)
            self.assertIn("bool BspPwm_ArePhasesDisabled(void)", pwm)
            self.assertIn("void BspPwm_SetDuty(", pwm)

    def test_bsp_does_not_dispatch_application_or_protocol(self):
        for p in (PLATFORM / "bsp").rglob("*.[ch]"):
            s = p.read_text(encoding="utf8")
            self.assertNotRegex(s, r"\bApp_\w+\s*\(")
            self.assertNotRegex(s, r'#include\s+"(?:app\.h|motor_command\.h|yg_protocol[^"]*)"')

    def test_at32_peripheral_state_does_not_require_split_implementation_headers(self):
        """同一ADC状态留在实现内，防止再次靠分层私有头和转发拼接。"""
        mcu = PLATFORM / "bsp/mcu/at32f456"
        for name in (
            "adc_sampling_internal.h",
            "adc_sampling_setup.c",
            "adc_sampling_storage.c",
            "system_setup.h",
            "system_setup.c",
        ):
            self.assertFalse((mcu / name).exists(), name)
        adc = (mcu / "adc.c").read_text(encoding="utf8")
        self.assertNotIn("AdcSamplingState", adc)
        self.assertNotIn("AT32_KEIL_OPENLOOP", adc)
        self.assertNotRegex(adc, r"return\s+AdcSampling_\w+\(&s_sampling")
        self.assertNotIn("gpio_pin_mux_config", (mcu / "system.c").read_text(encoding="utf8"))
        self.assertIn("void McuPwm_Init(void)", (mcu / "pwm.c").read_text(encoding="utf8"))


if __name__ == "__main__":
    unittest.main()
