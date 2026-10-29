"""核对共用BSP、MCU实现和器件协议的依赖边界。"""

import re
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
PLATFORM = ROOT / "firmware/platform"


class PlatformLayoutTests(unittest.TestCase):
    def test_platform_source_has_only_four_roots(self):
        roots = {
            p.relative_to(PLATFORM).parts[0]
            for p in PLATFORM.rglob("*")
            if p.is_file() and len(p.relative_to(PLATFORM).parts) > 1
        }
        self.assertEqual(roots, {"bsp", "config", "devices", "sdk"})

    def test_board_config_does_not_own_application_requirements(self):
        for p in (PLATFORM / "config").rglob("*.h"):
            self.assertNotRegex(
                p.read_text(encoding="utf8"),
                r"BSP_REQUIRED_|R40_APP_(?:PHASE_LOCK|BOUNDED_TEST)|PARAM_MOTOR_THERMAL_",
            )
        self.assertTrue((ROOT / "firmware/app/config/app_feature_requirements.h").is_file())

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
            for kind in ("adc", "pwm", "encoder", "can"):
                self.assertEqual(
                    [p for p in sources if p.name == f"bsp_{kind}.c"],
                    [PLATFORM / "bsp" / f"bsp_{kind}.c"],
                )

    def test_common_bsp_does_not_know_board_or_sdk(self):
        for p in (PLATFORM / "bsp").glob("bsp_*.c"):
            self.assertNotRegex(
                p.read_text(encoding="utf8"), r"at32f|stm32g|\b(?:TMR1|TIM1|ADC1|HAL_|R40_)"
            )
        for p in (PLATFORM / "bsp/mcu").rglob("*.c"):
            public_functions = set(
                re.findall(
                    r"(?m)^(?:bool|void|uint16_t|Bsp\w+) (Bsp(?:Sampling|Pwm|Encoder|Can)_\w+)\(",
                    p.read_text(encoding="utf8"),
                )
            )
            # 无状态的关相查询直接由MCU实现，公共保护和事务仍只有一份。
            expected = {"BspPwm_ArePhasesDisabled"} if p.name == "pwm.c" else set()
            self.assertEqual(public_functions, expected, str(p))

    def test_one_private_header_without_implementation(self):
        """私有声明集中，应用不依赖；器件协议继续独立于寄存器实现。"""
        bsp = PLATFORM / "bsp"
        self.assertFalse((bsp / "internal").exists())
        header = (bsp / "bsp_internal.h").read_text(encoding="utf8")
        self.assertNotRegex(header, r"\)\s*\{")
        self.assertNotRegex(header, r"(?m)^\s*(?:static|extern)\s+")
        self.assertNotRegex(header, r"\b(?:HAL_|TMR1|TIM1|ADC1|at32f45x|stm32g4xx)")
        self.assertNotIn("EncoderSpiRequest", header)
        self.assertTrue((PLATFORM / "devices/encoder_protocol.h").is_file())
        self.assertTrue((PLATFORM / "devices/encoder_spi.h").is_file())
        # Git不跟踪空目录；只拒绝旧位置残留的实现或头文件。
        self.assertEqual(list((bsp / "devices").rglob("*.[ch]")), [])
        self.assertEqual(
            {p.name for p in (PLATFORM / "devices").iterdir() if p.is_file()},
            {"encoder_protocol.h", "encoder_spi.h", "encoder_mt6835.c", "encoder_tle5012b.c"},
        )
        for directory in ("app", "motor", "parameters", "communication"):
            for path in (ROOT / "firmware" / directory).rglob("*.[ch]"):
                self.assertNotIn('"bsp_internal.h"', path.read_text(encoding="utf8"), str(path))

    def test_direct_queries_have_no_common_forwarder(self):
        """查询仍由原硬件所有者实现，不重复公共状态和保护逻辑。"""
        common_adc = (PLATFORM / "bsp/bsp_adc.c").read_text(encoding="utf8")
        common_pwm = (PLATFORM / "bsp/bsp_pwm.c").read_text(encoding="utf8")
        self.assertNotIn("bool Adc_IsHealthy(void)", common_adc)
        self.assertNotIn("bool BspPwm_ArePhasesDisabled(void)", common_pwm)
        for board in ("at32f456", "stm32g4"):
            mcu = PLATFORM / "bsp/mcu" / board
            self.assertIn(
                "bool Adc_IsHealthy(void)",
                (mcu / "adc.c").read_text(encoding="utf8"),
            )
            self.assertIn(
                "bool BspPwm_ArePhasesDisabled(void)", (mcu / "pwm.c").read_text(encoding="utf8")
            )

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
            "r40_indicator.h",
        ):
            self.assertFalse((mcu / name).exists(), name)
        adc = (mcu / "adc.c").read_text(encoding="utf8")
        self.assertNotIn("AdcSamplingState", adc)
        self.assertNotIn("R40_KEIL_OPENLOOP", adc)
        self.assertNotRegex(adc, r"return\s+AdcSampling_\w+\(&s_sampling")
        self.assertNotIn("gpio_pin_mux_config", (mcu / "system.c").read_text(encoding="utf8"))
        self.assertIn("McuPwm_PreparePins", (mcu / "pwm.c").read_text(encoding="utf8"))


if __name__ == "__main__":
    unittest.main()
