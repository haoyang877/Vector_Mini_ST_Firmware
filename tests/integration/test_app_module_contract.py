"""检查生产 APP 的文件、公开函数前缀、依赖方向和实例所有权。"""

import re
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

APP = Path(__file__).resolve().parents[2] / "firmware/app"
MODULES = {
    "app": "App",
    "app_loader": "AppLoader",
    "app_protocol": "AppProtocol",
    "motor_axis": "MotorAxis",
    "motor_lifecycle": "MotorLifecycle",
    "motor_calibration": "MotorCalibration",
}


class AppModuleContractTests(unittest.TestCase):
    def test_only_loader_app_links_the_handoff(self):
        root = APP.parents[1]
        for project, loader_enabled in (
            ("projects/at32f456/application/ESC-APP-AT32F456.uvprojx", False),
            ("projects/stm32g4/MDK-ARM/ESC-APP-STM32G431.uvprojx", False),
            ("projects/stm32g4/MDK-ARM/ESC-APP-STM32G431-LoaderApp.uvprojx", True),
        ):
            target = ET.parse(root / project).find(".//Targets/Target")
            sources = [item.text for item in target.findall(".//FileName")]
            self.assertEqual(sources.count("app_loader.c"), int(loader_enabled), project)
            defines = target.findtext("TargetOption/TargetArmAds/Cads/VariousControls/Define")
            self.assertEqual("YG_APP_LOADER_IMAGE=1" in defines.split(","), loader_enabled, project)

    def test_public_names_and_modules(self):
        self.assertEqual({p.stem for p in APP.glob("*.c")}, set(MODULES))
        for name, prefix in MODULES.items():
            with self.subTest(module=name):
                source = (APP / f"{name}.h").read_text(encoding="utf-8")
                declarations = re.findall(r"^\w+[ \t*]+(\w+)\s*\(", source, re.MULTILINE)
                public = [func for func in declarations if not func.startswith("RTT_")]
                self.assertTrue(public)
                self.assertTrue(all(func.startswith(prefix + "_") for func in public), public)

    def test_module_state_is_instance_owned(self):
        # C 函数体与函数声明均不匹配；检查文件级和函数内静态工作对象。
        for name in MODULES:
            text = (APP / f"{name}.c").read_text(encoding="utf-8")
            text = re.sub(r"/\*.*?\*/|//[^\n]*", "", text, flags=re.DOTALL)
            self.assertNotRegex(
                text, r"\bstatic\s+(?!const\b)[\w\s*]+?\s+\w+\s*(?:\[[^;]*?\]\s*)?[=;]", name
            )

    def test_runtime_modules_do_not_import_composition(self):
        for name in ("motor_axis", "motor_lifecycle", "motor_calibration"):
            for suffix in ("c", "h"):
                text = (APP / f"{name}.{suffix}").read_text(encoding="utf-8")
                self.assertNotRegex(text, r'#include\s+"(?:app(?:_\w+)?|yg_protocol)\.h"')
