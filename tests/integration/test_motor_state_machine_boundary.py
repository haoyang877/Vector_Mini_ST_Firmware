"""约束状态所有权和功率许可，防止旧模式体系重新进入生产调用链。"""

import re
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
from project_paths import ROOT


def production_text(path: Path) -> str:
    """去除说明文字，只检查 C 生产声明和调用。"""
    return re.sub(r"/\*.*?\*/|//[^\n]*", "", path.read_text(encoding="utf-8"), flags=re.S)


class MotorStateMachineBoundaryTests(unittest.TestCase):
    def test_no_legacy_state_contract(self) -> None:
        retired = (
            r"\b(?:ModeNow|ModeLast|ModeNow_TypeDef|MotorWorkOutcome_TypeDef|next_mode|power_off)\b"
        )
        for directory in ("app", "motor", "services"):
            for path in (ROOT / "firmware" / directory).rglob("*"):
                if path.suffix in (".c", ".h"):
                    self.assertIsNone(re.search(retired, production_text(path)), path.as_posix())
        for name in ("foc_run", "foc_run_state", "foc_mode_dispatch"):
            for suffix in (".c", ".h"):
                self.assertFalse((ROOT / "firmware/app" / (name + suffix)).exists())
        self.assertFalse((ROOT / "firmware/motor/motor_work.h").exists())

    def test_normal_enable_has_one_owner(self) -> None:
        callers = set()
        for directory in ("app", "motor", "services", "communication"):
            for path in (ROOT / "firmware" / directory).rglob("*.c"):
                if re.search(r"\bpower_stage_hw_start\s*\(", production_text(path)):
                    callers.add(path.relative_to(ROOT).as_posix())
        self.assertEqual(callers, {"firmware/app/motor_state_machine.c"})


if __name__ == "__main__":
    unittest.main()
