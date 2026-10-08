"""基础位置模块边界：保持纯计算，不再依赖轨迹、前馈或诊断协议。"""

import sys as _sys
from pathlib import Path as _Path

_sys.path.insert(0, str(_Path(__file__).resolve().parents[2] / "tools"))
import unittest

from project_paths import ROOT


class PositionServoBoundaryTests(unittest.TestCase):
    def test_basic_module_has_no_retired_or_hardware_dependencies(self) -> None:
        source = (ROOT / "firmware/motor/position/position_cascade.c").read_text(encoding="utf-8")
        header = (ROOT / "firmware/motor/position/position_cascade.h").read_text(encoding="utf-8")
        for name in (
            "PositionSmooth",
            "friction",
            "PositionCascadeTelemetry",
            "POSITION_SERVO_PHASE",
            "HAL_",
            "stm32",
            "position_smooth_trajectory.h",
            "fast_loop_profile.h",
        ):
            self.assertNotIn(name, source + header)
        self.assertIn("PiController_Run", source)
        self.assertIn("PiController_TrackOutput", source)
        self.assertIn("PositionCascadeControlOutput", header)


if __name__ == "__main__":
    unittest.main()
