# 核对摩擦功能的模块边界，避免业务代码越过硬件端口。
import sys as _sys
from pathlib import Path as _Path

_sys.path.insert(0, str(_Path(__file__).resolve().parents[2] / "tools"))
import re
import unittest

from project_paths import ROOT

CORE_HEADER = ROOT / "firmware/motor/identification/friction_identification.h"
CORE_SOURCE = ROOT / "firmware/motor/identification/friction_identification.c"


class FrictionModuleBoundaryTests(unittest.TestCase):
    def test_core_has_only_standard_and_own_includes(self) -> None:
        includes = re.findall(
            r'^#include\s+([<"][^>"]+[>"])',
            CORE_SOURCE.read_text(encoding="utf-8"),
            flags=re.MULTILINE,
        )
        self.assertEqual(
            includes,
            ['"friction_identification.h"', "<math.h>", "<string.h>"],
        )

    def test_core_public_api_has_no_firmware_types(self) -> None:
        public_api = CORE_HEADER.read_text(encoding="utf-8")
        for forbidden in (
            "MotorControlState",
            "FocState",
            "EncoderState",
            "PiController",
            "PARAM_FRICTION_IDENT",
        ):
            self.assertNotIn(forbidden, public_api)


if __name__ == "__main__":
    unittest.main()
