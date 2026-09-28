
import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(_Path(__file__).resolve().parents[2] / "tools"))
from project_paths import ROOT, NATIVE_INCLUDE_FLAGS

import re
import unittest
from pathlib import Path


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
            "MotorControl_TypeDef",
            "FOC_TypeDef",
            "Encoder_TypeDef",
            "PI_Controller_TypeDef",
            "PARAM_FRICTION_IDENT",
        ):
            self.assertNotIn(forbidden, public_api)


if __name__ == "__main__":
    unittest.main()
