import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CORE_HEADER = ROOT / "Firmware" / "Domain" / "Identification" / "friction_identification.h"
CORE_SOURCE = ROOT / "Firmware" / "Domain" / "Identification" / "friction_identification.c"


class FrictionModuleBoundaryTests(unittest.TestCase):
    def test_core_has_only_standard_and_own_includes(self) -> None:
        includes = re.findall(r'^#include\s+([<"][^>"]+[>"])',
                              CORE_SOURCE.read_text(encoding="utf-8"), re.MULTILINE)
        self.assertEqual(includes, ['"friction_identification.h"', "<math.h>", "<string.h>"])

    def test_core_public_api_has_no_runtime_types(self) -> None:
        public_api = CORE_HEADER.read_text(encoding="utf-8")
        for forbidden in ("MotorControlContext", "CurrentControlContext", "EncoderContext",
                          "PiController", "MechanicalLoadProfile"):
            self.assertNotIn(forbidden, public_api)


if __name__ == "__main__":
    unittest.main()
