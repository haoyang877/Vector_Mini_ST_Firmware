"""校验 RTT 4 通道帧布局、JScope 描述符与配套波形工程的一致性；不连接硬件。"""

import sys as _sys
from pathlib import Path as _Path

_sys.path.insert(0, str(_Path(__file__).resolve().parents[2] / "tools"))
import re
import unittest
import xml.etree.ElementTree as ET

from project_paths import ROOT

RTT_TELEMETRY = ROOT / "firmware/app/rtt_telemetry.c"
MAIN_SOURCE = ROOT / "firmware/platform/stm32g4/cubemx/Core/Src/main.c"
HW_CONF = ROOT / "firmware/platform/stm32g4/bsp/hw_conf.h"
SCOPES = (
    ROOT / "tools/bench/scopes/pro_lks.lksscope",
    ROOT / "tools/bench/scopes/pro_lks_servo_hil.lksscope",
    ROOT / "tools/bench/scopes/pro_lks_calibration.lksscope",
)

EXPECTED_FIELDS = ["position", "speed", "iq_reference", "iq_feedback"]
EXPECTED_UNITS = ["Q15:180°/32768", "0.1 rpm", "mA", "mA"]
EXPECTED_DESCRIPTIONS = ["机械位置", "机械转速", "Iq 指令", "Iq 反馈"]


class RttTelemetryTests(unittest.TestCase):
    def test_firmware_frame_has_fixed_documented_layout(self) -> None:
        source = RTT_TELEMETRY.read_text(encoding="utf-8")
        frame = re.search(
            r"typedef struct\s*\{(?P<body>.*?)\}\s*RTT_TelemetryFrame_TypeDef;",
            source,
            flags=re.DOTALL,
        )
        self.assertIsNotNone(frame)
        fields = re.findall(r"\bint16_t\s+(\w+)\s*;", frame.group("body"))
        self.assertEqual(fields, EXPECTED_FIELDS)
        self.assertIn("sizeof(RTT_TelemetryFrame_TypeDef) == 8U", source)
        self.assertNotIn("PositionImpedance_GetTelemetry", source)
        self.assertNotIn("RTT_SERVO_STATUS", source)

    def test_j_scope_descriptor_matches_int16_frame(self) -> None:
        source = MAIN_SOURCE.read_text(encoding="utf-8")
        self.assertIn("SEGGER_RTT_ConfigUpBuffer(1, RTT_JSCOPE_DESCRIPTOR", source)
        header = HW_CONF.read_text(encoding="utf-8")
        descriptor = re.search(r'#define RTT_JSCOPE_DESCRIPTOR\s*"([^"]+)"', header)
        self.assertIsNotNone(descriptor)
        self.assertEqual(descriptor.group(1), "JScope_" + "i2" * len(EXPECTED_FIELDS))

    def test_scope_projects_expose_every_channel_once(self) -> None:
        for path in SCOPES:
            with self.subTest(scope=path.name):
                root = ET.parse(path).getroot()
                form = root.find(".//form[@type='5']")
                self.assertIsNotNone(form)
                channels = [variable.attrib["name"] for variable in form.findall("var")]
                self.assertEqual(
                    channels,
                    [f"rtt_channel1.data{index}" for index in range(len(EXPECTED_FIELDS))],
                )

    def test_maintained_scopes_use_shared_units(self) -> None:
        for name in ("pro_lks_servo_hil.lksscope", "pro_lks_calibration.lksscope"):
            with self.subTest(scope=name):
                root = ET.parse(ROOT / "tools/bench/scopes" / name).getroot()
                form = root.find(".//form[@type='5']")
                self.assertEqual(
                    [variable.attrib["unit"] for variable in form.findall("var")],
                    EXPECTED_UNITS,
                )
                self.assertEqual(
                    [variable.attrib["desc"] for variable in form.findall("var")],
                    EXPECTED_DESCRIPTIONS,
                )
                self.assertEqual(form.attrib["trigName"], "rtt_channel1.data0")

    def test_calibration_scope_drops_stale_counter_and_keeps_rate(self) -> None:
        source = (ROOT / "tools/bench/scopes/pro_lks_calibration.lksscope").read_text(
            encoding="utf-8"
        )
        self.assertNotIn("rtt_calibration_dropped_frames", source)
        root = ET.fromstring(source)
        self.assertEqual(root.find(".//param[@name='rttFreq']").attrib["value"], "2000")


if __name__ == "__main__":
    unittest.main()
