import re
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
FOC_TASK = ROOT / "Foc" / "foc_task.c"
MAIN_SOURCE = ROOT / "Core" / "Src" / "main.c"
SCOPE_PROJECT = ROOT / "pro_lks.lksscope"

EXPECTED_FIELDS = [
    "target_position", "trajectory_position", "position_feedback", "position_error",
    "trajectory_speed", "speed_feedback", "iq_reference", "feedforward_current",
    "iq_feedback", "feedback_current", "hold_current", "servo_status",
]


class RttControlTelemetryTests(unittest.TestCase):
    def test_firmware_frame_has_fixed_documented_layout(self) -> None:
        source = FOC_TASK.read_text(encoding="utf-8")
        frame = re.search(
            r"typedef struct\s*\{(?P<body>.*?)\}\s*RTT_ControlFrame_TypeDef;",
            source,
            flags=re.DOTALL,
        )
        self.assertIsNotNone(frame)
        fields = re.findall(r"\bint16_t\s+(\w+)\s*;", frame.group("body"))
        self.assertEqual(fields, EXPECTED_FIELDS)
        self.assertIn("sizeof(RTT_ControlFrame_TypeDef) == 24U", source)
        self.assertNotIn("PositionImpedance_GetTelemetry", source)
        self.assertIn("RTT_SERVO_STATUS_FRICTION_LANDING", source)
        self.assertIn("RTT_SERVO_STATUS_SETTLE_RECOVERY", source)
        self.assertIn("RTT_SERVO_STATUS_HOLD_CANDIDATE", source)

    def test_j_scope_descriptor_matches_int16_frame(self) -> None:
        source = MAIN_SOURCE.read_text(encoding="utf-8")
        descriptor = re.search(
            r'SEGGER_RTT_ConfigUpBuffer\(1,\s*"([^"]+)"', source
        )
        self.assertIsNotNone(descriptor)
        self.assertEqual(descriptor.group(1), "JScope_" + "i2" * len(EXPECTED_FIELDS))

    def test_scope_project_exposes_every_channel_once(self) -> None:
        root = ET.parse(SCOPE_PROJECT).getroot()
        form = root.find(".//form[@type='5']")
        self.assertIsNotNone(form)
        channels = [variable.attrib["name"] for variable in form.findall("var")]
        self.assertEqual(
            channels,
            [f"rtt_channel1.data{index}" for index in range(len(EXPECTED_FIELDS))],
        )

    def test_both_scope_projects_use_new_units(self):
        for filename in ('pro_lks.lksscope', 'pro_lks_servo_hil.lksscope'):
            root = ET.parse(ROOT / filename).getroot()
            variables = root.find(".//form[@type='5']").findall('var')
            self.assertEqual([v.attrib['name'] for v in variables],
                             [f'rtt_channel1.data{i}' for i in range(12)])
            self.assertEqual([v.attrib['unit'] for v in variables],
                             ['0.01°'] * 4 + ['0.01°/s'] * 2 + ['mA'] * 5 + ['bits'])
            self.assertEqual(variables[7].attrib['desc'], '前馈电流')
            self.assertEqual(variables[8].attrib['desc'], '反馈电流')


if __name__ == "__main__":
    unittest.main()
