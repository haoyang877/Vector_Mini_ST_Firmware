
import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(_Path(__file__).resolve().parents[2] / "tools"))
from project_paths import ROOT, NATIVE_INCLUDE_FLAGS

import re
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


FOC_TASK = ROOT / "firmware/app/foc_task.c"
MAIN_SOURCE = ROOT / "firmware/platform/stm32g4/cubemx/Core/Src/main.c"
SCOPE_PROJECT = ROOT / "tools/bench/scopes/pro_lks.lksscope"

EXPECTED_FIELDS = [
    "target_position", "trajectory_position", "position_feedback", "position_error",
    "trajectory_speed", "speed_feedback", "iq_reference", "feedforward_current",
    "iq_feedback", "feedback_current", "hold_current", "servo_status",
]


class RttControlTelemetryTests(unittest.TestCase):
    def test_firmware_frame_has_fixed_documented_layout(self) -> None:
        source = FOC_TASK.read_text(encoding="utf-8")
        frame = re.search(
            r"typedef struct\s*\{(?P<body>.*?)\}\s*RTT_Data_TypeDef;",
            source,
            flags=re.DOTALL,
        )
        self.assertIsNotNone(frame)
        fields = re.findall(r"\bint16_t\s+(\w+)\s*;", frame.group("body"))
        self.assertEqual(fields, [f"data{i}" for i in range(11)])
        self.assertIn("sizeof(RTT_Data_TypeDef) == 22U", source)

    def test_j_scope_descriptor_matches_int16_frame(self) -> None:
        source = MAIN_SOURCE.read_text(encoding="utf-8")
        self.assertIn('SEGGER_RTT_ConfigUpBuffer(1, RTT_JSCOPE_DESCRIPTOR', source)
        source = (ROOT / 'firmware/platform/stm32g4/bsp/hw_conf.h').read_text(encoding='utf-8')
        descriptor = re.search(
            r'#define RTT_JSCOPE_DESCRIPTOR\s*"([^"]+)"', source
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
            [f"rtt_channel1.data{index}" for index in range(11)],
        )

    def test_servo_hil_scope_uses_v2_units(self):
        # pro_lks.lksscope is the user's generic/custom scope. The HIL scope
        # remains the maintained v2 layout; calibration has its own project.
        for filename in ('tools/bench/scopes/pro_lks_servo_hil.lksscope',):
            root = ET.parse(ROOT / filename).getroot()
            variables = root.find(".//form[@type='5']").findall('var')
            self.assertEqual([v.attrib['name'] for v in variables],
                             [f'rtt_channel1.data{i}' for i in range(12)])
            self.assertEqual([v.attrib['unit'] for v in variables],
                         ['0.01°'] * 4 + ['0.01°/s'] * 2 + ['mA'] * 5 + ['bits'])
            self.assertEqual(variables[7].attrib['desc'], '前馈电流')
            self.assertEqual(variables[8].attrib['desc'], '反馈电流')

    def test_calibration_scope_and_descriptor(self):
        root = ET.parse(ROOT / 'tools/bench/scopes/pro_lks_calibration.lksscope').getroot()
        variables = root.find(".//form[@type='5']").findall('var')
        self.assertEqual([v.attrib['name'] for v in variables],
                         [f'rtt_channel1.data{i}' for i in range(11)])
        self.assertEqual([v.attrib['unit'] for v in variables],
                         ['mA']*3 + ['mV']*2 + ['0.0002 rad']*2 + ['0.0055 deg'] + ['mA']*3)
        self.assertEqual(variables[0].attrib['desc'], 'Iq参考')
        self.assertEqual(variables[1].attrib['desc'], 'Iq反馈')
        self.assertEqual(root.find(".//param[@name='rttFreq']").attrib['value'], '2000')
        self.assertTrue(all(not v.attrib.get('addr') for v in root.findall(".//form[@type='7']/var")))
        header = (ROOT / 'firmware/platform/stm32g4/bsp/hw_conf.h').read_text(encoding='utf-8')
        self.assertIn('"JScope_' + 'i2'*11 + '"', header)


if __name__ == "__main__":
    unittest.main()
