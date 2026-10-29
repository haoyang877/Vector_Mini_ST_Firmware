"""编码器公共流程、协议与传输互不混用。"""

import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
BSP = ROOT / "firmware/platform/bsp"
DEVICES = ROOT / "firmware/platform/devices"


class EncoderLayeringTests(unittest.TestCase):
    def test_projects_link_one_protocol_and_one_transport(self):
        for p in (ROOT / "projects").rglob("*.uvprojx"):
            paths = [
                (p.parent / n.text.replace("\\", "/")).resolve()
                for n in ET.parse(p).findall(".//FilePath")
            ]
            board = "at32f456" if "at32f456" in p.parts else "stm32g4"
            self.assertEqual(
                [x for x in paths if x.name == "encoder_spi.c"],
                [BSP / "mcu" / board / "encoder_spi.c"],
            )
            self.assertEqual(
                sorted([x for x in paths if x.parent == DEVICES]),
                [DEVICES / name for name in ("encoder_mt6835.c", "encoder_tle5012b.c")],
            )
            self.assertIn(BSP / "bsp_encoder.c", paths)

    def test_only_device_layer_decodes_samples(self):
        for p in (DEVICES).glob("*.c"):
            s = p.read_text(encoding="utf8")
            self.assertIn("EncoderProtocol_Decode", s)
            self.assertNotRegex(s, r"EncoderSpi_\w+\(|HAL_|SPI2|at32f45x")
        for p in (BSP / "mcu").glob("*/encoder_spi.c"):
            self.assertNotRegex(p.read_text(encoding="utf8"), r"angle_q15|frame_crc|config_words")
        common = (BSP / "bsp_encoder.c").read_text(encoding="utf8")
        for name in ("EncoderProtocol_Decode", "s_config_words", "EncoderSpi_Complete"):
            self.assertIn(name, common)


if __name__ == "__main__":
    unittest.main()
