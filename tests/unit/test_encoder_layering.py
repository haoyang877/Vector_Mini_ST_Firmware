"""核对两板编码器单一硬件实现和便携反馈边界。"""

import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
BSP = ROOT / "firmware/platform/bsp"


class EncoderLayeringTests(unittest.TestCase):
    def test_projects_link_one_encoder_backend(self):
        """每个工程只选择本板硬件实现，不链接退役转发或器件包装。"""
        for project in (ROOT / "projects").rglob("*.uvprojx"):
            paths = [
                (project.parent / node.text.replace("\\", "/")).resolve()
                for node in ET.parse(project).findall(".//FilePath")
            ]
            board = "at32f456" if "at32f456" in project.parts else "stm32g4"
            self.assertEqual(
                [path for path in paths if path.name == "encoder_spi.c"],
                [BSP / "mcu" / board / "encoder_spi.c"],
                project.name,
            )
            self.assertFalse(any(path.parent.name == "devices" for path in paths))
            self.assertNotIn(BSP / "bsp_encoder.c", paths)

    def test_feedback_uses_public_angle_contract(self):
        """反馈只消费统一角度接口，方向和LUT保留在motor域。"""
        source = (ROOT / "firmware/motor/feedback/encoder_feedback.c").read_text(encoding="utf-8")
        self.assertIn("BspEncoder_ReadAngle(BSP_ENCODER_16BIT)", source)
        self.assertNotRegex(source, r"HAL_|SPI[0-9]|at32f45x|EncoderSpi_")
        header = (BSP / "include/bsp_encoder.h").read_text(encoding="utf-8")
        self.assertIn("void BspEncoder_Begin(void)", header)
        self.assertIn("uint32_t BspEncoder_ReadAngle(BspEncoderResolution resolution)", header)
        self.assertFalse((ROOT / "firmware/platform/devices").exists())


if __name__ == "__main__":
    unittest.main()
