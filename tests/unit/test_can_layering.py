"""核对单一CAN模块、私有错误处理与唯一构建实现。"""

import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
BSP = ROOT / "firmware/platform/bsp"


class CanLayeringTests(unittest.TestCase):
    def test_targets_select_one_board_and_shared_codec(self):
        """工程仅链接本板CAN和真实共用的帧辅助。"""
        for project in (ROOT / "projects").rglob("*.uvprojx"):
            paths = [
                (project.parent / node.text.replace("\\", "/")).resolve()
                for node in ET.parse(project).findall(".//FilePath")
            ]
            board = "at32f456" if "at32f456" in project.parts else "stm32g4"
            for source in (
                BSP / "bsp_can.c",
                BSP / "mcu" / board / "can.c",
                BSP / "can_frame_codec.c",
            ):
                self.assertEqual([p for p in paths if p.name == source.name], [source])

    def test_no_old_api_or_protocol_inside_controller(self):
        """控制器辅助不再形成公开接口，BSP不解释业务协议。"""
        for board in ("at32f456", "stm32g4"):
            source = (BSP / "mcu" / board / "can.c").read_text(encoding="utf8")
            for forbidden in ("YgProtocol", "yg_protocol", "CanController_", "CanIrq_"):
                self.assertNotIn(forbidden, source)
            for name in ("can_controller.h", "can_irq.h", "can_controller.c", "can_irq.c"):
                self.assertFalse((BSP / "mcu" / board / name).exists())
        self.assertFalse((BSP / "can_disabled.h").exists())
        for path in (BSP / "can_frame_codec.c", BSP / "bsp_can.c"):
            source = path.read_text(encoding="utf8")
            for hardware in ("HAL_", "CAN1", "FDCAN1", "at32f45x", "hfdcan1"):
                self.assertNotIn(hardware, source)

    def test_irq_and_init_are_wired_through_public_contract(self):
        """保留初始化及接收开放时机，IRQ通过BSP边界进入通信。"""
        at = (ROOT / "firmware/platform/bsp/mcu/at32f456/system.c").read_text(encoding="utf8")
        st = (ROOT / "firmware/platform/bsp/mcu/stm32g4/system.c").read_text(encoding="utf8")
        irq = st
        for text in (at, irq):
            self.assertIn("Can_OnReceiveInterrupt(BspApp_OnCanReceive", text)
        self.assertNotIn("can_rxbuf_status_get", at)
        self.assertNotIn("YgProtocol", at + st)
        self.assertIn(
            "McuCan_EnableReceiveInterrupt()", (BSP / "bsp_can.c").read_text(encoding="utf8")
        )
        for text in (at, st):
            self.assertIn("Can_Init()", text)


if __name__ == "__main__":
    unittest.main()
