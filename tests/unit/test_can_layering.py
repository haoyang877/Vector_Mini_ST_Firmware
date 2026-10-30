"""核对各MCU的CAN实现与唯一构建输入。"""

import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
BSP = ROOT / "firmware/platform/bsp"


class CanLayeringTests(unittest.TestCase):
    def test_targets_select_one_board_can(self):
        """工程只链接本板CAN实现，DLC换算不再单独编译。"""
        for project in (ROOT / "projects").rglob("*.uvprojx"):
            paths = [
                (project.parent / node.text.replace("\\", "/")).resolve()
                for node in ET.parse(project).findall(".//FilePath")
            ]
            board = "at32f456" if "at32f456" in project.parts else "stm32g4"
            source = BSP / "mcu" / board / "can.c"
            self.assertEqual([p for p in paths if p.name == "can.c"], [source])
            self.assertNotIn("bsp_can.c", [p.name for p in paths])
            self.assertNotIn("can_frame_codec.c", [p.name for p in paths])

    def test_no_old_api_or_protocol_inside_controller(self):
        """控制器辅助不再形成公开接口，BSP不解释业务协议。"""
        for board in ("at32f456", "stm32g4"):
            source = (BSP / "mcu" / board / "can.c").read_text(encoding="utf8")
            for forbidden in ("YgProtocol", "yg_protocol", "CanController_", "CanIrq_"):
                self.assertNotIn(forbidden, source)
            for name in ("can_controller.h", "can_irq.h", "can_controller.c", "can_irq.c"):
                self.assertFalse((BSP / "mcu" / board / name).exists())
        self.assertFalse((BSP / "can_disabled.h").exists())
        self.assertFalse((BSP / "can_frame_codec.c").exists())
        self.assertFalse((BSP / "can_frame_codec.h").exists())
        self.assertFalse((BSP / "bsp_can.c").exists())

    def test_irq_and_init_are_wired_through_public_contract(self):
        """保留初始化及接收开放时机，IRQ通过BSP边界进入通信。"""
        at = (ROOT / "firmware/platform/bsp/mcu/at32f456/system.c").read_text(encoding="utf8")
        st = (ROOT / "firmware/platform/bsp/mcu/stm32g4/system.c").read_text(encoding="utf8")
        at_can = (BSP / "mcu/at32f456/can.c").read_text(encoding="utf8")
        st_can = (BSP / "mcu/stm32g4/can.c").read_text(encoding="utf8")
        self.assertIn("void CAN1_RX_IRQHandler(void)", at_can)
        self.assertIn("BspApp_OnCanReceive();", at_can)
        self.assertIn("McuCan_Init();", at)
        self.assertIn("HAL_FDCAN_RxFifo0Callback", st_can)
        self.assertNotIn("McuCan_EnableReceiveInterrupt", st_can)
        self.assertIn("McuCan_Init();", st)
        self.assertNotIn("can_rxbuf_status_get", at)
        self.assertNotIn("YgProtocol", at + st)


if __name__ == "__main__":
    unittest.main()
