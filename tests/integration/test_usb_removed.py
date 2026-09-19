"""离线校验 USB 移除后的 CAN-only 配置：构建输入、启动代码与 2 kHz 监督任务不得引用 USB。"""

import re
import sys
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))
from project_paths import CUBEMX, KEIL, ROOT


class UsbRemovalTests(unittest.TestCase):
    def test_both_projects_exclude_usb_sources_and_search_paths(self):
        for project in KEIL.glob("*.uvprojx"):
            tree = ET.parse(project)
            paths = [
                node.text or ""
                for tag in ("FilePath", "IncludePath")
                for node in tree.findall(".//" + tag)
            ]
            for path in paths:
                self.assertNotRegex(path.lower(), r"usb|pcd|ring_buffer")

    def test_startup_and_supervisor_keep_can_without_usb_calls(self):
        main = (CUBEMX / "Core/Src/main.c").read_text(encoding="utf-8")
        task = (ROOT / "firmware/app/bsp_task.c").read_text(encoding="utf-8")
        self.assertIn("MX_FDCAN1_Init();", main)
        self.assertIn("CAN_Service();", task)
        self.assertIn("FOC2kHzSupervisor();", task)
        self.assertIn("CAN_DisConnect_Handle();", task)
        self.assertNotRegex(main + task, r"USB_|usb_device|HSI48_ON")

    def test_usb_irq_slots_remain_but_no_application_handlers(self):
        startup = (KEIL / "startup_stm32g431xx.s").read_text()
        irq = (CUBEMX / "Core/Src/stm32g4xx_it.c").read_text(encoding="utf-8")
        for priority in ("HP", "LP"):
            self.assertRegex(startup, r"DCD\s+USB_" + priority + "_IRQHandler")
            self.assertNotIn("void USB_" + priority + "_IRQHandler", irq)
        self.assertNotIn("hpcd_USB_FS", irq)

    def test_cubemx_cannot_regenerate_usb(self):
        text = (CUBEMX / "Vector_Mini_ST.ioc").read_text()
        self.assertNotIn("USB", text)
        for pin in ("PA11", "PA12"):
            self.assertIn(pin + ".Signal=GPIO_Analog", text)
        self.assertIn("ProjectManager.MainLocation=Core/Src", text)
        self.assertIn("ProjectManager.TargetToolchain=MDK-ARM V5.32", text)
        values = dict(line.split("=", 1) for line in text.splitlines() if "=" in line)
        for prefix, count in (("IP", "IPsNb"), ("Pin", "PinsNb")):
            keys = sorted(
                int(re.fullmatch(r"Mcu\." + prefix + r"(\d+)", key)[1])
                for key in values
                if re.fullmatch(r"Mcu\." + prefix + r"\d+", key)
            )
            self.assertEqual(keys, list(range(int(values["Mcu." + count]))))

    def test_hal_pcd_and_application_usb_files_removed(self):
        config = (CUBEMX / "Core/Inc/stm32g4xx_hal_conf.h").read_text()
        self.assertNotRegex(config, r"(?m)^\s*#define\s+HAL_PCD_MODULE_ENABLED\b")
        self.assertFalse((CUBEMX / "USB_Device").exists())
        self.assertFalse((ROOT / "firmware/communication/usb").exists())
        self.assertFalse((ROOT / "firmware/common/ring_buffer.c").exists())


if __name__ == "__main__":
    unittest.main()
