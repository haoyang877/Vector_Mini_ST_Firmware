"""R40基础工程的来源、隔离和构建安全检查；不编译或连接硬件。"""

import hashlib
import json
import re
import sys
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools/build"))
from check_project_layout import check_project  # noqa: E402


class At32TargetTests(unittest.TestCase):
    def test_app_stack_is_separate_from_vendor_and_reset_record_is_uninitialized(self):
        base = ROOT / "firmware/platform/at32f456/application"
        local = (base / "startup_at32f456_app.s").read_text(encoding="utf8")
        vendor = (ROOT / "firmware/third_party/at32f45x/cmsis/startup_at32f45x.s").read_text(
            encoding="utf8"
        )
        stack = re.search(r"Stack_Size\s+EQU\s+(0x[0-9A-Fa-f]+)", local)
        heap = re.search(r"Heap_Size\s+EQU\s+(0x[0-9A-Fa-f]+)", local)
        self.assertIsNotNone(stack)
        self.assertIsNotNone(heap)
        self.assertGreaterEqual(int(stack[1], 16), 8192)
        self.assertEqual(int(heap[1], 16), 0)
        # 栈定制不能改动原厂中断向量和复位/运行库入口。
        self.assertEqual(
            local[local.index("__Vectors       DCD") :],
            vendor[vendor.index("__Vectors       DCD") :],
        )
        project = (base / "R40_F456_App_Off.uvprojx").read_text(encoding="utf8")
        self.assertIn("<FilePath>startup_at32f456_app.s</FilePath>", project)
        scatter = (base / "r40_app.sct").read_text(encoding="utf8")
        self.assertRegex(scatter, r"RW_DIAGNOSTIC\s+0x2001AF80\s+UNINIT\s+0x00000080")
        self.assertIn("(.bss.r40_noinit)", scatter)

    def test_app_off_links_shared_application_and_cannot_enable_power(self):
        project = ROOT / "firmware/platform/at32f456/application/R40_F456_App_Off.uvprojx"
        errors, _, _ = check_project(project)
        self.assertEqual(errors, [])
        tree = ET.parse(project)
        defines = tree.findtext(".//Cads/VariousControls/Define")
        for flag in (
            "R40_APP_PHASE_LOCK=1",
            "R40_APP_BINDINGS=1",
            "APP_TARGET_R40_F456=1",
            "FOC_ACTIVE_HW_PROFILE=2",
            "FOC_ACTIVE_MOTOR_PROFILE=2",
        ):
            self.assertIn(flag, defines)
        paths = [value.text.replace("\\", "/") for value in tree.findall(".//FilePath")]
        for name in (
            "app_startup.c",
            "app_tasks.c",
            "motor_axis.c",
            "motor_state_machine.c",
            "board_at32f456.c",
            "indicator_at32f456.c",
            "boot_at32f456.c",
        ):
            self.assertTrue(any(path.endswith("/" + name) for path in paths), name)
        self.assertFalse(any("stm32g4" in path.lower() for path in paths))
        self.assertFalse(
            any("closedloop_bench" in path or "openloop_model" in path for path in paths)
        )
        self.assertEqual(tree.findtext(".//UpdateFlashBeforeDebugging"), "0")
        for hook in ("BeforeCompile", "BeforeMake", "AfterMake"):
            for index in (1, 2):
                self.assertEqual(tree.findtext(f".//{hook}/RunUserProg{index}"), "0")
        entry = project.with_name("main.c").read_text(encoding="utf8")
        for call in ("App_Init();", "App_Process();", "App_FastTick();", "App_SupervisorTick();"):
            self.assertIn(call, entry)
        self.assertNotIn("param_store_write(", entry)

    def test_closedloop_is_explicit_and_uses_unchanged_control_modules(self):
        base = ROOT / "firmware/platform/at32f456/foundation"
        project = base / "R40_F456_ClosedLoop.uvprojx"
        errors, _, _ = check_project(project)
        self.assertEqual(errors, [])
        tree = ET.parse(project)
        defines = tree.findtext(".//Cads/VariousControls/Define")
        self.assertIn("R40_CLOSEDLOOP_TEST=1", defines)
        self.assertIn("R40_OL_CURRENT_LIMIT_A=3.0f", defines)
        self.assertEqual(tree.findtext(".//UpdateFlashBeforeDebugging"), "0")
        files = [node.text.replace("\\", "/") for node in tree.findall(".//FilePath")]
        for name in ("foc_algorithm.c", "foc_pid.c", "speed_control.c"):
            self.assertTrue(any(path.endswith("/" + name) for path in files))
        entry = (base / "closedloop_bench.c").read_text(encoding="utf-8")
        self.assertIn("FOC_CurrentWithReference(", entry)
        self.assertIn("SpeedMode_UpdateControl(", entry)
        self.assertNotIn("param_store_", entry)
        self.assertNotIn("comm_hw_", entry)

    def test_openloop_is_explicit_and_defaults_to_no_power(self):
        base = ROOT / "firmware/platform/at32f456/foundation"
        projects = [
            base / f"R40_F456_OpenLoop_{kind}.uvprojx" for kind in ("Dry", "Power", "Combined")
        ]
        targets = [ET.parse(project).find(".//Target") for project in projects]
        self.assertEqual(
            [t.findtext("TargetName") for t in targets],
            ["R40_F456_OpenLoop_Dry", "R40_F456_OpenLoop_Power", "R40_F456_OpenLoop_Combined"],
        )
        for target, flag in zip(targets, ("0", "1", "1"), strict=True):
            self.assertIn(
                f"R40_OPENLOOP_POWER_ENABLED={flag}",
                target.findtext(".//Cads/VariousControls/Define"),
            )
            self.assertEqual(target.findtext(".//UpdateFlashBeforeDebugging"), "0")
        self.assertIn(
            "R40_OUTPUT_SINCOS_DIAGNOSTICS=1",
            targets[0].findtext(".//Cads/VariousControls/Define"),
        )
        combined_defines = targets[2].findtext(".//Cads/VariousControls/Define")
        self.assertIn("R40_OUTPUT_SINCOS_DIAGNOSTICS=1", combined_defines)
        self.assertIn("R40_OPENLOOP_SINCOS_TEST=1", combined_defines)
        entry = (base / "openloop_main.c").read_text(encoding="utf-8")
        self.assertNotIn("R40_CanInit(", entry)
        self.assertNotIn("comm_hw_", entry)
        self.assertNotIn("param_store_", entry)
        self.assertIn("#define R40_OPENLOOP_POWER_ENABLED 0", entry)
        for project in projects:
            errors, _, _ = check_project(project)
            self.assertEqual(errors, [])

    def test_foundation_project_is_isolated_and_has_no_build_hooks(self):
        project = ROOT / "firmware/platform/at32f456/foundation/R40_F456_Foundation.uvprojx"
        errors, _, _ = check_project(project)
        self.assertEqual(errors, [])
        tree = ET.parse(project)
        paths = [node.text.replace("\\", "/") for node in tree.findall(".//FilePath")]
        self.assertTrue(paths)
        self.assertFalse(any("stm32" in value.lower() or "/app/" in value for value in paths))
        for hook in ("BeforeCompile", "BeforeMake", "AfterMake"):
            for index in (1, 2):
                self.assertEqual(tree.findtext(f".//{hook}/RunUserProg{index}"), "0")
        self.assertEqual(tree.findtext(".//UpdateFlashBeforeDebugging"), "0")
        entry = project.with_name("main.c").read_text(encoding="utf-8")
        self.assertNotIn("power_stage_hw_start(", entry)
        self.assertNotIn("App_FastTick(", entry)
        self.assertIn("../ports/encoder_mt6835.c", paths)
        self.assertIn("encoder_sensor_init();", entry)
        self.assertLess(entry.index("encoder_sensor_init();"), entry.index("R40_AdcStart()"))

        old_project = ROOT / "firmware/platform/stm32g4/cubemx/MDK-ARM/YG_ESC_APP.uvprojx"
        self.assertNotIn("encoder_mt6835.c", old_project.read_text(encoding="utf-8"))
        self.assertNotIn("param_store_write(", entry)
        self.assertNotIn("motor_hw_temperature_poll(", entry)
        self.assertIn("../ports/motor_temperature_at32f456.c", paths)
        self.assertIn("R40_MotorTemperaturePoll(&temperature)", entry)
        self.assertNotIn("motor_temperature_at32f456", old_project.read_text(encoding="utf-8"))

    def test_parameter_sectors_are_excluded_from_image(self):
        base = ROOT / "firmware/platform/at32f456"
        layout = (base / "bsp/r40_storage_layout.h").read_text(encoding="utf-8")
        values = {
            name: int(value, 0)
            for name, value in re.findall(r"#define (R40_\w+) (0x[0-9A-Fa-f]+|[0-9]+)U", layout)
        }
        scatter = (base / "foundation/r40_foundation.sct").read_text(encoding="utf-8")
        ranges = re.findall(r"(?:LR_IROM1|ER_IROM1)\s+(0x\w+)\s+(0x\w+)", scatter)
        self.assertEqual(len(ranges), 2)
        for start, length in ranges:
            self.assertEqual(int(start, 0) + int(length, 0), values["R40_PARAM_BASE"])
        self.assertEqual(values["R40_PARAM_BASE"] % values["R40_FLASH_SECTOR_BYTES"], 0)
        self.assertLessEqual(values["R40_PARAM_RAW_BYTES"], 2 * values["R40_FLASH_SECTOR_BYTES"])
        self.assertEqual(
            values["R40_PARAM_BASE"] + values["R40_PARAM_BYTES"],
            values["R40_FLASH_BASE"] + values["R40_FLASH_BYTES"],
        )

    def test_vendor_import_retains_manifest_content(self):
        base = ROOT / "firmware/third_party/at32f45x"
        manifest = json.loads((base / "manifest.json").read_text(encoding="utf-8"))
        self.assertTrue(manifest["files"])
        for entry in manifest["files"]:
            with self.subTest(path=entry["path"]):
                self.assertEqual(
                    hashlib.sha256((base / entry["path"]).read_bytes()).hexdigest(),
                    entry["sha256"],
                )

    def test_keil_rtt_is_isolated_and_vendor_files_are_pinned(self):
        base = ROOT / "firmware/platform/at32f456/foundation"
        tree = ET.parse(base / "R40_F456_OpenLoop_Combined.uvprojx")
        self.assertIn("R40_KEIL_OPENLOOP=1", tree.findtext(".//Cads/VariousControls/Define"))
        for name in ("R40_F456_ClosedLoop", "R40_F456_Foundation", "R40_F456_OpenLoop_Dry"):
            self.assertNotIn("openloop_telemetry.c", (base / f"{name}.uvprojx").read_text())
        vendor = ROOT / "firmware/third_party/segger_rtt"
        manifest = json.loads((vendor / "manifest.json").read_text())
        self.assertEqual(len(manifest["revision"]), 40)
        for entry in manifest["files"]:
            self.assertEqual(
                hashlib.sha256((vendor / entry["path"]).read_bytes()).hexdigest(), entry["sha256"]
            )


if __name__ == "__main__":
    unittest.main()
