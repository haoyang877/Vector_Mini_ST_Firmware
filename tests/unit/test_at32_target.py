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


def target_defines(tree: ET.ElementTree, filename: str) -> str:
    """检查实际预包含入口，再读取其公共及目标配置供结构断言使用。"""
    options = tree.findtext(".//Cads/VariousControls/MiscControls").replace("\\", "/")
    relative = f"../../../firmware/app/config/targets/{filename}"
    if options != f"--diag_suppress=1 --preinclude {relative}":
        raise AssertionError(f"Unexpected target configuration entry: {options}")
    base = ROOT / "firmware/app/config/targets"
    text = (base / filename).read_text(encoding="utf8")
    text += (base / "r40_app_common.h").read_text(encoding="utf8")
    return ",".join(
        f"{key}={value}" for key, value in re.findall(r"^#define (\w+) ([^\n]+)", text, re.M)
    )


class At32TargetTests(unittest.TestCase):
    def test_full_app_grouping_and_common_options_match_old_board(self):
        """双板沿用公共工程配置；AT32快环采用明确的O3速度优化。"""
        old_path = ROOT / "projects/stm32g4/MDK-ARM/ESC-APP-STM32G431.uvprojx"
        old = ET.parse(old_path)
        group_names = [group.findtext("GroupName") for group in old.findall(".//Group")]
        shared_groups = ("00_App", "10_Motor", "20_Parameters", "30_Communication", "50_Shared")

        def files_by_group(tree, project):
            return {
                group.findtext("GroupName"): [
                    (
                        (project.parent / file.findtext("FilePath").replace("\\", "/"))
                        .resolve()
                        .relative_to(ROOT),
                        file.find("FileOption"),
                    )
                    for file in group.findall("./Files/File")
                ]
                for group in tree.findall(".//Group")
            }

        def canonical(element):
            return (
                ET.canonicalize(ET.tostring(element, encoding="unicode"), strip_text=True)
                if element is not None
                else None
            )

        def shared_options(path, options):
            if path.name in ("motor_state_machine.c", "motor_state_rules.c"):
                options = ET.fromstring(ET.tostring(options))
                for tag in ("Optim", "oTime"):
                    options.find(f".//Cads/{tag}").text = "board-timing-option"
            return canonical(options)

        baseline = files_by_group(old, old_path)
        for name in ("ESC-APP-AT32F456", "ESC-APP-AT32F456-Commissioning"):
            project = ROOT / f"projects/at32f456/application/{name}.uvprojx"
            tree = ET.parse(project)
            with self.subTest(target=name):
                actual = files_by_group(tree, project)
                self.assertEqual(list(actual), group_names)
                for setting in ("ToolsetNumber", "ToolsetName", "pCCUsed", "uAC6"):
                    self.assertEqual(tree.findtext(f".//{setting}"), old.findtext(f".//{setting}"))
                for group in shared_groups:
                    self.assertEqual(
                        [
                            (path, shared_options(path, options))
                            for path, options in actual[group]
                            if path.as_posix() != "firmware/communication/telemetry/app_telemetry.c"
                        ],
                        [
                            (path, shared_options(path, options))
                            for path, options in baseline[group]
                        ],
                    )
                for file in tree.findall(".//File"):
                    if file.findtext("FileName") in (
                        "motor_state_machine.c",
                        "motor_state_rules.c",
                    ):
                        self.assertEqual(file.findtext(".//Cads/Optim"), "4")
                        self.assertEqual(file.findtext(".//Cads/oTime"), "1")
                old_cads = old.find(".//TargetArmAds/Cads")
                new_cads = tree.find(".//TargetArmAds/Cads")
                # AT32快环实机时限要求O3；其余公共设置仍必须和STM32一致。
                self.assertEqual(new_cads.findtext("Optim"), "4")
                self.assertEqual(new_cads.findtext("oTime"), "1")
                board_optimization = {"VariousControls", "Optim", "oTime"}
                self.assertEqual(
                    [
                        (item.tag, item.text)
                        for item in new_cads
                        if item.tag not in board_optimization
                    ],
                    [
                        (item.tag, item.text)
                        for item in old_cads
                        if item.tag not in board_optimization
                    ],
                )
                self.assertEqual(
                    canonical(tree.find(".//TargetArmAds/Aads")),
                    canonical(old.find(".//TargetArmAds/Aads")),
                )
                for path, _ in actual["40_Platform"]:
                    self.assertTrue(
                        path.as_posix().startswith(
                            (
                                "firmware/platform/bsp/mcu/at32f456/",
                                "firmware/platform/devices/",
                                "firmware/platform/bsp/can_frame_codec.c",
                                "firmware/platform/bsp/bsp_",
                            )
                        )
                        or path.as_posix().startswith("projects/at32f456/application/")
                        or path.as_posix()
                        in {
                            "projects/at32f456/application/startup_at32f456_app.s",
                            "firmware/platform/devices/encoder_mt6835.c",
                            "firmware/app/runtime/bus_current_feedback.c",
                            "firmware/platform/bsp/pwm_output_permit.c",
                            "firmware/app/runtime/pwm_power_window.c",
                            "firmware/platform/bsp/can_frame_codec.c",
                        }
                    )
                for path, _ in actual["90_Vendor"]:
                    self.assertTrue(
                        path.as_posix().startswith(
                            ("firmware/third_party/", "firmware/platform/sdk/")
                        )
                    )

    def test_all_full_apps_link_own_board_entry_with_identical_template(self):
        """每工程仅链接本板main与装配实现，两板入口模板逐字节一致。"""
        projects = (
            ROOT / "projects/stm32g4/MDK-ARM/ESC-APP-STM32G431.uvprojx",
            ROOT / "projects/stm32g4/MDK-ARM/ESC-APP-STM32G431-LoaderApp.uvprojx",
            ROOT / "projects/at32f456/application/ESC-APP-AT32F456.uvprojx",
            ROOT / "projects/at32f456/application/ESC-APP-AT32F456-Commissioning.uvprojx",
        )
        old_entry = ROOT / "projects/stm32g4/application/main.c"
        new_entry = ROOT / "projects/at32f456/application/main.c"
        self.assertEqual(old_entry.read_bytes(), new_entry.read_bytes())
        for project in projects:
            entries = []
            linked_sources = []
            for node in ET.parse(project).findall(".//FilePath"):
                path = (project.parent / node.text.replace("\\", "/")).resolve()
                linked_sources.append(path)
                if path.suffix == ".c" and re.search(
                    r"(?m)^int main\(void\)", path.read_text(encoding="utf8", errors="replace")
                ):
                    entries.append(path)
            expected = old_entry if "STM32G431" in project.name else new_entry
            self.assertEqual(entries, [expected], project.name)
            board = "stm32g4" if "STM32G431" in project.name else "at32f456"
            self.assertEqual(
                linked_sources.count(ROOT / f"firmware/platform/bsp/mcu/{board}/system.c"), 1
            )
            self.assertNotIn(expected.with_name("platform_app.c"), linked_sources)

    def test_all_full_app_targets_share_identical_portable_sources(self):
        projects = (
            ROOT / "projects/stm32g4/MDK-ARM/ESC-APP-STM32G431.uvprojx",
            ROOT / "projects/at32f456/application/ESC-APP-AT32F456.uvprojx",
            ROOT / "projects/at32f456/application/ESC-APP-AT32F456-Commissioning.uvprojx",
        )
        source_sets = []
        for project in projects:
            paths = (
                (project.parent / node.text.replace("\\", "/")).resolve().relative_to(ROOT)
                for node in ET.parse(project).findall(".//FilePath")
            )
            source_sets.append(
                {
                    path
                    for path in paths
                    if path.suffix == ".c"
                    and path.as_posix()
                    not in {
                        "firmware/communication/telemetry/app_telemetry.c",
                        "firmware/app/runtime/bus_current_feedback.c",
                        "firmware/app/runtime/pwm_power_window.c",
                    }
                    and not path.as_posix().startswith("firmware/app/targets/")
                    and any(
                        path.as_posix().startswith(f"firmware/{folder}/")
                        for folder in ("app", "motor", "parameters", "communication")
                    )
                }
            )
        self.assertTrue(source_sets[0])
        self.assertEqual(source_sets[0], source_sets[1])
        self.assertEqual(source_sets[0], source_sets[2])

    def test_optional_rtt_diagnostics_are_shared_source_selected_only_for_new_apps(self):
        """只读RTT为可选通信模块，不能扩大旧板资源或混入控制源码差异。"""
        diagnostic = ROOT / "firmware/communication/telemetry/app_telemetry.c"
        for project in (ROOT / "projects").rglob("*.uvprojx"):
            sources = [
                (project.parent / node.text.replace("\\", "/")).resolve()
                for node in ET.parse(project).findall(".//FilePath")
            ]
            expected = project.stem in {"ESC-APP-AT32F456", "ESC-APP-AT32F456-Commissioning"}
            self.assertEqual(sources.count(diagnostic), int(expected), project.name)
            if expected:
                self.assertTrue(any(p.name == "SEGGER_RTT.c" for p in sources))

    def test_commissioning_is_separate_bounded_full_app(self):
        base = ROOT / "projects/at32f456/application"
        project = base / "ESC-APP-AT32F456-Commissioning.uvprojx"
        errors, _, _ = check_project(project)
        self.assertEqual(errors, [])
        tree = ET.parse(project)
        defines = target_defines(tree, "r40_app_commissioning.h")
        for flag in (
            "R40_APP_PHASE_LOCK=0",
            "R40_APP_BOUNDED_TEST=1",
            "APP_ENABLED_DIAGNOSTIC_TELEMETRY=0",
            "BSP_REQUIRED_MCU_TEMPERATURE=0",
            "FOC_ACTIVE_MOTOR_PROFILE=2",
            "BSP_ENCODER_FAST_SPI=1",
            "BSP_ENCODER_WORD_TRANSFER=1",
            "BSP_ENCODER_DMA_TRANSFER=1",
        ):
            self.assertIn(flag, defines)
        self.assertEqual(tree.findtext(".//UpdateFlashBeforeDebugging"), "0")
        common = (ROOT / "firmware/app/config/targets/r40_app_common.h").read_text(encoding="utf8")
        self.assertIn("#ifndef APP_ENABLED_DIAGNOSTIC_TELEMETRY", common)
        self.assertNotIn(
            "app_telemetry.o", (base / "r40_app_commissioning.sct").read_text(encoding="utf8")
        )
        paths = [node.text.replace("\\", "/") for node in tree.findall(".//FilePath")]
        for name in (
            "motor_axis.c",
            "encoder_calibration_task.c",
            "pwm_power_window.c",
            "app_telemetry.c",
        ):
            self.assertTrue(any(path.endswith("/" + name) for path in paths))
        self.assertFalse(
            any("closedloop_bench" in path or "openloop_model" in path for path in paths)
        )

    def test_app_stack_is_separate_from_vendor(self):
        base = ROOT / "projects/at32f456/application"
        local = (base / "startup_at32f456_app.s").read_text(encoding="utf8")
        vendor = (ROOT / "firmware/platform/sdk/at32f45x/cmsis/startup_at32f45x.s").read_text(
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
        project = (base / "ESC-APP-AT32F456.uvprojx").read_text(encoding="utf8")
        self.assertIn("<FilePath>startup_at32f456_app.s</FilePath>", project)

    def test_app_off_links_shared_application_and_cannot_enable_power(self):
        project = ROOT / "projects/at32f456/application/ESC-APP-AT32F456.uvprojx"
        errors, _, _ = check_project(project)
        self.assertEqual(errors, [])
        tree = ET.parse(project)
        defines = target_defines(tree, "r40_app_off.h")
        for flag in (
            "R40_APP_PHASE_LOCK=1",
            "BSP_APP_LIFECYCLE=1",
            "APP_TARGET_R40_F456=1",
            "FOC_ACTIVE_HW_PROFILE=2",
            "FOC_ACTIVE_MOTOR_PROFILE=2",
            "BSP_ENCODER_FAST_SPI=1",
            "BSP_ENCODER_WORD_TRANSFER=1",
            "BSP_ENCODER_DMA_TRANSFER=1",
        ):
            self.assertIn(flag, defines)
        paths = [value.text.replace("\\", "/") for value in tree.findall(".//FilePath")]
        for name in (
            "app_startup.c",
            "app_tasks.c",
            "motor_axis.c",
            "motor_state_machine.c",
            "system.c",
            "led.c",
            "boot.c",
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
        entry = (ROOT / "projects/at32f456/application/main.c").read_text(encoding="utf8")
        platform = (ROOT / "firmware/platform/bsp/mcu/at32f456/system.c").read_text(encoding="utf8")
        startup = (ROOT / "firmware/app/composition/app_startup.c").read_text(encoding="utf8")
        for call in (
            "App_Init();",
            "App_Process();",
            "Bsp_Init()",
            "Bsp_Start()",
            "App_Start();",
        ):
            self.assertIn(call, entry)
        for call in ("BspApp_OnFastSample();", "BspApp_OnControlTick();"):
            self.assertIn(call, platform)
        for method in ("Init", "Start"):
            self.assertIn(f"BspResult Bsp_{method}(void)", platform)
        self.assertIn("int main(void)", entry)
        self.assertNotIn("int main(void)", startup)
        self.assertNotIn("int main(void)", platform)
        self.assertNotIn("param_store_write(", platform)

    def test_closedloop_is_explicit_and_uses_unchanged_control_modules(self):
        base = ROOT / "tests/bench/at32f456"
        project = ROOT / "projects/at32f456/bench/R40_F456_ClosedLoop.uvprojx"
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
        base = ROOT / "tests/bench/at32f456"
        projects = [
            ROOT / f"projects/at32f456/bench/R40_F456_OpenLoop_{kind}.uvprojx"
            for kind in ("Dry", "Power", "Combined")
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
        entry = (base / "openloop_main.c").read_text(encoding="utf-8")
        self.assertNotIn("BspCan_Init(", entry)
        self.assertNotIn("comm_hw_", entry)
        self.assertNotIn("param_store_", entry)
        self.assertIn("#define R40_OPENLOOP_POWER_ENABLED 0", entry)
        for project in projects:
            errors, _, _ = check_project(project)
            self.assertEqual(errors, [])

    def test_foundation_project_is_isolated_and_has_no_build_hooks(self):
        project = ROOT / "projects/at32f456/bench/R40_F456_Foundation.uvprojx"
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
        entry = (ROOT / "tests/bench/at32f456/main.c").read_text(encoding="utf-8")
        self.assertNotIn("BspPwm_Enable(", entry)
        self.assertNotIn("App_FastTick(", entry)
        self.assertIn("../../../firmware/platform/devices/encoder_mt6835.c", paths)
        self.assertIn("../../../firmware/platform/bsp/mcu/at32f456/encoder_spi.c", paths)
        self.assertIn("Encoder_Init();", entry)
        self.assertLess(entry.index("Encoder_Init();"), entry.index("Adc_Start()"))

        old_project = ROOT / "projects/stm32g4/MDK-ARM/ESC-APP-STM32G431.uvprojx"
        # 器件协议由板配置选中，工程可纳入全部纯协议源；MCU实现仍必须隔离。
        old_paths = [
            node.text.replace("\\", "/") for node in ET.parse(old_project).findall(".//FilePath")
        ]
        self.assertFalse(any("/mcu/at32f456/" in path for path in old_paths))
        self.assertNotIn("param_store_write(", entry)
        self.assertNotIn("AdcTemperature_Poll(", entry)
        self.assertIn("../../../firmware/platform/bsp/mcu/at32f456/temperature.c", paths)
        self.assertIn("BspAdc_ReadChannel(BSP_ADC_MOTOR_TEMPERATURE", entry)
        self.assertNotIn("motor_temperature_at32f456", old_project.read_text(encoding="utf-8"))

    def test_parameter_sectors_are_excluded_from_image(self):
        layout = (ROOT / "firmware/platform/config/boards/at32f456.h").read_text(encoding="utf-8")
        values = {
            name: int(value, 0)
            for name, value in re.findall(r"#define (R40_\w+) (0x[0-9A-Fa-f]+|[0-9]+)U", layout)
        }
        scatter = (ROOT / "projects/at32f456/bench/r40_foundation.sct").read_text(encoding="utf-8")
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
        base = ROOT / "firmware/platform/sdk/at32f45x"
        manifest = json.loads((base / "manifest.json").read_text(encoding="utf-8"))
        self.assertTrue(manifest["files"])
        for entry in manifest["files"]:
            with self.subTest(path=entry["path"]):
                self.assertEqual(
                    hashlib.sha256((base / entry["path"]).read_bytes()).hexdigest(),
                    entry["sha256"],
                )

    def test_keil_rtt_is_isolated_and_vendor_files_are_pinned(self):
        tree = ET.parse(ROOT / "projects/at32f456/bench/R40_F456_OpenLoop_Combined.uvprojx")
        self.assertIn("R40_KEIL_OPENLOOP=1", tree.findtext(".//Cads/VariousControls/Define"))
        for name in ("R40_F456_ClosedLoop", "R40_F456_Foundation", "R40_F456_OpenLoop_Dry"):
            self.assertNotIn(
                "openloop_telemetry.c",
                (ROOT / f"projects/at32f456/bench/{name}.uvprojx").read_text(),
            )
        vendor = ROOT / "firmware/third_party/segger_rtt"
        manifest = json.loads((vendor / "manifest.json").read_text())
        self.assertEqual(len(manifest["revision"]), 40)
        for entry in manifest["files"]:
            self.assertEqual(
                hashlib.sha256((vendor / entry["path"]).read_bytes()).hexdigest(), entry["sha256"]
            )


if __name__ == "__main__":
    unittest.main()
