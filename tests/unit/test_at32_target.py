"""AT32F456基础工程的来源、隔离和构建安全检查；不编译或连接硬件。"""

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


def check_target_options(tree: ET.ElementTree, target: str) -> None:
    """工程只选择板卡和应用构建；不再解析已删除的逐项功能配置。"""
    options = tree.findtext(".//Cads/VariousControls/MiscControls").replace("\\", "/")
    expected = (
        f"--diag_suppress=1 -DFIRMWARE_TARGET_{target} "
        + "--preinclude ../../../firmware/config/board_config.h"
    )
    if options != expected:
        raise AssertionError(f"Unexpected target configuration entry: {options}")
    defines = tree.findtext(".//Cads/VariousControls/Define")
    if "BSP_BOARD=2U" not in defines:
        raise AssertionError("AT32工程必须显式选择板卡")


class At32TargetTests(unittest.TestCase):
    def test_full_app_grouping_and_common_options_match_old_board(self):
        """双板沿用公共工程配置；AT32快环采用明确的O3速度优化。"""
        old_path = ROOT / "projects/stm32g4/MDK-ARM/ESC-APP-STM32G431.uvprojx"
        old = ET.parse(old_path)
        group_names = [group.findtext("GroupName") for group in old.findall(".//Group")]
        shared_groups = ("00_App", "10_Motor", "20_Parameters", "30_Communication", "50_Shared")
        # AT32 使用用户整理后的显示名称；共享源码、选项及板级隔离仍逐项比较。
        at32_group_names = {
            "00_App": "App",
            "10_Motor": "Motor",
            "20_Parameters": "Parameters",
            "30_Communication": "Communication",
            "40_Platform": "Platform",
            "50_Shared": "Shared",
            "90_Vendor": "Mcu_SDK",
        }

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
            if path.name in (
                "motor_state_machine.c",
                "motor_lifecycle.c",
            ):
                options = ET.fromstring(ET.tostring(options))
                for tag in ("Optim", "oTime"):
                    options.find(f".//Cads/{tag}").text = "board-timing-option"
            return canonical(options)

        baseline = files_by_group(old, old_path)
        for name in ("ESC-APP-AT32F456",):
            project = ROOT / f"projects/at32f456/application/{name}.uvprojx"
            tree = ET.parse(project)
            with self.subTest(target=name):
                actual = files_by_group(tree, project)
                self.assertEqual(list(actual), [*(at32_group_names[g] for g in group_names), "RTT"])
                self.assertEqual(
                    [path.as_posix() for path, _ in actual["RTT"]],
                    ["firmware/third_party/segger_rtt/SEGGER_RTT.c"],
                )
                self.assertIn(
                    Path("firmware/app/composition/app_rtt.c"),
                    [path for path, _ in actual["App"]],
                )
                for setting in ("ToolsetNumber", "ToolsetName", "pCCUsed", "uAC6"):
                    self.assertEqual(tree.findtext(f".//{setting}"), old.findtext(f".//{setting}"))
                for group in shared_groups:
                    # 文件显示顺序可因板工程整理不同；共享源码和编译选项必须一致。
                    self.assertEqual(
                        sorted(
                            (path, shared_options(path, options))
                            for path, options in actual[at32_group_names[group]]
                            if path != Path("firmware/app/composition/app_rtt.c")
                        ),
                        sorted(
                            (path, shared_options(path, options))
                            for path, options in baseline[group]
                        ),
                    )
                for file in tree.findall(".//File"):
                    if file.findtext("FileName") in (
                        "motor_state_machine.c",
                        "motor_lifecycle.c",
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
                for path, _ in actual["Platform"]:
                    self.assertTrue(
                        path.as_posix().startswith(
                            (
                                "firmware/platform/bsp/mcu/at32f456/",
                                "firmware/platform/devices/",
                                "firmware/platform/bsp/bsp_",
                            )
                        )
                        or path.as_posix().startswith("projects/at32f456/application/")
                        or path.as_posix()
                        in {
                            "projects/at32f456/application/startup_at32f456_app.s",
                            "firmware/platform/devices/encoder_mt6835.c",
                            "firmware/app/composition/bus_current_feedback.c",
                            "firmware/platform/bsp/pwm_output_permit.c",
                        }
                    )
                for path, _ in actual["Mcu_SDK"]:
                    self.assertTrue(
                        path.as_posix().startswith(
                            ("firmware/third_party/", "firmware/platform/sdk/")
                        )
                    )

    def test_all_full_apps_link_own_board_entry(self):
        """每工程仅链接本板main与装配实现；ADC适配随MCU不同。"""
        projects = (
            ROOT / "projects/stm32g4/MDK-ARM/ESC-APP-STM32G431.uvprojx",
            ROOT / "projects/stm32g4/MDK-ARM/ESC-APP-STM32G431-LoaderApp.uvprojx",
            ROOT / "projects/at32f456/application/ESC-APP-AT32F456.uvprojx",
        )
        old_entry = ROOT / "projects/stm32g4/application/main.c"
        new_entry = ROOT / "projects/at32f456/application/main.c"
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
                        "firmware/app/composition/bus_current_feedback.c",
                        "firmware/app/composition/app_rtt.c",  # 板级选择的调试通道。
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

    def test_fast_control_and_sensing_remain_in_sram(self):
        """控制和测量拆分后保留原SRAM快环，LUT提交仍留在Flash。"""
        base = ROOT / "projects/at32f456/application"
        for filename in ("at32f456_app.sct",):
            with self.subTest(filename=filename):
                text = (base / filename).read_text(encoding="utf-8")
                flash, fast = text.split("ER_FAST", 1)
                fast = fast.split("}", 1)[0]
                for obj in (
                    "foc*.o",
                    "pi_controller.o",
                    "motor_current.o",
                    "motor_voltage.o",
                    "mcu_temperature_sensing.o",
                    "encoder_feedback.o",
                ):
                    self.assertIn(f"{obj} (+RO)", fast)
                self.assertIn("encoder_calibration.o (i.EncoderCalibration_PublishLut)", flash)

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

    def test_app_off_links_shared_application_and_explicit_board(self):
        project = ROOT / "projects/at32f456/application/ESC-APP-AT32F456.uvprojx"
        errors, _, _ = check_project(project)
        self.assertEqual(errors, [])
        tree = ET.parse(project)
        check_target_options(tree, "AT32_OFF")
        paths = [value.text.replace("\\", "/") for value in tree.findall(".//FilePath")]
        for name in (
            "app.c",
            "app_protocol.c",
            "motor_axis.c",
            "motor_state_machine.c",
            "system.c",
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
        startup = (ROOT / "firmware/app/composition/app.c").read_text(encoding="utf8")
        for call in (
            "App_Init();",
            "App_Process();",
            "Bsp_Init()",
            "Bsp_Start()",
            "App_Start();",
        ):
            self.assertIn(call, entry)
        adc = (ROOT / "firmware/platform/bsp/mcu/at32f456/adc.c").read_text(encoding="utf8")
        self.assertIn("if (BspAdc_CompleteFast())", entry)
        self.assertIn("MotorAxis_SampleFast();", entry)
        self.assertIn("MotorAxis_ControlFast();", entry)
        self.assertIn("++g_app_fast_samples;", entry)
        self.assertNotIn("BspAdc_HandleInterrupt", adc)
        self.assertIn("BspApp_OnControlTick();", platform)
        for method in ("Init", "Start"):
            self.assertIn(f"BspResult Bsp_{method}(void)", platform)
        self.assertIn("int main(void)", entry)
        self.assertNotIn("int main(void)", startup)
        self.assertNotIn("int main(void)", platform)
        self.assertNotIn("param_store_write(", platform)

    def test_closedloop_is_explicit_and_uses_unchanged_control_modules(self):
        base = ROOT / "tests/bench/at32f456"
        project = ROOT / "projects/at32f456/bench/AT32F456_ClosedLoop.uvprojx"
        errors, _, _ = check_project(project)
        self.assertEqual(errors, [])
        tree = ET.parse(project)
        defines = tree.findtext(".//Cads/VariousControls/Define")
        self.assertIn("AT32_CLOSEDLOOP_TEST=1", defines)
        self.assertIn("AT32_OL_CURRENT_LIMIT_A=3.0f", defines)
        self.assertEqual(tree.findtext(".//UpdateFlashBeforeDebugging"), "0")
        files = [node.text.replace("\\", "/") for node in tree.findall(".//FilePath")]
        for name in ("foc.c", "pi_controller.c", "speed_control.c"):
            self.assertTrue(any(path.endswith("/" + name) for path in files))
        entry = (base / "closedloop_bench.c").read_text(encoding="utf-8")
        self.assertIn("Foc_RunCurrentRef(", entry)
        self.assertIn("SpeedControl_Update(", entry)
        self.assertNotIn("param_store_", entry)
        self.assertNotIn("comm_hw_", entry)

    def test_openloop_is_explicit_and_defaults_to_no_power(self):
        base = ROOT / "tests/bench/at32f456"
        projects = [
            ROOT / f"projects/at32f456/bench/AT32F456_OpenLoop_{kind}.uvprojx"
            for kind in ("Dry", "Power", "Combined")
        ]
        targets = [ET.parse(project).find(".//Target") for project in projects]
        self.assertEqual(
            [t.findtext("TargetName") for t in targets],
            ["AT32F456_OpenLoop_Dry", "AT32F456_OpenLoop_Power", "AT32F456_OpenLoop_Combined"],
        )
        for target, flag in zip(targets, ("0", "1", "1"), strict=True):
            self.assertIn(
                f"AT32_OPENLOOP_POWER_ENABLED={flag}",
                target.findtext(".//Cads/VariousControls/Define"),
            )
            self.assertEqual(target.findtext(".//UpdateFlashBeforeDebugging"), "0")
        entry = (base / "openloop_main.c").read_text(encoding="utf-8")
        self.assertNotIn("BspCan_Init(", entry)
        self.assertNotIn("comm_hw_", entry)
        self.assertNotIn("param_store_", entry)
        self.assertIn("#define AT32_OPENLOOP_POWER_ENABLED 0", entry)
        for project in projects:
            errors, _, _ = check_project(project)
            self.assertEqual(errors, [])

    def test_foundation_project_is_isolated_and_has_no_build_hooks(self):
        project = ROOT / "projects/at32f456/bench/AT32F456_Foundation.uvprojx"
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
        self.assertFalse(any("/platform/devices/" in path for path in paths))
        self.assertIn("../../../firmware/platform/bsp/mcu/at32f456/encoder_spi.c", paths)
        self.assertIn("McuEncoder_Init();", entry)
        self.assertLess(entry.index("McuEncoder_Init();"), entry.index("Adc_Start()"))

        old_project = ROOT / "projects/stm32g4/MDK-ARM/ESC-APP-STM32G431.uvprojx"
        # 器件协议由板配置选中，工程可纳入全部纯协议源；MCU实现仍必须隔离。
        old_paths = [
            node.text.replace("\\", "/") for node in ET.parse(old_project).findall(".//FilePath")
        ]
        self.assertFalse(any("/mcu/at32f456/" in path for path in old_paths))
        self.assertNotIn("param_store_write(", entry)
        self.assertNotIn("AdcTemperature_Poll(", entry)
        self.assertIn("../../../firmware/platform/bsp/mcu/at32f456/adc.c", paths)
        self.assertIn("BspAdc_ReadChannel(BSP_ADC_MOTOR_TEMPERATURE", entry)
        self.assertNotIn("motor_temperature_at32f456", old_project.read_text(encoding="utf-8"))

    def test_parameter_sectors_are_excluded_from_image(self):
        layout = (ROOT / "firmware/config/board_config.h").read_text(encoding="utf-8")
        values = {
            name: int(value, 0)
            for name, value in re.findall(
                r"#define\s+(AT32_\w+)\s+(0x[0-9A-Fa-f]+|[0-9]+)U", layout
            )
        }
        scatter = (ROOT / "projects/at32f456/bench/at32f456_foundation.sct").read_text(
            encoding="utf-8"
        )
        ranges = re.findall(r"(?:LR_IROM1|ER_IROM1)\s+(0x\w+)\s+(0x\w+)", scatter)
        self.assertEqual(len(ranges), 2)
        for start, length in ranges:
            self.assertEqual(int(start, 0) + int(length, 0), values["AT32_PARAM_BASE"])
        self.assertEqual(values["AT32_PARAM_BASE"] % values["AT32_FLASH_SECTOR_BYTES"], 0)
        self.assertLessEqual(values["AT32_PARAM_RAW_BYTES"], 3 * values["AT32_FLASH_SECTOR_BYTES"])
        self.assertEqual(
            values["AT32_PARAM_BASE"] + values["AT32_PARAM_BYTES"],
            values["AT32_FLASH_BASE"] + values["AT32_FLASH_BYTES"],
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
        tree = ET.parse(ROOT / "projects/at32f456/bench/AT32F456_OpenLoop_Combined.uvprojx")
        self.assertIn("AT32_KEIL_OPENLOOP=1", tree.findtext(".//Cads/VariousControls/Define"))
        for name in ("AT32F456_ClosedLoop", "AT32F456_Foundation", "AT32F456_OpenLoop_Dry"):
            self.assertNotIn(
                "openloop_telemetry.c",
                (ROOT / f"projects/at32f456/bench/{name}.uvprojx").read_text(),
            )
        vendor = ROOT / "firmware/third_party/segger_rtt"
        manifest = json.loads((vendor / "manifest.json").read_text(encoding="utf-8"))
        self.assertEqual(len(manifest["revision"]), 40)
        self.assertEqual(
            {entry["path"] for entry in manifest["files"]},
            {
                path.name
                for pattern in ("*.c", "*.h", "LICENSE.md")
                for path in vendor.glob(pattern)
            },
        )
        for entry in manifest["files"]:
            self.assertEqual(
                hashlib.sha256((vendor / entry["path"]).read_bytes()).hexdigest(), entry["sha256"]
            )


if __name__ == "__main__":
    unittest.main()
