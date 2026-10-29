"""目标配置贯通回归；临时目标仅用于离线测试，不代表新增 MCU 驱动。"""

import copy
import json
import shutil
import struct
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
from unittest.mock import patch

from host_app.upgrade_client import UpgradePackage
from loader.tools import generate_target, package_image
from tools.build import check_project_layout, generate_loader_app_project
from tools.build.build_loader_bundle import verify_target_inputs

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "shared/targets"))
from target_contract import load_target, repository_path, target_value  # noqa: E402


class LoaderTargetTests(unittest.TestCase):
    """用不同目标身份验证显式选择，默认 G431 和拒绝路径均保留。"""

    def setUp(self):
        """读取真实目标并创建隔离的派生配置。"""
        self.target = load_target()
        self.temp = tempfile.TemporaryDirectory(dir=ROOT / "outputs")
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)

    def save(self, target):
        """保存测试目标，不向正式目标目录注册。"""
        path = self.root / (target["target"] + ".json")
        path.write_text(json.dumps(target), encoding="utf-8")
        return path

    def test_default_and_explicit_g431_match(self):
        """默认目标与显式名称产生同一个契约。"""
        self.assertEqual(self.target, load_target("vector_mini_st_g431"))

    def test_project_name_accepts_hyphens_but_not_paths_or_commands(self):
        """工程名支持连字符，不放宽路径、命令字符或发布MCU约束。"""
        target = copy.deepcopy(self.target)
        target["build"]["app_name"] = "ESC-APP-STM32G431-LoaderApp"
        self.assertEqual(load_target(self.save(target)), target)
        for key, value in (
            ("app_name", "../outside"),
            ("app_name", "bad\\path"),
            ("app_name", "bad;command"),
            ("app_name", "-option"),
            ("artifact_mcu", "STM32-G431"),
        ):
            invalid = copy.deepcopy(target)
            invalid["build"][key] = value
            with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                load_target(self.save(invalid))

    def test_unknown_name_does_not_fall_back(self):
        """选择不存在的目标不会静默使用 G431。"""
        with self.assertRaises(FileNotFoundError):
            load_target("missing_target", directory=self.root)

    def test_contract_rejects_unsupported_or_overlapping_layout(self):
        """写入单位、启动架构和分区不兼容时在接触设备前失败。"""
        for field, value in (
            ("TARGET_PROGRAM_BYTES", "16U"),
            ("TARGET_APP_BYTES", "0x40000U"),
            ("TARGET_APP_BASE", "0x08000000U"),
            ("TARGET_MAILBOX_ADDRESS", "TARGET_MAILBOX_ADDRESS"),
        ):
            target = copy.deepcopy(self.target)
            target["constants"][field] = value
            with self.subTest(field=field), self.assertRaises(ValueError):
                load_target(self.save(target))
        target = copy.deepcopy(self.target)
        target["image_architecture"] = "unsupported-test-architecture"
        with self.assertRaises(ValueError):
            load_target(self.save(target))

    def test_path_cannot_escape_repository(self):
        """构建输入不能通过相对路径越过目标仓库。"""
        with self.assertRaises(ValueError):
            repository_path(self.root, "../outside")

    def test_explicit_target_package_and_host_agree(self):
        """同一显式身份可以贯通打包与主机，默认 G431 必须拒绝。"""
        target = copy.deepcopy(self.target)
        target["target"] = "offline_fixture"
        target["constants"]["TARGET_MCU"] = "0x12345678U"
        path = self.save(target)
        vector = json.loads(
            (ROOT / "tests/fixtures/loader_image_v3.json").read_text(encoding="utf-8")
        )
        raw = bytearray.fromhex(vector["source_hex"])
        offset = target_value(target, "TARGET_DESCRIPTOR_OFFSET")
        struct.pack_into("<I", raw, 4, target_value(target, "TARGET_APP_BASE") + offset + 97)
        for at, key in (
            (12, "TARGET_LAYOUT"),
            (16, "TARGET_MCU"),
            (20, "TARGET_PRODUCT"),
            (24, "TARGET_HARDWARE"),
            (28, "TARGET_APP_VERSION"),
            (36, "TARGET_PARAM_ABI"),
        ):
            struct.pack_into(">I", raw, offset + at, target_value(target, key))
        schema = target_value(target, "TARGET_SCHEMA")
        struct.pack_into(">HH", raw, offset + 40, schema, schema)
        image, manifest, metadata = package_image.build_package(raw, 7, target)
        package = self.root / "package"
        package.mkdir()
        (package / "image.bin").write_bytes(image)
        (package / "manifest.bin").write_bytes(manifest)
        (package / "package.json").write_text(json.dumps(metadata), encoding="utf-8")
        self.assertEqual(UpgradePackage.load(package, path).image, image)
        with self.assertRaises(ValueError):
            UpgradePackage.load(package)

    def test_generators_use_selected_build_inputs(self):
        """两个真实生成器读取同一目标并让 APP 后处理继承该选择。"""
        target = copy.deepcopy(self.target)
        target["target"] = "offline_fixture"
        build = target["build"]
        build.update(
            target_directory="loader/targets/offline_fixture",
            selection_directory="shared/targets/offline_fixture",
            target_header="shared/targets/offline_fixture.h",
            loader_project="loader/targets/offline_fixture/fixture.uvprojx",
            app_project="app_fixture/fixture.uvprojx",
            app_defines="USE_HAL_DRIVER,OFFLINE_MCU,BOARD_PRODUCT_VALUE=6U",
        )
        target["device"] = "OFFLINE_MCU_DEVICE"
        source_template = ROOT / build["template"]
        template = self.root / build["template"]
        template.parent.mkdir(parents=True)
        (self.root / "shared/targets").mkdir(parents=True)
        shutil.copy2(source_template, template)
        path = self.save(target)
        with (
            patch.object(generate_target, "ROOT", self.root),
            patch.object(sys, "argv", ["generate_target", "--target", str(path)]),
        ):
            generate_target.main()
        with (
            patch.object(generate_loader_app_project, "ROOT", self.root),
            patch.object(sys, "argv", ["generate_app", "--target", str(path)]),
        ):
            generate_loader_app_project.main()
        selection = (self.root / build["selection_directory"] / "loader_target.h").read_text(
            encoding="utf-8"
        )
        self.assertIn("offline_fixture.h", selection)
        project = (self.root / build["app_project"]).read_text(encoding="utf-8")
        self.assertIn("--target offline_fixture", project)
        self.assertIn("offline_fixture", project)
        self.assertTrue((self.root / build["loader_project"]).is_file())
        parsed = ET.fromstring(project)
        self.assertEqual(parsed.findtext(".//TargetCommonOption/Device"), "OFFLINE_MCU_DEVICE")
        defines = parsed.findtext(".//Cads/VariousControls/Define")
        self.assertIn("OFFLINE_MCU", defines)
        self.assertIn("BOARD_PRODUCT_VALUE=6U", defines)
        self.assertNotIn("STM32G431xx", defines)
        generated_sources = {
            ((self.root / build["app_project"]).parent / node.text.replace("\\", "/")).resolve()
            for node in parsed.findall(".//FilePath")
        }
        self.assertIn(
            (self.root / "projects/stm32g4/application/main.c").resolve(),
            generated_sources,
        )
        # 工程、链接、源清单和后处理均来自同一清单；任何一个过期都应在编译前拒绝。
        for relative in (
            build["loader_project"],
            build["app_project"],
            build["target_directory"] + "/loader.sct",
            build["target_directory"] + "/app.sct",
        ):
            file = self.root / relative
            original = file.read_text(encoding="utf-8")
            file.write_text(original + "\n; stale target input\n", encoding="utf-8")
            with self.subTest(path=relative), self.assertRaises(ValueError):
                verify_target_inputs(self.root, target)
            file.write_text(original, encoding="utf-8")
        verify_target_inputs(self.root, target)
        header = self.root / build["target_header"]
        header.write_text(
            header.read_text(encoding="utf-8").replace("#define TARGET_MCU", "#define OTHER_MCU"),
            encoding="utf-8",
        )
        with self.assertRaises(ValueError):
            verify_target_inputs(self.root, target)

    def test_project_checks_only_reachable_headers(self):
        """实际编译单元的缺失头必须拒绝，不混入另一目标的未引用源码。"""
        source = self.root / "loader/runtime/main.c"
        source.parent.mkdir(parents=True)
        source.write_text('#include "port.h"', encoding="utf-8")
        headers = self.root / "loader/ports/include"
        headers.mkdir(parents=True)
        header = headers / "port.h"
        header.write_text('#include "missing.h"', encoding="utf-8")
        unused = source.parent / "other_target.c"
        unused.write_text('#include "other_target_missing.h"', encoding="utf-8")
        project = self.root / "test.uvprojx"
        project.write_text(
            "<Project><IncludePath>loader/ports/include</IncludePath>"
            "<FilePath>loader/runtime/main.c</FilePath></Project>",
            encoding="utf-8",
        )
        with patch.object(check_project_layout, "ROOT", self.root):
            errors, _, _ = check_project_layout.check_project(project)
            self.assertEqual(len(errors), 1)
            self.assertIn("missing.h", errors[0])
            header.write_text("", encoding="utf-8")
            self.assertEqual(check_project_layout.check_project(project)[0], [])

    def test_forced_configuration_has_explicit_project_path(self):
        """强制包含不能靠无关include目录碰巧找到已搬迁的目标配置。"""
        config = self.root / "config.h"
        config.write_text("", encoding="utf-8")
        project = self.root / "test.uvprojx"
        for option in ("--preinclude", "-include"):
            for filename, valid in (("config.h", True), ("stale/config.h", False)):
                project.write_text(
                    f"<Project><MiscControls>{option} {filename}</MiscControls></Project>",
                    encoding="utf-8",
                )
                with self.subTest(option=option, filename=filename):
                    errors, _, _ = check_project_layout.check_project(project)
                    self.assertEqual(not errors, valid)
