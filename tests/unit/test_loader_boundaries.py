"""Loader 分层检查的反例测试；只扫描临时源文件，不编译或连接设备。"""

import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from tools.harness import check_architecture


class LoaderBoundaryTests(unittest.TestCase):
    """验证真实分层规则会拒绝端口反向依赖和未解析厂商头。"""

    def scan(self, source: str, layer: str) -> list:
        """在小型目录中应用仓库规则，不替换扫描实现。"""
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder).resolve()
            paths = {
                "loader_core": "loader/core",
                "loader_runtime": "loader/runtime",
                "loader_api": "loader/ports/include",
                "loader_mcu": "loader/ports/stm32g4",
                "loader_board": "loader/boards/test",
                "shared": "shared/image",
                "target_profile": "shared/targets",
            }
            for directory in paths.values():
                (root / directory).mkdir(parents=True)
            (root / paths["loader_api"] / "platform.h").write_text("", encoding="utf-8")
            (root / paths["loader_mcu"] / "private.h").write_text("", encoding="utf-8")
            (root / paths["target_profile"] / "loader_target.h").write_text("", encoding="utf-8")
            (root / paths["shared"] / "image.h").write_text("", encoding="utf-8")
            (root / paths[layer] / "example.c").write_text(source, encoding="utf-8")
            layers, _ = check_architecture.layer_config()
            roots = [(root / directory, name) for name, directory in paths.items()]
            roots.sort(key=lambda item: len(item[0].parts), reverse=True)
            with (
                patch.object(check_architecture, "ROOT", root),
                patch.object(check_architecture, "layer_config", return_value=(layers, roots)),
            ):
                return check_architecture.scan()

    def test_core_rejects_mcu_private_header(self):
        """核心不得通过相对路径穿透平台边界。"""
        errors = self.scan('#include "../ports/stm32g4/private.h"', "loader_core")
        self.assertEqual(len(errors), 1)
        self.assertEqual(errors[0]["to_layer"], "loader_mcu")

    def test_portable_layers_reject_unresolved_vendor_headers(self):
        """厂商头未注册在依赖图中，也不能绕过检查。"""
        for layer in ("loader_core", "loader_runtime", "loader_api"):
            with self.subTest(layer=layer):
                errors = self.scan("#include <stm32g4xx_hal.h>", layer)
                self.assertEqual(len(errors), 1)
                self.assertEqual(errors[0]["to_layer"], "hardware-header")

    def test_runtime_accepts_public_port(self):
        """组合层可以调用不暴露厂商类型的公共端口。"""
        self.assertEqual(self.scan('#include "platform.h"', "loader_runtime"), [])

    def test_core_accepts_shared_image(self):
        """通用核心继续复用共享镜像契约。"""
        self.assertEqual(self.scan('#include "image.h"', "loader_core"), [])

    def test_mcu_keeps_vendor_headers(self):
        """真实硬件端口不禁止使用厂商头。"""
        self.assertEqual(self.scan('#include "stm32g4xx_hal.h"', "loader_mcu"), [])

    def test_core_rejects_target_profile(self):
        """目标组合不应从共享目标目录渗入核心。"""
        errors = self.scan('#include "loader_target.h"', "loader_core")
        self.assertEqual(len(errors), 1)
        self.assertEqual(errors[0]["to_layer"], "target_profile")

    def test_future_mcu_directories_are_classified(self):
        """新增 MCU 端口默认受分层检查，公共接口仍归独立层。"""
        _, roots = check_architecture.layer_config()
        self.assertEqual(
            check_architecture.owner(
                check_architecture.ROOT / "loader/ports/future_family/flash.c", roots
            ),
            "loader_mcu",
        )
        self.assertEqual(
            check_architecture.owner(
                check_architecture.ROOT / "loader/ports/include/platform.h", roots
            ),
            "loader_api",
        )
