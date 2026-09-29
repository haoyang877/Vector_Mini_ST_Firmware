"""验证Loader启动救援和APP喂狗组合层，全部硬件由内存桩替换。"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tools"))
from harness.common import zig_path
from project_paths import NATIVE_INCLUDE_FLAGS, ROOT
from run_position_servo_tests import function_source


def main() -> None:
    """编译真实组合层并将编译、运行日志保存到outputs。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", default=None)
    parser.add_argument("--image-bin", type=Path, help="可选：用实际打包镜像替代合成黄金镜像")
    parser.add_argument("--out", type=Path, default=ROOT / "outputs/tests/loader_runtime")
    args = parser.parse_args()
    compiler = Path(args.cc) if args.cc else zig_path()
    if compiler is None:
        parser.error("未找到纯内存测试编译器")
    output = args.out.resolve()
    output.mkdir(parents=True, exist_ok=True)
    if args.image_bin:
        image = args.image_bin.read_bytes()
    else:
        vector = json.loads(
            (ROOT / "tests/fixtures/loader_image_v2.json").read_text(encoding="utf-8")
        )
        image = bytes.fromhex(vector["image_hex"])
    header = "static const uint8_t LoaderV2Image[] = {" + ",".join(map(str, image)) + "};\n"
    (output / "loader_image_v2_vector.h").write_text(header, encoding="utf-8")
    board = (ROOT / "firmware/app/board_config.c").read_text(encoding="utf-8")
    health_fixture = (ROOT / "tests/unit/loader_app_health_test.c").read_text(encoding="utf-8")
    interval = next(
        line for line in board.splitlines() if line.startswith("#define BOOT_HEALTH_INTERVAL_MS")
    )
    health_source = output / "app_health.c"
    health_source.write_text(
        interval + "\n" + health_fixture + "\n" + function_source(board, "Board_ServiceBoot"),
        encoding="utf-8",
    )
    platform = (ROOT / "loader/ports/stm32g4/platform.c").read_text(encoding="utf-8")
    device = (
        ROOT
        / "firmware/platform/stm32g4/cubemx/Drivers/CMSIS/Device/ST/STM32G4xx/Include/stm32g431xx.h"
    ).read_text(encoding="utf-8")
    sr_defines = "\n".join(
        line for line in device.splitlines() if line.startswith("#define FLASH_SR_")
    )
    error_defines = (
        "#define FLASH_ERR_MASK"
        + platform.split("#define FLASH_ERR_MASK", 1)[1].split("#define FLASH_KEY1_VALUE", 1)[0]
    )
    flash_source = output / "flash_wait.c"
    flash_source.write_text(
        sr_defines
        + "\n"
        + error_defines
        + (ROOT / "tests/unit/loader_flash_wait_test.c").read_text(encoding="utf-8")
        + "\n"
        + function_source(platform, "flash_ready"),
        encoding="utf-8",
    )
    base = [str(compiler)] + (["cc"] if compiler.stem.lower() == "zig" else [])
    base += NATIVE_INCLUDE_FLAGS + [
        "-I",
        str(ROOT / "firmware/services/lifecycle"),
        "-I",
        str(ROOT / "loader/ports/stm32g4"),
        "-I",
        str(ROOT / "loader/core"),
        "-I",
        str(output),
        "-std=c99",
        "-O1",
        "-UNDEBUG",
        "-Wall",
        "-Wextra",
        "-Werror",
        "-ffunction-sections",
        "-Wl,--gc-sections",
    ]
    sources = (
        "loader/core/upgrade.c",
        "shared/image/image_transfer.c",
        "shared/boot/image_mailbox.c",
        "shared/checksum/crc32.c",
        "shared/protocol/yg_protocol_upgrade_wire.c",
        "shared/protocol/yg_protocol_can_id.c",
        "shared/protocol/yg_protocol_frame_codec.c",
        "shared/protocol/yg_protocol_crc.c",
        "shared/protocol/yg_protocol_fragment.c",
    )
    logs = []
    for name, source, extra in (
        ("runtime", "tests/unit/loader_runtime_test.c", [str(ROOT / f) for f in sources]),
        ("app_health", str(health_source), ["-DYG_APP_LOADER_IMAGE=1"]),
        ("flash_wait", str(flash_source), []),
    ):
        executable = output / (name + ".exe")
        for command in (
            base + [str(ROOT / source)] + extra + ["-o", str(executable)],
            [str(executable)],
        ):
            result = subprocess.run(command, capture_output=True, text=True, check=False)
            logs.append(result.stdout + result.stderr)
            (output / "test.log").write_text("\n".join(logs), encoding="utf-8")
            print(logs[-1], end="")
            result.check_returncode()


if __name__ == "__main__":
    main()
