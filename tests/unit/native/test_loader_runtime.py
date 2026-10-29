"""验证Loader启动救援和APP喂狗组合层，全部硬件由内存桩替换。"""

from __future__ import annotations

import argparse
import json
import struct
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tools"))
from harness.common import zig_path
from project_paths import NATIVE_INCLUDE_FLAGS, ROOT
from run_position_servo_tests import function_source

sys.path.insert(0, str(ROOT / "loader/tools"))
from package_image import build_package


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
            (ROOT / "tests/fixtures/loader_image_v3.json").read_text(encoding="utf-8")
        )
        target = json.loads(
            (ROOT / "shared/targets/vector_mini_st_g431.json").read_text(encoding="utf-8")
        )
        constants = target["constants"]
        source = bytearray.fromhex(vector["source_hex"])
        offset = int(constants["TARGET_DESCRIPTOR_OFFSET"].rstrip("U"), 0)
        app_base = int(constants["TARGET_APP_BASE"].rstrip("U"), 0)
        struct.pack_into("<I", source, 4, app_base + offset + 96 + 1)
        struct.pack_into(">I", source, offset + 12, int(constants["TARGET_LAYOUT"].rstrip("U"), 0))
        struct.pack_into(
            ">I", source, offset + 28, int(constants["TARGET_APP_VERSION"].rstrip("U"), 0)
        )
        struct.pack_into(
            ">I", source, offset + 36, int(constants["TARGET_PARAM_ABI"].rstrip("U"), 0)
        )
        schema = int(constants["TARGET_SCHEMA"].rstrip("U"), 0)
        struct.pack_into(">HH", source, offset + 40, schema, schema)
        image, _, _ = build_package(source, 7, target)
    header = "static const uint8_t LoaderV2Image[] = {" + ",".join(map(str, image)) + "};\n"
    (output / "loader_image_v2_vector.h").write_text(header, encoding="utf-8")
    board = (ROOT / "firmware/app/composition/app_startup.c").read_text(encoding="utf-8")
    health_fixture = (ROOT / "tests/unit/loader_app_health_test.c").read_text(encoding="utf-8")
    interval = next(
        line for line in board.splitlines() if line.startswith("#define BOOT_HEALTH_INTERVAL_MS")
    )
    health_source = output / "app_health.c"
    health_source.write_text(
        interval
        + "\n"
        + health_fixture
        + "\n"
        + function_source(board, "service_boot")
        + "\n"
        + function_source(board, "confirm_image")
        + "\n"
        + function_source(board, "App_Process")
        + "\n"
        + function_source(
            (ROOT / "firmware/app/composition/app_tasks.c").read_text(encoding="utf-8"),
            "mode_color",
        )
        + "\n"
        + function_source(
            (ROOT / "firmware/app/composition/app_tasks.c").read_text(encoding="utf-8"),
            "App_FastTick",
        )
        + "\n#define RGB_DIVIDER (SUPERVISOR_FREQ / 20U)\nstatic uint16_t s_rgb_cnt;\n"
        + function_source(
            (ROOT / "firmware/app/composition/app_tasks.c").read_text(encoding="utf-8"),
            "App_SupervisorTick",
        ),
        encoding="utf-8",
    )
    platform = (ROOT / "loader/ports/stm32g4/platform.c").read_text(encoding="utf-8")
    device = (
        ROOT
        / "firmware/platform/sdk/stm32g4/Drivers/CMSIS/Device/ST/STM32G4xx/Include/stm32g431xx.h"
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
    boot = (ROOT / "firmware/platform/bsp/mcu/stm32g4/boot_hw_stm32g4.c").read_text(
        encoding="utf-8"
    )
    confirm_source = output / "app_confirm.c"
    register_defines = "\n".join(
        line
        for line in device.splitlines()
        if line.startswith(("#define FLASH_", "#define TIM_CCER_"))
    )
    confirm_source.write_text(
        register_defines
        + "\n"
        + (ROOT / "tests/unit/loader_app_confirm_test.c").read_text(encoding="utf-8")
        + "\n"
        + "\n".join(
            function_source(boot, name)
            for name in (
                "read_be32",
                "marker_valid",
                "outputs_off",
                "BootHw_Watchdog",
                "BootHw_IsAvailable",
                "BootHw_IsConfirmed",
                "BootHw_ImageCrc32",
                "BootHw_Confirm",
            )
        ),
        encoding="utf-8",
    )
    parameter_source = output / "parameters.c"
    parameter_source.write_text(
        (ROOT / "tests/unit/loader_parameter_test.c").read_text(encoding="utf-8")
        + "\n"
        + (ROOT / "loader/product/parameters.c").read_text(encoding="utf-8"),
        encoding="utf-8",
    )
    parameter_io_source = output / "parameter_io.c"
    parameter_io_source.write_text(
        (ROOT / "tests/unit/loader_parameter_io_test.c").read_text(encoding="utf-8")
        + "\n"
        + function_source(platform, "LoaderPlatform_ReadParameters"),
        encoding="utf-8",
    )
    flash_domain_source = output / "flash_domain.c"
    flash_domain_source.write_text(
        (ROOT / "tests/unit/loader_flash_domain_test.c").read_text(encoding="utf-8")
        + "\n"
        + "\n".join(
            function_source(platform, name)
            for name in (
                "inside",
                "read_flash",
                "erase_flash",
                "program_flash",
                "bind_io",
                "LoaderPlatform_ImageIo",
            )
        ),
        encoding="utf-8",
    )
    board_source = output / "board_safety.c"
    board_code = (ROOT / "loader/boards/vector_mini_st/board.c").read_text(encoding="utf-8")
    board_source.write_text(
        (ROOT / "tests/unit/loader_board_test.c").read_text(encoding="utf-8")
        + "\n"
        + "\n".join(
            function_source(board_code, name)
            for name in ("pins", "LoaderBoard_Safe", "LoaderBoard_OutputsOff")
        ),
        encoding="utf-8",
    )
    base = [str(compiler)] + (["cc"] if compiler.stem.lower() == "zig" else [])
    base += NATIVE_INCLUDE_FLAGS + [
        "-I",
        str(ROOT / "firmware/app/interfaces"),
        "-I",
        str(ROOT / "loader/ports/include"),
        "-I",
        str(ROOT / "loader/product"),
        "-I",
        str(ROOT / "shared/targets/vector_mini_st_g431"),
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
        "loader/product/parameters.c",
        "shared/image/image_transfer.c",
        "shared/boot/image_mailbox.c",
        "shared/boot/device_identity.c",
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
        (
            "app_health",
            str(health_source),
            ["-DYG_APP_LOADER_IMAGE=1"],
        ),
        ("flash_wait", str(flash_source), []),
        ("app_confirm", str(confirm_source), []),
        ("parameters", str(parameter_source), []),
        ("parameter_io", str(parameter_io_source), []),
        ("flash_domain", str(flash_domain_source), []),
        ("board_safety", str(board_source), []),
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
