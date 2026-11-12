"""编译真实AT32 Loader端口的寄存器模型，验证发送故障不触发激活。"""

import argparse
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tools"))
from _bsp_test_support import bitrate_cases
from harness.common import zig_path
from project_paths import ROOT
from run_position_servo_tests import function_source


def main():
    """只运行宿主寄存器模型，不加载J-Link或CAN驱动。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc")
    parser.add_argument("--out", type=Path, default=ROOT / "outputs/tests/at32-loader")
    args = parser.parse_args()
    compiler = Path(args.cc) if args.cc else zig_path()
    args.out.mkdir(parents=True, exist_ok=True)
    executable = args.out.resolve() / "at32_loader_can.exe"
    command = [str(compiler)] + (["cc"] if compiler.stem == "zig" else [])
    command += [
        "-std=c11",
        "-O2",
        "-UNDEBUG",
        "-Wall",
        "-Werror",
        "-Wno-unused-function",
        "-ffunction-sections",
        "-DAT32F456CCU7",
        "-DUSE_STDPERIPH_DRIVER",
    ]
    for include in (
        "loader/ports/include",
        "loader/boards/yg_esc_v1",
        "shared/image",
        "shared/targets",
        "shared/protocol",
        "shared/boot",
        "loader/targets/yg_esc_v1_at32f456",
        "firmware/platform/sdk/at32f45x/cmsis/core",
        "firmware/platform/sdk/at32f45x/cmsis/device",
        "firmware/platform/sdk/at32f45x/drivers/inc",
    ):
        command += ["-I", str(ROOT / include)]
    command += [
        str(ROOT / "tests/unit/at32_loader_can_test.c"),
        "-Wl,--gc-sections",
        "-o",
        str(executable),
    ]
    subprocess.run(command, check=True)
    for case in bitrate_cases(executable):
        subprocess.run(case, check=True)
    # 直接注入生产确认代码，覆盖半写、输出联锁及继承看门狗期限。
    boot = (ROOT / "firmware/platform/bsp/mcu/at32f456/boot_loader.c").read_text(encoding="utf-8")
    flags = (ROOT / "firmware/platform/sdk/at32f45x/drivers/inc/at32f45x_flash.h").read_text(
        encoding="utf-8"
    )
    defines = "\n".join(
        line
        for line in flags.splitlines()
        if line.startswith("#define FLASH_") and "_FLAG " in line
    )
    source = args.out.resolve() / "at32_confirm.c"
    source.write_text(
        defines
        + "\n"
        + (ROOT / "tests/unit/at32_loader_confirm_test.c").read_text(encoding="utf-8")
        + "\n"
        + "\n".join(
            function_source(boot, name).replace(
                "FLASH->sts = FLASH_ODF_FLAG | errors;",
                "model_flash_clear(FLASH_ODF_FLAG | errors);",
            )
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
    extra = []
    for directory in ("firmware/platform/bsp/include", "shared/image", "shared/targets"):
        extra += ["-I", str(ROOT / directory)]
    confirm = args.out.resolve() / "at32_confirm.exe"
    subprocess.run(
        command[: command.index(str(ROOT / "tests/unit/at32_loader_can_test.c"))]
        + extra
        + [str(source), "-o", str(confirm)],
        check=True,
    )
    subprocess.run([str(confirm)], check=True)
    port = (ROOT / "loader/ports/at32f456/platform.c").read_text(encoding="utf-8")
    flash_source = args.out.resolve() / "at32_flash.c"
    flash_source.write_text(
        defines
        + "\n"
        + (ROOT / "tests/unit/at32_loader_flash_test.c").read_text(encoding="utf-8")
        + "\n"
        + function_source(port, "LoaderPlatform_NowMs").replace("LoaderPlatform_NowMs", "timer_ms")
        + "\n"
        + function_source(port, "inside")
        + "\n"
        + function_source(port, "flash_ready").replace(
            "FLASH->sts = FLASH_ODF_FLAG | FLASH_PRGMERR_FLAG | FLASH_EPPERR_FLAG;",
            "model_clear(FLASH_ODF_FLAG | FLASH_PRGMERR_FLAG | FLASH_EPPERR_FLAG);",
        ),
        encoding="utf-8",
    )
    flash_executable = args.out.resolve() / "at32_flash.exe"
    subprocess.run(
        command[: command.index(str(ROOT / "tests/unit/at32_loader_can_test.c"))]
        + extra
        + [str(flash_source), "-o", str(flash_executable)],
        check=True,
    )
    subprocess.run([str(flash_executable)], check=True)


if __name__ == "__main__":
    main()
