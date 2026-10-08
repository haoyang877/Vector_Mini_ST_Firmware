"""离线编译启动身份生产核心与真实写域/状态源；只使用内存Flash模型。"""

import argparse
import json
import struct
import subprocess
import sys
import zlib
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tools"))
from harness.common import zig_path
from project_paths import NATIVE_INCLUDE_FLAGS, ROOT
from run_position_servo_tests import function_source


def main() -> None:
    """检查版本化向量，并在本地编译执行实际provider、写域门禁和状态源。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc")
    parser.add_argument("--out", type=Path, default=ROOT / "outputs/tests/boot_epoch")
    args = parser.parse_args()
    compiler = Path(args.cc) if args.cc else zig_path()
    if compiler is None:
        parser.error("未找到原生C编译器")
    output = args.out.resolve()
    output.mkdir(parents=True, exist_ok=True)
    vectors = json.loads(
        (ROOT / "docs/protocols/boot_epoch_record_v1_vectors.json").read_text(encoding="utf-8")
    )
    for vector in vectors["vectors"]:
        value = int(vector["epoch"], 0)
        body = struct.pack("<IIQQ", 0x31455042, 1, value, value ^ 0xFFFFFFFFFFFFFFFF)
        if zlib.crc32(body) != int(vector["crc32"], 0):
            raise ValueError(f"启动记录CRC向量错误：{vector}")
    flash = (ROOT / "firmware/platform/stm32g4/bsp/flash.c").read_text(encoding="utf-8")
    guard = (ROOT / "tests/unit/native/boot_epoch_flash_guard_test.c").read_text(encoding="utf-8")
    production = "\n".join(
        function_source(flash, name)
        for name in (
            "epoch_range",
            "flash_write_param",
            "BootEpochFlash_Program",
            "BootEpochFlash_Erase",
        )
    )
    guard_path = output / "flash_guards.c"
    guard_path.write_text(
        guard.replace(
            "/* BOOT_EPOCH_FLASH_PRODUCTION：脚本插入真实参数和epoch写域生产体。 */", production
        ),
        encoding="utf-8",
    )
    adapter = (ROOT / "firmware/app/composition/yg_protocol_status_adapter.c").read_text(
        encoding="utf-8"
    )
    status = (ROOT / "tests/unit/native/boot_epoch_status_test.c").read_text(encoding="utf-8")
    status_path = output / "status_source.c"
    status_path.write_text(
        status.replace(
            "/* BOOT_EPOCH_STATUS_PRODUCTION：脚本插入真实状态源刷新体。 */",
            function_source(adapter, "refresh_boot_source"),
        ),
        encoding="utf-8",
    )
    checks = (
        ("provider", [ROOT / "tests/unit/boot_epoch_test.c", ROOT / "shared/checksum/crc32.c"], []),
        ("flash_guards", [guard_path], []),
        (
            "status_normal",
            [status_path, ROOT / "firmware/communication/protocol/yg_protocol_readonly_payload.c"],
            [],
        ),
        (
            "status_loaderapp",
            [status_path, ROOT / "firmware/communication/protocol/yg_protocol_readonly_payload.c"],
            ["-DYG_APP_LOADER_IMAGE=1"],
        ),
    )
    prefix = [str(compiler)] + (["cc"] if compiler.stem.lower() == "zig" else [])
    for name, sources, definitions in checks:
        executable = output / f"{name}.exe"
        command = (
            prefix
            + NATIVE_INCLUDE_FLAGS
            + [
                "-I",
                str(ROOT / "firmware/platform/stm32g4/ports/board"),
                "-std=c99",
                "-O1",
                "-Wall",
                "-Wextra",
                "-Werror",
                "-UNDEBUG",
            ]
            + definitions
            + [str(source) for source in sources]
            + ["-o", str(executable)]
        )
        logs = []
        for invocation in (command, [str(executable)]):
            result = subprocess.run(invocation, capture_output=True, text=True)
            logs.append(result.stdout + result.stderr)
            (output / f"{name}.log").write_text("\n".join(logs), encoding="utf-8")
            print(logs[-1], end="")
            result.check_returncode()


if __name__ == "__main__":
    main()
