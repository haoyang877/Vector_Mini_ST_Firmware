"""电机 STOP/DISABLE 通信适配器离线测试入口；编译并运行原生夹具，不访问硬件。"""

import argparse
import os
import re
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tools"))

from harness.common import zig_path
from project_paths import NATIVE_INCLUDE_FLAGS, ROOT

LIFECYCLE_INCLUDE_FLAGS = ["-I", str(ROOT / "firmware/app/interfaces")]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", default=None)
    parser.add_argument("--out", type=Path, default=None)
    args = parser.parse_args()

    compiler = Path(args.cc) if args.cc else zig_path()
    if compiler is None:
        parser.error("未找到 Zig 或 C 编译器，请通过 --cc 指定")
    output_dir = (args.out or ROOT / "outputs/tests/yg_protocol_motor_adapter").resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    executable = output_dir / (
        "yg_protocol_motor_adapter.exe" if os.name == "nt" else "yg_protocol_motor_adapter"
    )
    # 使用实际APP能力配置验证编码结果，防止控制入口已接通但发现页仍遗漏模式。
    startup = (ROOT / "firmware/app/composition/app_startup.c").read_text(encoding="utf-8")
    capabilities = re.search(
        r"yg_protocol_readonly_caps_page0_t capabilities = \{.*?\};", startup, re.DOTALL
    )
    if capabilities is None:
        raise RuntimeError("未找到APP能力配置")
    caps_source = output_dir / "application_caps.c"
    caps_source.write_text(
        '#include "yg_protocol_readonly.h"\n#include <string.h>\n'
        "bool test_application_capabilities(void) {\n"
        + capabilities.group(0)
        + "\nconst uint8_t expected[16] = {0x00,0x1d,0,0,0x04,0x06,0x40,0,"
        "0x2e,0,0,0,0,0,0,0};\nuint8_t payload[16]; size_t length = 0;\n"
        "return yg_protocol_readonly_encode_caps_page0(&capabilities, payload, sizeof(payload),"
        " &length) == YG_PROTOCOL_OK && length == sizeof(expected) &&"
        " memcmp(payload, expected, sizeof(expected)) == 0;\n}\n",
        encoding="utf-8",
    )
    command = [str(compiler)]
    if compiler.stem.lower() == "zig":
        command.append("cc")
    command.extend(
        NATIVE_INCLUDE_FLAGS
        + LIFECYCLE_INCLUDE_FLAGS
        + [
            "-std=c99",
            "-UNDEBUG",
            "-Wall",
            "-Wextra",
            "-Werror",
            str(ROOT / "tests/unit/yg_protocol_motor_adapter_test.c"),
            str(caps_source),
            str(ROOT / "firmware/app/interfaces/motor_stop_request.c"),
            str(ROOT / "firmware/app/motor_state_machine/motor_state_rules.c"),
            str(ROOT / "firmware/communication/commands/yg_protocol_motor_adapter.c"),
            str(ROOT / "firmware/communication/commands/yg_protocol_motor.c"),
            str(ROOT / "shared/protocol/yg_protocol_crc.c"),
            str(ROOT / "firmware/communication/commands/yg_protocol_readonly.c"),
            "-o",
            str(executable),
        ]
    )
    subprocess.run(command, check=True)
    subprocess.run([str(executable)], check=True)


if __name__ == "__main__":
    main()
