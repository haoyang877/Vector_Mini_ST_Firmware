"""电机线路编码及能力黄金向量离线测试入口；编译并运行原生夹具，不访问硬件。"""

import argparse
import json
import os
import re
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tools"))

from harness.common import zig_path
from project_paths import NATIVE_INCLUDE_FLAGS, ROOT


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", default=None)
    parser.add_argument("--out", type=Path, default=None)
    args = parser.parse_args()

    compiler = Path(args.cc) if args.cc else zig_path()
    if compiler is None:
        parser.error("未找到 Zig 或 C 编译器，请通过 --cc 指定")
    output_dir = (args.out or ROOT / "outputs/tests/yg_protocol_motor_wire").resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    executable = output_dir / (
        "yg_protocol_motor_wire.exe" if os.name == "nt" else "yg_protocol_motor_wire"
    )
    # 使用实际APP能力配置验证编码结果，防止控制入口已接通但发现页仍遗漏模式。
    startup = (ROOT / "firmware/app/app.c").read_text(encoding="utf-8")
    capabilities = re.search(r"ProtocolCaps capabilities = \{.*?\};", startup, re.DOTALL)
    if capabilities is None:
        raise RuntimeError("未找到APP能力配置")
    caps_source = output_dir / "application_caps.c"
    caps_source.write_text(
        '#include "yg_protocol.h"\n#include <string.h>\n'
        "bool test_application_capabilities(void) {\n"
        + capabilities.group(0)
        + "\nconst uint8_t expected[16] = {0x00,0x1d,0,0,0x4e,0x06,0x40,0,"
        "0x2e,0,0,0,0,0,0,0};\nProtocolResponse response = {0};\n"
        "return encode_device_caps(&capabilities, &response) == YG_PROTOCOL_OK &&"
        " response.length == sizeof(expected) &&"
        " memcmp(response.data, expected, sizeof(expected)) == 0;\n}\n",
        encoding="utf-8",
    )
    fixture = output_dir / "motor_wire_fixture.c"
    fixture.write_text(
        '#include "'
        + (ROOT / "tests/unit/yg_protocol_motor_wire_test.c").as_posix()
        + '"\n'
        + caps_source.read_text(encoding="utf-8"),
        encoding="utf-8",
    )
    command = [str(compiler)]
    if compiler.stem.lower() == "zig":
        command.append("cc")
    command.extend(
        NATIVE_INCLUDE_FLAGS
        + [
            "-std=c99",
            "-UNDEBUG",
            "-Wall",
            "-Wextra",
            "-Werror",
            str(fixture),
            str(ROOT / "shared/protocol/yg_protocol_crc.c"),
            str(ROOT / "shared/protocol/yg_protocol_frame_codec.c"),
            str(ROOT / "shared/protocol/yg_protocol_can_id.c"),
            str(ROOT / "tests/unit/yg_protocol_bsp_stubs.c"),
            str(ROOT / "tests/unit/bsp_preemption_stubs.c"),
            "-o",
            str(executable),
        ]
    )
    subprocess.run(command, check=True)
    result = subprocess.run([str(executable)], check=True, capture_output=True, text=True)
    print(result.stdout, end="")
    vectors = json.loads(
        (ROOT / "tests/fixtures/protocol/yg_protocol_motor_state_v1_7_vectors.json").read_text(
            encoding="utf-8"
        )
    )
    vector = next(item for item in vectors["vectors"] if item["name"] == "minimal_state")
    if f"VECTOR minimal_state {vector['payload_hex']}" not in result.stdout.splitlines():
        raise RuntimeError("最小状态反馈与v1.7黄金向量不符")


if __name__ == "__main__":
    main()
