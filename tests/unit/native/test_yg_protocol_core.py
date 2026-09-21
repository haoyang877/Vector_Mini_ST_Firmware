"""验证 yg_protocol 核心接口之间的纯 C 调用关系；不访问硬件。"""

import argparse
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
    output_dir = (args.out or ROOT / "outputs/tests/yg_protocol_core").resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    executable = output_dir / (
        "yg_protocol_core.exe" if __import__("os").name == "nt" else "yg_protocol_core"
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
            str(ROOT / "tests/unit/yg_protocol_core_test.c"),
            str(ROOT / "tests/unit/yg_protocol_canfd_test.c"),
            str(ROOT / "tests/unit/yg_protocol_contract_test.c"),
            str(ROOT / "firmware/communication/can/yg_protocol_canfd.c"),
            str(ROOT / "firmware/communication/can/yg_protocol_endpoint.c"),
            str(ROOT / "firmware/communication/protocol/yg_protocol_crc.c"),
            str(ROOT / "firmware/communication/protocol/yg_protocol_frame_codec.c"),
            str(ROOT / "firmware/communication/protocol/yg_protocol_can_id.c"),
            str(ROOT / "firmware/communication/protocol/yg_protocol_message_registry.c"),
            str(ROOT / "firmware/communication/protocol/yg_protocol_fragment.c"),
            str(ROOT / "firmware/communication/protocol/yg_protocol_router.c"),
            str(ROOT / "firmware/communication/protocol/yg_protocol_readonly.c"),
            str(ROOT / "firmware/communication/protocol/yg_protocol_readonly_payload.c"),
            str(ROOT / "firmware/communication/protocol/yg_protocol_motor_status.c"),
            str(ROOT / "firmware/communication/protocol/yg_protocol_motor.c"),
            str(ROOT / "firmware/communication/protocol/yg_protocol_parameter.c"),
            str(ROOT / "firmware/communication/protocol/yg_protocol_job.c"),
            str(ROOT / "firmware/communication/protocol/yg_protocol_update.c"),
            str(ROOT / "firmware/communication/protocol/yg_protocol_transfer.c"),
            "-o",
            str(executable),
        ]
    )
    subprocess.run(command, check=True)
    subprocess.run([str(executable)], check=True)


if __name__ == "__main__":
    main()
