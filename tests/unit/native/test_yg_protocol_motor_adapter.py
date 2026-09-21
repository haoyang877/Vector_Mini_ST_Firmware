"""电机 STOP/DISABLE 通信适配器离线测试入口；编译并运行原生夹具，不访问硬件。"""

import argparse
import os
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tools"))

from harness.common import zig_path
from project_paths import NATIVE_INCLUDE_FLAGS, ROOT

LIFECYCLE_INCLUDE_FLAGS = ["-I", str(ROOT / "firmware/services/lifecycle")]


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
            str(ROOT / "firmware/services/lifecycle/motor_stop_service.c"),
            str(ROOT / "firmware/services/lifecycle/app_lifecycle.c"),
            str(ROOT / "firmware/communication/protocol/yg_protocol_motor_adapter.c"),
            str(ROOT / "firmware/communication/protocol/yg_protocol_motor.c"),
            "-o",
            str(executable),
        ]
    )
    subprocess.run(command, check=True)
    subprocess.run([str(executable)], check=True)


if __name__ == "__main__":
    main()
