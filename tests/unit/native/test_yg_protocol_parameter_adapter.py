"""验证参数只读业务服务与 yg_protocol READ 适配器；纯 C 逻辑，不访问硬件。"""

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
    output_dir = (args.out or ROOT / "outputs/tests/yg_protocol_parameter_adapter").resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    executable = output_dir / (
        "yg_protocol_parameter_adapter.exe"
        if sys.platform == "win32"
        else "yg_protocol_parameter_adapter"
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
            str(ROOT / "tests/unit/yg_protocol_parameter_adapter_test.c"),
            str(ROOT / "firmware/services/parameters/parameter_read_service.c"),
            str(ROOT / "firmware/communication/protocol/yg_protocol_parameter_adapter.c"),
            str(ROOT / "firmware/communication/protocol/yg_protocol_parameter.c"),
            "-o",
            str(executable),
        ]
    )
    subprocess.run(command, check=True)
    subprocess.run([str(executable)], check=True)


if __name__ == "__main__":
    main()
