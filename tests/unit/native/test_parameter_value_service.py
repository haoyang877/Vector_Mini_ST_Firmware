"""参数夹具使用真实活动值服务，替代退役的离线只读适配器；不访问硬件。"""

import argparse
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tools"))

from harness.common import zig_path
from project_paths import NATIVE_INCLUDE_FLAGS, ROOT


def main() -> None:
    """编译并运行参数标量契约夹具；本函数仅由显式验证入口调用。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", default=None)
    parser.add_argument("--out", type=Path, default=None)
    args = parser.parse_args()

    compiler = Path(args.cc) if args.cc else zig_path()
    if compiler is None:
        parser.error("未找到 Zig 或 C 编译器，请通过 --cc 指定")
    output_dir = (args.out or ROOT / "outputs/tests/parameter_value_service").resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    executable = output_dir / (
        "parameter_value_service.exe" if sys.platform == "win32" else "parameter_value_service"
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
            str(ROOT / "tests/unit/parameter_value_service_test.c"),
            str(ROOT / "firmware/services/parameters/parameter_value_service.c"),
            str(ROOT / "firmware/services/parameters/motor_axis_profile.c"),
            str(ROOT / "shared/checksum/crc32.c"),
            str(ROOT / "firmware/motor/protection/motor_thermal.c"),
            str(ROOT / "firmware/motor/protection/motor_thermal_config.c"),
            str(ROOT / "firmware/motor/foc/speed_feedforward.c"),
            "-lm",
            "-o",
            str(executable),
        ]
    )
    subprocess.run(command, check=True)
    subprocess.run([str(executable)], check=True)


if __name__ == "__main__":
    main()
