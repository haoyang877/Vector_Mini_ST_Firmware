"""对比生产参数与 CAN 编码的整数实现和冻结双精度参考；不访问设备。"""

import argparse
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tools"))
from harness.common import zig_path
from project_paths import NATIVE_INCLUDE_FLAGS, ROOT


def main() -> None:
    """构建生产源码夹具并检查舍入、边界、非有限值和失败输出不变。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", default=None)
    parser.add_argument("--out", type=Path, default=ROOT / "outputs/tests/numeric_encoding")
    args = parser.parse_args()
    compiler = Path(args.cc) if args.cc else zig_path()
    if compiler is None:
        parser.error("未找到原生编译器")
    output = args.out.resolve()
    output.mkdir(parents=True, exist_ok=True)
    executable = output / "numeric_encoding.exe"
    command = [str(compiler)] + (["cc"] if compiler.stem.lower() == "zig" else [])
    command += NATIVE_INCLUDE_FLAGS + [
        "-std=c99",
        "-O2",
        "-UNDEBUG",
        "-Wall",
        "-Wextra",
        "-Werror",
        str(ROOT / "tests/unit/numeric_encoding_test.c"),
        str(ROOT / "firmware/services/parameters/motor_axis_profile.c"),
        str(ROOT / "firmware/common/crc32.c"),
        "-o",
        str(executable),
    ]
    logs = []
    for cmd in (command, [str(executable)]):
        result = subprocess.run(cmd, capture_output=True, text=True)
        logs.append(result.stdout + result.stderr)
        (output / "test.log").write_text("\n".join(logs), encoding="utf-8")
        print(logs[-1], end="")
        result.check_returncode()


if __name__ == "__main__":
    main()
