"""验证固定维护池能够容纳现有最大借用并在释放后恢复连续容量；不访问硬件。"""

import argparse
import os
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tools"))

from harness.common import zig_path
from project_paths import NATIVE_INCLUDE_FLAGS, ROOT


def main() -> None:
    """编译和执行纯软件容量检查，所有产物置于outputs。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", default=None)
    parser.add_argument("--out", type=Path, default=None)
    args = parser.parse_args()
    compiler = Path(args.cc) if args.cc else zig_path()
    if compiler is None:
        parser.error("未找到Zig或C编译器，请通过--cc指定")
    output_dir = (args.out or ROOT / "outputs/tests/heap_capacity").resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    executable = output_dir / ("heap_capacity.exe" if os.name == "nt" else "heap_capacity")
    command = [str(compiler)]
    if compiler.stem.lower() == "zig":
        command.append("cc")
    command.extend(
        NATIVE_INCLUDE_FLAGS
        + [
            "-std=c99",
            "-Wall",
            "-Wextra",
            "-Werror",
            str(ROOT / "tests/unit/heap_capacity_test.c"),
            str(ROOT / "firmware/common/heap.c"),
            "-o",
            str(executable),
        ]
    )
    subprocess.run(command, check=True)
    subprocess.run([str(executable)], check=True)


if __name__ == "__main__":
    main()
