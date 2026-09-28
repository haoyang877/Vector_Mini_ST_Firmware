"""后期执行Loader纯内存事务用例；只链接通用C内核，不访问真实硬件。"""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tools"))
from harness.common import zig_path
from project_paths import ROOT


def main() -> None:
    """编译并执行纯内存模型，证据留在outputs。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", default=None)
    parser.add_argument("--out", type=Path, default=ROOT / "outputs/tests/loader_upgrade")
    args = parser.parse_args()
    compiler = Path(args.cc) if args.cc else zig_path()
    if compiler is None:
        parser.error("未找到编译器，请通过--cc指定Zig或C编译器")
    output = args.out.resolve()
    output.mkdir(parents=True, exist_ok=True)
    executable = output / ("loader_upgrade.exe" if sys.platform == "win32" else "loader_upgrade")
    command = [str(compiler)]
    if compiler.stem.lower() == "zig":
        command.append("cc")
    for directory in ("shared/image", "shared/boot", "loader/core"):
        command += ["-I", str(ROOT / directory)]
    command += ["-std=c99", "-UNDEBUG", "-Wall", "-Wextra", "-Werror"]
    command += [
        str(ROOT / source)
        for source in (
            "tests/unit/loader_upgrade_test.c",
            "loader/core/upgrade.c",
            "shared/boot/image_mailbox.c",
            "shared/image/image_transfer.c",
        )
    ]
    command += ["-o", str(executable)]
    subprocess.run(command, check=True)
    subprocess.run([str(executable)], check=True)


if __name__ == "__main__":
    main()
