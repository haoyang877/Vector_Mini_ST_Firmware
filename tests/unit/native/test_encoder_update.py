"""以两板配置编译真实编码器反馈，验证快慢估计配置绑定与对象生命周期。"""

import argparse
import subprocess
from pathlib import Path

from run_position_servo_tests import NATIVE_INCLUDE_FLAGS, ROOT


def main() -> None:
    """运行纯离线编码器回归并保存逐拍反馈，无硬件访问。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", required=True, type=Path)
    parser.add_argument("--out", required=True, type=Path)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    compiler = [str(args.cc)] + (["cc"] if args.cc.stem.lower() == "zig" else [])
    for board in (1, 2):
        executable = args.out.resolve() / f"encoder_update_{board}.exe"
        command = (
            compiler
            + NATIVE_INCLUDE_FLAGS
            + [
                "-std=c99",
                "-O2",
                "-UNDEBUG",
                "-Wall",
                "-Wextra",
                "-Werror",
                f"-DBSP_BOARD={board}",
                str(ROOT / "tests/unit/native/encoder_update_test.c"),
                str(ROOT / "tests/unit/bsp_preemption_stubs.c"),
                str(ROOT / "firmware/motor/feedback/encoder_feedback.c"),
                "-lm",
                "-o",
                str(executable),
            ]
        )
        subprocess.run(command, check=True)
        result = subprocess.run([str(executable)], capture_output=True, text=True, check=True)
        (args.out / f"encoder_trace_{board}.txt").write_text(result.stdout, encoding="utf-8")
        print(f"board {board}: {result.stdout.splitlines()[-1]}")


if __name__ == "__main__":
    main()
