"""验证最小状态元数据与标定、运行、故障的线路映射。"""

import argparse
import subprocess
from pathlib import Path

from run_position_servo_tests import NATIVE_INCLUDE_FLAGS, ROOT, function_source


def main() -> None:
    """编译真实读取函数，确认只报告状态而不伪造测量。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", required=True, type=Path)
    parser.add_argument("--out", type=Path, default=ROOT / "outputs/motor_status_snapshot")
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    source = (ROOT / "firmware/app/motor_lifecycle.c").read_text(encoding="utf-8")
    functions = "\n".join(function_source(source, name) for name in ("MotorLifecycle_ReadStatus",))
    status_adapter = (ROOT / "firmware/app/app_protocol.c").read_text(encoding="utf-8")
    functions += "\n" + function_source(status_adapter, "protocol_state")
    fixture = (ROOT / "tests/unit/native/motor_status_snapshot_test.c").read_text(encoding="utf-8")
    generated = args.out / "snapshot.c"
    generated.write_text(fixture.replace("/* REAL_FUNCTIONS */", functions), encoding="utf-8")
    exe = args.out / "snapshot.exe"
    compiler = [str(args.cc)] + (["cc"] if args.cc.stem.lower() == "zig" else [])
    command = (
        compiler
        + NATIVE_INCLUDE_FLAGS
        + [
            "-std=c99",
            "-O1",
            "-UNDEBUG",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-DBSP_BOARD=1",
            str(generated),
            "-o",
            str(exe),
        ]
    )
    log = []
    for invocation in (command, [str(exe)]):
        result = subprocess.run(invocation, capture_output=True, text=True, check=False)
        log.append(result.stdout + result.stderr)
        print(log[-1], end="")
        (args.out / "test.log").write_text("".join(log), encoding="utf-8")
        result.check_returncode()


if __name__ == "__main__":
    main()
