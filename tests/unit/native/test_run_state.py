"""验证新五态核心及APP运行入口；自检仅占位，不接触硬件。"""

import argparse
import json
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tools"))
from project_paths import NATIVE_INCLUDE_FLAGS, ROOT


def main() -> None:
    """分别编译全部生产单元，覆盖两板配置下的五态和异步业务。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", required=True)
    parser.add_argument("--out", type=Path, default=ROOT / "outputs/five_state_tests")
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    compiler = [args.cc] + (["cc"] if Path(args.cc).stem.lower() == "zig" else [])
    sources = [
        ROOT / "tests/unit/native/five_state_lifecycle_test.c",
        *sorted((ROOT / "firmware/app").glob("motor_lifecycle*.c")),
        ROOT / "firmware/motor/protection/motor_fault.c",
        ROOT / "firmware/app/app_protocol.c",
        ROOT / "shared/protocol/yg_protocol_crc.c",
        ROOT / "shared/protocol/yg_protocol_frame_codec.c",
        ROOT / "shared/protocol/yg_protocol_can_id.c",
        ROOT / "tests/unit/yg_protocol_bsp_stubs.c",
        ROOT / "tests/unit/bsp_preemption_stubs.c",
    ]
    summary = []
    for board in (1, 2):
        exe = args.out / f"five_state_{board}.exe"
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
                f"-DBSP_BOARD={board}",
                *map(str, sources),
                "-o",
                str(exe),
            ]
        )
        logs = []
        for invocation in (command, [str(exe)]):
            result = subprocess.run(invocation, capture_output=True, text=True, check=False)
            logs.append(result.stdout + result.stderr)
            (args.out / f"board_{board}.log").write_text("\n".join(logs), encoding="utf-8")
            print(logs[-1], end="")
            result.check_returncode()
        summary.append({"board": board, "status": "PASS", "hardware_access": False})
    (args.out / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")


if __name__ == "__main__":
    main()
