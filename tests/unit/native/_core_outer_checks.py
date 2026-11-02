"""三项核心外环夹具共用真实生产编译与日志机制；不接触硬件。"""

import argparse
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tools"))
from project_paths import ROOT, native_include_flags, resolve_source_path
from run_position_servo_tests import function_source


def main(case: str) -> None:
    """仅编译核心组装、原速度算法和边界桩，按指定合同运行检查。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--reference-root", type=Path, default=ROOT)
    args = parser.parse_args()
    output = args.out.resolve()
    output.mkdir(parents=True, exist_ok=True)
    compiler = [args.cc] + (["cc"] if Path(args.cc).stem.lower() == "zig" else [])
    executable = output / f"core_outer_{case}.exe"
    reference = args.reference_root.resolve()
    flags = ["-I", str(ROOT / "firmware/app/runtime"), *native_include_flags(reference)]
    fixture = ROOT / "tests/unit/native/core_outer_test.c"
    position_sources = []
    fixture_text = fixture.read_text(encoding="utf-8")
    if case == "runtime":
        # 运行检查链接真实位置核心；配置/拒绝检查使用明确的边界桩。
        text = fixture_text
        for name in ("PositionCascade_Reset", "PositionCascade_UpdateControlLimited"):
            text = text.replace(function_source(text, name), "")
        text = text.replace("unsigned resets = s_resets;", "")
        text = text.replace(
            "static unsigned s_updates, s_resets, s_current_calls;",
            "static unsigned s_current_calls;",
        )
        text = text.replace(
            "CHECK(s_resets > resets && MotorOuterLoop_IsReady());",
            "CHECK(MotorOuterLoop_IsReady());",
        )
        fixture = output / "runtime_fixture.c"
        fixture.write_text(text, encoding="utf-8")
        position_sources = [
            str(
                resolve_source_path(
                    reference,
                    "firmware/motor/control/position_control.c",
                    "firmware/motor/position/position_cascade.c",
                )
            )
        ]
    else:
        fixture = output / f"fixture_{case}.c"
        fixture.write_text(fixture_text, encoding="utf-8")
    command = (
        compiler
        + flags
        + [
            "-std=c99",
            "-O1",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-DBSP_BOARD=1",
            "-DCURRENT_SENSE_SHUNT_MILLIOHM=6U",
            str(fixture),
            str(ROOT / "firmware/motor/control/motor_control.c"),
            str(ROOT / "firmware/motor/control/speed_control.c"),
            str(ROOT / "firmware/motor/control/speed_feedforward.c"),
            str(ROOT / "firmware/motor/control/pi_controller.c"),
            str(ROOT / "firmware/motor/math/control_math.c"),
            *position_sources,
            "-lm",
            "-o",
            str(executable),
        ]
    )
    logs = []
    for invocation in (command, [str(executable), case]):
        result = subprocess.run(invocation, cwd=ROOT, capture_output=True, text=True)
        logs.append(result.stdout + result.stderr)
        (output / f"{case}.log").write_text("\n".join(logs), encoding="utf-8")
        print(logs[-1], end="")
        result.check_returncode()
