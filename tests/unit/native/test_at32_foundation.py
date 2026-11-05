"""编译真实AT32F456端口并通过SDK替身检查启动、关相和故障；不连接硬件。"""

import sys as _sys
from pathlib import Path as _Path

_sys.path.insert(0, str(_Path(__file__).resolve().parents[3] / "tools"))
import argparse
import json
import subprocess
from pathlib import Path

from project_paths import NATIVE_INCLUDE_FLAGS, ROOT


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    base = ROOT / "firmware/platform/bsp/mcu/at32f456"
    native = ROOT / "tests/unit/native"
    for profile in (1, 2):
        feedback_executable = args.out.resolve() / f"app_bus_current_{profile}.exe"
        feedback_command = [args.cc] + (["cc"] if Path(args.cc).stem == "zig" else [])
        subprocess.run(
            feedback_command
            + NATIVE_INCLUDE_FLAGS
            + [str(ROOT / "tests/unit/bsp_preemption_stubs.c")]
            + [
                "-I",
                str(base),
                "-std=c11",
                "-O2",
                "-UNDEBUG",
                "-Wall",
                "-Wextra",
                "-Werror",
                f"-DBSP_BOARD={profile}",
                str(native / "app_bus_current_test.c"),
                str(ROOT / "firmware/app/composition/app_protocol.c"),
                str(ROOT / "firmware/app/interfaces/motor_stop_request.c"),
                str(ROOT / "shared/protocol/yg_protocol_crc.c"),
                str(ROOT / "shared/protocol/yg_protocol_frame_codec.c"),
                str(ROOT / "shared/protocol/yg_protocol_can_id.c"),
                str(ROOT / "tests/unit/yg_protocol_bsp_stubs.c"),
                "-lm",
                "-o",
                str(feedback_executable),
            ],
            check=True,
        )
        result = subprocess.run(
            [str(feedback_executable)], check=True, capture_output=True, text=True
        )
        print(result.stdout, end="")
        vectors = json.loads(
            (ROOT / "tests/fixtures/protocol/yg_protocol_motor_live_v1_9_vectors.json").read_text(
                encoding="utf-8"
            )
        )
        expected = next(
            item["payload_hex"]
            for item in vectors["vectors"]
            if item["name"] == "live_current_feedback"
        )
        if "VECTOR live_current_feedback " + expected not in result.stdout.splitlines():
            raise RuntimeError("实时反馈黄金向量不匹配")
    executable = args.out.resolve() / "at32f456_foundation.exe"
    command = [args.cc] + (["cc"] if Path(args.cc).stem == "zig" else [])
    command += [
        "-DBSP_BOARD=2",
        "-std=c11",
        "-ffunction-sections",
        "-fdata-sections",
        "-O2",
        "-UNDEBUG",
        "-Wall",
        "-Wextra",
        "-Werror",
    ]
    for include in (
        native / "at32f456_fake",
        base,
        ROOT / "firmware/platform/bsp/include",
        ROOT / "firmware/config",
        ROOT / "firmware/config",
        ROOT / "firmware/platform/bsp",
        ROOT / "firmware/platform/bsp",
    ):
        command += ["-I", str(include)]
    sources = [
        base / "pwm.c",
        base / "system.c",
        native / "at32f456_foundation_test.c",
    ]
    subprocess.run(command + [str(p) for p in sources] + ["-o", str(executable)], check=True)
    subprocess.run([str(executable)], check=True)
    for voltage, duration, current, frequency in (
        ("0.20", 1000, "0.60", "2.0"),
        ("0.30", 1000, "0.60", "2.0"),
        ("1.0", 10000, "0.60", "2.0"),
        ("1.0", 10000, "2.0", "2.0"),
        ("1.0", 10000, "3.0", "2.0"),
        ("1.0", 10000, "3.0", "20.0"),
        ("2.0", 10000, "3.0", "20.0"),
    ):
        openloop_executable = (
            args.out.resolve() / f"at32f456_openloop_{voltage}_{duration}_{current}_{frequency}.exe"
        )
        subprocess.run(
            command[: command.index("-I")]
            + [
                f"-DAT32_OL_VOLTAGE_V={voltage}f",
                f"-DAT32_OL_DURATION_MS={duration}U",
                f"-DAT32_OL_CURRENT_LIMIT_A={current}f",
                f"-DAT32_OL_ELECTRICAL_HZ={frequency}f",
                str(ROOT / "tests/bench/at32f456/openloop_model.c"),
                str(ROOT / "firmware/motor/math/control_math.c"),
                str(native / "at32f456_openloop_test.c"),
                "-lm",
                "-o",
                str(openloop_executable),
            ],
            check=True,
        )
        subprocess.run([str(openloop_executable)], check=True)
    closedloop_executable = args.out.resolve() / "at32f456_closedloop.exe"
    motor = ROOT / "firmware/motor"
    subprocess.run(
        command[: command.index("-I")]
        + NATIVE_INCLUDE_FLAGS
        + [str(ROOT / "tests/unit/bsp_preemption_stubs.c")]
        + ["-I", str(ROOT / "tests/bench/at32f456")]
        + [
            "-DAT32_OL_CURRENT_LIMIT_A=3.0f",
            "-DAT32_OL_DURATION_MS=10000U",
            "-DAT32_CLOSEDLOOP_TEST=1",
        ]
        + [
            str(path)
            for path in (
                ROOT / "tests/bench/at32f456/closedloop_bench.c",
                ROOT / "tests/bench/at32f456/openloop_model.c",
                motor / "math/control_math.c",
                motor / "control/foc.c",
                motor / "control/foc_output.c",
                motor / "control/pi_controller.c",
                motor / "control/speed_control.c",
                motor / "control/speed_feedforward.c",
                motor / "feedback/encoder_feedback.c",
                motor / "protection/motor_thermal.c",
                native / "at32f456_closedloop_test.c",
            )
        ]
        + ["-lm", "-o", str(closedloop_executable)],
        check=True,
    )
    subprocess.run([str(closedloop_executable)], check=True)
    keil_executable = args.out.resolve() / "at32f456_keil_control.exe"
    subprocess.run(
        command[: command.index("-I")]
        + [
            "-I",
            str(ROOT / "tests/bench/at32f456"),
            "-DAT32_OL_VOLTAGE_V=2.0f",
            "-DAT32_OL_ELECTRICAL_HZ=20.0f",
            "-DAT32_OL_CURRENT_LIMIT_A=3.0f",
            "-DAT32_OL_DURATION_MS=10000U",
            str(ROOT / "tests/bench/at32f456/openloop_keil_control.c"),
            str(ROOT / "tests/bench/at32f456/openloop_model.c"),
            str(motor / "math/control_math.c"),
            str(native / "at32f456_keil_control_test.c"),
            "-lm",
            "-o",
            str(keil_executable),
        ],
        check=True,
    )
    subprocess.run([str(keil_executable)], check=True)
    rtt_executable = args.out.resolve() / "at32f456_rtt.exe"
    subprocess.run(
        command[: command.index("-I")]
        + [
            "-I",
            str(ROOT / "tests/bench/at32f456"),
            "-I",
            str(ROOT / "firmware/third_party/segger_rtt"),
            "-I",
            str(ROOT / "firmware/config"),
            "-I",
            str(ROOT / "firmware/config"),
            "-I",
            str(ROOT / "firmware/config"),
            "-I",
            str(ROOT / "firmware/platform/bsp"),
            str(ROOT / "tests/bench/at32f456/openloop_telemetry.c"),
            str(ROOT / "firmware/third_party/segger_rtt/SEGGER_RTT.c"),
            str(native / "at32f456_rtt_test.c"),
            "-lm",
            "-o",
            str(rtt_executable),
        ],
        check=True,
    )
    subprocess.run([str(rtt_executable)], check=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
