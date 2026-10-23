"""编译真实R40端口并通过SDK替身检查启动、关相和故障；不连接硬件。"""

import sys as _sys
from pathlib import Path as _Path

_sys.path.insert(0, str(_Path(__file__).resolve().parents[3] / "tools"))
import argparse
import subprocess
from pathlib import Path

from project_paths import NATIVE_INCLUDE_FLAGS, ROOT


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    base = ROOT / "firmware/platform/at32f456"
    native = ROOT / "tests/unit/native"
    for profile in (1, 2):
        feedback_executable = args.out.resolve() / f"app_bus_current_{profile}.exe"
        feedback_command = [args.cc] + (["cc"] if Path(args.cc).stem == "zig" else [])
        subprocess.run(
            feedback_command
            + NATIVE_INCLUDE_FLAGS
            + [
                "-I",
                str(base / "bsp"),
                "-std=c11",
                "-O2",
                "-UNDEBUG",
                "-Wall",
                "-Wextra",
                "-Werror",
                f"-DFOC_ACTIVE_HW_PROFILE={profile}",
                str(native / "app_bus_current_test.c"),
                str(ROOT / "firmware/app/composition/yg_protocol_status_adapter.c"),
                str(ROOT / "firmware/communication/protocol/yg_protocol_motor_status.c"),
                "-lm",
                "-o",
                str(feedback_executable),
            ],
            check=True,
        )
        subprocess.run([str(feedback_executable)], check=True)
    executable = args.out.resolve() / "r40_foundation.exe"
    command = [args.cc] + (["cc"] if Path(args.cc).stem == "zig" else [])
    command += ["-std=c11", "-O2", "-UNDEBUG", "-Wall", "-Wextra", "-Werror"]
    for include in (native / "r40_fake", base / "bsp", ROOT / "firmware/platform/api"):
        command += ["-I", str(include)]
    sources = [
        base / "bsp/r40_clock.c",
        base / "bsp/r40_platform.c",
        base / "ports/power_stage_at32f456.c",
        base / "ports/motor_pwm_at32f456.c",
        base / "ports/time_at32f456.c",
        native / "r40_foundation_test.c",
    ]
    subprocess.run(command + [str(p) for p in sources] + ["-o", str(executable)], check=True)
    for failure in range(6):
        subprocess.run([str(executable), str(failure)], check=True)
    bench_executable = args.out.resolve() / "r40_bench.exe"
    bench_command = command[: command.index("-I")]
    for include in (native / "r40_bench_fake", base / "bsp", ROOT / "firmware/platform/api"):
        bench_command += ["-I", str(include)]
    subprocess.run(
        bench_command + [str(native / "r40_bench_test.c"), "-o", str(bench_executable)],
        check=True,
    )
    subprocess.run([str(bench_executable)], check=True)
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
            args.out.resolve() / f"r40_openloop_{voltage}_{duration}_{current}_{frequency}.exe"
        )
        subprocess.run(
            command[: command.index("-I")]
            + [
                f"-DR40_OL_VOLTAGE_V={voltage}f",
                f"-DR40_OL_DURATION_MS={duration}U",
                f"-DR40_OL_CURRENT_LIMIT_A={current}f",
                f"-DR40_OL_ELECTRICAL_HZ={frequency}f",
                str(base / "foundation/openloop_model.c"),
                str(ROOT / "firmware/motor/math/control_math.c"),
                str(native / "r40_openloop_test.c"),
                "-lm",
                "-o",
                str(openloop_executable),
            ],
            check=True,
        )
        subprocess.run([str(openloop_executable)], check=True)
    closedloop_executable = args.out.resolve() / "r40_closedloop.exe"
    motor = ROOT / "firmware/motor"
    subprocess.run(
        command[: command.index("-I")]
        + NATIVE_INCLUDE_FLAGS
        + ["-I", str(base / "foundation")]
        + [
            "-DR40_OL_CURRENT_LIMIT_A=3.0f",
            "-DR40_OL_DURATION_MS=10000U",
            "-DR40_CLOSEDLOOP_TEST=1",
        ]
        + [
            str(path)
            for path in (
                base / "foundation/closedloop_bench.c",
                base / "foundation/openloop_model.c",
                motor / "math/control_math.c",
                motor / "foc/foc_algorithm.c",
                motor / "foc/foc_pid.c",
                motor / "foc/speed_control.c",
                motor / "foc/speed_feedforward.c",
                motor / "feedback/angle_feedback.c",
                motor / "protection/motor_thermal.c",
                native / "r40_closedloop_test.c",
            )
        ]
        + ["-lm", "-o", str(closedloop_executable)],
        check=True,
    )
    subprocess.run([str(closedloop_executable)], check=True)
    keil_executable = args.out.resolve() / "r40_keil_control.exe"
    subprocess.run(
        command[: command.index("-I")]
        + [
            "-I",
            str(base / "foundation"),
            "-DR40_OL_VOLTAGE_V=2.0f",
            "-DR40_OL_ELECTRICAL_HZ=20.0f",
            "-DR40_OL_CURRENT_LIMIT_A=3.0f",
            "-DR40_OL_DURATION_MS=10000U",
            str(base / "foundation/openloop_keil_control.c"),
            str(base / "foundation/openloop_model.c"),
            str(motor / "math/control_math.c"),
            str(native / "r40_keil_control_test.c"),
            "-lm",
            "-o",
            str(keil_executable),
        ],
        check=True,
    )
    subprocess.run([str(keil_executable)], check=True)
    rtt_executable = args.out.resolve() / "r40_rtt.exe"
    subprocess.run(
        command[: command.index("-I")]
        + [
            "-I",
            str(base / "foundation"),
            "-I",
            str(ROOT / "firmware/third_party/segger_rtt"),
            str(base / "foundation/openloop_telemetry.c"),
            str(ROOT / "firmware/third_party/segger_rtt/SEGGER_RTT.c"),
            str(native / "r40_rtt_test.c"),
            "-lm",
            "-o",
            str(rtt_executable),
        ],
        check=True,
    )
    subprocess.run([str(rtt_executable)], check=True)
    app_rtt_executable = args.out.resolve() / "r40_app_rtt.exe"
    subprocess.run(
        command[: command.index("-I")]
        + NATIVE_INCLUDE_FLAGS
        + [
            "-I",
            str(base / "application"),
            "-I",
            str(ROOT / "firmware/third_party/segger_rtt"),
            str(base / "application/app_telemetry.c"),
            str(ROOT / "firmware/third_party/segger_rtt/SEGGER_RTT.c"),
            str(native / "r40_app_telemetry_test.c"),
            "-o",
            str(app_rtt_executable),
        ],
        check=True,
    )
    subprocess.run([str(app_rtt_executable)], check=True)
    window_executable = args.out.resolve() / "r40_power_window.exe"
    subprocess.run(
        command[: command.index("-I")]
        + [
            "-DR40_APP_BOUNDED_TEST=1",
            "-I",
            str(native / "r40_fake"),
            "-I",
            str(base / "bsp"),
            "-I",
            str(ROOT / "firmware/platform/api"),
            str(base / "bsp/r40_power_window.c"),
            str(base / "ports/power_stage_at32f456.c"),
            str(native / "r40_power_window_test.c"),
            "-o",
            str(window_executable),
        ],
        check=True,
    )
    subprocess.run([str(window_executable)], check=True)
    for name, includes, sources, flags in (
        (
            "indicator",
            (native / "r40_indicator_fake", base / "bsp", ROOT / "firmware/platform/api"),
            (base / "ports/indicator_at32f456.c", native / "r40_indicator_test.c"),
            [],
        ),
        (
            "app_lock",
            (native / "r40_fake", ROOT / "firmware/platform/api"),
            (base / "ports/power_stage_at32f456.c", native / "r40_app_lock_test.c"),
            ["-DR40_APP_PHASE_LOCK=1"],
        ),
    ):
        cmd = command[: command.index("-I")] + flags
        for directory in includes:
            cmd += ["-I", str(directory)]
        binary = args.out.resolve() / f"r40_{name}.exe"
        subprocess.run(cmd + [str(p) for p in sources] + ["-o", str(binary)], check=True)
        subprocess.run([str(binary)], check=True)
    # 未锁定的新电机完整APP仍拒绝；关相目标验证临时热配置走真实派生和校验路径。
    profile_source = args.out.resolve() / "profile_selection_test.c"
    profile_source.write_text(
        '#include "motor_profile_defaults.h"\n'
        '#include "motor_thermal_config.h"\n'
        "#include <assert.h>\n"
        "#include <math.h>\n"
        "int main(void) { assert(PARAM_MOTOR_POLE_PAIRS == 14); "
        "assert(fabsf(PARAM_MOTOR_FLUX_WB - 0.003814285714f) < 1e-9f); "
        "assert(PARAM_MOTOR_ROTOR_INERTIA_KG_M2 == 0.0005f); "
        "MotorWindingThermalConfig config; MotorThermalConfig derived; "
        "MotorWindingThermal_Default(&config); "
        "assert(MotorWindingThermal_Derive(&config, &derived)); "
        "assert(derived.heating_tau_s == 3600.0f && derived.cooling_tau_s == 3600.0f); "
        "assert(derived.continuous_vector_a == 3.0f && derived.peak_a == 3.0f); "
        "assert(derived.samples_per_update == 20U); return 0; }\n",
        encoding="utf8",
    )
    profile_command = (
        command[: command.index("-I")]
        + NATIVE_INCLUDE_FLAGS
        + [
            "-DFOC_ACTIVE_MOTOR_PROFILE=2",
            str(profile_source),
            str(motor / "protection/motor_thermal_config.c"),
            str(motor / "protection/motor_thermal.c"),
            "-lm",
            "-o",
            str(args.out.resolve() / "profile_selection.exe"),
        ]
    )
    denied = subprocess.run(profile_command, capture_output=True, check=False)
    if denied.returncode == 0 or b"Motor APP defaults incomplete" not in denied.stderr:
        raise RuntimeError("Incomplete production motor defaults were not rejected")
    subprocess.run(
        profile_command + ["-DAPP_TARGET_R40_F456=1", "-DR40_APP_PHASE_LOCK=1"], check=True
    )
    subprocess.run([str(args.out.resolve() / "profile_selection.exe")], check=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
