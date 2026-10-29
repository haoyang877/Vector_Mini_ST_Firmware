"""编译真实R40端口并通过SDK替身检查启动、关相和故障；不连接硬件。"""

import sys as _sys
from pathlib import Path as _Path

_sys.path.insert(0, str(_Path(__file__).resolve().parents[3] / "tools"))
import argparse
import subprocess
from pathlib import Path

from project_paths import NATIVE_INCLUDE_FLAGS, ROOT
from run_position_servo_tests import function_source


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    base = ROOT / "firmware/platform/bsp/mcu/at32f456"
    native = ROOT / "tests/unit/native"
    permit_probe = args.out.resolve() / "permit_config_probe.c"
    permit_probe.write_text('#include "pwm_output_config.h"\n', encoding="utf8")
    cc = [args.cc] + (["cc"] if Path(args.cc).stem == "zig" else [])
    for flags in (
        [],
        ["-DR40_APP_PHASE_LOCK=1", "-DBSP_PWM_HARD_LOCK=1"],
        ["-DR40_APP_BOUNDED_TEST=1"],
    ):
        subprocess.run(
            cc
            + [
                "-c",
                "-I",
                str(ROOT / "firmware/platform/bsp"),
                "-I",
                str(ROOT / "firmware/app/config"),
                "-I",
                str(ROOT / "firmware/platform/config"),
                *flags,
                str(permit_probe),
                "-o",
                str(args.out.resolve() / "permit_config_probe.o"),
            ],
            check=True,
        )
    for flags in (
        ["-DR40_APP_PHASE_LOCK=1", "-DBSP_PWM_PERMIT_MODE=0"],
        ["-DR40_APP_BOUNDED_TEST=1", "-DBSP_PWM_PERMIT_MODE=0"],
        ["-DBSP_PWM_PERMIT_MODE=3"],
        ["-DR40_APP_PHASE_LOCK=1", "-DR40_APP_BOUNDED_TEST=1"],
        ["-DR40_APP_PHASE_LOCK=1"],
        ["-DBSP_PWM_HARD_LOCK=1"],
    ):
        rejected = subprocess.run(
            cc
            + [
                "-c",
                "-I",
                str(ROOT / "firmware/platform/bsp"),
                "-I",
                str(ROOT / "firmware/app/config"),
                "-I",
                str(ROOT / "firmware/platform/config"),
                *flags,
                str(permit_probe),
                "-o",
                str(args.out.resolve() / "permit_config_probe.o"),
            ],
            capture_output=True,
            check=False,
        )
        if rejected.returncode == 0 or not any(
            marker in rejected.stderr
            for marker in (
                b"Invalid PWM",
                b"Phase-locked target",
                b"Bounded target",
                b"Application PWM permit",
            )
        ):
            raise RuntimeError("Invalid power permit configuration accepted: " + str(flags))
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
                f"-DFOC_ACTIVE_HW_PROFILE={profile}",
                str(native / "app_bus_current_test.c"),
                str(ROOT / "firmware/app/composition/yg_protocol_status_adapter.c"),
                str(ROOT / "firmware/communication/commands/yg_protocol_motor.c"),
                str(ROOT / "shared/protocol/yg_protocol_crc.c"),
                str(ROOT / "firmware/communication/commands/yg_protocol_readonly.c"),
                "-lm",
                "-o",
                str(feedback_executable),
            ],
            check=True,
        )
        subprocess.run([str(feedback_executable)], check=True)
    executable = args.out.resolve() / "r40_foundation.exe"
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
        native / "r40_fake",
        base,
        ROOT / "firmware/platform/bsp/include",
        ROOT / "firmware/platform/config",
        ROOT / "firmware/app/config",
        ROOT / "firmware/platform/bsp",
        ROOT / "firmware/platform/devices",
        ROOT / "firmware/platform/bsp",
        ROOT / "firmware/platform/devices",
    ):
        command += ["-I", str(include)]
    sources = [
        base / "pwm.c",
        ROOT / "firmware/platform/bsp/bsp_pwm.c",
        base / "system.c",
        native / "r40_foundation_test.c",
    ]
    subprocess.run(command + [str(p) for p in sources] + ["-o", str(executable)], check=True)
    for failure in range(6):
        subprocess.run([str(executable), str(failure)], check=True)
    pwm_runtime = (ROOT / "firmware/platform/bsp/bsp_pwm.c").read_text(encoding="utf8")
    pwm_backend_source = (base / "pwm.c").read_text(encoding="utf8")
    compare_start = pwm_backend_source.index("#if BSP_PWM_PHASE_A_CHANNEL")
    compare_end = pwm_backend_source.index("void McuPwm_DisableOutputs", compare_start)
    (args.out / "pwm_duty_production.inc").write_text(
        pwm_backend_source[compare_start:compare_end]
        + "\n"
        + "\n".join(
            function_source((base / "pwm.c").read_text(encoding="utf8"), name)
            for name in (
                "McuPwm_PeriodCounts",
                "pwm_can_commit",
                "McuPwm_CommitThree",
                "McuPwm_CommitPhase",
                "McuPwm_ReadOutputs",
            )
        )
        + "\nstatic bool s_initialized=true, s_output_fault;\n"
        + "static float s_period_counts = 4500.0f;\n"
        + "static uint32_t s_disable_sequence;\n"
        + function_source(pwm_runtime, "BspPwm_GetStatus")
        + function_source(pwm_runtime, "duty_valid")
        + function_source(pwm_runtime, "pwm_commit_missed")
        + "\n".join(
            function_source(pwm_runtime, name) for name in ("BspPwm_SetDuty", "BspPwm_SetPhaseDuty")
        ),
        encoding="utf8",
    )
    bench_executable = args.out.resolve() / "r40_bench.exe"
    bench_command = command[: command.index("-I")] + ["-I", str(args.out.resolve())]
    for include in (
        native / "r40_bench_fake",
        base,
        ROOT / "firmware/platform/bsp/include",
        ROOT / "firmware/platform/config",
        ROOT / "firmware/app/config",
        ROOT / "firmware/platform/bsp",
        ROOT / "firmware/platform/devices",
    ):
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
                str(ROOT / "tests/bench/at32f456/openloop_model.c"),
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
        + [str(ROOT / "tests/unit/bsp_preemption_stubs.c")]
        + ["-I", str(ROOT / "tests/bench/at32f456")]
        + [
            "-DR40_OL_CURRENT_LIMIT_A=3.0f",
            "-DR40_OL_DURATION_MS=10000U",
            "-DR40_CLOSEDLOOP_TEST=1",
        ]
        + [
            str(path)
            for path in (
                ROOT / "tests/bench/at32f456/closedloop_bench.c",
                ROOT / "tests/bench/at32f456/openloop_model.c",
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
            str(ROOT / "tests/bench/at32f456"),
            "-DR40_OL_VOLTAGE_V=2.0f",
            "-DR40_OL_ELECTRICAL_HZ=20.0f",
            "-DR40_OL_CURRENT_LIMIT_A=3.0f",
            "-DR40_OL_DURATION_MS=10000U",
            str(ROOT / "tests/bench/at32f456/openloop_keil_control.c"),
            str(ROOT / "tests/bench/at32f456/openloop_model.c"),
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
            str(ROOT / "tests/bench/at32f456"),
            "-I",
            str(ROOT / "firmware/third_party/segger_rtt"),
            "-I",
            str(ROOT / "firmware/platform/config"),
            "-I",
            str(ROOT / "firmware/communication/telemetry"),
            "-I",
            str(ROOT / "firmware/app/config"),
            "-I",
            str(ROOT / "firmware/platform/bsp"),
            str(ROOT / "tests/bench/at32f456/openloop_telemetry.c"),
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
        + [str(ROOT / "tests/unit/bsp_preemption_stubs.c")]
        + [
            "-I",
            str(ROOT / "projects/at32f456/application"),
            "-I",
            str(ROOT / "firmware/third_party/segger_rtt"),
            "-I",
            str(ROOT / "firmware/platform/config"),
            "-I",
            str(ROOT / "firmware/communication/telemetry"),
            "-I",
            str(ROOT / "firmware/app/config"),
            "-I",
            str(ROOT / "firmware/platform/bsp"),
            str(ROOT / "firmware/communication/telemetry/app_telemetry.c"),
            str(ROOT / "firmware/third_party/segger_rtt/SEGGER_RTT.c"),
            str(native / "r40_app_telemetry_test.c"),
            "-o",
            str(app_rtt_executable),
        ],
        check=True,
    )
    subprocess.run([str(app_rtt_executable)], check=True)
    pwm_fixture = args.out.resolve() / "pwm_initialized.c"
    pwm_fixture.write_text(
        (ROOT / "firmware/platform/bsp/bsp_pwm.c")
        .read_text(encoding="utf8")
        .replace(
            "static bool s_initialized;",
            "static bool s_initialized = true;",
        ),
        encoding="utf8",
    )
    window_executable = args.out.resolve() / "r40_power_window.exe"
    subprocess.run(
        command[: command.index("-I")]
        + [
            "-DR40_APP_BOUNDED_TEST=1",
            "-I",
            str(ROOT / "firmware/app/runtime"),
            "-I",
            str(native / "r40_fake"),
            "-I",
            str(base),
            "-I",
            str(ROOT / "firmware/platform/bsp/include"),
            "-I",
            str(ROOT / "firmware/platform/config"),
            "-I",
            str(ROOT / "firmware/communication/telemetry"),
            "-I",
            str(ROOT / "firmware/app/config"),
            "-I",
            str(ROOT / "firmware/platform/bsp"),
            str(ROOT / "firmware/app/runtime/pwm_power_window.c"),
            str(native / "r40_fake/pwm_init_unexpected.c"),
            str(base / "pwm.c"),
            str(pwm_fixture),
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
            (
                native / "r40_indicator_fake",
                base,
                ROOT / "firmware/platform/bsp/include",
                ROOT / "firmware/platform/config",
                ROOT / "firmware/app/config",
                ROOT / "firmware/platform/bsp",
                ROOT / "firmware/platform/devices",
            ),
            (
                base / "led.c",
                ROOT / "firmware/platform/bsp/bsp_led.c",
                native / "r40_indicator_test.c",
            ),
            [],
        ),
        (
            "app_lock",
            (
                native / "r40_fake",
                ROOT / "firmware/platform/bsp/include",
                ROOT / "firmware/platform/config",
                ROOT / "firmware/app/config",
                ROOT / "firmware/platform/bsp",
                ROOT / "firmware/platform/devices",
                ROOT / "firmware/platform/bsp",
                ROOT / "firmware/platform/devices",
            ),
            (
                base / "pwm.c",
                pwm_fixture,
                native / "r40_app_lock_test.c",
                native / "r40_fake/pwm_init_unexpected.c",
            ),
            ["-DR40_APP_PHASE_LOCK=1", "-DBSP_PWM_HARD_LOCK=1"],
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
        + [str(ROOT / "tests/unit/bsp_preemption_stubs.c")]
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
        profile_command + ["-include", str(ROOT / "firmware/app/config/targets/r40_app_off.h")],
        check=True,
    )
    subprocess.run([str(args.out.resolve() / "profile_selection.exe")], check=True)
    # 复用原夹具文件，验证真实关相/限时功率契约及中断读回竞态。
    entry = (ROOT / "firmware/app/composition/app_tasks.c").read_text(encoding="utf8")
    fixture = (native / "r40_current_pulse_test.c").read_text(encoding="utf8")
    contract_source = args.out.resolve() / "power_contract_fixture.c"
    pwm_source = (ROOT / "firmware/platform/bsp/bsp_pwm.c").read_text(encoding="utf8")
    snapshot_source = (
        (
            function_source((base / "pwm.c").read_text(encoding="utf8"), "McuPwm_ReadOutputs")
            + "\n"
            + function_source(pwm_source, "BspPwm_GetStatus")
        )
        .replace("TMR1->cctrl", "probe_cctrl()")
        .replace("TMR1->brk", "probe_brk()")
    )
    contract_source.write_text(
        fixture.replace("/* PWM_SNAPSHOT_PRODUCTION */", snapshot_source)
        .replace(
            "/* POWER_CONTRACT_PRODUCTION */",
            function_source(entry, "power_contract_valid")
            .replace("TMR1->cctrl", "probe_cctrl()")
            .replace("TMR1->brk", "probe_brk()"),
        )
        .replace(
            "/* HALT_DISABLED_PRODUCTION */",
            function_source((base / "system.c").read_text(encoding="utf8"), "halt_disabled"),
        ),
        encoding="utf8",
    )
    for bounded in (0, 1):
        contract_binary = args.out.resolve() / f"power_contract_{bounded}.exe"
        subprocess.run(
            command[: command.index("-I")]
            + NATIVE_INCLUDE_FLAGS
            + [str(ROOT / "tests/unit/bsp_preemption_stubs.c")]
            + [
                f"-DR40_APP_BOUNDED_TEST={bounded}",
                "-I",
                str(base),
                str(contract_source),
                "-o",
                str(contract_binary),
            ],
            check=True,
        )
        subprocess.run([str(contract_binary)], check=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
