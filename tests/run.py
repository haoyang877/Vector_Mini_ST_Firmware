"""运行离线 Python 测试与可选原生 C 检查；不访问任何硬件。"""

import argparse
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from project_paths import ROOT

NATIVE = (
    "run_position_servo_tests",
    "test_outer_loop_runtime",
    "test_wheel_speed_limits",
    "test_bus_voltage_protection",
    "test_current_oversampling",
    "test_current_precision",
    "test_startup_current_offsets",
    "test_adc_fast_dispatch",
    "test_encoder_sample_overlap",
    "test_fast_loop_math",
    "test_shared_contracts",
    "test_foc_equivalence",
    "test_numeric_encoding",
    "test_loader_upgrade",
    "test_loader_runtime",
    "test_maintenance_workspace",
    "test_run_state",
    "test_motor_state_request",
    "test_execution_dispatch",
    "test_sensorless_transitions",
    "test_mcu_temperature",
    "test_yg_protocol_core",
    "test_yg_protocol_fragment",
    "test_yg_protocol_motor_adapter",
    "test_parameter_value_service",
    "test_yg_protocol_link",
    "test_comm_bus_off",
    "test_motor_thermal",
    "test_speed_feedforward",
    "test_observer_calibration_hotpath",
)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", help="Native compiler executable; Zig is supported")
    parser.add_argument("--out", type=Path, default=ROOT / "outputs/tests")
    args = parser.parse_args()
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    commands = [
        (
            name,
            [
                sys.executable,
                "-m",
                "unittest",
                "discover",
                "-s",
                str(ROOT / "tests" / name),
                "-p",
                "test_*.py",
            ],
        )
        for name in ("unit", "integration")
    ]
    if args.cc:
        for name in NATIVE:
            commands.append(
                (
                    name,
                    [
                        sys.executable,
                        str(ROOT / "tests/unit/native" / (name + ".py")),
                        "--cc",
                        args.cc,
                        "--out",
                        str(out / name),
                    ],
                )
            )
    failed = []
    for name, command in commands:
        result = subprocess.run(command, cwd=ROOT, capture_output=True)
        (out / (name + ".log")).write_bytes(result.stdout + result.stderr)
        print(
            f"{name}: {'PASS' if result.returncode == 0 else 'FAIL'} (exit {result.returncode})",
            flush=True,
        )
        if result.returncode:
            failed.append(name)
    print("Logs:", out)
    return int(bool(failed))


if __name__ == "__main__":
    raise SystemExit(main())
