"""Run offline Python tests and optional native C checks; never access hardware."""
import argparse
from pathlib import Path
import subprocess
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from project_paths import ROOT

NATIVE = (
    'run_position_servo_tests', 'run_can_status_tests', 'test_outer_loop_runtime',
    'test_wheel_speed_limits', 'test_bus_voltage_protection', 'test_current_oversampling', 'test_current_precision',
    'test_adc_fast_dispatch', 'test_encoder_sample_overlap', 'test_fast_loop_math',
    'test_position_config_cache', 'test_rtt_calibration', 'test_sensorless_transitions',
    'test_cogging_calibration',
    'test_mcu_temperature',
)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cc', help='Native compiler executable; Zig is supported')
    parser.add_argument('--reference-source', type=Path, help='Pre-change position_cascade.c for equivalence checks')
    parser.add_argument('--out', type=Path, default=ROOT / 'outputs/tests')
    args = parser.parse_args()
    if args.reference_source and not args.cc:
        parser.error('--reference-source requires --cc')
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    commands = [(name, [sys.executable, '-m', 'unittest', 'discover', '-s', str(ROOT / 'tests' / name), '-p', 'test_*.py'])
                for name in ('unit', 'integration')]
    if args.cc:
        for name in NATIVE:
            commands.append((name, [sys.executable, str(ROOT / 'tests/unit/native' / (name + '.py')),
                                    '--cc', args.cc, '--out', str(out / name)]))
        if args.reference_source:
            commands.append(('equivalence', [sys.executable, str(ROOT / 'tests/unit/native/test_position_core_equivalence.py'),
                            '--cc', args.cc, '--reference-source', str(args.reference_source.resolve()),
                            '--out', str(out / 'equivalence')]))
    failed = []
    for name, command in commands:
        result = subprocess.run(command, cwd=ROOT, capture_output=True)
        (out / (name + '.log')).write_bytes(result.stdout + result.stderr)
        print(f'{name}: {"PASS" if result.returncode == 0 else "FAIL"} (exit {result.returncode})', flush=True)
        if result.returncode:
            failed.append(name)
    print('Logs:', out)
    return int(bool(failed))


if __name__ == '__main__':
    raise SystemExit(main())
