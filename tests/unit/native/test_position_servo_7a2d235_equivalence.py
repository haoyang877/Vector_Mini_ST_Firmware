"""离线比较恢复内核与 7a2d235 的相同输入输出，不访问硬件。"""

import argparse
import hashlib
import json
import os
import shutil
import subprocess
from pathlib import Path

from run_position_servo_tests import NATIVE_INCLUDE_FLAGS, ROOT, function_source

REFERENCE_COMMIT = "7a2d235bf94966bf8743b85d0962a1f00abc7158"
REFERENCE_FILES = (
    "Foc/position_cascade.c",
    "Foc/position_cascade.h",
    "Foc/position_cascade_config.h",
    "Foc/position_smooth_trajectory.c",
    "Foc/position_smooth_trajectory.h",
    "Foc/foc_pid.c",
    "Foc/foc_pid.h",
    "System/fast_loop_profile.h",
)


def fixture(config_source: str) -> str:
    """同一夹具固定反馈轨迹，逐字段输出浮点位型和状态；不模拟机械闭环。"""
    return (
        r"""
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "position_cascade.h"
#include "position_cascade_config.h"
"""
        + config_source
        + r"""
static void emit_float(FILE *stream, float value) {
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    fprintf(stream, ",%08lx", (unsigned long)bits);
}
int main(int argc, char **argv) {
    assert(argc == 2);
    FILE *stream = fopen(argv[1], "w");
    assert(stream != NULL);
    for (unsigned scenario = 0; scenario < 8; ++scenario) {
        PositionCascadeConfig_TypeDef c = config();
        if (scenario == 3) { c.current_limit = 0.2f; }
        if (scenario == 4) {
            c.position_kp = 8.0f; c.position_kd = 2.0f;
            c.speed_kp = 0.5f; c.speed_ki = 1.0f;
            c.acceleration_feedforward_gain = 0.25f;
        }
        if (scenario == 6) { c.following_error_limit = 0.01f; }
        if (scenario == 7) { c.call_divider = 10; }
        PositionCascade_Reset();
        for (unsigned tick = 0; tick < 4000; ++tick) {
            PositionCascadeOutput_TypeDef o;
            PositionCascadeTelemetry_TypeDef t;
            float position = 0.0f, speed = 0.0f;
            c.target_position = tick < 1400 ? 0.35f : (tick < 2800 ? -0.2f : 0.0f);
            if (scenario == 0) {
                c.target_position = 0.0f;
                position = 0.0003f * sinf((float)tick * 0.11f);
                speed = 0.005f * cosf((float)tick * 0.11f);
            } else if (scenario == 1 || scenario == 4 || scenario == 7) {
                position = 0.15f * sinf((float)tick * 0.0017f);
                speed = 0.3f * cosf((float)tick * 0.0017f);
            } else if (scenario == 2) {
                position = tick < 2200 ? 0.34f : -0.19f;
                speed = tick < 100 ? 0.07f : 0.0f;
            } else if (scenario == 3) {
                position = -0.8f;
                speed = -0.01f;
            } else if (scenario == 5) {
                c.friction_feedforward_enabled = false;
                c.update_period_s = tick < 2000 ? 0.0005f : 0.001f;
                c.velocity_filter_hz = tick < 2000 ? 10.0f : 5.0f;
                c.hold_velocity_filter_hz = tick < 2000 ? 0.0f : 3.0f;
                c.target_position = 0.0f;
                speed = 0.02f * sinf((float)tick * 0.17f);
            }
            if (scenario == 7 && tick > 0) {
                assert(PositionCascade_UpdateTarget(c.target_position, position, speed, &o));
            } else {
                assert(PositionCascade_Update(&c, position, speed, &o));
            }
            assert(PositionCascade_GetTelemetry(&t));
#ifdef RESTORED_CONTROLLER
            assert(!c.thermal_limit_active);
            assert(o.support_current_a == 0.0f && !t.passive_hold);
            assert(t.support_current_a == 0.0f && !t.support_adjusting);
            assert(!t.support_limited && !t.support_refining);
            assert(t.passive_confirm_ticks == 0 && t.qualification_position_rad == 0.0f);
#endif
            fprintf(stream, "%u,%u", scenario, tick);
            emit_float(stream, o.position_reference);
            emit_float(stream, o.speed_reference);
            emit_float(stream, o.acceleration_reference);
            emit_float(stream, o.speed_feedback);
            emit_float(stream, o.iq_reference);
            emit_float(stream, o.feedback_current);
            emit_float(stream, o.acceleration_feedforward_current);
            emit_float(stream, o.friction_feedforward_current);
            emit_float(stream, o.hold_current);
            emit_float(stream, t.position_reference);
            emit_float(stream, t.trajectory_speed_reference);
            emit_float(stream, t.speed_command);
            emit_float(stream, t.speed_feedback);
            emit_float(stream, t.acceleration_reference);
            emit_float(stream, t.feedback_current);
            emit_float(stream, t.acceleration_feedforward_current);
            emit_float(stream, t.friction_feedforward_current);
            emit_float(stream, t.hold_current);
            fprintf(stream, ",%u,%u,%u,%u,%u,%u,%u,%u,%u,%u\n",
                (unsigned)o.phase, o.target_reached, (unsigned)t.phase,
                t.target_reached, t.current_saturated, t.friction_landing_active,
                t.settle_recovery_active, t.hold_candidate_active,
                t.trajectory_limited, t.stiction_integrating);
        }
    }
    assert(fclose(stream) == 0);
    return 0;
}
"""
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", default=os.environ.get("SERVO_CC"))
    parser.add_argument(
        "--out", type=Path, default=ROOT / "outputs/position_servo_7a2d235_equivalence"
    )
    args = parser.parse_args()
    cc = args.cc or shutil.which("zig") or shutil.which("gcc") or shutil.which("clang")
    if not cc:
        parser.error("Provide --cc with a native C99 compiler or Zig executable")
    if Path(cc).is_file():
        cc = str(Path(cc).resolve())
    compiler = [cc] + (["cc"] if Path(cc).stem == "zig" else [])
    output = args.out.resolve()
    output.mkdir(parents=True, exist_ok=True)
    log: list[str] = []

    def run(command: list[str | Path]) -> None:
        result = subprocess.run(
            [str(item) for item in command], cwd=ROOT, capture_output=True, text=True
        )
        log.append(result.stdout + result.stderr)
        (output / "test.log").write_text("\n".join(log), encoding="utf-8")
        if result.stdout or result.stderr:
            print(result.stdout + result.stderr, end="")
        result.check_returncode()

    historical = output / "reference_source"
    source_hashes: dict[str, str] = {}
    for relative in REFERENCE_FILES:
        content = subprocess.check_output(
            ["git", "show", f"{REFERENCE_COMMIT}:{relative}"], cwd=ROOT
        )
        destination = historical / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_bytes(content)
        source_hashes[relative] = hashlib.sha256(content).hexdigest()
    historical_test = subprocess.check_output(
        ["git", "show", f"{REFERENCE_COMMIT}:tests/unit/position_servo_test.c"],
        cwd=ROOT,
    ).decode("utf-8")
    source = output / "equivalence_fixture.c"
    source.write_text(fixture(function_source(historical_test, "config")), encoding="utf-8")
    common = ["-std=c99", "-O1", "-UNDEBUG", "-Wall", "-Wextra", "-Werror"]
    suffix = ".exe" if os.name == "nt" else ""
    link = [] if os.name == "nt" else ["-lm"]
    old_exe, new_exe = output / ("reference" + suffix), output / ("restored" + suffix)
    run(
        compiler
        + common
        + ["-I", historical / "Foc", source]
        + [historical / relative for relative in REFERENCE_FILES if relative.endswith(".c")]
        + link
        + ["-o", old_exe]
    )
    current_sources = [
        ROOT / "firmware/motor/position/position_cascade.c",
        ROOT / "firmware/motor/trajectory/position_smooth_trajectory.c",
        ROOT / "firmware/motor/foc/foc_pid.c",
    ]
    run(
        compiler
        + common
        + NATIVE_INCLUDE_FLAGS
        + ["-DRESTORED_CONTROLLER=1", source]
        + current_sources
        + link
        + ["-o", new_exe]
    )
    reference_csv, restored_csv = output / "reference.csv", output / "restored.csv"
    run([old_exe, reference_csv])
    run([new_exe, restored_csv])
    reference, restored = reference_csv.read_bytes(), restored_csv.read_bytes()
    if reference != restored:
        for index, (left, right) in enumerate(
            zip(reference.splitlines(), restored.splitlines(), strict=False)
        ):
            if left != right:
                raise RuntimeError(f"Output mismatch at row {index}: {left!r} != {right!r}")
        raise RuntimeError("Output row count differs")
    summary = {
        "status": "PASS",
        "reference_commit": REFERENCE_COMMIT,
        "scenarios": 8,
        "rows": len(reference.splitlines()),
        "float_comparison": "exact IEEE-754 binary32 bit pattern for 18 output/telemetry fields",
        "state_comparison": "10 output/telemetry enum or boolean fields",
        "thermal_limit_active": False,
        "compatibility_fields_zero": True,
        "fixed_sensor_inputs_only_not_closed_loop_performance": True,
        "output_sha256": hashlib.sha256(reference).hexdigest(),
        "reference_source_sha256": source_hashes,
        "current_source_sha256": {
            str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in current_sources
        },
    }
    (output / "summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print(f"PASS exact 7a2d235/current output equivalence: {summary['rows']} rows in 8 scenarios")


if __name__ == "__main__":
    main()
