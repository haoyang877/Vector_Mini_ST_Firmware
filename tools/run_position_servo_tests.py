"""Compile and execute the real C servo; optionally replay recorded sensor inputs.

Example: python tools/run_position_servo_tests.py --cc /path/to/zig.exe
Replay fixes the old measured motion as an input, so it is NOT a plant simulation.
"""
import argparse
import json
import os
from pathlib import Path
import re
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def function_source(source, name):
    start = source.rfind("\n", 0, source.index(name + "(")) + 1
    brace = source.index("{", start)
    depth = 1
    end = brace + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end]


def encoder_fixture():
    source = (ROOT / "Bsp/encoder.c").read_text(encoding="utf-8")
    header = (ROOT / "Bsp/encoder.h").read_text(encoding="utf-8")
    macros = []
    for name, text in [("ENCODER_Q15_CPR", header), ("ENCODER_VELOCITY_WINDOW", header),
                       ("ENCODER_VELOCITY_ZERO_THRESHOLD_Q15", source)]:
        macros.append(re.search(r"^#define\s+" + name + r"\s+[^\r\n]+", text, re.M)[0])
    # Only peripheral-independent estimator functions are compiled here. The
    # hardware build separately checks the actual full Encoder_TypeDef layout.
    return """
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <assert.h>
#include <stdint.h>
#include <stdbool.h>
#include <limits.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#define _2PI 6.2831853072f
#define SPEED_LOOP_DIVIDER 10U
#define Speed_Ts 0.0005f
""" + "\n".join(macros) + """
typedef struct {
    int32_t velocity_delta_history[ENCODER_VELOCITY_WINDOW];
    uint8_t velocity_divider, velocity_history_index, velocity_sample_count;
    int32_t velocity_delta_sum;
    bool velocity_ready;
    int64_t velocity_shadow_q15, shadow_q15;
    float vel_mech, vel_elec, vel_mech_continuous;
} Encoder_TypeDef;
""" + "\n".join(function_source(source, name) for name in
    ["Encoder_ResetVelocity", "Encoder_UpdateVelocity2kHz", "Encoder_GetMecVel",
     "Encoder_GetMecVelContinuous"]) + """
int main(void) {
    Encoder_TypeDef e = {0};
    int k, n;
    Encoder_ResetVelocity(&e);
    for (k=0; k<400; ++k) {
        if ((k % 4)==0) e.shadow_q15++;
        for(n=0; n<10; ++n) Encoder_UpdateVelocity2kHz(&e, 7);
    }
    assert(Encoder_GetMecVel(&e)==0);
    assert(fabsf(Encoder_GetMecVelContinuous(&e)-_2PI/(65536.0f*.002f))<.00001f);
    Encoder_ResetVelocity(&e);
    assert(Encoder_GetMecVelContinuous(&e)==0 && !e.velocity_ready);
    for (k=0; k<400; ++k) {
        if ((k % 4)==0) e.shadow_q15--;
        for(n=0; n<10; ++n) Encoder_UpdateVelocity2kHz(&e, 7);
    }
    assert(Encoder_GetMecVel(&e)==0 && Encoder_GetMecVelContinuous(&e)<-.04f);
    for(k=0;k<100;++k) {
        e.shadow_q15+=20;
        for(n=0;n<10;++n) Encoder_UpdateVelocity2kHz(&e, 7);
    }
    assert(Encoder_GetMecVel(&e)==Encoder_GetMecVelContinuous(&e));
    assert(e.vel_elec==e.vel_mech*7);
    puts("PASS actual encoder estimator: low speed, both directions, reset, legacy path");
    return 0;
}
"""


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", default=os.environ.get("SERVO_CC"))
    parser.add_argument("--recording", type=Path)
    parser.add_argument("--out", type=Path, default=ROOT / "outputs/servo_optimization_20260908")
    args = parser.parse_args()
    cc = args.cc or shutil.which("zig") or shutil.which("gcc") or shutil.which("clang")
    if not cc:
        parser.error("Provide --cc with a native C99 compiler or Zig executable")
    args.out.mkdir(parents=True, exist_ok=True)
    compiler = [cc] + (["cc"] if Path(cc).stem == "zig" else [])
    logs = []

    def run(command):
        result = subprocess.run([str(x) for x in command], cwd=ROOT, capture_output=True, text=True)
        logs.append(result.stdout + result.stderr)
        print(result.stdout + result.stderr, end="")
        (args.out / "host_tests.log").write_text("\n".join(logs), encoding="utf-8")
        result.check_returncode()

    def build(name, files):
        executable = args.out / (name + (".exe" if os.name == "nt" else ""))
        run(compiler + ["-std=c99", "-O1", "-Wall", "-Wextra", "-Werror", "-I", "Foc"] +
            list(files) + ([] if os.name == "nt" else ["-lm"]) + ["-o", executable])
        run([executable])
        return executable

    exe = build("position_servo_test", ["tests/unit/position_servo_test.c",
                                        "Foc/position_cascade.c", "Foc/foc_pid.c"])
    fixture = args.out / "encoder_estimator_test.c"
    fixture.write_text(encoder_fixture(), encoding="utf-8")
    build("encoder_estimator_test", [fixture])
    build("servo_hil_test", ["tests/unit/servo_hil_test.c"])
    if args.recording:
        import numpy as np
        data = np.loadtxt(args.recording, delimiter="\t", skiprows=1)
        status = data[:, 11].astype(int)
        valid = (status & 128) != 0
        indexes = np.flatnonzero(valid)
        assert len(indexes) and np.all(np.diff(indexes) == 1), "Replay requires one contiguous valid interval"
        first = indexes[0]
        phase = status[indexes] & 3
        starts = np.flatnonzero((phase == 0) & np.r_[True, phase[:-1] != 0])
        position = np.unwrap(data[indexes, 1] * np.pi / 32768)
        reference = np.unwrap(data[indexes, 0] * np.pi / 32768)
        target = np.empty(len(indexes))
        episodes = []
        for n, a in enumerate(starts):
            end = starts[n+1] if n+1 < len(starts) else len(indexes)
            settled = a + np.flatnonzero(phase[a:end] != 0)[0]
            target[a:end] = reference[settled]
            episodes.append((int(a), int(settled), int(end)))
        assert starts[0] == 0
        velocity = np.zeros(len(indexes))
        velocity[16:] = (position[16:] - position[:-16]) / .008
        inputs = args.out / "replay_inputs.csv"
        output = args.out / "replay_outputs.csv"
        np.savetxt(inputs, np.column_stack([indexes / 2000., target, position, velocity]),
                   delimiter=",", fmt="%.9f")
        run([exe, inputs, output])
        replay = np.loadtxt(output, delimiter=",", skiprows=1)
        assert len(replay) == len(indexes) and np.isfinite(replay).all()
        tails = []
        for n, (_, settled, end) in enumerate(episodes[:-1]):
            tail = slice(max(settled, end-4000), end)
            ff_max = float(np.max(np.abs(replay[tail, 4])))
            assert ff_max < .001, "Settled original tail must not receive new breakaway pulses"
            tails.append({"move": n+1, "end_time_s": (end+first)/2000,
                          "max_abs_ff_A": ff_max,
                          "hold_fraction": float(np.mean(replay[tail, 7] != 0))})
        result = {"input_rows": len(indexes), "original_motion_is_fixed": True,
                  "not_a_prediction_of_new_closed_loop_motion": True,
                  "targets_inferred_from_original_trajectory_end": True,
                  "velocity_reconstructed_from_quantized_position_16_sample_window": True,
                  "settled_tails": tails}
        (args.out / "replay_summary.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
        print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
