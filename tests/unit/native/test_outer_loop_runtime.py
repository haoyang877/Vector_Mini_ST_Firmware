"""以确定性慢拍编译并运行真实 2 kHz 外环。

覆盖发布所有权、模式切换/复位、无效输入与故障拒绝、速度 PI/斜坡算术不变与
应用堆容量；合成反馈不构成物理被控对象验收。
"""

import sys as _sys
from pathlib import Path as _Path

_sys.path.insert(0, str(_Path(__file__).resolve().parents[3] / "tools"))
import argparse
import re
import subprocess
from pathlib import Path

from project_paths import NATIVE_INCLUDE_FLAGS, ROOT
from run_position_servo_tests import function_source


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--cc", required=True)
    p.add_argument("--out", type=Path, default=ROOT / "outputs/outer_loop_host")
    args = p.parse_args()
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    (out / "main.h").write_text("#include <stdint.h>\n#include <stddef.h>\n")
    lut_size = re.search(
        r"^#define\s+ENCODER_OFFSET_LUT_SIZE\s+(\d+)U",
        (ROOT / "firmware/motor/feedback/angle_feedback.h").read_text(encoding="utf-8"),
        re.M,
    )[1]
    src = (ROOT / "firmware/app/runtime/motor_execution.c").read_text(encoding="utf-8")
    speed_src = (ROOT / "firmware/motor/foc/foc_speed.c").read_text(encoding="utf-8")
    block = src.split("/* OUTER_RUNTIME_BEGIN", 1)[1].split("/* OUTER_RUNTIME_END */", 1)[0]
    block = "/* OUTER_RUNTIME_BEGIN" + block
    funcs = (
        "\n".join(
            function_source(speed_src, n)
            for n in ["MotorAxis_UpdateSpeedRamp", "SpeedMode_UpdateControl"]
        )
        + "\n"
        + "\n".join(
            function_source(src, n)
            for n in [
                "position_mode_apply_friction_configuration",
                "position_mode_effective_deceleration",
                "position_mode_effective_maximum_speed",
                "position_mode_update_configuration",
                "position_mode_same_tuning_value",
                "position_mode_configuration_matches",
            ]
        )
    )
    # Use the real parameter structure to guard the mutually exclusive heap users.
    param = (ROOT / "firmware/services/parameters/foc_param.h").read_text(encoding="utf-8")
    param = param[
        param.index("typedef struct") : param.index("} InterfaceParam;") + len("} InterfaceParam;")
    ]
    fixture = (
        r"""
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "motor_types.h"
#include "hw_conf.h"
#include "foc_pid.h"
#include "position_cascade.h"
#include "position_cascade_config.h"
#include "heap.h"
#include "cogging_calibration.h"
#define FOC_CONFIG_NOINLINE
#define ENCODER_OFFSET_LUT_SIZE 1024U
typedef struct { float position, speed; } EncoderState;
static MotorControlState live;
static PiController live_pi;
static EncoderState enc;
static float Encoder_GetMecPos(EncoderState *e) { return e->position; }
static float Encoder_GetMecVel(EncoderState *e) { return e->speed; }
static float Encoder_GetMecVelContinuous(EncoderState *e) { return e->speed; }
static void MotorFault_Report(MotorControlState *m, MotorFaultCode error) { m->fault_code = error; }
"""
        + param
        + funcs
        + block
        + r"""
static void slow_tick(void) { MotorOuterLoop_SlowTick(&live, &live_pi, &enc); }
static void setup(MotorControlMode mode) {
    memset(&s_outer, 0, sizeof(s_outer));
    memset(&live, 0, sizeof(live));
    memset(&live_pi, 0, sizeof(live_pi));
    enc.position = .3f; enc.speed = 0;
    PositionCascade_Reset();
    live.control_mode = mode; live.application.axis_profile_valid = true;
    live.application.position_error_window_rad = .001f; live.application.position_acceleration_rad_s2 = .785398f; live.application.position_deceleration_rad_s2 = .523599f;
    live.application.position_maximum_speed_rad_s = .6f; live.application.speed_limit_rad_s = .8f;
    live.application.cascade_position_kp = 8; live.application.cascade_position_kd = 2;
    live.application.speed_kp = .5f; live.application.speed_ki = 2; live.application.current_limit_a = 4;
    live.position_target_rad = enc.position; live.speed_target_rad_s = .2f;
    live.application.speed_acceleration_rad_s2 = .8f; live.application.speed_deceleration_rad_s2 = .5f;
}
int main(void) {
    /* Calibration and parameter I/O are serialized; the pool must fit either peak. */
    for (unsigned trial = 0; trial < 100; trial++) {
        void *calibration = Heap_Allocate(ENCODER_OFFSET_LUT_SIZE * sizeof(int32_t) +
                                        ENCODER_OFFSET_LUT_SIZE * sizeof(uint16_t));
        assert(calibration);
        assert(Heap_Allocate(sizeof(InterfaceParam)) == NULL);
        Heap_Free(calibration);
        void *parameter = Heap_Allocate(sizeof(InterfaceParam));
        assert(parameter);
        Heap_Free(parameter);
    }
    printf("heap capacity passed: calibration=6144 parameter=%zu pool=%zu minimum_free=%zu\n",
        sizeof(InterfaceParam), TOTAL_HEAP_SIZE, Heap_GetMinimumFreeBytes());
    for (unsigned mode = MOTOR_CONTROL_SPEED; mode <= MOTOR_CONTROL_POSITION; mode++) {
        setup((MotorControlMode)mode);
        for (unsigned i = 0; i < 20000; i++) {
            if (i % 1000 == 0)
            {
                live.position_target_rad = .3f + (i % 2000 ? .05f : -.05f);
            }
            slow_tick();
            assert(live.fault_code == MOTOR_FAULT_NONE);
            assert(isfinite(live.iq_target_a) && fabsf(live.iq_target_a) <= live.application.current_limit_a);
        }
        assert(MotorOuterLoop_IsReady());
        /* Tuning/config invalidity is rejected on the next slow tick with zero output. */
        setup((MotorControlMode)mode); slow_tick();
        assert(MotorOuterLoop_IsReady());
        if (mode == MOTOR_CONTROL_POSITION)
        {
            live.position_target_rad = NAN;
        }
        else
        {
            live.speed_target_rad_s = NAN;
        }
        slow_tick();
        assert(live.fault_code == MOTOR_FAULT_INVALID_PARAMETERS && live.iq_target_a == 0 && !MotorOuterLoop_IsReady());
        /* Mode change clears readiness; disabled modes publish nothing. */
        setup((MotorControlMode)mode); slow_tick();
        live.control_mode = MOTOR_CONTROL_NONE; live.iq_target_a = 0;
        slow_tick();
        assert(!MotorOuterLoop_IsReady() && live.iq_target_a == 0);
        /* Reset requests are consumed inside the slow tick; the controller rebuilds and republishes. */
        setup((MotorControlMode)mode); slow_tick();
        motor_outer_loop_request_reset();
        assert(!MotorOuterLoop_IsReady());
        slow_tick();
        assert(MotorOuterLoop_IsReady());
        /* Existing faults block output publication. */
        setup((MotorControlMode)mode); slow_tick();
        live.fault_code = MOTOR_FAULT_OVERCURRENT; live.iq_target_a = 0; slow_tick();
        assert(live.iq_target_a == 0);
    }
    /* No stale validation cache may conceal a changed live tuning field. */
    {
        const size_t fields[] = {
            offsetof(MotorControlState, application.current_limit_a), offsetof(MotorControlState, application.speed_kp),
            offsetof(MotorControlState, application.speed_ki), offsetof(MotorControlState, application.position_error_window_rad),
            offsetof(MotorControlState, application.position_acceleration_rad_s2), offsetof(MotorControlState, application.position_deceleration_rad_s2),
            offsetof(MotorControlState, application.position_maximum_speed_rad_s), offsetof(MotorControlState, application.speed_limit_rad_s),
            offsetof(MotorControlState, application.cascade_position_kp), offsetof(MotorControlState, application.cascade_position_kd),
            offsetof(MotorControlState, device.friction_coulomb_pos_a), offsetof(MotorControlState, device.friction_coulomb_neg_a),
            offsetof(MotorControlState, device.friction_viscous_pos_a_per_rad_s), offsetof(MotorControlState, device.friction_viscous_neg_a_per_rad_s)
        };
        for (unsigned i = 0; i < sizeof(fields)/sizeof(fields[0]); i++) {
            float invalid = NAN;
            setup(MOTOR_CONTROL_POSITION); live.device.friction_model_valid = true;
            slow_tick(); slow_tick();
            assert(MotorOuterLoop_IsReady());
            memcpy((char *)&live + fields[i], &invalid, sizeof(invalid));
            slow_tick(); assert(live.fault_code == MOTOR_FAULT_INVALID_PARAMETERS && live.iq_target_a == 0);
        }
    }
    /* PI/ramp arithmetic must match the shared sequential implementation exactly. */
    setup(MOTOR_CONTROL_SPEED);
    slow_tick();
    MotorControlState expected = live;
    PiController expected_pi = live_pi;
    for (unsigned step = 0; step < 20000; step++) {
        if (step % 700 == 0)
        {
            live.speed_target_rad_s = expected.speed_target_rad_s = -live.speed_target_rad_s;
        }
        SpeedMode_UpdateControl(&expected, &expected_pi, enc.speed);
        slow_tick();
        if (live.speed_ramped_rad_s != expected.speed_ramped_rad_s || live.iq_target_a != expected.iq_target_a) {
            fprintf(stderr, "step %u shadow %.9g/%.9g iq %.9g/%.9g error %u\n",
                step, live.speed_ramped_rad_s, expected.speed_ramped_rad_s, live.iq_target_a, expected.iq_target_a,
                live.fault_code);
            assert(0);
        }
        assert(memcmp(&live_pi, &expected_pi, sizeof(live_pi)) == 0);
    }
    puts("PASS: 2 kHz slow tick; publication ownership, mode/reset, invalid input, fault hold, PI/ramp");
}
"""
    )
    fixture = fixture.replace(
        "#define ENCODER_OFFSET_LUT_SIZE 1024U", "#define ENCODER_OFFSET_LUT_SIZE " + lut_size + "U"
    )
    (out / "fixture.c").write_text(fixture, encoding="utf-8")
    sources = [
        out / "fixture.c",
        ROOT / "firmware/motor/position/position_cascade.c",
        ROOT / "firmware/motor/trajectory/position_smooth_trajectory.c",
        ROOT / "firmware/motor/foc/foc_pid.c",
        ROOT / "firmware/common/heap.c",
    ]
    cmd = [
        args.cc,
        "cc",
        "-std=c99",
        "-O2",
        "-UNDEBUG",
        "-ffp-contract=off",
        "-Wall",
        "-Wextra",
        "-Werror",
        "-I" + str(out),
        "-I" + str(ROOT / "firmware/motor/foc"),
        "-I" + str(ROOT / "firmware/common"),
        "-I" + str(ROOT / "firmware/platform/stm32g4/bsp"),
        "-I" + str(ROOT),
    ]
    cmd += NATIVE_INCLUDE_FLAGS
    cmd += [str(x) for x in sources] + ["-o", str(out / "outer_runtime.exe")]
    subprocess.run(cmd, check=True, cwd=ROOT)
    subprocess.run([str(out / "outer_runtime.exe")], check=True, cwd=ROOT)


if __name__ == "__main__":
    main()
