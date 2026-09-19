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
        (ROOT / "firmware/motor/position/angle_feedback.h").read_text(encoding="utf-8"),
        re.M,
    )[1]
    src = (ROOT / "firmware/app/foc_run.c").read_text(encoding="utf-8")
    speed_src = (ROOT / "firmware/motor/foc/foc_speed.c").read_text(encoding="utf-8")
    block = src.split("/* OUTER_RUNTIME_BEGIN", 1)[1].split("/* OUTER_RUNTIME_END */", 1)[0]
    block = "/* OUTER_RUNTIME_BEGIN" + block
    funcs = (
        "\n".join(
            function_source(speed_src, n)
            for n in ["MotorControl_UpdateSpeedRamp", "SpeedMode_UpdateControl"]
        )
        + "\n"
        + "\n".join(
            function_source(src, n)
            for n in [
                "PositionMode_ApplyFrictionConfiguration",
                "PositionMode_EffectiveDeceleration",
                "PositionMode_EffectiveMaximumSpeed",
                "PositionMode_UpdateConfiguration",
                "PositionMode_SameTuningValue",
                "PositionMode_ConfigurationMatches",
            ]
        )
    )
    # Use the real parameter structure to guard the full current allocation set.
    param = (ROOT / "firmware/services/parameters/foc_param.h").read_text(encoding="utf-8")
    param = param[
        param.index("typedef struct") : param.index("} InterfaceParam_TypeDef;")
        + len("} InterfaceParam_TypeDef;")
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
#include "data_type.h"
#include "hw_conf.h"
#include "foc_pid.h"
#include "position_cascade.h"
#include "position_cascade_config.h"
#include "position_impedance_config.h"
#include "heap.h"
#include "cogging_calibration.h"
#define FOC_CONFIG_NOINLINE
#define ENCODER_OFFSET_LUT_SIZE 1024U
typedef struct { float position, speed; } Encoder_TypeDef;
static MotorControl_TypeDef live;
static PI_Controller_TypeDef live_pi;
static Encoder_TypeDef enc;
static float Encoder_GetMecPos(Encoder_TypeDef *e) { return e->position; }
static float Encoder_GetMecVel(Encoder_TypeDef *e) { return e->speed; }
static float Encoder_GetMecVelContinuous(Encoder_TypeDef *e) { return e->speed; }
static void Set_ErrorNow(ErrorNow_TypeDef error) { live.ErrorNow = error; }
"""
        + param
        + funcs
        + block
        + r"""
static void slow_tick(void) { MotorOuterLoop_SlowTick(&live, &live_pi, &enc); }
static void setup(ModeNow_TypeDef mode) {
    memset(&outer, 0, sizeof(outer));
    memset(&live, 0, sizeof(live));
    memset(&live_pi, 0, sizeof(live_pi));
    enc.position = .3f; enc.speed = 0;
    PositionCascade_Reset();
    live.ModeNow = mode; live.axis_profile_valid = true;
    live.pos_error_window = .001f; live.posAcc = .785398f; live.posDec = .523599f;
    live.pos_maxspeed = .6f; live.speed_limit = .8f;
    live.cascade_pos_Kp = 8; live.cascade_pos_Kd = 2;
    live.speed_Kp = .5f; live.speed_Ki = 2; live.current_limit = 4;
    live.posRef = enc.position; live.speedRef = .2f;
    live.speedAcc = .8f; live.speedDec = .5f;
}
int main(void) {
    /* All known live heap allocations, even parameter I/O overlapping calibration. */
    for (unsigned trial = 0; trial < 100; trial++) {
        void *a = HEAP_malloc(ENCODER_OFFSET_LUT_SIZE * sizeof(int32_t));
        void *b = HEAP_malloc(ENCODER_OFFSET_LUT_SIZE * sizeof(uint16_t));
        void *c = HEAP_malloc(ENCODER_OFFSET_LUT_SIZE * sizeof(int16_t));
        void *d = HEAP_malloc(sizeof(InterfaceParam_TypeDef));
        assert(a && b && c && d);
        HEAP_free(b); HEAP_free(d); HEAP_free(a); HEAP_free(c);
    }
    printf("heap capacity passed: calibration=8192 parameter=%zu pool=%zu minimum_free=%zu\n",
        sizeof(InterfaceParam_TypeDef), TOTAL_HEAP_SIZE, HEAP_get_minimumEver_free_size());
    for (unsigned mode = Speed_Mode; mode <= Position_Mode; mode++) {
        setup((ModeNow_TypeDef)mode);
        for (unsigned i = 0; i < 20000; i++) {
            if (i % 1000 == 0)
            {
                live.posRef = .3f + (i % 2000 ? .05f : -.05f);
            }
            slow_tick();
            assert(live.ErrorNow == No_Error);
            assert(isfinite(live.iqRef) && fabsf(live.iqRef) <= live.current_limit);
        }
        assert(MotorOuterLoop_IsReady());
        /* Tuning/config invalidity is rejected on the next slow tick with zero output. */
        setup((ModeNow_TypeDef)mode); slow_tick();
        assert(MotorOuterLoop_IsReady());
        if (mode == Position_Mode)
        {
            live.posRef = NAN;
        }
        else
        {
            live.speedRef = NAN;
        }
        slow_tick();
        assert(live.ErrorNow == MotorParam_Error && live.iqRef == 0 && !MotorOuterLoop_IsReady());
        /* Mode change clears readiness; disabled modes publish nothing. */
        setup((ModeNow_TypeDef)mode); slow_tick();
        live.ModeNow = Motor_Disable; live.iqRef = 0;
        slow_tick();
        assert(!MotorOuterLoop_IsReady() && live.iqRef == 0);
        /* Reset requests are consumed inside the slow tick; the controller rebuilds and republishes. */
        setup((ModeNow_TypeDef)mode); slow_tick();
        MotorOuterLoop_RequestReset();
        assert(!MotorOuterLoop_IsReady());
        slow_tick();
        assert(MotorOuterLoop_IsReady());
        /* Existing faults block output publication. */
        setup((ModeNow_TypeDef)mode); slow_tick();
        live.ErrorNow = Over_Current; live.iqRef = 0; slow_tick();
        assert(live.iqRef == 0);
    }
    /* No stale validation cache may conceal a changed live tuning field. */
    {
        const size_t fields[] = {
            offsetof(MotorControl_TypeDef, current_limit), offsetof(MotorControl_TypeDef, speed_Kp),
            offsetof(MotorControl_TypeDef, speed_Ki), offsetof(MotorControl_TypeDef, pos_error_window),
            offsetof(MotorControl_TypeDef, posAcc), offsetof(MotorControl_TypeDef, posDec),
            offsetof(MotorControl_TypeDef, pos_maxspeed), offsetof(MotorControl_TypeDef, speed_limit),
            offsetof(MotorControl_TypeDef, cascade_pos_Kp), offsetof(MotorControl_TypeDef, cascade_pos_Kd),
            offsetof(MotorControl_TypeDef, friction_coulomb_pos_a), offsetof(MotorControl_TypeDef, friction_coulomb_neg_a),
            offsetof(MotorControl_TypeDef, friction_viscous_pos_a_per_rad_s), offsetof(MotorControl_TypeDef, friction_viscous_neg_a_per_rad_s)
        };
        for (unsigned i = 0; i < sizeof(fields)/sizeof(fields[0]); i++) {
            float invalid = NAN;
            setup(Position_Mode); live.friction_model_valid = true;
            slow_tick(); slow_tick();
            assert(MotorOuterLoop_IsReady());
            memcpy((char *)&live + fields[i], &invalid, sizeof(invalid));
            slow_tick(); assert(live.ErrorNow == MotorParam_Error && live.iqRef == 0);
        }
    }
    /* PI/ramp arithmetic must match the shared sequential implementation exactly. */
    setup(Speed_Mode);
    slow_tick();
    MotorControl_TypeDef expected = live;
    PI_Controller_TypeDef expected_pi = live_pi;
    for (unsigned step = 0; step < 20000; step++) {
        if (step % 700 == 0)
        {
            live.speedRef = expected.speedRef = -live.speedRef;
        }
        SpeedMode_UpdateControl(&expected, &expected_pi, enc.speed);
        slow_tick();
        if (live.speedShadow != expected.speedShadow || live.iqRef != expected.iqRef) {
            fprintf(stderr, "step %u shadow %.9g/%.9g iq %.9g/%.9g error %u\n",
                step, live.speedShadow, expected.speedShadow, live.iqRef, expected.iqRef,
                live.ErrorNow);
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
