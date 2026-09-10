"""Compile the actual deferred runtime with deterministic interrupt injection.

Checks publication ownership, stale reset/fault rejection, deadline faults,
2 kHz cadence, unchanged speed PI/ramp arithmetic, and application heap capacity.
Synthetic feedback is not an acceptance test of the physical plant.
"""
import argparse
from pathlib import Path
import subprocess
import re
from run_position_servo_tests import ROOT, function_source


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--cc', required=True)
    p.add_argument('--out', type=Path, default=ROOT / 'outputs/outer_loop_host')
    args = p.parse_args()
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    (out / 'main.h').write_text('#include <stdint.h>\n#include <stddef.h>\n')
    lut_size = re.search(r'^#define\s+ENCODER_OFFSET_LUT_SIZE\s+(\d+)U',
                         (ROOT / 'Bsp/encoder.h').read_text(), re.M)[1]
    src = (ROOT / 'Foc/foc_run.c').read_text()
    block = src.split('/* OUTER_RUNTIME_BEGIN', 1)[1].split('/* OUTER_RUNTIME_END */', 1)[0]
    block = '/* OUTER_RUNTIME_BEGIN' + block
    funcs = '\n'.join(function_source(src, n) for n in [
        'SpeedMode_UpdateControl', 'PositionMode_UpdateConfiguration',
        'PositionMode_SameTuningValue', 'PositionMode_ConfigurationMatches'])
    # Use the real parameter structure to guard the full current allocation set.
    param = (ROOT / 'Foc/foc_param.h').read_text()
    param = param[param.index('typedef struct'):param.index('} InterfaceParam_TypeDef;') + len('} InterfaceParam_TypeDef;')]
    fixture = r'''
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
#define FOC_CONFIG_NOINLINE
#define ENCODER_OFFSET_LUT_SIZE 1024U
typedef struct { float position, speed; } Encoder_TypeDef;
static MotorControl_TypeDef live;
static PI_Controller_TypeDef live_pi;
static Encoder_TypeDef enc;
static unsigned schedules, injection;
static float Encoder_GetMecPos(Encoder_TypeDef *e) { return e->position; }
static float Encoder_GetMecVel(Encoder_TypeDef *e) { return e->speed; }
static float Encoder_GetMecVelContinuous(Encoder_TypeDef *e) { return e->speed; }
static void Set_ErrorNow(ErrorNow_TypeDef error) { live.ErrorNow = error; }
static void motor_hw_outer_barrier(void);
static void motor_hw_outer_schedule(void) { schedules++; }
''' + param + funcs + block + r'''
static void tick(void) { MotorOuterLoop_FastTick(&live, &live_pi, &enc); }
static void motor_hw_outer_barrier(void) {
    if (injection && outer.status == OUTER_RUNNING) {
        unsigned action = injection;
        float old_iq = live.iqRef;
        PositionCascadeTelemetry_TypeDef before, after;
        bool valid = MotorOuterLoop_GetTelemetry(&before);
        injection = 0;
        if (action == 1) {
            tick();
            assert(live.iqRef == old_iq);
            assert(valid == MotorOuterLoop_GetTelemetry(&after));
            if (valid) assert(memcmp(&before, &after, sizeof(before)) == 0);
        } else if (action == 2) {
            live.ModeNow = Motor_Disable;
            live.iqRef = 0;
            tick();
        } else if (action == 3) {
            live.ErrorNow = Over_Current;
            live.iqRef = 0;
            tick();
        } else if (action == 4) {
            for (unsigned i = 0; i < 10; i++) tick();
            assert(live.ErrorNow == ControlOverrun_Error);
        } else if (action == 5) {
            MotorOuterLoop_RequestReset();
            live.iqRef = 0;
        }
    }
}
static void setup(ModeNow_TypeDef mode) {
    memset(&outer, 0, sizeof(outer));
    memset(&live, 0, sizeof(live));
    memset(&live_pi, 0, sizeof(live_pi));
    enc.position = .3f; enc.speed = 0;
    schedules = injection = 0;
    PositionCascade_Reset();
    live.ModeNow = mode; live.axis_profile_valid = true;
    live.pos_error_window = .001f; live.posAcc = .785398f; live.posDec = .523599f;
    live.pos_maxspeed = .6f; live.speed_limit = .8f;
    live.cascade_pos_Kp = 8; live.cascade_pos_Kd = 2;
    live.speed_Kp = .5f; live.speed_Ki = 2; live.current_limit = 4;
    live.posRef = enc.position; live.speedRef = .2f;
    live.speedAcc = .8f; live.speedDec = .5f;
}
static void run_step(void) {
    tick();
    if (outer.status == OUTER_QUEUED) MotorOuterLoop_Service();
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
        for (unsigned i = 0; i < 200000; i++) {
            if (i % 10000 == 0) live.posRef = .3f + (i % 20000 ? .05f : -.05f);
            run_step();
            assert(live.ErrorNow == No_Error);
            assert(isfinite(live.iqRef) && fabsf(live.iqRef) <= live.current_limit);
        }
        assert(schedules == 20000 && outer.completed == 20000);
        assert(outer.maximum_age == 1 && outer.deadline_misses == 0);
        assert(outer.status == OUTER_IDLE);
        /* Inject a preemption at the completed calculation before publication. */
        for (unsigned action = 1; action <= 5; action++) {
            setup((ModeNow_TypeDef)mode);
            tick(); assert(outer.status == OUTER_QUEUED);
            injection = action; MotorOuterLoop_Service();
            assert(injection == 0 && outer.status == OUTER_DONE);
            tick(); assert(outer.status == OUTER_IDLE);
            if (action >= 2) assert(live.iqRef == 0 && !outer.ready);
            if (action == 2 || action == 3 || action == 5) assert(outer.discarded == 1);
            if (action == 4) assert(outer.deadline_misses == 1);
        }
        /* A queued job may outlive STOP; no output may be resurrected. */
        setup((ModeNow_TypeDef)mode); tick();
        live.ModeNow = Motor_Disable; live.iqRef = 0; tick();
        MotorOuterLoop_Service(); tick();
        assert(live.iqRef == 0 && outer.discarded == 1 && !outer.ready);
        /* Validate invalid commands immediately, even on a non-release tick. */
        setup((ModeNow_TypeDef)mode); run_step(); run_step();
        if (mode == Position_Mode) live.posRef = NAN; else live.speedRef = NAN;
        tick(); assert(live.ErrorNow == MotorParam_Error && live.iqRef == 0);
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
            run_step(); run_step();
            assert(outer.ready && MotorOuterLoop_SameTuning(&live));
            memcpy((char *)&live + fields[i], &invalid, sizeof(invalid));
            tick(); assert(live.ErrorNow == MotorParam_Error && live.iqRef == 0);
        }
    }
    /* PI/ramp arithmetic must match the shared sequential implementation exactly. */
    setup(Speed_Mode);
    MotorControl_TypeDef expected = live;
    PI_Controller_TypeDef expected_pi = live_pi;
    for (unsigned step = 0; step < 2000; step++) {
        if (step % 70 == 0) live.speedRef = expected.speedRef = -live.speedRef;
        SpeedMode_UpdateControl(&expected, &expected_pi, enc.speed);
        for (unsigned i = 0; i < 10; i++) run_step();
        if (live.speedShadow != expected.speedShadow || live.iqRef != expected.iqRef) {
            fprintf(stderr, "step %u shadow %.9g/%.9g iq %.9g/%.9g status %u error %u\n",
                step, live.speedShadow, expected.speedShadow, live.iqRef, expected.iqRef,
                outer.status, live.ErrorNow);
            assert(0);
        }
        assert(memcmp(&live_pi, &expected_pi, sizeof(live_pi)) == 0);
    }
    puts("PASS: 420000 fast ticks; cadence, coherent publication, preemption, STOP/fault/reset, deadline, PI/ramp");
}
'''
    fixture = fixture.replace('#define ENCODER_OFFSET_LUT_SIZE 1024U',
                              '#define ENCODER_OFFSET_LUT_SIZE ' + lut_size + 'U')
    (out / 'fixture.c').write_text(fixture)
    sources = [out / 'fixture.c', ROOT / 'Foc/position_cascade.c',
               ROOT / 'Foc/position_smooth_trajectory.c', ROOT / 'Foc/foc_pid.c', ROOT / 'System/heap.c']
    cmd = [args.cc, 'cc', '-std=c99', '-O2', '-ffp-contract=off', '-Wall', '-Wextra', '-Werror',
           '-I' + str(out), '-I' + str(ROOT / 'Foc'), '-I' + str(ROOT / 'System'),
           '-I' + str(ROOT / 'Bsp'), '-I' + str(ROOT)]
    cmd += [str(x) for x in sources] + ['-o', str(out / 'outer_runtime.exe')]
    subprocess.run(cmd, check=True, cwd=ROOT)
    subprocess.run([str(out / 'outer_runtime.exe')], check=True, cwd=ROOT)


if __name__ == '__main__':
    main()
