"""Compile the actual deferred runtime with deterministic interrupt injection.

Checks publication ownership, stale reset/fault rejection, deadline faults,
2 kHz cadence, unchanged speed PI/ramp arithmetic, and application heap capacity.
Synthetic feedback is not an acceptance test of the physical plant.
"""

import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(_Path(__file__).resolve().parents[3] / "tools"))
from project_paths import ROOT, NATIVE_INCLUDE_FLAGS

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
                         (ROOT / 'firmware/platform/stm32g4/bsp/encoder.h').read_text(encoding="utf-8"), re.M)[1]
    src = (ROOT / 'firmware/app/foc_run.c').read_text(encoding="utf-8")
    block = src.split('/* OUTER_RUNTIME_BEGIN', 1)[1].split('/* OUTER_RUNTIME_END */', 1)[0]
    block = '/* OUTER_RUNTIME_BEGIN' + block
    funcs = '\n'.join(function_source(src, n) for n in [
        'SpeedMode_UpdateControl', 'PositionMode_UpdateConfiguration',
        'PositionMode_SameTuningValue', 'PositionMode_ConfigurationMatches'])
    # Use the real parameter structure to guard the full current allocation set.
    param = (ROOT / 'firmware/services/parameters/foc_param.h').read_text(encoding="utf-8")
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
#include "motor_load_profile.h"
#include "hw_conf.h"
#include "foc_pid.h"
#include "position_cascade.h"
#include "position_cascade_config.h"
#include "position_impedance_config.h"
#include "motor_thermal.h"
#include "heap.h"
#define FOC_CONFIG_NOINLINE
#define ENCODER_OFFSET_LUT_SIZE 1024U
typedef struct { float position, speed; } Encoder_TypeDef;
static MotorControl_TypeDef live;
static PI_Controller_TypeDef live_pi;
static Encoder_TypeDef enc;
static unsigned schedules, injection;
static unsigned position_reads, speed_reads, continuous_speed_reads;
static float Encoder_GetMecPos(Encoder_TypeDef *e) { ++position_reads; return e->position; }
static float Encoder_GetMecVel(Encoder_TypeDef *e) { ++speed_reads; return e->speed; }
static float Encoder_GetMecVelContinuous(Encoder_TypeDef *e) { ++continuous_speed_reads; return e->speed; }
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
    position_reads = speed_reads = continuous_speed_reads = 0;
    PositionCascade_Reset();
    live.ModeNow = mode; live.axis_profile_valid = true;
    live.pos_error_window = .001f; live.posAcc = .785398f; live.posDec = .523599f;
    live.pos_maxspeed = .6f; live.speed_limit = .8f;
    live.cascade_pos_Kp = 8; live.cascade_pos_Kd = 2;
    live.speed_Kp = .5f; live.speed_Ki = 2; live.current_limit = 4;
    live.position_speed_Kp = .5f; live.position_speed_Ki = 2;
    live.friction_attack_slew_a_per_s = 2; live.friction_release_slew_a_per_s = 15;
    live.posRef = enc.position; live.speedRef = .2f;
    live.speedAcc = .8f; live.speedDec = .5f;
}
static void run_step(void) {
    tick();
    if (outer.status == OUTER_QUEUED) MotorOuterLoop_Service();
}
int main(void) {
    /* 非活动模式仍执行状态清理，但不能重复获取本模式不消费的编码器量。 */
    const ModeNow_TypeDef inactive_modes[]={Motor_Disable,Current_Mode,Sensorless_Speed_Mode,
        Calib_EncoderObserver,Save_Param,Clear_Error};
    for(unsigned i=0;i<sizeof(inactive_modes)/sizeof(inactive_modes[0]);++i) {
        setup(inactive_modes[i]); enc.position=enc.speed=NAN;
        for(unsigned tick_index=0;tick_index<30;++tick_index) tick();
        assert(position_reads==0 && speed_reads==0 && continuous_speed_reads==0);
        assert(live.ErrorNow==No_Error && schedules==0);
    }
    setup(Speed_Mode); tick();
    assert(position_reads==1 && speed_reads==1 && continuous_speed_reads==0);
    setup(Position_Mode); tick();
    assert(position_reads==1 && speed_reads==0 && continuous_speed_reads==1);
    /* 标定工作区已静态化；堆仅保留一份 Flash 参数记录及分配头/对齐空间。 */
    assert(TOTAL_HEAP_SIZE==4096);
    for (unsigned trial = 0; trial < 100; trial++) {
        void *d = HEAP_malloc(sizeof(InterfaceParam_TypeDef));
        assert(d);
        memset(d,0xA5,sizeof(InterfaceParam_TypeDef));
        HEAP_free(d);
    }
    printf("heap capacity passed: calibration_static=8192 parameter=%zu pool=%zu minimum_free=%zu\n",
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
            offsetof(MotorControl_TypeDef, current_limit), offsetof(MotorControl_TypeDef, position_speed_Kp),
            offsetof(MotorControl_TypeDef, position_speed_Ki), offsetof(MotorControl_TypeDef, pos_error_window),
            offsetof(MotorControl_TypeDef, friction_attack_slew_a_per_s), offsetof(MotorControl_TypeDef, friction_release_slew_a_per_s),
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
    /* 非活动模式的 PI 不参与本模式验证或计算，两组不共享控制器状态。 */
    setup(Position_Mode);
    live.speed_Kp = NAN; live.speed_Ki = NAN;
    for (unsigned i = 0; i < 30; ++i) run_step();
    assert(live.ErrorNow == No_Error);
    assert(PositionCascade_GetConfiguration()->speed_kp == .5f);
    assert(PositionCascade_GetConfiguration()->speed_ki == 2);
    live.position_speed_Kp = .3f; live.position_speed_Ki = .7f;
    for (unsigned i = 0; i < 30; ++i) run_step();
    assert(live.ErrorNow == No_Error);
    assert(PositionCascade_GetConfiguration()->speed_kp == .3f);
    assert(PositionCascade_GetConfiguration()->speed_ki == .7f);
    setup(Speed_Mode);
    live.position_speed_Kp = NAN; live.position_speed_Ki = NAN;
    for (unsigned i = 0; i < 30; ++i) run_step();
    assert(live.ErrorNow == No_Error && live_pi.Kp == .5f && live_pi.Ki == 2);
    live.ModeNow = Position_Mode;
    tick();
    assert(live.ErrorNow == MotorParam_Error && live.iqRef == 0);
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
    /* 前馈状态由慢环计算、快环提交；停机/故障不能提交过期结果。 */
    setup(Speed_Mode);
    SpeedFeedforward_Default(&live.speed_ff_config);live.speed_ff_config.enabled=true;
    live.speedRef=20;enc.speed=0;
    for(unsigned i=0;i<10000;i++) run_step();
    assert(live.ErrorNow==No_Error && live.speed_ff_state.initialized);
    assert(live.speedShadow==live.speed_ff_state.speed_rad_s && live.speedShadow>1);
    assert(fabsf(live.speed_ff_state.current_a)<=2 && fabsf(live.iqRef)<=live.current_limit);
    live.ModeNow=Motor_Disable;tick();
    assert(!live.speed_ff_state.initialized && live.speed_ff_state.current_a==0);
    setup(Speed_Mode);
    SpeedFeedforward_Default(&live.speed_ff_config);live.speed_ff_config.enabled=true;
    live.speedRef=20;enc.speed=0;
    for(unsigned i=0;i<100;i++) run_step();
    live.ErrorNow=Over_Current;tick();
    assert(!live.speed_ff_state.initialized && live.speed_ff_state.current_a==0);
    setup(Speed_Mode);
    SpeedFeedforward_Default(&live.speed_ff_config);live.speed_ff_config.enabled=true;
    live.speed_ff_config.gain_a_per_rad_s2=NAN;tick();
    assert(live.ErrorNow==MotorParam_Error && live.iqRef==0);
    setup(Position_Mode);
    live.speed_ff_config.enabled=true;live.speed_ff_config.gain_a_per_rad_s2=NAN;
    for(unsigned i=0;i<30;i++) run_step();
    assert(live.ErrorNow==No_Error);
    /* 实际反馈越界仍可向合法目标运行；缓存不能掩盖新目标越界。 */
    for (int sign = -1; sign <= 1; sign += 2) {
        setup(Position_Mode);
        assert(MotorAxisProfile_Create(&live.axis_profile, "roll", -.2f, .2f, .6f));
        enc.position = sign * .25f;
        live.posRef = sign * .1f;
        for (unsigned i = 0; i < 200; ++i) run_step();
        assert(live.ErrorNow == No_Error && outer.ready);
        assert(enc.position == sign * .25f && sign * live.posShadow > .2f);
        assert(sign * live.iqRef < 0);
        enc.position = sign * .35f;
        for (unsigned i = 0; i < 20; ++i) run_step();
        assert(live.ErrorNow == No_Error);
        live.posRef = sign * .201f;
        tick();
        assert(live.ErrorNow == MotorParam_Error && live.iqRef == 0);
    }
    puts("PASS: legacy cadence/publication/PI and optional FF state publication, STOP/fault isolation");
    puts("PASS: real feedback outside travel retained; only out-of-range target faults");
}
'''
    fixture = fixture.replace('#define ENCODER_OFFSET_LUT_SIZE 1024U',
                              '#define ENCODER_OFFSET_LUT_SIZE ' + lut_size + 'U')
    (out / 'fixture.c').write_text(fixture)
    sources = [out / 'fixture.c', ROOT / 'firmware/services/parameters/motor_axis_profile.c', ROOT / 'firmware/motor/motor_load_profile.c', ROOT / 'firmware/motor/speed_feedforward.c', ROOT / 'firmware/motor/position/position_cascade.c',
               ROOT / 'firmware/motor/trajectory/position_smooth_trajectory.c', ROOT / 'firmware/motor/foc/foc_pid.c', ROOT / 'firmware/common/heap.c']
    cmd = [args.cc, 'cc', '-std=c99', '-O2', '-ffp-contract=off', '-Wall', '-Wextra', '-Werror',
           '-I' + str(out), '-I' + str(ROOT / 'firmware/motor/foc'), '-I' + str(ROOT / 'firmware/common'),
           '-I' + str(ROOT / 'firmware/platform/stm32g4/bsp'), '-I' + str(ROOT)]
    cmd += NATIVE_INCLUDE_FLAGS
    cmd += [str(x) for x in sources] + ['-o', str(out / 'outer_runtime.exe')]
    subprocess.run(cmd, check=True, cwd=ROOT)
    subprocess.run([str(out / 'outer_runtime.exe')], check=True, cwd=ROOT)


if __name__ == '__main__':
    main()
