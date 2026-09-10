"""Compare the real mode-3 adapter against the pre-cache adapter, tick by tick.

The frozen reference is extracted from Git, generated only under --out, and is
never part of firmware. Feedback is synthetic, not a motor/plant simulation.
"""
import argparse
from pathlib import Path
import re
import subprocess
from run_position_servo_tests import ROOT, function_source


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--cc', required=True)
    p.add_argument('--out', type=Path, default=ROOT / 'outputs/position_config_cache')
    p.add_argument('--reference-ref', default='f092e3d')
    args = p.parse_args()
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    source = (ROOT / 'Foc/foc_run.c').read_text(encoding='utf-8')
    reference = subprocess.check_output(['git', 'show', args.reference_ref + ':Foc/foc_run.c'],
                                        cwd=ROOT).decode('utf-8')
    reference = function_source(reference, 'Task_Position_Mode').replace(
        'Task_Position_Mode(', 'Task_Position_Mode_Reference(')
    actual = '\n'.join(function_source(source, n) for n in [
        'PositionMode_UpdateConfiguration', 'PositionMode_SameTuningValue',
        'PositionMode_ConfigurationMatches', 'Task_Position_Mode'])
    fields = sorted(set(re.findall(r'MotorControl->(\w+)', actual + reference)) -
                    {'axis_profile', 'axis_profile_valid', 'friction_model_valid', 'isReachTargetPos'})
    typedef = 'typedef struct {\n' + '\n'.join('float ' + f + ';' for f in fields) + '''
MotorAxisProfile axis_profile;
bool axis_profile_valid, friction_model_valid, isReachTargetPos;
} MotorControl_TypeDef;
'''
    # Only the hardware definitions consumed by the adapter's tuning header.
    (out / 'hw_conf.h').write_text('''
#define MOTOR_DAMPING_FEEDFORWARD_ENABLED 1U
#define MOTOR_DAMPING_FEEDFORWARD 1U
#define Cascade_Position_Ts 0.0005f
#define CASCADE_POSITION_LOOP_DIVIDER 10U
''', encoding='utf-8')
    fixture = r'''
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <assert.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdint.h>
#include "System/fast_loop_profile.h"
#include "position_cascade.h"
#include "position_cascade_config.h"
#include "position_impedance_config.h"
#include "software/config/motor_axis_profile.h"
#define FOC_CONFIG_NOINLINE
''' + typedef + r'''
typedef struct { float position, velocity; } Encoder_TypeDef;
typedef struct { unsigned calls; } FOC_TypeDef;
enum { MotorParam_Error = 12 };
static unsigned error;
static void Set_ErrorNow(unsigned value) { error = value; }
static float Encoder_GetElePhase(Encoder_TypeDef *e) { return e->position * 7.0f; }
static float Encoder_GetMecPos(Encoder_TypeDef *e) { return e->position; }
static float Encoder_GetEleVel(Encoder_TypeDef *e) { return e->velocity * 7.0f; }
static float Encoder_GetMecVelContinuous(Encoder_TypeDef *e) { return e->velocity; }
/* Deliberately no register/PWM writes in the host fixture. */
static void FOC_Current(FOC_TypeDef *f, MotorControl_TypeDef *m, float a, float v) {
    (void)m; (void)a; (void)v; f->calls++;
}
''' + reference + '\n' + actual + r'''
typedef struct {
    MotorControl_TypeDef motor;
    PositionCascadeTelemetry_TypeDef telemetry;
    unsigned error, calls;
    bool defer;
} Sample;
static Sample expected[1200];
static void setup(MotorControl_TypeDef *m) {
    memset(m, 0, sizeof(*m));
    m->axis_profile_valid = true;
    m->pos_error_window = .001f; m->posAcc = .785398f; m->posDec = .523599f;
    m->pos_maxspeed = .6f; m->speed_limit = .8f;
    m->cascade_pos_Kp = 8; m->cascade_pos_Kd = 2;
    m->speed_Kp = .5f; m->speed_Ki = 2; m->current_limit = 6;
    m->friction_coulomb_pos_a = 1.4f; m->friction_coulomb_neg_a = 1.3f;
    m->friction_viscous_pos_a_per_rad_s = .1f;
    m->friction_viscous_neg_a_per_rad_s = .2f;
}
static void change(MotorControl_TypeDef *m, unsigned scenario, unsigned tick) {
    if (tick == 127 || tick == 253 || tick == 411) m->posRef += tick == 253 ? -.18f : .09f;
    if (tick != 131 && tick != 271) return;
    switch(scenario) {
''' + '\n'.join(
        f'case {i}: m->{field} = tick == 131 ? {value} : NAN; break;'
        for i, (field, value) in enumerate([
            ('pos_error_window', '.0011f'), ('posAcc', '.5f'), ('posDec', '.3f'),
            ('pos_maxspeed', '.4f'), ('speed_limit', '.7f'), ('cascade_pos_Kp', '9.0f'),
            ('cascade_pos_Kd', '1.0f'), ('speed_Kp', '.3f'), ('speed_Ki', '1.0f'),
            ('current_limit', '4.0f'), ('friction_coulomb_pos_a', '1.2f'),
            ('friction_coulomb_neg_a', '1.1f'), ('friction_viscous_pos_a_per_rad_s', '.3f'),
            ('friction_viscous_neg_a_per_rad_s', '.4f')])) + r'''
    case 14: m->friction_model_valid = !m->friction_model_valid; break;
    case 15: m->axis_profile.magic=1; m->axis_profile.minimum_position_rad=-1;
        m->axis_profile.maximum_position_rad=1;
        m->axis_profile.maximum_speed_rad_s=tick==131 ? .3f : .2f; break;
    case 16: m->axis_profile_valid=false; break;
    case 17: m->posRef=NAN; break;
    case 18: m->speed_Ki=tick==131 ? 0.0f : -0.0f; break;
    case 19: m->posDec=tick==131 ? 1.0f : 2.0f; break;
    default: break;
    }
}
int main(void) {
    unsigned scenario, pass, tick;
    PositionCascadeOutput_TypeDef o;
    PositionCascade_Reset();
    assert(PositionCascade_GetConfiguration()==NULL);
    assert(!PositionCascade_UpdateTarget(0,0,0,&o));
    for(scenario=0;scenario<21;scenario++) for(pass=0;pass<2;pass++) {
        MotorControl_TypeDef m; FOC_TypeDef f={0}; Encoder_TypeDef e;
        setup(&m); m.friction_model_valid=true;
        PositionCascade_Reset();
        for(tick=0;tick<1200;tick++) {
            Sample s;
            if(tick==601) { PositionCascade_Reset(); setup(&m); }
            change(&m,scenario,tick);
            e.position=.0001f * (float)((int)(tick%17)-8);
            e.velocity=.0002f * (float)((int)(tick%11)-5);
            if(scenario==20 && tick==263) e.velocity=NAN;
            error=0; f.calls=0;
            if(pass==0) Task_Position_Mode_Reference(&f,&m,&e);
            else Task_Position_Mode(&f,&m,&e);
            memset(&s,0,sizeof(s)); s.motor=m;
            PositionCascade_GetTelemetry(&s.telemetry);
            s.error=error; s.calls=f.calls; s.defer=PositionCascade_ShouldDeferTelemetry();
            if(pass==0) expected[tick]=s;
            else if(memcmp(&expected[tick],&s,sizeof(s))!=0) {
                fprintf(stderr,"Mismatch scenario=%u tick=%u error=%u/%u defer=%d/%d\n",
                    scenario,tick,expected[tick].error,s.error,expected[tick].defer,s.defer);
                return 1;
            }
        }
    }
    assert(!PositionCascade_UpdateTarget(NAN,0,0,&o));
    assert(!PositionCascade_UpdateTarget(0,NAN,0,&o));
    assert(!PositionCascade_UpdateTarget(0,0,NAN,&o));
    assert(!PositionCascade_UpdateTarget(0,0,0,NULL));
    puts("PASS 25200 tick comparisons: every live tuning field, NaN, signed zero, target/axis bounds, reset, telemetry/divider, same-tick rejection");
    return 0;
}
'''
    src = out / 'config_cache_test.c'
    src.write_text(fixture, encoding='utf-8')
    exe = out / 'config_cache_test.exe'
    cc = [args.cc] + (['cc'] if Path(args.cc).stem == 'zig' else [])
    log = []
    for command in [cc + ['-std=c99', '-O2', '-Wall', '-Wextra', '-Werror', '-I', str(out),
                         '-I', 'Foc', '-I', '.', str(src), 'Foc/position_cascade.c',
                         'Foc/position_smooth_trajectory.c', 'Foc/foc_pid.c', '-o', str(exe)], [str(exe)]]:
        r = subprocess.run(command, cwd=ROOT, capture_output=True, text=True)
        log.append(r.stdout + r.stderr)
        (out / 'test.log').write_text('\n'.join(log), encoding='utf-8')
        print(log[-1], end='')
        r.check_returncode()


if __name__ == '__main__':
    main()
