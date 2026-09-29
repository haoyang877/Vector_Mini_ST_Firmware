"""在无电机硬件下验证无感交接代码。

覆盖电流矢量连续性、交接期间的实时速度反馈与制动限幅；
这不是物理被控对象验收测试。
"""

import argparse
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tools"))
from project_paths import NATIVE_INCLUDE_FLAGS, ROOT
from run_position_servo_tests import function_source

PRELUDE = r"""
#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>
#include "motor_execution.h"
#include "foc_sensorless_run.h"
#include "foc_speed.h"
#include "hw_conf.h"
#include "motor_startup_profile.h"
#include "utils.h"
static MotorControlState motor;
static FocState foc;
static PiController pi;
static FluxObserverState observer;
static SensorlessStartupState startup;
static EncoderState encoder;
static unsigned current_calls, stop_calls;
static bool pwm_enabled;
static float applied_observer_iq, applied_observer_id, applied_phase, applied_velocity;
void MotorFault_Report(MotorControlState *m, MotorFaultCode e) { m->fault_code=e; }
void FOC_CurrentController_Reset(FocState *f) {
    PiController_Reset(&f->id_pi); PiController_Reset(&f->iq_pi);
}
void FOC_Current(FocState *f, MotorControlState *m, float phase, float velocity) {
    (void)f;
    float offset=phase-observer.electrical_angle_rad;
    applied_observer_iq=m->id_target_a*sinf(offset)+m->iq_target_a*cosf(offset);
    applied_observer_id=m->id_target_a*cosf(offset)-m->iq_target_a*sinf(offset);
    applied_phase=phase; applied_velocity=velocity; ++current_calls;
}
void FluxObserver_ParamInit(FluxObserverState *o) { memset(o,0,sizeof(*o)); }
float Observer_GetElePhase(FluxObserverState *o) { return o->electrical_angle_rad; }
float Observer_GetEleVel(FluxObserverState *o) { return o->electrical_velocity_rad_s; }
float Observer_GetElePosition(FluxObserverState *o) { return o->electrical_position_rad; }
uint32_t Observer_GetPositionEpoch(FluxObserverState *o) { return o->position_epoch; }
bool Encoder_IsOnline(const EncoderState *e) { return e->has_valid_sample; }
float Encoder_GetMecVelContinuous(const EncoderState *e) { return e->mechanical_velocity_continuous_rad_s; }
void Encoder_ResetVelocity(EncoderState *e) { e->velocity_ready=false; }
static void *Heap_Allocate(size_t n) { return malloc(n); }
static void Heap_Free(void *p) { free(p); }
"""

CASES = r"""
/* 编码器标定配置已随 foc-Calibration 功能移除：夹具内联同参配置供无感用例使用。 */
static const SensorlessStartupConfig test_startup_config = {
    SENSORLESS_ENCODER_CALIB_ALIGN_CURRENT_RAMP_TIME_S,
    SENSORLESS_ENCODER_CALIB_ALIGN_HOLD_TIME_S,
    SENSORLESS_ENCODER_CALIB_ALIGN_CURRENT_A,
    SENSORLESS_ENCODER_CALIB_STARTUP_IQ_INITIAL_A,
    SENSORLESS_ENCODER_CALIB_STARTUP_IQ_A,
    SENSORLESS_ENCODER_CALIB_STARTUP_IQ_RAMP_TIME_S,
    SENSORLESS_ENCODER_CALIB_STARTUP_ID_A,
    SENSORLESS_ENCODER_CALIB_MIN_CURRENT_LIMIT_A,
    SENSORLESS_ENCODER_CALIB_MIN_ELEC_VEL_RAD_S,
    SENSORLESS_ENCODER_CALIB_TARGET_ELEC_VEL_RAD_S,
    SENSORLESS_ENCODER_CALIB_STARTUP_RAMP_TIME_S,
    SENSORLESS_ENCODER_CALIB_SPEED_LOCK_TIME_S,
    SENSORLESS_ENCODER_CALIB_SPEED_LOCK_FILTER_ALPHA,
    SENSORLESS_ENCODER_CALIB_OBSERVER_LOCK_RATIO,
    SENSORLESS_ENCODER_CALIB_ANGLE_HANDOFF_TIME_S,
    SENSORLESS_ENCODER_CALIB_LOCK_TIMEOUT_S,
    SENSORLESS_ENCODER_CALIB_ID_RAMP_DOWN_TIME_S,
    SENSORLESS_ENCODER_CALIB_OBSERVER_LOSS_TIME_S};
static const SensorlessStartupConfig *cfg=&test_startup_config;
static void setup(void) {
    memset(&motor,0,sizeof(motor)); memset(&foc,0,sizeof(foc));
    memset(&observer,0,sizeof(observer)); memset(&encoder,0,sizeof(encoder));
    PiController_Reset(&pi); SensorlessStartup_Reset(&startup);
    motor.model.motor_pole_pairs=21; motor.model.motor_phase_resistance_ohm=1.905f;
    motor.model.motor_d_inductance_h=motor.model.motor_q_inductance_h=.001635f;
    motor.model.motor_flux_wb=.0175025f; motor.application.current_limit_a=6;
    motor.application.speed_kp=.02f; motor.application.speed_ki=.5f;
    motor.application.speed_acceleration_rad_s2=motor.application.speed_deceleration_rad_s2=314.159f;
    motor.control_mode=MOTOR_CONTROL_NONE;
    motor.speed_target_rad_s=cfg->target_electrical_velocity_rad_s/21;
    motor.speed_ramped_rad_s=motor.speed_target_rad_s;
    observer.electrical_velocity_rad_s=cfg->target_electrical_velocity_rad_s;
    encoder.has_valid_sample=true; encoder.velocity_ready=true;
    encoder.mechanical_velocity_continuous_rad_s=motor.speed_target_rad_s; encoder.calib_flag=ENC_CALIB_LINEARIZED;
    startup.speed_feedback_rad_s=motor.speed_target_rad_s;
    current_calls=stop_calls=0; pwm_enabled=true;
}
static void tick(void) { SensorlessStartup_Run(&foc,&motor,&pi,&observer,&startup,cfg); }
static void enter_handoff(float direction, float phase_delta) {
    setup(); motor.speed_target_rad_s*=direction; motor.speed_ramped_rad_s=motor.speed_target_rad_s;
    observer.electrical_velocity_rad_s*=direction;
    observer.electrical_angle_rad=6.27f;
    startup.state=SENSORLESS_STARTUP_SPEED_LOCK;
    startup.direction=direction;
    startup.open_loop_velocity_rad_s=observer.electrical_velocity_rad_s;
    startup.open_loop_angle_rad=Math_NormalizeAngle(observer.electrical_angle_rad+phase_delta-observer.electrical_velocity_rad_s*Current_Ts);
    startup.lock_ticks=(uint32_t)(cfg->speed_lock_time_s/Current_Ts)-1U;
    tick(); assert(startup.state==SENSORLESS_STARTUP_HANDOFF);
}
static void check_handoff(float direction) {
    float delta=-direction*.7f;
    enter_handoff(direction,delta);
    float iq_before=applied_observer_iq;
    assert(fabsf(pi.output*motor.application.current_limit_a-iq_before)<.002f);
    assert(fabsf(startup.speed_feedback_rad_s-motor.speed_target_rad_s)<.0001f);
    /* Keep a constant observer velocity: changing coordinate frame alone must
     * not turn the fixed open-loop iq_a into a new torque demand. */
    float max_error=0;
    unsigned steps=0;
    while(startup.state==SENSORLESS_STARTUP_HANDOFF) {
        observer.electrical_angle_rad=Math_NormalizeAngle(observer.electrical_angle_rad+observer.electrical_velocity_rad_s*Current_Ts);
        tick();
        max_error=fmaxf(max_error,fabsf(applied_observer_iq-iq_before));
        assert(fabsf(applied_observer_iq-pi.output*motor.application.current_limit_a)<.005f);
        assert(++steps<10000);
    }
    assert(max_error<.005f && motor.fault_code==MOTOR_FAULT_NONE);
    assert(fabsf(motor.id_target_a-cfg->startup_id_a)<.001f);
    float last_iq=motor.iq_target_a, last_ui=pi.integral_output;
    tick(); assert(fabsf(motor.iq_target_a-last_iq)<.0001f && fabsf(pi.integral_output-last_ui)<.0001f);

    /* An overspeed must already reduce torque in state 4, not wait 100 ms. */
    enter_handoff(direction,delta); iq_before=pi.output*motor.application.current_limit_a;
    observer.electrical_velocity_rad_s*=1.2f;
    for(unsigned i=0;i<100;++i) tick();
    assert(startup.state==SENSORLESS_STARTUP_HANDOFF);
    assert(direction*(pi.output*motor.application.current_limit_a-iq_before)<-.05f);
}
static void check_voltage_rotation(void) {
    enter_handoff(1,-.7f);
    foc.id_pi.integral_output=2; foc.iq_pi.integral_output=3;
    float initial_offset=startup.handoff_phase_delta_rad;
    float d=2*cosf(initial_offset)-3*sinf(initial_offset);
    float q=2*sinf(initial_offset)+3*cosf(initial_offset);
    while(startup.state==SENSORLESS_STARTUP_HANDOFF) tick();
    assert(fabsf(foc.id_pi.integral_output-d)<.04f && fabsf(foc.iq_pi.integral_output-q)<.04f);
}
static void check_braking_cap(void) {
    setup(); startup.state=SENSORLESS_STARTUP_CLOSED_LOOP;
    startup.speed_pi_output_max=0;
    pi.output=pi.integral_output=.5f; /* A previous positive PI output cannot leak between ticks. */
    tick(); assert(motor.iq_target_a<=0);
    for(unsigned i=0;i<100;++i) tick();
    assert(pi.output_max==0 && motor.iq_target_a<=0);
    startup.speed_pi_output_max=1;
    for(unsigned i=0;i<10;++i) tick();
    assert(motor.iq_target_a>0); /* Ordinary mode retains its positive torque authority. */
}
int main(void) {
    check_handoff(1); check_handoff(-1); check_voltage_rotation();
    check_braking_cap();
    puts("PASS observer-frame current continuity, state-4 speed feedback, PI cap, braking authority");
}
"""


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--cc", required=True)
    ap.add_argument("--out", type=Path, default=ROOT / "outputs/sensorless_transition_tests")
    a = ap.parse_args()
    out = a.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    (out / "main.h").write_text("#include <stdint.h>\n#include <stddef.h>\n")
    source = (ROOT / "firmware/motor/foc/foc_sensorless_run.c").read_text(encoding="utf-8")
    run = source.split("/* SENSORLESS_RUNTIME_BEGIN", 1)[1].split(
        "/* SENSORLESS_RUNTIME_END */", 1
    )[0]
    run = "/* SENSORLESS_RUNTIME_BEGIN" + run
    reset = function_source(
        (ROOT / "firmware/motor/foc/foc_sensorless_run.c").read_text(encoding="utf-8"),
        "SensorlessStartup_Reset",
    )
    ramp = function_source(
        (ROOT / "firmware/motor/foc/foc_speed.c").read_text(encoding="utf-8"),
        "MotorAxis_UpdateSpeedRamp",
    )
    fixture = PRELUDE + reset + ramp + run + CASES
    source = out / "transitions.c"
    source.write_text(fixture, encoding="utf-8")
    compiler = [a.cc] + (["cc"] if Path(a.cc).stem == "zig" else [])
    logs = []
    for damping in (0, 1):
        exe = out / f"transitions_{damping}.exe"
        cmd = (
            compiler
            + ["-I", str(out)]
            + NATIVE_INCLUDE_FLAGS
            + [
                "-std=c99",
                "-O1",
                "-UNDEBUG",
                "-Wall",
                "-Wextra",
                "-Werror",
                "-Wno-unused-function",
                "-DMOTOR_HAS_DAMPING_RING=" + str(damping),
                str(source),
                str(ROOT / "firmware/motor/foc/foc_pid.c"),
                str(ROOT / "firmware/common/utils.c"),
                "-o",
                str(exe),
                "-lm",
            ]
        )
        for command in (cmd, [str(exe)]):
            r = subprocess.run(
                command, capture_output=True, text=True, encoding="utf-8", errors="replace"
            )
            logs.append(r.stdout + r.stderr)
            print(logs[-1], end="")
            (out / "test.log").write_text("\n".join(logs), encoding="utf-8")
            r.check_returncode()


if __name__ == "__main__":
    main()
