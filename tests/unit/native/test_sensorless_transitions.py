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
#include "foc_run.h"
#include "foc_sensorless_run.h"
#include "foc_speed.h"
#include "hw_conf.h"
#include "motor_startup_profile.h"
#include "utils.h"
static MotorControl_TypeDef motor;
static FOC_TypeDef foc;
static PI_Controller_TypeDef pi;
static Fluxobserver_TypeDef observer;
static SensorlessStartup_TypeDef startup;
static Encoder_TypeDef encoder;
static unsigned current_calls, stop_calls;
static bool pwm_enabled;
static float applied_observer_iq, applied_observer_id, applied_phase, applied_velocity;
void Set_ErrorNow(ErrorNow_TypeDef e) { motor.ErrorNow=e; }
void Set_ModeNow(ModeNow_TypeDef m) {
    if(m==Save_Param) assert(!pwm_enabled && stop_calls);
    motor.ModeNow=m;
}
void Stop_PWM_Generate(void) { pwm_enabled=false; ++stop_calls; }
void FOC_CurrentController_Reset(FOC_TypeDef *f) {
    PI_Controller_Reset(&f->id_pi); PI_Controller_Reset(&f->iq_pi);
}
void FOC_Current(FOC_TypeDef *f, MotorControl_TypeDef *m, float phase, float velocity) {
    (void)f;
    float offset=phase-observer.theta_e;
    applied_observer_iq=m->idRef*sinf(offset)+m->iqRef*cosf(offset);
    applied_observer_id=m->idRef*cosf(offset)-m->iqRef*sinf(offset);
    applied_phase=phase; applied_velocity=velocity; ++current_calls;
}
void Fluxobserver_ParamInit(Fluxobserver_TypeDef *o) { memset(o,0,sizeof(*o)); }
float Observer_GetElePhase(Fluxobserver_TypeDef *o) { return o->theta_e; }
float Observer_GetEleVel(Fluxobserver_TypeDef *o) { return o->omega_e; }
float Observer_GetElePosition(Fluxobserver_TypeDef *o) { return o->theta_e_unwrapped; }
uint32_t Observer_GetPositionEpoch(Fluxobserver_TypeDef *o) { return o->position_epoch; }
bool Encoder_IsOnline(const Encoder_TypeDef *e) { return e->has_valid_sample; }
float Encoder_GetMecVelContinuous(const Encoder_TypeDef *e) { return e->vel_mech_continuous; }
void Encoder_ResetVelocity(Encoder_TypeDef *e) { e->velocity_ready=false; }
static void *HEAP_malloc(size_t n) { return malloc(n); }
static void HEAP_free(void *p) { free(p); }
"""

CASES = r"""
/* 编码器标定配置已随 FOC-Calibration 功能移除：夹具内联同参配置供无感用例使用。 */
static const SensorlessStartupConfig_TypeDef test_startup_config = {
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
static const SensorlessStartupConfig_TypeDef *cfg=&test_startup_config;
static void setup(void) {
    memset(&motor,0,sizeof(motor)); memset(&foc,0,sizeof(foc));
    memset(&observer,0,sizeof(observer)); memset(&encoder,0,sizeof(encoder));
    PI_Controller_Reset(&pi); SensorlessStartup_Reset(&startup);
    motor.motor_pole_pairs=21; motor.motor_phase_resistance=1.905f;
    motor.motor_d_inductance=motor.motor_q_inductance=.001635f;
    motor.motor_flux=.0175025f; motor.current_limit=6;
    motor.speed_Kp=.02f; motor.speed_Ki=.5f;
    motor.speedAcc=motor.speedDec=314.159f;
    motor.ModeNow=Calib_EncoderObserver;
    motor.speedRef=cfg->target_electrical_velocity_rad_s/21;
    motor.speedShadow=motor.speedRef;
    observer.omega_e=cfg->target_electrical_velocity_rad_s;
    encoder.has_valid_sample=true; encoder.velocity_ready=true;
    encoder.vel_mech_continuous=motor.speedRef; encoder.calib_flag=ENC_CALIB_LINEARIZED;
    startup.speed_feedback=motor.speedRef;
    current_calls=stop_calls=0; pwm_enabled=true;
}
static void tick(void) { SensorlessStartup_Run(&foc,&motor,&pi,&observer,&startup,cfg); }
static void enter_handoff(float direction, float phase_delta) {
    setup(); motor.speedRef*=direction; motor.speedShadow=motor.speedRef;
    observer.omega_e*=direction;
    observer.theta_e=6.27f;
    startup.state=SENSORLESS_STARTUP_SPEED_LOCK;
    startup.direction=direction;
    startup.open_loop_omega=observer.omega_e;
    startup.open_loop_theta=normalizeAngle(observer.theta_e+phase_delta-observer.omega_e*Current_Ts);
    startup.lock_ticks=(uint32_t)(cfg->speed_lock_time_s/Current_Ts)-1U;
    tick(); assert(startup.state==SENSORLESS_STARTUP_HANDOFF);
}
static void check_handoff(float direction) {
    float delta=-direction*.7f;
    enter_handoff(direction,delta);
    float iq_before=applied_observer_iq;
    assert(fabsf(pi.Out*motor.current_limit-iq_before)<.002f);
    assert(fabsf(startup.speed_feedback-motor.speedRef)<.0001f);
    /* Keep a constant observer velocity: changing coordinate frame alone must
     * not turn the fixed open-loop Iq into a new torque demand. */
    float max_error=0;
    unsigned steps=0;
    while(startup.state==SENSORLESS_STARTUP_HANDOFF) {
        observer.theta_e=normalizeAngle(observer.theta_e+observer.omega_e*Current_Ts);
        tick();
        max_error=fmaxf(max_error,fabsf(applied_observer_iq-iq_before));
        assert(fabsf(applied_observer_iq-pi.Out*motor.current_limit)<.005f);
        assert(++steps<10000);
    }
    assert(max_error<.005f && motor.ErrorNow==No_Error);
    assert(fabsf(motor.idRef-cfg->startup_id_a)<.001f);
    float last_iq=motor.iqRef, last_ui=pi.Ui;
    tick(); assert(fabsf(motor.iqRef-last_iq)<.0001f && fabsf(pi.Ui-last_ui)<.0001f);

    /* An overspeed must already reduce torque in state 4, not wait 100 ms. */
    enter_handoff(direction,delta); iq_before=pi.Out*motor.current_limit;
    observer.omega_e*=1.2f;
    for(unsigned i=0;i<100;++i) tick();
    assert(startup.state==SENSORLESS_STARTUP_HANDOFF);
    assert(direction*(pi.Out*motor.current_limit-iq_before)<-.05f);
}
static void check_voltage_rotation(void) {
    enter_handoff(1,-.7f);
    foc.id_pi.Ui=2; foc.iq_pi.Ui=3;
    float initial_offset=startup.handoff_phase_delta;
    float d=2*cosf(initial_offset)-3*sinf(initial_offset);
    float q=2*sinf(initial_offset)+3*cosf(initial_offset);
    while(startup.state==SENSORLESS_STARTUP_HANDOFF) tick();
    assert(fabsf(foc.id_pi.Ui-d)<.04f && fabsf(foc.iq_pi.Ui-q)<.04f);
}
static void check_braking_cap(void) {
    setup(); startup.state=SENSORLESS_STARTUP_CLOSED_LOOP;
    startup.speed_pi_output_max=0;
    pi.Out=pi.Ui=.5f; /* A previous positive PI output cannot leak between ticks. */
    tick(); assert(motor.iqRef<=0);
    for(unsigned i=0;i<100;++i) tick();
    assert(pi.Umax==0 && motor.iqRef<=0);
    startup.speed_pi_output_max=1;
    for(unsigned i=0;i<10;++i) tick();
    assert(motor.iqRef>0); /* Ordinary mode retains its positive torque authority. */
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
        "MotorControl_UpdateSpeedRamp",
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
