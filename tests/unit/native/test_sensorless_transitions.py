"""Exercise production handoff and calibration-stop code without motor hardware.

Checks current-vector continuity, active speed feedback during handoff, braking
limits, and stop-before-save ordering. This is not a physical-plant acceptance test.
"""
import argparse
from pathlib import Path
import subprocess
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'tools'))
from project_paths import ROOT, NATIVE_INCLUDE_FLAGS
from run_position_servo_tests import function_source

PRELUDE = r'''
#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>
#include "foc_calibration.h"
#include "foc_run.h"
#include "foc_param_profile.h"
#include "hw_conf.h"
#include "utils.h"
#include "motor_thermal.h"
static MotorControl_TypeDef motor;
static FOC_TypeDef foc;
static PI_Controller_TypeDef pi;
static Fluxobserver_TypeDef observer;
static SensorlessStartup_TypeDef startup;
static Encoder_TypeDef encoder;
static unsigned current_calls, voltage_calls, stop_calls, high_side_calls, save_calls;
static bool pwm_enabled, watch_lut_commit;
static bool samples_released(void);
static Encoder_TypeDef original_encoder;
static float applied_observer_iq, applied_observer_id, applied_phase, applied_velocity;
static void check_old_calibration(void) {
    assert(memcmp(encoder.linearization_lut_q15,original_encoder.linearization_lut_q15,
        sizeof(encoder.linearization_lut_q15))==0);
    assert(encoder.electrical_zero_q15==original_encoder.electrical_zero_q15);
    assert(encoder.mechanical_zero_q15==original_encoder.mechanical_zero_q15);
    assert(encoder.mechanical_zero_shadow_q15==original_encoder.mechanical_zero_shadow_q15);
    assert(encoder.calib_flag==original_encoder.calib_flag);
}
void Set_ErrorNow(ErrorNow_TypeDef e) { motor.ErrorNow=e; }
void Set_ModeNow(ModeNow_TypeDef m) {
    if(m==Save_Param && motor.ModeNow==Calib_EncoderObserver) {
        assert(!pwm_enabled && stop_calls);
        if(watch_lut_commit) {
            for(unsigned i=0;i<ENCODER_OFFSET_LUT_SIZE;++i) {
                assert(encoder.linearization_lut_q15[i]==37);
            }
            assert(encoder.electrical_zero_q15==0 && encoder.mechanical_zero_q15==0);
            assert(encoder.mechanical_zero_shadow_q15==0 && samples_released());
            assert(encoder.calib_flag==ENC_CALIB_LINEARIZED);
        }
    }
    if(m==Save_Param) ++save_calls;
    motor.ModeNow=m;
}
void Stop_PWM_Generate(void) {
    if(watch_lut_commit) check_old_calibration();
    pwm_enabled=false; ++stop_calls;
}
void PWM_TurnOnHighSides(void) { ++high_side_calls; }
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
void FOC_Voltage(FOC_TypeDef *f, float vd, float vq, float phase) {
    (void)f; (void)vd; (void)vq; (void)phase; ++voltage_calls;
}
void Fluxobserver_ParamInit(Fluxobserver_TypeDef *o) { memset(o,0,sizeof(*o)); }
float Observer_GetElePhase(Fluxobserver_TypeDef *o) { return o->theta_e; }
float Observer_GetEleVel(Fluxobserver_TypeDef *o) { return o->omega_e; }
float Observer_GetElePosition(Fluxobserver_TypeDef *o) { return o->theta_e_unwrapped; }
uint32_t Observer_GetPositionEpoch(Fluxobserver_TypeDef *o) { return o->position_epoch; }
bool Encoder_IsOnline(const Encoder_TypeDef *e) { return e->has_valid_sample; }
float Encoder_GetMecVelContinuous(const Encoder_TypeDef *e) { return e->vel_mech_continuous; }
void Encoder_ResetVelocity(Encoder_TypeDef *e) {
    if(watch_lut_commit) assert(!pwm_enabled && stop_calls);
    e->velocity_ready=false;
}
'''

CASES = r'''
static uint32_t active_flags;
static const SensorlessStartupConfig_TypeDef *cfg;
static bool samples_released(void) {
    return !p_error_sum && !calibration_samples && !candidate_linearization_lut;
}
static void poison_workspace(void) {
    /* 固定区不会自动清除；显式毒化验证首次使用及复用都不依赖 BSS 初值。 */
    memset(calibration_error_sum_storage,0xA5,sizeof(calibration_error_sum_storage));
    memset(calibration_samples_storage,0xA5,sizeof(calibration_samples_storage));
    memset(calibration_lut_storage,0xA5,sizeof(calibration_lut_storage));
}
static void setup(void) {
    watch_lut_commit=false;
    memset(&motor,0,sizeof(motor)); memset(&foc,0,sizeof(foc));
    memset(&observer,0,sizeof(observer)); memset(&encoder,0,sizeof(encoder));
    PI_Controller_Reset(&pi); SensorlessStartup_Reset(&startup); EncoderCalibration_Reset();
    poison_workspace();
    motor.motor_pole_pairs=21; motor.motor_phase_resistance=1.905f;
    motor.motor_d_inductance=motor.motor_q_inductance=.001635f;
    motor.motor_flux=.0175025f;
    /* Use the production default: the encoder-calibration profile refuses to
     * start when the configured limit cannot deliver its own startup vector. */
    motor.load_profile_flags=active_flags;
    motor.current_limit=8.0f;
    motor.speed_Kp=.02f; motor.speed_Ki=.5f;
    motor.speedAcc=motor.speedDec=314.159f;
    motor.ModeNow=Calib_EncoderObserver;
    motor.speedRef=cfg->target_electrical_velocity_rad_s/21;
    motor.speedShadow=motor.speedRef;
    observer.omega_e=cfg->target_electrical_velocity_rad_s;
    encoder.has_valid_sample=true; encoder.velocity_ready=true;
    encoder.vel_mech_continuous=motor.speedRef; encoder.calib_flag=ENC_CALIB_LINEARIZED;
    startup.speed_feedback=motor.speedRef;
    current_calls=voltage_calls=stop_calls=high_side_calls=save_calls=0; pwm_enabled=true;
}
static void tick(void) { Task_Sensorless_Speed_Mode(&foc,&motor,&pi,&observer,&startup,cfg); }
static void calibration_tick(void) { Task_Calib_EncoderObserver(&foc,&motor,&pi,&encoder,&observer,&startup); }
static void prepare_calibration_power(void) {
    /* 调度契约：准备与启用各占一拍，启用拍任务仍检查守卫但不计算 FOC。 */
    pwm_enabled=false;
    assert(!EncoderObserverCalibration_IsPowerStartReady());
    calibration_tick();
    assert(EncoderObserverCalibration_IsPowerStartReady() && current_calls==0);
    assert(startup.state==SENSORLESS_STARTUP_IDLE && startup.state_ticks==0);
    assert(!samples_released() && motor.ErrorNow==No_Error && !pwm_enabled);
    calibration_tick();
    assert(!EncoderObserverCalibration_IsPowerStartReady() && CalibStep==CS_OBS_ALIGN_ORIGIN);
    assert(startup.state==SENSORLESS_STARTUP_IDLE && startup.state_ticks==0);
    assert(current_calls==0 && motor.ErrorNow==No_Error);
    pwm_enabled=true;
}
static void prepare_pending_candidate(void) {
    setup(); calibration_tick();
    assert(candidate_linearization_lut && p_error_sum && calibration_samples);
    for(unsigned i=0;i<ENCODER_OFFSET_LUT_SIZE;++i) {
        encoder.linearization_lut_q15[i]=(int16_t)(i%17-8);
        candidate_linearization_lut[i]=37;
    }
    encoder.electrical_zero_q15=1234; encoder.mechanical_zero_q15=4321;
    encoder.mechanical_zero_shadow_q15=54321; encoder.calib_flag=ENC_CALIB_ALL;
    original_encoder=encoder; watch_lut_commit=true;
    startup.state=SENSORLESS_STARTUP_CLOSED_LOOP; startup.direction=1;
    startup.speed_feedback=cfg->target_electrical_velocity_rad_s/21;
    startup.speed_pi_output_max=1;
    observer.omega_e=cfg->target_electrical_velocity_rad_s;
    motor.speedShadow=startup.speed_feedback;
    current_calls=stop_calls=high_side_calls=save_calls=0;
}
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
static void check_startup_configuration_guards(void) {
    SensorlessStartupConfig_TypeDef invalid=*cfg;
    /* 两个生产档均从 IDLE 正常进入对齐，并能按既定时长转入开环。 */
    setup(); tick();
    assert(motor.ErrorNow==No_Error && startup.state==SENSORLESS_STARTUP_ALIGN);
    assert(current_calls==1 && motor.idRef>0 && motor.iqRef==0);
    startup.state_ticks=(uint32_t)((cfg->align_current_ramp_time_s+
        cfg->align_hold_time_s)/Current_Ts)-1U;
    tick();
    assert(motor.ErrorNow==No_Error && startup.state==SENSORLESS_STARTUP_OPEN_LOOP);

    /* 只读档位的非法时间和跨字段速度约束必须在入场时拒绝。 */
    setup(); invalid.angle_handoff_time_s=0;
    Task_Sensorless_Speed_Mode(&foc,&motor,&pi,&observer,&startup,&invalid);
    assert(motor.ErrorNow==MotorParam_Error && current_calls==0 &&
        startup.state==SENSORLESS_STARTUP_IDLE);
    setup(); invalid=*cfg;
    invalid.minimum_electrical_velocity_rad_s=invalid.target_electrical_velocity_rad_s+1;
    Task_Sensorless_Speed_Mode(&foc,&motor,&pi,&observer,&startup,&invalid);
    assert(motor.ErrorNow==MotorParam_Error && current_calls==0 &&
        startup.state==SENSORLESS_STARTUP_IDLE);

    setup();
    Task_Sensorless_Speed_Mode(&foc,&motor,&pi,&observer,&startup,NULL);
    assert(motor.ErrorNow==MotorParam_Error && current_calls==0);
    enter_handoff(1,-.7f); current_calls=0;
    Task_Sensorless_Speed_Mode(&foc,&motor,&pi,&observer,&startup,NULL);
    assert(motor.ErrorNow==MotorParam_Error && current_calls==0);
}
static void check_live_model_guards(void) {
    float *live_values[]={&motor.motor_phase_resistance,&motor.motor_d_inductance,
        &motor.motor_q_inductance,&motor.motor_flux,&motor.current_limit};
    /* 入场优化不能缓存可变模型/限流的有效性；交接中修改后下一拍即拒绝。 */
    for(unsigned i=0;i<sizeof(live_values)/sizeof(live_values[0]);++i) {
        enter_handoff(1,-.7f); current_calls=0;
        *live_values[i]=0;
        tick(); assert(motor.ErrorNow==MotorParam_Error && current_calls==0);
    }
    enter_handoff(1,-.7f); current_calls=0;
    motor.motor_pole_pairs=0;
    tick(); assert(motor.ErrorNow==MotorParam_Error && current_calls==0);
    enter_handoff(1,-.7f); current_calls=0;
    motor.current_limit=cfg->align_current_a*.5f;
    tick(); assert(motor.ErrorNow==Sensorless_Error && current_calls==0);
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
static void check_handoff_trig_cache(void) {
    const SensorlessStartupConfig_TypeDef *production_cfg=cfg;
    const float durations[]={.0001f,.1f,.100025f};
    float maximum_vector_error=0;
    for(unsigned duration=0;duration<sizeof(durations)/sizeof(durations[0]);++duration) {
        SensorlessStartupConfig_TypeDef config=*production_cfg;
        config.angle_handoff_time_s=durations[duration]; cfg=&config;
        for(unsigned point=0;point<=64;++point) {
            float delta=-_PI+2.0f*_PI*(float)point/64.0f;
            float direction=point&1U ? -1.0f : 1.0f;
            enter_handoff(direction,delta);
            assert(startup.handoff_phase_sin==sinf(startup.handoff_phase_delta));
            assert(startup.handoff_phase_cos==cosf(startup.handoff_phase_delta));
            unsigned ticks=0;
            while(startup.state==SENSORLESS_STARTUP_HANDOFF) {
                float blend=constrain((float)(startup.state_ticks+1U)*Current_Ts/
                    cfg->angle_handoff_time_s,0,1);
                float phase=(1-blend)*startup.handoff_phase_delta;
                tick();
                float ds=startup.handoff_phase_sin-sinf(phase);
                float dc=startup.handoff_phase_cos-cosf(phase);
                float error=sqrtf(ds*ds+dc*dc);
                maximum_vector_error=fmaxf(maximum_vector_error,error);
                /* 8 A 向量的交接误差小于 1 mA，覆盖大步长回退和非整拍结束。 */
                assert(8.0f*error<.001f);
                assert(++ticks<2100);
            }
            assert(startup.handoff_phase_sin==0 && startup.handoff_phase_cos==1);
            assert(motor.ErrorNow==No_Error);
        }
    }
    cfg=production_cfg;
    enter_handoff(1,-.7f); tick();
    motor.speedRef=0; tick();
    assert(startup.state==SENSORLESS_STARTUP_IDLE &&
        startup.handoff_phase_sin==0 && startup.handoff_phase_cos==1);
    startup.handoff_phase_sin=NAN; startup.handoff_phase_cos=NAN;
    SensorlessStartup_Reset(&startup);
    assert(startup.handoff_phase_sin==0 && startup.handoff_phase_cos==1);
    enter_handoff(-1,1.2f); tick();
    assert(motor.ErrorNow==No_Error && isfinite(motor.iqRef));
    printf("PASS handoff trig cache: max 8 A vector error %.9g A\n",
        (double)(8.0f*maximum_vector_error));
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
static void check_speed_limit_math(void) {
    const float caps[]={0,1,.4f,NAN,INFINITY,-INFINITY};
    const float limits[]={8,3,0,NAN,INFINITY};
    const float outputs[]={-2,-.1f,0,.3f,2,NAN};
    /* 直接调用快路径，覆盖快拍/PI 拍、动态热限幅和 fminf 的单/双 NaN 语义。 */
    for(unsigned c=0;c<sizeof(caps)/sizeof(caps[0]);++c) {
        for(unsigned l=0;l<sizeof(limits)/sizeof(limits[0]);++l) {
            for(unsigned o=0;o<sizeof(outputs)/sizeof(outputs[0]);++o) {
                for(unsigned slow=0;slow<2;++slow) {
                    setup(); motor.thermal_limit_active=true;
                    motor.current_limit=limits[l]; motor.thermal_current_limit_a=2;
                    startup.speed_pi_output_max=caps[c];
                    startup.speed_loop_ticks=slow ? SPEED_LOOP_DIVIDER-1U : 0U;
                    pi.Out=pi.Ui=outputs[o];
                    float ratio=MotorThermal_ResolveLimit(motor.current_limit,2,true)/motor.current_limit;
                    float upper=fminf(ratio,caps[c]);
                    float actual=Sensorless_UpdateSpeedLoop(&motor,&pi,&startup,
                        motor.speedRef*motor.motor_pole_pairs,motor.motor_pole_pairs);
                    float expected=constrain(pi.Out,-ratio,upper)*motor.current_limit;
                    assert((isnan(actual) && isnan(expected)) || actual==expected);
                    if(slow) assert((isnan(pi.Umax) && isnan(upper)) || pi.Umax==upper);
                }
            }
        }
    }
}
static float reference_speed_tick(MotorControl_TypeDef *m, PI_Controller_TypeDef *p,
    SensorlessStartup_TypeDef *s, float mechanical_speed) {
    float ratio=MotorThermal_ResolveLimit(m->current_limit,m->thermal_current_limit_a,
        m->thermal_limit_active)/m->current_limit;
    if(++s->speed_loop_ticks>=SPEED_LOOP_DIVIDER) {
        s->speed_feedback+=SENSORLESS_SPEED_FEEDBACK_LPF_ALPHA*(mechanical_speed-s->speed_feedback);
        Sensorless_UpdateSpeedReference(m);
        PI_Controller_Configure(p,m->speed_Kp,m->speed_Ki,Speed_Ts,-ratio,
            fminf(ratio,s->speed_pi_output_max));
        PI_Controller_Run(p,m->speedShadow,s->speed_feedback);
        if(m->thermal_limit_active) PI_Controller_TrackOutput(p,p->Out);
        s->speed_loop_ticks=0;
    }
    return constrain(p->Out,-ratio,fminf(ratio,s->speed_pi_output_max))*m->current_limit;
}
static void check_speed_tick_equivalence(void) {
    setup();
    MotorControl_TypeDef expected_motor=motor;
    PI_Controller_TypeDef expected_pi=pi;
    SensorlessStartup_TypeDef expected_startup=startup;
    /* 电角→机械角速度除法移入慢拍后，反馈/PI/斜坡/输出逐位不变。 */
    for(unsigned i=0;i<20000;++i) {
        float electrical_speed=315.0f*cosf((float)i*.003f);
        motor.motor_pole_pairs=expected_motor.motor_pole_pairs=i%3 ? 21:14;
        motor.speedRef=expected_motor.speedRef=i%2000<1000 ? 15:-15;
        motor.thermal_limit_active=expected_motor.thermal_limit_active=i%7!=0;
        motor.thermal_current_limit_a=expected_motor.thermal_current_limit_a=i%100<50 ? 2:8;
        startup.speed_pi_output_max=expected_startup.speed_pi_output_max=i%1000<500 ? 0:1;
        float expected=reference_speed_tick(&expected_motor,&expected_pi,&expected_startup,
            electrical_speed/(float)motor.motor_pole_pairs);
        float actual=Sensorless_UpdateSpeedLoop(&motor,&pi,&startup,electrical_speed,
            (float)motor.motor_pole_pairs);
        assert(memcmp(&actual,&expected,sizeof(actual))==0);
        assert(memcmp(&motor,&expected_motor,sizeof(motor))==0);
        assert(memcmp(&pi,&expected_pi,sizeof(pi))==0);
        assert(memcmp(&startup,&expected_startup,sizeof(startup))==0);
    }
}
static void check_id_ramp_equivalence(void) {
    const SensorlessStartupConfig_TypeDef *production_cfg=cfg;
    const float durations[]={.5f,.500025f};
    for(unsigned i=0;i<sizeof(durations)/sizeof(durations[0]);++i) {
        SensorlessStartupConfig_TypeDef config=*production_cfg;
        config.id_ramp_down_time_s=durations[i]; cfg=&config;
        setup(); startup.state=SENSORLESS_STARTUP_CLOSED_LOOP;
        unsigned expected_ticks=0;
        for(unsigned step=0;step<10020;++step) {
            float expected_id=cfg->startup_id_a*(1.0f-constrain(
                (float)expected_ticks*Current_Ts/cfg->id_ramp_down_time_s,0,1));
            if(expected_ticks<(uint32_t)(cfg->id_ramp_down_time_s/Current_Ts)) ++expected_ticks;
            tick();
            assert(motor.ErrorNow==No_Error && startup.id_ramp_ticks==expected_ticks);
            assert(memcmp(&motor.idRef,&expected_id,sizeof(expected_id))==0);
        }
        assert(startup.id_ramp_ticks==10000);
        assert(i==0 ? motor.idRef==0 : motor.idRef>0);
    }
    cfg=production_cfg;
}
static void expect_coast(CalibStep_TyepeDef step, float obs_speed, float enc_speed,
                         bool ready, bool tracking) {
    prepare_pending_candidate(); CalibStep=step;
    observer.omega_e=obs_speed; encoder.vel_mech_continuous=enc_speed;
    encoder.velocity_ready=ready;
    if(!tracking) ++observer.position_epoch;
    motor.iqRef=-.12f; pi.Ui=-.02f;
    calibration_tick();
    assert(motor.ModeNow==Calib_EncoderObserver && CalibStep==CS_OBS_COMMIT);
    assert(!pwm_enabled && stop_calls==1 && current_calls==0 && high_side_calls==0);
    assert(motor.idRef==0 && motor.iqRef==0 && pi.Ui==0);
    assert(!samples_released() && save_calls==0); check_old_calibration();
    calibration_tick();
    assert(motor.ModeNow==Save_Param && CalibStep==CS_NULL);
    assert(!pwm_enabled && stop_calls==1 && current_calls==0 && samples_released());
    assert(encoder.reverse==0 && encoder.calib_flag==ENC_CALIB_LINEARIZED);
}
static void check_calibration_stop(void) {
    float minimum=cfg->minimum_electrical_velocity_rad_s;
    for(unsigned step=CS_OBS_STOP_DECEL;step<=CS_OBS_STOP_CURRENT;++step) {
        expect_coast((CalibStep_TyepeDef)step,minimum,15,true,true);
        expect_coast((CalibStep_TyepeDef)step,-100,15,true,true);
        expect_coast((CalibStep_TyepeDef)step,NAN,15,true,true);
        expect_coast((CalibStep_TyepeDef)step,INFINITY,15,true,true);
        expect_coast((CalibStep_TyepeDef)step,315,-1,true,true);
        expect_coast((CalibStep_TyepeDef)step,315,NAN,true,true);
        expect_coast((CalibStep_TyepeDef)step,315,INFINITY,true,true);
        expect_coast((CalibStep_TyepeDef)step,315,15,true,false);
    }
    prepare_pending_candidate(); CalibStep=CS_OBS_STOP_CURRENT;
    observer.theta_e=NAN; calibration_tick();
    assert(CalibStep==CS_OBS_COMMIT && !pwm_enabled && current_calls==0);
    calibration_tick();
    assert(motor.ModeNow==Save_Param && !pwm_enabled && current_calls==0);
    prepare_pending_candidate(); CalibStep=CS_OBS_STOP_CURRENT;
    encoder.velocity_ready=false; /* 测速未就绪时仍保留观测器的独立截止条件。 */
    calibration_tick(); assert(current_calls==1 && pwm_enabled && stop_calls==0);
    /* Replay a deceleration through zero with an otherwise frozen/stale state.
     * PWM must be cut while speed is still positive, before the sign crossing. */
    for(float speed=14;speed>=-8 && motor.ModeNow!=Save_Param;speed-=.02f) {
        observer.omega_e=speed*21; encoder.vel_mech_continuous=speed;
        calibration_tick();
        if(motor.ModeNow==Save_Param) assert(speed>0);
    }
    assert(motor.ModeNow==Save_Param && !pwm_enabled);
}

static void enter_verified_candidate(void) {
    const float mechanical_turn= _2PI*21;
    const float verify_travel=mechanical_turn*SENSORLESS_ENCODER_CALIB_VERIFY_MECH_TURNS;
    prepare_pending_candidate(); CalibStep=CS_OBS_VERIFY_CW;
    /* 注入理想观测角和候选补偿值，遍历验证段；不模拟电机物理响应。 */
    for(unsigned i=1;i<=4097 && CalibStep==CS_OBS_VERIFY_CW;++i) {
        observer.theta_e_unwrapped=verify_travel*(float)i/4096;
        observer.theta_e=normalizeAngle(observer.theta_e_unwrapped);
        encoder.directed_q15=(uint16_t)((uint16_t)(observer.theta_e_unwrapped*
            ((float)ENCODER_Q15_CPR/mechanical_turn))+37U);
        calibration_tick();
        check_old_calibration();
        assert(pwm_enabled && stop_calls==0 && save_calls==0 && !samples_released());
    }
    assert(CalibStep==CS_OBS_STOP_DECEL && motor.ErrorNow==No_Error);
    assert(candidate_linearization_lut && p_error_sum && calibration_samples);
}
static void check_clear_prefix(unsigned cleared_bins) {
    assert(p_error_sum && calibration_samples && candidate_linearization_lut);
    assert(p_error_sum==calibration_error_sum_storage && calibration_samples==calibration_samples_storage);
    assert(candidate_linearization_lut==calibration_lut_storage);
    for(unsigned i=0;i<ENCODER_OFFSET_LUT_SIZE;++i) {
        assert(p_error_sum[i]==(i<cleared_bins ? 0 : (int32_t)0xA5A5A5A5));
        assert(calibration_samples[i]==(i<cleared_bins ? 0 : (uint16_t)0xA5A5));
        assert(candidate_linearization_lut[i]==(int16_t)0xA5A5);
    }
}
static void check_alignment_command(unsigned ticks) {
    float expected=cfg->align_current_a*constrain((float)ticks*Current_Ts/
        cfg->align_current_ramp_time_s,0,1);
    assert(memcmp(&motor.idRef,&expected,sizeof(expected))==0);
    assert(motor.iqRef==0 && applied_phase==0 && applied_velocity==0);
    assert(current_calls==ticks && motor.ErrorNow==No_Error);
}
static void check_candidate_initialization(void) {
    setup();
    foc.id_pi.Ref=foc.id_pi.Fbk=foc.id_pi.Err=foc.id_pi.Up=foc.id_pi.Ui=foc.id_pi.Out=7;
    foc.iq_pi=foc.id_pi; pi=foc.id_pi;
    prepare_calibration_power();
    check_clear_prefix(0);
    calibration_tick();
    assert(motor.ErrorNow==No_Error && CalibStep==CS_OBS_ALIGN_ORIGIN);
    assert(startup.state==SENSORLESS_STARTUP_ALIGN && startup.state_ticks==1);
    assert(current_calls==1 && applied_phase==0 && applied_velocity==0);
    assert(motor.iqRef==0 && fabsf(motor.idRef-
        cfg->align_current_a*Current_Ts/cfg->align_current_ramp_time_s)<1e-7f);
    PI_Controller_TypeDef *controllers[]={&foc.id_pi,&foc.iq_pi,&pi};
    for(unsigned i=0;i<3;++i) {
        PI_Controller_TypeDef *p=controllers[i];
        assert(p->Ref==0 && p->Fbk==0 && p->Err==0 && p->Up==0 && p->Ui==0 && p->Out==0);
    }
    /* 非零固定区每拍仅清 16 bin；电流输出、FOC 次数和原对齐时长不变。 */
    check_clear_prefix(16); check_alignment_command(1);
    for(unsigned ticks=2;ticks<=ENCODER_OFFSET_LUT_SIZE/16;++ticks) {
        calibration_tick(); check_clear_prefix(ticks*16); check_alignment_command(ticks);
        assert(CalibStep==CS_OBS_ALIGN_ORIGIN && startup.state_ticks==ticks);
    }
    unsigned align_ticks=(unsigned)((cfg->align_current_ramp_time_s+
        cfg->align_hold_time_s)/Current_Ts);
    for(unsigned ticks=ENCODER_OFFSET_LUT_SIZE/16+1;ticks<=align_ticks;++ticks) {
        calibration_tick(); check_alignment_command(ticks);
        if(ticks<align_ticks) {
            assert(CalibStep==CS_OBS_ALIGN_ORIGIN && startup.state_ticks==ticks);
        }
    }
    assert(CalibStep==CS_OBS_WAIT_CLOSED_LOOP && startup.state==SENSORLESS_STARTUP_OPEN_LOOP);
    assert(startup.state_ticks==0); check_clear_prefix(ENCODER_OFFSET_LUT_SIZE);
    startup.state=SENSORLESS_STARTUP_CLOSED_LOOP; startup.speed_pi_output_max=1;
    startup.speed_feedback=cfg->target_electrical_velocity_rad_s/21;
    observer.omega_e=cfg->target_electrical_velocity_rad_s;
    motor.speedShadow=startup.speed_feedback;
    /* 经过真实采样完成边界，使构表游标从清零结束处复位。 */
    CalibStep=CS_OBS_SAMPLE_CW;
    observer.theta_e_unwrapped=(float)MotorLoadProfile_Calibration(active_flags)->scan_turns*
        _2PI*(float)motor.motor_pole_pairs;
    calibration_tick(); assert(CalibStep==CS_OBS_BUILD_LUT);
    for(unsigned i=0;i<ENCODER_OFFSET_LUT_SIZE;++i) {
        p_error_sum[i]=37*SENSORLESS_ENCODER_CALIB_MIN_SAMPLES_PER_BIN;
        calibration_samples[i]=SENSORLESS_ENCODER_CALIB_MIN_SAMPLES_PER_BIN;
    }
    for(unsigned i=0;i<4096 && CalibStep==CS_OBS_BUILD_LUT;++i) {
        calibration_tick();
        assert(motor.ErrorNow==No_Error && save_calls==0 && !samples_released());
    }
    assert(CalibStep==CS_OBS_VERIFY_CW);
    for(unsigned i=0;i<ENCODER_OFFSET_LUT_SIZE;++i) {
        assert(candidate_linearization_lut[i]==37);
        assert(encoder.linearization_lut_q15[i]==0);
    }
    EncoderCalibration_Reset();
    assert(!candidate_linearization_lut && !p_error_sum && !calibration_samples);

    /* Mode 12 仍在原对齐入口同步清完，旧构表路径覆盖全部非零固定区内容。 */
    setup(); motor.ModeNow=Calib_EncoderOffset; motor.calib_current=3;
    Task_Calib_EncoderOffset(&foc,&motor,&encoder,&observer);
    assert(CalibStep==CS_ENC_OFFSET_ALIGN && voltage_calls==0);
    Task_Calib_EncoderOffset(&foc,&motor,&encoder,&observer);
    assert(CalibStep==CS_ENC_OFFSET_ALIGN_LOOP && voltage_calls==1 && current_calls==0);
    check_clear_prefix(ENCODER_OFFSET_LUT_SIZE);
    for(unsigned i=0;i<ENCODER_OFFSET_LUT_SIZE;++i) {
        assert(candidate_linearization_lut[i]==(int16_t)0xA5A5);
        p_error_sum[i]=37; calibration_samples[i]=1;
    }
    assert(Encoder_Calib_BuildLut(1));
    for(unsigned i=0;i<ENCODER_OFFSET_LUT_SIZE;++i) assert(candidate_linearization_lut[i]==37);
    EncoderCalibration_Reset();
}
static void check_initialization_stop_restart(void) {
    /* 清零未完成时 STOP 或上游故障，丢弃缓冲；重启必须从第 0 bin 清起。 */
    for(unsigned kind=0;kind<2;++kind) {
        setup();
        for(unsigned i=0;i<ENCODER_OFFSET_LUT_SIZE;++i) encoder.linearization_lut_q15[i]=(int16_t)(i%17-8);
        encoder.electrical_zero_q15=1234; encoder.mechanical_zero_q15=4321;
        encoder.mechanical_zero_shadow_q15=54321; encoder.calib_flag=ENC_CALIB_ALL;
        original_encoder=encoder;
        prepare_calibration_power();
        for(unsigned i=0;i<7;++i) calibration_tick();
        check_clear_prefix(7*16); check_old_calibration();
        unsigned previous_calls=current_calls;
        if(kind==0) {
            Set_ModeNow(Motor_Disable); EncoderCalibration_Reset(); Stop_PWM_Generate();
        } else {
            motor.ErrorNow=Sensorless_Error; calibration_tick();
        }
        assert(CalibStep==CS_NULL && motor.ModeNow==Motor_Disable && !pwm_enabled);
        assert(current_calls==previous_calls && stop_calls==1 && save_calls==0 && samples_released());
        assert(!p_error_sum && !calibration_samples && !candidate_linearization_lut);
        check_old_calibration();
        motor.ErrorNow=No_Error; motor.ModeNow=Calib_EncoderObserver; pwm_enabled=true;
        current_calls=stop_calls=save_calls=0;
        poison_workspace(); prepare_calibration_power();
        for(unsigned ticks=1;ticks<=ENCODER_OFFSET_LUT_SIZE/16;++ticks) {
            calibration_tick(); check_clear_prefix(ticks*16); check_alignment_command(ticks);
            assert(CalibStep==CS_OBS_ALIGN_ORIGIN && startup.state_ticks==ticks);
            check_old_calibration();
        }
        EncoderCalibration_Reset();
    }
    /* 即使人为跳到对齐末拍且已有原点样本，也禁止使用未清完的缓冲。 */
    setup(); prepare_calibration_power(); calibration_tick();
    startup.state_ticks=(unsigned)((cfg->align_current_ramp_time_s+
        cfg->align_hold_time_s)/Current_Ts)-2;
    calibration_tick();
    assert(CalibStep==CS_OBS_ALIGN_ORIGIN && motor.ErrorNow==No_Error);
    calibration_tick();
    assert(motor.ErrorNow==Encoder_Error && CalibStep==CS_NULL && !pwm_enabled);
    assert(motor.ModeNow==Motor_Disable && stop_calls==1 && samples_released() && save_calls==0);
    assert(!p_error_sum && !calibration_samples && !candidate_linearization_lut);
}
static void check_preparation_cancel(void) {
    /* 两个未运行 FOC 的入口窗口均可 STOP；下一次重新准备才允许上电。 */
    for(unsigned pending_tick=1;pending_tick<=2;++pending_tick) {
        for(unsigned fault=0;fault<2;++fault) {
            setup(); pwm_enabled=false; original_encoder=encoder;
            for(unsigned i=0;i<pending_tick;++i) calibration_tick();
            assert(!samples_released() && current_calls==0);
            if(fault) {
                motor.ErrorNow=Sensorless_Error; calibration_tick();
            } else {
                Set_ModeNow(Motor_Disable); EncoderCalibration_Reset(); Stop_PWM_Generate();
            }
            assert(CalibStep==CS_NULL && !EncoderObserverCalibration_IsPowerStartReady());
            assert(samples_released() && current_calls==0 && !pwm_enabled && save_calls==0);
            check_old_calibration();
        }
    }
    /* 待启用拍仍执行实时参数/编码器守卫，不因已有 ready 而跳过检查。 */
    for(unsigned after_prepare=0;after_prepare<2;++after_prepare) {
        for(unsigned invalid=0;invalid<8;++invalid) {
            setup(); pwm_enabled=false;
            if(after_prepare) {
                calibration_tick(); assert(EncoderObserverCalibration_IsPowerStartReady());
            }
            if(invalid==0) encoder.has_valid_sample=false;
            if(invalid==1) motor.motor_pole_pairs=0;
            if(invalid==2) motor.motor_phase_resistance=0;
            if(invalid==3) motor.current_limit=0;
            if(invalid==4) motor.current_limit=.6f;
            if(invalid==5) {
                motor.thermal_limit_active=true; motor.thermal_current_limit_a=7;
            }
            if(invalid==6) motor.motor_pole_pairs=14; /* 固定机械转速未达观测器最低电角速度。 */
            if(invalid==7) motor.current_limit=NAN;
            calibration_tick();
            assert(CalibStep==CS_NULL && !pwm_enabled && motor.ModeNow==Motor_Disable);
            assert(motor.ErrorNow!=No_Error && current_calls==0 && samples_released());
            assert(!EncoderObserverCalibration_IsPowerStartReady() && save_calls==0);
            if(invalid==4) assert(motor.ErrorNow==(active_flags ? MotorParam_Error:Sensorless_Error));
            if(invalid==5) assert(motor.ErrorNow==MotorThermal_Protection);
            if(invalid>=6) assert(motor.ErrorNow==Sensorless_Error);
        }
    }
}
static void check_pending_candidate_lifetime(void) {
    enter_verified_candidate();
    /* 正常提前滑停：关相时仍为旧标定，提交与释放完成后才发布保存模式。 */
    observer.omega_e=cfg->minimum_electrical_velocity_rad_s;
    unsigned previous_calls=current_calls;
    calibration_tick();
    assert(CalibStep==CS_OBS_COMMIT && !pwm_enabled && stop_calls==1 && save_calls==0);
    assert(!samples_released() && current_calls==previous_calls); check_old_calibration();
    calibration_tick();
    assert(motor.ModeNow==Save_Param && save_calls==1 && stop_calls==1);
    assert(current_calls==previous_calls && !pwm_enabled);
    assert(!candidate_linearization_lut && !p_error_sum && !calibration_samples);

    /* 外部 STOP 调用的 Reset 只丢弃候选，不替换此前 LUT 和两个零位。 */
    enter_verified_candidate();
    Set_ModeNow(Motor_Disable); EncoderCalibration_Reset(); Stop_PWM_Generate();
    check_old_calibration();
    assert(CalibStep==CS_NULL && save_calls==0 && samples_released());
    assert(!candidate_linearization_lut && !p_error_sum && !calibration_samples);

    /* 传感器掉线与同拍上游故障均不能借低速收尾路径保存候选。 */
    for(unsigned kind=0;kind<2;++kind) {
        enter_verified_candidate();
        observer.omega_e=0;
        if(kind==0) encoder.has_valid_sample=false;
        else motor.ErrorNow=Sensorless_Error;
        previous_calls=current_calls;
        calibration_tick();
        check_old_calibration();
        assert(motor.ModeNow==Motor_Disable && CalibStep==CS_NULL && !pwm_enabled);
        assert(current_calls==previous_calls && save_calls==0 && samples_released());
        assert(motor.ErrorNow==(kind==0 ? Encoder_Error : Sensorless_Error));
        assert(!candidate_linearization_lut && !p_error_sum && !calibration_samples);
    }
    /* 关相至提交之间取消：旧 LUT/零位不变，不能留下半张新表。 */
    for(unsigned kind=0;kind<6;++kind) {
        enter_verified_candidate(); observer.omega_e=0; calibration_tick();
        assert(CalibStep==CS_OBS_COMMIT && !pwm_enabled && save_calls==0);
        check_old_calibration(); previous_calls=current_calls;
        if(kind==0) {
            Set_ModeNow(Motor_Disable); EncoderCalibration_Reset();
        } else {
            if(kind==1) motor.ErrorNow=Sensorless_Error;
            if(kind==2) encoder.has_valid_sample=false;
            if(kind==3) motor.motor_phase_resistance=0;
            if(kind==4) motor.current_limit=0;
            if(kind==5) motor.ModeNow=Motor_Disable; /* 模拟提交任务前已写入 STOP。 */
            calibration_tick();
        }
        check_old_calibration();
        assert(CalibStep==CS_NULL && motor.ModeNow==Motor_Disable && !pwm_enabled);
        assert(current_calls==previous_calls && save_calls==0 && samples_released());
    }
}
static void check_current_ramp_completion(void) {
    const MotorCalibrationProfile *profile=MotorLoadProfile_Calibration(active_flags);
    unsigned ramp_ticks=(unsigned)(profile->stop_current_ramp_time_s/Current_Ts);
    enter_verified_candidate();
    /* 先走真实减速状态转移，使卸流起点来自最后一拍制动电流。 */
    float stop_speed=Encoder_ObserverCalib_GetStopSpeed(profile,
        profile->startup.minimum_electrical_velocity_rad_s/(float)motor.motor_pole_pairs);
    observer.omega_e=stop_speed*21; encoder.vel_mech_continuous=stop_speed;
    pi.Out=pi.Ui=-.1f;
    for(unsigned i=0;i<22000 && CalibStep==CS_OBS_STOP_DECEL;++i) {
        calibration_tick(); check_old_calibration();
    }
    assert(CalibStep==CS_OBS_STOP_CURRENT && pwm_enabled && stop_calls==0);
    assert(motor.iqRef<0);
    float initial_current=motor.iqRef;
    unsigned previous_calls=current_calls;
    for(unsigned i=1;i<ramp_ticks;++i) {
        calibration_tick();
        assert(CalibStep==CS_OBS_STOP_CURRENT && pwm_enabled && stop_calls==0);
        assert(current_calls==previous_calls+i);
        float expected=initial_current*(1-(float)i*Current_Ts/profile->stop_current_ramp_time_s);
        assert(fabsf(motor.iqRef-expected)<1e-6f);
        check_old_calibration();
    }
    previous_calls=current_calls;
    calibration_tick();
    assert(CalibStep==CS_OBS_COMMIT && motor.ModeNow==Calib_EncoderObserver);
    assert(!pwm_enabled && current_calls==previous_calls && save_calls==0);
    assert(!samples_released()); check_old_calibration();
    calibration_tick();
    assert(motor.ModeNow==Save_Param && CalibStep==CS_NULL && !pwm_enabled);
    assert(current_calls==previous_calls && save_calls==1 && samples_released());
    assert(motor.idRef==0 && motor.iqRef==0 && pi.Ui==0 && !encoder.velocity_ready);
}

static void check_calibration_limits_and_reset(void) {
    setup(); CalibStep=CS_NULL;
    motor.current_limit=active_flags ? cfg->minimum_current_limit_a-.01f : .6f;
    calibration_tick();
    assert(motor.ErrorNow==(active_flags ? MotorParam_Error : Sensorless_Error) &&
        current_calls==0 && !pwm_enabled);
    setup(); motor.ModeNow=Calib_EleAngelOffset; motor.calib_current=3;
    motor.current_limit=active_flags ? 4.99f : 0;
    Task_Calib_EleAngelOffset(&foc,&motor,&encoder);
    assert(motor.ErrorNow==MotorParam_Error && current_calls==0);
    setup(); motor.ModeNow=Calib_EleAngelOffset; motor.calib_current=3;
    if(active_flags) { motor.current_limit=5.0f; }
    encoder.linearized_q15=12345; encoder.read_status=ENCODER_READ_OK;
    for(unsigned i=0;i<200;i++) Task_Calib_EleAngelOffset(&foc,&motor,&encoder);
    assert(electrical_zero_loop_count==200 && motor.idRef>0);
    EncoderCalibration_Reset();
    assert(electrical_zero_loop_count==0 && CalibStep==CS_NULL);
    if(active_flags) {
        motor.current_limit=8.0f;
        motor.calib_current=5.0f;
    }
    Task_Calib_EleAngelOffset(&foc,&motor,&encoder);
    assert(fabsf(motor.idRef-(active_flags ? 5.0f : 3.0f)*Current_Ts/.5f)<1e-6f);
    if(active_flags) {
        for(unsigned i=0;i<11000;i++) {
            Task_Calib_EleAngelOffset(&foc,&motor,&encoder);
            assert(motor.idRef<=5.0f && motor.iqRef==0.0f);
        }
        assert(motor.idRef==5.0f);
        motor.current_limit=5.0f;
        motor.calib_current=8.0f;
        Task_Calib_EleAngelOffset(&foc,&motor,&encoder);
        assert(motor.idRef==5.0f);
    }
    for(unsigned i=0;i<31000 && motor.ModeNow!=Save_Param;i++) {
        Task_Calib_EleAngelOffset(&foc,&motor,&encoder);
    }
    assert(motor.ModeNow==Save_Param && encoder.calib_flag==ENC_CALIB_ALL);
    assert(encoder.electrical_zero_q15==12345);
    assert(motor.idRef==0 && motor.iqRef==0);
    EncoderCalibration_Reset();
    assert(encoder.calib_flag==ENC_CALIB_ALL && encoder.electrical_zero_q15==12345);
}
int main(void) {
    assert(CS_OBS_ALIGN_ORIGIN==22 && CS_OBS_STOP_CURRENT==30 && CS_ENC_OFFSET_END==35);
    assert(sizeof(calibration_error_sum_storage)+sizeof(calibration_samples_storage)+
        sizeof(calibration_lut_storage)==8192);
    for (active_flags=0; active_flags<=1; ++active_flags) {
        cfg=&MotorLoadProfile_Calibration(active_flags)->startup;
        check_startup_configuration_guards(); check_live_model_guards();
        check_handoff(1); check_handoff(-1); check_voltage_rotation();
        check_handoff_trig_cache();
        check_braking_cap(); check_speed_limit_math();
        check_speed_tick_equivalence(); check_id_ramp_equivalence();
        check_calibration_stop(); check_candidate_initialization(); check_initialization_stop_restart();
        check_preparation_cancel();
        check_pending_candidate_lifetime();
        check_current_ramp_completion(); check_calibration_limits_and_reset();
    }
    puts("PASS both-profile startup and guards, handoff continuity, braking cutoff, fixed 8192 B workspace, separate preparation/PWM/alignment ticks, poisoned-buffer incremental initialization and STOP/fault restart, unchanged alignment timing and mode 12 initialization, separate PWM-off and atomic LUT commit, STOP/fault preserves old calibration, commit/release before save, encoder direction preserved");
}
'''


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--cc', required=True)
    ap.add_argument('--out', type=Path, default=ROOT/'outputs/sensorless_transition_tests')
    a=ap.parse_args(); out=a.out.resolve(); out.mkdir(parents=True,exist_ok=True)
    (out/'main.h').write_text('#include <stdint.h>\n#include <stddef.h>\n')
    run=(ROOT/'firmware/app/foc_run.c').read_text(encoding="utf-8")
    run=run[run.index('static float Sensorless_AngleDifference'):run.index('/**\n\t* @brief  Mode-3')]
    cal=(ROOT/'firmware/motor/identification/foc_calibration.c').read_text(encoding="utf-8")
    helpers=cal[cal.index('static int32_t calibration_error_sum_storage'):cal.index('/**\n\t* @brief  Calibrate Rs')]
    reset=function_source((ROOT/'firmware/motor/foc/foc_sensorless.c').read_text(encoding="utf-8"),'SensorlessStartup_Reset')
    encoder_source=(ROOT/'firmware/platform/stm32g4/bsp/encoder.c').read_text(encoding='utf-8')
    fixture=PRELUDE+reset+run+helpers+function_source(encoder_source,'Encoder_SetElectricalZeroQ15')+function_source(cal,'Task_Calib_EncoderOffset')+function_source(cal,'Task_Calib_EncoderObserver')+function_source(cal,'Task_Calib_EleAngelOffset')+CASES
    source=out/'transitions.c'; source.write_text(fixture,encoding='utf-8')
    compiler=[a.cc]+(['cc'] if Path(a.cc).stem=='zig' else [])
    logs=[]
    for label in ("both_profiles",):
        exe=out/f'transitions_{label}.exe'
        cmd=compiler+['-I',str(out)]+NATIVE_INCLUDE_FLAGS+[
            '-std=c99','-O1','-UNDEBUG','-Wall','-Wextra','-Werror','-Wno-unused-function',
            str(source), str(ROOT/'firmware/motor/motor_load_profile.c'),
            str(ROOT/'firmware/motor/foc/foc_pid.c'),str(ROOT/'firmware/common/utils.c'),
            '-o',str(exe),'-lm']
        for command in (cmd,[str(exe)]):
            r=subprocess.run(command,capture_output=True,text=True)
            logs.append(r.stdout+r.stderr); print(logs[-1],end='')
            (out/'test.log').write_text('\n'.join(logs),encoding='utf-8')
            r.check_returncode()


if __name__=='__main__': main()
