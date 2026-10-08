"""使用合成编码器/电流数据执行生产齿槽核心与 foc 适配层。

覆盖整圈双向采样、符号/量程、双向平均、拒绝样本、有界失败、先停机后保存、
旧记录保持，以及补偿纯核心的插值/准入/渐变/限幅与台架保护判定。
不访问硬件；合成被控对象不代表实物整定结论。
"""

import argparse
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tools"))
from project_paths import NATIVE_INCLUDE_FLAGS, ROOT

FIXTURE = r"""
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "foc_cogging_calibration.h"
#include "hw_conf.h"
MotorControlState motor_state;
EncoderState rotor_encoder;
static FocState foc;
static PiController pi;
static bool pwm, calibration_running;
static unsigned enabled;
static unsigned current_calls, current_resets;
void power_stage_hw_stop(void) { pwm=false; }
void power_stage_hw_start(void) { pwm=true; ++enabled; }
void MotorFault_Report(MotorControlState *m,MotorFaultCode e) { m->fault_code=e; }
bool Encoder_IsOnline(const EncoderState *e) { return e->has_valid_sample; }
uint32_t critical_hw_enter(void) { return 0U; }
void critical_hw_exit(uint32_t state) { (void)state; }
float Encoder_GetMecVelContinuous(const EncoderState *e) { return e->mechanical_velocity_continuous_rad_s; }
float Encoder_GetMecPos(const EncoderState *e)
{ return (float)(e->shadow_q15 - e->mechanical_zero_shadow_q15) * (6.283185307f / 65536.0f); }
float Encoder_GetElePhase(const EncoderState *e) { return e->electrical_angle_rad; }
float Encoder_GetEleVel(const EncoderState *e) { return e->electrical_velocity_rad_s; }
uint8_t Encoder_GetCalibFlag(const EncoderState *e) { return e->calib_flag; }
uint8_t Encoder_GetReverse(const EncoderState *e) { return e->reverse; }
void FOC_CurrentController_Reset(FocState *f) { (void)f; ++current_resets; }
void FOC_Current(FocState *f,MotorControlState *m,float p,float v) {
    ++current_calls;
    (void)f;(void)p;(void)v; assert(isfinite(m->iq_target_a) && fabsf(m->iq_target_a)<=1.001f);
}
/* 直接编译生产适配层，允许离线检查私有状态，不增加运行时 getter。 */
#include "foc_cogging_calibration.c"
#include "foc_cogging_runtime.c"
static CoggingCalibration c;
static CoggingMapRecord record, before;
static void run_point(float iq) {
    unsigned old=c.points_done;
    while(c.points_done==old && c.state!=COGGING_FAILED)
        CoggingCalibration_Update(&c,c.target_rad,0,iq,false);
}
/* 模拟所有者消费功能事实：PREPARED 许可启相，FINALIZE 先关相后整理；
 * FAULT 锁存故障、停相并回 MOTOR_CONTROL_NONE（等价旧模块内联动作的归属方）。 */
static void consume(MotorTaskResult o) {
    if(o.result==MOTOR_TASK_PREPARED) {
        assert(!pwm);
        power_stage_hw_start();
    } else if(o.result==MOTOR_TASK_FINALIZE || o.result==MOTOR_TASK_STOP_REQUESTED) {
        power_stage_hw_stop(); calibration_running=false;
        motor_state.control_mode=MOTOR_CONTROL_NONE;
    } else if(o.result==MOTOR_TASK_FAILED) {
        MotorFault_Report(&motor_state,o.error);
        if(pwm) power_stage_hw_stop();
        calibration_running=false;
        motor_state.control_mode=MOTOR_CONTROL_NONE;
    }
}
static void core_tests(void) {
    assert(sizeof(record.iq_q15)==2048 && sizeof(record)==2064);
    assert(!CoggingCalibration_Start(&c,NAN,82.5f));
    assert(!CoggingCalibration_Start(&c,0,0));
    for(unsigned shunt=0;shunt<2;shunt++) {
        float scale=shunt ? 27.5f : 82.5f;
        bool seen[2][1024]={{false}};
        assert(CoggingCalibration_Start(&c,6.282f,scale));
        while(c.state!=COGGING_COMPLETE) {
            unsigned pass=c.direction_pass,index=c.index;
            assert(fabsf(c.target_rad-(float)c.target_grid*COGGING_STEP_RAD)<0.000001f);
            if(c.state==COGGING_TURNAROUND) {
                for(unsigned i=0;i<COGGING_SETTLE_TICKS;i++) CoggingCalibration_Update(&c,c.target_rad,0,0,false);
                assert(c.state==COGGING_SETTLING); continue;
            }
            assert(!seen[pass][index]); seen[pass][index]=true;
            float cog=.20f*sinf((float)index*COGGING_STEP_RAD*21);
            run_point(cog+.07f+(pass ? -.10f : .10f));
            assert(c.state!=COGGING_FAILED);
        }
        for(unsigned i=0;i<1024;i++) assert(seen[0][i] && seen[1][i]);
        assert(CoggingMap_Build(c.iq_q15,c.full_scale_a,42,&record));
        assert(CoggingMap_Valid(&record,42) && !CoggingMap_Valid(&record,43));
        for(unsigned i=0;i<1024;i++) {
            float expected=.2f*sinf((float)i*COGGING_STEP_RAD*21);
            assert(fabsf(record.iq_q15[i]*scale/32768.0f-expected)<2.0f*scale/32768.0f);
        }
        record.iq_q15[99]^=1; assert(!CoggingMap_Valid(&record,42));
    }
    /* Positive full scale saturates at +32767; negative is exactly -32768. */
    assert(CoggingCalibration_Start(&c,0,82.5f));run_point(82.5f);assert(c.iq_q15[1]==32767);
    run_point(-82.5f);assert(c.iq_q15[2]==-32768);
    before=record;CoggingCalibration_Abort(&c,COGGING_CANCELLED);
    assert(c.state==COGGING_FAILED);
    assert(CoggingCalibration_Start(&c,0,82.5f));
    for(unsigned i=0;i<COGGING_SETTLE_TICKS+50;i++) CoggingCalibration_Update(&c,c.target_rad,0,.1f,false);
    assert(c.samples==50);CoggingCalibration_Update(&c,c.target_rad,.3f,.1f,false);
    assert(c.samples==0 && c.state==COGGING_SETTLING);
    /* An 8 ms quantized speed estimate may toggle above 0.08 rad/s while
     * raw position holds the grid. Its slow drift, not each toggle, gates us. */
    assert(CoggingCalibration_Start(&c,0,82.5f));
    for(unsigned i=0;i<COGGING_SETTLE_TICKS+COGGING_SAMPLE_TICKS;i++)
        CoggingCalibration_Update(&c,c.target_rad,i%2 ? .1f : -.1f,.1f,false);
    assert(c.points_done==1);
    /* Isolated invalid samples are excluded, persistent/too frequent noise
     * restarts the window; no out-of-gate current may bias the mean. */
    assert(CoggingCalibration_Start(&c,0,82.5f));
    for(unsigned i=0;i<COGGING_SETTLE_TICKS;i++) CoggingCalibration_Update(&c,c.target_rad,0,.1f,false);
    for(unsigned i=0;i<COGGING_SAMPLE_TICKS;i++) {
        if(i%20==0) CoggingCalibration_Update(&c,c.target_rad+COGGING_POSITION_TOL_RAD*1.1f,0,50,false);
        CoggingCalibration_Update(&c,c.target_rad,0,.1f,false);
    }
    assert(c.points_done==1);
    assert(abs(c.iq_q15[1]-40)<=1);
    for(unsigned i=0;i<COGGING_SETTLE_TICKS+50;i++) CoggingCalibration_Update(&c,c.target_rad,0,.1f,false);
    for(unsigned i=0;i<4;i++) CoggingCalibration_Update(&c,c.target_rad+COGGING_POSITION_TOL_RAD*1.1f,0,50,false);
    assert(c.state==COGGING_SETTLING && c.samples==0);
    for(unsigned i=0;i<COGGING_SETTLE_TICKS;i++) CoggingCalibration_Update(&c,c.target_rad,0,.1f,false);
    for(unsigned i=0;i<11;i++) {
        CoggingCalibration_Update(&c,c.target_rad+COGGING_POSITION_TOL_RAD*1.1f,0,50,false);
        CoggingCalibration_Update(&c,c.target_rad,0,.1f,false);
    }
    assert(c.state==COGGING_SETTLING && c.samples==0);
    assert(CoggingCalibration_Start(&c,0,82.5f));
    for(unsigned i=0;i<1000;i++) CoggingCalibration_Update(&c,c.target_rad+COGGING_POSITION_TOL_RAD*1.1f,0,.1f,false);
    assert(c.points_done==0 && c.samples==0);
    for(unsigned i=0;i<COGGING_SETTLE_TICKS+COGGING_SAMPLE_TICKS;i++)
        CoggingCalibration_Update(&c,c.target_rad+COGGING_POSITION_TOL_RAD*.75f,0,.1f,false);
    assert(c.points_done==1);
    /* At every grid over two turns, exactly four encoder counts qualify;
     * a fifth count does not. Include float rounding at the wrap boundary. */
    for(unsigned grid=1;grid<2048;grid++) {
        for(int direction=-1;direction<=1;direction+=2) {
            assert(CoggingCalibration_Start(&c,0,82.5f));
            c.target_rad=(float)grid*COGGING_STEP_RAD;
            float position=(float)((int)grid*64+direction*4)*(6.283185307179586f/65536.0f);
            CoggingCalibration_Update(&c,position,0,.1f,false);
            assert(c.stable_ticks==1);
            position=(float)((int)grid*64+direction*5)*(6.283185307179586f/65536.0f);
            CoggingCalibration_Update(&c,position,0,.1f,false);
            assert(c.stable_ticks==1);
        }
    }
    assert(CoggingCalibration_Start(&c,0,82.5f));
    for(unsigned i=0;i<20001;i++) CoggingCalibration_Update(&c,0,1,0,false);
    assert(c.state==COGGING_FAILED && c.reason==COGGING_POINT_TIMEOUT);
    assert(CoggingCalibration_Start(&c,0,82.5f));
    for(unsigned i=0;i<400;i++) CoggingCalibration_Update(&c,c.target_rad,0,1,true);
    assert(c.reason==COGGING_CURRENT_LIMIT);
    assert(CoggingCalibration_Start(&c,0,82.5f));CoggingCalibration_Update(&c,NAN,0,0,false);
    assert(c.reason==COGGING_INVALID_INPUT);
    puts("PASS full 1024-point two-pass coverage, phase-wrap, both shunt scales, signed endpoints, DC/friction removal, CRC, unstable samples, timeout, saturation, cancel");
}
static void setup(void) {
    FocCogging_Abort(); memset(&motor_state,0,sizeof(motor_state));
    memset(&rotor_encoder,0,sizeof(rotor_encoder)); memset(&foc,0,sizeof(foc));
    motor_state.control_mode=MOTOR_CONTROL_NONE; calibration_running=true; motor_state.application.axis_profile_valid=true;
    motor_state.application.current_limit_a=6;motor_state.model.calibration_current_a=3;motor_state.application.speed_limit_rad_s=20;
    motor_state.application.speed_kp=.05f;motor_state.application.speed_ki=.5f;motor_state.model.motor_pole_pairs=21;
    rotor_encoder.has_valid_sample=true;rotor_encoder.calib_flag=3;rotor_encoder.velocity_ready=true;
    foc.bus_voltage_v=foc.bus_voltage_filtered_v=30;foc.temperature_c=25;pwm=false;enabled=0;
    current_calls=current_resets=0;
}
static void terminal_tests(void) {
    /* 用同一真实任务入口覆盖采样超时/饱和终止，不绕过每拍安全门。 */
    for(unsigned failure=0;failure<2;failure++) {
        setup();before=s_cogging_map;
        consume(FocCogging_Task(&foc,&motor_state,&pi,&rotor_encoder));
        assert(pwm && current_calls==1 && current_resets==1);
        s_cogging_calib.points_done=1;
        if(failure==0) {
            s_cogging_calib.point_ticks=10000;
        } else {
            s_cogging_calib.saturation_ticks=399;
        }
        motor_state.iq_target_a=1;motor_state.id_target_a=.5f;
        motor_state.speed_target_rad_s=motor_state.speed_ramped_rad_s=.1f;
        pi.integral_output=pi.output=.3f;s_divider=SPEED_LOOP_DIVIDER-1;
        MotorTaskResult out=FocCogging_Task(&foc,&motor_state,&pi,&rotor_encoder);
        assert(out.result==MOTOR_TASK_FAILED && out.error==MOTOR_FAULT_COGGING_CALIBRATION);
        assert(s_cogging_calib.reason==(failure==0 ? COGGING_POINT_TIMEOUT : COGGING_CURRENT_LIMIT));
        assert(current_calls==1 && current_resets==2 && !s_session_active);
        assert(motor_state.iq_target_a==0 && motor_state.id_target_a==0);
        assert(motor_state.speed_target_rad_s==0 && motor_state.speed_ramped_rad_s==0);
        assert(pi.integral_output==0 && pi.output==0 && FocCogging_IsReleased());
        assert(!memcmp(&s_cogging_map,&before,sizeof(before)));
        assert(pwm);consume(out);assert(!pwm);
    }
    puts("PASS adapter timeout/saturation cleanup and owner-only phase stop");
}
static void adapter_tests(void) {
    setup();foc.bus_voltage_v=foc.bus_voltage_filtered_v=20;
    MotorTaskResult out=FocCogging_Task(&foc,&motor_state,&pi,&rotor_encoder);
    assert(out.result==MOTOR_TASK_FAILED && out.error==MOTOR_FAULT_COGGING_CALIBRATION);
    consume(out);
    assert(!pwm && !enabled && motor_state.control_mode==MOTOR_CONTROL_NONE);
    setup();motor_state.application.axis_profile.magic=MOTOR_AXIS_PROFILE_MAGIC;
    assert(!FocCogging_CanStart(&motor_state,&rotor_encoder));
    setup();rotor_encoder.calib_flag=1;
    assert(!FocCogging_CanStart(&motor_state,&rotor_encoder));
    setup();before=s_cogging_map;
    out=FocCogging_Task(&foc,&motor_state,&pi,&rotor_encoder);consume(out);assert(pwm);
    rotor_encoder.has_valid_sample=false;
    out=FocCogging_Task(&foc,&motor_state,&pi,&rotor_encoder);
    assert(out.result==MOTOR_TASK_FAILED && out.error==MOTOR_FAULT_COGGING_CALIBRATION);
    consume(out);
    assert(!pwm && s_cogging_calib.state==COGGING_FAILED);
    assert(!memcmp(&s_cogging_map,&before,sizeof(before)));
    setup();
    consume(FocCogging_Task(&foc,&motor_state,&pi,&rotor_encoder));
    double synthetic_position=0.0,synthetic_velocity=0.0;
    uint32_t trace=2166136261U,tick_count=0;
    for(unsigned ticks=0;calibration_running && ticks<36000000;ticks++) {
        /* Independent inertial plant, driven by actual controller output. */
        double load=.03*sin(synthetic_position*21.0)+.10*synthetic_velocity;
        synthetic_velocity+=(motor_state.iq_target_a-load)/.005*Current_Ts;
        synthetic_position+=synthetic_velocity*Current_Ts;
        rotor_encoder.mechanical_velocity_continuous_rad_s=(float)synthetic_velocity;
        rotor_encoder.shadow_q15=(int64_t)llround(synthetic_position*65536.0/6.283185307);
        rotor_encoder.linearized_q15=(uint16_t)rotor_encoder.shadow_q15;
        foc.iq_a=motor_state.iq_target_a;
        out=FocCogging_Task(&foc,&motor_state,&pi,&rotor_encoder);
        /* 每拍记录明确字段的位模式，不读结构填充；基线与简化输出须逐拍一致。 */
        float values[]={motor_state.iq_target_a,motor_state.id_target_a,
            motor_state.speed_target_rad_s,motor_state.speed_ramped_rad_s,
            pi.integral_output,pi.output};
        uint32_t words[12];memcpy(words,values,sizeof(values));
        words[6]=out.result;words[7]=out.error;words[8]=s_cogging_calib.state;
        words[9]=s_cogging_calib.reason;words[10]=s_cogging_calib.points_done;
        words[11]=motor_state.fault_code;
        for(unsigned i=0;i<12;i++) trace=(trace^words[i])*16777619U;
        ++tick_count;
        if(out.result!=MOTOR_TASK_RUNNING) consume(out);
    }
    assert(s_cogging_calib.state==COGGING_COMPLETE && s_cogging_calib.points_done==2048);
    assert(FocCogging_GetState()==COGGING_SAVING);
    assert(!pwm && motor_state.control_mode==MOTOR_CONTROL_NONE && rotor_encoder.reverse==0);
    assert(FocCogging_BeginFinalize());
    assert(FocCogging_BuildCandidate(&record,&motor_state,&rotor_encoder));
    assert(FocCogging_PublishCandidate(&record));
    FocCogging_ReleaseFinalize(true);
    assert(FocCogging_GetState()==COGGING_SAVING && s_save_pending && FocCogging_IsReleased());
    printf("TRACE ticks=%u controls_states=%08x candidate_crc=%08x\n",tick_count,trace,record.crc32);
    motor_state.control_mode=MOTOR_CONTROL_NONE;
    assert(FocCogging_TableValid(&motor_state,&rotor_encoder));
    FocCogging_SaveResult(false);assert(s_cogging_calib.reason==COGGING_SAVE_FAILED);
    rotor_encoder.electrical_zero_q15++;assert(!FocCogging_TableValid(&motor_state,&rotor_encoder));
    puts("PASS production foc adapter result protocol: unsafe-start fault, encoder loss fault, complete-then-save, failed-save status, unchanged direction and stale-map rejection");
}
static void compensation_tests(void) {
    setup();motor_state.control_mode=MOTOR_CONTROL_NONE;
    memset(&c,0,sizeof(c));c.state=COGGING_COMPLETE;c.points_done=2048;c.full_scale_a=CURRENT_SENSE_PROFILE_FULL_SCALE_A;
    c.iq_q15[0]=100;c.iq_q15[1023]=-100;
    assert(CoggingMap_Build(c.iq_q15,c.full_scale_a,Cogging_EncoderSignature(0,0,21,rotor_encoder.linearization_lut_q15),&s_cogging_map));
    assert(FocCogging_SetCompensation(true,&motor_state,&rotor_encoder));motor_state.control_mode=MOTOR_CONTROL_CURRENT;motor_state.iq_target_a=.2f;
    for(unsigned i=0;i<4001;i++) FocCogging_Apply(&motor_state,&rotor_encoder);
    float scale=82.5f/32768;
    assert(fabsf(s_cogging_compensation.blend-1)<1e-6f);
    assert(fabsf(s_cogging_compensation.total_a-(.2f+100*scale))<1e-6f && motor_state.iq_target_a==.2f);
    rotor_encoder.linearized_q15=32;
    assert(fabsf(FocCogging_Apply(&motor_state,&rotor_encoder)-(.2f+50*scale))<1e-6f);
    rotor_encoder.linearized_q15=65504; /* halfway across the last-to-first wrap */
    assert(fabsf(FocCogging_Apply(&motor_state,&rotor_encoder)-.2f)<1e-6f);
    rotor_encoder.linearized_q15=65472;motor_state.application.current_limit_a=.3f;motor_state.iq_target_a=-.2f;
    assert(FocCogging_Apply(&motor_state,&rotor_encoder)==-.3f);
    motor_state.iq_target_a=0;assert(FocCogging_SetCompensation(false,&motor_state,&rotor_encoder));
    assert(FocCogging_Apply(&motor_state,&rotor_encoder)<-.2f); /* smooth off */
    for(unsigned i=0;i<4001;i++) FocCogging_Apply(&motor_state,&rotor_encoder);
    assert(s_cogging_compensation.total_a==0);
    assert(FocCogging_SetCompensation(true,&motor_state,&rotor_encoder));rotor_encoder.electrical_zero_q15++;
    assert(FocCogging_Apply(&motor_state,&rotor_encoder)==0 && s_cogging_compensation.enabled==0);
    assert(!FocCogging_SetCompensation(true,&motor_state,&rotor_encoder));rotor_encoder.electrical_zero_q15--;
    s_cogging_map.iq_q15[3]++;assert(!FocCogging_SetCompensation(true,&motor_state,&rotor_encoder));
    s_cogging_map.iq_q15[3]--; /* restore CRC-valid table, first request while running */
    FocCogging_RequestCompensation(true);
    assert(FocCogging_Apply(&motor_state,&rotor_encoder)==0);
    assert(s_cogging_compensation.enabled==0);
    FocCogging_Service(&motor_state,&rotor_encoder);assert(s_cogging_compensation.enabled==1);
    FocCogging_RequestCompensation(true);FocCogging_RequestCompensation(false);
    FocCogging_Service(&motor_state,&rotor_encoder);assert(s_cogging_compensation.enabled==0);
    CoggingTorqueGuard guard;
    assert(!FocCogging_ReadTorqueGuard(NULL));
    assert(FocCogging_SetTorqueGuard(2,2));
    assert(!FocCogging_SetTorqueGuard(100,NAN));
    assert(!FocCogging_SetTorqueGuard(100,0));
    assert(!FocCogging_SetTorqueGuard(100,-1));
    assert(FocCogging_ReadTorqueGuard(&guard) && guard.enabled==1 && guard.lease_ticks==2);
    assert(FocCogging_TorqueGuard(&motor_state,&rotor_encoder));
    assert(FocCogging_TorqueGuard(&motor_state,&rotor_encoder));
    pwm=true;assert(!FocCogging_TorqueGuard(&motor_state,&rotor_encoder));
    /* 跳闸只置挂起标志；停相与模式回退由调度层转为结果协议后执行（此处模拟消费）。 */
    assert(FocCogging_ReadTorqueGuard(&guard) && guard.trip==1 && guard.lease_ticks==0);
    assert(FocCogging_SetTorqueGuard(0,0));
    assert(FocCogging_TakeTorqueTrip()); /* 关闭保护不能撤回已经挂起的STOP。 */
    consume((MotorTaskResult){MOTOR_TASK_FINALIZE, MOTOR_FAULT_NONE});
    assert(!pwm && motor_state.control_mode==MOTOR_CONTROL_NONE);
    assert(FocCogging_SetTorqueGuard(100,2));rotor_encoder.mechanical_velocity_continuous_rad_s=3;
    assert(!FocCogging_TorqueGuard(&motor_state,&rotor_encoder));
    assert(FocCogging_ReadTorqueGuard(&guard) && guard.trip==2);
    assert(FocCogging_TakeTorqueTrip());
    rotor_encoder.mechanical_velocity_continuous_rad_s=0;assert(FocCogging_SetTorqueGuard(0,0));
    assert(FocCogging_TorqueGuard(&motor_state,&rotor_encoder));
    CoggingTorqueFrame sample, previous;
    assert(sizeof(sample)==36 && !FocCogging_ReadTorqueSample(NULL));
    assert(!FocCogging_ReadTorqueSample(&sample));
    assert(FocCogging_RequestTorqueSample());
    assert(!FocCogging_RequestTorqueSample() && !FocCogging_ReadTorqueSample(&sample));
    motor_state.iq_target_a=.25f;foc.iq_a=.2f;foc.bus_voltage_filtered_v=30;
    FocCogging_TorqueObserve(&foc,&motor_state,&rotor_encoder);
    assert(!FocCogging_ReadTorqueSample(NULL) && !FocCogging_RequestTorqueSample());
    /* 未消费的帧不覆盖，空指针不丢帧，消费后调用方拥有独立副本。 */
    motor_state.iq_target_a=.5f;foc.iq_a=.4f;foc.bus_voltage_filtered_v=31;
    FocCogging_TorqueObserve(&foc,&motor_state,&rotor_encoder);
    assert(FocCogging_ReadTorqueSample(&sample));
    assert(sample.command_a==.25f && sample.feedback_a==.2f && sample.vbus_v==30);
    assert(!FocCogging_ReadTorqueSample(&sample));
    previous=sample;
    assert(FocCogging_RequestTorqueSample());
    FocCogging_TorqueObserve(&foc,&motor_state,&rotor_encoder);
    assert(FocCogging_ReadTorqueSample(&sample));
    assert(sample.tick>previous.tick && sample.command_a==.5f && previous.command_a==.25f);
    puts("PASS compensation interpolation/wrap/sign, command ownership, ramp, total clamp, stale/corrupt rejection and independent torque watchdog trip flag");
}
static void lookup_tests(void) {
    /* 纯插值核心：中点、子步、跨圈与表内满量程换算；不依赖硬件。 */
    CoggingMapRecord map;
    memset(&map,0,sizeof(map));
    map.full_scale_a=82.5f;
    map.iq_q15[0]=100;map.iq_q15[1]=200;map.iq_q15[1023]=-100;
    float scale=82.5f/32768.0f;
    assert(fabsf(CoggingMap_LookupA(&map,0)-100*scale)<1e-9f);
    assert(fabsf(CoggingMap_LookupA(&map,32)-150*scale)<1e-9f);
    assert(fabsf(CoggingMap_LookupA(&map,63)-(100+100*63.0f/64.0f)*scale)<1e-9f);
    assert(fabsf(CoggingMap_LookupA(&map,65504))<1e-9f); /* 1023 -> 0 跨圈中点 */
    map.full_scale_a=27.5f;
    assert(fabsf(CoggingMap_LookupA(&map,0)-100*(27.5f/32768.0f))<1e-9f);
    puts("PASS cogging map lookup midpoint, substep, wrap and record scale");
}
static void update_core_tests(void) {
    /* 纯 Update：未准入时只做指令限幅并输出总指令。 */
    CoggingCompensationControl control;
    CoggingCompensationTick tick;
    CoggingMapRecord map;
    memset(&control,0,sizeof(control));
    memset(&tick,0,sizeof(tick));memset(&map,0,sizeof(map));
    map.full_scale_a=82.5f;map.iq_q15[0]=1000;
    tick.mode_is_current=true;tick.error_clear=true;tick.sensorless_off=true;
    tick.encoder_usable=true;tick.command_a=.5f;tick.limit_a=.3f;tick.tick_s=Current_Ts;
    assert(fabsf(CoggingCompensation_Update(&control,&map,&tick)-.3f)<1e-6f);
    assert(control.table_a==0 && control.blend==0);
    tick.command_a=-.5f;
    assert(fabsf(CoggingCompensation_Update(&control,&map,&tick)+.3f)<1e-6f);
    puts("PASS compensation core supervision and command clamping");
}
int main(void) {lookup_tests();update_core_tests();core_tests();terminal_tests();adapter_tests();compensation_tests();}
"""

RUNTIME_FIXTURE = r"""
#include <assert.h>
#include <string.h>
#include "foc_cogging_runtime.h"
#include "current_sense_profile.h"
/* 独立链接运行边界，不提供标定任务、采样状态机、PI 或电流环符号。 */
uint32_t critical_hw_enter(void) { return 0U; }
void critical_hw_exit(uint32_t mask) { (void)mask; }
bool Encoder_IsOnline(const EncoderState *e) { return e->has_valid_sample; }
uint8_t Encoder_GetCalibFlag(const EncoderState *e) { return e->calib_flag; }
uint8_t Encoder_GetReverse(const EncoderState *e) { return e->reverse; }
float Encoder_GetMecPos(const EncoderState *e) { return e->mechanical_position_rad; }
int main(void) {
    MotorControlState motor={0};EncoderState encoder={0};
    CoggingMapRecord record;int16_t samples[COGGING_MAP_POINTS]={0};samples[0]=128;
    motor.model.motor_pole_pairs=21;motor.application.current_limit_a=2;
    encoder.has_valid_sample=true;encoder.calib_flag=ENC_CALIB_ALL;
    assert(sizeof(CoggingTorqueGuard)==16 && sizeof(CoggingTorqueFrame)==36);
    assert(sizeof(CoggingCompensationControl)==28 && sizeof(record)==2064);
    assert(CoggingMap_Build(samples,CURRENT_SENSE_PROFILE_FULL_SCALE_A,
        Cogging_EncoderSignature(0,0,21,encoder.linearization_lut_q15),&record));
    FocCogging_ImportMap(&record);
    assert(FocCogging_TableValid(&motor,&encoder));
    FocCogging_RequestCompensation(true);FocCogging_Service(&motor,&encoder);
    motor.control_mode=MOTOR_CONTROL_CURRENT;motor.iq_target_a=.25f;
    assert(FocCogging_Apply(&motor,&encoder)>.25f && motor.iq_target_a==.25f);
    assert(FocCogging_SetTorqueGuard(1,2));
    assert(FocCogging_TorqueGuard(&motor,&encoder));
    assert(!FocCogging_TorqueGuard(&motor,&encoder) && motor.iq_target_a==0);
    assert(FocCogging_TakeTorqueTrip());
    FocCogging_ResetMap();FocCogging_ExportMap(&record,&motor,&encoder);
    assert(record.magic==0);
    return 0;
}
"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cc", required=True)
    ap.add_argument("--out", type=Path, default=ROOT / "outputs/cogging_tests")
    a = ap.parse_args()
    out = a.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    (out / "main.h").write_text(
        "#ifndef TEST_MAIN_H\n#define TEST_MAIN_H\n#include <stdint.h>\n#include <stddef.h>\n"
        "static inline uint32_t __get_PRIMASK(void) {return 0;}\n"
        "static inline void __disable_irq(void) {}\n"
        "static inline void __set_PRIMASK(uint32_t p) {(void)p;}\n#endif\n"
    )
    src = out / "cogging_test.c"
    src.write_text(FIXTURE)
    compiler = [a.cc] + (["cc"] if Path(a.cc).stem == "zig" else [])
    exe = out / "cogging_test.exe"
    cmd = (
        compiler
        + ["-std=c99", "-O1", "-UNDEBUG", "-Wall", "-Wextra", "-Werror", "-I", str(out)]
        + NATIVE_INCLUDE_FLAGS
    )
    cmd += [str(src)] + [
        str(ROOT / p)
        for p in [
            "firmware/motor/calibration/cogging_calibration.c",
            "firmware/motor/calibration/cogging_map.c",
            "firmware/motor/foc/cogging_compensation.c",
            "firmware/motor/foc/foc_pid.c",
            "firmware/motor/math/control_math.c",
            "shared/checksum/crc32.c",
        ]
    ]
    cmd += ["-o", str(exe), "-lm"]
    logs = []
    for command in (cmd, [str(exe)]):
        r = subprocess.run(command, capture_output=True, text=True)
        logs.append(r.stdout + r.stderr)
        print(logs[-1], end="")
        (out / "test.log").write_text("\n".join(logs))
        r.check_returncode()

    runtime_src = out / "cogging_runtime_test.c"
    runtime_src.write_text(RUNTIME_FIXTURE)
    runtime_exe = out / "cogging_runtime_test.exe"
    runtime_cmd = (
        compiler
        + ["-std=c99", "-O1", "-UNDEBUG", "-Wall", "-Wextra", "-Werror", "-I", str(out)]
        + NATIVE_INCLUDE_FLAGS
        + [str(runtime_src)]
        + [
            str(ROOT / p)
            for p in [
                "firmware/motor/calibration/foc_cogging_runtime.c",
                "firmware/motor/calibration/cogging_map.c",
                "firmware/motor/foc/cogging_compensation.c",
                "firmware/motor/math/control_math.c",
                "shared/checksum/crc32.c",
            ]
        ]
        + ["-o", str(runtime_exe), "-lm"]
    )
    for command in (runtime_cmd, [str(runtime_exe)]):
        subprocess.run(command, check=True)
    message = (
        "PASS runtime public boundary links without calibration task/sampler or PI/current loop"
    )
    (out / "runtime_boundary.log").write_text(message + "\n")
    print(message)


if __name__ == "__main__":
    main()
