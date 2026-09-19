"""使用合成编码器/电流数据执行生产齿槽核心与 FOC 适配层。

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
MotorControl_TypeDef MotorControl;
Encoder_TypeDef OnBoard_Encoder;
static FOC_TypeDef foc;
static PI_Controller_TypeDef pi;
static bool pwm;
static unsigned enabled;
void Stop_PWM_Generate(void) { pwm=false; }
void Start_PWM_Generate(void) { pwm=true; ++enabled; }
void Set_ModeNow(ModeNow_TypeDef m) { if(m==Save_Param) assert(!pwm); MotorControl.ModeNow=m; }
void Set_ErrorNow(ErrorNow_TypeDef e) { MotorControl.ErrorNow=e; }
bool Encoder_IsOnline(const Encoder_TypeDef *e) { return e->has_valid_sample; }
uint32_t critical_hw_enter(void) { return 0U; }
void critical_hw_exit(uint32_t state) { (void)state; }
float Encoder_GetMecVelContinuous(const Encoder_TypeDef *e) { return e->vel_mech_continuous; }
float Encoder_GetElePhase(const Encoder_TypeDef *e) { return e->theta_elec; }
float Encoder_GetEleVel(const Encoder_TypeDef *e) { return e->vel_elec; }
uint8_t Encoder_GetCalibFlag(const Encoder_TypeDef *e) { return e->calib_flag; }
uint8_t Encoder_GetReverse(const Encoder_TypeDef *e) { return e->reverse; }
void FOC_CurrentController_Reset(FOC_TypeDef *f) { (void)f; }
void FOC_Current(FOC_TypeDef *f,MotorControl_TypeDef *m,float p,float v) {
    (void)f;(void)p;(void)v; assert(isfinite(m->iqRef) && fabsf(m->iqRef)<=1.001f);
}
static CoggingCalibration c;
static CoggingMapRecord record, before;
static void run_point(float iq) {
    unsigned old=c.points_done;
    while(c.points_done==old && c.state!=COGGING_FAILED)
        CoggingCalibration_Update(&c,c.target_rad,0,iq,false);
}
/* 模拟运行状态机对结果协议的消费：SWITCH_MODE 按 power_off 停相并应用模式；
 * FAULT 锁存故障、停相并回 Motor_Disable（等价旧模块内联动作的归属方）。 */
static void consume(MotorWorkOutcome_TypeDef o) {
    if(o.result==MOTOR_WORK_SWITCH_MODE) {
        if(o.power_off) Stop_PWM_Generate();
        MotorControl.ModeNow=o.next_mode;
    } else if(o.result==MOTOR_WORK_FAULT) {
        Set_ErrorNow(o.error);
        if(pwm) Stop_PWM_Generate();
        MotorControl.ModeNow=Motor_Disable;
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
    FocCogging_Abort(); memset(&MotorControl,0,sizeof(MotorControl));
    memset(&OnBoard_Encoder,0,sizeof(OnBoard_Encoder)); memset(&foc,0,sizeof(foc));
    MotorControl.ModeNow=Calib_Anticogging; MotorControl.axis_profile_valid=true;
    MotorControl.current_limit=6;MotorControl.calib_current=3;MotorControl.speed_limit=20;
    MotorControl.speed_Kp=.05f;MotorControl.speed_Ki=.5f;MotorControl.motor_pole_pairs=21;
    OnBoard_Encoder.has_valid_sample=true;OnBoard_Encoder.calib_flag=3;OnBoard_Encoder.velocity_ready=true;
    foc.Vbus=foc.Vbus_filt=30;foc.temp=25;pwm=false;enabled=0;
}
static void adapter_tests(void) {
    setup();foc.Vbus=foc.Vbus_filt=20;
    MotorWorkOutcome_TypeDef out=FocCogging_Task(&foc,&MotorControl,&pi,&OnBoard_Encoder);
    assert(out.result==MOTOR_WORK_FAULT && out.error==CoggingCalibration_Error);
    consume(out);
    assert(!pwm && !enabled && MotorControl.ModeNow==Motor_Disable);
    setup();MotorControl.axis_profile.magic=MOTOR_AXIS_PROFILE_MAGIC;
    assert(!FocCogging_CanStart(&MotorControl,&OnBoard_Encoder));
    setup();OnBoard_Encoder.calib_flag=1;
    assert(!FocCogging_CanStart(&MotorControl,&OnBoard_Encoder));
    setup();before=CoggingMap;
    out=FocCogging_Task(&foc,&MotorControl,&pi,&OnBoard_Encoder);consume(out);assert(pwm);
    OnBoard_Encoder.has_valid_sample=false;
    out=FocCogging_Task(&foc,&MotorControl,&pi,&OnBoard_Encoder);
    assert(out.result==MOTOR_WORK_FAULT && out.error==CoggingCalibration_Error);
    consume(out);
    assert(!pwm && CoggingCalib.state==COGGING_FAILED);
    assert(!memcmp(&CoggingMap,&before,sizeof(before)));
    setup();
    consume(FocCogging_Task(&foc,&MotorControl,&pi,&OnBoard_Encoder));
    double synthetic_position=0.0,synthetic_velocity=0.0;
    for(unsigned ticks=0;MotorControl.ModeNow==Calib_Anticogging && ticks<36000000;ticks++) {
        /* Independent inertial plant, driven by actual controller output. */
        double load=.03*sin(synthetic_position*21.0)+.10*synthetic_velocity;
        synthetic_velocity+=(MotorControl.iqRef-load)/.005*Current_Ts;
        synthetic_position+=synthetic_velocity*Current_Ts;
        OnBoard_Encoder.vel_mech_continuous=(float)synthetic_velocity;
        OnBoard_Encoder.shadow_q15=(int64_t)llround(synthetic_position*65536.0/6.283185307);
        OnBoard_Encoder.linearized_q15=(uint16_t)OnBoard_Encoder.shadow_q15;
        foc.Iq=MotorControl.iqRef;
        out=FocCogging_Task(&foc,&MotorControl,&pi,&OnBoard_Encoder);
        if(out.result!=MOTOR_WORK_RUNNING) consume(out);
    }
    assert(CoggingCalib.state==COGGING_COMPLETE && CoggingCalib.points_done==2048);
    assert(FocCogging_GetState()==COGGING_SAVING);
    assert(!pwm && MotorControl.ModeNow==Motor_Disable && OnBoard_Encoder.reverse==0);
    FocCogging_Service();assert(MotorControl.ModeNow==Save_Param && FocCogging_TableValid());
    FocCogging_SaveResult(false);assert(CoggingCalib.reason==COGGING_SAVE_FAILED);
    OnBoard_Encoder.electrical_zero_q15++;assert(!FocCogging_TableValid());
    puts("PASS production FOC adapter result protocol: unsafe-start fault, encoder loss fault, complete-then-save, failed-save status, unchanged direction and stale-map rejection");
}
static void compensation_tests(void) {
    setup();MotorControl.ModeNow=Motor_Disable;
    memset(&c,0,sizeof(c));c.state=COGGING_COMPLETE;c.points_done=2048;c.full_scale_a=CURRENT_SENSE_PROFILE_FULL_SCALE_A;
    c.iq_q15[0]=100;c.iq_q15[1023]=-100;
    assert(CoggingMap_Build(c.iq_q15,c.full_scale_a,Cogging_EncoderSignature(0,0,21,OnBoard_Encoder.linearization_lut_q15),&CoggingMap));
    assert(FocCogging_SetCompensation(true));MotorControl.ModeNow=Current_Mode;MotorControl.iqRef=.2f;
    for(unsigned i=0;i<4001;i++) FocCogging_Apply(&MotorControl,&OnBoard_Encoder);
    float scale=82.5f/32768;
    assert(fabsf(CoggingCompensation.blend-1)<1e-6f);
    assert(fabsf(CoggingCompensation.total_a-(.2f+100*scale))<1e-6f && MotorControl.iqRef==.2f);
    OnBoard_Encoder.linearized_q15=32;
    assert(fabsf(FocCogging_Apply(&MotorControl,&OnBoard_Encoder)-(.2f+50*scale))<1e-6f);
    OnBoard_Encoder.linearized_q15=65504; /* halfway across the last-to-first wrap */
    assert(fabsf(FocCogging_Apply(&MotorControl,&OnBoard_Encoder)-.2f)<1e-6f);
    OnBoard_Encoder.linearized_q15=65472;MotorControl.current_limit=.3f;MotorControl.iqRef=-.2f;
    assert(FocCogging_Apply(&MotorControl,&OnBoard_Encoder)==-.3f);
    MotorControl.iqRef=0;assert(FocCogging_SetCompensation(false));
    assert(FocCogging_Apply(&MotorControl,&OnBoard_Encoder)<-.2f); /* smooth off */
    for(unsigned i=0;i<4001;i++) FocCogging_Apply(&MotorControl,&OnBoard_Encoder);
    assert(CoggingCompensation.total_a==0);
    assert(FocCogging_SetCompensation(true));OnBoard_Encoder.electrical_zero_q15++;
    assert(FocCogging_Apply(&MotorControl,&OnBoard_Encoder)==0 && CoggingCompensation.enabled==0);
    assert(!FocCogging_SetCompensation(true));OnBoard_Encoder.electrical_zero_q15--;
    CoggingMap.iq_q15[3]++;assert(!FocCogging_SetCompensation(true));
    CoggingMap.iq_q15[3]--; /* restore CRC-valid table, first request while running */
    CoggingCompensation.request=1;
    assert(FocCogging_Apply(&MotorControl,&OnBoard_Encoder)==0);
    assert(CoggingCompensation.request==1 && CoggingCompensation.enabled==0);
    FocCogging_Service();assert(CoggingCompensation.enabled==1);
    CoggingCompensation.request=0;FocCogging_Service();assert(CoggingCompensation.enabled==0);
    TorqueGuard.enabled=1;TorqueGuard.lease_ticks=2;TorqueGuard.speed_limit_rad_s=2;
    assert(FocCogging_TorqueGuard(&MotorControl,&OnBoard_Encoder));
    assert(FocCogging_TorqueGuard(&MotorControl,&OnBoard_Encoder));
    pwm=true;assert(!FocCogging_TorqueGuard(&MotorControl,&OnBoard_Encoder));
    /* 跳闸只置挂起标志；停相与模式回退由调度层转为结果协议后执行（此处模拟消费）。 */
    assert(FocCogging_TakeTorqueTrip() && TorqueGuard.trip==1);
    consume((MotorWorkOutcome_TypeDef){MOTOR_WORK_SWITCH_MODE,Motor_Disable,No_Error,true});
    assert(!pwm && MotorControl.ModeNow==Motor_Disable);
    TorqueGuard.lease_ticks=100;OnBoard_Encoder.vel_mech_continuous=3;
    assert(!FocCogging_TorqueGuard(&MotorControl,&OnBoard_Encoder) && TorqueGuard.trip==2);
    assert(FocCogging_TakeTorqueTrip());
    OnBoard_Encoder.vel_mech_continuous=0;TorqueGuard.enabled=0;
    assert(FocCogging_TorqueGuard(&MotorControl,&OnBoard_Encoder));
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
int main(void) {lookup_tests();update_core_tests();core_tests();adapter_tests();compensation_tests();}
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
            "firmware/motor/identification/cogging_calibration.c",
            "firmware/motor/identification/cogging_map.c",
            "firmware/motor/identification/foc_cogging_calibration.c",
            "firmware/motor/foc/cogging_compensation.c",
            "firmware/motor/foc/foc_pid.c",
            "firmware/common/utils.c",
            "firmware/common/crc32.c",
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


if __name__ == "__main__":
    main()
