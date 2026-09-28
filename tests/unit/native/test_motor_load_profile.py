"""离线执行生产参数加载和快环切换逻辑；不连接或控制硬件。"""

import argparse
import re
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tools"))
from project_paths import NATIVE_INCLUDE_FLAGS, ROOT
from run_position_servo_tests import function_source


def fixture():
    """组合生产函数与外围替身，覆盖持久化和控制所有权边界。"""
    param = (ROOT / "firmware/services/parameters/foc_param.c").read_text(encoding="utf-8")
    task = (ROOT / "firmware/app/foc_task.c").read_text(encoding="utf-8")
    reset = (ROOT / "firmware/motor/protection/foc_errhandle.c").read_text(encoding="utf-8")
    observer = (ROOT / "firmware/motor/foc/foc_sensorless.c").read_text(encoding="utf-8")
    can = (ROOT / "firmware/communication/can/interface_can.c").read_text(encoding="utf-8")
    # 仅抽取新增 CAN 分派段；不复制其校验实现。
    dispatch = function_source(can, "CAN_ReceiveMessage_Update")
    dispatch = dispatch[: dispatch.index("\t/* Read-only handshake;")] + "}\n"
    return (
        r"""
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "foc_param.h"
#include "foc_param_profile.h"
#include "foc_run.h"
#include "motor_load_control.h"
#include "interface_can.h"
#include "utils.h"
#define MAGIC_WORD 0x454e4332U
#define FLUX_OBSERVER_DEFAULT_GAMMA 800000.0f
MotorControl_TypeDef MotorControl;
Encoder_TypeDef OnBoard_Encoder;
CANMsg_TypeDef CANMsg;
FOC_TypeDef FOC;
Fluxobserver_TypeDef Fluxobserver;
SensorlessStartup_TypeDef SensorlessStartup;
PI_Controller_TypeDef PI_Speed;
ModeNow_TypeDef ModeLast;
static volatile uint32_t pending_load_profile = UINT32_MAX;
static bool gate_off=true;
static unsigned position_resets, friction_resets, replies;
static CAN_PARAM_ID reply_id;
static float reply_value;
bool motor_hw_phase_outputs_disabled(void) { return gate_off; }
void motor_hw_outer_barrier(void) { }
void EncoderCalibration_Reset(void) { }
void Task_Position_Mode_Reset(void) { ++position_resets; }
void FOC_CurrentController_Reset(FOC_TypeDef *f) {
    PI_Controller_Reset(&f->id_pi); PI_Controller_Reset(&f->iq_pi);
}
void FocFrictionIdentification_Init(void) { ++friction_resets; }
void FocFrictionIdentification_Abort(MotorControl_TypeDef *m, PI_Controller_TypeDef *p) {
    (void)m; (void)p;
}
void CAN_SendMessage_Update(CAN_PARAM_ID id, float value) {
    ++replies; reply_id=id; reply_value=value;
}
"""
        + re.sub(r"^#include.*$", "", param, flags=re.M)
        + "\n"
        + "\n".join(
            [
                function_source(observer, "Fluxobserver_ResetState"),
                function_source(observer, "Fluxobserver_ParamInit"),
                function_source(observer, "SensorlessStartup_Reset"),
                function_source(reset, "Clear_RunningData"),
                function_source(task, "MotorControl_RequestLoadProfile"),
                function_source(task, "MotorControl_GetLoadProfile"),
                function_source(task, "MotorControl_ApplyLoadProfile"),
                function_source(task, "MotorControl_IsConfigurationValid"),
                dispatch,
            ]
        )
        + r"""
static void seed(InterfaceParam_TypeDef *baseline) {
    Param_Return_Default(); MotorControl.ModeNow=Motor_Disable; ModeLast=Motor_Disable;
    OnBoard_Encoder.calib_flag=7; OnBoard_Encoder.electrical_zero_q15=12480;
    OnBoard_Encoder.mechanical_zero_q15=213; OnBoard_Encoder.reverse=1;
    OnBoard_Encoder.linearization_lut_q15[71]=-42;
    MotorControl.speed_Kp=.071f; MotorControl.speed_Ki=.64f;
    MotorControl.current_limit=6; MotorControl.calib_current=3;
    MotorControl.friction_model_valid=true;
    Param_Upload(baseline); baseline->magic_word=MAGIC_WORD;
}
static void check_preserved(const InterfaceParam_TypeDef *before) {
    InterfaceParam_TypeDef after, expected=*before;
    Param_Upload(&after); after.magic_word=MAGIC_WORD;
    /* 唯一允许持久化差异是装配记录与旧机械摩擦模型有效位。 */
    expected.load_profile=after.load_profile;
    expected.friction_model_valid=after.friction_model_valid;
    assert(memcmp(&after,&expected,sizeof(after))==0);
}
int main(void) {
    InterfaceParam_TypeDef baseline, saved;
    MotorLoadRecord record;
    uint32_t flags=99;
    const uint32_t golden_crc[4]={0x19984c4fU,0xa1242b2aU,0U,0x0b2de3a1U};
    assert(offsetof(InterfaceParam_TypeDef,load_profile)==2240U);
    assert(offsetof(InterfaceParam_TypeDef,magic_word)==2164U);
    assert(offsetof(InterfaceParam_TypeDef,axis_profile)==2208U);
    const MotorCalibrationProfile *wheel=MotorLoadProfile_Calibration(0);
    const MotorCalibrationProfile *roll=MotorLoadProfile_Calibration(1);
    assert(wheel && roll && wheel!=roll && MotorLoadProfile_Calibration(3)==roll);
    assert(!MotorLoadProfile_Calibration(2) && !MotorLoadProfile_Calibration(UINT32_MAX));
    assert(wheel->startup.startup_iq_a==.5f && wheel->speed_mechanical_rad_s==15);
    assert(roll->startup.startup_iq_a==6.5f && roll->electrical_zero_current_a==8);
    assert(roll->startup.minimum_current_limit_a >=
        hypotf(roll->startup.startup_iq_a,roll->startup.startup_id_a));
    assert(!MotorLoadProfile_FeedforwardEnabled(0) && !MotorLoadProfile_FeedforwardEnabled(1));
    assert(MotorLoadProfile_FeedforwardEnabled(3));
    for (unsigned v=0;v<4;v++) {
        if(v==2) continue;
        assert(MotorLoadRecord_Create(v,&record));
        assert(record.crc32==golden_crc[v]);
        assert(MotorLoadRecord_Load(&record,&flags) && flags==v);
        /* 每一位单比特损坏都必须被拒绝；输出不能被部分覆盖。 */
        for(unsigned bit=0;bit<128;bit++) {
            MotorLoadRecord bad=record;
            ((unsigned char*)&bad)[bit/8]^=(unsigned char)(1U<<(bit%8)); flags=99;
            assert(!MotorLoadRecord_Load(&bad,&flags) && flags==99);
        }
        printf("record %u %08x %08x %08x %08x\n",v,record.magic,record.version,record.flags,record.crc32);
    }
    memset(&record,0,sizeof(record)); assert(MotorLoadRecord_Load(&record,&flags) && flags==0);
    memset(&record,255,sizeof(record)); assert(MotorLoadRecord_Load(&record,&flags) && flags==0);
    seed(&baseline);
    MotorControl.idRef=2; MotorControl.iqRef=3; MotorControl.speedRef=4;
    PI_Speed.Ui=.5f; FOC.id_pi.Ui=2; SensorlessStartup.state=SENSORLESS_STARTUP_HANDOFF;
    Fluxobserver.omega_e=100;
    assert(MotorControl_RequestLoadProfile(1)); assert(MotorControl_GetLoadProfile()==-1);
    assert(!MotorControl_RequestLoadProfile(0));
    MotorControl_ApplyLoadProfile();
    assert(MotorControl_GetLoadProfile()==1 && MotorControl.ModeNow==Motor_Disable);
    assert(MotorControl.idRef==0 && MotorControl.iqRef==0 && MotorControl.speedRef==0);
    assert(PI_Speed.Ui==0 && FOC.id_pi.Ui==0 && Fluxobserver.omega_e==0);
    assert(SensorlessStartup.state==SENSORLESS_STARTUP_IDLE && position_resets==1);
    assert(!MotorControl.friction_model_valid && friction_resets==1);
    check_preserved(&baseline);
    MotorControl.friction_model_valid=true;
    assert(MotorControl_RequestLoadProfile(3)); MotorControl_ApplyLoadProfile();
    assert(MotorControl.friction_model_valid && friction_resets==1);
    for(unsigned mode=1;mode<MODE_NUM;mode++) {
        MotorControl.ModeNow=(ModeNow_TypeDef)mode;
        assert(!MotorControl_RequestLoadProfile(0));
    }
    MotorControl.ModeNow=Motor_Disable; ModeLast=Speed_Mode;
    assert(!MotorControl_RequestLoadProfile(0)); ModeLast=Motor_Disable;
    gate_off=false; assert(!MotorControl_RequestLoadProfile(0)); gate_off=true;
    MotorControl.ErrorNow=Over_Current; assert(!MotorControl_RequestLoadProfile(0));
    MotorControl.ErrorNow=No_Error;
    assert(!MotorControl_RequestLoadProfile(2) && !MotorControl_RequestLoadProfile(UINT32_MAX));
    assert(MotorControl_RequestLoadProfile(0)); MotorControl.ModeNow=Speed_Mode;
    MotorControl_ApplyLoadProfile(); assert(MotorControl_GetLoadProfile()==3);
    MotorControl.ModeNow=Motor_Disable;
    assert(MotorControl_RequestLoadProfile(0)); gate_off=false;
    MotorControl_ApplyLoadProfile(); assert(MotorControl_GetLoadProfile()==3); gate_off=true;
    Param_Upload(&saved); saved.magic_word=MAGIC_WORD;
    MotorControl.load_profile_flags=0; assert(!Param_Download(&saved));
    assert(MotorControl_GetLoadProfile()==3 && MotorControl_IsConfigurationValid());
    check_preserved(&baseline);
    saved.load_profile.crc32^=1; assert(!Param_Download(&saved));
    assert(!MotorControl_IsConfigurationValid() && MotorControl_GetLoadProfile()==-1);
    assert(OnBoard_Encoder.calib_flag==7 && OnBoard_Encoder.linearization_lut_q15[71]==-42);
    saved=baseline; memset(&saved.load_profile,255,sizeof(saved.load_profile));
    assert(!Param_Download(&saved)); assert(MotorControl_GetLoadProfile()==0);
    assert(OnBoard_Encoder.electrical_zero_q15==12480);
    const float invalid[]={2,.5f,-1,NAN,INFINITY,1e30f};
    for(unsigned i=0;i<sizeof(invalid)/sizeof(invalid[0]);i++) {
        CAN_ReceiveMessage_Update(CAN_SET_LOAD_PROFILE,invalid[i]);
        assert(reply_id==CAN_GET_LOAD_PROFILE && reply_value==-1 && pending_load_profile==UINT32_MAX);
    }
    CAN_ReceiveMessage_Update(CAN_GET_LOAD_PROFILE_REVISION,0); assert(reply_value==1);
    CAN_ReceiveMessage_Update(CAN_SET_LOAD_PROFILE,1); MotorControl_ApplyLoadProfile();
    CAN_ReceiveMessage_Update(CAN_GET_LOAD_PROFILE,0); assert(reply_value==1);
    CAN_ReceiveMessage_Update(CAN_SET_LOAD_PROFILE,0); MotorControl_ApplyLoadProfile();
    assert(MotorControl_GetLoadProfile()==0 && !MotorControl.friction_model_valid);
    check_preserved(&baseline);
    puts("PASS dual runtime profiles, CRC/legacy, parameter preservation, switch ownership, stop guards, CAN values");
    return 0;
}
"""
    )


def main():
    """构建并执行离线回归，产物只写入 outputs。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", required=True)
    parser.add_argument("--out", type=Path, default=ROOT / "outputs/motor_load_profile")
    args = parser.parse_args()
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    (out / "main.h").write_text("#include <stdint.h>\n", encoding="utf-8")
    source = out / "load_profile_test.c"
    source.write_text(fixture(), encoding="utf-8")
    exe = out / "load_profile_test.exe"
    compiler = [args.cc] + (["cc"] if Path(args.cc).stem == "zig" else [])
    command = (
        compiler
        + ["-std=c99", "-O1", "-UNDEBUG", "-Wall", "-Wextra", "-Werror", "-I", str(out)]
        + NATIVE_INCLUDE_FLAGS
    )
    command += [
        str(source),
        str(ROOT / "firmware/motor/motor_load_profile.c"),
        str(ROOT / "firmware/services/parameters/motor_axis_profile.c"),
        str(ROOT / "firmware/motor/foc/foc_pid.c"),
        str(ROOT / "firmware/common/utils.c"),
        "-o",
        str(exe),
    ]
    logs = []
    for invocation in (command, [str(exe)]):
        result = subprocess.run(invocation, cwd=ROOT, capture_output=True, text=True)
        logs.append(result.stdout + result.stderr)
        (out / "test.log").write_text("\n".join(logs), encoding="utf-8")
        print(logs[-1], end="")
        result.check_returncode()


if __name__ == "__main__":
    main()
