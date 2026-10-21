"""Host regression of production parameter loading and CAN limit cases."""

import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(_Path(__file__).resolve().parents[3] / "tools"))
from project_paths import ROOT, NATIVE_INCLUDE_FLAGS

import argparse
from pathlib import Path
import re
import subprocess
from run_position_servo_tests import ROOT, function_source


def command_case(path, name):
    source = (ROOT / path).read_text(encoding="utf-8")
    source = function_source(source, "CAN_ReceiveMessage_Update")
    start = source.index("case " + name + ":")
    end = source.index("break;", start) + len("break;")
    return source[start:end]


def fixture():
    prelude = r'''
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "foc_param.h"
#include "foc_param_profile.h"
#include "interface_can.h"
#include "utils.h"
#include "motor_thermal_control.h"
MotorControl_TypeDef MotorControl;
Encoder_TypeDef OnBoard_Encoder;
CANMsg_TypeDef CANMsg;
ModeNow_TypeDef ModeLast;
volatile CAN_Diagnostics_TypeDef CAN_Diagnostics;
static bool phase_outputs_disabled = true;
static bool filter_succeeds = true;
static unsigned filter_calls;
static uint8_t filter_node;
bool motor_hw_phase_outputs_disabled(void) { return phase_outputs_disabled; }
bool comm_hw_can_set_node_filter(uint8_t node) {
    ++filter_calls;
    filter_node=node;
    return filter_succeeds;
}
void motor_hw_outer_barrier(void) { }
'''
    utils = (ROOT / "firmware/common/utils.c").read_text(encoding="utf-8")
    prelude += "\n".join(function_source(utils, n) for n in ("constrain", "fast_min"))
    production = (ROOT / "firmware/services/parameters/foc_param.c").read_text(encoding="utf-8")
    prelude += re.sub(r'^#include.*$', '', production, flags=re.M)
    can_source = (ROOT / "firmware/communication/can/interface_can.c").read_text(encoding="utf-8")
    prelude += '\n' + '\n'.join(function_source(can_source, name) for name in
                                ("CAN_Count", "CAN_SetNode"))
    prelude += '\nstatic void can_set(int id, float data) {\n'
    prelude += 'if (id==CAN_SET_NODE_ID) { CAN_SetNode(data); return; }\nswitch(id) {\n'
    prelude += command_case("firmware/communication/can/interface_can.c", "CAN_SET_SPEED_LIMIT")
    return prelude + '\n} }\n' + r'''
int main(void) {
    InterfaceParam_TypeDef p, saved;
    unsigned node;
    Param_Return_Default(); Param_Upload(&p); p.magic_word=MAGIC_WORD;
    p.encoder_electrical_zero_q15=3155; p.encoder_calib_flag=3;
    p.encoder_linearization_lut_q15[71]=-42; p.speed_kp=.02f;
    saved=p;
    for(node=0; node<=7; node++) {
        float cap=(node==1 || node==2) ? 20 : .5f*_2PI;
        assert(Param_SpeedLimitRadS(node)==cap);
        p=saved; p.node_id=(float)node; p.speed_limit=21;
        assert(!Param_Download(&p)); assert(MotorControl.speed_limit==cap);
        assert(CANMsg.node_id==node && MotorControl.axis_profile_valid);
        assert(OnBoard_Encoder.electrical_zero_q15==3155);
        assert(OnBoard_Encoder.linearization_lut_q15[71]==-42);
        assert(MotorControl.speed_Kp==.02f);
        p.speed_limit=1.25f; Param_Download(&p); assert(MotorControl.speed_limit==1.25f);
        p.speed_limit=NAN; Param_Download(&p); assert(MotorControl.speed_limit==cap);
        p.speed_limit=0; Param_Download(&p); assert(MotorControl.speed_limit==cap);
    }
    puts("PASS all node ceilings, stored lower limits, invalid fallback, calibration preserved");
    /* 临时联调时，合法 Flash 节点号优先于轴身份。 */
    p=saved; p.node_id=1; p.speed_limit=20;
    assert(MotorAxisProfile_CreateJoint(&p.axis_profile, MOTOR_JOINT_ROLL, 1, -.3f, .9f, .7f));
    Param_Download(&p);
    assert(CANMsg.node_id==1 && MotorControl.speed_limit==20);
    p.node_id=NAN; Param_Download(&p);
    assert(CANMsg.node_id==3 && MotorControl.speed_limit==.5f*_2PI);
    p=saved; p.node_id=3;
    assert(MotorAxisProfile_CreateJoint(&p.axis_profile, MOTOR_JOINT_YAW, 1,
        -3.141592654f, 3.141592654f, .5f));
    Param_Download(&p); assert(CANMsg.node_id==3 && MotorControl.axis_profile_valid);
    Param_Upload(&p); p.magic_word=MAGIC_WORD; assert(p.node_id==3);
    Param_Download(&p); assert(CANMsg.node_id==3);
    puts("PASS stored node survives yaw save/load; invalid node falls back to identity");
    for(node=1; node<=2; node++) {
        CANMsg.node_id=node; MotorControl.speedRef=25; MotorControl.pos_maxspeed=24;
        can_set(CAN_SET_SPEED_LIMIT,20);
        assert(MotorControl.speed_limit==20 && MotorControl.speedRef==20 && MotorControl.pos_maxspeed==20);
        can_set(CAN_SET_SPEED_LIMIT,20.01f); assert(MotorControl.speed_limit==20);
        assert(!Param_SetSpeedLimit(NAN) && !Param_SetSpeedLimit(INFINITY));
        assert(!Param_SetSpeedLimit(0) && !Param_SetSpeedLimit(-1));
        MotorControl.speedRef=-20;
        assert(Param_SetSpeedLimit(10) && MotorControl.speedRef==-10);
    }
    puts("PASS CAN rad/s boundaries; reduced limits clamp both command signs");
    MotorControl.ModeNow=Motor_Disable; ModeLast=Motor_Disable;
    CANMsg.node_id=2; Param_SetSpeedLimit(20);
    MotorControl.speedRef=19; MotorControl.pos_maxspeed=18;
    for(unsigned rejection=0; rejection<4; ++rejection) {
        MotorControl.ModeNow=(rejection==0) ? Speed_Mode : Motor_Disable;
        ModeLast=(rejection==1) ? Speed_Mode : Motor_Disable;
        phase_outputs_disabled=(rejection!=2);
        filter_succeeds=(rejection!=3);
        can_set(CAN_SET_NODE_ID,3);
        assert(CANMsg.node_id==2 && MotorControl.speed_limit==20);
        assert(MotorControl.speedRef==19 && MotorControl.pos_maxspeed==18);
        assert(filter_calls==(rejection==3 ? 1U : 0U));
    }
    assert(CAN_Diagnostics.node_rejected==3 && CAN_Diagnostics.node_filter_failed==1);
    MotorControl.ModeNow=Motor_Disable; ModeLast=Motor_Disable;
    phase_outputs_disabled=true; filter_succeeds=true;
    puts("PASS node admission/filter failures preserve node, limit and both commands");
    CANMsg.node_id=2; MotorControl.speedRef=20; Param_SetSpeedLimit(20);
    can_set(CAN_SET_NODE_ID,3);
    assert(filter_calls==2 && filter_node==3);
    assert(CANMsg.node_id==3 && MotorControl.speed_limit==.5f*_2PI && MotorControl.speedRef==.5f*_2PI);
    can_set(CAN_SET_SPEED_LIMIT,20); assert(MotorControl.speed_limit==.5f*_2PI);
    can_set(CAN_SET_NODE_ID,1); assert(MotorControl.speed_limit==.5f*_2PI);
    Param_SetSpeedLimit(20); MotorControl.speedRef=-20;
    can_set(CAN_SET_NODE_ID,4);
    assert(CANMsg.node_id==4 && MotorControl.speed_limit==.5f*_2PI && MotorControl.speedRef==-.5f*_2PI);
    can_set(CAN_SET_SPEED_LIMIT,20); assert(MotorControl.speed_limit==.5f*_2PI);
    puts("PASS node changes reduce limits and never silently increase a stored lower limit");
    return 0;
}
'''


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cc", required=True)
    ap.add_argument("--out", type=Path, default=ROOT / "outputs/wheel_speed_20260917/host")
    args = ap.parse_args()
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    (out / "main.h").write_text('#include <stdint.h>\n')
    source = out / "wheel_speed_test.c"
    source.write_text(fixture(), encoding="utf-8")
    cc = [str(Path(args.cc).resolve())] if Path(args.cc).is_file() else [args.cc]
    if Path(args.cc).stem == "zig":
        cc += ["cc"]
    cc += NATIVE_INCLUDE_FLAGS
    exe = out / "wheel_speed_test.exe"
    includes = [str(out)] + [str(ROOT / p) for p in ("firmware/common", "firmware/motor/foc", "firmware/platform/stm32g4/bsp", "firmware/communication")]
    command = cc + ["-std=c99", "-O1", "-UNDEBUG", "-Wall", "-Wextra", "-Werror"]
    command += [item for p in includes for item in ("-I", p)]
    command += [str(source), str(ROOT / "firmware/services/parameters/motor_axis_profile.c"), str(ROOT / "firmware/motor/motor_load_profile.c"), str(ROOT / "firmware/motor/speed_feedforward.c"), str(ROOT / "firmware/motor/protection/motor_thermal.c"), str(ROOT / "firmware/app/motor_thermal_control.c"), "-lm", "-o", str(exe)]
    logs = []
    for cmd in (command, [str(exe)]):
        result = subprocess.run(cmd, capture_output=True, text=True)
        logs.append(result.stdout + result.stderr)
        print(logs[-1], end="")
        (out / "test.log").write_text("\n".join(logs))
        result.check_returncode()


if __name__ == "__main__":
    main()
