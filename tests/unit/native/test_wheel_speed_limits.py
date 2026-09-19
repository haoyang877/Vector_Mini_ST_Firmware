"""轮子限速相关的主机回归：生产参数装载与 CAN 边界用例。"""

import sys as _sys
from pathlib import Path as _Path

_sys.path.insert(0, str(_Path(__file__).resolve().parents[3] / "tools"))
import argparse
import re
import subprocess
from pathlib import Path

from project_paths import NATIVE_INCLUDE_FLAGS, ROOT
from run_position_servo_tests import function_source


def command_case(path, name):
    source = (ROOT / path).read_text(encoding="utf-8")
    source = function_source(source, "CAN_ReceiveMessage_Update")
    start = source.index("case " + name + ":")
    end = source.index("break;", start) + len("break;")
    return source[start:end]


def fixture():
    prelude = r"""
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "foc_param.h"
#include "foc_param_profile.h"
#include "interface_can.h"
#include "utils.h"
#define MAGIC_WORD 0x454e4332U
MotorControl_TypeDef MotorControl;
Encoder_TypeDef OnBoard_Encoder;
uint8_t Encoder_GetCalibFlag(const Encoder_TypeDef *e) { return e->calib_flag; }
CANMsg_TypeDef CANMsg;
"""
    utils = (ROOT / "firmware/common/utils.c").read_text(encoding="utf-8")
    prelude += "\n".join(function_source(utils, n) for n in ("constrain", "fast_min"))
    production = (ROOT / "firmware/services/parameters/foc_param.c").read_text(encoding="utf-8")
    prelude += re.sub(r"^#include.*$", "", production, flags=re.M)
    prelude += "\nstatic void can_set(int id, float data) { int data_int=(int)data; switch(id) {\n"
    prelude += "\n".join(
        command_case("firmware/communication/can/interface_can.c", n)
        for n in ("CAN_SET_NODE_ID", "CAN_SET_SPEED_LIMIT")
    )
    return (
        prelude
        + "\n} }\n"
        + r"""
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
    /* Axis record wins over stale legacy node ID before selecting the cap. */
    p=saved; p.node_id=1; p.speed_limit=20;
    assert(MotorAxisProfile_CreateJoint(&p.axis_profile, MOTOR_JOINT_ROLL, 1, -.3f, .9f, .7f));
    Param_Download(&p);
    assert(CANMsg.node_id==3 && MotorControl.speed_limit==.5f*_2PI);
    puts("PASS resolved joint identity selects the ceiling before parameter loading");
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
    CANMsg.node_id=2; MotorControl.speedRef=20; Param_SetSpeedLimit(20);
    can_set(CAN_SET_NODE_ID,3);
    assert(CANMsg.node_id==3 && MotorControl.speed_limit==.5f*_2PI && MotorControl.speedRef==.5f*_2PI);
    can_set(CAN_SET_SPEED_LIMIT,20); assert(MotorControl.speed_limit==.5f*_2PI);
    can_set(CAN_SET_NODE_ID,1); assert(MotorControl.speed_limit==.5f*_2PI);
    Param_SetSpeedLimit(20); MotorControl.speedRef=-20;
    can_set(CAN_SET_NODE_ID,4);
    assert(CANMsg.node_id==4 && MotorControl.speed_limit==.5f*_2PI && MotorControl.speedRef==-.5f*_2PI);
    can_set(CAN_SET_SPEED_LIMIT,20); assert(MotorControl.speed_limit==.5f*_2PI);
    puts("PASS node changes reduce limits and never silently increase a stored lower limit");
    /* Schema 10 -> 11 preserves tuned gains and encoder calibration, never
     * interprets the erased old tail as a valid cogging map. */
    p=saved; p.schema_version=10; p.node_id=1; p.speed_limit=20;
    p.pos_kp=1.3f; p.pos_ki=.7f; p.pos_kd=.15f; p.speed_kp=.023f;
    memset(&p.cogging,0xff,sizeof(p.cogging));
    assert(Param_Download(&p));
    assert(MotorControl.pos_Kp==1.3f && MotorControl.pos_Ki==.7f && MotorControl.pos_Kd==.15f);
    assert(MotorControl.speed_Kp==.023f && MotorControl.speed_limit==20);
    assert(OnBoard_Encoder.electrical_zero_q15==3155 && OnBoard_Encoder.reverse==0);
    assert(CoggingMap.magic==0);
    Param_Upload(&p); p.magic_word=MAGIC_WORD;
    {
        CoggingCalibration c={0};
        uint32_t signature=Cogging_EncoderSignature(p.encoder_reverse,p.encoder_electrical_zero_q15,
            (int32_t)p.motor_pole_pairs,p.encoder_linearization_lut_q15);
        c.state=COGGING_COMPLETE; c.points_done=2048;
        c.full_scale_a=CURRENT_SENSE_PROFILE_FULL_SCALE_A;
        for(unsigned i=0;i<1024;i++) c.iq_q15[i]=(i&1) ? 123 : -123;
        assert(CoggingMap_Build(c.iq_q15,c.full_scale_a,signature,&CoggingMap));
        Param_Upload(&p); p.magic_word=MAGIC_WORD;
        memset(&CoggingMap,0,sizeof(CoggingMap));
        assert(!Param_Download(&p) && CoggingMap_Valid(&CoggingMap,signature));
        assert(CoggingMap.iq_q15[17]==123);
        p.cogging.iq_q15[5]^=1;
        Param_Download(&p); assert(CoggingMap.magic==0);
        p.cogging.iq_q15[5]^=1; p.encoder_electrical_zero_q15++;
        Param_Download(&p); assert(CoggingMap.magic==0);
    }
    puts("PASS schema migration, Q15 map round trip, corrupt/stale map rejection");
    return 0;
}
"""
    )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cc", required=True)
    ap.add_argument("--out", type=Path, default=ROOT / "outputs/wheel_speed_20260917/host")
    args = ap.parse_args()
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    (out / "main.h").write_text("#include <stdint.h>\n")
    source = out / "wheel_speed_test.c"
    source.write_text(fixture(), encoding="utf-8")
    cc = [str(Path(args.cc).resolve())] if Path(args.cc).is_file() else [args.cc]
    if Path(args.cc).stem == "zig":
        cc += ["cc"]
    cc += NATIVE_INCLUDE_FLAGS
    exe = out / "wheel_speed_test.exe"
    includes = [str(out)] + [
        str(ROOT / p)
        for p in (
            "firmware/common",
            "firmware/motor/foc",
            "firmware/platform/stm32g4/bsp",
            "firmware/communication",
        )
    ]
    command = cc + ["-std=c99", "-O1", "-UNDEBUG", "-Wall", "-Wextra", "-Werror"]
    command += [item for p in includes for item in ("-I", p)]
    command += [
        str(source),
        str(ROOT / "firmware/services/parameters/motor_axis_profile.c"),
        str(ROOT / "firmware/motor/identification/cogging_map.c"),
        str(ROOT / "firmware/common/crc32.c"),
        "-o",
        str(exe),
    ]
    logs = []
    for cmd in (command, [str(exe)]):
        result = subprocess.run(cmd, capture_output=True, text=True)
        logs.append(result.stdout + result.stderr)
        print(logs[-1], end="")
        (out / "test.log").write_text("\n".join(logs))
        result.check_returncode()


if __name__ == "__main__":
    main()
