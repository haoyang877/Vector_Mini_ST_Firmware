"""轮子限速相关的主机回归：生产参数装载与业务限速边界用例。"""

import sys as _sys
from pathlib import Path as _Path

_sys.path.insert(0, str(_Path(__file__).resolve().parents[3] / "tools"))
import argparse
import re
import subprocess
from pathlib import Path

from project_paths import NATIVE_INCLUDE_FLAGS, ROOT
from run_position_servo_tests import function_source


def fixture():
    prelude = r"""
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "foc_param.h"
#include "foc_param_profile.h"
#include "param_comm_bridge.h"
#include "utils.h"
#include "foc_cogging_calibration.h"
MotorControl_TypeDef MotorControl;
Encoder_TypeDef OnBoard_Encoder;
/* 参数服务只传值：夹具持有表，模拟齿槽边界并使用真实签名与 CRC。 */
static CoggingMapRecord CoggingMap;
void FocCogging_ResetMap(void) { memset(&CoggingMap,0,sizeof(CoggingMap)); }
void FocCogging_ImportMap(const CoggingMapRecord *record) { CoggingMap=*record; }
void FocCogging_ExportMap(CoggingMapRecord *record, const MotorControl_TypeDef *m, const Encoder_TypeDef *e) {
    uint32_t signature=Cogging_EncoderSignature(e->reverse,e->electrical_zero_q15,m->motor_pole_pairs,e->linearization_lut_q15);
    memset(record,0,sizeof(*record));
    if((e->calib_flag&ENC_CALIB_ALL)==ENC_CALIB_ALL && CoggingMap_Valid(&CoggingMap,signature)) { *record=CoggingMap; }
}
uint8_t Encoder_GetCalibFlag(const Encoder_TypeDef *e) { return e->calib_flag; }
static uint8_t node_id;
uint8_t Protocol_NodeId_Get(void) { return node_id; }
void Protocol_NodeId_Set(uint8_t value) { node_id = value; }
"""
    utils = (ROOT / "firmware/common/utils.c").read_text(encoding="utf-8")
    prelude += "\n".join(function_source(utils, n) for n in ("constrain", "fast_min"))
    production = (ROOT / "firmware/services/parameters/foc_param.c").read_text(encoding="utf-8")
    prelude += re.sub(r"^#include.*$", "", production, flags=re.M)
    return (
        prelude
        + r"""
int main(void) {
    InterfaceParam_TypeDef p, saved;
    unsigned node;
    Param_Return_Default(&MotorControl,&OnBoard_Encoder); Param_Upload(&p,&MotorControl,&OnBoard_Encoder); p.magic_word=MAGIC_WORD;
    p.encoder_electrical_zero_q15=3155; p.encoder_calib_flag=3;
    p.encoder_linearization_lut_q15[71]=-42; p.speed_kp=.02f;
    saved=p;
    for(node=0; node<=7; node++) {
        float cap=(node==1 || node==2) ? 20 : .5f*_2PI;
        assert(Param_SpeedLimitRadS(node)==cap);
        p=saved; p.node_id=(float)node; p.speed_limit=21;
        Param_Download(&p,&MotorControl,&OnBoard_Encoder); assert(MotorControl.speed_limit==cap);
        assert(node_id==node && MotorControl.axis_profile_valid);
        assert(OnBoard_Encoder.electrical_zero_q15==3155);
        assert(OnBoard_Encoder.linearization_lut_q15[71]==-42);
        assert(MotorControl.speed_Kp==.02f);
        p.speed_limit=1.25f; Param_Download(&p,&MotorControl,&OnBoard_Encoder); assert(MotorControl.speed_limit==1.25f);
        p.speed_limit=NAN; Param_Download(&p,&MotorControl,&OnBoard_Encoder); assert(MotorControl.speed_limit==cap);
        p.speed_limit=0; Param_Download(&p,&MotorControl,&OnBoard_Encoder); assert(MotorControl.speed_limit==cap);
    }
    puts("PASS all node ceilings, stored lower limits, invalid fallback, calibration preserved");
    /* Axis record wins over stale legacy node ID before selecting the cap. */
    p=saved; p.node_id=1; p.speed_limit=20;
    assert(MotorAxisProfile_CreateJoint(&p.axis_profile, MOTOR_JOINT_ROLL, 1, -.3f, .9f, .7f));
    Param_Download(&p,&MotorControl,&OnBoard_Encoder);
    assert(node_id==3 && MotorControl.speed_limit==.5f*_2PI);
    puts("PASS resolved joint identity selects the ceiling before parameter loading");
    /* 只接受当前版本：旧版、未知版和无效识别字均回退，不修改输入记录。 */
    {
        const uint32_t rejected[] = {0,1,2,3,4,5,6,7,8,9,10,12,UINT32_MAX};
        for(unsigned i=0;i<sizeof(rejected)/sizeof(rejected[0]);++i) {
            p=saved; p.schema_version=rejected[i];
            InterfaceParam_TypeDef before=p;
            Param_Download(&p,&MotorControl,&OnBoard_Encoder);
            assert(memcmp(&p,&before,sizeof(p))==0);
            assert(node_id==(uint8_t)PARAM_HW_CAN_NODE_ID);
            assert(OnBoard_Encoder.calib_flag==0 && OnBoard_Encoder.electrical_zero_q15==0);
            assert(OnBoard_Encoder.linearization_lut_q15[71]==0 && CoggingMap.magic==0);
            assert(MotorControl.cascade_pos_Kp==PARAM_APP_CASCADE_POSITION_KP);
            assert(MotorControl.speed_Kp==PARAM_APP_SPEED_KP);
        }
        p=saved; p.magic_word=0;
        Param_Download(&p,&MotorControl,&OnBoard_Encoder);
        assert(OnBoard_Encoder.calib_flag==0 && CoggingMap.magic==0);
    }
    puts("PASS old/unknown schema rejection, defaults and immutable input");
    /* 当前布局的黄金字节与偏移保持；加载保留既有整定和标定数据。 */
    p=saved; p.node_id=1; p.speed_limit=20;
    p.reserved_pos_kp=0x7fc00000U; p.reserved_pos_ki=0xffffffffU;
    p.reserved_pos_kd=0x7f800000U; p.reserved_pos_integral_limit=0x80000000U;
    p.can_hb=NAN;
    p.cascade_pos_kp=.07f; p.cascade_pos_kd=.6f; p.speed_kp=.023f;
    {
        const uint8_t schema_bytes[] = {0x0b,0,0,0};
        const uint8_t magic_bytes[] = {0x32,0x43,0x4e,0x45};
        assert(sizeof(p)==4304 && offsetof(InterfaceParam_TypeDef,schema_version)==2160);
        assert(offsetof(InterfaceParam_TypeDef,magic_word)==2164);
        assert(offsetof(InterfaceParam_TypeDef,can_hb)==2156);
        assert(offsetof(InterfaceParam_TypeDef,reserved_pos_kp)==2136);
        assert(offsetof(InterfaceParam_TypeDef,reserved_pos_kd)==2140);
        assert(offsetof(InterfaceParam_TypeDef,reserved_pos_ki)==2168);
        assert(offsetof(InterfaceParam_TypeDef,reserved_pos_integral_limit)==2176);
        assert(memcmp((const uint8_t *)&p+2160,schema_bytes,sizeof(schema_bytes))==0);
        assert(memcmp((const uint8_t *)&p+2164,magic_bytes,sizeof(magic_bytes))==0);
    }
    {
        InterfaceParam_TypeDef before=p;
        Param_Download(&p,&MotorControl,&OnBoard_Encoder);
        assert(memcmp(&p,&before,sizeof(p))==0);
    }
    assert(MotorControl.cascade_pos_Kp==.07f && MotorControl.cascade_pos_Kd==.6f);
    assert(MotorControl.speed_Kp==.023f && MotorControl.speed_limit==20);
    assert(OnBoard_Encoder.electrical_zero_q15==3155 && OnBoard_Encoder.reverse==0);
    assert(OnBoard_Encoder.calib_flag==3 && OnBoard_Encoder.linearization_lut_q15[71]==-42);
    assert(CoggingMap.magic==0);
    Param_Upload(&p,&MotorControl,&OnBoard_Encoder); p.magic_word=MAGIC_WORD;
    assert(p.reserved_pos_kp==0 && p.reserved_pos_kd==0 && p.reserved_pos_ki==0 && p.reserved_pos_integral_limit==0);
    {
        const uint8_t zero_bytes[] = {0,0,0,0};
        assert(memcmp((const uint8_t *)&p+2156,zero_bytes,sizeof(zero_bytes))==0);
    }
    {
        CoggingCalibration c={0};
        uint32_t signature=Cogging_EncoderSignature(p.encoder_reverse,p.encoder_electrical_zero_q15,
            (int32_t)p.motor_pole_pairs,p.encoder_linearization_lut_q15);
        c.state=COGGING_COMPLETE; c.points_done=2048;
        c.full_scale_a=CURRENT_SENSE_PROFILE_FULL_SCALE_A;
        for(unsigned i=0;i<1024;i++) c.iq_q15[i]=(i&1) ? 123 : -123;
        assert(CoggingMap_Build(c.iq_q15,c.full_scale_a,signature,&CoggingMap));
        Param_Upload(&p,&MotorControl,&OnBoard_Encoder); p.magic_word=MAGIC_WORD;
        memset(&CoggingMap,0,sizeof(CoggingMap));
        Param_Download(&p,&MotorControl,&OnBoard_Encoder); assert(CoggingMap_Valid(&CoggingMap,signature));
        assert(CoggingMap.iq_q15[17]==123);
        p.cogging.iq_q15[5]^=1;
        Param_Download(&p,&MotorControl,&OnBoard_Encoder); assert(CoggingMap.magic==0);
        p.cogging.iq_q15[5]^=1; p.encoder_electrical_zero_q15++;
        Param_Download(&p,&MotorControl,&OnBoard_Encoder); assert(CoggingMap.magic==0);
    }
    puts("PASS schema11 golden layout, Q15 map round trip, corrupt/stale map rejection");
    /* 当前硬件配置变化仍使电流域参数失效，不属于已删除的版本迁移。 */
    p=saved;
    p.current_sense_shunt_milliohm=CURRENT_SENSE_SHUNT_MILLIOHM==2U ? 6U : 2U;
    p.calib_current=.1f; p.current_limit=.1f;
    p.friction_model_valid=1U;
    p.friction_coulomb_pos_a=.2f; p.friction_coulomb_neg_a=.2f;
    Param_Download(&p,&MotorControl,&OnBoard_Encoder);
    assert(MotorControl.calib_current==PARAM_MOTOR_CALIB_CURRENT_A);
    assert(MotorControl.current_limit==PARAM_MOTOR_CURRENT_LIMIT_A);
    assert(!MotorControl.friction_model_valid && CoggingMap.magic==0);
    assert(OnBoard_Encoder.calib_flag==3 && OnBoard_Encoder.linearization_lut_q15[71]==-42);
    puts("PASS current-sense mismatch invalidates current-domain settings only");
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
