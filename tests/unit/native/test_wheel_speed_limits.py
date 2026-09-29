"""轮子限速相关的主机回归：生产参数装载与业务限速边界用例。"""

import sys as _sys
from pathlib import Path as _Path

_sys.path.insert(0, str(_Path(__file__).resolve().parents[3] / "tools"))
import argparse
import json
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
#include "crc32.h"
#include "parameter_value_service.h"
#include "foc_sensing.h"
#include "motor_profile_defaults.h"
#include "board_profile_defaults.h"
#include "application_profile_defaults.h"
#include "device_profile_defaults.h"
#include "control_math.h"
#include "foc_cogging_calibration.h"
MotorControlState motor_state;
EncoderState rotor_encoder;
/* 参数服务只传值：夹具持有表，模拟齿槽边界并使用真实签名与 CRC。 */
static CoggingMapRecord CoggingMap;
void FocCogging_ResetMap(void) { memset(&CoggingMap,0,sizeof(CoggingMap)); }
void FocCogging_ImportMap(const CoggingMapRecord *record) { CoggingMap=*record; }
void FocCogging_ExportMap(CoggingMapRecord *record, const MotorControlState *m, const EncoderState *e) {
    uint32_t signature=Cogging_EncoderSignature(e->reverse,e->electrical_zero_q15,m->model.motor_pole_pairs,e->linearization_lut_q15);
    memset(record,0,sizeof(*record));
    if((e->calib_flag&ENC_CALIB_ALL)==ENC_CALIB_ALL && CoggingMap_Valid(&CoggingMap,signature)) { *record=CoggingMap; }
}
uint8_t Encoder_GetCalibFlag(const EncoderState *e) { return e->calib_flag; }
"""
    vectors = json.loads(
        (ROOT / "docs/protocols/parameter_schema12_vectors.json").read_text(encoding="utf-8")
    )
    prelude += f"\n#define PARAM_RECORD_GOLDEN_CRC {vectors['vectors'][0]['crc32']}U\n"
    utils = (ROOT / "firmware/motor/math/control_math.c").read_text(encoding="utf-8")
    sensing = (ROOT / "firmware/motor/foc/foc_sensing.c").read_text(encoding="utf-8")
    prelude += (
        "\n#define CURRENT_OFFSET_MIN_COUNTS 1948.0f\n#define CURRENT_OFFSET_MAX_COUNTS 2148.0f\n"
    )
    prelude += function_source(sensing, "MotorCurrentOffsets_Valid")
    prelude += "\n".join(function_source(utils, n) for n in ("Math_Constrain", "Math_Min"))
    production = (ROOT / "firmware/services/parameters/foc_param.c").read_text(encoding="utf-8")
    prelude += re.sub(r"^#include.*$", "", production, flags=re.M)
    return (
        prelude
        + r"""
int main(void) {
    InterfaceParam p, saved;
    unsigned node;
    Param_Return_Default(&motor_state,&rotor_encoder); Param_Upload(&p,&motor_state,&rotor_encoder); p.magic_word=MAGIC_WORD;
    p.encoder_electrical_zero_q15=3155; p.encoder_calib_flag=3;
    p.encoder_linearization_lut_q15[71]=-42; p.speed_kp=.02f;
    saved=p;
    for(node=1; node<=7; node++) {
        float cap=(node==1 || node==2) ? 20 : .5f*_2PI;

        p=saved; p.node_id=(float)node; p.speed_limit_rad_s=21;
        p.record_crc32=Param_RecordCrc(&p); Param_Download(&p,&motor_state,&rotor_encoder); assert(motor_state.application.speed_limit_rad_s==cap); assert(motor_state.application.speed_ceiling_rad_s==cap);
        assert(motor_state.device.node_id==node && motor_state.application.axis_profile_valid);
        assert(rotor_encoder.electrical_zero_q15==3155);
        assert(rotor_encoder.linearization_lut_q15[71]==-42);
        assert(motor_state.application.speed_kp==.02f);
        p.speed_limit_rad_s=1.25f; p.record_crc32=Param_RecordCrc(&p); Param_Download(&p,&motor_state,&rotor_encoder); assert(motor_state.application.speed_limit_rad_s==1.25f);
        p.speed_limit_rad_s=NAN; p.record_crc32=Param_RecordCrc(&p); Param_Download(&p,&motor_state,&rotor_encoder); assert(motor_state.application.speed_limit_rad_s==cap); assert(motor_state.application.speed_ceiling_rad_s==cap);
        p.speed_limit_rad_s=0; p.record_crc32=Param_RecordCrc(&p); Param_Download(&p,&motor_state,&rotor_encoder); assert(motor_state.application.speed_limit_rad_s==cap); assert(motor_state.application.speed_ceiling_rad_s==cap);
    }
    puts("PASS all node ceilings, stored lower limits, invalid fallback, calibration preserved");
    /* Axis record wins over stale legacy node ID before selecting the cap. */
    p=saved; p.node_id=1; p.speed_limit_rad_s=20;
    assert(MotorAxisProfile_CreateJoint(&p.axis_profile, MOTOR_JOINT_ROLL, 1, -.3f, .9f, .7f));
    p.record_crc32=Param_RecordCrc(&p); Param_Download(&p,&motor_state,&rotor_encoder);
    assert(motor_state.device.node_id==3 && motor_state.application.speed_limit_rad_s==.5f*_2PI);
    puts("PASS resolved joint identity selects the ceiling before parameter loading");
    /* 只接受当前版本：旧版、未知版和无效识别字均回退，不修改输入记录。 */
    {
        const uint32_t rejected[] = {0,1,2,3,4,5,6,7,8,9,10,11,13,UINT32_MAX};
        for(unsigned i=0;i<sizeof(rejected)/sizeof(rejected[0]);++i) {
            p=saved; p.schema_version=rejected[i];
            p.record_crc32=Param_RecordCrc(&p); InterfaceParam before=p;
            p.record_crc32=Param_RecordCrc(&p); Param_Download(&p,&motor_state,&rotor_encoder);
            assert(memcmp(&p,&before,sizeof(p))==0);
            assert(motor_state.device.node_id==(uint8_t)PARAM_DEVICE_CAN_NODE_ID);
            assert(rotor_encoder.calib_flag==0 && rotor_encoder.electrical_zero_q15==0);
            assert(rotor_encoder.linearization_lut_q15[71]==0 && CoggingMap.magic==0);
            assert(motor_state.application.cascade_position_kp==PARAM_APP_CASCADE_POSITION_KP);
            assert(motor_state.application.speed_kp==PARAM_APP_SPEED_KP);
        }
        p=saved; p.magic_word=0;
        p.record_crc32=Param_RecordCrc(&p); Param_Download(&p,&motor_state,&rotor_encoder);
        assert(rotor_encoder.calib_flag==0 && CoggingMap.magic==0);
    }
    puts("PASS old/unknown schema rejection, defaults and immutable input");
    /* 当前布局的黄金字节与偏移保持；加载保留既有整定和标定数据。 */
    p=saved; p.node_id=1; p.speed_limit_rad_s=20;
    p.configuration_flags=1U; p.reserved_pos_ki=0xffffffffU;
    p.record_crc32=0U; p.reserved_pos_integral_limit=0x80000000U;
    p.can_hb=NAN;
    p.cascade_pos_kp=.07f; p.cascade_pos_kd=.6f; p.speed_kp=.023f;
    {
        const uint8_t schema_bytes[] = {0x0c,0,0,0};
        const uint8_t magic_bytes[] = {0x32,0x43,0x4e,0x45};
        assert(sizeof(p)==4304 && offsetof(InterfaceParam,schema_version)==2160);
        assert(offsetof(InterfaceParam,magic_word)==2164);
        assert(offsetof(InterfaceParam,can_hb)==2156);
        assert(offsetof(InterfaceParam,configuration_flags)==2136);
        assert(offsetof(InterfaceParam,record_crc32)==2140);
        assert(offsetof(InterfaceParam,reserved_pos_ki)==2168);
        assert(offsetof(InterfaceParam,reserved_pos_integral_limit)==2176);
        assert(memcmp((const uint8_t *)&p+2160,schema_bytes,sizeof(schema_bytes))==0);
        assert(memcmp((const uint8_t *)&p+2164,magic_bytes,sizeof(magic_bytes))==0);
    }
    {
        p.record_crc32=Param_RecordCrc(&p); InterfaceParam before=p;
        p.record_crc32=Param_RecordCrc(&p); Param_Download(&p,&motor_state,&rotor_encoder);
        assert(memcmp(&p,&before,sizeof(p))==0);
    }
    assert(motor_state.application.cascade_position_kp==.07f && motor_state.application.cascade_position_kd==.6f);
    assert(motor_state.application.speed_kp==.023f && motor_state.application.speed_limit_rad_s==20);
    assert(rotor_encoder.electrical_zero_q15==3155 && rotor_encoder.reverse==0);
    assert(rotor_encoder.calib_flag==3 && rotor_encoder.linearization_lut_q15[71]==-42);
    assert(CoggingMap.magic==0);
    Param_Upload(&p,&motor_state,&rotor_encoder); p.magic_word=MAGIC_WORD;
    assert(p.configuration_flags==1U && p.record_crc32==Param_RecordCrc(&p) && p.reserved_pos_ki==0 && p.reserved_pos_integral_limit==0);
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
        Param_Upload(&p,&motor_state,&rotor_encoder); p.magic_word=MAGIC_WORD;
        memset(&CoggingMap,0,sizeof(CoggingMap));
        p.record_crc32=Param_RecordCrc(&p); Param_Download(&p,&motor_state,&rotor_encoder); assert(CoggingMap_Valid(&CoggingMap,signature));
        assert(CoggingMap.iq_q15[17]==123);
        p.cogging.iq_q15[5]^=1;
        p.record_crc32=Param_RecordCrc(&p); Param_Download(&p,&motor_state,&rotor_encoder); assert(CoggingMap.magic==0);
        p.cogging.iq_q15[5]^=1; p.encoder_electrical_zero_q15++;
        p.record_crc32=Param_RecordCrc(&p); Param_Download(&p,&motor_state,&rotor_encoder); assert(CoggingMap.magic==0);
    }
    puts("PASS schema12 golden layout, Q15 map round trip, corrupt/stale map rejection");
    /* 当前硬件配置变化仍使电流域参数失效，不属于已删除的版本迁移。 */
    p=saved;
    p.current_sense_shunt_milliohm=CURRENT_SENSE_SHUNT_MILLIOHM==2U ? 6U : 2U;
    p.calibration_current_a=.1f; p.current_limit_a=.1f;
    p.friction_model_valid=1U;
    p.friction_coulomb_pos_a=.2f; p.friction_coulomb_neg_a=.2f;
    p.record_crc32=Param_RecordCrc(&p); Param_Download(&p,&motor_state,&rotor_encoder);
    assert(motor_state.model.calibration_current_a==PARAM_MOTOR_CALIB_CURRENT_A);
    assert(motor_state.application.current_limit_a==PARAM_MOTOR_CURRENT_LIMIT_A);
    assert(!motor_state.device.friction_model_valid && CoggingMap.magic==0);
    assert(rotor_encoder.calib_flag==3 && rotor_encoder.linearization_lut_q15[71]==-42);
    puts("PASS current-sense mismatch invalidates current-domain settings only");

    /* 独立 zlib 黄金值固定整条 CRC 契约；记录 CRC 槽不参与计算。 */
    memset(&p,0,sizeof(p)); p.schema_version=12U; p.magic_word=MAGIC_WORD;
    assert(Param_RecordCrc(&p)==PARAM_RECORD_GOLDEN_CRC);
    p.record_crc32=UINT32_MAX; assert(Param_RecordCrc(&p)==PARAM_RECORD_GOLDEN_CRC);
    Param_Return_Default(&motor_state,&rotor_encoder);
    assert(!motor_state.configuration_confirmed && rotor_encoder.calib_flag==0U);
    assert(motor_state.device.node_id==1U && motor_state.application.speed_limit_rad_s==PARAM_MOTOR_SPEED_LIMIT_RPS*_2PI);
    Param_Upload(&p,&motor_state,&rotor_encoder);
    Param_Download(&p,&motor_state,&rotor_encoder);
    assert(!motor_state.configuration_confirmed && motor_state.configuration_source==MOTOR_CONFIGURATION_STORED);
    motor_state.configuration_confirmed=true; rotor_encoder.calib_flag=ENC_CALIB_LINEARIZED;
    Param_Upload(&p,&motor_state,&rotor_encoder); saved=p;
    Param_Return_Default(&motor_state,&rotor_encoder);
    Param_Download(&p,&motor_state,&rotor_encoder);
    assert(motor_state.configuration_confirmed && rotor_encoder.calib_flag==ENC_CALIB_LINEARIZED);
    p.current_limit_a+=1.0f;
    Param_Download(&p,&motor_state,&rotor_encoder);
    assert(!motor_state.configuration_confirmed && motor_state.configuration_source==MOTOR_CONFIGURATION_CORRUPT);
    assert(rotor_encoder.calib_flag==0U && motor_state.application.axis_profile_valid);
    p=saved; p.configuration_flags=2U; p.record_crc32=Param_RecordCrc(&p);
    Param_Download(&p,&motor_state,&rotor_encoder);
    assert(!motor_state.configuration_confirmed && motor_state.configuration_source==MOTOR_CONFIGURATION_CORRUPT);
    p=saved; p.motor_flux_wb=NAN; p.record_crc32=Param_RecordCrc(&p);
    Param_Download(&p,&motor_state,&rotor_encoder);
    assert(!motor_state.configuration_confirmed && motor_state.configuration_source==MOTOR_CONFIGURATION_CORRUPT);
    p=saved; p.node_id=0.0f; p.record_crc32=Param_RecordCrc(&p);
    Param_Download(&p,&motor_state,&rotor_encoder);
    assert(!motor_state.configuration_confirmed && motor_state.device.node_id==1U && motor_state.configuration_source==MOTOR_CONFIGURATION_CORRUPT);
    puts("PASS independent schema12 CRC golden, unconfirmed/confirmed reload, partial calibration and corruption fallback");
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
            "firmware/motor/math",
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
        str(ROOT / "firmware/motor/calibration/cogging_map.c"),
        str(ROOT / "shared/checksum/crc32.c"),
        str(ROOT / "firmware/services/parameters/parameter_value_service.c"),
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
