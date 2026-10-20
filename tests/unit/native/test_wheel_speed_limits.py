"""参数归属定向检查：真实APP装载与Loader节点解析，全部I/O由内存替代。"""

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
#include "image_transfer.h"
#include "vector_mini_st_g431.h"
MotorControlState motor_state;
EncoderState rotor_encoder;
uint8_t Encoder_GetCalibFlag(const EncoderState *e) { return e->calib_flag; }
static uint8_t parameter_flash[TARGET_PARAM_BYTES];
void LoaderPlatform_Watchdog(void) {}
bool LoaderFlash_Read(uint32_t address, void *output, uint32_t count) {
    if(address < TARGET_PARAM_BASE || count > TARGET_IDENTITY_BASE - address) { return false; }
    memcpy(output, parameter_flash + address - TARGET_PARAM_BASE, count);
    return true;
}
"""
    vectors = json.loads(
        (ROOT / "docs/protocols/parameter_schema12_vectors.json").read_text(encoding="utf-8")
    )
    prelude += f"\n#define PARAM_RECORD_GOLDEN_CRC {vectors['vectors'][0]['crc32']}U\n"
    vectors13 = json.loads(
        (ROOT / "docs/protocols/parameter_schema13_vectors.json").read_text(encoding="utf-8")
    )
    prelude += f"\n#define PARAM_RECORD13_GOLDEN_CRC {vectors13['vectors'][0]['crc32']}U\n"
    vectors14 = json.loads(
        (ROOT / "docs/protocols/parameter_schema14_vectors.json").read_text(encoding="utf-8")
    )
    prelude += f"\n#define PARAM_RECORD14_GOLDEN_CRC {vectors14['vectors'][0]['crc32']}U\n"
    utils = (ROOT / "firmware/motor/math/control_math.c").read_text(encoding="utf-8")
    sensing = (ROOT / "firmware/motor/foc/foc_sensing.c").read_text(encoding="utf-8")
    prelude += (
        "\n#define CURRENT_OFFSET_MIN_COUNTS 1948.0f\n#define CURRENT_OFFSET_MAX_COUNTS 2148.0f\n"
    )
    prelude += function_source(sensing, "MotorCurrentOffsets_Valid")
    prelude += "\n".join(function_source(utils, n) for n in ("Math_Constrain", "Math_Min"))
    production = (ROOT / "firmware/services/parameters/foc_param.c").read_text(encoding="utf-8")
    prelude += re.sub(r"^#include.*$", "", production, flags=re.M)
    platform = (ROOT / "loader/ports/stm32g4/platform.c").read_text(encoding="utf-8")
    prelude += "\n" + function_source(platform, "LoaderPlatform_Parameters")
    policy = json.loads(
        (ROOT / "docs/protocols/parameter_node_application_v2_vectors.json").read_text(
            encoding="utf-8"
        )
    )
    prelude += "\nstatic void policy_golden_vectors(const InterfaceParam *base) {\n"
    for vector in policy["vectors"]:
        prelude += "{ uint8_t zero[4304]={0}, legacy[4304]={0}; InterfaceParam candidate=*base;\n"
        for offset, value in vector["fields"].items():
            values = ",".join(f"0x{byte:02x}" for byte in bytes.fromhex(value))
            prelude += (
                "{ const unsigned char bytes[]={"
                + values
                + "}; "
                + f"memcpy((unsigned char *)&zero+{offset},bytes,sizeof(bytes)); "
                + f"memcpy((unsigned char *)&candidate+{offset},bytes,sizeof(bytes)); }}\n"
            )
        prelude += f"assert(record_crc(zero,sizeof(zero))=={vector['zero_fill_crc32']}U);\n"
        prelude += (
            "candidate.record_crc32=Param_RecordCrc(&candidate); InterfaceParam before=candidate;\n"
        )
        prelude += "memcpy(legacy,&candidate,2240); candidate.record_crc32=record_crc(legacy,sizeof(legacy)); before=candidate; memcpy(legacy+2140,&candidate.record_crc32,4); Param_Download(legacy,sizeof(legacy),&motor_state,&rotor_encoder);\n"
        prelude += (
            f"assert(motor_state.device.node_id=={vector['new_node']}U && "
            f"motor_state.application.speed_limit_rad_s=={vector['new_loaded_speed_limit_rad_s']}f);\n"
        )
        prelude += "assert(memcmp(&before,&candidate,sizeof(candidate))==0); }\n"
    prelude += "}\n"
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
    /* 原默认路径先赋值 float 相模型，再乘带宽；比较全部 PI 的位模式。 */
    {
        volatile float resistance=PARAM_MOTOR_PHASE_RESISTANCE_OHM;
        volatile float ld=PARAM_MOTOR_D_INDUCTANCE_H;
        volatile float lq=PARAM_MOTOR_Q_INDUCTANCE_H;
        const float expected[]={ld*PARAM_MOTOR_CURRENT_LOOP_BANDWIDTH_RAD_S,
                                lq*PARAM_MOTOR_CURRENT_LOOP_BANDWIDTH_RAD_S,
                                resistance*PARAM_MOTOR_CURRENT_LOOP_BANDWIDTH_RAD_S,
                                resistance*PARAM_MOTOR_CURRENT_LOOP_BANDWIDTH_RAD_S};
        const float actual[]={motor_state.model.id_kp,motor_state.model.iq_kp,
                              motor_state.model.id_ki,motor_state.model.iq_ki};
        assert(memcmp(actual,expected,sizeof(expected))==0);
    }
    /* 各版有效记录的手调 PI 不随 R/L 重新派生，导出仍逐位保留。 */
    for(unsigned version=12;version<=14;version++) {
        p=saved; p.schema_version=version;
        p.motor_phase_resistance_ohm=1.25f;
        p.motor_d_inductance_h=.003f; p.motor_q_inductance_h=.004f;
        p.id_kp=.875f; p.iq_kp=1.625f; p.id_ki=2.375f; p.iq_ki=3.125f;
        const float expected[]={p.id_kp,p.iq_kp,p.id_ki,p.iq_ki};
        uint8_t input[4352]={0};
        unsigned bytes=version==12 ? 4304 : version==13 ? 4352 : sizeof(p);
        memcpy(input,&p,version==14 ? sizeof(p) : 2240);
        if(version==13) { memcpy(input+4304,(const uint8_t *)&p+2240,48); }
        uint32_t crc=record_crc(input,bytes); memcpy(input+2140,&crc,4);
        Param_Download(input,bytes,&motor_state,&rotor_encoder);
        const float actual[]={motor_state.model.id_kp,motor_state.model.iq_kp,
                              motor_state.model.id_ki,motor_state.model.iq_ki};
        assert(motor_state.configuration_source==MOTOR_CONFIGURATION_STORED);
        assert(memcmp(actual,expected,sizeof(expected))==0);
        Param_Upload(&p,&motor_state,&rotor_encoder);
        const float exported[]={p.id_kp,p.iq_kp,p.id_ki,p.iq_ki};
        assert(memcmp(exported,expected,sizeof(expected))==0);
    }
    puts("PASS default PI bit equivalence and stored schema12/13/14 custom PI retention");
    policy_golden_vectors(&saved);
    for(node=1; node<=254; node++) {
        float cap=PARAM_APP_SPEED_CEILING_RAD_S;
        p=saved; p.node_id=(float)node; p.speed_limit_rad_s=21;
        p.record_crc32=Param_RecordCrc(&p); Param_Download(&p,sizeof(p),&motor_state,&rotor_encoder); assert(motor_state.application.speed_limit_rad_s==PARAM_APP_DEFAULT_SPEED_LIMIT_RPS*_2PI); assert(motor_state.application.speed_ceiling_rad_s==cap);
        assert(motor_state.device.node_id==node && motor_state.application.axis_profile_valid);
        memset(parameter_flash,255,sizeof(parameter_flash)); memcpy(parameter_flash,&p,sizeof(p));
        uint8_t loader_node=0; assert(LoaderPlatform_Parameters(&loader_node)==YI_PARAMS_RAW11 && loader_node==node);
        assert(rotor_encoder.electrical_zero_q15==3155);
        assert(rotor_encoder.linearization_lut_q15[71]==-42);
        assert(motor_state.application.speed_kp==.02f);
        p.speed_limit_rad_s=1.25f; p.record_crc32=Param_RecordCrc(&p); Param_Download(&p,sizeof(p),&motor_state,&rotor_encoder); assert(motor_state.application.speed_limit_rad_s==1.25f);
        p.speed_limit_rad_s=NAN; p.record_crc32=Param_RecordCrc(&p); Param_Download(&p,sizeof(p),&motor_state,&rotor_encoder); assert(motor_state.application.speed_limit_rad_s==PARAM_APP_DEFAULT_SPEED_LIMIT_RPS*_2PI); assert(motor_state.application.speed_ceiling_rad_s==cap);
        p.speed_limit_rad_s=0; p.record_crc32=Param_RecordCrc(&p); Param_Download(&p,sizeof(p),&motor_state,&rotor_encoder); assert(motor_state.application.speed_limit_rad_s==PARAM_APP_DEFAULT_SPEED_LIMIT_RPS*_2PI); assert(motor_state.application.speed_ceiling_rad_s==cap);
    }
    puts("PASS 254 explicit APP/Loader addresses, independent application ceiling, low fallback and calibration");
    /* 版本化行为变化：旧node3把字段10钳到pi，新版保留显式字段10，不生成目标。 */
    p=saved; p.node_id=3; p.speed_limit_rad_s=10;
    p.record_crc32=Param_RecordCrc(&p); InterfaceParam address_before=p;
    Param_Download(&p,sizeof(p),&motor_state,&rotor_encoder);
    assert(motor_state.device.node_id==3 && motor_state.application.speed_limit_rad_s==10);
    assert(motor_state.application.speed_ceiling_rad_s==20 && memcmp(&p,&address_before,sizeof(p))==0);
    /* 轴配置仅收窄应用；同一地址不随AXS1/AXS2角色改变，输入始终不变。 */
    for(unsigned format=0;format<2;format++) {
        p=saved; p.node_id=7; p.speed_limit_rad_s=20;
        if(format==0) { assert(MotorAxisProfile_Create(&p.axis_profile,"roll",-1,1,.7f)); }
        else { assert(MotorAxisProfile_CreateJoint(&p.axis_profile,MOTOR_JOINT_ROLL,1,-.3f,.9f,.7f)); }
        p.record_crc32=Param_RecordCrc(&p); InterfaceParam before=p;
        Param_Download(&p,sizeof(p),&motor_state,&rotor_encoder);
        assert(motor_state.device.node_id==7 && motor_state.application.speed_limit_rad_s==.7f);
        assert(motor_state.application.speed_ceiling_rad_s==.7f && memcmp(&p,&before,sizeof(p))==0);
        memset(parameter_flash,255,sizeof(parameter_flash)); memcpy(parameter_flash,&p,sizeof(p));
        uint8_t loader_node=0; assert(LoaderPlatform_Parameters(&loader_node)==YI_PARAMS_RAW11 && loader_node==7);
    }
    {
        const float invalid_nodes[]={3.5f,NAN,INFINITY,-1.0f,255.0f};
        for(unsigned i=0;i<sizeof(invalid_nodes)/sizeof(invalid_nodes[0]);i++) {
            p=saved; p.node_id=invalid_nodes[i]; p.record_crc32=Param_RecordCrc(&p);
            Param_Download(&p,sizeof(p),&motor_state,&rotor_encoder);
            assert(!motor_state.configuration_confirmed && motor_state.configuration_source==MOTOR_CONFIGURATION_CORRUPT);
            memset(parameter_flash,255,sizeof(parameter_flash)); memcpy(parameter_flash,&p,sizeof(p));
            uint8_t loader_node=7; assert(LoaderPlatform_Parameters(&loader_node)==YI_PARAMS_UNSAFE && loader_node==7);
        }
    }
    puts("PASS AXS only narrows application and never overrides node; fractional addresses rejected by APP/Loader");
    /* 只接受当前版本：旧版、未知版和无效识别字均回退，不修改输入记录。 */
    {
        const uint32_t rejected[] = {0,1,2,3,4,5,6,7,8,9,10,11,15,UINT32_MAX};
        for(unsigned i=0;i<sizeof(rejected)/sizeof(rejected[0]);++i) {
            p=saved; p.schema_version=rejected[i];
            p.record_crc32=Param_RecordCrc(&p); InterfaceParam before=p;
            p.record_crc32=Param_RecordCrc(&p); Param_Download(&p,sizeof(p),&motor_state,&rotor_encoder);
            assert(memcmp(&p,&before,sizeof(p))==0);
            assert(motor_state.device.node_id==(uint8_t)PARAM_DEVICE_CAN_NODE_ID);
            assert(rotor_encoder.calib_flag==0 && rotor_encoder.electrical_zero_q15==0);
            assert(rotor_encoder.linearization_lut_q15[71]==0);
            assert(motor_state.application.cascade_position_kp==PARAM_APP_CASCADE_POSITION_KP);
            assert(motor_state.application.speed_kp==PARAM_APP_SPEED_KP);
        }
        p=saved; p.magic_word=0;
        p.record_crc32=Param_RecordCrc(&p); Param_Download(&p,sizeof(p),&motor_state,&rotor_encoder);
        assert(rotor_encoder.calib_flag==0);
    }
    puts("PASS old/unknown schema rejection, defaults and immutable input");
    /* 当前布局的黄金字节与偏移保持；加载保留既有整定和标定数据。 */
    p=saved; p.node_id=1; p.speed_limit_rad_s=20;
    p.configuration_flags=1U; p.reserved_pos_ki=0xffffffffU;
    p.record_crc32=0U; p.reserved_pos_integral_limit=0x80000000U;
    p.can_hb=NAN;
    p.cascade_pos_kp=.07f; p.cascade_pos_kd=.6f; p.speed_kp=.023f;
    {
        const uint8_t schema_bytes[] = {0x0e,0,0,0};
        const uint8_t magic_bytes[] = {0x32,0x43,0x4e,0x45};
        assert(sizeof(p)==2288 && offsetof(InterfaceParam,schema_version)==2160);
        assert(offsetof(InterfaceParam,thermal)==2240);
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
        p.record_crc32=Param_RecordCrc(&p); Param_Download(&p,sizeof(p),&motor_state,&rotor_encoder);
        assert(memcmp(&p,&before,sizeof(p))==0);
    }
    assert(motor_state.application.cascade_position_kp==.07f && motor_state.application.cascade_position_kd==.6f);
    assert(motor_state.application.speed_kp==.023f && motor_state.application.speed_limit_rad_s==20);
    assert(rotor_encoder.electrical_zero_q15==3155 && rotor_encoder.reverse==0);
    assert(rotor_encoder.calib_flag==3 && rotor_encoder.linearization_lut_q15[71]==-42);
    Param_Upload(&p,&motor_state,&rotor_encoder); p.magic_word=MAGIC_WORD;
    assert(p.configuration_flags==1U && p.record_crc32==Param_RecordCrc(&p) && p.reserved_pos_ki==0 && p.reserved_pos_integral_limit==0);
    {
        const uint8_t zero_bytes[] = {0,0,0,0};
        assert(memcmp((const uint8_t *)&p+2156,zero_bytes,sizeof(zero_bytes))==0);
    }
    /* 退役槽不应用到控制；导出清零，但schema12大小和CRC完整性保持。 */
    p=saved;
    p.friction_model_valid=1U; p.friction_coulomb_pos_a=.2f;
    p.record_crc32=Param_RecordCrc(&p);
    Param_Download(&p,sizeof(p),&motor_state,&rotor_encoder);
    assert(rotor_encoder.calib_flag==3 && rotor_encoder.linearization_lut_q15[71]==-42);
    Param_Upload(&p,&motor_state,&rotor_encoder);
    assert(p.friction_model_valid==0);
    puts("PASS schema12 golden layout and retired fields ignored/zero export");
    /* 当前硬件配置变化仍使电流域参数失效，不属于已删除的版本迁移。 */
    p=saved;
    p.current_sense_shunt_milliohm=CURRENT_SENSE_SHUNT_MILLIOHM==2U ? 6U : 2U;
    p.calibration_current_a=.1f; p.current_limit_a=.1f;
    p.friction_model_valid=1U;
    p.friction_coulomb_pos_a=.2f; p.friction_coulomb_neg_a=.2f;
    p.record_crc32=Param_RecordCrc(&p); Param_Download(&p,sizeof(p),&motor_state,&rotor_encoder);
    assert(motor_state.model.calibration_current_a==PARAM_MOTOR_CALIB_CURRENT_A);
    assert(motor_state.application.current_limit_a==PARAM_APP_DEFAULT_CURRENT_LIMIT_A);
    assert(rotor_encoder.calib_flag==3 && rotor_encoder.linearization_lut_q15[71]==-42);
    puts("PASS current-sense mismatch invalidates current-domain settings only");

    /* 独立 zlib 黄金值固定整条 CRC 契约；记录 CRC 槽不参与计算。 */
    uint8_t zero_legacy[4352]={0};
    uint32_t old_schema=12U, magic=MAGIC_WORD;
    memcpy(zero_legacy+2160,&old_schema,4); memcpy(zero_legacy+2164,&magic,4);
    assert(record_crc(zero_legacy,4304)==PARAM_RECORD_GOLDEN_CRC);
    old_schema=13U; memcpy(zero_legacy+2160,&old_schema,4);
    assert(record_crc(zero_legacy,4352)==PARAM_RECORD13_GOLDEN_CRC);
    memset(&p,0,sizeof(p)); p.schema_version=PARAM_SCHEMA_VERSION; p.magic_word=MAGIC_WORD;
    assert(Param_RecordCrc(&p)==PARAM_RECORD14_GOLDEN_CRC);
    p.record_crc32=UINT32_MAX; assert(Param_RecordCrc(&p)==PARAM_RECORD14_GOLDEN_CRC);
    Param_Return_Default(&motor_state,&rotor_encoder);
    assert(!motor_state.configuration_confirmed && rotor_encoder.calib_flag==0U);
    assert(motor_state.device.node_id==1U && motor_state.application.speed_limit_rad_s==PARAM_APP_DEFAULT_SPEED_LIMIT_RPS*_2PI);
    /* 运行DEFAULTS保留地址由完整owner夹具验证；本处验证该node7默认配置的真实存储边界。 */
    motor_state.device.node_id=7U;
    Param_Upload(&p,&motor_state,&rotor_encoder);
    assert(p.node_id==7.0f && p.configuration_flags==0U && p.record_crc32==Param_RecordCrc(&p));
    memset(parameter_flash,255,sizeof(parameter_flash)); memcpy(parameter_flash,&p,sizeof(p));
    uint8_t default_loader_node=0;
    assert(LoaderPlatform_Parameters(&default_loader_node)==YI_PARAMS_RAW11 && default_loader_node==7U);
    Param_Return_Default(&motor_state,&rotor_encoder);
    assert(motor_state.device.node_id==1U);
    Param_Download(&p,sizeof(p),&motor_state,&rotor_encoder);
    assert(motor_state.device.node_id==7U && !motor_state.configuration_confirmed && rotor_encoder.calib_flag==0U);
    assert(motor_state.application.speed_limit_rad_s==PARAM_APP_DEFAULT_SPEED_LIMIT_RPS*_2PI);
    puts("PASS node7 default configuration export/reload/Loader consistency; blank startup still node1");
    Param_Return_Default(&motor_state,&rotor_encoder);
    Param_Upload(&p,&motor_state,&rotor_encoder);
    Param_Download(&p,sizeof(p),&motor_state,&rotor_encoder);
    assert(!motor_state.configuration_confirmed && motor_state.configuration_source==MOTOR_CONFIGURATION_STORED);
    motor_state.configuration_confirmed=true; rotor_encoder.calib_flag=ENC_CALIB_LINEARIZED;
    Param_Upload(&p,&motor_state,&rotor_encoder); saved=p;
    Param_Return_Default(&motor_state,&rotor_encoder);
    Param_Download(&p,sizeof(p),&motor_state,&rotor_encoder);
    assert(motor_state.configuration_confirmed && rotor_encoder.calib_flag==ENC_CALIB_LINEARIZED);
    p.current_limit_a+=1.0f;
    Param_Download(&p,sizeof(p),&motor_state,&rotor_encoder);
    assert(!motor_state.configuration_confirmed && motor_state.configuration_source==MOTOR_CONFIGURATION_CORRUPT);
    assert(rotor_encoder.calib_flag==0U && motor_state.application.axis_profile_valid);
    p=saved; p.configuration_flags=2U; p.record_crc32=Param_RecordCrc(&p);
    Param_Download(&p,sizeof(p),&motor_state,&rotor_encoder);
    assert(!motor_state.configuration_confirmed && motor_state.configuration_source==MOTOR_CONFIGURATION_CORRUPT);
    p=saved; p.motor_flux_wb=NAN; p.record_crc32=Param_RecordCrc(&p);
    Param_Download(&p,sizeof(p),&motor_state,&rotor_encoder);
    assert(!motor_state.configuration_confirmed && motor_state.configuration_source==MOTOR_CONFIGURATION_CORRUPT);
    p=saved; p.node_id=0.0f; p.record_crc32=Param_RecordCrc(&p);
    Param_Download(&p,sizeof(p),&motor_state,&rotor_encoder);
    assert(!motor_state.configuration_confirmed && motor_state.device.node_id==1U && motor_state.configuration_source==MOTOR_CONFIGURATION_CORRUPT);
    puts("PASS independent schema12 CRC golden, unconfirmed/confirmed reload, partial calibration and corruption fallback");
    /* 旧schema12只读装载；尾部随机值不得应用，保存才升级13。 */
    uint8_t legacy[4352], original[4352];
    memset(legacy,0xa5,sizeof(legacy));
    p=saved; p.schema_version=12U; memcpy(legacy,&p,2240);
    uint32_t crc=record_crc(legacy,4304); memcpy(legacy+2140,&crc,4);
    memcpy(original,legacy,sizeof(legacy));
    Param_Download(legacy,sizeof(legacy),&motor_state,&rotor_encoder);
    assert(motor_state.configuration_source==MOTOR_CONFIGURATION_STORED);
    assert(motor_state.device.node_id==(uint8_t)p.node_id);
    assert(motor_state.model.thermal.continuous_vector_a==3.0f);
    assert(!motor_state.application.speed_feedforward.enabled);
    assert(memcmp(legacy,original,sizeof(legacy))==0);
    Param_Download(legacy,4303,&motor_state,&rotor_encoder);
    assert(motor_state.configuration_source==MOTOR_CONFIGURATION_CORRUPT);
    Param_Download(legacy,sizeof(legacy),&motor_state,&rotor_encoder);
    p=saved; p.schema_version=13U; memcpy(legacy,&p,2240);
    memcpy(legacy+4304,(const uint8_t *)&p+2240,48);
    crc=record_crc(legacy,4352); memcpy(legacy+2140,&crc,4);
    Param_Download(legacy,sizeof(legacy),&motor_state,&rotor_encoder);
    assert(motor_state.configuration_source==MOTOR_CONFIGURATION_STORED);
    assert(rotor_encoder.linearization_lut_q15[71]==p.encoder_linearization_lut_q15[71]);
    legacy[3000]^=1;
    Param_Download(legacy,sizeof(legacy),&motor_state,&rotor_encoder);
    assert(motor_state.configuration_source==MOTOR_CONFIGURATION_CORRUPT);
    legacy[3000]^=1;
    Param_Download(legacy,sizeof(legacy),&motor_state,&rotor_encoder);
    motor_state.model.thermal.peak_a=6.0f;
    motor_state.application.speed_feedforward.enabled=true;
    motor_state.application.speed_feedforward.gain_a_per_rad_s2=.04f;
    Param_Upload(&p,&motor_state,&rotor_encoder);
    assert(p.schema_version==14U && p.thermal.peak_a==6.0f && p.speed_feedforward_enabled==1U);
    Param_Return_Default(&motor_state,&rotor_encoder);
    Param_Download(&p,sizeof(p),&motor_state,&rotor_encoder);
    assert(motor_state.model.thermal.peak_a==6.0f);
    assert(motor_state.application.speed_feedforward.enabled);
    assert(motor_state.application.speed_feedforward.gain_a_per_rad_s2==.04f);
    p.thermal.peak_a=NAN; p.record_crc32=Param_RecordCrc(&p);
    Param_Download(&p,sizeof(p),&motor_state,&rotor_encoder);
    assert(motor_state.configuration_source==MOTOR_CONFIGURATION_CORRUPT && !motor_state.configuration_confirmed);
    puts("PASS schema12 bounded read and schema13 thermal/feedforward roundtrip; invalid extension rejected");
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
        str(ROOT / "shared/checksum/crc32.c"),
        str(ROOT / "firmware/services/parameters/parameter_value_service.c"),
        str(ROOT / "firmware/motor/protection/motor_thermal.c"),
        str(ROOT / "firmware/motor/protection/motor_thermal_config.c"),
        str(ROOT / "firmware/motor/foc/speed_feedforward.c"),
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
