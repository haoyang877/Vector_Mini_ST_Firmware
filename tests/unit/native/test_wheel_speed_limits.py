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
#include "parameter_record.h"
#include "motor_axis_profile.h"
#include "application_config.h"
#include "crc32.h"
#include "parameter_values.h"
#include "motor_sensing.h"
#include "application_config.h"
#include "board_config.h"
#include "board_config.h"
#include "application_config.h"
#include "application_config.h"
#include "control_math.h"
#include "image_transfer.h"
#include "vector_mini_st_g431.h"
#include "parameters.h"
MotorConfiguration configuration;
static uint8_t parameter_flash[TARGET_PARAM_BYTES];
void LoaderPlatform_Watchdog(void) {}
bool LoaderPlatform_ReadParameters(uint32_t offset, void *output, uint32_t count) {
    uint32_t address = TARGET_PARAM_BASE + offset;
    if(address < TARGET_PARAM_BASE || count > TARGET_IDENTITY_BASE - address) { return false; }
    memcpy(output, parameter_flash + address - TARGET_PARAM_BASE, count);
    return true;
}
"""
    vectors14 = json.loads(
        (ROOT / "docs/protocols/parameter_schema14_vectors.json").read_text(encoding="utf-8")
    )
    prelude += f"\n#define PARAM_RECORD14_GOLDEN_CRC {vectors14['vectors'][0]['crc32']}U\n"
    utils = (ROOT / "firmware/motor/math/control_math.c").read_text(encoding="utf-8")
    sensing = (ROOT / "firmware/motor/sensing/motor_current.c").read_text(encoding="utf-8")
    prelude += (
        "\n#define CURRENT_OFFSET_MIN_COUNTS 1948.0f\n#define CURRENT_OFFSET_MAX_COUNTS 2148.0f\n"
    )
    prelude += function_source(sensing, "MotorCurrentOffsets_IsValid")
    prelude += "\n".join(function_source(utils, n) for n in ("Math_Constrain", "Math_Min"))
    production = (ROOT / "firmware/parameters/parameter_record.c").read_text(encoding="utf-8")
    prelude += re.sub(r"^#include.*$", "", production, flags=re.M)
    prelude += "\n" + (ROOT / "loader/product/parameters.c").read_text(encoding="utf-8")
    policy = json.loads(
        (ROOT / "docs/protocols/parameter_node_application_v2_vectors.json").read_text(
            encoding="utf-8"
        )
    )
    prelude += "\nstatic void policy_golden_vectors(const InterfaceParam *base) {\n"
    for vector in policy["vectors"]:
        prelude += "{ InterfaceParam candidate=*base;\n"
        for offset, value in vector["fields"].items():
            if int(offset) == 2160:
                continue  # 固定向量的旧版号不覆盖当前记录版本。
            values = ",".join(f"0x{byte:02x}" for byte in bytes.fromhex(value))
            prelude += (
                "{ const unsigned char bytes[]={"
                + values
                + "}; "
                + f"memcpy((unsigned char *)&candidate+{offset},bytes,sizeof(bytes)); }}\n"
            )
        prelude += (
            "candidate.record_crc32=Param_RecordCrc(&candidate); InterfaceParam before=candidate;\n"
        )
        prelude += "Param_Download(&candidate,sizeof(candidate),&configuration);\n"
        prelude += (
            f"assert(configuration.device.node_id=={vector['new_node']}U && "
            f"configuration.application.speed_limit_rad_s=={vector['new_loaded_speed_limit_rad_s']}f);\n"
        )
        prelude += "assert(memcmp(&before,&candidate,sizeof(candidate))==0); }\n"
    prelude += "}\n"
    return (
        prelude
        + r"""
int main(void) {
    InterfaceParam p, saved;
    unsigned node;
    Param_Return_Default(&configuration); Param_Upload(&p,&configuration); p.magic_word=MAGIC_WORD;
    p.encoder_electrical_zero_q15=3155; p.encoder_calib_flag=3;
    p.encoder_linearization_lut_q15[71]=-42; p.speed_kp=.02f;
    saved=p;
    /* 常量归位不能改变已发布默认值；期望值固定为本轮基线。 */
    {
        const MotorWindingThermalConfig *thermal=&configuration.model.thermal;
        const SpeedFeedforwardConfig *feedforward=&configuration.application.speed_feedforward;
        const float actual[]={thermal->continuous_vector_a,thermal->peak_a,thermal->heating_tau_s,
            thermal->derating_enter_heat,thermal->derating_exit_heat,thermal->trip_heat,thermal->clear_heat,
            feedforward->gain_a_per_rad_s2,feedforward->acceleration_limit_rad_s2,
            feedforward->jerk_limit_rad_s3,feedforward->current_limit_a,
            configuration.application.speed_limit_rad_s,configuration.application.speed_acceleration_rad_s2,
            configuration.application.speed_deceleration_rad_s2,configuration.application.position_acceleration_rad_s2,
            configuration.application.position_deceleration_rad_s2,configuration.application.position_maximum_speed_rad_s};
        const float expected[]={3.0f,7.0f,65.98486785f,.8f,.7f,1.0f,.5f,
            .028f,20.0f,200.0f,2.0f,.5f*_2PI,50.0f*_2PI,50.0f*_2PI,.125f*_2PI,.125f*_2PI,.125f*_2PI};
        assert(!feedforward->enabled && memcmp(actual,expected,sizeof(expected))==0);
    }
    /* 原默认路径先赋值 float 相模型，再乘带宽；比较全部 PI 的位模式。 */
    {
        volatile float resistance=PARAM_MOTOR_PHASE_RESISTANCE_OHM;
        volatile float ld=PARAM_MOTOR_D_INDUCTANCE_H;
        volatile float lq=PARAM_MOTOR_Q_INDUCTANCE_H;
        const float expected[]={ld*CURRENT_LOOP_BANDWIDTH_RAD_S,
                                lq*CURRENT_LOOP_BANDWIDTH_RAD_S,
                                resistance*CURRENT_LOOP_BANDWIDTH_RAD_S,
                                resistance*CURRENT_LOOP_BANDWIDTH_RAD_S};
        const float actual[]={configuration.model.id_kp,configuration.model.iq_kp,
                              configuration.model.id_ki,configuration.model.iq_ki};
        assert(memcmp(actual,expected,sizeof(expected))==0);
    }
    /* 当前有效记录的手调 PI 不随 R/L 重新派生，导出仍逐位保留。 */
    {
        p=saved;
        p.motor_phase_resistance_ohm=1.25f;
        p.motor_d_inductance_h=.003f; p.motor_q_inductance_h=.004f;
        p.id_kp=.875f; p.iq_kp=1.625f; p.id_ki=2.375f; p.iq_ki=3.125f;
        const float expected[]={p.id_kp,p.iq_kp,p.id_ki,p.iq_ki};
        p.record_crc32=Param_RecordCrc(&p);
        Param_Download(&p,sizeof(p),&configuration);
        const float actual[]={configuration.model.id_kp,configuration.model.iq_kp,
                              configuration.model.id_ki,configuration.model.iq_ki};
        assert(configuration.configuration_source==MOTOR_CONFIGURATION_STORED);
        assert(memcmp(actual,expected,sizeof(expected))==0);
        Param_Upload(&p,&configuration);
        const float exported[]={p.id_kp,p.iq_kp,p.id_ki,p.iq_ki};
        assert(memcmp(exported,expected,sizeof(expected))==0);
    }
    puts("PASS default PI bit equivalence and stored schema14 custom PI retention");
    policy_golden_vectors(&saved);
    for(node=1; node<=254; node++) {
        float cap=PARAM_APP_SPEED_CEILING_RAD_S;
        p=saved; p.node_id=(float)node; p.speed_limit_rad_s=21;
        p.record_crc32=Param_RecordCrc(&p); Param_Download(&p,sizeof(p),&configuration); assert(configuration.application.speed_limit_rad_s==PARAM_APP_DEFAULT_SPEED_LIMIT_RPS*_2PI); assert(configuration.application.speed_ceiling_rad_s==cap);
        assert(configuration.device.node_id==node && configuration.application.axis_profile_valid);
        memset(parameter_flash,255,sizeof(parameter_flash)); memcpy(parameter_flash,&p,sizeof(p));
        uint8_t loader_node=0; assert(LoaderParameters_ReadNode(&loader_node)==YI_PARAMS_RAW11 && loader_node==node);
        assert(configuration.device.encoder.electrical_zero_q15==3155);
        assert(configuration.device.encoder.linearization_lut_q15[71]==-42);
        assert(configuration.application.speed_kp==.02f);
        p.speed_limit_rad_s=1.25f; p.record_crc32=Param_RecordCrc(&p); Param_Download(&p,sizeof(p),&configuration); assert(configuration.application.speed_limit_rad_s==1.25f);
        p.speed_limit_rad_s=NAN; p.record_crc32=Param_RecordCrc(&p); Param_Download(&p,sizeof(p),&configuration); assert(configuration.application.speed_limit_rad_s==PARAM_APP_DEFAULT_SPEED_LIMIT_RPS*_2PI); assert(configuration.application.speed_ceiling_rad_s==cap);
        p.speed_limit_rad_s=0; p.record_crc32=Param_RecordCrc(&p); Param_Download(&p,sizeof(p),&configuration); assert(configuration.application.speed_limit_rad_s==PARAM_APP_DEFAULT_SPEED_LIMIT_RPS*_2PI); assert(configuration.application.speed_ceiling_rad_s==cap);
    }
    puts("PASS 254 explicit APP/Loader addresses, independent application ceiling, low fallback and calibration");
    /* 版本化行为变化：旧node3把字段10钳到pi，新版保留显式字段10，不生成目标。 */
    p=saved; p.node_id=3; p.speed_limit_rad_s=10;
    p.record_crc32=Param_RecordCrc(&p); InterfaceParam address_before=p;
    Param_Download(&p,sizeof(p),&configuration);
    assert(configuration.device.node_id==3 && configuration.application.speed_limit_rad_s==10);
    assert(configuration.application.speed_ceiling_rad_s==20 && memcmp(&p,&address_before,sizeof(p))==0);
    /* 轴配置仅收窄应用；同一地址不随AXS1/AXS2角色改变，输入始终不变。 */
    for(unsigned format=0;format<2;format++) {
        p=saved; p.node_id=7; p.speed_limit_rad_s=20;
        if(format==0) { assert(MotorAxisProfile_Create(&p.axis_profile,"roll",-1,1,.7f)); }
        else { assert(MotorAxisProfile_CreateJoint(&p.axis_profile,MOTOR_JOINT_ROLL,1,-.3f,.9f,.7f)); }
        p.record_crc32=Param_RecordCrc(&p); InterfaceParam before=p;
        Param_Download(&p,sizeof(p),&configuration);
        assert(configuration.device.node_id==7 && configuration.application.speed_limit_rad_s==.7f);
        assert(configuration.application.speed_ceiling_rad_s==.7f && memcmp(&p,&before,sizeof(p))==0);
        memset(parameter_flash,255,sizeof(parameter_flash)); memcpy(parameter_flash,&p,sizeof(p));
        uint8_t loader_node=0; assert(LoaderParameters_ReadNode(&loader_node)==YI_PARAMS_RAW11 && loader_node==7);
    }
    {
        const float invalid_nodes[]={3.5f,NAN,INFINITY,-1.0f,255.0f};
        for(unsigned i=0;i<sizeof(invalid_nodes)/sizeof(invalid_nodes[0]);i++) {
            p=saved; p.node_id=invalid_nodes[i]; p.record_crc32=Param_RecordCrc(&p);
            Param_Download(&p,sizeof(p),&configuration);
            assert(!configuration.configuration_confirmed && configuration.configuration_source==MOTOR_CONFIGURATION_CORRUPT);
            memset(parameter_flash,255,sizeof(parameter_flash)); memcpy(parameter_flash,&p,sizeof(p));
            uint8_t loader_node=7; assert(LoaderParameters_ReadNode(&loader_node)==YI_PARAMS_UNSAFE && loader_node==7);
        }
    }
    puts("PASS AXS only narrows application and never overrides node; fractional addresses rejected by APP/Loader");
    /* 只接受当前版本：旧版、未知版和无效识别字均回退，不修改输入记录。 */
    {
        const uint32_t rejected[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,15,UINT32_MAX};
        for(unsigned i=0;i<sizeof(rejected)/sizeof(rejected[0]);++i) {
            p=saved; p.schema_version=rejected[i];
            p.record_crc32=Param_RecordCrc(&p); InterfaceParam before=p;
            p.record_crc32=Param_RecordCrc(&p); Param_Download(&p,sizeof(p),&configuration);
            assert(memcmp(&p,&before,sizeof(p))==0);
            assert(configuration.device.node_id==(uint8_t)PARAM_DEVICE_CAN_NODE_ID);
            assert(configuration.device.encoder.calib_flag==0 && configuration.device.encoder.electrical_zero_q15==0);
            assert(configuration.device.encoder.linearization_lut_q15[71]==0);
            assert(configuration.application.cascade_position_kp==PARAM_APP_CASCADE_POSITION_KP);
            assert(configuration.application.speed_kp==PARAM_APP_SPEED_KP);
            assert(configuration.configuration_source==MOTOR_CONFIGURATION_UNSUPPORTED);
            assert(!configuration.configuration_confirmed);
        }
        p=saved; p.magic_word=0;
        p.record_crc32=Param_RecordCrc(&p); Param_Download(&p,sizeof(p),&configuration);
        assert(configuration.device.encoder.calib_flag==0);
    }
    puts("PASS old/unknown schema rejection, defaults and immutable input");
    /* 旧长度及旧CRC均正确也不得装载，输入保持不变。 */
    for (unsigned version=12U;version<=13U;version++) {
        union { uint32_t alignment; uint8_t bytes[4352]; } legacy;
        uint8_t before[4352];
        const unsigned bytes=version==12U ? 4304U : 4352U;
        memset(&legacy,0,sizeof(legacy)); p=saved; p.schema_version=version;
        memcpy(legacy.bytes,&p,2240U);
        if(version==13U) memcpy(legacy.bytes+4304U,(const uint8_t *)&p+2240U,48U);
        uint32_t crc=record_crc(legacy.bytes,bytes); memcpy(legacy.bytes+2140U,&crc,4U);
        memcpy(before,legacy.bytes,sizeof(before));
        Param_Download(legacy.bytes,bytes,&configuration);
        assert(configuration.configuration_source==MOTOR_CONFIGURATION_UNSUPPORTED);
        assert(!configuration.configuration_confirmed && configuration.device.encoder.calib_flag==0U);
        assert(memcmp(before,legacy.bytes,sizeof(before))==0);
    }
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
        p.record_crc32=Param_RecordCrc(&p); Param_Download(&p,sizeof(p),&configuration);
        assert(memcmp(&p,&before,sizeof(p))==0);
    }
    assert(configuration.application.cascade_position_kp==.07f && configuration.application.cascade_position_kd==.6f);
    assert(configuration.application.speed_kp==.023f && configuration.application.speed_limit_rad_s==20);
    assert(configuration.device.encoder.electrical_zero_q15==3155 && configuration.device.encoder.reverse==0);
    assert(configuration.device.encoder.calib_flag==3 && configuration.device.encoder.linearization_lut_q15[71]==-42);
    Param_Upload(&p,&configuration); p.magic_word=MAGIC_WORD;
    assert(p.configuration_flags==1U && p.record_crc32==Param_RecordCrc(&p) && p.reserved_pos_ki==0 && p.reserved_pos_integral_limit==0);
    {
        const uint8_t zero_bytes[] = {0,0,0,0};
        assert(memcmp((const uint8_t *)&p+2156,zero_bytes,sizeof(zero_bytes))==0);
    }
    /* 退役槽不应用到控制；导出清零，当前记录的CRC完整性保持。 */
    p=saved;
    p.friction_model_valid=1U; p.friction_coulomb_pos_a=.2f;
    p.record_crc32=Param_RecordCrc(&p);
    Param_Download(&p,sizeof(p),&configuration);
    assert(configuration.device.encoder.calib_flag==3 && configuration.device.encoder.linearization_lut_q15[71]==-42);
    Param_Upload(&p,&configuration);
    assert(p.friction_model_valid==0);
    puts("PASS schema14 golden layout and retired fields ignored/zero export");
    /* 当前硬件配置变化仍使电流域参数失效，不属于已删除的版本迁移。 */
    p=saved;
    p.current_sense_shunt_milliohm=CURRENT_SENSE_SHUNT_MILLIOHM==2U ? 6U : 2U;
    p.calibration_current_a=.1f; p.current_limit_a=.1f;
    p.friction_model_valid=1U;
    p.friction_coulomb_pos_a=.2f; p.friction_coulomb_neg_a=.2f;
    p.record_crc32=Param_RecordCrc(&p); Param_Download(&p,sizeof(p),&configuration);
    assert(configuration.model.calibration_current_a==PARAM_MOTOR_CALIB_CURRENT_A);
    assert(configuration.application.current_limit_a==PARAM_APP_DEFAULT_CURRENT_LIMIT_A);
    assert(configuration.device.encoder.calib_flag==3 && configuration.device.encoder.linearization_lut_q15[71]==-42);
    puts("PASS current-sense mismatch invalidates current-domain settings only");

    /* 独立 zlib 黄金值固定整条 CRC 契约；记录 CRC 槽不参与计算。 */
    memset(&p,0,sizeof(p)); p.schema_version=PARAM_SCHEMA_VERSION; p.magic_word=MAGIC_WORD;
    assert(Param_RecordCrc(&p)==PARAM_RECORD14_GOLDEN_CRC);
    p.record_crc32=UINT32_MAX; assert(Param_RecordCrc(&p)==PARAM_RECORD14_GOLDEN_CRC);
    Param_Return_Default(&configuration);
    assert(!configuration.configuration_confirmed && configuration.device.encoder.calib_flag==0U);
    assert(configuration.device.node_id==1U && configuration.application.speed_limit_rad_s==PARAM_APP_DEFAULT_SPEED_LIMIT_RPS*_2PI);
    /* 运行DEFAULTS保留地址由完整owner夹具验证；本处验证该node7默认配置的真实存储边界。 */
    configuration.device.node_id=7U;
    Param_Upload(&p,&configuration);
    assert(p.node_id==7.0f && p.configuration_flags==0U && p.record_crc32==Param_RecordCrc(&p));
    memset(parameter_flash,255,sizeof(parameter_flash)); memcpy(parameter_flash,&p,sizeof(p));
    uint8_t default_loader_node=0;
    assert(LoaderParameters_ReadNode(&default_loader_node)==YI_PARAMS_RAW11 && default_loader_node==7U);
    Param_Return_Default(&configuration);
    assert(configuration.device.node_id==1U);
    Param_Download(&p,sizeof(p),&configuration);
    assert(configuration.device.node_id==7U && !configuration.configuration_confirmed && configuration.device.encoder.calib_flag==0U);
    assert(configuration.application.speed_limit_rad_s==PARAM_APP_DEFAULT_SPEED_LIMIT_RPS*_2PI);
    puts("PASS node7 default configuration export/reload/Loader consistency; blank startup still node1");
    Param_Return_Default(&configuration);
    Param_Upload(&p,&configuration);
    Param_Download(&p,sizeof(p),&configuration);
    assert(!configuration.configuration_confirmed && configuration.configuration_source==MOTOR_CONFIGURATION_STORED);
    configuration.configuration_confirmed=true; configuration.device.encoder.calib_flag=ENC_CALIB_LINEARIZED;
    Param_Upload(&p,&configuration); saved=p;
    Param_Return_Default(&configuration);
    Param_Download(&p,sizeof(p),&configuration);
    assert(configuration.configuration_confirmed && configuration.device.encoder.calib_flag==ENC_CALIB_LINEARIZED);
    p.current_limit_a+=1.0f;
    Param_Download(&p,sizeof(p),&configuration);
    assert(!configuration.configuration_confirmed && configuration.configuration_source==MOTOR_CONFIGURATION_CORRUPT);
    assert(configuration.device.encoder.calib_flag==0U && configuration.application.axis_profile_valid);
    p=saved; p.configuration_flags=2U; p.record_crc32=Param_RecordCrc(&p);
    Param_Download(&p,sizeof(p),&configuration);
    assert(!configuration.configuration_confirmed && configuration.configuration_source==MOTOR_CONFIGURATION_CORRUPT);
    p=saved; p.motor_flux_wb=NAN; p.record_crc32=Param_RecordCrc(&p);
    Param_Download(&p,sizeof(p),&configuration);
    assert(!configuration.configuration_confirmed && configuration.configuration_source==MOTOR_CONFIGURATION_CORRUPT);
    p=saved; p.node_id=0.0f; p.record_crc32=Param_RecordCrc(&p);
    Param_Download(&p,sizeof(p),&configuration);
    assert(!configuration.configuration_confirmed && configuration.device.node_id==1U && configuration.configuration_source==MOTOR_CONFIGURATION_CORRUPT);
    puts("PASS independent schema14 CRC golden, unconfirmed/confirmed reload, partial calibration and corruption fallback");
    /* 当前记录被截断时拒绝，不触碰记录外的内容。 */
    p=saved;
    Param_Download(&p,sizeof(p)-1U,&configuration);
    assert(configuration.configuration_source==MOTOR_CONFIGURATION_CORRUPT);
    assert(!configuration.configuration_confirmed);
    Param_Download(&p,sizeof(p),&configuration);
    configuration.model.thermal.peak_a=6.0f;
    configuration.application.speed_feedforward.enabled=true;
    configuration.application.speed_feedforward.gain_a_per_rad_s2=.04f;
    Param_Upload(&p,&configuration);
    assert(p.schema_version==14U && p.thermal.peak_a==6.0f && p.speed_feedforward_enabled==1U);
    Param_Return_Default(&configuration);
    Param_Download(&p,sizeof(p),&configuration);
    assert(configuration.model.thermal.peak_a==6.0f);
    assert(configuration.application.speed_feedforward.enabled);
    assert(configuration.application.speed_feedforward.gain_a_per_rad_s2==.04f);
    p.thermal.peak_a=NAN; p.record_crc32=Param_RecordCrc(&p);
    Param_Download(&p,sizeof(p),&configuration);
    assert(configuration.configuration_source==MOTOR_CONFIGURATION_CORRUPT && !configuration.configuration_confirmed);
    puts("PASS schema14 bounded read and thermal/feedforward roundtrip; invalid extension rejected");
    return 0;
}
"""
    )


def verify_independent_inputs(cc: list[str], out: Path) -> None:
    """板卡和电机头可独立包含，两个正式目标各自选择正确默认型号。"""
    board = out / "board_independence.c"
    board.write_text(
        '#include "board_config.h"\n'
        "#if defined(FOC_ACTIVE_MOTOR_PROFILE) || defined(PARAM_MOTOR_POLE_PAIRS)\n"
        '#error "board selected a motor"\n#endif\n',
        encoding="utf-8",
    )
    motor = out / "motor_independence.c"
    motor.write_text(
        '#include "motor_profile_config.h"\n'
        "#if defined(BSP_BOARD) || defined(BSP_SUPPORTED_PWM)\n"
        '#error "motor selected a board"\n#endif\n'
        "#if FOC_ACTIVE_MOTOR_PROFILE != EXPECTED_MOTOR_PROFILE\n"
        '#error "wrong default motor"\n#endif\n',
        encoding="utf-8",
    )
    for target, model in (("STM32_APP", 1), ("AT32_OFF", 2)):
        for source in (board, motor):
            subprocess.run(
                cc
                + [
                    "-std=c11",
                    "-Wall",
                    "-Wextra",
                    "-Werror",
                    f"-DFIRMWARE_TARGET_{target}=1",
                    f"-DEXPECTED_MOTOR_PROFILE={model}",
                    "-c",
                    str(source),
                    "-o",
                    str(out / f"independent_{target}_{source.stem}.o"),
                ],
                check=True,
            )
    print("PASS independent board and motor headers for both APP targets")


def verify_motor_profiles(cc: list[str], out: Path) -> None:
    """编译并运行型号/阻尼环矩阵，同时验证不完整APP和非法配置拒绝。"""
    cases = (
        (1, 0, "HT8115-4", "HT8115-4/no-damping-ring", 21, 1.905, 0.001635, 1, 300, 3, 1, 0),
        (1, 1, "HT8115-4", "HT8115-4/damping-ring", 21, 1.905, 0.001635, 1, 300, 3, 1, 0),
        (2, 0, "YQ-5708", "YQ-5708/8:1", 14, 0.55, 0.0001425, 8, 2500, 3.535533906, 0, 0),
        (3, 0, "YQ-5740", "YQ-5740/40:1", 14, 0.55, 0.0001425, 40, 2500, 3.535533906, 0, 0),
        (4, 0, "YQ-8116", "YQ-8116/HT8115-4-baseline", 21, 1.905, 0.001635, 1, 300, 3, 0, 1),
    )
    source = ROOT / "tests/unit/native/motor_profiles_test.c"
    flags = ["-std=c99", "-O1", "-UNDEBUG", "-Wall", "-Wextra", "-Werror"]
    for model, damping, *expected in cases:
        executable = out / f"motor_profile_{model}_{damping}.exe"
        subprocess.run(
            cc
            + flags
            + [f"-DFOC_ACTIVE_MOTOR_PROFILE={model}", f"-DMOTOR_HAS_DAMPING_RING={damping}"]
            + [str(source), "-lm", "-o", str(executable)],
            check=True,
        )
        subprocess.run([str(executable), *map(str, expected)], check=True)
    rejected = out / "motor_profile_rejected.c"
    for header, definitions, reason in (
        ("motor_profile_config.h", ["-DFOC_ACTIVE_MOTOR_PROFILE=99"], "Unsupported FOC_ACTIVE"),
        ("motor_profile_config.h", ["-DMOTOR_HAS_DAMPING_RING=2"], "Unsupported MOTOR_HAS"),
        (
            "motor_profile_config.h",
            ["-DFOC_ACTIVE_MOTOR_PROFILE=2", "-DMOTOR_HAS_DAMPING_RING=1"],
            "Damping-ring configuration",
        ),
        (
            "application_config.h",
            ["-DFOC_ACTIVE_MOTOR_PROFILE=2", "-DPARAM_MOTOR_APP_TEST_DEFAULTS=1"],
            "test target must supply flux and thermal parameters",
        ),
        (
            (out / "targets/r40_app_common.h").as_posix(),
            [],
            "Temporary motor defaults require a phase-locked or bounded-test target",
        ),
        *(
            (
                "application_config.h",
                [f"-DFOC_ACTIVE_MOTOR_PROFILE={model}"],
                "Motor APP defaults incomplete",
            )
            for model in (2, 3, 4)
        ),
    ):
        rejected.write_text(f'#include "{header}"\n', encoding="utf-8")
        result = subprocess.run(
            cc
            + flags
            + definitions
            + (["-DCONFIG_VALIDATE_FULL_APP=1"] if header == "application_config.h" else [])
            + ["-c", str(rejected), "-o", str(out / "rejected.o")],
            capture_output=True,
            text=True,
        )
        if result.returncode == 0 or reason not in result.stderr:
            raise RuntimeError(f"电机配置应拒绝但未得到预期错误: {definitions}: {result.stderr}")
    print("PASS unknown model, invalid damping and incomplete APP rejection")


def verify_custom_config(cc: list[str], out: Path) -> None:
    """独立板型和型号输入可改变派生值，且不能绕过目标与热数据准入。"""
    resource = (ROOT / "firmware/config/board_config.h").read_text(encoding="utf8")
    motor = (ROOT / "firmware/config/motor_profile_config.h").read_text(encoding="utf8")
    for name, start, end in (
        (
            "stm32g4.h",
            "#elif BSP_BOARD == BSP_BOARD_STM32G4\n",
            '\n#else\n#error "Unknown BSP board"',
        ),
        (
            "at32f456.h",
            "#elif BSP_BOARD == BSP_BOARD_AT32F456\n",
            "\n#elif BSP_BOARD == BSP_BOARD_STM32G4",
        ),
        (
            "ht8115_4.h",
            "FOC_ACTIVE_MOTOR_PROFILE == FOC_MOTOR_PROFILE_YQ_8116\n",
            "\n#elif FOC_ACTIVE_MOTOR_PROFILE == FOC_MOTOR_PROFILE_YQ_5708",
        ),
    ):
        source = motor if name == "ht8115_4.h" else resource
        (out / name).write_text(source.split(start, 1)[1].split(end, 1)[0], encoding="utf8")
    (out / "custom_motor.h").write_text(
        '#include "ht8115_4.h"\n'
        "#undef PARAM_MOTOR_POLE_PAIRS\n#define PARAM_MOTOR_POLE_PAIRS 14\n"
        "#undef PARAM_MOTOR_PHASE_RESISTANCE_OHM\n#define PARAM_MOTOR_PHASE_RESISTANCE_OHM 0.75f\n"
        "#undef PARAM_MOTOR_D_INDUCTANCE_H\n#define PARAM_MOTOR_D_INDUCTANCE_H 0.0002f\n"
        "#undef PARAM_MOTOR_CALIB_SPEED_MEC_RAD_S\n#define PARAM_MOTOR_CALIB_SPEED_MEC_RAD_S 25.0f\n",
        encoding="utf-8",
    )
    (out / "custom_stm.h").write_text(
        '#define CURRENT_SENSE_SHUNT_MILLIOHM 2U\n#include "stm32g4.h"\n'
        "#undef CURRENT_SENSE_PROFILE_AMPLIFIER_GAIN\n"
        "#define CURRENT_SENSE_PROFILE_AMPLIFIER_GAIN 5.0f\n",
        encoding="utf-8",
    )
    (out / "custom_at32.h").write_text(
        '#include "at32f456.h"\n'
        "#undef AT32_PHASE_SHUNT_MILLIOHM\n#define AT32_PHASE_SHUNT_MILLIOHM 5U\n"
        "#undef AT32_ADC_RESOLUTION_BITS\n#define AT32_ADC_RESOLUTION_BITS 10U\n"
        "#undef AT32_CURRENT_GAIN\n#define AT32_CURRENT_GAIN 10.0f\n",
        encoding="utf-8",
    )
    (out / "invalid_motor.h").write_text(
        '#include "custom_motor.h"\n#undef PARAM_MOTOR_POLE_PAIRS\n'
        "#define PARAM_MOTOR_POLE_PAIRS 0\n",
        encoding="utf-8",
    )
    (out / "invalid_at32.h").write_text(
        '#include "at32f456.h"\n#undef AT32_ADC_RESOLUTION_BITS\n'
        "#define AT32_ADC_RESOLUTION_BITS 32U\n",
        encoding="utf-8",
    )
    (out / "invalid_capability.h").write_text(
        '#include "stm32g4.h"\n#undef BSP_SUPPORTED_CAN\n#define BSP_SUPPORTED_CAN 2\n',
        encoding="utf-8",
    )
    (out / "invalid_clock.h").write_text(
        '#include "at32f456.h"\n'
        "#undef AT32_HEXT_HZ\n#define AT32_HEXT_HZ 8100000U\n"
        "#undef AT32_PLL_MULTIPLIER\n#define AT32_PLL_MULTIPLIER 40U\n",
        encoding="utf-8",
    )
    (out / "invalid_fast_clock.h").write_text(
        '#include "at32f456.h"\n'
        "#undef AT32_HEXT_HZ\n#define AT32_HEXT_HZ 10000000U\n"
        "#undef AT32_PLL_MULTIPLIER\n#define AT32_PLL_MULTIPLIER 80U\n",
        encoding="utf-8",
    )
    targets = out / "targets"
    flags = ["-std=c11", "-O1", "-UNDEBUG", "-Wall", "-Wextra", "-Werror", "-I", str(out)]
    source = ROOT / "tests/unit/native/config_custom_profile_test.c"
    for target, board in (("stm32g4_app.h", "custom_stm.h"), ("r40_app_off.h", "custom_at32.h")):
        exe = out / f"custom_{board}.exe"
        subprocess.run(
            cc
            + flags
            + [
                f'-DBSP_BOARD_CONFIG_HEADER="{board}"',
                '-DMOTOR_PROFILE_HEADER="custom_motor.h"',
                "-include",
                str(targets / target),
                str(source),
                "-lm",
                "-o",
                str(exe),
            ],
            check=True,
        )
        subprocess.run([str(exe)], check=True)

    # PLL /4 与 MCU 初始化一致；不同晶振、倍频和分频的时钟由同一组输入计算。
    (out / "custom_clock.h").write_text(
        '#include "at32f456.h"\n'
        "#undef AT32_HEXT_HZ\n#define AT32_HEXT_HZ 16000000U\n"
        "#undef AT32_PLL_MULTIPLIER\n#define AT32_PLL_MULTIPLIER 60U\n"
        "#undef AT32_PLL_PRE_DIVIDER\n#define AT32_PLL_PRE_DIVIDER 3U\n",
        encoding="utf-8",
    )
    clock_test = out / "custom_clock_test.c"
    clock_test.write_text(
        '#include "board_config.h"\n#include "application_config.h"\n'
        "_Static_assert(AT32_CORE_HZ == 80000000U && AT32_TIMER_HZ == 80000000U && "
        'AT32_CAN_HZ == 80000000U, "clock derivation failed");\n'
        "_Static_assert(AT32_CAN_NOMINAL_SEG1 == 60U && AT32_CAN_NOMINAL_SEG2 == 20U && "
        'AT32_CAN_DATA_SEG1 == 12U && AT32_CAN_DATA_SEG2 == 4U, "CAN timing failed");\n'
        '_Static_assert(APP_FAST_MAX_CYCLES == 4000U, "fast-loop deadline failed");\n'
        "int main(void) { return 0; }\n",
        encoding="utf-8",
    )
    subprocess.run(
        cc
        + flags
        + [
            '-DBSP_BOARD_CONFIG_HEADER="custom_clock.h"',
            "-include",
            str(targets / "r40_app_off.h"),
            str(clock_test),
            "-o",
            str(out / "custom_clock_test.exe"),
        ],
        check=True,
    )
    subprocess.run([str(out / "custom_clock_test.exe")], check=True)

    # 厂商时钟配置必须使用相同晶振输入，避免只修改板卡配置后出现两套频率。
    clock_probe = out / "custom_clock_sdk_test.c"
    clock_probe.write_text(
        '#include "at32f45x_conf.h"\n'
        '#if HEXT_VALUE != 16000000U\n#error "SDK clock input disagrees"\n#endif\n',
        encoding="utf-8",
    )
    sdk = ROOT / "firmware/platform/sdk/at32f45x"
    subprocess.run(
        cc
        + flags
        + [
            "-DAT32F456CCU7",
            '-DBSP_BOARD_CONFIG_HEADER="custom_clock.h"',
            "-include",
            str(targets / "r40_app_off.h"),
            "-I",
            str(ROOT / "firmware/platform/bsp/mcu/at32f456"),
            "-I",
            str(sdk / "drivers/inc"),
            "-I",
            str(sdk / "cmsis/device"),
            "-I",
            str(sdk / "cmsis/core"),
            "-E",
            str(clock_probe),
            "-o",
            str(out / "custom_clock_sdk_test.i"),
        ],
        check=True,
    )

    # 10位变体仅验证换算；实际AT32驱动固定12位，必须拒绝不匹配的板卡输入。
    driver = ROOT / "firmware/platform/bsp/mcu/at32f456/adc.c"
    bad_adc = subprocess.run(
        cc
        + flags
        + [
            '-DBSP_BOARD_CONFIG_HEADER="custom_at32.h"',
            "-include",
            str(targets / "r40_app_off.h"),
            "-I",
            str(ROOT / "tests/unit/native/r40_io_fake"),
            "-I",
            str(ROOT / "firmware/platform/sdk/at32f45x/drivers/inc"),
            "-I",
            str(ROOT / "firmware/platform/bsp"),
            "-E",
            str(driver),
        ],
        capture_output=True,
        text=True,
    )
    if (
        bad_adc.returncode == 0
        or "AT32F456 ADC initialization requires 12-bit sampling" not in bad_adc.stderr
    ):
        raise RuntimeError(f"ADC位宽不一致时应拒绝构建: {bad_adc.stderr}")

    rejected = out / "custom_rejected.c"
    rejected.write_text(
        '#include "application_config.h"\n#include "board_config.h"\n', encoding="utf-8"
    )
    for flags_extra, reason in (
        (
            ["-include", str(targets / "stm32g4_app.h"), '-DBSP_BOARD_CONFIG_HEADER="at32f456.h"'],
            "Board configuration and MCU BSP family disagree",
        ),
        (
            [
                "-include",
                str(targets / "stm32g4_app.h"),
                '-DMOTOR_PROFILE_HEADER="custom_motor.h"',
                "-DFOC_ACTIVE_MOTOR_PROFILE=1",
            ],
            "Choose MOTOR_PROFILE_HEADER or FOC_ACTIVE_MOTOR_PROFILE",
        ),
        (
            ["-include", str(targets / "r40_app_off.h"), "-DFOC_ACTIVE_MOTOR_PROFILE=4"],
            "Motor APP defaults incomplete",
        ),
        (
            [
                "-include",
                str(targets / "stm32g4_app.h"),
                '-DMOTOR_PROFILE_HEADER="invalid_motor.h"',
            ],
            "Motor profile must specify positive pole pairs",
        ),
        (
            [
                "-include",
                str(targets / "r40_app_off.h"),
                '-DBSP_BOARD_CONFIG_HEADER="invalid_at32.h"',
            ],
            "Invalid current-sense ADC resolution",
        ),
        (
            [
                "-include",
                str(targets / "stm32g4_app.h"),
                '-DBSP_BOARD_CONFIG_HEADER="invalid_capability.h"',
            ],
            "Invalid or unsupported BSP capability",
        ),
        (
            [
                "-include",
                str(targets / "r40_app_off.h"),
                '-DBSP_BOARD_CONFIG_HEADER="invalid_clock.h"',
            ],
            "AT32 CAN clock cannot represent",
        ),
        (
            [
                "-include",
                str(targets / "r40_app_off.h"),
                '-DBSP_BOARD_CONFIG_HEADER="invalid_fast_clock.h"',
            ],
            "AT32 core clock exceeds validated ADC/SPI timing",
        ),
    ):
        result = subprocess.run(
            cc + flags + flags_extra + ["-c", str(rejected), "-o", str(out / "custom_rejected.o")],
            capture_output=True,
            text=True,
        )
        if result.returncode == 0 or reason not in result.stderr:
            raise RuntimeError(f"配置应拒绝但未得到预期错误：{flags_extra}: {result.stderr}")
    print("PASS custom configuration conflicts and incomplete motor rejection")


def verify_application_defaults(cc: list[str], out: Path) -> None:
    """用真实默认值和校验函数检查新旧板；不以手写夹具覆盖默认电流。"""
    source = "\n".join(
        function_source((ROOT / path).read_text(encoding="utf-8"), name)
        for path, name in (
            ("firmware/parameters/parameter_record.c", "Param_Return_Default"),
            ("firmware/parameters/parameter_values.c", "ParameterValue_ConfigurationValid"),
        )
    )
    (out / "app_defaults_subject.inc").write_text(source, encoding="utf-8")
    # 真实目标配置同时覆盖旧板两种采样电阻，以及新板关相/限时目标。
    target_dir = out / "targets"
    cases = (
        ("old_6m", [], "6.0f", "0.05f", "0.5f", "90.0f", "18.0f", 21, "1.905f", 0),
        (
            "old_2m",
            ["-DCURRENT_SENSE_SHUNT_MILLIOHM=2"],
            "6.0f",
            "0.05f",
            "0.5f",
            "90.0f",
            "18.0f",
            21,
            "1.905f",
            0,
        ),
        (
            "r40_off",
            ["-include", str(target_dir / "r40_app_off.h")],
            "3.0f",
            "0.05f",
            "0.5f",
            "90.0f",
            "3.0f",
            14,
            "0.55f",
            1,
        ),
        (
            "r40_commissioning",
            ["-include", str(target_dir / "r40_app_commissioning.h")],
            "3.0f",
            "0.005f",
            "0.05f",
            "800.0f",
            "3.0f",
            14,
            "0.55f",
            1,
        ),
    )
    for name, selection, current, kp, ki, pll, trip, poles, resistance, bus_sensor in cases:
        executable = out / f"app_defaults_{name}.exe"
        flags = selection + [
            f"-DEXPECTED_DEFAULT_A={current}",
            f"-DEXPECTED_SPEED_KP={kp}",
            f"-DEXPECTED_SPEED_KI={ki}",
            f"-DEXPECTED_PLL_HZ={pll}",
            f"-DEXPECTED_TRIP_A={trip}",
            f"-DEXPECTED_POLE_PAIRS={poles}",
            f"-DEXPECTED_PHASE_R_OHM={resistance}",
            f"-DEXPECTED_BUS_SENSOR={bus_sensor}",
        ]
        subprocess.run(
            cc
            + ["-std=c11", "-O2", "-UNDEBUG", "-Wall", "-Wextra", "-Werror", "-I", str(out)]
            + flags
            + [
                str(ROOT / "tests/unit/native/app_defaults_test.c"),
                str(ROOT / "firmware/motor/protection/motor_thermal_config.c"),
                str(ROOT / "firmware/motor/protection/motor_thermal.c"),
                str(ROOT / "firmware/motor/control/speed_feedforward.c"),
                str(ROOT / "firmware/motor/math/control_math.c"),
                "-lm",
                "-o",
                str(executable),
            ],
            check=True,
        )
        subprocess.run([str(executable)], check=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cc", required=True)
    ap.add_argument("--out", type=Path, default=ROOT / "outputs/wheel_speed_20260917/host")
    args = ap.parse_args()
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    target_dir = out / "targets"
    target_dir.mkdir(exist_ok=True)
    for name, target in (
        ("stm32g4_app.h", "STM32_APP"),
        ("r40_app_off.h", "AT32_OFF"),
        ("r40_app_commissioning.h", "AT32_COMMISSIONING"),
        ("r40_app_common.h", "AT32_COMMON"),
    ):
        (target_dir / name).write_text(
            f'#define FIRMWARE_TARGET_{target} 1\n#include "board_config.h"\n', encoding="utf8"
        )
    (out / "main.h").write_text("#include <stdint.h>\n")
    source = out / "wheel_speed_test.c"
    source.write_text(fixture(), encoding="utf-8")
    cc = [str(Path(args.cc).resolve())] if Path(args.cc).is_file() else [args.cc]
    if Path(args.cc).stem == "zig":
        cc += ["cc"]
    cc += NATIVE_INCLUDE_FLAGS
    verify_independent_inputs(cc, out)
    verify_motor_profiles(cc, out)
    verify_custom_config(cc, out)
    verify_application_defaults(cc, out)
    exe = out / "wheel_speed_test.exe"
    includes = [str(out)] + [
        str(ROOT / p)
        for p in (
            "firmware/motor/math",
            "firmware/motor/control",
            "firmware/config",
            "firmware/platform/bsp/mcu/stm32g4",
            "firmware/communication",
        )
    ]
    command = cc + ["-std=c99", "-O1", "-UNDEBUG", "-Wall", "-Wextra", "-Werror"]
    command += [item for p in includes for item in ("-I", p)]
    command += [
        str(source),
        str(ROOT / "firmware/parameters/motor_axis_profile.c"),
        str(ROOT / "shared/checksum/crc32.c"),
        str(ROOT / "firmware/parameters/parameter_values.c"),
        str(ROOT / "firmware/motor/protection/motor_thermal.c"),
        str(ROOT / "firmware/motor/protection/motor_thermal_config.c"),
        str(ROOT / "firmware/motor/control/speed_feedforward.c"),
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
