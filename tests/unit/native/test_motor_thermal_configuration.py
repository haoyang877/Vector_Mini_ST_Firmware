"""离线验证真实热参数CAN分派、热历史保留及MTH1存储，不接触硬件。"""

import argparse
import json
import subprocess
import sys
from pathlib import Path

import test_motor_load_profile as storage
from project_paths import NATIVE_INCLUDE_FLAGS, ROOT
from run_position_servo_tests import function_source


def can_fixture(vectors: dict) -> str:
    """抽取生产收发及热分派，硬件边界仅用内存帧替身。"""
    source = (ROOT / "firmware/communication/can/interface_can.c").read_text(encoding="utf-8")
    # 抽取真实热分支；不以恒真验证器代替业务接受结果。
    dispatch = function_source(source, "CAN_ReceiveMessage_Update")
    read = function_source(source, "CAN_ReadParameter")
    read_cases = "\n".join(
        line
        for line in read.splitlines()
        if "case CAN_GET_THERMAL_REVISION:" in line
        or "case CAN_GET_TEMPERATURE_PROTECTION:" in line
    )
    thermal_read = read[
        read.index("    case CAN_GET_THERMAL_CONTINUOUS:") : read.index(
            "    default: return false;"
        )
    ]
    thermal_write = dispatch[
        dispatch.index("    case CAN_SET_TEMPERATURE_PROTECTION:") : dispatch.index(
            "    case CAN_SET_SPEED_FF_ENABLE:"
        )
    ]
    dispatch = (
        function_source(source, "CAN_RequestKind")
        + "\n"
        + (
            "static bool CAN_ReadParameter(CAN_PARAM_ID param_id) { float value; switch(param_id) {\n"
            + read_cases
            + "\n"
            + thermal_read
            + "default:return false;} CAN_SendMessage_Update(param_id,value);return true;}\n"
            + "bool CAN_ReceiveMessage_Update(CAN_PARAM_ID param_id,float data) {\n"
            + "if(CAN_RequestKind(param_id)==1U) {return CAN_ReadParameter(param_id);}\n"
            + "switch(param_id) {\n"
            + thermal_write
            + "default:return false;}}\n"
        )
    )
    functions = [
        "CAN_CommandEncoding",
        "CAN_ReplyEncoding",
        "CAN_Milli32",
        "CAN_Centi32",
        "CAN_Milli16",
        "CAN_SendMessage_Update",
        "CANRxIRQHandler",
    ]
    fixture = r"""
#include <assert.h>
#include <float.h>
#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "interface_can.h"
#include "data_type.h"
#include "motor_thermal_control.h"
#include "motor_hw.h"
#include "comm_hw.h"
#include "can_motor_status.h"
#include "firmware_version.h"
#include "utils.h"
CANMsg_TypeDef CANMsg;
volatile CAN_Diagnostics_TypeDef CAN_Diagnostics;
MotorControl_TypeDef MotorControl;
ModeNow_TypeDef ModeLast;
const uint32_t g_firmware_software_version = FIRMWARE_VERSION_CODE;
typedef enum {CAN_VALUE_FLOAT32,CAN_VALUE_MILLI_I32,CAN_VALUE_CENTI_I32,CAN_VALUE_MILLI_I16} CanValueEncoding;
static CommHwCanFrame incoming;
static bool gate_off = true;
static uint32_t clock_ms;
/* Loader 请求不属于此热协议夹具；保持未请求态，仍运行真实 RX 路由。 */
static bool BootHandoff_Pending(void) { return false; }
static bool BootHandoff_Request(uint8_t node,uint32_t now) { (void)node;(void)now;return false; }
static uint32_t time_hw_now_ms(void) { return clock_ms; }
static void MotorControl_LockMaintenance(void) { }
bool comm_hw_can_receive(CommHwCanFrame *frame) { *frame=incoming; return true; }
bool motor_hw_phase_outputs_disabled(void) { return gate_off; }
void motor_hw_outer_barrier(void) { }
static void Set_ErrorNow(ErrorNow_TypeDef error) { MotorControl.ErrorNow=error; }
"""
    fixture += source.split("/* CAN_REPLY_RUNTIME_BEGIN */")[1].split(
        "/* CAN_REPLY_RUNTIME_END */"
    )[0]
    fixture += "\n".join(function_source(source, name) for name in functions[:5])
    fixture += "\n" + function_source(source, "CAN_SendMessage_Update") + "\n" + dispatch
    fixture += "\n" + function_source(source, "CANRxIRQHandler")
    fixture += r"""
static MotorThermalDiagnostics snapshot(void) {
    MotorThermalDiagnostics result;
    assert(MotorThermalControl_GetDiagnostics(&result)); return result;
}
static void frame(unsigned id, float value) {
    uint32_t bits=FloatToIntBit(value); memset(&incoming,0,sizeof(incoming));
    incoming.identifier=id; incoming.length=4;
    for(unsigned i=0;i<4;i++) incoming.data[i]=(uint8_t)(bits>>(24-i*8));
    CANMsg.can_tx_en=false; can_reply.state=CAN_REPLY_FREE; CANRxIRQHandler();
}
static void write(unsigned id,float value) {
    frame(0x400|id,value); assert(CANMsg.can_tx_en && CANMsg.tx_param_id==(id+1));
}
static void consume(void) {
    MotorThermalControl_Sample(0,0,0,true,clock_ms);
}
static void zero_step(void) {
    ++clock_ms;
    for(unsigned i=0;i<20;i++) MotorThermalControl_Sample(0,0,0,true,clock_ms);
}
int main(void) {
    MotorThermalDiagnostics before,after;
    float value;
    CANMsg.node_id=4; MotorControl.ModeNow=Motor_Disable; ModeLast=Motor_Disable;
    assert(!MotorThermalControl_RequestParameter(MOTOR_THERMAL_PEAK,6,0,true));
    const float bad_boot[]={3,7,65,80,.7f,1,.5f};
    assert(!MotorThermalControl_LoadParameters(bad_boot));
    const float small_boot[]={.01f,7,65,.8f,.7f,1,.5f};
    assert(MotorThermalControl_LoadParameters(small_boot));
    assert(MotorThermalControl_LoadParameters(NULL));
    MotorThermalControl_Init(0);
    /* 原热参数场景显式开启保护，避免默认关闭掩盖降额和故障断言。 */
    assert(!MotorThermalControl_ProtectionEnabled());
    assert(MotorThermalControl_RequestProtection(true,Motor_Disable,true));
    for(unsigned i=0;i<20;i++) MotorThermalControl_Sample(0,0,0,true,0);
    assert(MotorThermalControl_ProtectionEnabled() && snapshot().model.sample_count==0);
    const unsigned ids[]={0x81,0x83,0x85,0x87,0x89,0x8b,0x8d};
    const unsigned char golden[][4]={GOLDEN_CAN_BYTES};
    for(unsigned i=0;i<7;i++) {
        frame(0x400|ids[i],NAN);
        assert(CANMsg.can_tx_en && CANMsg.tx_data_len==4);
        assert(memcmp(CANMsg.tx_data_u8,golden[i],4)==0);
    }
    frame(0x48e,0); assert(CANMsg.tx_data==THERMAL_PROTOCOL_REVISION);
    for(unsigned n=0;n<1000;n++) {
        ++clock_ms;
        for(unsigned k=0;k<20;k++) MotorThermalControl_Sample(3,-1.5f,-1.5f,true,clock_ms);
    }
    before=snapshot(); assert(before.model.max_heat>0);
    MotorControl.ModeNow=Current_Mode; write(0x82,6); assert(CANMsg.tx_data==-1);
    MotorControl.ModeNow=Motor_Disable; gate_off=false; write(0x82,6); assert(CANMsg.tx_data==-1);
    gate_off=true;
    const float invalid[]={NAN,INFINITY,FLT_MAX,-1,0};
    for(unsigned i=0;i<5;i++) { write(0x80,invalid[i]); assert(CANMsg.tx_data==-1); }
    write(0x86,65); assert(CANMsg.tx_data==-1);
    write(0x8a,101); assert(CANMsg.tx_data==-1);
    after=snapshot(); assert(memcmp(&before.model,&after.model,sizeof(before.model))==0);
    write(0x84,200); assert(CANMsg.tx_data==200);
    assert(!MotorThermalControl_CanRunMode(Position_Mode));
    assert(MotorThermalControl_CurrentLimit(7)==0);
    assert(MotorThermalControl_CanRunMode(Save_Param));
    write(0x82,6); assert(CANMsg.tx_data==-1);
    MotorThermalControl_Sample(0,0,0,false,clock_ms);
    assert(snapshot().model.config.heating_tau_s==before.model.config.heating_tau_s);
    consume(); after=snapshot();
    assert(after.model.config.heating_tau_s==200 && after.model.config.cooling_tau_s==200);
    assert(after.model.max_heat>0 && after.model.valid);
    before=after;
    write(0x80,1.5f); assert(CANMsg.tx_data==1.5f); consume(); after=snapshot();
    assert(fabsf(after.model.config.continuous_phase_rms_a-1.5f/sqrtf(2))<1e-6f);
    for(unsigned i=0;i<4;i++) assert(fabsf(after.model.heat[i]-before.model.heat[i]*4)<1e-5f);
    before=after; write(0x80,3); consume(); after=snapshot();
    for(unsigned i=0;i<4;i++) assert(fabsf(after.model.heat[i]-before.model.heat[i])<1e-5f);
    write(0x8c,5); consume(); write(0x88,7); consume(); write(0x86,8); consume();
    write(0x8a,10); consume();
    after=snapshot(); assert(after.model.fault_latched && after.model.available_a==0);
    assert(!MotorThermalControl_CanClearFault());
    write(0x8a,100); consume(); assert(snapshot().model.fault_latched);
    assert(!MotorThermalControl_LoadParameters(NULL));
    for(unsigned i=0;i<400000 && !MotorThermalControl_CanClearFault();i++) zero_step();
    assert(MotorThermalControl_CanClearFault() && MotorThermalControl_ClearFault());
    assert(!snapshot().model.fault_latched && snapshot().model.max_heat>0);
    assert(MotorControl.ModeNow==Motor_Disable);
    frame(0x481,0); incoming.length=2; CANMsg.can_tx_en=false; CANRxIRQHandler();
    assert(!CANMsg.can_tx_en);
    frame(0x381,0); assert(!CANMsg.can_tx_en);
    assert(!MotorThermalControl_GetParameter(MOTOR_THERMAL_PARAMETER_COUNT,&value));
    puts("PASS actual thermal CAN: golden frames, stopped-only writes, atomic rejection, derived values, retained heat/latch and configurable clear");
    return 0;
}
"""
    rows = [
        "{" + ",".join(f"0x{byte:02x}" for byte in bytes.fromhex(v["default_hex"])) + "}"
        for v in vectors["parameters"]
    ]
    return fixture.replace("GOLDEN_CAN_BYTES", ",".join(rows)).replace(
        "THERMAL_PROTOCOL_REVISION", str(vectors["revision"])
    )


def storage_fixture(vectors: dict, original: str) -> str:
    """在既有生产参数夹具中检验尾记录兼容性与损坏拒绝。"""
    checks = r"""
static void thermal_storage_checks(void) {
    InterfaceParam_TypeDef p,saved;
    assert(sizeof(MotorThermalRecord)==40 && sizeof(p)==THERMAL_PARAMETER_BYTES);
    assert(offsetof(InterfaceParam_TypeDef,motor_thermal)==THERMAL_RECORD_OFFSET);
    seed(&p);
    const unsigned char golden[]={GOLDEN_RECORD_BYTES};
    assert(memcmp(&p.motor_thermal,golden,sizeof(golden))==0);
    saved=p; saved.motor_thermal.values[MOTOR_THERMAL_CONTINUOUS_VECTOR]=3.5f;
    saved.motor_thermal.crc32=motor_thermal_record_crc(&saved.motor_thermal);
    assert(!Param_Download(&saved));
    float value; assert(MotorThermalControl_GetParameter(MOTOR_THERMAL_CONTINUOUS_VECTOR,&value));
    assert(value==3.5f && OnBoard_Encoder.electrical_zero_q15==12480);
    InterfaceParam_TypeDef roundtrip; Param_Upload(&roundtrip);
    assert(memcmp(&roundtrip.motor_thermal,&saved.motor_thermal,sizeof(MotorThermalRecord))==0);
    assert(memcmp(&saved,&p,THERMAL_RECORD_OFFSET)==0);
    for(unsigned bit=0;bit<320;bit++) {
        saved=p; ((unsigned char*)&saved.motor_thermal)[bit/8]^=(unsigned char)(1U<<(bit%8));
        assert(!Param_Download(&saved) && !MotorControl.axis_profile_valid);
    }
    for(unsigned erased=0;erased<2;erased++) {
        saved=p; memset(&saved.motor_thermal,erased?255:0,sizeof(saved.motor_thermal));
        assert(!Param_Download(&saved) && MotorControl.axis_profile_valid);
        assert(MotorThermalControl_GetParameter(MOTOR_THERMAL_CONTINUOUS_VECTOR,&value) && value==3);
    }
    saved=p; saved.motor_thermal.version=2;
    saved.motor_thermal.crc32=motor_thermal_record_crc(&saved.motor_thermal);
    assert(!Param_Download(&saved) && !MotorControl.axis_profile_valid);
    saved=p; saved.motor_thermal.values[MOTOR_THERMAL_HEATING_TAU]=NAN;
    saved.motor_thermal.crc32=motor_thermal_record_crc(&saved.motor_thermal);
    assert(!Param_Download(&saved) && !MotorControl.axis_profile_valid);
    puts("PASS MTH1: golden CRC, saved configuration roundtrip, unchanged 2344-byte prefix, erased legacy and corruption rejection");
}
"""
    checks = checks.replace(
        "GOLDEN_RECORD_BYTES",
        ",".join(f"0x{byte:02x}" for byte in bytes.fromhex(vectors["flash_record_default_hex"])),
    )
    checks = checks.replace("THERMAL_PARAMETER_BYTES", str(vectors["flash_parameter_bytes"]))
    checks = checks.replace("THERMAL_RECORD_OFFSET", str(vectors["flash_offset"]))
    return original.replace(
        "int main(void) {", checks + "\nint main(void) {\n thermal_storage_checks();"
    )


def main() -> None:
    """构建运行真实C代码和存储夹具，日志仅写入outputs。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", required=True)
    parser.add_argument("--out", type=Path, default=ROOT / "outputs/thermal_configuration")
    args = parser.parse_args()
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    vectors = json.loads(
        (ROOT / "tests/vectors/motor_thermal_configuration_v1.json").read_text(encoding="utf-8")
    )
    (out / "main.h").write_text("#include <stdint.h>\n#include <stddef.h>\n", encoding="utf-8")
    source = out / "thermal_can_test.c"
    source.write_text(can_fixture(vectors), encoding="utf-8")
    exe = out / "thermal_can_test.exe"
    compiler = [args.cc] + (["cc"] if Path(args.cc).stem == "zig" else [])
    command = compiler + [
        "-std=c99",
        "-O2",
        "-UNDEBUG",
        "-Wall",
        "-Wextra",
        "-Werror",
        "-I",
        str(out),
    ]
    command += NATIVE_INCLUDE_FLAGS + [
        str(source),
        str(ROOT / "firmware/app/motor_thermal_control.c"),
        str(ROOT / "firmware/motor/protection/motor_thermal.c"),
        str(ROOT / "firmware/common/utils.c"),
        "-lm",
        "-o",
        str(exe),
    ]
    logs = []
    for invocation in (command, [str(exe)]):
        result = subprocess.run(invocation, cwd=ROOT, capture_output=True, text=True)
        logs.append(result.stdout + result.stderr)
        (out / "tests.log").write_text("\n".join(logs), encoding="utf-8")
        print(logs[-1], end="", flush=True)
        result.check_returncode()
    original = storage.fixture()
    storage.fixture = lambda: storage_fixture(vectors, original)
    sys.argv = [sys.argv[0], "--cc", args.cc, "--out", str(out / "storage")]
    storage.main()
    (out / "summary.json").write_text(
        json.dumps(
            {
                "status": "passed",
                "checks": ["production_can_and_model", "production_parameter_storage"],
                "hardware_actions": False,
            },
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )


if __name__ == "__main__":
    main()
