"""离线验证温度保护运行开关、真实热模型及 CAN 0x90/0x91，不访问硬件。"""

import argparse
import hashlib
import json
import subprocess
from pathlib import Path

import test_motor_thermal as thermal
import test_motor_thermal_configuration as configuration

ROOT = thermal.ROOT


def control_fixture() -> str:
    """独立进程模拟上电；只经公开接口读写，避免测试自行清零热历史。"""
    return r"""
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "motor_thermal_control.h"
#include "data_type.h"

void motor_hw_outer_barrier(void) { }

static uint32_t now_ms;
static MotorThermalDiagnostics snapshot(void) {
    MotorThermalDiagnostics result;
    assert(MotorThermalControl_GetDiagnostics(&result));
    return result;
}
static void step(float current_a, bool outputs_disabled) {
    ++now_ms;
    for (unsigned i=0; i<20; ++i) {
        MotorThermalControl_Sample(current_a,-current_a/2,-current_a/2,
                                  outputs_disabled,now_ms);
    }
}
static void blocked(void) {
    assert(!MotorThermalControl_CanRunMode(Current_Mode));
    assert(!MotorThermalControl_Guard(Calib_EleAngelOffset));
    assert(MotorThermalControl_CurrentLimit(20)==0);
    assert(!MotorThermalControl_CanClearFault());
    assert(!MotorThermalControl_ClearFault());
    assert(MotorThermalControl_CanRunMode(Motor_Disable));
    assert(MotorThermalControl_CanRunMode(Save_Param));
}
static void set_enabled(bool enabled) {
    MotorThermalDiagnostics before=snapshot();
    assert(MotorThermalControl_RequestProtection(enabled,Motor_Disable,true));
    assert(!MotorThermalControl_CanRunMode(Current_Mode));
    assert(MotorThermalControl_CurrentLimit(20)==0);
    assert(MotorThermalControl_SamplesUntilUpdate()==0);
    assert(!MotorThermalControl_RequestProtection(!enabled,Motor_Disable,true));
    assert(!MotorThermalControl_RequestParameter(MOTOR_THERMAL_PEAK,8,Motor_Disable,true));
    assert(MotorThermalControl_ProtectionEnabled()!=enabled);
    MotorThermalControl_Sample(0,0,0,false,now_ms);
    assert(MotorThermalControl_ProtectionEnabled()!=enabled);
    MotorThermalControl_Sample(0,0,0,true,now_ms);
    assert(MotorThermalControl_ProtectionEnabled()==enabled);
    MotorThermalDiagnostics after=snapshot();
    assert(memcmp(before.model.heat,after.model.heat,sizeof(before.model.heat))==0);
    assert(before.model.fault_latched==after.model.fault_latched);
}
static void lifecycle(void) {
    assert(MotorThermalControl_CurrentLimit(20)==9);
    assert(MotorThermalControl_CurrentLimit(2)==2);
    assert(MotorThermalControl_CurrentLimit(NAN)==0);
    assert(MotorThermalControl_CurrentLimit(-1)==0);
    set_enabled(true);
    assert(!MotorThermalControl_RequestProtection(false,Current_Mode,true));
    assert(!MotorThermalControl_RequestProtection(false,Motor_Disable,false));
    assert(!MotorThermalControl_RequestProtection(false,UINT32_MAX,true));
    set_enabled(false);
    assert(MotorThermalControl_CanRunMode(Current_Mode));
    assert(MotorThermalControl_CurrentLimit(20)==9);
    assert(MotorThermalControl_CurrentLimit(6)==6);
    assert(MotorThermalControl_RequestParameter(MOTOR_THERMAL_PEAK,5,Motor_Disable,true));
    assert(!MotorThermalControl_RequestProtection(true,Motor_Disable,true));
    blocked();
    MotorThermalControl_Sample(0,0,0,true,now_ms);
    assert(MotorThermalControl_CurrentLimit(20)==5);
    MotorThermalControl_Init(123);
    assert(!MotorThermalControl_ProtectionEnabled());
    set_enabled(true);
    assert(MotorThermalControl_CurrentLimit(20)==5);
    assert(!MotorThermalControl_CanRunMode(MODE_NUM));
    assert(!MotorThermalControl_CanRunMode(Calib_Anticogging));
}
static void hot(bool derating_only) {
    set_enabled(true);
    /* 应用开关会保留部分采样窗；先补零到完整窗口，再构造阈值两侧的热状态。 */
    while (snapshot().model.sample_count!=0) {
        MotorThermalControl_Sample(0,0,0,true,now_ms);
    }
    step(.046f,true);
    step(.046f,true);
    MotorThermalDiagnostics before=snapshot();
    assert(before.model.valid && before.model.derating);
    assert(before.model.max_heat>.8f && before.model.max_heat<1);
    assert(!MotorThermalControl_CanRunMode(Calib_EleAngelOffset));
    if (!derating_only) {
        step(7,true);
        before=snapshot();
        assert(before.model.valid && before.model.fault_latched && before.model.max_heat>1);
        assert(!MotorThermalControl_CanRunMode(Current_Mode));
        assert(MotorThermalControl_CurrentLimit(20)==0);
    }
    set_enabled(false);
    const uint32_t modes[]={Current_Mode,Speed_Mode,Position_Mode,Vq_Mode,
        Voltage_OpenLoop,Calib_EleAngelOffset,Calib_EncoderObserver,Calib_Friction};
    for (unsigned i=0; i<sizeof(modes)/sizeof(modes[0]); ++i) {
        assert(MotorThermalControl_CanRunMode(modes[i]));
        assert(MotorThermalControl_Guard(modes[i]));
    }
    assert(MotorThermalControl_CurrentLimit(20)==9);
    assert(MotorThermalControl_CurrentLimit(6)==6);
    before=snapshot();
    assert(MotorThermalControl_CanClearFault() && MotorThermalControl_ClearFault());
    MotorThermalDiagnostics after=snapshot();
    assert(memcmp(&before.model,&after.model,sizeof(before.model))==0);
    /* 关闭期间仍累计热量；重新开启必须看到保留的热状态。 */
    if (!derating_only) {
        float old_heat=after.model.max_heat;
        step(7,true);
        assert(snapshot().model.max_heat>old_heat);
    }
    set_enabled(true);
    assert(!MotorThermalControl_CanRunMode(Calib_EleAngelOffset));
    assert(!MotorThermalControl_CanClearFault());
    if (!derating_only) {
        assert(!MotorThermalControl_CanRunMode(Current_Mode));
        assert(MotorThermalControl_CurrentLimit(20)==0);
    }
}
int main(int argc, char **argv) {
    assert(argc==2);
    assert(!MotorThermalControl_ProtectionEnabled());
    assert(!MotorThermalControl_RequestProtection(false,Motor_Disable,true));
    assert(!MotorThermalControl_CanRunMode(Current_Mode));
    assert(MotorThermalControl_CurrentLimit(20)==0);
    const bool heating=strcmp(argv[1],"hot")==0 || strcmp(argv[1],"derating")==0;
    const float values[]={heating?.01f:3,9,heating?.1f:65,.8f,.7f,1,.5f};
    assert(MotorThermalControl_LoadParameters(values));
    MotorThermalControl_Init(0);
    assert(!MotorThermalControl_ProtectionEnabled());
    if (strcmp(argv[1],"unconfirmed")==0) {
        assert(!MotorThermalControl_ParametersConfirmed());
        set_enabled(true);
        set_enabled(false);
        blocked();
    } else if (strcmp(argv[1],"lifecycle")==0) {
        lifecycle();
    } else if (heating) {
        hot(strcmp(argv[1],"derating")==0);
    } else {
        assert(!MotorThermalControl_ProtectionEnabled());
        if (strcmp(argv[1],"clock")==0) {
            assert(!MotorThermalControl_CheckWatchdog(3,false));
            assert(snapshot().watchdog_fault_pending);
            blocked();
            assert(!MotorThermalControl_RequestProtection(true,Motor_Disable,true));
            MotorThermalControl_Sample(0,0,0,true,3);
        } else if (strcmp(argv[1],"sample_clock")==0) {
            for (unsigned i=0; i<20; ++i) {
                MotorThermalControl_Sample(0,0,0,false,3);
            }
        } else {
            assert(strncmp(argv[1],"nan_",4)==0);
            MotorThermalControl_Sample(argv[1][4]=='a'?NAN:0,
                argv[1][4]=='b'?NAN:0,argv[1][4]=='c'?NAN:0,true,now_ms);
        }
        assert(!snapshot().model.valid);
        blocked();
        MotorThermalControl_Init(0);
        assert(!snapshot().model.valid);
        assert(!MotorThermalControl_RequestProtection(true,Motor_Disable,true));
    }
    printf("PASS temperature protection runtime: %s\n",argv[1]);
    return 0;
}
"""


def can_fixture(vectors: dict, switch_vectors: dict) -> str:
    """沿用真实 CAN 编解码夹具，并逐条执行开关协议黄金帧。"""
    fixture = configuration.can_fixture(vectors)
    original_main = thermal.function_source(fixture, "main")
    if "ModeNow_TypeDef ModeLast;" not in fixture:
        fixture = fixture.replace(
            "MotorControl_TypeDef MotorControl;",
            "MotorControl_TypeDef MotorControl;\nModeNow_TypeDef ModeLast;",
        )
    frame_rows = []
    for row in switch_vectors["frames"]:
        request = bytes.fromhex(row["request_hex"])
        reply = bytes.fromhex(row["reply_hex"])
        if len(request) != 4 or len(reply) != 4:
            raise ValueError("温度保护黄金帧必须使用四字节载荷")
        frame_rows.append(
            "{%d,0x%x,0x%08xU,0x%x,%s,{%s}}"
            % (
                row["node_id"],
                row["request_id"],
                int.from_bytes(request, "big"),
                row["reply_id"],
                "true"
                if row.get("applied_enabled", switch_vectors["default_enabled"])
                else "false",
                ",".join(f"0x{value:02x}" for value in reply),
            )
        )
    invalid_bits = [int(row["hex"], 16) for row in switch_vectors["invalid_set_values"]]
    rejected = next(row for row in switch_vectors["value_vectors"] if row["name"] == "rejected")
    rejected_bytes = bytes.fromhex(rejected["hex"])
    golden = r"""
static void golden_frames(void) {
    static const struct {
        unsigned node_id,request_id,request_bits,reply_id;
        bool applied_enabled;
        unsigned char reply[4];
    } frames[]={FRAME_ROWS};
    static const uint32_t invalid_bits[]={INVALID_BITS};
    static const unsigned char rejected[4]={REJECTED_BYTES};
    MotorControl.ModeNow=ModeLast=Motor_Disable;
    gate_off=true;
    for (unsigned i=0; i<sizeof(frames)/sizeof(frames[0]); ++i) {
        CANMsg.node_id=frames[i].node_id;
        if (MotorThermalControl_ProtectionEnabled()!=frames[i].applied_enabled) {
            assert(MotorThermalControl_RequestProtection(frames[i].applied_enabled,
                                                        Motor_Disable,true));
            consume();
        }
        frame(frames[i].request_id,IntBitToFloat(frames[i].request_bits));
        assert(CANMsg.can_tx_en && CANMsg.tx_data_len==4);
        assert(can_replies[can_reply_read % CAN_REPLY_CAPACITY].identifier==frames[i].reply_id && can_replies[can_reply_read % CAN_REPLY_CAPACITY].length==4);
        assert((unsigned)CANMsg.tx_param_id==(frames[i].reply_id & 0xff));
        assert(memcmp(CANMsg.tx_data_u8,frames[i].reply,4)==0);
        for (unsigned j=0;j<4;++j) assert(can_replies[can_reply_read % CAN_REPLY_CAPACITY].data[j]==frames[i].reply[j]);
        consume();
    }
    /* 非法值逐一通过真实解码器，节点3和4都必须保持原开关状态。 */
    for (unsigned node=3; node<=4; ++node) {
        CANMsg.node_id=node;
        if (!MotorThermalControl_ProtectionEnabled()) {
            assert(MotorThermalControl_RequestProtection(true,Motor_Disable,true));
            consume();
        }
        for (unsigned i=0;i<sizeof(invalid_bits)/sizeof(invalid_bits[0]);++i) {
            frame((node<<8)|SET_ID,IntBitToFloat(invalid_bits[i]));
            assert(CANMsg.can_tx_en && CANMsg.tx_data_len==4);
            assert(can_replies[can_reply_read % CAN_REPLY_CAPACITY].identifier==((node<<8)|GET_ID) && can_replies[can_reply_read % CAN_REPLY_CAPACITY].length==4);
            assert(memcmp(CANMsg.tx_data_u8,rejected,4)==0);
            for (unsigned j=0;j<4;++j) assert(can_replies[can_reply_read % CAN_REPLY_CAPACITY].data[j]==rejected[j]);
            consume();
            assert(MotorThermalControl_ProtectionEnabled());
        }
    }
    puts("PASS golden CAN frames: node3/node4 response IDs and bytes, all invalid SET values");
}
"""
    golden = (
        golden.replace("FRAME_ROWS", ",\n".join(frame_rows))
        .replace("INVALID_BITS", ",".join(f"0x{value:08x}U" for value in invalid_bits))
        .replace("REJECTED_BYTES", ",".join(f"0x{value:02x}" for value in rejected_bytes))
        .replace("SET_ID", str(switch_vectors["set_id"]))
        .replace("GET_ID", str(switch_vectors["get_id"]))
    )
    replacement = (
        golden
        + r"""
int main(void) {
    CANMsg.node_id=4;
    MotorControl.ModeNow=ModeLast=Motor_Disable;
    MotorThermalControl_Init(0);
    assert(!MotorThermalControl_ProtectionEnabled());
    frame(0x491,NAN); assert(CANMsg.can_tx_en && CANMsg.tx_data==0);
    frame(0x48e,0); assert(CANMsg.can_tx_en && CANMsg.tx_data==3);
    assert(MotorThermalControl_RequestProtection(true,Motor_Disable,true));
    consume(); assert(MotorThermalControl_ProtectionEnabled());
    const float invalid[]={NAN,INFINITY,-1,.5f,2};
    for (unsigned i=0; i<sizeof(invalid)/sizeof(invalid[0]); ++i) {
        write(0x90,invalid[i]);
        assert(CANMsg.tx_data==-1 && MotorThermalControl_ProtectionEnabled());
    }
    MotorControl.ModeNow=Current_Mode;
    write(0x90,0); assert(CANMsg.tx_data==-1);
    MotorControl.ModeNow=Motor_Disable; ModeLast=Current_Mode;
    write(0x90,0); assert(CANMsg.tx_data==-1);
    ModeLast=Motor_Disable; gate_off=false;
    write(0x90,0); assert(CANMsg.tx_data==-1);
    gate_off=true; write(0x90,0); assert(CANMsg.tx_data==0);
    assert(MotorThermalControl_ProtectionEnabled());
    frame(0x491,NAN); assert(CANMsg.tx_data==1);
    assert(!MotorThermalControl_CanRunMode(Current_Mode));
    assert(MotorThermalControl_CurrentLimit(7)==0);
    write(0x90,1); assert(CANMsg.tx_data==-1);
    consume(); assert(!MotorThermalControl_ProtectionEnabled());
    frame(0x491,NAN); assert(CANMsg.tx_data==0);
    zero_step();
    MotorThermalDiagnostics before=snapshot();
    assert(MotorThermalControl_CanRunMode(Current_Mode));
    write(0x90,1); assert(CANMsg.tx_data==1);
    consume(); assert(MotorThermalControl_ProtectionEnabled());
    MotorThermalDiagnostics after=snapshot();
    assert(memcmp(before.model.heat,after.model.heat,sizeof(before.model.heat))==0);
    frame(0x491,NAN); assert(CANMsg.tx_data==1);
    golden_frames();
    puts("PASS actual CAN 0x90/0x91: strict boolean, mode0, gates, deferred apply, readback, revision3");
    return 0;
}
"""
    )
    return fixture.replace(original_main, replacement)


def main() -> None:
    """编译生产热模块并运行独立场景，生成结果只写入 outputs。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", required=True)
    parser.add_argument("--out", type=Path, default=ROOT / "outputs/temperature_protection_switch")
    args = parser.parse_args()
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    (out / "main.h").write_text("#include <stdint.h>\n#include <stddef.h>\n", encoding="utf-8")
    source = out / "temperature_control_test.c"
    source.write_text(control_fixture(), encoding="utf-8")
    core = ROOT / "firmware/motor/protection/motor_thermal.c"
    control = ROOT / "firmware/app/motor_thermal_control.c"
    compiler = [args.cc] + (["cc"] if Path(args.cc).stem == "zig" else [])
    base = (
        compiler
        + ["-std=c99", "-O2", "-UNDEBUG", "-Wall", "-Wextra", "-Werror", "-I", str(out)]
        + thermal.NATIVE_INCLUDE_FLAGS
    )
    logs: list[str] = []
    checks: list[str] = []

    def execute(command: list[str]) -> None:
        result = subprocess.run(command, cwd=ROOT, capture_output=True, text=True)
        logs.append(result.stdout + result.stderr)
        (out / "tests.log").write_text("".join(logs), encoding="utf-8")
        print(logs[-1], end="", flush=True)
        result.check_returncode()

    for confirmed in (1, 0):
        exe = out / f"temperature_control_{confirmed}.exe"
        execute(
            base
            + [
                f"-DMOTOR_THERMAL_PARAMETERS_CONFIRMED={confirmed}",
                str(source),
                str(control),
                str(core),
                "-lm",
                "-o",
                str(exe),
            ]
        )
        scenarios = (
            ("lifecycle", "derating", "hot", "clock", "sample_clock", "nan_a", "nan_b", "nan_c")
            if confirmed
            else ("unconfirmed",)
        )
        for scenario in scenarios:
            execute([str(exe), scenario])
            checks.append(scenario)
    vectors = json.loads(
        (ROOT / "tests/vectors/motor_thermal_configuration_v1.json").read_text(encoding="utf-8")
    )
    switch_vectors_path = ROOT / "tests/vectors/motor_temperature_protection_switch_v1.json"
    switch_vectors = json.loads(switch_vectors_path.read_text(encoding="utf-8"))
    can_source = out / "temperature_can_test.c"
    can_source.write_text(can_fixture(vectors, switch_vectors), encoding="utf-8")
    can_exe = out / "temperature_can_test.exe"
    execute(
        base
        + [
            str(can_source),
            str(control),
            str(core),
            str(ROOT / "firmware/common/utils.c"),
            "-lm",
            "-o",
            str(can_exe),
        ]
    )
    execute([str(can_exe)])
    checks.append("actual_can_90_91")
    sources = (
        source,
        can_source,
        control,
        core,
        ROOT / "firmware/communication/can/interface_can.c",
        switch_vectors_path,
    )
    (out / "summary.json").write_text(
        json.dumps(
            {
                "status": "passed",
                "checks": checks,
                "hardware_actions": False,
                "golden_frames": len(switch_vectors["frames"]),
                "invalid_set_values_per_node": len(switch_vectors["invalid_set_values"]),
                "golden_nodes": sorted({row["node_id"] for row in switch_vectors["frames"]}),
                "sources_sha256": {
                    str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
                    for path in sources
                },
            },
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )


if __name__ == "__main__":
    main()
