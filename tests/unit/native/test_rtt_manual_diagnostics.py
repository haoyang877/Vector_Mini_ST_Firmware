"""离线编译实际 RTT 编码器，验证诊断布局、快照有效性和丢帧计数；不接触硬件。"""

import argparse
import json
from pathlib import Path
import subprocess

from run_position_servo_tests import ROOT, NATIVE_INCLUDE_FLAGS, rtt_frame_fixture


def diagnostic_fixture() -> str:
    """复用旧帧测试的实际源码提取，替换诊断输入与传输桩。"""
    fixture = rtt_frame_fixture()
    fixture = (
        fixture.replace(
            "#define RTT_TELEMETRY_PROFILE RTT_TELEMETRY_CALIBRATION",
            "#define RTT_TELEMETRY_PROFILE RTT_TELEMETRY_MANUAL_DIAGNOSTICS\n"
            "#define Position_Mode 3",
        )
        .replace(
            "static struct { int ModeNow; float iqRef; } MotorControl;",
            "static struct { int ModeNow; float iqRef, posRef, pos_vel_filtered; } MotorControl;",
        )
        .replace(
            "static struct { float theta_elec; } OnBoard_Encoder;",
            "static struct { float theta_elec, theta_mech, vel_mech; } OnBoard_Encoder;",
        )
        .replace(
            "static struct { float Iq, Id, Vq, Vd, Ia, Ib, Ic; } FOC;",
            "static struct { float Iq, Id, Vq, Vd, Ia, Ib, Ic; } FOC;\n"
            "typedef struct { float speed_feedback, friction_feedforward_current; } "
            "PositionCascadeTelemetry_TypeDef;\n"
            "static PositionCascadeTelemetry_TypeDef planned;\nstatic bool valid = true;\n"
            "static bool MotorOuterLoop_GetTelemetry(PositionCascadeTelemetry_TypeDef *out) "
            "{ *out = planned; return valid; }\nstatic bool skip;",
        )
        .replace("wire[11]", "wire[10]")
        .replace("length == 22", "length == 20")
        .replace("++writes; return length;", "++writes; return skip ? 0U : length;")
    )
    fixture = (
        fixture[: fixture.index("int main(void)")]
        + r"""
int main(void)
{
    MotorControl.ModeNow = 3; MotorControl.iqRef = 1.5f;
    MotorControl.posRef = _PI / 2; MotorControl.pos_vel_filtered = .3f;
    FOC.Iq = 1.4f; FOC.Id = -.2f; FOC.Vq = 2.5f; FOC.Vd = -.6f;
    OnBoard_Encoder.theta_mech = -_PI / 4; OnBoard_Encoder.vel_mech = _PI;
    planned.speed_feedback = _PI / 4;
    planned.friction_feedforward_current = 2.5f;
    for (unsigned i = 0; i < 9; ++i) { RTT_Sampling(); }
    assert(writes == 0); RTT_Sampling(); assert(writes == 1);
    puts("GOLDEN");
    for (unsigned i = 0; i < 10; ++i) { printf("%d%s", wire[i], i == 9 ? "\n" : ","); }
    near_count(0,9000); near_count(1,-4500); near_count(2,18000); near_count(3,4500);
    assert(wire[4] == 1500 && wire[5] == 1400 && wire[6] == -200);
    assert(wire[7] == 2500 && wire[8] == -600 && wire[9] == 2500);
    assert(rtt_manual_dropped_frames == 0);
    skip = true; sample(); assert(rtt_manual_dropped_frames == 1);
    skip = false; sample(); assert(rtt_manual_dropped_frames == 1);
    valid = false; sample(); near_count(3,1718); assert(wire[9] == 0);
    valid = true; MotorControl.ModeNow = 0; sample();
    near_count(3,1718); assert(wire[9] == 0);
    MotorControl.posRef = 100; FOC.Iq = NAN; sample();
    assert(wire[0] == 32767 && wire[5] == 0);
    puts("PASS ten diagnostic channels, golden vector, idle validity and SKIP");
    return 0;
}
"""
    )
    return fixture


def main() -> None:
    """构建并执行本机桩，同时检查 Scope 与黄金向量的一致性。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", required=True)
    parser.add_argument(
        "--out", type=Path, default=ROOT / "outputs/rtt_manual_diagnostics_tests"
    )
    args = parser.parse_args()
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    source = out / "manual.c"
    executable = out / "manual.exe"
    source.write_text(diagnostic_fixture(), encoding="utf-8")
    compiler = [args.cc] + (["cc"] if Path(args.cc).stem == "zig" else [])
    subprocess.run(
        compiler
        + NATIVE_INCLUDE_FLAGS
        + ["-std=c99", "-O2", str(source), "-o", str(executable), "-lm"],
        check=True,
    )
    result = subprocess.run(
        [str(executable)], check=True, capture_output=True, text=True
    )
    print(result.stdout, end="")
    vector = json.loads(
        (ROOT / "tests/vectors/rtt_manual_diagnostics_v1.json").read_text()
    )
    counts = [int(value) for value in result.stdout.splitlines()[1].split(",")]
    if counts != vector["counts"]:
        raise ValueError(f"RTT 黄金向量不匹配：{counts}")
    import xml.etree.ElementTree as ET

    scope = ET.parse(ROOT / "tools/bench/scopes/pro_lks_manual_diagnostics.lksscope")
    channels = {
        int(item.attrib["name"].split(".data")[1]): item.attrib["unit"]
        for item in scope.findall(".//form[@type='5']/var")
    }
    if channels != {item["index"]: item["unit"] for item in vector["channels"]}:
        raise ValueError("Scope 通道索引或单位与黄金向量不一致")
    frequency = scope.find(".//param[@name='rttFreq']")
    if frequency is None or frequency.attrib["value"] != "2000":
        raise ValueError("Scope 采样率必须为 2000 Hz")


if __name__ == "__main__":
    main()
