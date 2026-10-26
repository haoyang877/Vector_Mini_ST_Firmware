"""编译真实外部参考、发布邮箱和两种入口；只使用显式内存反馈。"""

import argparse
import hashlib
import json
import subprocess
import sys
from pathlib import Path

from run_position_servo_tests import NATIVE_INCLUDE_FLAGS, ROOT, function_source


def main():
    """执行采样、超时、重入模式、边界及默认算法回归。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    (out / "main.h").write_text("#include <stdint.h>\n#include <stddef.h>\n", encoding="utf-8")
    logs = []

    def run(command):
        result = subprocess.run(command, cwd=ROOT, capture_output=True, text=True)
        logs.append(result.stdout + result.stderr)
        (out / "tests.log").write_text("".join(logs), encoding="utf-8")
        print(logs[-1], end="", flush=True)
        result.check_returncode()

    source = (ROOT / "tests/unit/position_servo_test.c").read_text(encoding="utf-8")
    (out / "config.inc").write_text(function_source(source, "config"), encoding="utf-8")
    compiler = [args.cc] + (["cc"] if Path(args.cc).stem == "zig" else [])
    common = compiler + NATIVE_INCLUDE_FLAGS + ["-std=c99", "-O2", "-UNDEBUG", "-Wall", "-Wextra", "-Werror", "-I", str(out), "-DPOSITION_EXTERNAL_REFERENCE_ENABLE=1"]
    controls = [ROOT / "firmware/motor/position/position_cascade.c",
                ROOT / "firmware/motor/trajectory/position_smooth_trajectory.c",
                ROOT / "firmware/motor/foc/foc_pid.c"]
    exe = out / "external_reference.exe"
    run(common + [str(ROOT / "tests/unit/position_external_reference_test.c"), *map(str, controls), "-o", str(exe)])
    run([str(exe)])

    # 默认分支先运行原回归，再复用其硬件边界替身测试候选路径的真实入口。
    run([sys.executable, str(ROOT / "tests/unit/native/test_outer_loop_runtime.py"), "--cc", args.cc, "--out", str(out / "legacy")])
    fixture = (out / "legacy/fixture.c").read_text(encoding="utf-8")
    app = (ROOT / "firmware/app/foc_run.c").read_text(encoding="utf-8")
    mailbox = app.split("/* POSITION_COMMAND_MAILBOX_BEGIN */")[1].split("/* POSITION_COMMAND_MAILBOX_END */")[0]
    prefix = "#include <float.h>\n#include \"fast_loop_profile.h\"\n#define test_now_ms motion_test_now_ms\n\n"
    at = fixture.index("static void motor_hw_outer_schedule(void)")
    at = fixture.index("\n", at) + 1
    fixture = fixture[:at] + prefix + mailbox + fixture[at:]
    fixture = fixture.replace("int main(void)", "int legacy_unused_main(void)")
    legacy_main = function_source(fixture, "legacy_unused_main")
    fixture = fixture.replace(legacy_main, legacy_main[:-1] + "return 0;\n}")
    fixture += '''
typedef struct {unsigned unused;} FOC_TypeDef;
static float Encoder_GetElePhase(Encoder_TypeDef *e) { (void)e;return 0; }
static float Encoder_GetEleVel(Encoder_TypeDef *e) { return e->speed; }
static void FOC_Current(FOC_TypeDef *f,MotorControl_TypeDef *m,float p,float v)
{(void)f;(void)m;(void)p;(void)v;}
'''
    fixture += function_source(app, "Task_Position_Mode_Reset")
    fixture += function_source(app, "Task_Position_Mode")
    fixture += '''
int main(void) {
    setup(Position_Mode);memset((void *)&position_command_mailbox,0,sizeof(position_command_mailbox));
    test_now_ms=0;enc.position=.2f;live.posRef=.2f;
    run_ticks(40);assert(fabsf(live.posShadow-.2f)<.000001f);
    MotorOuterLoop_PublishPositionTarget(.201f,0);
    run_ticks(40);assert(!outer.published_telemetry.external_reference_stale);
    test_now_ms=100;run_ticks(40);assert(outer.published_telemetry.external_reference_stale);
    live.ModeNow=Motor_Disable;live.iqRef=0;run_ticks(20);
    enc.position=.1f;live.ModeNow=Position_Mode;test_now_ms=110;
    run_ticks(40);assert(fabsf(live.posShadow-.1f)<.000001f);
    assert(outer.published_telemetry.external_reference_stale);
    MotorOuterLoop_PublishPositionTarget(.1f,110);run_ticks(40);
    assert(!outer.published_telemetry.external_reference_stale);
    assert(fabsf(live.posShadow-.1f)<.000001f);
    /* 非deferred入口也必须拒绝正好100ms旧帧。 */
    FOC_TypeDef foc={0};PositionCascade_Reset();test_now_ms=210;enc.position=.15f;
    Task_Position_Mode(&foc,&live,&enc);
    assert(fabsf(live.posShadow-.15f)<.000001f);
    PositionCascadeTelemetry_TypeDef t;assert(PositionCascade_GetTelemetry(&t));
    assert(t.external_reference_stale);
    /* 发布序号回绕到0仍是一条已发布的有效消息。 */
    position_command_mailbox.sequence=UINT32_MAX-1U;
    MotorOuterLoop_PublishPositionTarget(.15f,210);
    PositionExternalInput input={0};assert(MotorOuterLoop_ReadPositionTarget(&input));
    assert(input.sequence==0 && input.position_rad==.15f);
    position_command_mailbox.sequence=1;assert(!MotorOuterLoop_ReadPositionTarget(&input));
    puts("PASS actual publisher/deferred/direct: mode reentry, measured reseed, expiry100ms, sequence wrap, interrupted publication");
    return 0;
}
'''
    path = out / "entry.c"
    path.write_text(fixture, encoding="utf-8")
    exe = out / "entry.exe"
    run(common + ["-Wno-unused-function", str(path), *map(str, controls),
                  str(ROOT / "firmware/services/parameters/motor_axis_profile.c"),
                  str(ROOT / "firmware/motor/motor_load_profile.c"),
                  str(ROOT / "firmware/motor/speed_feedforward.c"), "-o", str(exe)])
    run([str(exe)])
    sources = ["firmware/app/foc_run.c", "firmware/motor/position/position_cascade.c", "firmware/motor/position/position_cascade_config.h"]
    (out / "summary.json").write_text(json.dumps({"status":"PASS", "hardware_accessed":False,
        "sources":{p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in sources}},indent=2)+"\n",encoding="utf-8")


if __name__ == "__main__":
    main()
