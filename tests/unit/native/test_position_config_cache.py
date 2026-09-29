"""对照修改前真实 2 kHz 外环，逐拍检查调参缓存、故障及遥测发布。"""

import sys as _sys
from pathlib import Path as _Path

_sys.path.insert(0, str(_Path(__file__).resolve().parents[3] / "tools"))
import argparse
import subprocess
from pathlib import Path

from project_paths import NATIVE_INCLUDE_FLAGS, ROOT
from run_position_servo_tests import function_source

CONFIG_HELPERS = (
    "position_mode_apply_friction_configuration",
    "position_mode_effective_deceleration",
    "position_mode_effective_maximum_speed",
    "position_mode_update_configuration",
    "position_mode_same_tuning_value",
    "position_mode_configuration_matches",
)


def main():
    """冻结旧外环只编入主机夹具；生产始终只有一个运行入口。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", required=True)
    parser.add_argument("--out", type=Path, default=ROOT / "outputs/position_config_cache")
    parser.add_argument("--reference-ref", default="05d98016")
    args = parser.parse_args()
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    actual = (ROOT / "firmware/app/motor_execution.c").read_text(encoding="utf-8")
    reference = subprocess.check_output(
        ["git", "show", args.reference_ref + ":firmware/app/motor_execution.c"], cwd=ROOT
    ).decode("utf-8")
    speed = (ROOT / "firmware/motor/foc/foc_speed.c").read_text(encoding="utf-8")
    (out / "speed.c").write_text(
        '#include "motor_types.h"\n#include "utils.h"\n#include "control_config.h"\n'
        '#include "foc_pid.h"\n'
        + "\n".join(
            function_source(speed, n)
            for n in ("MotorAxis_UpdateSpeedRamp", "SpeedMode_UpdateControl")
        ),
        encoding="utf-8",
    )
    for tag, source in (("Reference", reference), ("Actual", actual)):
        block = (
            "/* OUTER_RUNTIME_BEGIN"
            + source.split("/* OUTER_RUNTIME_BEGIN", 1)[1].split("/* OUTER_RUNTIME_END */", 1)[0]
        )
        prelude = r"""
#include <math.h>
#include <string.h>
#include "motor_types.h"
#include "utils.h"
#include "control_config.h"
#include "foc_pid.h"
#include "position_cascade.h"
#include "position_cascade_config.h"
#define FOC_CONFIG_NOINLINE
typedef struct { float position, velocity; } EncoderState;
static float Encoder_GetMecPos(EncoderState *e) { return e->position; }
static float Encoder_GetMecVel(EncoderState *e) { return e->velocity; }
static float Encoder_GetMecVelContinuous(EncoderState *e) { return e->velocity; }
static void MotorFault_Report(MotorControlState *m, MotorFaultCode fault) { m->fault_code=fault; }
void SpeedMode_UpdateControl(MotorControlState *, PiController *, float);
"""
        # 每个翻译单元有自己的缓存和发布状态；两次遍历之间显式复位共享纯控制器。
        prelude += "\n".join(
            "#define MotorOuterLoop_" + n + " MotorOuterLoop_" + n + "_" + tag
            for n in ("SlowTick", "IsReady", "GetTelemetry")
        )
        source = (
            prelude
            + "\n"
            + "\n".join(function_source(source, n) for n in CONFIG_HELPERS)
            + "\n"
            + block
        )
        source += """
void Cache_TAG_Reset(void) { memset(&s_outer,0,sizeof(s_outer)); PositionCascade_Reset(); }
void Cache_TAG_Step(MotorControlState *m, PiController *pi, float p, float v) {
    EncoderState e={p,v}; MotorOuterLoop_SlowTick(m,pi,&e);
}
void Cache_TAG_Read(PositionCascadeTelemetry *t, bool *ready, bool *valid) {
    *ready=MotorOuterLoop_IsReady(); *valid=MotorOuterLoop_GetTelemetry(t);
}
""".replace("TAG", tag)
        (out / (tag + ".c")).write_text(source, encoding="utf-8")
    fixture = (
        r"""
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <assert.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "motor_types.h"
#include "position_cascade.h"
#include "foc_pid.h"
void Cache_Reference_Reset(void);
void Cache_Actual_Reset(void);
void Cache_Reference_Step(MotorControlState *,PiController *,float,float);
void Cache_Actual_Step(MotorControlState *,PiController *,float,float);
void Cache_Reference_Read(PositionCascadeTelemetry *,bool *,bool *);
void Cache_Actual_Read(PositionCascadeTelemetry *,bool *,bool *);
typedef struct {
    MotorControlState motor;
    PiController pi;
    PositionCascadeTelemetry telemetry;
    bool ready, valid, defer;
} Sample;
static Sample expected[1200];
static void setup(MotorControlState *m) {
    memset(m, 0, sizeof(*m));
    m->axis_profile_valid = true;
    m->position_error_window_rad = .001f; m->position_acceleration_rad_s2 = .785398f; m->position_deceleration_rad_s2 = .523599f;
    m->position_maximum_speed_rad_s = .6f; m->speed_limit_rad_s = .8f;
    m->cascade_position_kp = 8; m->cascade_position_kd = 2;
    m->speed_kp = .5f; m->speed_ki = 2; m->current_limit_a = 6;
    m->friction_coulomb_pos_a = 1.4f; m->friction_coulomb_neg_a = 1.3f;
    m->friction_viscous_pos_a_per_rad_s = .1f;
    m->friction_viscous_neg_a_per_rad_s = .2f;
}
static void change(MotorControlState *m, unsigned scenario, unsigned tick) {
    if (tick == 127 || tick == 253 || tick == 411)
    {
        m->position_target_rad += tick == 253 ? -.18f : .09f;
    }
    if (tick != 131 && tick != 271)
    {
        return;
    }
    switch(scenario) {
"""
        + "\n".join(
            f"case {i}: m->{field} = tick == 131 ? {value} : NAN; break;"
            for i, (field, value) in enumerate(
                [
                    ("position_error_window_rad", ".0011f"),
                    ("position_acceleration_rad_s2", ".5f"),
                    ("position_deceleration_rad_s2", ".3f"),
                    ("position_maximum_speed_rad_s", ".4f"),
                    ("speed_limit_rad_s", ".7f"),
                    ("cascade_position_kp", "9.0f"),
                    ("cascade_position_kd", "1.0f"),
                    ("speed_kp", ".3f"),
                    ("speed_ki", "1.0f"),
                    ("current_limit_a", "4.0f"),
                    ("friction_coulomb_pos_a", "1.2f"),
                    ("friction_coulomb_neg_a", "1.1f"),
                    ("friction_viscous_pos_a_per_rad_s", ".3f"),
                    ("friction_viscous_neg_a_per_rad_s", ".4f"),
                ]
            )
        )
        + r"""
    case 14: m->friction_model_valid = !m->friction_model_valid; break;
    case 15: m->axis_profile.magic=1; m->axis_profile.minimum_position_rad=-1;
        m->axis_profile.maximum_position_rad=1;
        m->axis_profile.maximum_speed_rad_s=tick==131 ? .3f : .2f; break;
    case 16: m->axis_profile_valid=false; break;
    case 17: m->position_target_rad=NAN; break;
    case 18: m->speed_ki=tick==131 ? 0.0f : -0.0f; break;
    case 19: m->position_deceleration_rad_s2=tick==131 ? 1.0f : 2.0f; break;
    default: break;
    }
}
int main(void) {
    for(unsigned scenario=0;scenario<21;scenario++) for(unsigned pass=0;pass<2;pass++) {
        MotorControlState m; PiController pi={0};
        setup(&m); m.control_mode=MOTOR_CONTROL_POSITION; m.friction_model_valid=true;
        if(pass==0) Cache_Reference_Reset(); else Cache_Actual_Reset();
        for(unsigned tick=0;tick<1200;tick++) {
            Sample s={0};
            if(tick==601) {
                if(pass==0) Cache_Reference_Reset(); else Cache_Actual_Reset();
                setup(&m); m.control_mode=MOTOR_CONTROL_POSITION; memset(&pi,0,sizeof(pi));
            }
            change(&m,scenario,tick);
            float position=.0001f*(float)((int)(tick%17)-8);
            float velocity=.0002f*(float)((int)(tick%11)-5);
            if(scenario==20 && tick==263) velocity=NAN;
            if(pass==0) Cache_Reference_Step(&m,&pi,position,velocity);
            else Cache_Actual_Step(&m,&pi,position,velocity);
            s.motor=m; s.pi=pi; s.defer=PositionCascade_ShouldDeferTelemetry();
            if(pass==0) Cache_Reference_Read(&s.telemetry,&s.ready,&s.valid);
            else Cache_Actual_Read(&s.telemetry,&s.ready,&s.valid);
            if(pass==0) expected[tick]=s;
            else if(memcmp(&expected[tick],&s,sizeof(s))) {
                fprintf(stderr,"cache mismatch scenario=%u tick=%u\n",scenario,tick);
                return 1;
            }
        }
    }
    puts("PASS 25200 actual 2 kHz outer-loop comparisons: live tuning fields, NaN, signed zero, axis bounds, reset, fault and telemetry");
    return 0;
}
"""
    )
    (out / "scenario.c").write_text(fixture, encoding="utf-8")
    cc = [args.cc] + (["cc"] if Path(args.cc).stem == "zig" else [])
    exe = out / "cache.exe"
    command = (
        cc + NATIVE_INCLUDE_FLAGS + ["-std=c99", "-O2", "-UNDEBUG", "-Wall", "-Wextra", "-Werror"]
    )
    command += [str(out / n) for n in ("Reference.c", "Actual.c", "speed.c", "scenario.c")]
    command += [
        "firmware/motor/position/position_cascade.c",
        "firmware/motor/trajectory/position_smooth_trajectory.c",
        "firmware/motor/foc/foc_pid.c",
        "-o",
        str(exe),
    ]
    logs = []
    for invocation in (command, [str(exe)]):
        result = subprocess.run(invocation, cwd=ROOT, capture_output=True, text=True)
        logs.append(result.stdout + result.stderr)
        (out / "test.log").write_text("\n".join(logs), encoding="utf-8")
        print(logs[-1], end="")
        result.check_returncode()


if __name__ == "__main__":
    main()
