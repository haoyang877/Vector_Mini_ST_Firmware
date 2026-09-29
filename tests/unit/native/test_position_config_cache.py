"""将真实模式 3 适配器与缓存前基线逐 tick 对比。

冻结基线从 Git 提取、只在 --out 下生成，永不进入固件；反馈为合成数据，
不是电机/被控对象仿真。
"""

import sys as _sys
from pathlib import Path as _Path

_sys.path.insert(0, str(_Path(__file__).resolve().parents[3] / "tools"))
import argparse
import re
import subprocess
from pathlib import Path

from _naming_reference import project_reference
from project_paths import NATIVE_INCLUDE_FLAGS, ROOT
from run_position_servo_tests import function_source


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--cc", required=True)
    p.add_argument("--out", type=Path, default=ROOT / "outputs/position_config_cache")
    p.add_argument("--reference-ref", default="f092e3d")
    args = p.parse_args()
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    source = (ROOT / "firmware/app/motor_execution.c").read_text(encoding="utf-8")
    reference = subprocess.check_output(
        ["git", "show", args.reference_ref + ":Foc/foc_run.c"], cwd=ROOT
    ).decode("utf-8")
    reference = function_source(reference, "Task_Position_Mode").replace(
        "Task_Position_Mode(", "Task_Position_Mode_Reference("
    )
    reference = project_reference(reference)
    # 历史核心没有独立保持速度滤波字段；新契约显式补足相同产品默认值。
    # 否则历史栈上的未初始化字段会触发现行核心的配置拒绝，无法比较缓存行为。
    reference = reference.replace(
        "config.velocity_filter_hz = POSITION_SERVO_VELOCITY_FILTER_HZ;",
        "config.velocity_filter_hz = POSITION_SERVO_VELOCITY_FILTER_HZ;\n"
        "config.hold_velocity_filter_hz = POSITION_SERVO_HOLD_VELOCITY_FILTER_HZ;",
    )
    # 仅将冻结基线中的常量名映射到现有等值常量；不恢复已删除的固件头文件。
    reference = reference.replace(
        "POSITION_IMPEDANCE_FRICTION_POSITIVE_A", "POSITION_SERVO_FALLBACK_FRICTION_POSITIVE_A"
    ).replace(
        "POSITION_IMPEDANCE_FRICTION_NEGATIVE_A", "POSITION_SERVO_FALLBACK_FRICTION_NEGATIVE_A"
    )
    actual = "\n".join(
        function_source(source, n)
        for n in [
            "position_mode_apply_friction_configuration",
            "position_mode_effective_deceleration",
            "position_mode_effective_maximum_speed",
            "position_mode_update_configuration",
            "position_mode_same_tuning_value",
            "position_mode_configuration_matches",
            "position_mode_reject_request",
            "Task_Position_Mode",
        ]
    )
    fields = sorted(
        set(re.findall(r"motor_state->(\w+)", actual + reference))
        - {"axis_profile", "axis_profile_valid", "friction_model_valid", "position_target_reached"}
    )
    typedef = (
        "typedef struct {\n"
        + "\n".join("float " + f + ";" for f in fields)
        + """
MotorAxisProfile axis_profile;
bool axis_profile_valid, friction_model_valid, position_target_reached;
} MotorControlState;
"""
    )
    # Only the hardware definitions consumed by the adapter's tuning header.
    (out / "hw_conf.h").write_text(
        """
#define MOTOR_DAMPING_FEEDFORWARD_ENABLED 1U
#define MOTOR_DAMPING_FEEDFORWARD 1U
#define Cascade_Position_Ts 0.0005f
#define CASCADE_POSITION_LOOP_DIVIDER 10U
""",
        encoding="utf-8",
    )
    fixture = (
        r"""
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <assert.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdint.h>
#include "firmware/platform/api/fast_loop_profile.h"
#include "position_cascade.h"
#include "hw_conf.h"
#include "position_cascade_config.h"
#include "firmware/services/parameters/motor_axis_profile.h"
#define FOC_CONFIG_NOINLINE
"""
        + typedef
        + r"""
typedef struct { float position, velocity; } EncoderState;
typedef struct { unsigned calls; } FocState;
enum { MOTOR_FAULT_INVALID_PARAMETERS = 12 };
static unsigned error;
static void Set_ErrorNow(unsigned value) { error = value; }
static void MotorFault_Report(MotorControlState *m, unsigned value) { (void)m; error = value; }
static float Encoder_GetElePhase(EncoderState *e) { return e->position * 7.0f; }
static float Encoder_GetMecPos(EncoderState *e) { return e->position; }
static float Encoder_GetEleVel(EncoderState *e) { return e->velocity * 7.0f; }
static float Encoder_GetMecVelContinuous(EncoderState *e) { return e->velocity; }
/* Deliberately no register/PWM writes in the host fixture. */
static void FOC_Current(FocState *f, MotorControlState *m, float a, float v) {
    (void)m; (void)a; (void)v; f->calls++;
}
"""
        + reference
        + "\n"
        + actual
        + r"""
typedef struct {
    MotorControlState motor;
    PositionCascadeTelemetry telemetry;
    unsigned error, calls;
    bool defer;
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
    unsigned scenario, pass, tick;
    PositionCascadeOutput o;
    PositionCascade_Reset();
    assert(PositionCascade_GetConfiguration()==NULL);
    assert(!PositionCascade_UpdateTarget(0,0,0,&o));
    for(scenario=0;scenario<21;scenario++) for(pass=0;pass<2;pass++) {
        MotorControlState m; FocState f={0}; EncoderState e;
        setup(&m); m.friction_model_valid=true;
        PositionCascade_Reset();
        for(tick=0;tick<1200;tick++) {
            Sample s;
            if(tick==601) { PositionCascade_Reset(); setup(&m); }
            change(&m,scenario,tick);
            e.position=.0001f * (float)((int)(tick%17)-8);
            e.velocity=.0002f * (float)((int)(tick%11)-5);
            if(scenario==20 && tick==263)
            {
                e.velocity=NAN;
            }
            error=0; f.calls=0;
            if(pass==0)
            {
                Task_Position_Mode_Reference(&f,&m,&e);
            }
            else
            {
                Task_Position_Mode(&f,&m,&e);
            }
            memset(&s,0,sizeof(s)); s.motor=m;
            PositionCascade_GetTelemetry(&s.telemetry);
            s.error=error; s.calls=f.calls; s.defer=PositionCascade_ShouldDeferTelemetry();
            if(pass==0)
            {
                expected[tick]=s;
            }
            else if(memcmp(&expected[tick],&s,sizeof(s))!=0) {
                fprintf(stderr,"Mismatch scenario=%u tick=%u error=%u/%u defer=%d/%d\n",
                    scenario,tick,expected[tick].error,s.error,expected[tick].defer,s.defer);
                return 1;
            }
        }
    }
    assert(!PositionCascade_UpdateTarget(NAN,0,0,&o));
    assert(!PositionCascade_UpdateTarget(0,NAN,0,&o));
    assert(!PositionCascade_UpdateTarget(0,0,NAN,&o));
    assert(!PositionCascade_UpdateTarget(0,0,0,NULL));
    puts("PASS 25200 tick comparisons: every live tuning field, NaN, signed zero, target/axis bounds, reset, telemetry/divider, same-tick rejection");
    return 0;
}
"""
    )
    src = out / "config_cache_test.c"
    src.write_text(fixture, encoding="utf-8")
    exe = out / "config_cache_test.exe"
    cc = [args.cc] + (["cc"] if Path(args.cc).stem == "zig" else [])
    log = []
    for command in [
        cc
        + [
            "-std=c99",
            "-O2",
            "-UNDEBUG",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-I",
            str(out),
            *NATIVE_INCLUDE_FLAGS,
            "-I",
            "firmware/motor/foc",
            "-I",
            ".",
            str(src),
            "firmware/motor/position/position_cascade.c",
            "firmware/motor/trajectory/position_smooth_trajectory.c",
            "firmware/motor/foc/foc_pid.c",
            "-o",
            str(exe),
        ],
        [str(exe)],
    ]:
        r = subprocess.run(command, cwd=ROOT, capture_output=True, text=True)
        log.append(r.stdout + r.stderr)
        (out / "test.log").write_text("\n".join(log), encoding="utf-8")
        print(log[-1], end="")
        r.check_returncode()


if __name__ == "__main__":
    main()
