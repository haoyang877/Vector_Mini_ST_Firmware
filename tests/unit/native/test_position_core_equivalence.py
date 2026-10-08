"""离线对照冻结的旧/新位置核心，逐拍比较私有状态和控制输出。

Reference sources and instrumented wrappers exist only under --out. No test
accessors or reference implementation are linked into firmware.
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


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", required=True)
    parser.add_argument("--reference-ref", default="3980f92")
    parser.add_argument(
        "--reference-source",
        type=Path,
        help="Optional frozen pre-change core, instead of the Git reference",
    )
    parser.add_argument("--out", type=Path, default=ROOT / "outputs/position_core_equivalence")
    args = parser.parse_args()
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    header = (ROOT / "firmware/motor/position/position_cascade.h").read_text(encoding="utf-8-sig")
    names = sorted(set(re.findall(r"\bPositionCascade_\w+(?=\()", header)))
    old_header = header
    names += ["PositionCascade_Update", "PositionCascade_UpdateTarget"]
    old = (
        args.reference_source.read_text(encoding="utf-8-sig")
        if args.reference_source
        else subprocess.check_output(
            ["git", "show", args.reference_ref + ":Foc/position_cascade.c"], cwd=ROOT
        ).decode()
    )
    old = old.replace("../System/fast_loop_profile.h", "fast_loop_profile.h").replace(
        "../system/fast_loop_profile.h", "fast_loop_profile.h"
    )
    reference_uses_control_output = "bool PositionCascade_UpdateControl(" in old
    old = project_reference(old, "firmware/motor/position/position_cascade.c")
    reference_state_type = re.search(r"typedef struct\s*\{[^{}]+\}\s*PositionCascadeState;", old)[0]
    legacy_header = subprocess.check_output(
        ["git", "show", "05d98016:firmware/motor/position/position_cascade.h"], cwd=ROOT
    ).decode("utf-8")
    legacy_end = legacy_header.index("} PositionCascadeOutput;") + len("} PositionCascadeOutput;")
    legacy_start = legacy_header.rfind("typedef struct", 0, legacy_end)
    rich_type = legacy_header[legacy_start:legacy_end]
    legacy_prototypes = """
bool PositionCascade_Update_Reference(const PositionCascadeConfig *,float,float,PositionCascadeOutput *);
bool PositionCascade_UpdateTarget_Reference(float,float,float,PositionCascadeOutput *);
"""
    for name in names:
        old = re.sub(r"\b" + name + r"\b", name + "_Reference", old)
        old_header = re.sub(r"\b" + name + r"\b", name + "_Reference", old_header)
    # Use the original type declarations once, followed by reference prototypes.
    # 定位真实声明，避免把可翻译的契约注释作为源码切分标记。
    prototypes = old_header[
        old_header.index("bool PositionCascade_UpdateControl_Reference(") : old_header.rindex(
            "#endif"
        )
    ]
    prototypes += legacy_prototypes
    old = old.replace(
        '#include "position_cascade.h"',
        '#include "position_cascade.h"\n' + rich_type + "\n" + prototypes,
    )
    old += "\nconst void *PositionCore_ReferenceState(void) { return &s_state; }\n"
    old += "size_t PositionCore_ReferenceSize(void) { return sizeof(s_state); }\n"
    actual = (ROOT / "firmware/motor/position/position_cascade.c").read_text(encoding="utf-8-sig")
    separated_state = "PositionCascadeCoreState core;" in actual
    for name in ["Reset", "Update", "UpdateTarget", "UpdateControl", "UpdateTargetControl"]:
        actual = actual.replace(
            "PositionCascade_" + name + "(", "PositionCascade_" + name + "_Actual("
        )
    actual += (
        "\n"
        + rich_type
        + "\n"
        + prototypes
        + r"""
#include <stdio.h>
#include <stdlib.h>
extern const void *PositionCore_ReferenceState(void);
extern size_t PositionCore_ReferenceSize(void);
unsigned long PositionCore_ComparedTicks;
static void compare_state(void) {
    const unsigned char *a = (const unsigned char *)&s_state;
    const unsigned char *b = (const unsigned char *)PositionCore_ReferenceState();
    size_t i;
    if (sizeof(s_state) != PositionCore_ReferenceSize()) abort();
    for (i=0; i<sizeof(s_state); ++i) if (a[i] != b[i]) {
        fprintf(stderr,"State mismatch tick %lu byte %zu: %02x / %02x\n",
            PositionCore_ComparedTicks,i,a[i],b[i]); abort();
    }
}
void PositionCascade_Reset(void) {
    PositionCascade_Reset_Actual(); PositionCascade_Reset_Reference(); compare_state();
}
static bool compare_control(bool x,bool y,PositionCascadeControlOutput *output,
    const PositionCascadeControlOutput *a,const PositionCascadeOutput *b) {
    PositionCascadeControlOutput expected={0};
    expected.position_reference=b->position_reference;
    expected.speed_reference=b->speed_reference;
    expected.speed_feedback_rad_s=b->speed_feedback_rad_s;
    expected.iq_reference=b->iq_reference;
    expected.target_reached=b->target_reached;
    ++PositionCore_ComparedTicks;
    if(x!=y || memcmp(a,&expected,sizeof(expected))) abort();
    compare_state();
    if(output && x) *output=*a;
    return x;
}
bool PositionCascade_UpdateControl(const PositionCascadeConfig *config,
    float position,float speed,PositionCascadeControlOutput *output) {
    PositionCascadeControlOutput a={0}; PositionCascadeOutput b={0};
    bool x=PositionCascade_UpdateControl_Actual(config,position,speed,output ? &a : NULL);
    bool y=PositionCascade_Update_Reference(config,position,speed,output ? &b : NULL);
    return compare_control(x,y,output,&a,&b);
}
bool PositionCascade_UpdateTargetControl(float target,float position,float speed,
    PositionCascadeControlOutput *output) {
    PositionCascadeControlOutput a={0}; PositionCascadeOutput b={0};
    bool x=PositionCascade_UpdateTargetControl_Actual(target,position,speed,output ? &a : NULL);
    bool y=PositionCascade_UpdateTarget_Reference(target,position,speed,output ? &b : NULL);
    return compare_control(x,y,output,&a,&b);
}
"""
    )
    if reference_uses_control_output:
        # 当前基线已采用精简结果；继续比较完整私有状态及结果字节。
        actual = actual.replace(
            "const PositionCascadeOutput *b) {", "const PositionCascadeControlOutput *b) {"
        ).replace("PositionCascadeOutput b={0};", "PositionCascadeControlOutput b={0};")
        actual = actual.replace(
            "PositionCascade_Update_Reference(config,",
            "PositionCascade_UpdateControl_Reference(config,",
        )
        actual = actual.replace(
            "PositionCascade_UpdateTarget_Reference(target,",
            "PositionCascade_UpdateTargetControl_Reference(target,",
        )
    if separated_state:
        # 按冻结布局投影每个旧字段，覆盖删除的等价缓存；不比较新的布局填充。
        state_body = reference_state_type[
            reference_state_type.index("{") + 1 : reference_state_type.rindex("}")
        ]
        state_members = re.findall(r"\b(\w+)\s*;", state_body)
        field_map = {
            "speed_controller": "core.speed_controller",
            "speed_reference": "core.speed_controller.reference",
            "feedback_current": "core.speed_controller.output",
            "iq_reference": "core.iq_reference_a",
            "current_saturated": "core.current_saturated",
            "friction_feedforward_current": "friction.current_a",
            "friction_breakaway_start_position": "friction.breakaway_start_position_rad",
            "friction_stuck_counter": "friction.stuck_counter",
            "friction_direction": "friction.direction",
            "friction_breakaway_active": "friction.breakaway_active",
            "friction_landing_active": "friction.landing_active",
            "settle_recovery_active": "friction.recovery_active",
            "friction_current_ramping": "friction.current_ramping",
        }
        projection = reference_state_type.replace(
            "PositionCascadeState;", "PositionCascadeReferenceState;"
        )
        actual = actual.replace(
            "static void compare_state(void) {", projection + "\nstatic void compare_state(void) {"
        )
        copy_fields = "\n".join(
            f"    memcpy(&projected.{name}, &s_state.{field_map.get(name, name)}, sizeof(projected.{name}));"
            for name in state_members
        )
        actual = actual.replace(
            "    const unsigned char *a = (const unsigned char *)&s_state;",
            "    PositionCascadeReferenceState projected={0};\n"
            + copy_fields
            + "\n    const unsigned char *a = (const unsigned char *)&projected;",
        )
        actual = actual.replace(
            "if (sizeof(s_state) != PositionCore_ReferenceSize()) abort();",
            "if (sizeof(projected) != PositionCore_ReferenceSize()) abort();",
        ).replace("i<sizeof(s_state)", "i<sizeof(projected)")
        # 两套独立状态直接调用私有核心；同时核对生产组合状态未被读写耦合。
        friction_field_sources = {
            "current_limit_a": "current_limit_a",
            "deceleration_rad_s2": "deceleration",
            "attack_slew_a_per_s": "friction_attack_slew_rate",
            "breakaway_distance_rad": "friction_breakaway_distance",
            "breakaway_ratio": "friction_breakaway_ratio",
            "coulomb_negative_a": "friction_coulomb_negative",
            "coulomb_positive_a": "friction_coulomb_positive",
            "fast_release_slew_a_per_s": "friction_fast_release_slew_rate",
            "enabled": "friction_feedforward_enabled",
            "move_speed_rad_s": "friction_move_speed",
            "reference_speed_rad_s": "friction_reference_speed",
            "release_slew_a_per_s": "friction_release_slew_rate",
            "stop_speed_rad_s": "friction_stop_speed",
            "viscous_negative_a_per_rad_s": "friction_viscous_negative",
            "viscous_positive_a_per_rad_s": "friction_viscous_positive",
            "capture_enter_rad": "hold_enter_position",
            "capture_exit_rad": "hold_exit_position",
            "reference_finish_window_rad": "position_error_window",
            "update_period_s": "update_period_s",
        }
        isolation = r"""
void PositionCore_TestModuleIsolation(const PositionCascadeConfig *public_config) {
    unsigned char global_before[sizeof(s_state)];
    PositionCascadeCoreState first, second, second_before, first_before_friction;
    PositionCascadeFrictionState friction_first, friction_second, friction_second_before;
    PositionCascadeCoreSpeedConfig tuning={.position_kp_per_s=8,.position_kd=2,
        .maximum_speed_rad_s=1,.speed_limit_rad_s=2};
    PositionCascadeCoreMotion motion={.position_reference_rad=.2f,.trajectory_speed_rad_s=.1f,
        .measured_position_rad=.05f,.control_velocity_rad_s=.02f};
    PositionCascadeFrictionConfig model={ FRICTION_FIELDS };
    PositionCascadeFrictionInput friction_motion={.target_position_rad=.2f,
        .position_reference_rad=0,.trajectory_speed_rad_s=.1f,.measured_position_rad=0,
        .measured_speed_rad_s=0,.stuck_ticks=100,.moving=true};
    memcpy(global_before,&s_state,sizeof(s_state));
    position_cascade_core_reset(&first); position_cascade_core_reset(&second);
    memcpy(&second_before,&second,sizeof(second));
    float speed=position_cascade_core_speed(&tuning,&motion);
    PositionCascadeCoreCurrent current={speed,.02f,.3f,.4f,10,6};
    if(!position_cascade_core_run_current(&first,&current,.0005f,6)) abort();
    PositionCascadeCoreOutput output=position_cascade_core_output(&first);
    if(output.iq_reference_a!=6 || !output.current_saturated || output.feedback_current_a!=0) abort();
    if(memcmp(&second,&second_before,sizeof(second))) abort();
    PositionCascadeStaticIntegral request={.increment_a=.05f,.apply=true};
    position_cascade_core_apply_static_integral(&first,request,6);
    if(memcmp(&second,&second_before,sizeof(second))) abort();
    memcpy(&first_before_friction,&first,sizeof(first));
    position_cascade_friction_reset(&friction_first);
    position_cascade_friction_reset(&friction_second);
    memcpy(&friction_second_before,&friction_second,sizeof(friction_second));
    position_cascade_friction_update(&friction_first,&model,&friction_motion);
    PositionCascadeFrictionOutput friction_output=position_cascade_friction_output(&friction_first);
    if(friction_output.current_a<=0 || memcmp(&friction_second,&friction_second_before,sizeof(friction_second))) abort();
    position_cascade_friction_target_changed(&friction_first);
    if(position_cascade_friction_output(&friction_first).current_a!=friction_output.current_a) abort();
    model.enabled=false;
    position_cascade_friction_update(&friction_first,&model,&friction_motion);
    if(position_cascade_friction_output(&friction_first).current_a!=0) abort();
    if(memcmp(&first,&first_before_friction,sizeof(first))) abort();
    if(memcmp(global_before,&s_state,sizeof(s_state))) abort();
    puts("PASS independent core/friction instances: total-current tracking, retarget, disable, no PI/global coupling");
}
""".replace(
            "FRICTION_FIELDS",
            ",".join(
                f".{field}=public_config->{original}"
                for field, original in friction_field_sources.items()
            ),
        )
        actual += isolation
    fixture = (
        (ROOT / "tests/unit/position_servo_test.c")
        .read_text(encoding="utf-8-sig")
        .replace("int main(", "int scenarios_main(")
    )
    fixture += r"""
extern unsigned long PositionCore_ComparedTicks;
int main(int argc,char **argv) {
    int result=scenarios_main(argc,argv);
    PositionCascadeControlOutput output;
    PositionCascadeConfig tuning=config();
    unsigned tick;
    PositionCascade_Reset();
    assert(!PositionCascade_UpdateTargetControl(0,0,0,&output));
    tuning.call_divider=10;
    assert(PositionCascade_UpdateControl(&tuning,0,0,&output));
    for(tick=0;tick<2400;++tick) {
        float target=tick<800 ? .1f : tick<1600 ? -.1f : -0.0f;
        float position=.001f*(float)((int)(tick%17)-8);
        float speed=.002f*(float)((int)(tick%11)-5);
        assert(PositionCascade_UpdateTargetControl(target,position,speed,&output));
    }
    assert(!PositionCascade_UpdateTargetControl(NAN,0,0,&output));
    assert(!PositionCascade_UpdateTargetControl(0,NAN,0,&output));
    assert(!PositionCascade_UpdateTargetControl(0,0,NAN,&output));
    assert(!PositionCascade_UpdateTargetControl(0,0,0,NULL));
    printf("PASS %lu ticks: complete private state and outputs match reference byte-for-byte\n",
        PositionCore_ComparedTicks);
    return result;
}
"""
    if reference_uses_control_output:
        fields = re.findall(r"\bfloat\s+(\w+)\s*;", header.split("} PositionCascadeConfig;")[0])
        # 对每个调参字段检查边界，并在有/无旧缓存时比较拒绝后的目标更新。
        extra = r"""
#include <float.h>
static void compare_config_boundaries(void) {
    static const size_t offsets[] = { OFFSETS };
    static const float values[] = {
        NAN, INFINITY, -INFINITY, -FLT_MAX, FLT_MAX, -1.0f, -0.0f, 0.0f, .5f, 1.0f, 50.0f
    };
    PositionCascadeControlOutput output;
    for(unsigned cached=0;cached<2;++cached) {
        for(size_t field=0;field<sizeof(offsets)/sizeof(offsets[0]);++field) {
            for(size_t item=0;item<sizeof(values)/sizeof(values[0])+3;++item) {
                PositionCascadeConfig tuning=config();
                float original, value;
                memcpy(&original,(const unsigned char *)&tuning+offsets[field],sizeof(original));
                value=item<sizeof(values)/sizeof(values[0]) ? values[item] :
                    item==sizeof(values)/sizeof(values[0]) ? original :
                    nextafterf(original,item==sizeof(values)/sizeof(values[0])+1 ? -INFINITY : INFINITY);
                PositionCascade_Reset();
                if(cached) (void)PositionCascade_UpdateControl(&tuning,0,0,&output);
                memcpy((unsigned char *)&tuning+offsets[field],&value,sizeof(value));
                (void)PositionCascade_UpdateControl(&tuning,0,0,&output);
                (void)PositionCascade_UpdateTargetControl(.1f,0,0,&output);
            }
        }
    }
    PositionCascade_Reset();
    assert(!PositionCascade_UpdateControl(NULL,0,0,&output));
    for(unsigned cached=0;cached<2;++cached) {
        PositionCascadeConfig tuning=config();
        PositionCascade_Reset();
        if(cached) (void)PositionCascade_UpdateControl(&tuning,0,0,&output);
        tuning.call_divider=0;
        assert(!PositionCascade_UpdateControl(&tuning,0,0,&output));
    }
    puts("PASS every configuration float: NaN, infinities, signed zero, extremes, adjacent bounds, cached rejection");
}
""".replace("OFFSETS", ",".join(f"offsetof(PositionCascadeConfig,{name})" for name in fields))
        fixture = fixture.replace(
            "int main(int argc,char **argv) {", extra + "\nint main(int argc,char **argv) {"
        )
        fixture = fixture.replace(
            '    printf("PASS %lu ticks:',
            '    compare_config_boundaries();\n    printf("PASS %lu ticks:',
        )
    if separated_state:
        fixture = fixture.replace(
            "int main(int argc,char **argv) {",
            "void PositionCore_TestModuleIsolation(const PositionCascadeConfig *);\n"
            "int main(int argc,char **argv) {",
        ).replace(
            '    printf("PASS %lu ticks:',
            '    PositionCore_TestModuleIsolation(&tuning);\n    printf("PASS %lu ticks:',
        )
        fixture = fixture.replace(
            "complete private state and outputs match reference byte-for-byte",
            "every legacy state field and control output match reference byte-for-byte (explicit layout projection)",
        )
    paths = []
    for name, source in [("reference.c", old), ("actual.c", actual), ("scenarios.c", fixture)]:
        path = out / name
        path.write_text(source, encoding="utf-8")
        paths.append(str(path))
    cc = [args.cc] + (["cc"] if Path(args.cc).stem == "zig" else []) + NATIVE_INCLUDE_FLAGS
    exe = out / "equivalence.exe"
    command = (
        cc + ["-std=c99", "-O2", "-Wall", "-Wextra", "-Werror", "-I", "firmware/motor/foc"] + paths
    )
    command += [
        "firmware/motor/trajectory/position_smooth_trajectory.c",
        "firmware/motor/foc/foc_pid.c",
        "-o",
        str(exe),
    ]
    logs = []
    for cmd in [command, [str(exe)]]:
        result = subprocess.run(cmd, cwd=ROOT, text=True, capture_output=True)
        logs.append(result.stdout + result.stderr)
        (out / "test.log").write_text("\n".join(logs), encoding="utf-8")
        print(logs[-1], end="")
        result.check_returncode()


if __name__ == "__main__":
    main()
