"""Compile the actual deferred runtime with deterministic interrupt injection.

Checks publication ownership, stale reset/fault rejection, deadline faults,
2 kHz cadence and unchanged speed PI/ramp arithmetic.
Synthetic feedback is not an acceptance test of the physical plant.
"""

import argparse
import subprocess
from pathlib import Path

from run_position_servo_tests import NATIVE_INCLUDE_FLAGS, ROOT, function_source


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--cc", required=True)
    p.add_argument("--out", type=Path, default=ROOT / "outputs/outer_loop_host")
    args = p.parse_args()
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    (out / "main.h").write_text("#include <stdint.h>\n#include <stddef.h>\n")
    src = (ROOT / "firmware/app/foc_run.c").read_text(encoding="utf-8")
    block = src.split("/* OUTER_RUNTIME_BEGIN", 1)[1].split("/* OUTER_RUNTIME_END */", 1)[0]
    block = "/* OUTER_RUNTIME_BEGIN" + block
    funcs = "\n".join(
        function_source(src, n)
        for n in [
            "SpeedMode_UpdateControl",
            "PositionMode_UpdateConfiguration",
            "PositionMode_SameTuningValue",
            "PositionMode_ConfigurationMatches",
        ]
    )
    fixture = (
        r"""
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "data_type.h"
#include "motor_load_profile.h"
#include "hw_conf.h"
#include "foc_pid.h"
#include "position_cascade.h"
#include "position_cascade_config.h"
#include "position_impedance_config.h"
#include "motor_thermal.h"
#define FOC_CONFIG_NOINLINE
typedef struct { float position, speed; } Encoder_TypeDef;
static MotorControl_TypeDef live;
static PI_Controller_TypeDef live_pi;
static Encoder_TypeDef enc;
static unsigned schedules, injection;
/** @brief 单线程模拟一次ADC发布抢占，屏障消费后清除。 */
static bool motion_read_injection;
static unsigned position_reads, speed_reads, continuous_speed_reads;
static float Encoder_GetMecPos(Encoder_TypeDef *e) { ++position_reads; return e->position; }
static float Encoder_GetMecVel(Encoder_TypeDef *e) { ++speed_reads; return e->speed; }
static float Encoder_GetMecVelContinuous(Encoder_TypeDef *e) { ++continuous_speed_reads; return e->speed; }
static void Set_ErrorNow(ErrorNow_TypeDef error) { live.ErrorNow = error; }
/** @brief 测试拥有的设备单调毫秒，不读取硬件。 */
static uint32_t motion_test_now_ms;
/** @brief 返回测试毫秒时钟，不产生硬件访问。
 * @return 当前模拟时刻，允许uint32回绕。
 */
static uint32_t time_hw_now_ms(void) {return motion_test_now_ms;}
static void motor_hw_outer_barrier(void);
static void motor_hw_outer_schedule(void) { schedules++; }
"""
        + funcs
        + block
        + r"""
static void tick(void) { MotorOuterLoop_FastTick(&live, &live_pi, &enc); }
static void motor_hw_outer_barrier(void) {
    if (motion_read_injection) {
        motion_read_injection = false;
        outer.telemetry_sequence += 2;
        outer.published_telemetry.position_reference = 4;
        outer.published_telemetry.trajectory_speed_reference = 5;
        outer.published_telemetry.speed_command = 6;
    }
    if (injection && outer.status == OUTER_RUNNING) {
        unsigned action = injection;
        float old_iq = live.iqRef;
        PositionCascadeTelemetry_TypeDef before, after;
        bool valid = MotorOuterLoop_GetTelemetry(&before);
        injection = 0;
        if (action == 1) {
            tick();
            assert(live.iqRef == old_iq);
            assert(valid == MotorOuterLoop_GetTelemetry(&after));
            if (valid) assert(memcmp(&before, &after, sizeof(before)) == 0);
        } else if (action == 2) {
            live.ModeNow = Motor_Disable;
            live.iqRef = 0;
            tick();
        } else if (action == 3) {
            live.ErrorNow = Over_Current;
            live.iqRef = 0;
            tick();
        } else if (action == 4) {
            for (unsigned i = 0; i < 10; i++) tick();
            assert(live.ErrorNow == ControlOverrun_Error);
        } else if (action == 5) {
            MotorOuterLoop_RequestReset();
            live.iqRef = 0;
        }
    }
}
static void setup(ModeNow_TypeDef mode) {
    memset(&outer, 0, sizeof(outer));
    memset(&live, 0, sizeof(live));
    memset(&live_pi, 0, sizeof(live_pi));
    enc.position = .3f; enc.speed = 0;
    schedules = injection = 0;
    position_reads = speed_reads = continuous_speed_reads = 0;
    PositionCascade_Reset();
    live.ModeNow = mode; live.axis_profile_valid = true;
    live.pos_error_window = .001f; live.posAcc = .785398f; live.posDec = .523599f;
    live.pos_maxspeed = .6f; live.speed_limit = .8f;
    live.cascade_pos_Kp = 8; live.cascade_pos_Kd = 2;
    live.speed_Kp = .5f; live.speed_Ki = 2; live.current_limit = 4;
    live.position_speed_Kp = .5f; live.position_speed_Ki = 2;
    live.friction_attack_slew_a_per_s = 2; live.friction_release_slew_a_per_s = 15;
    live.posRef = enc.position; live.speedRef = .2f;
    live.speedAcc = .8f; live.speedDec = .5f;
}
static void run_step(void) {
    tick();
    if (outer.status == OUTER_QUEUED) MotorOuterLoop_Service();
}
static void run_ticks(unsigned count) {
    for (unsigned i = 0; i < count; ++i) run_step();
    assert(live.ErrorNow == No_Error);
}
/* 退役参数仍可从旧Flash读取；它们不能重启被动停靠或改变主动保持。 */
static void test_retired_support_metadata(void) {
    static float expected_iq[4000], expected_position[4000], expected_speed[4000];
    const char *axes[] = {"roll", "pitch"};
    for (unsigned axis = 0; axis < 2; ++axis) {
        for (unsigned pass = 0; pass < 2; ++pass) {
            setup(Position_Mode);
            enc.position = live.posRef = 0;
            assert(MotorAxisProfile_Create(&live.axis_profile, axes[axis], -.2f, .2f, .6f));
            live.load_profile_flags = pass == 0 ? 0 : MOTOR_LOAD_DAMPING_RING;
            live.position_support_limit_a = pass == 0 ? 0 : .5f;
            live.position_support_adjust_a_per_s = pass == 0 ? 0 : .5f;
            live.position_support_release_a_per_s = pass == 0 ? 0 : 2.0f;
            live.position_accel_ff_gain_a_per_rad_s2 = pass == 0 ? 0 : 1.034f;
            run_ticks(60);
            const PositionCascadeConfig_TypeDef *config = PositionCascade_GetConfiguration();
            assert(config != NULL && !config->passive_hold_enabled);
            assert(config->support_current_limit_a == 0);
            assert(config->support_adjust_rate_a_per_s == 0);
            assert(config->support_release_rate_a_per_s == 0);
            assert(config->acceleration_feedforward_gain == 0);
            assert(fabsf(config->hold_enter_position - .18f * .01745329252f) < .00000001f);
            assert(fabsf(config->hold_exit_position - .26f * .01745329252f) < .00000001f);
            for (unsigned tick_index = 0; tick_index < 4000; ++tick_index) {
                PositionCascadeTelemetry_TypeDef telemetry;
                enc.position = tick_index < 1000 ? 0 : .006f;
                if (tick_index == 2000 && pass != 0) {
                    live.position_support_limit_a = NAN;
                    live.position_support_adjust_a_per_s = NAN;
                    live.position_support_release_a_per_s = NAN;
                    live.position_accel_ff_gain_a_per_rad_s2 = NAN;
                }
                run_step();
                assert(live.ErrorNow == No_Error);
                assert(MotorOuterLoop_GetTelemetry(&telemetry));
                assert(!telemetry.passive_hold && telemetry.support_current_a == 0);
                assert(!telemetry.support_adjusting && !telemetry.support_limited);
                assert(!telemetry.support_refining && telemetry.passive_confirm_ticks == 0);
                assert(telemetry.qualification_position_rad == 0);
                if (pass == 0) {
                    expected_iq[tick_index] = live.iqRef;
                    expected_position[tick_index] = live.posShadow;
                    expected_speed[tick_index] = live.speedShadow;
                } else {
                    assert(live.iqRef == expected_iq[tick_index]);
                    assert(live.posShadow == expected_position[tick_index]);
                    assert(live.speedShadow == expected_speed[tick_index]);
                }
            }
            assert(live.iqRef < -.001f);
        }
    }
    /* 轮轴位置模式准入仍拒绝；速度模式不消费退役位置参数。 */
    setup(Speed_Mode);
    assert(MotorAxisProfile_CreateJoint(&live.axis_profile, MOTOR_JOINT_WHEEL_RIGHT,
        0U, 0.0f, 0.0f, 0.0f));
    live.position_support_limit_a = NAN;
    live.position_support_adjust_a_per_s = NAN;
    live.position_support_release_a_per_s = NAN;
    run_ticks(30);
    assert(PositionCascade_GetConfiguration() == NULL);
    live.ModeNow = Position_Mode;
    tick();
    assert(live.ErrorNow == MotorParam_Error && live.iqRef == 0);
    puts("PASS: retired support metadata ignored, precise active HOLD retained, wheel admission preserved");
}
/** @brief 验证生产getter的完整快照、有限值、过期、回绕和被快环抢占后的重读。 */
static void test_motion_snapshot(void) {
    float values[3] = {9,9,9}; uint16_t sequence = 99;
    memset(&outer,0,sizeof(outer));
    assert(!MotorOuterLoop_GetMotionTelemetry(values,&sequence));
    assert(sequence==99 && values[0]==9);
    outer.telemetry_valid=true;outer.telemetry_sequence=2;motion_test_now_ms=5;
    outer.published_telemetry.position_reference=1;
    outer.published_telemetry.trajectory_speed_reference=2;
    outer.published_telemetry.speed_command=3;
    assert(MotorOuterLoop_GetMotionTelemetry(values,&sequence));
    assert(sequence==1 && values[0]==1 && values[1]==2 && values[2]==3);
    motion_read_injection=true;
    assert(MotorOuterLoop_GetMotionTelemetry(values,&sequence));
    assert(sequence==2 && values[0]==4 && values[1]==5 && values[2]==6);
    outer.telemetry_sequence=5;
    assert(!MotorOuterLoop_GetMotionTelemetry(values,&sequence));
    outer.telemetry_sequence=6;motion_test_now_ms=40;
    assert(!MotorOuterLoop_GetMotionTelemetry(values,&sequence));
    motion_test_now_ms=0;outer.telemetry_sampled_ms=UINT32_MAX-5;
    assert(MotorOuterLoop_GetMotionTelemetry(values,&sequence));
    MotorOuterLoop_RequestReset();
    assert(!MotorOuterLoop_GetMotionTelemetry(values,&sequence));
    motion_test_now_ms=0;
}

int main(void) {
    test_motion_snapshot();
    /* 非活动模式仍执行状态清理，但不能重复获取本模式不消费的编码器量。 */
    const ModeNow_TypeDef inactive_modes[]={Motor_Disable,Current_Mode,Sensorless_Speed_Mode,
        Calib_EncoderObserver,Save_Param,Clear_Error};
    for(unsigned i=0;i<sizeof(inactive_modes)/sizeof(inactive_modes[0]);++i) {
        setup(inactive_modes[i]); enc.position=enc.speed=NAN;
        for(unsigned tick_index=0;tick_index<30;++tick_index) tick();
        assert(position_reads==0 && speed_reads==0 && continuous_speed_reads==0);
        assert(live.ErrorNow==No_Error && schedules==0);
    }
    setup(Speed_Mode); tick();
    assert(position_reads==1 && speed_reads==1 && continuous_speed_reads==0);
    setup(Position_Mode); tick();
    assert(position_reads==1 && speed_reads==0 && continuous_speed_reads==1);
    for (unsigned mode = Speed_Mode; mode <= Position_Mode; mode++) {
        setup((ModeNow_TypeDef)mode);
        for (unsigned i = 0; i < 200000; i++) {
            if (i % 10000 == 0) live.posRef = .3f + (i % 20000 ? .05f : -.05f);
            run_step();
            assert(live.ErrorNow == No_Error);
            assert(isfinite(live.iqRef) && fabsf(live.iqRef) <= live.current_limit);
        }
        assert(schedules == 20000 && outer.completed == 20000);
        assert(outer.maximum_age == 1 && outer.deadline_misses == 0);
        assert(outer.status == OUTER_IDLE);
        /* Inject a preemption at the completed calculation before publication. */
        for (unsigned action = 1; action <= 5; action++) {
            setup((ModeNow_TypeDef)mode);
            tick(); assert(outer.status == OUTER_QUEUED);
            injection = action; MotorOuterLoop_Service();
            assert(injection == 0 && outer.status == OUTER_DONE);
            tick(); assert(outer.status == OUTER_IDLE);
            if (action >= 2) assert(live.iqRef == 0 && !outer.ready);
            if (action == 2 || action == 3 || action == 5) assert(outer.discarded == 1);
            if (action == 4) assert(outer.deadline_misses == 1);
        }
        /* A queued job may outlive STOP; no output may be resurrected. */
        setup((ModeNow_TypeDef)mode); tick();
        live.ModeNow = Motor_Disable; live.iqRef = 0; tick();
        MotorOuterLoop_Service(); tick();
        assert(live.iqRef == 0 && outer.discarded == 1 && !outer.ready);
        /* Validate invalid commands immediately, even on a non-release tick. */
        setup((ModeNow_TypeDef)mode); run_step(); run_step();
        if (mode == Position_Mode) live.posRef = NAN; else live.speedRef = NAN;
        tick(); assert(live.ErrorNow == MotorParam_Error && live.iqRef == 0);
    }
    /* No stale validation cache may conceal a changed live tuning field. */
    {
        const size_t fields[] = {
            offsetof(MotorControl_TypeDef, current_limit), offsetof(MotorControl_TypeDef, position_speed_Kp),
            offsetof(MotorControl_TypeDef, position_speed_Ki), offsetof(MotorControl_TypeDef, pos_error_window),
            offsetof(MotorControl_TypeDef, friction_attack_slew_a_per_s), offsetof(MotorControl_TypeDef, friction_release_slew_a_per_s),
            offsetof(MotorControl_TypeDef, posAcc), offsetof(MotorControl_TypeDef, posDec),
            offsetof(MotorControl_TypeDef, pos_maxspeed), offsetof(MotorControl_TypeDef, speed_limit),
            offsetof(MotorControl_TypeDef, cascade_pos_Kp), offsetof(MotorControl_TypeDef, cascade_pos_Kd),
            offsetof(MotorControl_TypeDef, friction_coulomb_pos_a), offsetof(MotorControl_TypeDef, friction_coulomb_neg_a),
            offsetof(MotorControl_TypeDef, friction_viscous_pos_a_per_rad_s), offsetof(MotorControl_TypeDef, friction_viscous_neg_a_per_rad_s)
        };
        for (unsigned i = 0; i < sizeof(fields)/sizeof(fields[0]); i++) {
            float invalid = NAN;
            setup(Position_Mode); live.friction_model_valid = true;
            run_step(); run_step();
            assert(outer.ready && MotorOuterLoop_SameTuning(&live));
            memcpy((char *)&live + fields[i], &invalid, sizeof(invalid));
            tick(); assert(live.ErrorNow == MotorParam_Error && live.iqRef == 0);
        }
    }
    /* 非活动模式的 PI 不参与本模式验证或计算，两组不共享控制器状态。 */
    setup(Position_Mode);
    live.speed_Kp = NAN; live.speed_Ki = NAN;
    for (unsigned i = 0; i < 30; ++i) run_step();
    assert(live.ErrorNow == No_Error);
    assert(PositionCascade_GetConfiguration()->speed_kp == .5f);
    assert(PositionCascade_GetConfiguration()->speed_ki == 2);
    live.position_speed_Kp = .3f; live.position_speed_Ki = .7f;
    for (unsigned i = 0; i < 30; ++i) run_step();
    assert(live.ErrorNow == No_Error);
    assert(PositionCascade_GetConfiguration()->speed_kp == .3f);
    assert(PositionCascade_GetConfiguration()->speed_ki == .7f);
    setup(Speed_Mode);
    live.position_speed_Kp = NAN; live.position_speed_Ki = NAN;
    for (unsigned i = 0; i < 30; ++i) run_step();
    assert(live.ErrorNow == No_Error && live_pi.Kp == .5f && live_pi.Ki == 2);
    live.ModeNow = Position_Mode;
    tick();
    assert(live.ErrorNow == MotorParam_Error && live.iqRef == 0);
    /* PI/ramp arithmetic must match the shared sequential implementation exactly. */
    setup(Speed_Mode);
    MotorControl_TypeDef expected = live;
    PI_Controller_TypeDef expected_pi = live_pi;
    for (unsigned step = 0; step < 2000; step++) {
        if (step % 70 == 0) live.speedRef = expected.speedRef = -live.speedRef;
        SpeedMode_UpdateControl(&expected, &expected_pi, enc.speed);
        for (unsigned i = 0; i < 10; i++) run_step();
        if (live.speedShadow != expected.speedShadow || live.iqRef != expected.iqRef) {
            fprintf(stderr, "step %u shadow %.9g/%.9g iq %.9g/%.9g status %u error %u\n",
                step, live.speedShadow, expected.speedShadow, live.iqRef, expected.iqRef,
                outer.status, live.ErrorNow);
            assert(0);
        }
        assert(memcmp(&live_pi, &expected_pi, sizeof(live_pi)) == 0);
    }
    /* 前馈状态由慢环计算、快环提交；停机/故障不能提交过期结果。 */
    setup(Speed_Mode);
    SpeedFeedforward_Default(&live.speed_ff_config);live.speed_ff_config.enabled=true;
    live.speedRef=20;enc.speed=0;
    for(unsigned i=0;i<10000;i++) run_step();
    assert(live.ErrorNow==No_Error && live.speed_ff_state.initialized);
    assert(live.speedShadow==live.speed_ff_state.speed_rad_s && live.speedShadow>1);
    assert(fabsf(live.speed_ff_state.current_a)<=2 && fabsf(live.iqRef)<=live.current_limit);
    live.ModeNow=Motor_Disable;tick();
    assert(!live.speed_ff_state.initialized && live.speed_ff_state.current_a==0);
    setup(Speed_Mode);
    SpeedFeedforward_Default(&live.speed_ff_config);live.speed_ff_config.enabled=true;
    live.speedRef=20;enc.speed=0;
    for(unsigned i=0;i<100;i++) run_step();
    live.ErrorNow=Over_Current;tick();
    assert(!live.speed_ff_state.initialized && live.speed_ff_state.current_a==0);
    setup(Speed_Mode);
    SpeedFeedforward_Default(&live.speed_ff_config);live.speed_ff_config.enabled=true;
    live.speed_ff_config.gain_a_per_rad_s2=NAN;tick();
    assert(live.ErrorNow==MotorParam_Error && live.iqRef==0);
    setup(Position_Mode);
    live.speed_ff_config.enabled=true;live.speed_ff_config.gain_a_per_rad_s2=NAN;
    for(unsigned i=0;i<30;i++) run_step();
    assert(live.ErrorNow==No_Error);
    /* 实际反馈越界仍可向合法目标运行；缓存不能掩盖新目标越界。 */
    for (int sign = -1; sign <= 1; sign += 2) {
        setup(Position_Mode);
        assert(MotorAxisProfile_Create(&live.axis_profile, "roll", -.2f, .2f, .6f));
        enc.position = sign * .25f;
        live.posRef = sign * .1f;
        for (unsigned i = 0; i < 200; ++i) run_step();
        assert(live.ErrorNow == No_Error && outer.ready);
        assert(enc.position == sign * .25f && sign * live.posShadow > .2f);
        assert(sign * live.iqRef < 0);
        enc.position = sign * .35f;
        for (unsigned i = 0; i < 20; ++i) run_step();
        assert(live.ErrorNow == No_Error);
        live.posRef = sign * .201f;
        tick();
        assert(live.ErrorNow == MotorParam_Error && live.iqRef == 0);
    }
    puts("PASS: legacy cadence/publication/PI and optional FF state publication, STOP/fault isolation");
    puts("PASS: real feedback outside travel retained; only out-of-range target faults");
    test_retired_support_metadata();
}
"""
    )
    (out / "fixture.c").write_text(fixture, encoding="utf-8")
    sources = [
        out / "fixture.c",
        ROOT / "firmware/services/parameters/motor_axis_profile.c",
        ROOT / "firmware/motor/motor_load_profile.c",
        ROOT / "firmware/motor/speed_feedforward.c",
        ROOT / "firmware/motor/position/position_cascade.c",
        ROOT / "firmware/motor/trajectory/position_smooth_trajectory.c",
        ROOT / "firmware/motor/foc/foc_pid.c",
    ]
    cmd = [
        args.cc,
        "cc",
        "-std=c99",
        "-O2",
        "-ffp-contract=off",
        "-Wall",
        "-Wextra",
        "-Werror",
        "-I" + str(out),
        "-I" + str(ROOT / "firmware/motor/foc"),
        "-I" + str(ROOT / "firmware/common"),
        "-I" + str(ROOT / "firmware/platform/stm32g4/bsp"),
        "-I" + str(ROOT),
    ]
    cmd += NATIVE_INCLUDE_FLAGS
    cmd += [str(x) for x in sources] + ["-o", str(out / "outer_runtime.exe")]
    subprocess.run(cmd, check=True, cwd=ROOT)
    subprocess.run([str(out / "outer_runtime.exe")], check=True, cwd=ROOT)


if __name__ == "__main__":
    main()
