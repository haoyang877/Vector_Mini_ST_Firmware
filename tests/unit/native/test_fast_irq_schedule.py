"""离线验证实际快环交接错峰和 mode 13 分拍启用；不访问硬件。"""

import argparse
import json
from pathlib import Path
import subprocess

from run_position_servo_tests import NATIVE_INCLUDE_FLAGS, ROOT, function_source


def fixture(source: str) -> str:
    """提取生产调度函数，热采样和窗口查询链接真实实现，功率接口仅记录调用。"""
    return r'''
#undef NDEBUG
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "foc_run.h"
#include "foc_calibration.h"
#include "foc_phase_resistance.h"
#include "motor_thermal_control.h"
#include "hw_conf.h"
#undef SERVO_HIL_ENABLE
#define SERVO_HIL_ENABLE 0
#define FAST_PROFILE_BEGIN(x) ((void)0)
#define FAST_PROFILE_END(x) ((void)0)

MotorControl_TypeDef MotorControl;
Encoder_TypeDef OnBoard_Encoder;
FOC_TypeDef FOC;
Fluxobserver_TypeDef Fluxobserver;
SensorlessStartup_TypeDef SensorlessStartup;
PI_Controller_TypeDef PI_Speed;
ModeNow_TypeDef ModeLast;
ErrorNow_TypeDef ErrorLast;
CalibStep_TyepeDef CalibStep;
static const MotorCalibrationProfile test_profile={0};
static unsigned fast_ticks, starts, stops, task_calls, guard_calls, phase_queries, ready_queries;
static unsigned clears, other_calibration_calls;
static bool pwm_enabled, guard_allowed=true, load_valid=true;
static bool sampled, encoded, guarded, task_seen, ready_read;
enum { TASK_NOTHING, TASK_PREPARE, TASK_FAULT, TASK_STOP };
static unsigned task_action;
void motor_hw_outer_barrier(void) {}
static bool motor_hw_phase_outputs_disabled(void) { return !pwm_enabled; }
static uint32_t time_hw_now_ms(void) { return fast_ticks/20U; }
static void Set_ErrorNow(ErrorNow_TypeDef error) { MotorControl.ErrorNow=error; }
static void Set_ModeNow(ModeNow_TypeDef mode) { MotorControl.ModeNow=mode; }
static void start_pwm(void) {
    assert(guarded && MotorControl.ErrorNow==No_Error && ModeLast==Motor_Disable);
    assert(MotorControl.ModeNow!=Motor_Disable); ++starts; pwm_enabled=true;
}
static void stop_pwm(void) { ++stops; pwm_enabled=false; }
static void tracked_sample(float a,float b,float c,bool off,uint32_t now) {
    assert(!sampled); sampled=true; MotorThermalControl_Sample(a,b,c,off,now);
}
static bool tracked_guard(uint32_t mode) {
    assert(sampled && encoded && !task_seen); guarded=true; ++guard_calls;
    return guard_allowed && MotorThermalControl_Guard(mode);
}
static uint32_t tracked_remaining(void) {
    assert(sampled && encoded && guarded && !task_seen); ++phase_queries;
    return MotorThermalControl_SamplesUntilUpdate();
}
static void complete_encoder(MotorControl_TypeDef *m,Encoder_TypeDef *e,uint8_t token) {
    (void)m; assert(sampled && token==2U && !encoded); encoded=true;
    e->velocity_divider=(uint8_t)((e->velocity_divider+1U)%SPEED_LOOP_DIVIDER);
}
static bool power_ready(void) {
    assert(guarded);
    if(!task_seen) ready_read=true;
    ++ready_queries; return CalibStep==CS_OBS_START_PWM;
}
static void calibration_task(void) {
    assert(guarded && ready_read); task_seen=true; ++task_calls;
    if(task_action==TASK_PREPARE) CalibStep=CS_OBS_START_PWM;
    else if(task_action==TASK_FAULT) Set_ErrorNow(Encoder_Error);
    else if(task_action==TASK_STOP) Set_ModeNow(Motor_Disable);
    else if(CalibStep==CS_OBS_START_PWM) CalibStep=CS_OBS_ALIGN_ORIGIN;
}
static void other_calibration_task(ModeNow_TypeDef mode) {
    assert(guarded && !task_seen && MotorControl.ModeNow==mode);
    /* 新标定的入口不得继承 mode 13 尚未启用 PWM 的准备阶段。 */
    assert(CalibStep==CS_NULL); task_seen=true; ++other_calibration_calls;
    if(mode==Calib_EncoderOffset) CalibStep=CS_ENC_OFFSET_ALIGN;
}
static void clear_running_data(void) {
    /* 对应 task_reset_runtime -> EncoderCalibration_Reset 的阶段清理契约。 */
    ++clears; CalibStep=CS_NULL;
}
static void sensorless_task(void) { assert(guarded); task_seen=true; ++task_calls; }
#define MotorThermalControl_Sample tracked_sample
#define MotorThermalControl_Guard tracked_guard
#define MotorThermalControl_SamplesUntilUpdate tracked_remaining
#define Encoder_BeginSample() true
#define Encoder_BeginReadback(started) ((started)?2U:0U)
#define Encoder_CompleteSample complete_encoder
#define Vbus_Update(...) ((void)0)
#define Current_Cal(...) true
#define MotorControl_ApplyLoadProfile() ((void)0)
#define Fluxobserver_Update(...) ((void)0)
#define MotorLoadProfile_IsValid(flags) (load_valid)
#define MotorLoadProfile_Calibration(flags) (&test_profile)
#define MotorOuterLoop_FastTick(...) ((void)0)
#define MotorOuterLoop_IsReady() true
#define PhaseResistanceMode_Cancel(...) ((void)0)
#define Set_A_Duty(...) ((void)0)
#define Set_B_Duty(...) ((void)0)
#define Set_C_Duty(...) ((void)0)
#define FOC_Current(...) ((void)0)
#define Task_Current_Mode(...) ((void)0)
#define Task_Sensorless_Speed_Mode(...) ((void)profile,sensorless_task())
#define FocFrictionIdentification_Task(...) ((void)0)
#define Task_Calib_R_L_Flux(...) ((void)0)
#define PhaseResistanceMode_Run(...) PHASE_RESISTANCE_MODE_RUNNING
#define Task_Calib_EncoderOffset(...) other_calibration_task(Calib_EncoderOffset)
#define EncoderObserverCalibration_IsPowerStartReady power_ready
#define Task_Calib_EncoderObserver(...) calibration_task()
#define Task_Calib_EleAngelOffset(...) other_calibration_task(Calib_EleAngelOffset)
#define Task_Calib_CurrentOffset(...) ((void)0)
#define Task_Voltage_Mode(...) ((void)0)
#define Task_Vq_Mode(...) ((void)0)
#define Task_SetMechanicalZero(...) ((void)0)
#define Param_Return_Default() ((void)0)
#define Task_TryClearError() ((void)0)
#define LED_SetState(...) ((void)0)
#define task_reset_runtime clear_running_data
#define Stop_PWM_Generate stop_pwm
#define Start_PWM_Generate start_pwm
#define Start_Damping_Brake start_pwm
#define Detect_Mode_Error_Change() ((void)0)
#define MotorStatus_Sampling() ((void)0)
#define RTT_Sampling() ((void)0)
''' + "\n".join(function_source(source, name) for name in (
        "MotorControl_IsConfigurationValid", "Encoder_FeedbackRequired", "Task_ApplyThermalGuard",
        "FOC_SelectSensorlessSpeedPhase", "FOC20kHzIRQHandler",
    )) + r'''
#undef MotorThermalControl_SamplesUntilUpdate
static void setup(void) {
    memset(&MotorControl,0,sizeof(MotorControl)); memset(&FOC,0,sizeof(FOC));
    memset(&OnBoard_Encoder,0,sizeof(OnBoard_Encoder)); memset(&SensorlessStartup,0,sizeof(SensorlessStartup));
    MotorControl.axis_profile_valid=true; MotorControl.load_profile_valid=true;
    MotorControl.current_limit=3.0f;
    ModeLast=Motor_Disable; ErrorLast=No_Error; pwm_enabled=false;
    CalibStep=CS_NULL; guard_allowed=true; load_valid=true; task_action=TASK_NOTHING;
    starts=stops=task_calls=phase_queries=ready_queries=guard_calls=0U;
    clears=other_calibration_calls=0U;
}
static void tick(void) {
    MotorThermalDiagnostics before,after;
    assert(MotorThermalControl_GetDiagnostics(&before));
    sampled=encoded=guarded=task_seen=ready_read=false; ++fast_ticks;
    unsigned guards_before=guard_calls;
    FOC20kHzIRQHandler();
    assert(sampled && encoded && guarded && guard_calls==guards_before+1U);
    assert(MotorThermalControl_GetDiagnostics(&after));
    /* 查询/错峰不能篡改热窗口，真实 Sample 每次只增加一个实采样。 */
    assert(after.model.sample_count==(before.model.sample_count+1U)%20U);
    assert(after.updates==before.updates+(before.model.sample_count==19U));
}
static void check_phase_grid(void) {
    unsigned combinations=0U;
    assert(SPEED_LOOP_DIVIDER==10U);
    for(unsigned thermal=1U;thermal<=20U;++thermal) {
        for(unsigned divider=0U;divider<10U;++divider) {
            uint32_t counter=FOC_SelectSensorlessSpeedPhase(thermal,divider);
            unsigned first=UINT32_MAX,last=UINT32_MAX,updates=0U;
            assert(counter<10U);
            /* t=0 为本次调用，本拍热样本/编码器分频均已处理完。 */
            for(unsigned t=0U;t<400U;++t) {
                if(++counter>=10U) {
                    counter=0U;
                    if(first==UINT32_MAX) first=t;
                    if(last!=UINT32_MAX) assert(t-last==10U);
                    assert(!(t>=thermal && (t-thermal)%20U==0U));
                    unsigned encoder_remaining=10U-divider;
                    assert(!(t>=encoder_remaining && (t-encoder_remaining)%10U==0U));
                    last=t; ++updates;
                }
            }
            assert(first>=1U && first<=9U && updates==40U); ++combinations;
        }
    }
    const uint32_t unknown[]={0U,21U,100U,UINT32_MAX};
    for(unsigned i=0;i<sizeof(unknown)/sizeof(unknown[0]);++i)
        for(unsigned d=0;d<10U;++d) assert(FOC_SelectSensorlessSpeedPhase(unknown[i],d)==0U);
    for(unsigned t=1U;t<=20U;++t) {
        assert(FOC_SelectSensorlessSpeedPhase(t,10U)==0U);
        assert(FOC_SelectSensorlessSpeedPhase(t,UINT32_MAX)==0U);
    }
    printf("PASS actual phase selector: %u combinations, first PI calls 2..10, then every 10, no thermal/encoder collision, unknown fallback\n",combinations);
}
static void check_real_dispatch(void) {
    unsigned combinations=0U;
    for(unsigned mode=0U;mode<MODE_NUM;++mode) {
        for(unsigned state=0U;state<=SENSORLESS_STARTUP_CLOSED_LOOP;++state) {
            for(unsigned age=0U;age<2U;++age) {
                setup(); MotorControl.ModeNow=(ModeNow_TypeDef)mode; ModeLast=(ModeNow_TypeDef)mode;
                SensorlessStartup.state=(SensorlessStartupState_TypeDef)state;
                SensorlessStartup.state_ticks=age; SensorlessStartup.speed_loop_ticks=1234U;
                if(mode!=Calib_EncoderOffset && mode!=Calib_EleAngelOffset) CalibStep=CS_OBS_START_PWM;
                bool expected=(mode==Calib_EncoderObserver || mode==Sensorless_Speed_Mode) &&
                    state==SENSORLESS_STARTUP_HANDOFF && age==0U;
                tick();
                assert(phase_queries==(expected?1U:0U));
                if(expected) {
                    assert(SensorlessStartup.speed_loop_ticks==FOC_SelectSensorlessSpeedPhase(
                        MotorThermalControl_SamplesUntilUpdate(),OnBoard_Encoder.velocity_divider));
                } else assert(SensorlessStartup.speed_loop_ticks==1234U);
                ++combinations;
            }
        }
    }
    setup(); MotorControl.ModeNow=Calib_EncoderObserver;
    SensorlessStartup.state=SENSORLESS_STARTUP_HANDOFF; SensorlessStartup.state_ticks=0U;
    SensorlessStartup.speed_loop_ticks=1234U; guard_allowed=false;
    tick(); assert(!phase_queries && !task_calls && !starts);
    assert(MotorControl.ModeNow==Motor_Disable && SensorlessStartup.speed_loop_ticks==1234U);
    printf("PASS actual IRQ dispatch: %u mode/state/age combinations, mode 13/16 only, post-guard/post-sample query, preserved thermal count\n",combinations);
}
static void check_mode13_power_entry(void) {
    setup(); MotorControl.ModeNow=Calib_EncoderObserver;
    for(unsigned i=0;i<3U;++i) {
        tick(); assert(!starts && !pwm_enabled && ModeLast==Motor_Disable);
    }
    task_action=TASK_PREPARE;
    tick(); assert(CalibStep==CS_OBS_START_PWM && !starts && !pwm_enabled && ModeLast==Motor_Disable);
    task_action=TASK_NOTHING;
    tick(); assert(starts==1U && pwm_enabled && ModeLast==Calib_EncoderObserver);
    tick(); assert(starts==1U && pwm_enabled && ModeLast==Calib_EncoderObserver);
    MotorControl.ModeNow=Motor_Disable; tick();
    assert(stops==1U && !pwm_enabled && ModeLast==Motor_Disable);
    for(unsigned action=TASK_FAULT;action<=TASK_STOP;++action) {
        setup(); MotorControl.ModeNow=Calib_EncoderObserver; task_action=TASK_PREPARE;
        tick(); assert(CalibStep==CS_OBS_START_PWM && !starts && ModeLast==Motor_Disable);
        task_action=action; tick();
        assert(!starts && !pwm_enabled && MotorControl.ModeNow==Motor_Disable && ModeLast==Motor_Disable);
    }
    setup(); MotorControl.ModeNow=Calib_EncoderObserver; CalibStep=CS_OBS_START_PWM;
    MotorControl.ErrorNow=Encoder_Error; tick();
    assert(!starts && !pwm_enabled && ModeLast==Motor_Disable);
    setup(); MotorControl.ModeNow=Calib_EncoderObserver; CalibStep=CS_OBS_START_PWM; guard_allowed=false;
    tick(); assert(!starts && !task_calls && !pwm_enabled && ModeLast==Motor_Disable);
    setup(); MotorControl.ModeNow=Calib_EncoderObserver; CalibStep=CS_OBS_START_PWM;
    MotorControl.axis_profile_valid=false; tick();
    assert(!starts && !task_calls && !pwm_enabled && ModeLast==Motor_Disable);
    setup(); pwm_enabled=true; MotorControl.ModeNow=Motor_Disable;
    tick(); assert(stops==1U && !pwm_enabled && !starts);
    puts("PASS actual mode13 startup: prepare/enable separate ticks, ready read before task, Disabled until enable, same-tick fault/STOP/guard prevent enable");
}
static void check_prepared_stop_restart(void) {
    const ModeNow_TypeDef restart_modes[]={Calib_EncoderObserver,Calib_EncoderOffset,Calib_EleAngelOffset};
    for(unsigned i=0U;i<sizeof(restart_modes)/sizeof(restart_modes[0]);++i) {
        setup(); MotorControl.ModeNow=Calib_EncoderObserver; task_action=TASK_PREPARE;
        tick();
        assert(CalibStep==CS_OBS_START_PWM && ModeLast==Motor_Disable && !pwm_enabled && !starts);
        /* 准备之后尚无运行边沿，STOP 仍须清理阶段并关相一次。 */
        MotorControl.ModeNow=Motor_Disable; task_action=TASK_NOTHING; tick();
        assert(clears==1U && stops==1U && !starts && !pwm_enabled && CalibStep==CS_NULL);
        assert(ModeLast==Motor_Disable && MotorControl.ModeNow==Motor_Disable);
        tick();
        assert(clears==1U && stops==1U && !starts && CalibStep==CS_NULL);
        MotorControl.ModeNow=restart_modes[i];
        if(restart_modes[i]==Calib_EncoderObserver) {
            task_action=TASK_PREPARE; tick();
            assert(!starts && !pwm_enabled && ModeLast==Motor_Disable && CalibStep==CS_OBS_START_PWM);
            task_action=TASK_NOTHING; tick();
            assert(starts==1U && pwm_enabled && ModeLast==Calib_EncoderObserver && CalibStep==CS_OBS_ALIGN_ORIGIN);
            assert(!other_calibration_calls);
        } else {
            tick();
            assert(starts==1U && pwm_enabled && ModeLast==restart_modes[i] && other_calibration_calls==1U);
            assert(CalibStep==(restart_modes[i]==Calib_EncoderOffset?CS_ENC_OFFSET_ALIGN:CS_NULL));
        }
        assert(clears==1U && stops==1U);
    }
    puts("PASS actual prepared STOP: clears pending mode13 stage with Disabled/PWM-off, clean restart into modes 13/5/15");
}
int main(void) {
    MotorThermalControl_Init(0U);
    check_phase_grid(); check_real_dispatch(); check_mode13_power_entry(); check_prepared_stop_restart();
    return 0;
}
'''


def main() -> None:
    """编译和运行实际函数的本机夹具，保存无硬件证据。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", required=True)
    parser.add_argument("--out", type=Path, default=ROOT / "outputs/fast_irq_schedule")
    args = parser.parse_args()
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    source = (ROOT / "firmware/app/foc_task.c").read_text(encoding="utf-8")
    irq = function_source(source, "FOC20kHzIRQHandler")
    if not (irq.index("MotorThermalControl_Sample(") < irq.index("Encoder_CompleteSample(") <
            irq.index("Task_ApplyThermalGuard();") < irq.index("FOC_SelectSensorlessSpeedPhase(") <
            irq.index("switch(MotorControl.ModeNow)")):
        raise RuntimeError("交接错峰必须在本拍采样/编码器/热准入后、模式任务前")
    (out / "main.h").write_text("#include <stdint.h>\n#include <stddef.h>\n", encoding="utf-8")
    c = out / "fast_irq_schedule.c"
    c.write_text(fixture(source), encoding="utf-8")
    exe = out / "fast_irq_schedule.exe"
    compiler = [args.cc] + (["cc"] if Path(args.cc).stem == "zig" else [])
    command = compiler + ["-std=c99", "-O2", "-Wall", "-Wextra", "-Werror", "-I", str(out)]
    command += NATIVE_INCLUDE_FLAGS + [str(c), str(ROOT / "firmware/motor/protection/motor_thermal.c"),
        str(ROOT / "firmware/app/motor_thermal_control.c"), "-lm", "-o", str(exe)]
    logs = []
    for step in (command, [str(exe)]):
        result = subprocess.run(step, cwd=ROOT, capture_output=True, text=True)
        logs.append(result.stdout + result.stderr)
        (out / "test.log").write_text("".join(logs), encoding="utf-8")
        print(logs[-1], end="", flush=True)
        result.check_returncode()
    (out / "summary.json").write_text(json.dumps({"status": "passed", "hardware_actions": False,
        "phase_combinations": 200, "irq_mode_state_combinations": 252,
        "prepared_stop_restart_modes": [13, 5, 15]}, indent=2), encoding="utf-8")


if __name__ == "__main__":
    main()
