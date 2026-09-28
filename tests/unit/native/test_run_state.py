"""运行状态机接线检查：维护结果、真实关相、健康条件与目标撤销；不访问硬件。"""

import argparse
import subprocess
from pathlib import Path

from run_position_servo_tests import NATIVE_INCLUDE_FLAGS, ROOT, function_source

FIXTURE_HEAD = r"""
#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "app_lifecycle.h"
#include "motor_control_command.h"
#include "motor_axis_profile.h"
#include "control_config.h"
#include "foc_bench_speed.h"

static uint32_t critical_depth;

typedef int ModeNow_TypeDef;
typedef int ErrorNow_TypeDef;

typedef enum
{
    MOTOR_WORK_RUNNING = 0,
    MOTOR_WORK_SWITCH_MODE,
    MOTOR_WORK_STOP,
    MOTOR_WORK_FAULT
} MotorWorkResult_TypeDef;

typedef struct
{
    MotorWorkResult_TypeDef result;
    ModeNow_TypeDef next_mode;
    ErrorNow_TypeDef error;
    bool power_off;
} MotorWorkOutcome_TypeDef;

#define Motor_Disable 0
#define Save_Param 1
#define Default_Param 2
#define Clear_Error 3
#define Set_ZeroPosition 4
#define Calib_Anticogging 5
#define Position_Mode 6
#define Speed_Mode 7
#define Current_Mode 8
#define Vq_Mode 9
#define Position_Impedance_Mode 10
#define Sensorless_Speed_Mode 11
#define Voltage_OpenLoop 12
#define Calib_PhaseResistance 13
#define Calib_Friction 14
#define Calib_EncoderObserver 15
#define Calib_EleAngelOffset 16
#define Calib_EncoderOffset 17
#define No_Error 0
#define CurrentOffset_Error 1
#define Encoder_Error 2
#define PolePairs_Error 3
#define Large_Phase_Resistance 5
#define Large_Phase_Inductance 6
#define Over_Current 7
#define Over_Voltage 8
#define Under_Voltage 9
#define High_Temprature 10
#define Encoder_NotCalibrated 11
#define MotorParam_Error 12
#define Sensorless_Error 13
#define FrictionIdentification_Error 14
#define ControlOverrun_Error 15
#define CoggingCalibration_Error 16
#define TemperatureSensor_Error 17
#define Test_Error 99

/* 阶段 D 恢复矩阵常量与外部信号桩（真实模块为全局，这里由夹具提供）。 */
#define RUN_STATE_OC_RECOVER_TICKS 2000U
#define RUN_STATE_TEMP_RECOVER_C 80.0f
#define MCU_TEMPERATURE_TIMEOUT_TICKS 200U
#define BUS_VOLTAGE_ENABLE_MIN_V 25.6f
#define BUS_VOLTAGE_ENABLE_MAX_V 33.8f
#define BUS_VOLTAGE_HARD_OVERVOLTAGE_V 35.0f
#define FOC_MAX_MODULATION (0.95f * 0.866025403f)
#define MOTOR_SENSING_OVERCURRENT_TRIP_A 18.0f

typedef struct
{
    float Ia, Ib, Ic, Vbus, Vbus_filt, temp;
} FOCStub_TypeDef;
static FOCStub_TypeDef FOC;

typedef struct
{
    uint16_t bad_frame_streak;
    uint8_t calibrated;
    float position_rad;
    float speed_rad_s;
} EncoderStub_TypeDef;
static EncoderStub_TypeDef OnBoard_Encoder;
static uint16_t Encoder_GetBadFrameStreak(const EncoderStub_TypeDef *e)
{
    return e->bad_frame_streak;
}
#define ENC_CALIB_ALL 3U
static bool Encoder_IsOnline(const EncoderStub_TypeDef *e)
{
    assert(critical_depth > 0U);
    return e->bad_frame_streak == 0U;
}
static uint8_t Encoder_GetCalibFlag(const EncoderStub_TypeDef *e) { return e->calibrated; }
static float Encoder_GetMecPos(const EncoderStub_TypeDef *e) { return e->position_rad; }
static float Encoder_GetMecVel(const EncoderStub_TypeDef *e) { return e->speed_rad_s; }

typedef struct
{
    uint32_t valid, missed_ticks;
} McuTemperatureStub_TypeDef;
static McuTemperatureStub_TypeDef McuTemperature;

static float fast_abs(float x) { return x < 0.0f ? -x : x; }

/* 适配器内部状态（真实模块为文件级 static，这里由夹具提供）。 */
static AppLifecycle lifecycle;
static bool hardware_phases_disabled;
static bool hardware_stop_fails;
static ModeNow_TypeDef operation_mode;
enum
{
    SAVE_FINISH_NONE = 0,
    SAVE_FINISH_COMMITTED,
    SAVE_FINISH_FAILED
};
static volatile uint8_t save_finish;
static uint16_t overcurrent_recover_ticks;
static MotorWorkOutcome_TypeDef pending_outcome;
static volatile bool outcome_latched;
static MotorControlCommand pending_control;
static volatile bool control_pending;
static volatile uint32_t fast_sample_epoch;
static uint32_t observed_fast_sample_epoch;
static bool fast_samples_fresh;
static bool RunState_VbusInEnableWindow(void);
static uint32_t critical_hw_enter(void) { return critical_depth++; }
static void critical_hw_exit(uint32_t state)
{
    assert(critical_depth == state + 1U);
    critical_depth = state;
}

typedef struct
{
    ModeNow_TypeDef ModeNow;
    ErrorNow_TypeDef ErrorNow;
    uint8_t axis_profile_valid;
    MotorAxisProfile axis_profile;
    bool isUseSensorless;
    float current_limit, speed_limit;
    float iqRef, speedRef, posRef, vqRef, idRef, speedShadow;
    float speedAcc, speedDec, speed_Kp, speed_Ki;
    ModeNow_TypeDef ModeNow_f;
    ErrorNow_TypeDef ErrorNow_f;
} MotorControl_TypeDef;

static MotorControl_TypeDef MotorControl;
static ModeNow_TypeDef ModeLast;
static ErrorNow_TypeDef ErrorLast;
static bool ready = true;

static void Set_ErrorNow(ErrorNow_TypeDef error) { MotorControl.ErrorNow = error; }

#define LOG_CAP 256
static int log_codes[LOG_CAP];
static int log_args[LOG_CAP];
static int log_len;

static void log_event(int code, int arg)
{
    assert(log_len < LOG_CAP);
    log_codes[log_len] = code;
    log_args[log_len] = arg;
    ++log_len;
}

void indicator_hw_set_led(bool error_channel, uint8_t blink_num)
{
    log_event(1 + (int)error_channel, (int)blink_num);
}
void Clear_RunningData(void) { log_event(3, 0); }
void power_stage_hw_stop(void) { if(!hardware_stop_fails) hardware_phases_disabled=true; log_event(4, 0); }
void power_stage_hw_start(void) { hardware_phases_disabled=false; log_event(5, 0); }
static bool power_stage_hw_phases_disabled(void) { return hardware_phases_disabled; }
void Detect_Mode_Error_Change(void) { log_event(6, 0); }
/* 就绪查询是纯查询：适配器与旧实现的调用时机不同，不计入行为对拍。 */
bool MotorOuterLoop_IsReady(void) { return ready; }
static bool ModeSwitch_Handle(ModeNow_TypeDef mode)
{
    assert(critical_depth > 0U);
    MotorControl.ModeNow = mode;
    return true;
}
bool FocRunState_RequestRemoteStop(void);
"""

DRIVER = r"""
static void ResetWorld(ModeNow_TypeDef mode_last, ModeNow_TypeDef mode_now,
    ErrorNow_TypeDef error, uint8_t valid, bool is_ready)
{
    ModeLast = mode_last;
    MotorControl.ModeNow = mode_now;
    MotorControl.ErrorNow = error;
    MotorControl.axis_profile_valid = valid;
    MotorControl.ModeNow_f = 0;
    MotorControl.ErrorNow_f = 0;
    ready = is_ready;
    hardware_phases_disabled = true;
    hardware_stop_fails = false;
    OnBoard_Encoder.bad_frame_streak = 0U;
    OnBoard_Encoder.calibrated = ENC_CALIB_ALL;
    MotorControl.isUseSensorless = false;
    FOC = (FOCStub_TypeDef){.Vbus=30.0f, .Vbus_filt=30.0f, .temp=25.0f};
    McuTemperature.valid = 1U;
    McuTemperature.missed_ticks = 0U;
    FocRunState_Init();
    log_len = 0;
}

static void healthy_tick(void)
{
    FocRunState_PostOutcome((MotorWorkOutcome_TypeDef){MOTOR_WORK_RUNNING, Motor_Disable,
                                                     No_Error, false});
    FocRunState_Tick();
}

static void prepare_remote_mode(ModeNow_TypeDef mode)
{
    ResetWorld(Motor_Disable, mode, No_Error, 1, true);
    MotorControl.isUseSensorless = false;
    MotorControl.current_limit = 2.0f;
    MotorControl.speed_limit = 4.0f;
    MotorControl.axis_profile = (MotorAxisProfile){0};
    MotorControl.iqRef = 0.0f;
    MotorControl.speedRef = 0.0f;
    MotorControl.posRef = 0.0f;
    MotorControl.vqRef = 0.0f;
    FOC.Vbus = FOC.Vbus_filt = 30.0f;
    OnBoard_Encoder.bad_frame_streak = 0U;
    OnBoard_Encoder.calibrated = ENC_CALIB_ALL;
    OnBoard_Encoder.position_rad = 0.0f;
    healthy_tick();
    healthy_tick();
    assert(lifecycle.snapshot.state == APP_RUNNING && !hardware_phases_disabled);
}

static void test_remote_targets(void)
{
    MotorControlCommand command = {.mode=MOTOR_CONTROL_CURRENT, .iq_a=1.25f};
    prepare_remote_mode(Current_Mode);
    assert(FocRunState_SubmitRemoteControl(NULL) == MOTOR_CONTROL_INVALID);
    assert(FocRunState_SubmitRemoteControl(&command) == MOTOR_CONTROL_ACCEPTED);
    command.iq_a = 1.5f;
    assert(FocRunState_SubmitRemoteControl(&command) == MOTOR_CONTROL_BUSY);
    assert(MotorControl.iqRef == 0.0f);
    healthy_tick();
    assert(MotorControl.iqRef == 1.25f && !control_pending);
    command.iq_a = 2.1f;
    assert(FocRunState_SubmitRemoteControl(&command) == MOTOR_CONTROL_OUT_OF_RANGE);
    command.iq_a = NAN;
    assert(FocRunState_SubmitRemoteControl(&command) == MOTOR_CONTROL_INVALID);
    command.iq_a = 0.5f;
    command.position_rad = 0.01f;
    assert(FocRunState_SubmitRemoteControl(&command) == MOTOR_CONTROL_INVALID);
    command.position_rad = 0.0f;
    assert(FocRunState_SubmitRemoteControl(&command) == MOTOR_CONTROL_ACCEPTED);
    assert(FocRunState_RequestRemoteStop());
    healthy_tick();
    assert(!control_pending && hardware_phases_disabled && MotorControl.iqRef == 1.25f);
    assert(FocRunState_SubmitRemoteControl(&command) == MOTOR_CONTROL_DENIED);

    prepare_remote_mode(Current_Mode);
    assert(FocRunState_SubmitRemoteControl(&command) == MOTOR_CONTROL_ACCEPTED);
    MotorControl.current_limit = 0.1f;
    healthy_tick();
    assert(!control_pending && MotorControl.iqRef == 0.0f);
    MotorControl.current_limit = 2.0f;
    healthy_tick();
    assert(MotorControl.iqRef == 0.0f);
    assert(FocRunState_SubmitRemoteControl(&command) == MOTOR_CONTROL_ACCEPTED);
    MotorControl.ErrorNow = Over_Current;
    healthy_tick();
    assert(!control_pending && MotorControl.iqRef == 0.0f && hardware_phases_disabled);
    assert(FocRunState_SubmitRemoteControl(&command) == MOTOR_CONTROL_FAULT);

    prepare_remote_mode(Speed_Mode);
    command = (MotorControlCommand){.mode=MOTOR_CONTROL_SPEED, .speed_rad_s=-3.0f};
    assert(FocRunState_SubmitRemoteControl(&command) == MOTOR_CONTROL_ACCEPTED);
    healthy_tick();
    assert(MotorControl.speedRef == -3.0f);
    command.speed_rad_s = 4.01f;
    assert(FocRunState_SubmitRemoteControl(&command) == MOTOR_CONTROL_OUT_OF_RANGE);
    MotorControl.axis_profile.magic = MOTOR_AXIS_PROFILE_MAGIC;
    MotorControl.axis_profile.maximum_speed_rad_s = 2.0f;
    command.speed_rad_s = 3.0f;
    assert(FocRunState_SubmitRemoteControl(&command) == MOTOR_CONTROL_OUT_OF_RANGE);

    prepare_remote_mode(Position_Mode);
    MotorControl.axis_profile.magic = MOTOR_AXIS_PROFILE_MAGIC;
    MotorControl.axis_profile.minimum_position_rad = -1.0f;
    MotorControl.axis_profile.maximum_position_rad = 1.0f;
    command = (MotorControlCommand){.mode=MOTOR_CONTROL_POSITION, .position_rad=0.5f};
    assert(FocRunState_SubmitRemoteControl(&command) == MOTOR_CONTROL_ACCEPTED);
    healthy_tick();
    assert(MotorControl.posRef == 0.5f && MotorControl.ModeNow == Position_Mode);
    command.position_rad = 0.75f;
    assert(FocRunState_SubmitRemoteControl(&command) == MOTOR_CONTROL_ACCEPTED);
    OnBoard_Encoder.position_rad = 1.0f;
    healthy_tick();
    assert(!control_pending && MotorControl.posRef == 0.5f);
    OnBoard_Encoder.position_rad = 0.0f;
    command.position_rad = 1.1f;
    assert(FocRunState_SubmitRemoteControl(&command) == MOTOR_CONTROL_OUT_OF_RANGE);
    command = (MotorControlCommand){.mode=MOTOR_CONTROL_CURRENT, .iq_a=0.5f};
    assert(FocRunState_SubmitRemoteControl(&command) == MOTOR_CONTROL_DENIED);

    prepare_remote_mode(Vq_Mode);
    command = (MotorControlCommand){.mode=MOTOR_CONTROL_VQ, .vq_v=-2.0f};
    assert(FocRunState_SubmitRemoteControl(&command) == MOTOR_CONTROL_ACCEPTED);
    healthy_tick();
    assert(MotorControl.vqRef == -2.0f);
    command.vq_v = 20.0f;
    assert(FocRunState_SubmitRemoteControl(&command) == MOTOR_CONTROL_OUT_OF_RANGE);
    command.vq_v = 1.0f;
    OnBoard_Encoder.calibrated = 0U;
    assert(FocRunState_SubmitRemoteControl(&command) == MOTOR_CONTROL_DENIED);
    OnBoard_Encoder.calibrated = ENC_CALIB_ALL;
    assert(FocRunState_SubmitRemoteControl(&command) == MOTOR_CONTROL_ACCEPTED);
    FocRunState_PostOutcome((MotorWorkOutcome_TypeDef){MOTOR_WORK_STOP, Motor_Disable,
                                                     No_Error, false});
    healthy_tick();
    assert(!control_pending && hardware_phases_disabled && MotorControl.vqRef == -2.0f);
    assert(FOC_MaximumVoltage(NAN) == 0.0f && FOC_MaximumVoltage(-1.0f) == 0.0f);
    assert(critical_depth == 0U);
    printf("PASS remote targets: SI, mailbox, bounds, STOP/fault/limit races, no enable\n");
}

static void test_real_guards(void)
{
    ResetWorld(Motor_Disable, Motor_Disable, No_Error, 1, true);
    McuTemperature.valid = 0U;
    healthy_tick();
    assert(lifecycle.snapshot.state == APP_SELF_TEST && hardware_phases_disabled);
    assert(MotorControl.ErrorNow == No_Error);
    McuTemperature.valid = 1U;
    OnBoard_Encoder.calibrated = 0U;
    healthy_tick();
    assert(lifecycle.snapshot.state == APP_READY);
    assert(FocRunState_RequestRemoteStop());
    assert(!FocRunState_IsMaintenanceActive(Calib_Anticogging));

    prepare_remote_mode(Current_Mode);
    MotorControlCommand command = {.mode=MOTOR_CONTROL_CURRENT, .iq_a=0.5f};
    assert(FocRunState_SubmitRemoteControl(&command) == MOTOR_CONTROL_ACCEPTED);
    FocRunState_Tick();
    assert(lifecycle.snapshot.state == APP_FAULT && hardware_phases_disabled);
    assert(!control_pending && MotorControl.iqRef == 0.0f);
    healthy_tick();
    assert(lifecycle.snapshot.state == APP_FAULT);
    MotorControl.ModeNow = Clear_Error;
    healthy_tick();
    assert(lifecycle.snapshot.state == APP_READY && hardware_phases_disabled);

    ResetWorld(Motor_Disable, Motor_Disable, Encoder_Error, 1, true);
    OnBoard_Encoder.bad_frame_streak = 0U;
    hardware_phases_disabled = false;
    hardware_stop_fails = true;
    MotorControl.ModeNow = Clear_Error;
    healthy_tick();
    assert(MotorControl.ErrorNow == Encoder_Error);
    assert(lifecycle.snapshot.state == APP_FAULT);
    hardware_stop_fails = false;
    hardware_phases_disabled = true;
    MotorControl.ModeNow = Clear_Error;
    healthy_tick();
    assert(MotorControl.ErrorNow == No_Error && lifecycle.snapshot.state == APP_READY);

    prepare_remote_mode(Current_Mode);
    FocRunState_PostOutcome((MotorWorkOutcome_TypeDef){MOTOR_WORK_SWITCH_MODE, Current_Mode,
                                                     No_Error, false});
    assert(FocRunState_RequestRemoteStop());
    healthy_tick();
    assert(MotorControl.ModeNow == Motor_Disable && hardware_phases_disabled);
    assert(!outcome_latched && !control_pending);
    printf("PASS real guards: cold self-test, calibration not globally required, stale ADC,"
           " fault holds, real shutdown before CLEAR, STOP revokes queued mode\n");
}

static void prepare_bench(void)
{
    ResetWorld(Motor_Disable, Motor_Disable, No_Error, 1U, true);
    MotorControl.axis_profile = (MotorAxisProfile){0};
    MotorControl.current_limit = 2.0f;
    MotorControl.speed_limit = 10.0f;
    MotorControl.speedAcc = 20.0f;
    MotorControl.speedDec = 20.0f;
    MotorControl.speed_Kp = 0.05f;
    MotorControl.speed_Ki = 0.5f;
    OnBoard_Encoder.speed_rad_s = 0.0f;
    healthy_tick();
    assert(lifecycle.snapshot.state == APP_READY && hardware_phases_disabled);
    FocBenchSpeed.command = FOC_BENCH_SPEED_START;
    FocBenchSpeed.duration_ms = 1U;
    FocBenchSpeed.iq_limit_a = 1.0f;
    FocBenchSpeed.phase_limit_a = 6.0f;
    FocBenchSpeed.speed_limit_rad_s = 5.0f;
}

static void start_bench(void)
{
    FocBenchSpeed.request_seq = 2U;
    healthy_tick();
    assert(FocBenchSpeed.ack_seq == 2U);
    assert(FocBenchSpeed.ack_result == FOC_BENCH_SPEED_ACCEPTED);
    assert(lifecycle.snapshot.state == APP_STARTING && hardware_phases_disabled);
    assert(MotorControl.speedRef == 0.0f && MotorControl.iqRef == 0.0f);
    assert(MotorControl.current_limit == 1.0f && MotorControl.speed_limit == 5.0f);
    healthy_tick();
    assert(lifecycle.snapshot.state == APP_RUNNING && !hardware_phases_disabled);
}

static void test_bench_speed(void)
{
    prepare_bench();
    FocBenchSpeed.request_seq = 1U;
    healthy_tick();
    assert(FocBenchSpeed.ack_seq == 0U && !bench_speed.active);
    FocBenchSpeed.duration_ms = 0U;
    FocBenchSpeed.request_seq = 2U;
    healthy_tick();
    assert(FocBenchSpeed.ack_result == FOC_BENCH_SPEED_INVALID && hardware_phases_disabled);

    prepare_bench();
    FocBenchSpeed.iq_limit_a = NAN;
    FocBenchSpeed.request_seq = 2U;
    healthy_tick();
    assert(FocBenchSpeed.ack_result == FOC_BENCH_SPEED_INVALID && !bench_speed.active);

    prepare_bench();
    FocBenchSpeed.iq_limit_a = 3.0f;
    FocBenchSpeed.request_seq = 2U;
    healthy_tick();
    assert(FocBenchSpeed.ack_result == FOC_BENCH_SPEED_DENIED && !bench_speed.active);
    prepare_bench();
    FocBenchSpeed.speed_limit_rad_s = 11.0f;
    FocBenchSpeed.request_seq = 2U;
    healthy_tick();
    assert(FocBenchSpeed.ack_result == FOC_BENCH_SPEED_DENIED && !bench_speed.active);

    prepare_bench();
    McuTemperature.valid = 0U;
    FocBenchSpeed.request_seq = 2U;
    healthy_tick();
    assert(FocBenchSpeed.ack_result == FOC_BENCH_SPEED_DENIED && hardware_phases_disabled);
    prepare_bench();
    OnBoard_Encoder.calibrated = 0U;
    FocBenchSpeed.request_seq = 2U;
    healthy_tick();
    assert(FocBenchSpeed.ack_result == FOC_BENCH_SPEED_DENIED && hardware_phases_disabled);

    prepare_bench();
    FocBenchSpeed.request_seq = 1U;
    FocRunState_RequestRemoteStop();
    FocBenchSpeed.request_seq = 2U;
    healthy_tick();
    assert(FocBenchSpeed.ack_result == FOC_BENCH_SPEED_INTERRUPTED && !bench_speed.active);

    prepare_bench();
    start_bench();
    MotorControlCommand target = {.mode=MOTOR_CONTROL_SPEED, .speed_rad_s=1.0f};
    assert(FocRunState_SubmitRemoteControl(&target) == MOTOR_CONTROL_ACCEPTED);
    FocRunState_RequestRemoteStop();
    healthy_tick();
    assert(!control_pending && !bench_speed.active && hardware_phases_disabled);
    assert(FocBenchSpeed.state == FOC_BENCH_SPEED_STOPPED);
    assert(MotorControl.current_limit == 2.0f && MotorControl.speed_limit == 10.0f);
    assert(MotorControl.ErrorNow == No_Error);

    prepare_bench();
    start_bench();
    for (uint32_t i=0U; i<FOC_FREQ/1000U; ++i)
    {
        RunState_BenchFastGuard();
        FocRunState_FastFaultStop();
    }
    assert(hardware_phases_disabled && bench_speed.stop_requested);
    healthy_tick();
    assert(FocBenchSpeed.stop_reason == FOC_BENCH_SPEED_TIMEOUT);
    assert(FocBenchSpeed.state == FOC_BENCH_SPEED_STOPPED && !bench_speed.active);
    assert(MotorControl.ErrorNow == No_Error);
    healthy_tick();
    assert(hardware_phases_disabled && MotorControl.ModeNow == Motor_Disable);

    for (uint32_t phase=0U; phase<3U; ++phase)
    {
        prepare_bench();
        start_bench();
        if (phase == 0U)
        {
            FOC.Ia = 6.0f;
        }
        else if (phase == 1U)
        {
            FOC.Ib = -6.0f;
        }
        else
        {
            FOC.Ic = 6.0f;
        }
        RunState_BenchFastGuard();
        FocRunState_FastFaultStop();
        assert(MotorControl.ErrorNow == Over_Current && hardware_phases_disabled);
        healthy_tick();
        assert(FocBenchSpeed.stop_reason == FOC_BENCH_SPEED_PHASE_CURRENT);
        assert(lifecycle.snapshot.state == APP_FAULT && !bench_speed.active);
        assert((FocBenchSpeed.status_seq & 1U) == 0U);
    }
    printf("PASS bench speed: explicit zero start, bounds/health, STOP priority, timeout, phase trip\n");
}

int main(void)
{
    /* ===== 维护会话用例（阶段 C）：Save/Default/Zero ===== */
    {
        MotorWorkOutcome_TypeDef running = { MOTOR_WORK_RUNNING, Motor_Disable, No_Error, false };
        MotorWorkOutcome_TypeDef zero_done = { MOTOR_WORK_SWITCH_MODE, Save_Param, No_Error, false };

        /* SAVE 成功：COMMITTED 完成后回 READY。 */
        ResetWorld(Motor_Disable, Save_Param, No_Error, 1, true);
        FocRunState_PostOutcome(running); healthy_tick();
        assert(lifecycle.snapshot.state == APP_MAINTENANCE);
        assert(lifecycle.snapshot.operation == APP_OPERATION_SAVE);
        FocRunState_SaveFinished(true);
        FocRunState_PostOutcome(running); healthy_tick();
        assert(lifecycle.snapshot.state == APP_READY);
        assert(lifecycle.snapshot.operation == APP_OPERATION_NONE);
        assert(lifecycle.snapshot.operation_result == APP_OPERATION_COMPLETED);
        assert(lifecycle.snapshot.operation_effects == APP_EFFECT_COMMITTED);

        /* SAVE 失败：FAILED + 故障锁存；清除后回 READY。 */
        ResetWorld(Motor_Disable, Save_Param, No_Error, 1, true);
        FocRunState_PostOutcome(running); healthy_tick();
        FocRunState_SaveFinished(false);
        Set_ErrorNow(Test_Error);
        MotorControl.ModeNow = Motor_Disable;
        FocRunState_PostOutcome(running); healthy_tick();
        assert(lifecycle.snapshot.state == APP_FAULT);
        assert(lifecycle.snapshot.operation_result == APP_OPERATION_FAILED);
        assert(lifecycle.snapshot.latched_faults != 0U);
        Set_ErrorNow(No_Error);
        MotorControl.ModeNow = Clear_Error;
        FocRunState_PostOutcome(running); healthy_tick();
        assert(lifecycle.snapshot.state == APP_READY);
        assert(lifecycle.snapshot.latched_faults == 0U);

        /* ZERO → SAVE 链：ZERO 以 UNCOMMITTED 完成，随后开启 SAVE 会话。 */
        ResetWorld(Motor_Disable, Set_ZeroPosition, No_Error, 1, true);
        FocRunState_PostOutcome(running); healthy_tick();
        assert(lifecycle.snapshot.operation == APP_OPERATION_ZERO);
        FocRunState_PostOutcome(zero_done); healthy_tick();
        assert(lifecycle.snapshot.last_operation == APP_OPERATION_ZERO);
        assert(lifecycle.snapshot.operation_result == APP_OPERATION_COMPLETED);
        assert(lifecycle.snapshot.operation_effects == APP_EFFECT_UNCOMMITTED);
        assert(lifecycle.snapshot.operation == APP_OPERATION_NONE);
        FocRunState_PostOutcome(running); healthy_tick();
        assert(lifecycle.snapshot.operation == APP_OPERATION_SAVE);
        FocRunState_SaveFinished(true);
        FocRunState_PostOutcome(running); healthy_tick();
        assert(lifecycle.snapshot.operation == APP_OPERATION_NONE);
        assert(lifecycle.snapshot.operation_effects == APP_EFFECT_COMMITTED);

        /* DEFAULTS：等待 worker 请求保存后才完成，不能靠监督拍数猜测完成。 */
        ResetWorld(Motor_Disable, Default_Param, No_Error, 1, true);
        FocRunState_PostOutcome(running); healthy_tick();
        assert(lifecycle.snapshot.operation == APP_OPERATION_DEFAULTS);
        healthy_tick();
        assert(lifecycle.snapshot.operation == APP_OPERATION_DEFAULTS);
        FocRunState_PostOutcome(zero_done); healthy_tick();
        assert(lifecycle.snapshot.operation == APP_OPERATION_NONE);
        assert(lifecycle.snapshot.operation_result == APP_OPERATION_COMPLETED);
        assert(lifecycle.snapshot.operation_effects == APP_EFFECT_UNCOMMITTED);
        FocRunState_PostOutcome(running); healthy_tick();
        assert(lifecycle.snapshot.operation == APP_OPERATION_SAVE);

        /* 取消确认可在故障保持时收口，原故障直到显式 CLEAR 受理才清除。 */
        ResetWorld(Motor_Disable, Save_Param, No_Error, 1, true);
        FocRunState_PostOutcome(running); healthy_tick();
        assert(lifecycle.snapshot.operation == APP_OPERATION_SAVE);
        Set_ErrorNow(Test_Error);
        FocRunState_PostOutcome(running); healthy_tick();
        assert(lifecycle.snapshot.state == APP_FAULT);
        FocRunState_PostOutcome(running); healthy_tick();
        assert(lifecycle.snapshot.operation_result == APP_OPERATION_CANCELLED);
        assert(lifecycle.snapshot.operation == APP_OPERATION_NONE);
        assert(MotorControl.ErrorNow == Test_Error);
        Set_ErrorNow(Encoder_Error);
        OnBoard_Encoder.bad_frame_streak = 0U;
        MotorControl.ModeNow = Clear_Error;
        FocRunState_PostOutcome(running); healthy_tick();
        assert(lifecycle.snapshot.state == APP_READY);
        assert(lifecycle.snapshot.operation_result == APP_OPERATION_CANCELLED);
        assert(lifecycle.snapshot.operation == APP_OPERATION_NONE);

        printf("PASS operation sessions: save committed/failed, zero chain,"
               " defaults, cancel release\n");
    }

    /* ===== 标定会话用例（阶段 C 收尾）：齿槽 / 相电阻 ===== */
    {
        MotorWorkOutcome_TypeDef running = { MOTOR_WORK_RUNNING, Motor_Disable, No_Error, false };
        MotorWorkOutcome_TypeDef stop_tick = { MOTOR_WORK_STOP, Motor_Disable, No_Error, false };
        MotorWorkOutcome_TypeDef to_save = { MOTOR_WORK_SWITCH_MODE, Save_Param, No_Error, false };
        /* 编码器标定完成：切往 Save_Param 且先关断功率级（power_off=true）。 */
        MotorWorkOutcome_TypeDef to_save_off = { MOTOR_WORK_SWITCH_MODE, Save_Param, No_Error, true };

        /* 齿槽标定：完成切往 Save_Param → CALIBRATION 未提交完成，随后由 SAVE 提交。 */
        ResetWorld(Motor_Disable, Calib_Anticogging, No_Error, 1, true);
        FocRunState_PostOutcome(running); healthy_tick();
        assert(lifecycle.snapshot.operation == APP_OPERATION_CALIBRATION);
        FocRunState_PostOutcome(to_save); healthy_tick();
        assert(lifecycle.snapshot.last_operation == APP_OPERATION_CALIBRATION);
        assert(lifecycle.snapshot.operation_result == APP_OPERATION_COMPLETED);
        assert(lifecycle.snapshot.operation_effects == APP_EFFECT_UNCOMMITTED);
        assert(lifecycle.snapshot.operation == APP_OPERATION_NONE);

        /* 相电阻标定：入口自动启相（修复阶段 B 回归）；停机结果完成会话并关相。 */
        ResetWorld(Motor_Disable, Calib_PhaseResistance, No_Error, 1, true);
        FocRunState_PostOutcome(running); healthy_tick();
        assert(lifecycle.snapshot.operation == APP_OPERATION_CALIBRATION);
        assert(!hardware_phases_disabled);
        FocRunState_PostOutcome(stop_tick); healthy_tick();
        assert(lifecycle.snapshot.operation_result == APP_OPERATION_COMPLETED);
        assert(lifecycle.snapshot.operation_effects == APP_EFFECT_UNCOMMITTED);
        assert(lifecycle.snapshot.operation == APP_OPERATION_NONE);
        assert(hardware_phases_disabled);

        /* 摩擦辨识：入口自动启相（自自动使能迁移到会话）；停机结果完成会话并关相。 */
        ResetWorld(Motor_Disable, Calib_Friction, No_Error, 1, true);
        FocRunState_PostOutcome(running); healthy_tick();
        assert(lifecycle.snapshot.operation == APP_OPERATION_CALIBRATION);
        assert(!hardware_phases_disabled);
        FocRunState_PostOutcome(stop_tick); healthy_tick();
        assert(lifecycle.snapshot.operation_result == APP_OPERATION_COMPLETED);
        assert(lifecycle.snapshot.operation_effects == APP_EFFECT_UNCOMMITTED);
        assert(lifecycle.snapshot.operation == APP_OPERATION_NONE);
        assert(hardware_phases_disabled);

        /* 观测器 LUT 标定（Mode 13）：会话入口启相；完成关相并切往 Save_Param → 未提交完成。 */
        ResetWorld(Motor_Disable, Calib_EncoderObserver, No_Error, 1, true);
        FocRunState_PostOutcome(running); healthy_tick();
        assert(lifecycle.snapshot.operation == APP_OPERATION_CALIBRATION);
        assert(!hardware_phases_disabled);
        FocRunState_PostOutcome(to_save_off); healthy_tick();
        assert(lifecycle.snapshot.last_operation == APP_OPERATION_CALIBRATION);
        assert(lifecycle.snapshot.operation_result == APP_OPERATION_COMPLETED);
        assert(lifecycle.snapshot.operation_effects == APP_EFFECT_UNCOMMITTED);
        assert(lifecycle.snapshot.operation == APP_OPERATION_NONE);
        assert(hardware_phases_disabled);

        /* 电角度零位标定（Mode 15）：同构（完成关相并切往 Save_Param → 未提交完成）。 */
        ResetWorld(Motor_Disable, Calib_EleAngelOffset, No_Error, 1, true);
        FocRunState_PostOutcome(running); healthy_tick();
        assert(lifecycle.snapshot.operation == APP_OPERATION_CALIBRATION);
        assert(!hardware_phases_disabled);
        FocRunState_PostOutcome(to_save_off); healthy_tick();
        assert(lifecycle.snapshot.operation_result == APP_OPERATION_COMPLETED);
        assert(lifecycle.snapshot.operation_effects == APP_EFFECT_UNCOMMITTED);
        assert(lifecycle.snapshot.operation == APP_OPERATION_NONE);
        assert(hardware_phases_disabled);

        printf("PASS calibration sessions: anticogging then save, phase resistance and"
               " friction start/stop, encoder LUT/ele-zero save chains\n");
    }

    /* ===== 恢复矩阵用例（阶段 D1） ===== */
    {
        MotorWorkOutcome_TypeDef running = { MOTOR_WORK_RUNNING, Motor_Disable, No_Error, false };
        int i;

        /* 编码器：坏帧未清零拒绝；恢复后准入。 */
        ResetWorld(Motor_Disable, Motor_Disable, Encoder_Error, 1, true);
        OnBoard_Encoder.bad_frame_streak = 40U;
        MotorControl.ModeNow = Clear_Error;
        FocRunState_PostOutcome(running); healthy_tick();
        assert(MotorControl.ErrorNow == Encoder_Error);
        OnBoard_Encoder.bad_frame_streak = 0U;
        MotorControl.ModeNow = Clear_Error;
        FocRunState_PostOutcome(running); healthy_tick();
        assert(MotorControl.ErrorNow == No_Error);
        assert(lifecycle.snapshot.state == APP_READY);

        /* 高温：95 °C 拒绝，75 °C 准入（80 °C 滞回）。 */
        ResetWorld(Motor_Disable, Motor_Disable, High_Temprature, 1, true);
        McuTemperature.valid = 1U;
        FOC.temp = 95.0f;
        MotorControl.ModeNow = Clear_Error;
        FocRunState_PostOutcome(running); healthy_tick();
        assert(MotorControl.ErrorNow == High_Temprature);
        FOC.temp = 75.0f;
        MotorControl.ModeNow = Clear_Error;
        FocRunState_PostOutcome(running); healthy_tick();
        assert(MotorControl.ErrorNow == No_Error);

        /* 过压：窗口外拒绝，窗口内准入。 */
        ResetWorld(Motor_Disable, Motor_Disable, Over_Voltage, 1, true);
        FOC.Vbus_filt = 34.2f;
        MotorControl.ModeNow = Clear_Error;
        FocRunState_PostOutcome(running); healthy_tick();
        assert(MotorControl.ErrorNow == Over_Voltage);
        FOC.Vbus_filt = 32.0f;
        MotorControl.ModeNow = Clear_Error;
        FocRunState_PostOutcome(running); healthy_tick();
        assert(MotorControl.ErrorNow == No_Error);

        /* 过流：回落不足 100 ms 拒绝；驻留满后准入。 */
        ResetWorld(Motor_Disable, Motor_Disable, Over_Current, 1, true);
        FOC.Ia = 1.0f;
        FOC.Ib = 1.0f;
        FOC.Ic = 1.0f;
        MotorControl.ModeNow = Clear_Error;
        FocRunState_PostOutcome(running); healthy_tick();
        assert(MotorControl.ErrorNow == Over_Current);
        for (i = 0; i < 2100; ++i)
        {
            log_len = 0; /* 场景只累计恢复证据：丢弃指示灯日志避免溢出。 */
            FocRunState_PostOutcome(running); healthy_tick();
        }
        MotorControl.ModeNow = Clear_Error;
        FocRunState_PostOutcome(running); healthy_tick();
        assert(MotorControl.ErrorNow == No_Error);

        /* 未知故障值一律拒绝，避免隐式清错。 */
        ResetWorld(Motor_Disable, Motor_Disable, Test_Error, 1, true);
        MotorControl.ModeNow = Clear_Error;
        FocRunState_PostOutcome(running); healthy_tick();
        assert(MotorControl.ErrorNow == Test_Error);

        printf("PASS recovery matrix: encoder, temperature, voltage, overcurrent dwell,"
               " unknown denied\n");
    }

    test_remote_targets();
    test_real_guards();
    test_bench_speed();
    return 0;
}
"""


def run_state_fixture():
    source = (ROOT / "firmware/app/foc_run_state.c").read_text(encoding="utf-8")
    new_logic = (
        function_source(
            (ROOT / "firmware/motor/foc/foc_algorithm.c").read_text(encoding="utf-8"),
            "FOC_MaximumVoltage",
        )
        + function_source(source, "validate_remote_control")
        + function_source(source, "RunState_BenchCancel")
        + function_source(source, "apply_pending_control")
        + function_source(source, "Encoder_FeedbackRequired")
        + function_source(source, "RunState_ModeIsAutoStartable")
        + function_source(source, "RunState_ModeNeedsPreparation")
        + function_source(source, "RunState_ProjectControlMode")
        + function_source(source, "RunState_UpdateSampleProgress")
        + function_source(source, "RunState_BuildGuards")
        + function_source(source, "RunState_ApplyActions")
        + function_source(source, "RunState_Send")
        + function_source(source, "RunState_RunBootChain")
        + function_source(source, "RunState_RequestStop")
        + function_source(source, "RunState_ConfirmStart")
        + function_source(source, "RunState_VbusInEnableWindow")
        + function_source(source, "RunState_UpdateRecoveryEvidence")
        + function_source(source, "RunState_FaultSourceRecovered")
        + function_source(source, "RunState_TryClearFault")
        + function_source(source, "RunState_OperationFor")
        + function_source(source, "RunState_SendOperation")
        + function_source(source, "RunState_ServiceOperations")
        + function_source(source, "RunState_BeginOperationIfRequested")
        + function_source(source, "RunState_ServiceBenchRequest")
        + function_source(source, "RunState_BenchFastGuard")
        + function_source(source, "RunState_PublishBenchState")
        + function_source(source, "FocRunState_Init")
        + function_source(source, "FocRunState_SubmitRemoteControl")
        + function_source(source, "FocRunState_RequestRemoteStop")
        + function_source(source, "FocRunState_SaveFinished")
        + function_source(source, "FocRunState_PostOutcome")
        + function_source(source, "FocRunState_IsMaintenanceActive")
        + function_source(source, "FocRunState_FastFaultStop")
        + function_source(source, "FocRunState_TakeOutcome")
        + function_source(source, "FocRunState_Tick")
    )
    bench_context = source.split("/* BENCH_SPEED_CONTEXT_BEGIN */", 1)[1].split(
        "/* BENCH_SPEED_CONTEXT_END */", 1
    )[0]
    return FIXTURE_HEAD + bench_context + new_logic + DRIVER


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", required=True)
    parser.add_argument("--out", type=Path, default=ROOT / "outputs/run_state_tests")
    args = parser.parse_args()
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    source = out / "run_state_guards.c"
    source.write_text(run_state_fixture(), encoding="utf-8")
    compiler = (
        [args.cc]
        + (["cc"] if Path(args.cc).stem == "zig" else [])
        + NATIVE_INCLUDE_FLAGS
        + [
            "-I",
            str(ROOT / "firmware/services/lifecycle"),
            "-std=c99",
            "-O1",
            "-UNDEBUG",
            "-Wall",
            "-Wextra",
            "-Werror",
        ]
    )
    executable = out / "run_state_guards.exe"
    subprocess.run(
        compiler
        + [
            str(source),
            str(ROOT / "firmware/services/lifecycle/app_lifecycle.c"),
            "-o",
            str(executable),
        ],
        check=True,
    )
    subprocess.run([str(executable)], check=True)


if __name__ == "__main__":
    main()
