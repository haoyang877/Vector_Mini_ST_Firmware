"""运行状态机差分等价测试：旧实现与 AppLifecycle 适配器逐 tick 对拍；不访问硬件。"""

import argparse
import subprocess
from pathlib import Path

from run_position_servo_tests import NATIVE_INCLUDE_FLAGS, ROOT, function_source

OLD_LOGIC = r"""
/* ===== 旧实现（拆分前的三个函数，重命名对拍用） ===== */
static bool old_position_start_prepared;

static void Old_HandleFaultIndication(void)
{
    if (MotorControl.ErrorNow == No_Error)
    {
        LED_SetState(0, (uint8_t)MotorControl.ModeNow);
    }
    else
    {
        if (MotorControl.ModeNow != Save_Param && MotorControl.ModeNow != Default_Param)
        {
            MotorControl.ModeNow = Motor_Disable;
        }

        LED_SetState(1, (uint8_t)MotorControl.ErrorNow);
    }
}

static bool Old_ManagePowerStage(void)
{
    bool defer_position_power_start = false;

    if (ModeLast != Motor_Disable && MotorControl.ModeNow == Motor_Disable)
    {
        Clear_RunningData();
        Stop_PWM_Generate();
    }

    if (ModeLast == Motor_Disable && MotorControl.ModeNow != Motor_Disable &&
        MotorControl.ModeNow != Save_Param && MotorControl.ModeNow != Default_Param &&
        MotorControl.ModeNow != Clear_Error && MotorControl.ModeNow != Set_ZeroPosition &&
        MotorControl.ModeNow != Calib_Anticogging && MotorControl.axis_profile_valid)
    {
        if ((MotorControl.ModeNow == Position_Mode || MotorControl.ModeNow == Speed_Mode) &&
            (!old_position_start_prepared || !MotorOuterLoop_IsReady()))
        {
            old_position_start_prepared = true;
            defer_position_power_start = true;
        }
        else
        {
            Start_PWM_Generate();
        }
    }
    if (!defer_position_power_start)
    {
        old_position_start_prepared = false;
    }

    return defer_position_power_start;
}

static void Old_CommitModeAndError(bool defer_position_power_start)
{
    Detect_Mode_Error_Change();

    if (!defer_position_power_start)
    {
        ModeLast = MotorControl.ModeNow;
    }
    ErrorLast = MotorControl.ErrorNow;

    MotorControl.ModeNow_f = MotorControl.ModeNow;
    MotorControl.ErrorNow_f = MotorControl.ErrorNow;
}

static void Old_Tick(void)
{
    bool defer;
    Old_HandleFaultIndication();
    defer = Old_ManagePowerStage();
    Old_CommitModeAndError(defer);
}
"""

FIXTURE_HEAD = r"""
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "app_lifecycle.h"

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
#define No_Error 0
#define Test_Error 9
#define MotorParam_Error 12

/* 适配器内部状态（真实模块为文件级 static，这里由夹具提供）。 */
static AppLifecycle lifecycle;
static bool power_on;
static ModeNow_TypeDef operation_mode;
enum
{
    SAVE_FINISH_NONE = 0,
    SAVE_FINISH_COMMITTED,
    SAVE_FINISH_FAILED
};
static volatile uint8_t save_finish;

typedef struct
{
    ModeNow_TypeDef ModeNow;
    ErrorNow_TypeDef ErrorNow;
    uint8_t axis_profile_valid;
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

static void log2(int code, int arg)
{
    assert(log_len < LOG_CAP);
    log_codes[log_len] = code;
    log_args[log_len] = arg;
    ++log_len;
}

void LED_SetState(unsigned led, unsigned value) { log2(1 + (int)led, (int)value); }
void Clear_RunningData(void) { log2(3, 0); }
void Stop_PWM_Generate(void) { log2(4, 0); }
void Start_PWM_Generate(void) { log2(5, 0); }
void Detect_Mode_Error_Change(void) { log2(6, 0); }
/* 就绪查询是纯查询：适配器与旧实现的调用时机不同，不计入行为对拍。 */
bool MotorOuterLoop_IsReady(void) { return ready; }
"""

DRIVER = r"""
/* ===== 差分驱动 =====
 * 对拍两组：
 * A) 旧三函数（自行写模式/故障） vs 新适配器 Tick(MOTOR_WORK_RUNNING)；
 * B) 旧"worker 直接写模式/故障 + 旧三函数" vs 新" Tick(worker 结果)"。
 * 覆盖 LED/停机清理/停 PWM/启 PWM/变化检测与每 tick 后的状态。 */

static const ModeNow_TypeDef modes[] = {
    Motor_Disable, Save_Param, Default_Param, Clear_Error, Set_ZeroPosition,
    Calib_Anticogging, Position_Mode, Speed_Mode, Current_Mode, Vq_Mode,
};

static const MotorWorkOutcome_TypeDef outcomes[] = {
    { MOTOR_WORK_RUNNING, Motor_Disable, No_Error, false },
    { MOTOR_WORK_STOP, Motor_Disable, No_Error, false },
    { MOTOR_WORK_FAULT, Motor_Disable, Test_Error, false },
    { MOTOR_WORK_SWITCH_MODE, Motor_Disable, No_Error, false },
    { MOTOR_WORK_SWITCH_MODE, Save_Param, No_Error, false },
    { MOTOR_WORK_SWITCH_MODE, Default_Param, No_Error, false },
    { MOTOR_WORK_SWITCH_MODE, Clear_Error, No_Error, false },
    { MOTOR_WORK_SWITCH_MODE, Set_ZeroPosition, No_Error, false },
    { MOTOR_WORK_SWITCH_MODE, Calib_Anticogging, No_Error, false },
    { MOTOR_WORK_SWITCH_MODE, Position_Mode, No_Error, false },
    { MOTOR_WORK_SWITCH_MODE, Speed_Mode, No_Error, false },
    { MOTOR_WORK_SWITCH_MODE, Current_Mode, No_Error, false },
    { MOTOR_WORK_SWITCH_MODE, Vq_Mode, No_Error, false },
    /* power_off：等价于模块自行 Stop_PWM 后再写模式（标定完成路径）。 */
    { MOTOR_WORK_SWITCH_MODE, Save_Param, No_Error, true },
    { MOTOR_WORK_SWITCH_MODE, Motor_Disable, No_Error, true },
    { MOTOR_WORK_SWITCH_MODE, Current_Mode, No_Error, true },
};

#define TICKS 3
#define MODES_COUNT ((int)(sizeof(modes) / sizeof(modes[0])))
#define OUTCOMES_COUNT ((int)(sizeof(outcomes) / sizeof(outcomes[0])))

/* 旧 worker 的直接写入：等价于迁移前模块自行调用 Set_ModeNow/Set_ErrorNow。 */
static void ApplyDirect(MotorWorkOutcome_TypeDef outcome)
{
    switch (outcome.result)
    {
    case MOTOR_WORK_SWITCH_MODE:
        if (outcome.power_off)
        {
            Stop_PWM_Generate();
        }
        MotorControl.ModeNow = outcome.next_mode;
        break;
    case MOTOR_WORK_STOP:
        MotorControl.ModeNow = Motor_Disable;
        break;
    case MOTOR_WORK_FAULT:
        MotorControl.ErrorNow = outcome.error;
        break;
    default:
        break;
    }
}

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
    old_position_start_prepared = false;
    FocRunState_Init();
    log_len = 0;
}

typedef struct
{
    int codes[TICKS][LOG_CAP];
    int args[TICKS][LOG_CAP];
    int lens[TICKS];
    ModeNow_TypeDef mode[TICKS], mode_last[TICKS], mode_f[TICKS];
    ErrorNow_TypeDef error[TICKS], error_f[TICKS];
} Trace;

static void CaptureTick(Trace *trace, int tick, int start)
{
    trace->lens[tick] = log_len - start;
    memcpy(trace->codes[tick], log_codes + start, sizeof(int) * (size_t)trace->lens[tick]);
    memcpy(trace->args[tick], log_args + start, sizeof(int) * (size_t)trace->lens[tick]);
    trace->mode[tick] = MotorControl.ModeNow;
    trace->error[tick] = MotorControl.ErrorNow;
    trace->mode_last[tick] = ModeLast;
    trace->mode_f[tick] = MotorControl.ModeNow_f;
    trace->error_f[tick] = MotorControl.ErrorNow_f;
}

static int TraceEqual(const Trace *a, const Trace *b, int tick)
{
    return a->lens[tick] == b->lens[tick] &&
        memcmp(a->codes[tick], b->codes[tick], sizeof(int) * (size_t)a->lens[tick]) == 0 &&
        memcmp(a->args[tick], b->args[tick], sizeof(int) * (size_t)a->lens[tick]) == 0 &&
        a->mode[tick] == b->mode[tick] && a->error[tick] == b->error[tick] &&
        a->mode_last[tick] == b->mode_last[tick] &&
        a->mode_f[tick] == b->mode_f[tick] && a->error_f[tick] == b->error_f[tick];
}

static long CompareCase(ModeNow_TypeDef mode_last, ModeNow_TypeDef mode_now,
    ErrorNow_TypeDef error, uint8_t valid, bool is_ready,
    MotorWorkOutcome_TypeDef outcome, int *failed)
{
    Trace old_trace, new_trace;
    int tick, start;

    ResetWorld(mode_last, mode_now, error, valid, is_ready);
    for (tick = 0; tick < TICKS; ++tick)
    {
        start = log_len;
        ApplyDirect(outcome);
        Old_Tick();
        CaptureTick(&old_trace, tick, start);
    }

    ResetWorld(mode_last, mode_now, error, valid, is_ready);
    for (tick = 0; tick < TICKS; ++tick)
    {
        start = log_len;
        FocRunState_Tick(outcome);
        CaptureTick(&new_trace, tick, start);
    }

    for (tick = 0; tick < TICKS; ++tick)
    {
        if (!TraceEqual(&old_trace, &new_trace, tick))
        {
            printf("MISMATCH last=%d now=%d error=%d valid=%d ready=%d"
                   " result=%d next=%d tick=%d actions_old=%d actions_new=%d\n",
                   mode_last, mode_now, error, valid, is_ready,
                   (int)outcome.result, (int)outcome.next_mode, tick,
                   old_trace.lens[tick], new_trace.lens[tick]);
            *failed = 1;
            return 0;
        }
    }
    return TICKS;
}

int main(void)
{
    int i_last, i_now, i_error, i_valid, i_ready, i_outcome;
    long comparisons = 0;

    for (i_outcome = 0; i_outcome < OUTCOMES_COUNT; ++i_outcome)
    for (i_last = 0; i_last < MODES_COUNT; ++i_last)
    for (i_now = 0; i_now < MODES_COUNT; ++i_now)
    for (i_error = 0; i_error < 2; ++i_error)
    for (i_valid = 0; i_valid < 2; ++i_valid)
    for (i_ready = 0; i_ready < 2; ++i_ready)
    {
        int failed = 0;
        comparisons += CompareCase(modes[i_last], modes[i_now],
            i_error ? Test_Error : No_Error, (uint8_t)i_valid, i_ready != 0,
            outcomes[i_outcome], &failed);
        if (failed)
        {
            return 1;
        }
    }

    /* ===== 维护会话用例（阶段 C）：Save/Default/Zero ===== */
    {
        MotorWorkOutcome_TypeDef running = { MOTOR_WORK_RUNNING, Motor_Disable, No_Error, false };
        MotorWorkOutcome_TypeDef zero_done = { MOTOR_WORK_SWITCH_MODE, Save_Param, No_Error, false };

        /* SAVE 成功：COMMITTED 完成后回 READY。 */
        ResetWorld(Motor_Disable, Save_Param, No_Error, 1, true);
        FocRunState_Tick(running);
        assert(lifecycle.snapshot.state == APP_MAINTENANCE);
        assert(lifecycle.snapshot.operation == APP_OPERATION_SAVE);
        FocRunState_SaveFinished(true);
        FocRunState_Tick(running);
        assert(lifecycle.snapshot.state == APP_READY);
        assert(lifecycle.snapshot.operation == APP_OPERATION_NONE);
        assert(lifecycle.snapshot.operation_result == APP_OPERATION_COMPLETED);
        assert(lifecycle.snapshot.operation_effects == APP_EFFECT_COMMITTED);

        /* SAVE 失败：FAILED + 故障锁存；清除后回 READY。 */
        ResetWorld(Motor_Disable, Save_Param, No_Error, 1, true);
        FocRunState_Tick(running);
        FocRunState_SaveFinished(false);
        Set_ErrorNow(Test_Error);
        MotorControl.ModeNow = Motor_Disable;
        FocRunState_Tick(running);
        assert(lifecycle.snapshot.state == APP_FAULT);
        assert(lifecycle.snapshot.operation_result == APP_OPERATION_FAILED);
        assert(lifecycle.snapshot.latched_faults != 0U);
        Set_ErrorNow(No_Error);
        MotorControl.ModeNow = Clear_Error;
        FocRunState_Tick(running);
        assert(lifecycle.snapshot.state == APP_READY);
        assert(lifecycle.snapshot.latched_faults == 0U);

        /* ZERO → SAVE 链：ZERO 以 UNCOMMITTED 完成，随后开启 SAVE 会话。 */
        ResetWorld(Motor_Disable, Set_ZeroPosition, No_Error, 1, true);
        FocRunState_Tick(running);
        assert(lifecycle.snapshot.operation == APP_OPERATION_ZERO);
        FocRunState_Tick(zero_done);
        assert(lifecycle.snapshot.last_operation == APP_OPERATION_ZERO);
        assert(lifecycle.snapshot.operation_result == APP_OPERATION_COMPLETED);
        assert(lifecycle.snapshot.operation_effects == APP_EFFECT_UNCOMMITTED);
        assert(lifecycle.snapshot.operation == APP_OPERATION_NONE);
        FocRunState_Tick(running);
        assert(lifecycle.snapshot.operation == APP_OPERATION_SAVE);
        FocRunState_SaveFinished(true);
        FocRunState_Tick(running);
        assert(lifecycle.snapshot.operation == APP_OPERATION_NONE);
        assert(lifecycle.snapshot.operation_effects == APP_EFFECT_COMMITTED);

        /* DEFAULTS：UNCOMMITTED 完成；同值持续不重复开启。 */
        ResetWorld(Motor_Disable, Default_Param, No_Error, 1, true);
        FocRunState_Tick(running);
        assert(lifecycle.snapshot.operation == APP_OPERATION_DEFAULTS);
        FocRunState_Tick(running);
        assert(lifecycle.snapshot.operation == APP_OPERATION_NONE);
        assert(lifecycle.snapshot.operation_result == APP_OPERATION_COMPLETED);
        assert(lifecycle.snapshot.operation_effects == APP_EFFECT_UNCOMMITTED);
        FocRunState_Tick(running);
        assert(lifecycle.snapshot.operation == APP_OPERATION_NONE);

        /* 会话被故障打断：故障未清除前 CANCEL_DONE 被健康 guard 拒绝；
         * 清除 ErrorNow 的同一拍先确认取消再放行 CLEAR，回到 READY。 */
        ResetWorld(Motor_Disable, Save_Param, No_Error, 1, true);
        FocRunState_Tick(running);
        assert(lifecycle.snapshot.operation == APP_OPERATION_SAVE);
        Set_ErrorNow(Test_Error);
        FocRunState_Tick(running);
        assert(lifecycle.snapshot.state == APP_FAULT);
        FocRunState_Tick(running);
        assert(lifecycle.snapshot.operation_result == APP_OPERATION_CANCEL_REQUESTED);
        assert(lifecycle.snapshot.operation == APP_OPERATION_SAVE);
        Set_ErrorNow(No_Error);
        MotorControl.ModeNow = Clear_Error;
        FocRunState_Tick(running);
        assert(lifecycle.snapshot.state == APP_READY);
        assert(lifecycle.snapshot.operation_result == APP_OPERATION_CANCELLED);
        assert(lifecycle.snapshot.operation == APP_OPERATION_NONE);

        printf("PASS operation sessions: save committed/failed, zero chain,"
               " defaults, cancel release\n");
    }

    printf("PASS run state differential: %ld tick comparisons identical"
           " (legacy trio vs AppLifecycle adapter)\n", comparisons);
    return 0;
}
"""


def run_state_fixture():
    source = (ROOT / "firmware/app/foc_run_state.c").read_text(encoding="utf-8")
    new_logic = (
        function_source(source, "RunState_ModeIsAutoStartable")
        + function_source(source, "RunState_ModeNeedsPreparation")
        + function_source(source, "RunState_ProjectControlMode")
        + function_source(source, "RunState_BuildGuards")
        + function_source(source, "RunState_ApplyActions")
        + function_source(source, "RunState_Send")
        + function_source(source, "RunState_RunBootChain")
        + function_source(source, "RunState_RequestStop")
        + function_source(source, "RunState_ConfirmStart")
        + function_source(source, "RunState_OperationFor")
        + function_source(source, "RunState_SendOperation")
        + function_source(source, "RunState_ServiceOperations")
        + function_source(source, "RunState_BeginOperationIfRequested")
        + function_source(source, "FocRunState_Init")
        + function_source(source, "FocRunState_SaveFinished")
        + function_source(source, "FocRunState_Tick")
    )
    return FIXTURE_HEAD + OLD_LOGIC + new_logic + DRIVER


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", required=True)
    parser.add_argument("--out", type=Path, default=ROOT / "outputs/run_state_tests")
    args = parser.parse_args()
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    source = out / "run_state_diff.c"
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
    executable = out / "run_state_diff.exe"
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
