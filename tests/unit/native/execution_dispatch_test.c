#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "fast_loop_profile.h"
#include "foc_cogging_calibration.h"
#include "motor_execution.h"
#include "motor_hw.h"
#include "motor_maintenance.h"
#include "motor_state_machine.h"

/* 只替换算法、平台和所有者查询；调度体与控制分发体从生产源码原样编译。
 * 枚举交叉覆盖故障优先、维护独占、空闲预载及控制终态，不模拟物理电机。 */
static MotorControlState s_motor_state;
static FocState s_foc_state;
static PiController s_speed_pi;
static EncoderState s_rotor_encoder;
static FluxObserverState s_flux_observer;
static SensorlessStartupState s_sensorless_startup;
static MotorState lifecycle_state;
static MotorMaintenanceKind maintenance_kind;
static MotorTaskResult maintenance_result;
static bool maintenance_runnable, torque_trip;
static unsigned trace;
static unsigned long trace_hash = 2166136261UL;

enum
{
    TRACE_RUNNABLE = 1U,
    TRACE_KIND = 2U,
    TRACE_STATE = 4U,
    TRACE_IDLE = 8U,
    TRACE_CURRENT_TASK = 16U,
    TRACE_CURRENT_LOOP = 32U,
    TRACE_VQ = 64U,
    TRACE_TRIP = 128U,
    TRACE_MAINTENANCE = 256U,
    TRACE_PROFILE_BEGIN = 512U,
    TRACE_PROFILE_END = 1024U
};

bool MotorStateMachine_MaintenanceRunnable(void)
{
    trace |= TRACE_RUNNABLE;
    return maintenance_runnable;
}

MotorMaintenanceKind MotorStateMachine_GetMaintenanceKind(void)
{
    trace |= TRACE_KIND;
    return maintenance_kind;
}

MotorState MotorStateMachine_GetState(void)
{
    trace |= TRACE_STATE;
    return lifecycle_state;
}

void motor_hw_pwm_set_duty(float a, float b, float c)
{
    assert(a == 0.5f && b == 0.5f && c == 0.5f);
    trace |= TRACE_IDLE;
}

MotorTaskResult MotorMaintenance_Step(MotorMaintenanceKind kind,
                                      FocState *foc,
                                      MotorControlState *motor,
                                      PiController *pi,
                                      EncoderState *encoder,
                                      FluxObserverState *observer,
                                      SensorlessStartupState *startup)
{
    assert(kind == maintenance_kind && foc == &s_foc_state && motor == &s_motor_state);
    assert(pi == &s_speed_pi && encoder == &s_rotor_encoder);
    assert(observer == &s_flux_observer && startup == &s_sensorless_startup);
    trace |= TRACE_MAINTENANCE;
    return maintenance_result;
}

void Task_Current_Mode(FocState *foc,
                       MotorControlState *motor,
                       EncoderState *encoder,
                       FluxObserverState *observer)
{
    assert(foc == &s_foc_state && motor == &s_motor_state);
    assert(encoder == &s_rotor_encoder && observer == &s_flux_observer);
    trace |= TRACE_CURRENT_TASK;
}

bool FocCogging_TakeTorqueTrip(void)
{
    trace |= TRACE_TRIP;
    return torque_trip;
}

float Encoder_GetElePhase(const EncoderState *encoder)
{
    assert(encoder == &s_rotor_encoder);
    return 0.75f;
}

float Encoder_GetEleVel(const EncoderState *encoder)
{
    assert(encoder == &s_rotor_encoder);
    return 2.5f;
}

void FOC_Current(FocState *foc, MotorControlState *motor, float phase, float velocity)
{
    assert(foc == &s_foc_state && motor == &s_motor_state);
    assert(phase == 0.75f && velocity == 2.5f);
    trace |= TRACE_CURRENT_LOOP;
}

void Task_Vq_Mode(FocState *foc, MotorControlState *motor, EncoderState *encoder)
{
    assert(foc == &s_foc_state && motor == &s_motor_state && encoder == &s_rotor_encoder);
    trace |= TRACE_VQ;
}

#if defined(FAST_LOOP_STAGE_PROFILE) && FAST_LOOP_STAGE_PROFILE
void FastLoopProfile_Begin(unsigned stage)
{
    assert(stage == FAST_PROFILE_POSITION_WITH_CURRENT);
    trace |= TRACE_PROFILE_BEGIN;
}

void FastLoopProfile_End(unsigned stage)
{
    assert(stage == FAST_PROFILE_POSITION_WITH_CURRENT);
    trace |= TRACE_PROFILE_END;
}
#endif

/* 合成输入的行为期望独立于调度内部实现；不把受理状态当作物理输出事实。 */
static void check_result(MotorTaskResult actual)
{
    unsigned expected_trace = 0U;
    MotorTaskResult expected = {MOTOR_TASK_RUNNING, MOTOR_FAULT_NONE};
    if (s_motor_state.fault_code == MOTOR_FAULT_NONE)
    {
        expected_trace = TRACE_RUNNABLE;
        if (maintenance_runnable)
        {
            expected_trace |= TRACE_KIND | TRACE_MAINTENANCE;
            expected = maintenance_result;
        }
        else
        {
            expected_trace |= TRACE_STATE;
            if (lifecycle_state != MOTOR_STATE_STARTING && lifecycle_state != MOTOR_STATE_RUNNING)
            {
                expected_trace |= TRACE_IDLE;
            }
            else
            {
                switch (s_motor_state.control_mode)
                {
                case MOTOR_CONTROL_CURRENT:
                    expected_trace |= TRACE_CURRENT_TASK | TRACE_TRIP;
                    if (torque_trip)
                    {
                        expected.result = MOTOR_TASK_STOP_REQUESTED;
                    }
                    break;
                case MOTOR_CONTROL_POSITION:
#if defined(FAST_LOOP_STAGE_PROFILE) && FAST_LOOP_STAGE_PROFILE
                    expected_trace |= TRACE_PROFILE_BEGIN | TRACE_PROFILE_END;
#endif
                    expected_trace |= TRACE_CURRENT_LOOP;
                    break;
                case MOTOR_CONTROL_SPEED:
                    expected_trace |= TRACE_CURRENT_LOOP;
                    break;
                case MOTOR_CONTROL_VQ:
                    expected_trace |= TRACE_VQ;
                    break;
                default:
                    break;
                }
            }
        }
    }
    assert(trace == expected_trace);
    assert(actual.result == expected.result && actual.error == expected.error);
    trace_hash = (trace_hash ^ trace) * 16777619UL;
    trace_hash = (trace_hash ^ (unsigned)actual.result) * 16777619UL;
    trace_hash = (trace_hash ^ (unsigned)actual.error) * 16777619UL;
}

/* 生产函数在构建时插入；本入口始终调用生产调度，允许旧版与新版差分。 */
static MotorTaskResult dispatch_step(void);

int main(void)
{
    unsigned cases = 0U;
    memset(&s_motor_state, 0, sizeof(s_motor_state));
#if defined(STARTUP_CURRENT_OFFSETS_PRESENT)
    /* 校正pending/failed均未complete，所有模式和维护请求都不得进入功能分发。 */
    s_current_offset_calibration.complete = false;
    maintenance_runnable = true;
    maintenance_kind = MOTOR_MAINTENANCE_ENCODER_LUT;
    for (unsigned state = MOTOR_STATE_INIT; state <= MOTOR_STATE_FATAL; ++state)
    {
        lifecycle_state = (MotorState)state;
        for (unsigned mode = MOTOR_CONTROL_CURRENT; mode <= MOTOR_CONTROL_NONE; ++mode)
        {
            s_motor_state.control_mode = (MotorControlMode)mode;
            trace = 0U;
            MotorTaskResult actual = dispatch_step();
            assert(trace == 0U && actual.result == MOTOR_TASK_RUNNING &&
                   actual.error == MOTOR_FAULT_NONE);
        }
    }
    s_current_offset_calibration.complete = true;
#endif
    for (unsigned state = MOTOR_STATE_INIT; state <= MOTOR_STATE_FATAL; ++state)
    {
        lifecycle_state = (MotorState)state;
        for (unsigned mode = MOTOR_CONTROL_CURRENT; mode <= MOTOR_CONTROL_NONE; ++mode)
        {
            s_motor_state.control_mode = (MotorControlMode)mode;
            for (unsigned kind = 0U; kind <= MOTOR_MAINTENANCE_NONE; ++kind)
            {
                maintenance_kind = (MotorMaintenanceKind)kind;
                for (unsigned flags = 0U; flags < 8U; ++flags)
                {
                    maintenance_runnable = (flags & 1U) != 0U;
                    torque_trip = (flags & 2U) != 0U;
                    s_motor_state.fault_code =
                        (flags & 4U) ? MOTOR_FAULT_ENCODER : MOTOR_FAULT_NONE;
                    for (unsigned result = MOTOR_TASK_RUNNING; result <= MOTOR_TASK_FAILED;
                         ++result)
                    {
                        maintenance_result = (MotorTaskResult){(MotorTaskState)result,
                                                               result == MOTOR_TASK_FAILED
                                                                   ? MOTOR_FAULT_INVALID_PARAMETERS
                                                                   : MOTOR_FAULT_NONE};
                        trace = 0U;
                        check_result(dispatch_step());
                        ++cases;
                    }
                }
            }
        }
    }
    printf("PASS %u scheduling cases; trace=%lu\n", cases, trace_hash);
    return 0;
}
