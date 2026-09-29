#include "motor_maintenance.h"

#include <math.h>
#include "bus_voltage_profile.h"
#include "foc_cogging_calibration.h"
#include "encoder_calibration_task.h"
#include "friction_identification_task.h"
#include "phase_resistance_task.h"
#include "foc_param.h"
#include "motor_execution.h"
#include "motor_profile_defaults.h"
#include "board_profile_defaults.h"
#include "application_profile_defaults.h"
#include "device_profile_defaults.h"
#include "control_config.h"
#include "control_math.h"
#include "parameter_value_service.h"
#include "maintenance_workspace.h"

/* 产品默认值只在组合层装配，算法接收只读显式输入。 */
#define PHASE_RESISTANCE_RAMP_TICKS ((FOC_FREQ * PARAM_MOTOR_PHASE_RESISTANCE_RAMP_TIME_MS) / 1000U)
#define PHASE_RESISTANCE_SETTLE_TICKS                                                              \
    ((FOC_FREQ * PARAM_MOTOR_PHASE_RESISTANCE_SETTLE_TIME_MS) / 1000U)
#define PHASE_RESISTANCE_SAMPLE_TICKS                                                              \
    ((FOC_FREQ * PARAM_MOTOR_PHASE_RESISTANCE_SAMPLE_TIME_MS) / 1000U)
#define PHASE_RESISTANCE_PAUSE_TICKS                                                               \
    ((FOC_FREQ * PARAM_MOTOR_PHASE_RESISTANCE_PAUSE_TIME_MS) / 1000U)
#define PHASE_RESISTANCE_TIMEOUT_TICKS                                                             \
    ((FOC_FREQ * PARAM_MOTOR_PHASE_RESISTANCE_TIMEOUT_MS) / 1000U)
#define PHASE_RESISTANCE_TEST_CURRENT_MIN_A 0.5f
#define PHASE_RESISTANCE_CURRENT_TOLERANCE 0.10f
#define PHASE_RESISTANCE_Q_CURRENT_TOLERANCE 0.10f
#define PHASE_RESISTANCE_VOLTAGE_TOLERANCE 0.01f
#define PHASE_RESISTANCE_VOLTAGE_MIN_DELTA 0.005f
#define PHASE_RESISTANCE_VOLTAGE_FILTER 0.02f

#if PARAM_FRICTION_IDENT_SPEED_POINT_COUNT != 4U
#error "Update the friction composition when changing the profile point count"
#endif

static const FrictionIdentificationTaskConfig s_friction_config = {
    .core =
        {
            .update_period_s = Speed_Ts,
            .speed_point_count = PARAM_FRICTION_IDENT_SPEED_POINT_COUNT,
            .speed_points_rad_s = {PARAM_FRICTION_IDENT_SPEED_0_RPS * _2PI,
                                   PARAM_FRICTION_IDENT_SPEED_1_RPS *_2PI,
                                   PARAM_FRICTION_IDENT_SPEED_2_RPS *_2PI,
                                   PARAM_FRICTION_IDENT_SPEED_3_RPS *_2PI},
            .stable_time_s = PARAM_FRICTION_IDENT_STABLE_TIME_S,
            .track_timeout_s = PARAM_FRICTION_IDENT_TRACK_TIMEOUT_S,
            .sample_timeout_s = PARAM_FRICTION_IDENT_SAMPLE_TIMEOUT_S,
            .stop_hold_time_s = PARAM_FRICTION_IDENT_STOP_HOLD_TIME_S,
            .stop_timeout_s = PARAM_FRICTION_IDENT_STOP_TIMEOUT_S,
            .speed_tolerance_ratio = PARAM_FRICTION_IDENT_SPEED_TOLERANCE_RATIO,
            .minimum_speed_tolerance_rad_s = PARAM_FRICTION_IDENT_MIN_SPEED_TOL_RAD_S,
            .stop_speed_rad_s = PARAM_FRICTION_IDENT_STOP_SPEED_RAD_S,
            .sample_turns = PARAM_FRICTION_IDENT_SAMPLE_TURNS,
            .minimum_sample_time_s = PARAM_FRICTION_IDENT_MIN_SAMPLE_TIME_S,
            .saturation_time_s = PARAM_FRICTION_IDENT_SATURATION_TIME_S,
            .rmse_floor_a = PARAM_FRICTION_IDENT_RMSE_FLOOR_A,
            .rmse_ratio_max = PARAM_FRICTION_IDENT_RMSE_RATIO_MAX,
        },
    .current_ratio_max = PARAM_FRICTION_IDENT_CURRENT_RATIO_MAX,
};

static const PhaseResistanceConfig s_phase_resistance_config = {
    .test_current_low = PARAM_MOTOR_PHASE_RESISTANCE_TEST_CURRENT_LOW_A,
    .test_current_high = PARAM_MOTOR_PHASE_RESISTANCE_TEST_CURRENT_HIGH_A,
    .maximum_test_current = PARAM_MOTOR_PHASE_RESISTANCE_TEST_CURRENT_MAX_A,
    .minimum_test_current = PHASE_RESISTANCE_TEST_CURRENT_MIN_A,
    .ramp_ticks = PHASE_RESISTANCE_RAMP_TICKS,
    .settle_ticks = PHASE_RESISTANCE_SETTLE_TICKS,
    .sample_ticks = PHASE_RESISTANCE_SAMPLE_TICKS,
    .pause_ticks = PHASE_RESISTANCE_PAUSE_TICKS,
    .timeout_ticks = PHASE_RESISTANCE_TIMEOUT_TICKS,
    .current_tolerance = PHASE_RESISTANCE_CURRENT_TOLERANCE,
    .q_current_tolerance = PHASE_RESISTANCE_Q_CURRENT_TOLERANCE,
    .voltage_tolerance = PHASE_RESISTANCE_VOLTAGE_TOLERANCE,
    .voltage_min_delta = PHASE_RESISTANCE_VOLTAGE_MIN_DELTA,
    .voltage_filter = PHASE_RESISTANCE_VOLTAGE_FILTER,
    .path_compensation_ohm = PARAM_HW_PHASE_RESISTANCE_PATH_COMPENSATION_OHM,
    .balance_warning_pct = PARAM_MOTOR_PHASE_RESISTANCE_BALANCE_WARNING_PCT,
    .balance_fault_pct = PARAM_MOTOR_PHASE_RESISTANCE_BALANCE_FAULT_PCT,
};

void MotorMaintenance_Init(void)
{
    FrictionIdentificationTask_Init(&s_friction_config);
}

/* 维护调度只调用实际功能；生命周期、输出许可、保存衔接由唯一主状态机拥有。 */
MotorFaultCode MotorMaintenance_Admit(MotorMaintenanceKind kind,
                                      const MotorControlState *motor,
                                      const FocState *foc,
                                      const EncoderState *encoder)
{
    if ((unsigned)kind >= MOTOR_MAINTENANCE_NONE || !motor->application.axis_profile_valid)
    {
        return MOTOR_FAULT_INVALID_PARAMETERS;
    }
    bool torque_task =
        kind == MOTOR_MAINTENANCE_ENCODER_LUT || kind == MOTOR_MAINTENANCE_ELECTRICAL_ZERO ||
        kind == MOTOR_MAINTENANCE_COGGING || kind == MOTOR_MAINTENANCE_PHASE_RESISTANCE ||
        kind == MOTOR_MAINTENANCE_FRICTION;
    if (torque_task)
    {
        if (!motor->configuration_confirmed || !ParameterValue_ConfigurationValid(motor))
        {
            return MOTOR_FAULT_INVALID_PARAMETERS;
        }
        if (!isfinite(foc->bus_voltage_v) || !isfinite(foc->bus_voltage_filtered_v) ||
            foc->bus_voltage_v >= BUS_VOLTAGE_HARD_OVERVOLTAGE_V ||
            foc->bus_voltage_filtered_v > BUS_VOLTAGE_ENABLE_MAX_V)
        {
            return MOTOR_FAULT_OVERVOLTAGE;
        }
        if (foc->bus_voltage_filtered_v < BUS_VOLTAGE_ENABLE_MIN_V)
        {
            return MOTOR_FAULT_UNDERVOLTAGE;
        }
    }
    if (kind == MOTOR_MAINTENANCE_COGGING && !FocCogging_CanStart(motor, encoder))
    {
        return MOTOR_FAULT_COGGING_CALIBRATION;
    }
    if (kind == MOTOR_MAINTENANCE_ELECTRICAL_ZERO &&
        (Encoder_GetCalibFlag(encoder) & ENC_CALIB_LINEARIZED) == 0U)
    {
        return MOTOR_FAULT_ENCODER_NOT_CALIBRATED;
    }
    if (kind != MOTOR_MAINTENANCE_SAVE && kind != MOTOR_MAINTENANCE_DEFAULTS &&
        kind != MOTOR_MAINTENANCE_PHASE_RESISTANCE && !Encoder_IsOnline(encoder))
    {
        return MOTOR_FAULT_ENCODER;
    }
    if (kind == MOTOR_MAINTENANCE_FRICTION &&
        (Encoder_GetCalibFlag(encoder) & ENC_CALIB_ALL) != ENC_CALIB_ALL)
    {
        return MOTOR_FAULT_ENCODER_NOT_CALIBRATED;
    }
    return MOTOR_FAULT_NONE;
}

MotorTaskResult MotorMaintenance_Step(MotorMaintenanceKind kind,
                                      FocState *foc,
                                      MotorControlState *motor,
                                      PiController *pi,
                                      EncoderState *encoder,
                                      FluxObserverState *observer,
                                      SensorlessStartupState *startup)
{
    switch (kind)
    {
    case MOTOR_MAINTENANCE_ENCODER_LUT:
    {
        EncoderCalibrationWorkspace *workspace = MaintenanceWorkspace_BorrowEncoder();
        MotorTaskResult result =
            EncoderCalibrationTask_StepLut(foc, motor, pi, encoder, observer, startup, workspace);
        if (EncoderCalibrationTask_IsReleased())
        {
            MaintenanceWorkspace_ReleaseEncoder();
        }
        return result;
    }
    case MOTOR_MAINTENANCE_ELECTRICAL_ZERO:
        return EncoderCalibrationTask_StepElectricalZero(foc, motor, encoder);
    case MOTOR_MAINTENANCE_COGGING:
        return FocCogging_Task(foc, motor, pi, encoder);
    case MOTOR_MAINTENANCE_PHASE_RESISTANCE:
        return PhaseResistanceTask_Run(foc, motor, &s_phase_resistance_config);
    case MOTOR_MAINTENANCE_FRICTION:
        return FrictionIdentificationTask_Task(foc, motor, pi, encoder, &s_friction_config);
    case MOTOR_MAINTENANCE_MECHANICAL_ZERO:
        if (!Encoder_SetMechanicalZero(encoder))
        {
            return (MotorTaskResult){MOTOR_TASK_FAILED, MOTOR_FAULT_ENCODER};
        }
        MotorOuterLoop_RequestReset();
        return (MotorTaskResult){MOTOR_TASK_COMPLETED, MOTOR_FAULT_NONE};
    case MOTOR_MAINTENANCE_DEFAULTS:
        Param_Return_Default(motor, encoder);
        return (MotorTaskResult){MOTOR_TASK_COMPLETED, MOTOR_FAULT_NONE};
    case MOTOR_MAINTENANCE_SAVE:
        return (MotorTaskResult){MOTOR_TASK_RUNNING, MOTOR_FAULT_NONE};
    default:
        return (MotorTaskResult){MOTOR_TASK_FAILED, MOTOR_FAULT_INVALID_PARAMETERS};
    }
}

void MotorMaintenance_Cancel(MotorMaintenanceKind kind,
                             FocState *foc,
                             MotorControlState *motor,
                             PiController *pi)
{
    switch (kind)
    {
    case MOTOR_MAINTENANCE_ENCODER_LUT:
    case MOTOR_MAINTENANCE_ELECTRICAL_ZERO:
        EncoderCalibrationTask_Cancel();
        if (EncoderCalibrationTask_IsReleased())
        {
            MaintenanceWorkspace_ReleaseEncoder();
        }
        break;
    case MOTOR_MAINTENANCE_COGGING:
        FocCogging_Abort();
        break;
    case MOTOR_MAINTENANCE_PHASE_RESISTANCE:
        PhaseResistanceTask_Cancel(foc, motor);
        break;
    case MOTOR_MAINTENANCE_FRICTION:
        FrictionIdentificationTask_Abort(motor, pi);
        break;
    default:
        break;
    }
}

bool MotorMaintenance_IsReleased(void)
{
    return MaintenanceWorkspace_IsReleased() && EncoderCalibrationTask_IsReleased() &&
           FocCogging_IsReleased() && FrictionIdentificationTask_IsReleased() &&
           PhaseResistanceTask_IsReleased();
}
