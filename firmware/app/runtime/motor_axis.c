#include "motor_axis.h"
#include "application_config.h"
#include "bsp_encoder.h"
#include "bsp_pwm.h"
#include "bsp_system.h"
#include "encoder_calibration.h"
#include "motor_sensing.h"
#include "bsp_flash.h"
#include "parameter_record.h"
#include "parameter_values.h"

#include "encoder_feedback.h"
#include "foc.h"
#include "motor_fault.h"
#include "pi_controller.h"
#include "motor_control.h"
#include "motor_lifecycle.h"
#include "flux_observer.h"
#include "sensorless_startup.h"

/* 持有单轴对象并编排快慢环；全局符号保留给只读 Watch。 */
MotorConfiguration s_motor_configuration;
MotorControlState s_motor_state;
PiController s_speed_pi;
EncoderState s_rotor_encoder;
FluxObserverState s_flux_observer;
SensorlessStartupState s_sensorless_startup;
FocState s_foc_state;
CurrentOffsetCalibration s_current_offset_calibration;

const MotorAxisFeedback MotorAxis_Feedback = {&s_foc_state, &s_rotor_encoder};

/* 编码器线性化独占的6144字节静态工作区；发布或取消前不复用。 */
static EncoderCalibrationWorkspace s_encoder_linearization_workspace;
/* 启动装载和停机前台保存串行复用，避免再分配整条记录。 */
static InterfaceParam s_parameter_record;

static MotorTaskResult run_active_calibration_cycle(void)
{
    const EncoderCalibrationControl control = {&s_motor_configuration,
                                               &s_foc_state,
                                               &s_motor_state,
                                               &s_speed_pi,
                                               &s_rotor_encoder,
                                               &s_flux_observer,
                                               &s_sensorless_startup};
    switch (MotorLifecycle_GetCalibrationType())
    {
    case MOTOR_CALIBRATION_ENCODER_LINEARIZATION:
        return EncoderCalibration_StepLut(&control, &s_encoder_linearization_workspace);
    case MOTOR_CALIBRATION_ELECTRICAL_ZERO:
        return EncoderCalibration_StepElecZero(&control);
    case MOTOR_CALIBRATION_MECHANICAL_ZERO:
        if (Encoder_SetMechanicalZero(&s_motor_configuration.device.encoder, &s_rotor_encoder))
        {
            MotorOuterLoop_RequestReset();
            return (MotorTaskResult){MOTOR_TASK_COMPLETED, MOTOR_FAULT_NONE};
        }
        return (MotorTaskResult){MOTOR_TASK_FAILED, MOTOR_FAULT_ENCODER};
    default:
        return (MotorTaskResult){MOTOR_TASK_FAILED, MOTOR_FAULT_INVALID_PARAMETERS};
    }
}

void MotorAxis_Init(void)
{
    s_foc_state.bus_voltage_filter_alpha = 0.05f;
    s_foc_state.bus_current_filter_alpha = 0.05f;
    /* 固定启动暂存区与停机保存共用；装载失败沿用默认值。 */
    if (BspFlash_Read(&s_parameter_record, sizeof(s_parameter_record)) == BSP_OK)
    {
        Param_Download(&s_parameter_record, sizeof(s_parameter_record), &s_motor_configuration);
    }
    else
    {
        Param_Return_Default(&s_motor_configuration);
    }
    MotorCurrentOffsets_Init(&s_current_offset_calibration,
                             &s_motor_configuration,
                             &s_foc_state,
                             BSP_ENABLED_BUS_CURRENT != 0);
    MotorLifecycle_Init(&s_motor_configuration,
                        &s_motor_state,
                        &s_foc_state,
                        &s_speed_pi,
                        &s_rotor_encoder,
                        &s_sensorless_startup);
    Encoder_InitFeedback(&s_motor_configuration.device.encoder,
                         &s_motor_configuration.model.motor_pole_pairs,
                         &s_motor_configuration.application.axis_profile,
                         &s_motor_configuration.application.axis_profile_valid,
                         &s_rotor_encoder);

    FluxObserver_Init(&s_flux_observer);
    SensorlessStartup_Reset(&s_sensorless_startup);
    Foc_ResetCurrent(&s_foc_state);
    PiController_Reset(&s_speed_pi);
    Foc_ConfigureControllers(&s_motor_configuration, &s_foc_state, &s_speed_pi);
    Foc_UpdateCurrentLimits(&s_foc_state);
    s_speed_pi.output_min = -1.0f;
    s_speed_pi.output_max = 1.0f;

    s_motor_configuration.application.position_error_window_rad =
        PARAM_APP_POSITION_ERROR_WINDOW_RAD;
    s_motor_state.position_velocity_filtered_rad_s = 0.0f;

    MotorOuterLoop_RequestReset();

    /* 状态机初始化已禁用控制；无轴记录时继续锁存参数故障。 */
    if (!s_motor_configuration.application.axis_profile_valid)
    {
        MotorFault_Report(&s_motor_state, MOTOR_FAULT_INVALID_PARAMETERS);
    }
}

void MotorAxis_SupervisorTick(void)
{
    Encoder_UpdateSlowEstimate(&s_rotor_encoder);
    McuTemperature_Update(&s_foc_state, &s_motor_state);
    MotorLifecycle_SupervisorTick();
}

void MotorAxis_SampleFast(void)
{
    /* 编码器传输与采样和观测器计算重叠。 */
    BspEncoder_Begin();

    MotorVoltage_Update(&s_foc_state);
    Foc_UpdateCurrentLimits(&s_foc_state);
    /* 自检暂未实现；上电零偏采样在空闲且输出关闭时继续完成。 */
    if (MotorLifecycle_GetState() == MOTOR_STATE_IDLE)
    {
        (void)MotorCurrentOffsets_Calibrate(
            &s_current_offset_calibration, &s_motor_configuration, &s_motor_state);
    }
    if (s_current_offset_calibration.complete)
    {
        MotorCurrent_Update(&s_foc_state);
    }

    Clarke_Transform(&s_foc_state.phase_current_a, &s_foc_state.current_ab_a);

    /* 位置模式跳过观测器，随后完成编码器角度更新。 */
    if (s_current_offset_calibration.complete &&
        s_motor_state.control_mode != MOTOR_CONTROL_POSITION)
    {
        float modulation_to_voltage_v = s_foc_state.bus_voltage_filtered_v / 1.5f;
        const AlphaBetaVector voltage_ab_v = {s_foc_state.mod_ab.alpha * modulation_to_voltage_v,
                                              s_foc_state.mod_ab.beta * modulation_to_voltage_v};
        FluxObserver_Update(&s_motor_configuration.model,
                            &s_foc_state.current_ab_a,
                            &voltage_ab_v,
                            &s_flux_observer);
    }

    Encoder_Update(&s_rotor_encoder);
}

void MotorAxis_ControlFast(void)
{
    /* 采样阶段已完成编码器事务，再检查故障并处理本拍启相。 */
    MotorLifecycle_BeginFastTick();

    /* 捕获本拍代际；标定和正常控制互斥执行。 */
    uint32_t work_epoch = MotorLifecycle_WorkEpoch();
    MotorTaskResult outcome = {MOTOR_TASK_RUNNING, MOTOR_FAULT_NONE};
    if (s_current_offset_calibration.complete && s_motor_state.fault_code == MOTOR_FAULT_NONE)
    {
        MotorState state = MotorLifecycle_GetState();
        if (state == MOTOR_STATE_CALIBRATION && MotorLifecycle_IsCalibrationRunnable())
        {
            outcome = run_active_calibration_cycle();
        }
        else if (state == MOTOR_STATE_RUNNING)
        {
            outcome = MotorControl_Step(&s_motor_configuration,
                                        &s_foc_state,
                                        &s_motor_state,
                                        &s_rotor_encoder,
                                        &s_flux_observer);
        }
        else
        {
            BspPwm_SetDuty(0.5f, 0.5f, 0.5f);
        }
    }

    /* 所有分支统一发布，再关断本拍新产生的故障。 */
    MotorLifecycle_PostOutcome(outcome, work_epoch);
}

void MotorAxis_ProcessCalibrationPublication(void)
{
    /* 与候选发布共用最终临界区；STOP或故障到达后不能发布旧会话。 */
    uint32_t interrupt_mask = BspSystem_EnterCritical();
    uint32_t calibration_epoch;
    if (MotorLifecycle_ReadPendingCalibrationPublication(&calibration_epoch))
    {
        bool publication_succeeded =
            EncoderCalibration_PublishLut(&s_motor_configuration.device.encoder, &s_rotor_encoder);
        MotorLifecycle_CompleteCalibrationPublication(calibration_epoch, publication_succeeded);
    }
    BspSystem_ExitCritical(interrupt_mask);
}

MotorControlResult MotorAxis_SaveCalibration(void)
{
    if (s_motor_state.fault_code != MOTOR_FAULT_NONE)
    {
        return MOTOR_CONTROL_FAULT;
    }
    if (MotorLifecycle_GetState() != MOTOR_STATE_IDLE || !MotorLifecycle_IsStopped() ||
        !EncoderCalibration_IsIdle())
    {
        return MOTOR_CONTROL_BUSY;
    }
    uint8_t flags = s_motor_configuration.device.encoder.calib_flag;
    if (!s_current_offset_calibration.complete || s_current_offset_calibration.failed ||
        !MotorCurrentOffsets_IsValid(&s_motor_configuration) ||
        !ParameterValue_ConfigurationValid(&s_motor_configuration) || flags == 0U ||
        (flags & ~(ENC_CALIB_ALL | ENC_CALIB_MECHANICAL_ZERO)) != 0U ||
        s_motor_configuration.device.encoder.reverse > 1U)
    {
        return MOTOR_CONTROL_DENIED;
    }
    /* 前台串行，空闲且零偏完成后中断不改配置；不在整条CRC期间屏蔽中断。 */
    Param_Upload(&s_parameter_record, &s_motor_configuration);
    if (s_motor_state.fault_code != MOTOR_FAULT_NONE)
    {
        return MOTOR_CONTROL_FAULT;
    }
    if (MotorLifecycle_GetState() != MOTOR_STATE_IDLE || !MotorLifecycle_IsStopped())
    {
        return MOTOR_CONTROL_BUSY;
    }
    BspResult result = BspFlash_Write(&s_parameter_record, sizeof(s_parameter_record));
    if (result == BSP_OK)
    {
        return MOTOR_CONTROL_ACCEPTED;
    }
    return result == BSP_REJECTED ? MOTOR_CONTROL_DENIED : MOTOR_CONTROL_PERSIST_FAILED;
}

uint8_t MotorAxis_NodeId(void)
{
    return s_motor_configuration.device.node_id;
}
