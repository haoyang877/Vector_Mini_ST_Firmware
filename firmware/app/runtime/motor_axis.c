#include "motor_axis.h"
#include "application_config.h"
#include "bsp_encoder.h"
#include "bsp_adc.h"
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
#include "linear_hall.h"
#include <math.h>

/* 持有单轴对象并编排快慢环；全局符号保留给只读 Watch。 */
MotorConfiguration s_motor_configuration;
MotorControlState s_motor_state;
PiController s_speed_pi;
EncoderState s_rotor_encoder;
FluxObserverState s_flux_observer;
SensorlessStartupState s_sensorless_startup;
FocState s_foc_state;
CurrentOffsetCalibration s_current_offset_calibration;
LinearHall s_linear_hall;
OutputEncoder s_output_encoder;

const MotorAxisFeedback MotorAxis_Feedback = {
    &s_foc_state, &s_rotor_encoder, &s_motor_state, &s_linear_hall, &s_output_encoder};

/* 编码器线性化独占的6144字节静态工作区；发布或取消前不复用。 */
static EncoderCalibrationWorkspace s_encoder_linearization_workspace;
/* 启动装载和停机前台保存串行复用，避免再分配整条记录。 */
static MotorAxisRecord s_axis_record;
#if BSP_ENABLED_LINEAR_HALL
static LinearHallCalibration s_hall_sweep;
static OutputEncoderCalib s_output_calib;
#endif

static MotorTaskResult run_active_calibration_cycle(void)
{
    const EncoderCalibrationControl control = {&s_motor_configuration,
                                               &s_foc_state,
                                               &s_motor_state,
                                               &s_rotor_encoder,
                                               &s_flux_observer,
                                               &s_sensorless_startup};
    switch (MotorLifecycle_GetCalibrationType())
    {
    case MOTOR_CALIBRATION_ENCODER_LINEARIZATION:
        if (EncoderCalibration_GetPhase() == CS_NULL)
        {
            s_output_encoder.ready = false;
        }
#if BSP_ENABLED_LINEAR_HALL
        if (MotorAxisPreset_HasLinearHall(s_motor_configuration.preset) &&
            EncoderCalibration_GetPhase() == CS_NULL)
        {
            /* 新会话覆盖上次中止的极值；只在观测器闭环采样阶段收集。 */
            LinearHall_CalibrationBegin(&s_hall_sweep);
            OutputEncoder_CalibBegin(&s_output_calib, s_output_encoder.params.gear_ratio);
            s_output_encoder.ready = false;
        }
#endif
        return EncoderCalibration_StepLut(&control, &s_encoder_linearization_workspace);
    case MOTOR_CALIBRATION_JOINT_ELECTRICAL_ZERO:
    case MOTOR_CALIBRATION_ELECTRICAL_ZERO:
        return EncoderCalibration_StepElecZero(&control);
    case MOTOR_CALIBRATION_MECHANICAL_ZERO:
        if (!OutputEncoder_IsReady(&s_output_encoder))
        {
            return (MotorTaskResult){MOTOR_TASK_FAILED, MOTOR_FAULT_ENCODER};
        }
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
    if (BspFlash_Read(&s_axis_record, sizeof(s_axis_record)) == BSP_OK)
    {
        Param_Download(&s_axis_record.motor, sizeof(s_axis_record.motor), &s_motor_configuration);
    }
    else
    {
        Param_Return_Default(&s_motor_configuration);
    }
#if BSP_ENABLED_LINEAR_HALL
    LinearHall_Init(&s_linear_hall);
    if (s_motor_configuration.configuration_source == MOTOR_CONFIGURATION_STORED &&
        MotorAxisPreset_HasLinearHall(s_motor_configuration.preset))
    {
        Param_RestoreLinearHall(&s_axis_record.motor, &s_linear_hall);
    }
#endif
    float ratio = s_motor_configuration.preset != NULL
                      ? s_motor_configuration.preset->electrical->gear_ratio
                      : 1.0f;
    OutputEncoder_Init(&s_output_encoder, ratio);
    /* 直驱旧记录可直接沿用已有电机机械零位；减速轴须重新标定对应关系。 */
    if (ratio == 1.0f &&
        (s_motor_configuration.device.encoder.calib_flag & ENC_CALIB_MECHANICAL_ZERO) != 0U)
    {
        s_output_encoder.params.zero_rad =
            (float)s_motor_configuration.device.encoder.mechanical_zero_q15 *
            (6.283185307f / 65536.0f);
        s_output_encoder.params.status |= OUTPUT_ENCODER_ZERO_SET;
    }
    if (s_motor_configuration.configuration_source == MOTOR_CONFIGURATION_STORED)
    {
        (void)Param_RestoreOutputEncoder(&s_axis_record, &s_output_encoder);
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

    FluxObserver_InitConfigured(&s_flux_observer, &s_motor_configuration);
    if (s_motor_configuration.preset != NULL)
    {
        s_rotor_encoder.speed_pll_hz = s_motor_configuration.preset->electrical->speed_pll_hz;
    }
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
#if BSP_ENABLED_MCU_TEMPERATURE
    McuTemperature_Update(&s_foc_state, &s_motor_state);
#endif
    MotorLifecycle_SupervisorTick();
#if OUTER_LOOP_DIVIDER == SUPERVISOR_DIVIDER
    MotorAxis_OuterTick();
#endif
}

void MotorAxis_OuterTick(void)
{
    Encoder_UpdateSlowEstimate(&s_rotor_encoder);
#if BSP_ENABLED_LINEAR_HALL
    if (MotorAxisPreset_HasLinearHall(s_motor_configuration.preset))
    {
        float sin_counts = BspAdc_ReadRawChannel(BSP_ADC_LINEAR_HALL_SIN);
        float cos_counts = BspAdc_ReadRawChannel(BSP_ADC_LINEAR_HALL_COS);
        if (MotorLifecycle_GetCalibrationType() == MOTOR_CALIBRATION_ENCODER_LINEARIZATION &&
            EncoderCalibration_GetPhase() == CS_OBS_SAMPLE_CW)
        {
            LinearHall_CalibrationSample(&s_hall_sweep, sin_counts, cos_counts);
            OutputEncoder_CalibSample(&s_output_calib,
                                      s_rotor_encoder.directed_q15,
                                      (uint16_t)sin_counts,
                                      (uint16_t)cos_counts);
        }
        LinearHall_Update(&s_linear_hall, sin_counts, cos_counts);
    }
#endif
    /* 等待现有上电采样窗口结束，避免传感器启动初值参与首次辨圈。 */
    if (s_current_offset_calibration.complete && Encoder_IsOnline(&s_rotor_encoder) &&
        (s_motor_configuration.device.encoder.calib_flag & ENC_CALIB_LINEARIZED) != 0U &&
        (s_output_encoder.params.status & OUTPUT_ENCODER_CAL_OK) != 0U &&
        MotorLifecycle_GetState() != MOTOR_STATE_CALIBRATION)
    {
        uint16_t motor_q15 = s_rotor_encoder.linearized_q15;
        if (OutputEncoder_IsReady(&s_output_encoder))
        {
            OutputEncoder_Update(&s_output_encoder, motor_q15);
        }
        else if (s_output_encoder.params.gear_ratio == 1.0f ||
                 (s_linear_hall.calibration_success && isfinite(s_linear_hall.val.sin_cal) &&
                  isfinite(s_linear_hall.val.cos_cal) &&
                  (s_linear_hall.val.sin_cal != 0.0f || s_linear_hall.val.cos_cal != 0.0f)))
        {
            OutputEncoder_Locate(&s_output_encoder, motor_q15, s_linear_hall.val.angle_rad);
        }
    }
    MotorLifecycle_OuterTick();
}

void MotorAxis_OuterOverrun(void)
{
    MotorFault_Report(&s_motor_state, MOTOR_FAULT_CONTROL_OVERRUN);
    BspPwm_Disable();
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

    /* 编码器闭环使用实测角；标定及无感控制仍更新磁链观测器。 */
    if (s_current_offset_calibration.complete &&
        s_motor_state.control_mode != MOTOR_CONTROL_POSITION &&
        s_motor_state.control_mode != MOTOR_CONTROL_SPEED)
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

void MotorAxis_ProcessPositionPlan(void)
{
    static MotorPositionPlan work;
    uint32_t mask = BspSystem_EnterControlCritical();
    bool pending = MotorOuterLoop_TakePositionPlan(&work);
    BspSystem_ExitControlCritical(mask);
    if (pending)
    {
        bool valid = MotorOuterLoop_BuildPositionPlan(&work);
        mask = BspSystem_EnterControlCritical();
        MotorOuterLoop_PublishPositionPlan(&work, valid);
        BspSystem_ExitControlCritical(mask);
    }
}

void MotorAxis_ProcessCalibrationPublication(void)
{
    uint32_t mask = BspSystem_EnterCritical();
    uint32_t epoch;
    if (!MotorLifecycle_ReadPendingCalibrationPublication(&epoch))
    {
        BspSystem_ExitCritical(mask);
        return;
    }
    bool lut = MotorLifecycle_GetCalibrationType() == MOTOR_CALIBRATION_ENCODER_LINEARIZATION;
#if BSP_ENABLED_LINEAR_HALL
    if (lut && MotorAxisPreset_HasLinearHall(s_motor_configuration.preset))
    {
        /* 保持关相直至停稳；重复前台轮询不阻塞控制中断。 */
        static uint32_t coast_epoch, coast_start_ms, stable_since_ms;
        uint32_t now = BspSystem_TimeMs();
        if (epoch != coast_epoch)
        {
            coast_epoch = epoch;
            coast_start_ms = stable_since_ms = now;
        }
        if (!(fabsf(Encoder_GetMechVelContinuous(&s_rotor_encoder)) < 1.0f))
        {
            stable_since_ms = now;
        }
        if (now - stable_since_ms < 200U)
        {
            if (now - coast_start_ms >= 6000U)
            {
                MotorLifecycle_CompleteCalibrationPublication(epoch, false);
            }
            BspSystem_ExitCritical(mask);
            return;
        }
        if (EncoderCalibration_PublishLut(&s_motor_configuration.device.encoder, &s_rotor_encoder))
        {
            OutputEncoder_Init(&s_output_encoder, s_output_encoder.params.gear_ratio);
            MotorLifecycle_ContinueElectricalZero();
        }
        else
        {
            MotorLifecycle_CompleteCalibrationPublication(epoch, false);
        }
        BspSystem_ExitCritical(mask);
        return;
    }
#endif
    if (lut &&
        !EncoderCalibration_PublishLut(&s_motor_configuration.device.encoder, &s_rotor_encoder))
    {
        MotorLifecycle_CompleteCalibrationPublication(epoch, false);
        BspSystem_ExitCritical(mask);
        return;
    }
    OutputEncoder output = s_output_encoder;
    MotorCalibrationType kind = MotorLifecycle_GetCalibrationType();
    MotorFaultCode fault = MOTOR_FAULT_NONE;
    if (lut)
    {
        OutputEncoder_Init(&output, output.params.gear_ratio);
    }
    if (kind == MOTOR_CALIBRATION_MECHANICAL_ZERO)
    {
        OutputEncoder_Update(&output, s_rotor_encoder.linearized_q15);
        OutputEncoder_SetZero(&output);
    }
#if BSP_ENABLED_LINEAR_HALL
    LinearHall hall = s_linear_hall;
    if (kind == MOTOR_CALIBRATION_JOINT_ELECTRICAL_ZERO &&
        !LinearHall_CalibrationFinish(&s_hall_sweep, &hall))
    {
        fault = MOTOR_FAULT_LINEAR_HALL_CALIBRATION;
    }
#endif
    BspSystem_ExitCritical(mask);
#if BSP_ENABLED_LINEAR_HALL
    /* 三遍有限样本拟合允许控制中断抢占；提交前仍须复核原会话。 */
    if (kind == MOTOR_CALIBRATION_JOINT_ELECTRICAL_ZERO &&
        !OutputEncoder_CalibEnd(
            &output, &s_output_calib, &hall, &s_motor_configuration.device.encoder) &&
        fault == MOTOR_FAULT_NONE)
    {
        fault = MOTOR_FAULT_ENCODER;
    }
#endif
    Param_Upload(&s_axis_record.motor, &s_motor_configuration);
#if BSP_ENABLED_LINEAR_HALL
    if (MotorAxisPreset_HasLinearHall(s_motor_configuration.preset))
    {
        Param_StoreLinearHall(&s_axis_record.motor, &hall);
    }
#endif
    Param_StoreOutputEncoder(&s_axis_record, &output);
    mask = BspSystem_EnterCritical();
    uint32_t pending_epoch;
    if (MotorLifecycle_ReadPendingCalibrationPublication(&pending_epoch) && pending_epoch == epoch)
    {
        s_output_encoder = output;
#if BSP_ENABLED_LINEAR_HALL
        s_linear_hall = hall;
#endif
        BspResult saved = BspFlash_Write(&s_axis_record, sizeof(s_axis_record));
        if (saved != BSP_OK || fault != MOTOR_FAULT_NONE)
        {
            MotorFault_Report(&s_motor_state,
                              saved != BSP_OK ? MOTOR_FAULT_CALIBRATION_STORAGE : fault);
        }
        else
        {
            MotorLifecycle_CompleteCalibrationPublication(epoch, true);
        }
    }
    BspSystem_ExitCritical(mask);
}

uint8_t MotorAxis_NodeId(void)
{
    return s_motor_configuration.device.node_id;
}
