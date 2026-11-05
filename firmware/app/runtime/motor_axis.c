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
#include "control_math.h"
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

const MotorAxisFeedback MotorAxis_Feedback = {
    &s_foc_state, &s_rotor_encoder, &s_motor_state, &s_linear_hall};

/* 编码器线性化独占的6144字节静态工作区；发布或取消前不复用。 */
static EncoderCalibrationWorkspace s_encoder_linearization_workspace;
/* 启动装载和停机前台保存串行复用，避免再分配整条记录。 */
static InterfaceParam s_parameter_record;
#if BSP_ENABLED_LINEAR_HALL
static LinearHallCalibration s_hall_sweep;
static float s_hall_start_rad, s_hall_field_rad;
static uint32_t s_hall_ticks;

/* 电零位对齐结束时转子位于零电角；以限流慢转磁场扫过实测一整输出圈。 */
static MotorTaskResult step_hall(const EncoderCalibrationControl *drive)
{
    if (!s_hall_sweep.active)
    {
        MotorTaskResult result = EncoderCalibration_StepElecZero(drive);
        if (result.result == MOTOR_TASK_COMPLETED)
        {
            LinearHall_CalibrationBegin(&s_hall_sweep);
            s_hall_start_rad = Encoder_GetMechPos(drive->encoder);
            s_hall_ticks = 0U;
            s_hall_field_rad = 0.0f;
            return (MotorTaskResult){MOTOR_TASK_RUNNING, MOTOR_FAULT_NONE};
        }
        return result;
    }

    const float travel = Encoder_GetMechPos(drive->encoder) - s_hall_start_rad;
    const float target = _2PI * drive->configuration->preset->electrical->gear_ratio * 1.01f;
    const float elapsed = (float)++s_hall_ticks * FOC_TS_S;
    if (!isfinite(travel) || travel < -0.5f || elapsed > target / 2.0f + 3.0f)
    {
        return (MotorTaskResult){MOTOR_TASK_FAILED, MOTOR_FAULT_ENCODER};
    }
    if (travel >= target)
    {
        return (MotorTaskResult){MOTOR_TASK_COMPLETED, MOTOR_FAULT_NONE};
    }
    /* 电机轴最高 5 rad/s，起步和接近末端均降速。 */
    const float speed = Math_Min(Math_Min(5.0f, 5.0f * elapsed), 1.0f + 2.0f * (target - travel));
    const float elec_speed = speed * (float)drive->configuration->model.motor_pole_pairs;
    s_hall_field_rad += elec_speed * FOC_TS_S;
    if (s_hall_field_rad >= _2PI)
    {
        s_hall_field_rad -= _2PI;
    }
    drive->motor->id_target_a = Math_Min(drive->configuration->model.calibration_current_a,
                                         drive->configuration->application.current_limit_a);
    drive->motor->iq_target_a = 0.0f;
    Foc_RunCurrent(drive->configuration, drive->foc, drive->motor, s_hall_field_rad, elec_speed);
    return (MotorTaskResult){MOTOR_TASK_RUNNING, MOTOR_FAULT_NONE};
}
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
        return EncoderCalibration_StepLut(&control, &s_encoder_linearization_workspace);
#if BSP_ENABLED_LINEAR_HALL
    case MOTOR_CALIBRATION_LINEAR_HALL:
        return step_hall(&control);
#endif
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
#if BSP_ENABLED_LINEAR_HALL
    LinearHall_Init(&s_linear_hall);
    if (s_motor_configuration.configuration_source == MOTOR_CONFIGURATION_STORED &&
        MotorAxisPreset_HasLinearHall(s_motor_configuration.preset))
    {
        Param_RestoreLinearHall(&s_parameter_record, &s_linear_hall);
    }
#endif
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
        if (MotorLifecycle_GetCalibrationType() == MOTOR_CALIBRATION_LINEAR_HALL &&
            s_hall_sweep.active && MotorLifecycle_IsCalibrationRunnable())
        {
            LinearHall_CalibrationSample(&s_hall_sweep, sin_counts, cos_counts);
        }
        LinearHall_Update(&s_linear_hall, sin_counts, cos_counts);
    }
#endif
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

static bool can_save_calibration(const uint32_t *epoch)
{
    if (epoch == NULL)
    {
        return MotorLifecycle_GetState() == MOTOR_STATE_IDLE && MotorLifecycle_IsStopped();
    }
    uint32_t pending;
    return MotorLifecycle_ReadPendingCalibrationPublication(&pending) && pending == *epoch;
}

static MotorControlResult save_calibration(const uint32_t *epoch)
{
    if (s_motor_state.fault_code != MOTOR_FAULT_NONE)
    {
        return MOTOR_CONTROL_FAULT;
    }
    if (!can_save_calibration(epoch) || !EncoderCalibration_IsIdle())
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
#if BSP_ENABLED_LINEAR_HALL
    if (MotorAxisPreset_HasLinearHall(s_motor_configuration.preset))
    {
        Param_StoreLinearHall(&s_parameter_record, &s_linear_hall);
    }
#endif
    uint32_t mask = BspSystem_EnterCritical();
    if (s_motor_state.fault_code != MOTOR_FAULT_NONE)
    {
        BspSystem_ExitCritical(mask);
        return MOTOR_CONTROL_FAULT;
    }
    if (!can_save_calibration(epoch))
    {
        BspSystem_ExitCritical(mask);
        return MOTOR_CONTROL_BUSY;
    }
    BspResult result = BspFlash_Write(&s_parameter_record, sizeof(s_parameter_record));
    BspSystem_ExitCritical(mask);
    if (result == BSP_OK)
    {
        return MOTOR_CONTROL_ACCEPTED;
    }
    return result == BSP_REJECTED ? MOTOR_CONTROL_DENIED : MOTOR_CONTROL_PERSIST_FAILED;
}

MotorControlResult MotorAxis_SaveCalibration(void)
{
    return save_calibration(NULL);
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
        s_hall_sweep.active = false;
        if (!EncoderCalibration_PublishLut(&s_motor_configuration.device.encoder,
                                           &s_rotor_encoder) ||
            !MotorLifecycle_ContinueLinearHall(epoch))
        {
            MotorLifecycle_CompleteCalibrationPublication(epoch, false);
        }
        BspSystem_ExitCritical(mask);
        return;
    }
#endif
    bool published = !lut || EncoderCalibration_PublishLut(&s_motor_configuration.device.encoder,
                                                           &s_rotor_encoder);
#if BSP_ENABLED_LINEAR_HALL
    if (published && MotorAxisPreset_HasLinearHall(s_motor_configuration.preset))
    {
        bool hall_ok = MotorLifecycle_GetCalibrationType() != MOTOR_CALIBRATION_LINEAR_HALL ||
                       LinearHall_CalibrationFinish(&s_hall_sweep, &s_linear_hall);
        BspSystem_ExitCritical(mask);
        /* 先在前台编码，写前再复核会话和关相状态。完成通知在 Flash 回读之后。 */
        MotorControlResult saved = save_calibration(&epoch);
        mask = BspSystem_EnterCritical();
        if (can_save_calibration(&epoch))
        {
            if (saved != MOTOR_CONTROL_ACCEPTED || !hall_ok)
            {
                MotorFault_Report(&s_motor_state,
                                  saved != MOTOR_CONTROL_ACCEPTED
                                      ? MOTOR_FAULT_CALIBRATION_STORAGE
                                      : MOTOR_FAULT_LINEAR_HALL_CALIBRATION);
            }
            else
            {
                MotorLifecycle_CompleteCalibrationPublication(epoch, true);
            }
        }
    }
    else
#endif
    {
        MotorLifecycle_CompleteCalibrationPublication(epoch, published);
    }
    BspSystem_ExitCritical(mask);
}

uint8_t MotorAxis_NodeId(void)
{
    return s_motor_configuration.device.node_id;
}
