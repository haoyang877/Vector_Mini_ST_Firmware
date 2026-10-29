#include "pwm_output_config.h"
#ifndef APP_ENABLED_DIAGNOSTIC_TELEMETRY
#define APP_ENABLED_DIAGNOSTIC_TELEMETRY 0
#endif
#if APP_ENABLED_DIAGNOSTIC_TELEMETRY
#include "app_telemetry.h"
#endif
#if BSP_PWM_PERMIT_MODE == BSP_PWM_PERMIT_WINDOW
#include "pwm_power_window.h"
#endif
#include "motor_axis.h"
#include <math.h>
#include "bsp_system.h"
#include "motor_maintenance.h"
#include "bsp_pwm.h"
#include "foc_sensing.h"
#include "motor_parameter_transaction.h"
#include "motor_thermal_control.h"
#include "application_profile_defaults.h"

#include "angle_feedback.h"
#include "foc_algorithm.h"
#include "motor_fault.h"
#include "foc_pid.h"
#include "motor_control.h"
#include "motor_state_machine.h"
#include "flux_observer.h"
#include "sensorless_startup.h"
#include "fast_loop_profile.h"

/* 电机与控制对象全局符号供 Watch 观测；运行写入仍由 MotorAxis 组装和调度。 */
MotorConfiguration s_motor_configuration;
MotorControlState s_motor_state;
PiController s_speed_pi;
EncoderState s_rotor_encoder;
FluxObserverState s_flux_observer;
SensorlessStartupState s_sensorless_startup;

FocState s_foc_state;

/* Axis持有并显式借给采样模块；全局可见性保留既有只读Watch观测。 */
CurrentOffsetCalibration s_current_offset_calibration;

/* 快/慢ISR写完状态后推进；前台读者不能抢占写者，前台配置操作串行。 */
static volatile uint32_t s_status_revision;

uint8_t MotorAxis_NodeId(void)
{
    return s_motor_configuration.device.node_id;
}

/* 仅在配置装配/提交边界绑定固定参数；保留积分与动态限幅，重放请求不会放宽热额度。 */
static void configure_controllers(void)
{
    FOC_ConfigureControllers(&s_motor_configuration, &s_foc_state, &s_speed_pi);
}

/* 每个新鲜母线样本后只刷新电压额度，不改变增益、时基或积分状态；空闲也有效。 */
static void update_current_controller_limits(void)
{
    FOC_UpdateCurrentControllerLimits(&s_foc_state);
}

/* 不把2048或上次存储零点当作本次上电校正资格；NAN阻止原自检/启动门禁通过。 */
static void initialize_current_offsets(void)
{
    MotorCurrentOffsets_Init(&s_current_offset_calibration, &s_motor_configuration, &s_foc_state);
}

/* 只能由新鲜JEOS快速拍调用；不启相、不分配、不阻塞、不保存、不自动重试。 */
static void sample_phase_currents(const BspAdcFastSample *sample)
{
    if (s_current_offset_calibration.complete)
    {
        Current_Cal(&s_motor_configuration, &s_foc_state, &s_motor_state, sample);
        return;
    }
    (void)MotorCurrentOffsets_Sample(&s_current_offset_calibration, &s_motor_state);
}

/* 一次性求均值/发布不占用第20000拍的快中断预算；发布前保持INIT和无效电流。 */
static void finalize_current_offsets(void)
{
    if (MotorCurrentOffsets_Finalize(
            &s_current_offset_calibration, &s_motor_configuration, &s_motor_state))
    {
        MotorParameterTransaction_AdvanceRevision();
    }
}

void MotorAxis_Init(void)
{
    s_status_revision = 0U;
    MotorParameterTransaction_Init(&s_motor_configuration);
    if (s_motor_configuration.configuration_source != MOTOR_CONFIGURATION_STORED)
    {
        s_motor_state.vq_target_v = 0.0f;
    }
    if (!MotorThermalControl_Init(&s_motor_configuration.model.thermal, BspSystem_TimeMs()))
    {
        MotorFault_Report(&s_motor_state, MOTOR_FAULT_INVALID_PARAMETERS);
    }
    s_motor_state.thermal_current_limit_a =
        MotorThermalControl_CurrentLimit(s_motor_configuration.application.current_limit_a);
    s_motor_state.thermal_clear_ready = MotorThermalControl_CanClearFault();
    initialize_current_offsets();
    MotorStateMachine_Init(&s_motor_configuration,
                           &s_motor_state,
                           &s_foc_state,
                           &s_speed_pi,
                           &s_rotor_encoder,
                           &s_sensorless_startup);
    Encoder_ParamInit(&s_motor_configuration.device.encoder, &s_rotor_encoder);

    FluxObserver_ParamInit(&s_flux_observer);
    SensorlessStartup_Reset(&s_sensorless_startup);
    FOC_CurrentController_Reset(&s_foc_state);
    PiController_Reset(&s_speed_pi);
    configure_controllers();
    update_current_controller_limits();
    s_speed_pi.output_min = -1.0f;
    s_speed_pi.output_max = 1.0f;

    s_motor_configuration.application.position_error_window_rad =
        PARAM_APP_POSITION_ERROR_WINDOW_RAD;
    s_motor_state.position_velocity_filtered_rad_s = 0.0f;
    MotorOuterLoop_RequestReset();

    /* 未配置轴记录的关节在启动后保持禁用。 */
    /* 上电零点校正和基础自检只在关相状态推进，完成也不自动使能电机。 */
    s_motor_state.control_mode = MOTOR_CONTROL_NONE;
    if (!s_motor_configuration.application.axis_profile_valid)
    {
        MotorFault_Report(&s_motor_state, MOTOR_FAULT_INVALID_PARAMETERS);
    }
}

MotorControlMode MotorAxis_SupervisorTick(void)
{
    finalize_current_offsets();
    /* ADC快环丢失时仍由监督关断；只发布时钟失效事件，不重置热历史。 */
#if APP_ENABLED_WINDING_THERMAL_MODEL
    if (!MotorThermalControl_CheckWatchdog(BspSystem_TimeMs(), BspPwm_ArePhasesDisabled()))
    {
        s_motor_state.thermal_current_limit_a = 0.0f;
        s_motor_state.thermal_clear_ready = false;
        MotorFault_Report(&s_motor_state, MOTOR_FAULT_WINDING_THERMAL);
        MotorStateMachine_FastFaultStop();
    }
#endif
    /* 1. 编码器慢估计：多圈/机械角/速度是慢状态唯一写者。 */
    Encoder_UpdateSlowEstimate(&s_motor_configuration, &s_rotor_encoder);

    /* 2. 本拍温度事实先发布，状态/故障在正常控制计算前处理。 */
    Temperature_Update(&s_foc_state, &s_motor_state);
    MotorSupervisorStep step;
    if (MotorStateMachine_PrepareSupervisor(&step))
    {
        /* 算法及求值顺序不变；结果和PI仅写入本拍副本。 */
        MotorOuterLoop_SlowTick(
            &s_motor_configuration, &step.motor, &step.speed_pi, &s_rotor_encoder);
        step.calculated = true;
    }
    /* 3. 重读故障/STOP/代际及ready，最后才提交引用和启相。 */
    MotorStateMachine_CommitSupervisor(&step);
    ++s_status_revision;
    return s_motor_state.control_mode;
}
/* 热采样使用本拍实际三相电流；限流只是运行快照，绝不覆盖活动参数。 */
static void update_winding_thermal(void)
{
    bool outputs_disabled = BspPwm_ArePhasesDisabled();
#if APP_ENABLED_WINDING_THERMAL_MODEL
    if (s_current_offset_calibration.complete)
    {
        MotorThermalControl_Sample(s_foc_state.phase_a_current_a,
                                   s_foc_state.phase_b_current_a,
                                   s_foc_state.phase_c_current_a,
                                   outputs_disabled,
                                   BspSystem_TimeMs());
    }
    MotorMaintenanceKind kind = MotorStateMachine_GetMaintenanceKind();
    bool calibration =
        kind == MOTOR_MAINTENANCE_ENCODER_LUT || kind == MOTOR_MAINTENANCE_ELECTRICAL_ZERO;
    bool derating_allowed = !calibration && !MotorControlMode_IsVoltage(s_motor_state.control_mode);
    /* 保留原短路条件：模型允许时才查询功率事实；额外CanRun只读，不推进热历史。 */
    bool output_expected = s_current_offset_calibration.complete &&
                           MotorThermalControl_CanRun(true) && MotorStateMachine_OutputExpected();
    MotorThermalControl_UpdateProtection(&s_motor_configuration,
                                         &s_foc_state,
                                         &s_motor_state,
                                         s_current_offset_calibration.complete,
                                         outputs_disabled,
                                         output_expected,
                                         derating_allowed);
#else
    MotorThermalControl_UpdateProtection(&s_motor_configuration,
                                         &s_foc_state,
                                         &s_motor_state,
                                         s_current_offset_calibration.complete,
                                         outputs_disabled,
                                         false,
                                         false);
#endif
}

void MotorAxis_FastTick(void)
{
    /* 1. 取得本拍编码器事务，底层传输与后续计算重叠。 */
    FAST_PROFILE_BEGIN(FAST_PROFILE_ENCODER_REQUEST);
    uint8_t encoder_stage = Encoder_BeginSample();
    FAST_PROFILE_END(FAST_PROFILE_ENCODER_REQUEST);

    /* 2. 电压和电流共用一次ADC快照；失败时传入NULL，继续原保护与收尾。 */
    FAST_PROFILE_BEGIN(FAST_PROFILE_SENSING);
    BspAdcFastSample sample;
    const BspAdcFastSample *valid_sample = NULL;
    if (BspAdc_ReadFast(&sample) == BSP_OK)
    {
        valid_sample = &sample;
    }
    Vbus_Update(&s_foc_state, &s_motor_state, MotorStateMachine_OutputExpected(), valid_sample);
    update_current_controller_limits();
    sample_phase_currents(valid_sample);
    encoder_stage = Encoder_BeginReadback(encoder_stage != 0U);
    update_winding_thermal();

    /* 3. 读回与热保护/观测器计算重叠，之后完成角度更新；位置模式跳过观测器。 */
    if (s_current_offset_calibration.complete &&
        s_motor_state.control_mode != MOTOR_CONTROL_POSITION)
    {
        FluxObserver_Update(&s_motor_configuration, &s_foc_state, &s_flux_observer);
    }

    FAST_PROFILE_END(FAST_PROFILE_SENSING);
    FAST_PROFILE_BEGIN(FAST_PROFILE_ENCODER);
    /* SAVE可能暂停采样，重建测速以免将停顿中的位移按50us换算。 */
    if (MotorStateMachine_GetMaintenanceKind() == MOTOR_MAINTENANCE_SAVE)
    {
        Encoder_ResetVelocity(&s_rotor_encoder);
    }
    Encoder_CompleteSample(&s_motor_configuration, &s_rotor_encoder, encoder_stage);
    FAST_PROFILE_END(FAST_PROFILE_ENCODER);

    /* 4. 先完成编码器事务，再检查故障并立即关断。 */
    MotorStateMachine_CheckFastFaults();
    MotorStateMachine_FastFaultStop();
    MotorStateMachine_BeginFastTick();

    /* 5. 执行前捕获代际和维护许可，分发维护、控制或空闲预载。 */
    uint32_t work_epoch = MotorStateMachine_WorkEpoch();
    bool maintenance_runnable = MotorStateMachine_MaintenanceRunnable();
    MotorTaskResult outcome = {MOTOR_TASK_RUNNING, MOTOR_FAULT_NONE};
    MotorState state;
    if (!s_current_offset_calibration.complete || s_motor_state.fault_code != MOTOR_FAULT_NONE)
    {
        goto publish;
    }
    if (maintenance_runnable)
    {
        outcome = MotorMaintenance_Step(&s_motor_configuration,
                                        MotorStateMachine_GetMaintenanceKind(),
                                        &s_foc_state,
                                        &s_motor_state,
                                        &s_speed_pi,
                                        &s_rotor_encoder,
                                        &s_flux_observer,
                                        &s_sensorless_startup);
        goto publish;
    }
    state = MotorStateMachine_GetState();
    if (state != MOTOR_STATE_STARTING && state != MOTOR_STATE_RUNNING)
    {
        if (BspPwm_SetDuty(0.5f, 0.5f, 0.5f) != BSP_OK)
        {
            outcome = (MotorTaskResult){MOTOR_TASK_FAILED, MOTOR_FAULT_CONTROL_OVERRUN};
        }
        goto publish;
    }
    /* 普通路径只有模式选择和一条共用FOC，不进入维护/生命周期分发。 */
    outcome = MotorControl_Step(
        &s_motor_configuration, &s_foc_state, &s_motor_state, &s_rotor_encoder, &s_flux_observer);

publish:
    /* 6. 发布维护修订和任务结果，再关断本拍执行中新产生的故障。 */
    if (maintenance_runnable && s_motor_state.fault_code == MOTOR_FAULT_NONE &&
        outcome.result == MOTOR_TASK_COMPLETED)
    {
        if (MotorStateMachine_GetMaintenanceKind() == MOTOR_MAINTENANCE_DEFAULTS)
        {
            configure_controllers();
        }
        MotorParameterTransaction_AdvanceMaintenanceRevision(work_epoch);
    }
    MotorStateMachine_PostOutcome(outcome, work_epoch);
    MotorStateMachine_FastFaultStop();
    ++s_status_revision;
}

static MotorState read_status_snapshot(MotorStatus *sample,
                                       uint8_t *valid_bits,
                                       MotorMaintenanceKind *maintenance_kind,
                                       MotorControlProgress *control_progress)
{
    sample->fault = (uint16_t)s_motor_state.fault_code;
    sample->mode = (uint16_t)s_motor_state.control_mode;
    sample->position_target = s_motor_state.position_target_rad;
    sample->position_feedback = Encoder_GetMecPos(&s_rotor_encoder);
    sample->speed_target = s_motor_state.speed_target_rad_s;
    sample->speed_feedback_rad_s = (s_motor_state.control_mode == MOTOR_CONTROL_POSITION)
                                       ? s_motor_state.position_velocity_filtered_rad_s
                                       : Encoder_GetMecVel(&s_rotor_encoder);
    sample->current_reference = s_motor_state.iq_target_a;
    sample->current_feedback = s_foc_state.iq_a;
    sample->temperature = s_foc_state.temperature_c;
    sample->bus_voltage = s_foc_state.bus_voltage_filtered_v;
    sample->bus_current = s_foc_state.bus_current_filtered_a;
    sample->position_planned = NAN;
    sample->speed_planned = NAN;
    *valid_bits = 0x54U;
    if (s_motor_configuration.application.axis_profile_valid &&
        Encoder_GetBadFrameStreak(&s_rotor_encoder) == 0U &&
        Encoder_GetCalibFlag(&s_motor_configuration.device.encoder) != 0U)
    {
        *valid_bits |= 0x03U;
    }
    if (MotorTemperature_IsFresh())
    {
        *valid_bits |= 0x20U;
    }
    *maintenance_kind = MotorStateMachine_GetMaintenanceKind();
    MotorStateMachine_ReadControlProgress(control_progress);
    MotorState state = MotorStateMachine_GetState();
    return state;
}

MotorState MotorAxis_ReadStatus(MotorStatus *sample,
                                uint8_t *valid_bits,
                                MotorMaintenanceKind *maintenance_kind,
                                MotorControlProgress *control_progress)
{
    /* 锁只保护版本观察并约束编译器顺序，采集期间允许ISR完整发布下一版。 */
    for (unsigned attempt = 0U; attempt < 3U; ++attempt)
    {
        uint32_t key = BspSystem_EnterControlCritical();
        uint32_t revision = s_status_revision;
        BspSystem_ExitControlCritical(key);
        MotorState state =
            read_status_snapshot(sample, valid_bits, maintenance_kind, control_progress);
        key = BspSystem_EnterControlCritical();
        bool coherent = revision == s_status_revision;
        BspSystem_ExitControlCritical(key);
        if (coherent)
        {
            return state;
        }
    }
    /* 持续抢占时不冒充一致测量，也不阻塞快环；主机通过有效位识别本帧。 */
    *valid_bits = 0U;
    return MotorStateMachine_GetState();
}

void MotorAxis_ServiceBackground(void)
{
    FocBackgroundClaim claim;
    if (!MotorStateMachine_BeginBackground(&claim))
    {
        return;
    }
    if (claim.kind == FOC_BACKGROUND_LUT_PUBLISH)
    {
        /* 关相后原子发布，避免快环读取半张表；STOP/故障先于写入复核。 */
        uint32_t mask = BspSystem_EnterControlCritical();
        bool published = MotorStateMachine_BackgroundValid(&claim) &&
                         MotorMaintenance_PublishLut(&s_motor_configuration, &s_rotor_encoder);
        if (published)
        {
            MotorParameterTransaction_AdvanceMaintenanceRevision(claim.epoch);
        }
        else
        {
            MotorMaintenance_Cancel(MOTOR_MAINTENANCE_ENCODER_LUT);
        }
        MotorStateMachine_FinishBackground(&claim, published);
        BspSystem_ExitControlCritical(mask);
        return;
    }
    bool success = false;
    if (claim.kind == FOC_BACKGROUND_SAVE)
    {
        success = MotorParameterTransaction_SaveRecord(&s_motor_configuration, &claim);
    }
    uint32_t mask = BspSystem_EnterControlCritical();
    bool current = MotorStateMachine_BackgroundValid(&claim);
    MotorStateMachine_FinishBackground(&claim, success);
    if (claim.kind == FOC_BACKGROUND_SAVE)
    {
        MotorParameterTransaction_FinishSave(claim.epoch, current, success);
    }
    BspSystem_ExitControlCritical(mask);
}

/* 外部参数入口保留；事务状态与资格判断仅由参数事务模块推进。 */
ParameterStatus MotorAxis_ReadParameter(const ParameterReadRequest *request, ParameterValue *value)
{
    return MotorParameterTransaction_Read(&s_motor_configuration, request, value);
}

ParameterStatus MotorAxis_WriteParameter(const ParameterWriteRequest *request,
                                         ParameterValue *applied)
{
    uint32_t mask = BspSystem_EnterControlCritical();
    ParameterStatus result = MotorParameterTransaction_Write(
        &s_motor_configuration, &s_motor_state, &s_foc_state, &s_speed_pi, request, applied);
    if (result == PARAMETER_OK)
    {
        configure_controllers();
    }
    BspSystem_ExitControlCritical(mask);
    return result;
}

ParameterStatus MotorAxis_RequestSave(const ParameterSaveRequest *request, uint32_t *ticket)
{
    return MotorParameterTransaction_RequestSave(request, ticket);
}

ParameterStatus MotorAxis_PollSave(uint32_t ticket, uint32_t *persisted_revision)
{
    return MotorParameterTransaction_PollSave(ticket, persisted_revision);
}

bool MotorAxis_SaveIdle(void)
{
    return MotorParameterTransaction_SaveIdle();
}

bool MotorAxis_ConfigurationRevisionAvailable(void)
{
    return MotorParameterTransaction_RevisionAvailable();
}

void MotorAxis_ServiceFastDiagnostics(uint32_t fast_samples)
{
#if BSP_PWM_PERMIT_MODE == BSP_PWM_PERMIT_WINDOW
    if (g_pwm_power_window.state == PWM_POWER_ACTIVE &&
        fabsf(s_rotor_encoder.mechanical_velocity_continuous_rad_s) > 60.0f)
    {
        PwmPowerWindow_Abort();
        BspPwm_Disable();
    }
#endif
#if APP_ENABLED_DIAGNOSTIC_TELEMETRY
    R40AppTelemetry_Capture(fast_samples,
                            &s_foc_state,
                            &s_motor_state,
                            &s_rotor_encoder,
                            MotorStateMachine_GetState(),
                            s_motor_configuration.device.encoder.calib_flag,
                            MotorStateMachine_GetMaintenanceKind());
#else
    (void)fast_samples;
#endif
}
