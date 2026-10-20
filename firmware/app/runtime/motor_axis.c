#include "motor_axis.h"
#include <math.h>
#include "critical_hw.h"
#include "motor_maintenance.h"
#include "motor_hw.h"
#include "motor_sensing.h"
#include "power_stage_hw.h"
#include "foc_sensing.h"
#include "motor_parameter_transaction.h"
#include "motor_thermal_control.h"
#include "time_hw.h"
#include "application_profile_defaults.h"

#include "angle_feedback.h"
#include "foc_algorithm.h"
#include "foc_errhandle.h"
#include "foc_pid.h"
#include "motor_execution.h"
#include "motor_state_machine.h"
#include "foc_sensorless.h"
#include "foc_sensorless_run.h"

/* 电机与控制对象全局符号供 Watch 观测；运行写入仍由 MotorAxis 组装和调度。 */
MotorConfiguration s_motor_configuration;
MotorControlState s_motor_state;
PiController s_speed_pi;
EncoderState s_rotor_encoder;
FluxObserverState s_flux_observer;
SensorlessStartupState s_sensorless_startup;

FocState s_foc_state;

/* JEOS每帧推进一次；20kHz下累计1秒，最大四次和的总和仍可由uint32_t容纳。 */
#define CURRENT_OFFSET_SAMPLE_COUNT (FOC_FREQ * MOTOR_CURRENT_OFFSET_SAMPLE_TIME_MS / 1000U)
typedef struct
{
    uint32_t phase_sum_raw[3];
    uint32_t sample_count;
    bool complete, failed;
} CurrentOffsetCalibration;

/* 仅MotorAxis写入；全局可见性用于与既有运行对象一致的只读Watch观测。 */
CurrentOffsetCalibration s_current_offset_calibration;

uint8_t MotorAxis_NodeId(void)
{
    return s_motor_configuration.device.node_id;
}

/* 仅在配置装配/提交边界绑定固定参数；保留积分与动态限幅，重放请求不会放宽热额度。 */
static void configure_controllers(void)
{
    s_foc_state.id_pi.proportional_gain = s_motor_configuration.model.id_kp;
    s_foc_state.id_pi.integral_gain = s_motor_configuration.model.id_ki;
    s_foc_state.id_pi.sample_period_s = Current_Ts;
    s_foc_state.iq_pi.proportional_gain = s_motor_configuration.model.iq_kp;
    s_foc_state.iq_pi.integral_gain = s_motor_configuration.model.iq_ki;
    s_foc_state.iq_pi.sample_period_s = Current_Ts;
    s_speed_pi.proportional_gain = s_motor_configuration.application.speed_kp;
    s_speed_pi.integral_gain = s_motor_configuration.application.speed_ki;
    s_speed_pi.sample_period_s = Speed_Ts;
}

/* 每个新鲜母线样本后只刷新电压额度，不改变增益、时基或积分状态；空闲也有效。 */
static void update_current_controller_limits(void)
{
    float maximum_voltage_v = FOC_MaximumVoltage(s_foc_state.bus_voltage_filtered_v);
    s_foc_state.id_pi.output_min = s_foc_state.iq_pi.output_min = -maximum_voltage_v;
    s_foc_state.id_pi.output_max = s_foc_state.iq_pi.output_max = maximum_voltage_v;
}

/* 不把2048或上次存储零点当作本次上电校正资格；NAN阻止原自检/启动门禁通过。 */
static void initialize_current_offsets(void)
{
    s_current_offset_calibration = (CurrentOffsetCalibration){0};
    s_motor_configuration.device.phase_a_offset_counts = NAN;
    s_motor_configuration.device.phase_b_offset_counts = NAN;
    s_motor_configuration.device.phase_c_offset_counts = NAN;
    s_foc_state.phase_a_current_a = NAN;
    s_foc_state.phase_b_current_a = NAN;
    s_foc_state.phase_c_current_a = NAN;
}

/* 所有零点失败均锁存同一资格；保留采样证据和已经报告的首个故障。 */
static void fail_current_offset_calibration(void)
{
    s_current_offset_calibration.failed = true;
    MotorFault_Report(&s_motor_state, MOTOR_FAULT_CURRENT_OFFSET);
}

/* 只能由新鲜JEOS快速拍调用；不启相、不分配、不阻塞、不保存、不自动重试。 */
static void sample_phase_currents(void)
{
    if (s_current_offset_calibration.complete)
    {
        Current_Cal(&s_motor_configuration, &s_foc_state, &s_motor_state);
        return;
    }
    if (s_current_offset_calibration.failed)
    {
        return;
    }
    if (!power_stage_hw_phases_disabled() || s_motor_state.control_mode != MOTOR_CONTROL_NONE)
    {
        fail_current_offset_calibration();
        return;
    }
    uint16_t raw[3];
    for (unsigned phase = 0U; phase < 3U; ++phase)
    {
        raw[phase] = motor_hw_current_sample_raw((MotorHwCurrentPhase)phase);
        if (raw[phase] > MOTOR_SENSING_ADC_SAMPLE_COUNT * CURRENT_SENSE_PROFILE_ADC_MAX_COUNTS)
        {
            fail_current_offset_calibration();
            return;
        }
    }
    for (unsigned phase = 0U; phase < 3U; ++phase)
    {
        s_current_offset_calibration.phase_sum_raw[phase] += raw[phase];
    }
    if (++s_current_offset_calibration.sample_count < CURRENT_OFFSET_SAMPLE_COUNT)
    {
        return;
    }
    float scale =
        MOTOR_SENSING_ADC_SUM_TO_COUNTS / (float)s_current_offset_calibration.sample_count;
    s_motor_configuration.device.phase_a_offset_counts =
        (float)s_current_offset_calibration.phase_sum_raw[0] * scale;
    s_motor_configuration.device.phase_b_offset_counts =
        (float)s_current_offset_calibration.phase_sum_raw[1] * scale;
    s_motor_configuration.device.phase_c_offset_counts =
        (float)s_current_offset_calibration.phase_sum_raw[2] * scale;
    if (!MotorCurrentOffsets_Valid(&s_motor_configuration))
    {
        s_motor_configuration.device.phase_a_offset_counts = NAN;
        s_motor_configuration.device.phase_b_offset_counts = NAN;
        s_motor_configuration.device.phase_c_offset_counts = NAN;
        fail_current_offset_calibration();
        return;
    }
    s_current_offset_calibration.complete = true;
    MotorParameterTransaction_AdvanceRevision();
    Current_Cal(&s_motor_configuration, &s_foc_state, &s_motor_state);
}

void MotorAxis_Init(void)
{
    MotorParameterTransaction_Init(&s_motor_configuration);
    if (s_motor_configuration.configuration_source != MOTOR_CONFIGURATION_STORED)
    {
        s_motor_state.vq_target_v = 0.0f;
    }
    if (!MotorThermalControl_Init(&s_motor_configuration.model.thermal, time_hw_now_ms()))
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
    /* ADC快环丢失时仍由监督关断；只发布时钟失效事件，不重置热历史。 */
    if (!MotorThermalControl_CheckWatchdog(time_hw_now_ms(), power_stage_hw_phases_disabled()))
    {
        s_motor_state.thermal_current_limit_a = 0.0f;
        s_motor_state.thermal_clear_ready = false;
        MotorFault_Report(&s_motor_state, MOTOR_FAULT_WINDING_THERMAL);
        MotorStateMachine_FastFaultStop();
    }
    /* 1. 编码器慢估计：多圈/机械角/速度是慢状态唯一写者。 */
    Encoder_UpdateSlowEstimate(&s_motor_configuration, &s_rotor_encoder);

    /* 2. 外环控制：位置级联/轨迹与速度 PI 直接执行并发布 iq_target_a；
     *    本函数是外环输出唯一写者，快环只读取。 */
    MotorOuterLoop_SlowTick(&s_motor_configuration, &s_motor_state, &s_speed_pi, &s_rotor_encoder);

    /* 3. 主运行状态机：消费快环 worker 结果，完成故障指示、维护会话、
     *    功率级启停迁移与影子提交；故障的硬件关断由快环紧急快车道先行执行。 */
    MotorStateMachine_Tick();

    /* 4. MCU 温度与 VREFINT 换算属于 2 kHz 监督任务。 */
    Temperature_Update(&s_foc_state, &s_motor_state);
    return s_motor_state.control_mode;
}
/* 使用本拍捕获的维护许可；故障不预载 PWM，空闲预载不改变功率使能。 */
static MotorTaskResult run_fast_task(bool maintenance_runnable)
{
    if (!s_current_offset_calibration.complete || s_motor_state.fault_code != MOTOR_FAULT_NONE)
    {
        return (MotorTaskResult){MOTOR_TASK_RUNNING, MOTOR_FAULT_NONE};
    }
    if (maintenance_runnable)
    {
        return MotorMaintenance_Step(&s_motor_configuration,
                                     MotorStateMachine_GetMaintenanceKind(),
                                     &s_foc_state,
                                     &s_motor_state,
                                     &s_speed_pi,
                                     &s_rotor_encoder,
                                     &s_flux_observer,
                                     &s_sensorless_startup);
    }
    MotorState state = MotorStateMachine_GetState();
    if (state != MOTOR_STATE_STARTING && state != MOTOR_STATE_RUNNING)
    {
        motor_hw_pwm_set_duty(0.5f, 0.5f, 0.5f);
        return (MotorTaskResult){MOTOR_TASK_RUNNING, MOTOR_FAULT_NONE};
    }
    return MotorExecution_ControlStep(
        &s_motor_configuration, &s_foc_state, &s_motor_state, &s_rotor_encoder, &s_flux_observer);
}

/* 热采样使用本拍实际三相电流；限流只是运行快照，绝不覆盖活动参数。 */
static void update_winding_thermal(void)
{
    bool outputs_disabled = power_stage_hw_phases_disabled();
    if (s_current_offset_calibration.complete)
    {
        MotorThermalControl_Sample(s_foc_state.phase_a_current_a,
                                   s_foc_state.phase_b_current_a,
                                   s_foc_state.phase_c_current_a,
                                   outputs_disabled,
                                   time_hw_now_ms());
    }
    s_motor_state.thermal_current_limit_a =
        MotorThermalControl_CurrentLimit(s_motor_configuration.application.current_limit_a);
    s_motor_state.thermal_limit_active =
        s_motor_state.thermal_current_limit_a < s_motor_configuration.application.current_limit_a;
    s_motor_state.thermal_clear_ready = MotorThermalControl_CanClearFault();
    if (!s_current_offset_calibration.complete)
    {
        return;
    }
    MotorMaintenanceKind kind = MotorStateMachine_GetMaintenanceKind();
    bool calibration =
        kind == MOTOR_MAINTENANCE_ENCODER_LUT || kind == MOTOR_MAINTENANCE_ELECTRICAL_ZERO;
    bool derating_allowed = !calibration && s_motor_state.control_mode != MOTOR_CONTROL_VQ;
    if (!MotorThermalControl_CanRun(true) ||
        (MotorStateMachine_OutputExpected() && !MotorThermalControl_Guard(derating_allowed)))
    {
        MotorFault_Report(&s_motor_state, MOTOR_FAULT_WINDING_THERMAL);
    }
    if (!outputs_disabled && s_motor_state.control_mode == MOTOR_CONTROL_VQ)
    {
        float vector_squared_a2 =
            (2.0f / 3.0f) * (s_foc_state.phase_a_current_a * s_foc_state.phase_a_current_a +
                             s_foc_state.phase_b_current_a * s_foc_state.phase_b_current_a +
                             s_foc_state.phase_c_current_a * s_foc_state.phase_c_current_a);
        float limit_a = s_motor_state.thermal_current_limit_a;
        if (!(vector_squared_a2 <= limit_a * limit_a))
        {
            MotorFault_Report(&s_motor_state, MOTOR_FAULT_WINDING_THERMAL);
        }
    }
}

void MotorAxis_FastTick(void)
{
    /* 本拍模式 worker 的周期结果，仅由运行状态机消费。 */
    MotorTaskResult work_outcome;
    /* 编码器帧采样是否已在本拍成功发起；false 时稍后走同步兜底读取。 */
    uint8_t encoder_sample_stage;

    /* 1. 发起编码器帧采样：只启动传感器总线事务，不在本拍等待结果。
     *    返回 false 表示总线未就绪，由第 4 步同步兜底完成本帧。 */
    encoder_sample_stage = Encoder_BeginSample();

    /* 2. 母线电压采样：换算 bus_voltage_v、更新低通滤波值，并推进过压/欠压确认计数；
     *    禁用模式清零计数。保护判定基于本拍电压，必须先于控制输出执行。 */
    Vbus_Update(&s_foc_state, &s_motor_state, MotorStateMachine_OutputExpected());
    update_current_controller_limits();

    /* 3. 先用关相新鲜帧完成上电零点校正，再执行既有电流换算与过流保护。 */
    sample_phase_currents();
    encoder_sample_stage = Encoder_BeginReadback(encoder_sample_stage != 0U);
    update_winding_thermal();

    /* 5. 观测器选择：位置模式（模式 3）由编码器与外环提供角度，跳过观测器；
     *    其余模式在模式分发前更新，首次切进电流/无感模式的这一拍即有估计可用。 */
    if (s_current_offset_calibration.complete &&
        s_motor_state.control_mode != MOTOR_CONTROL_POSITION)
    {
        FluxObserver_Update(&s_motor_configuration, &s_foc_state, &s_flux_observer);
    }

    /* 第二段SPI回读与观测器运算重叠，完成后才允许故障判定及控制输出。 */
    Encoder_CompleteSample(&s_motor_configuration, &s_rotor_encoder, encoder_sample_stage);

    /* 6. 快速环故障检查：编码器离线或轴配置失效时立即置故障并停机；
     *    不清除既有故障，且先于任何控制输出执行。 */
    MotorStateMachine_CheckFastFaults();

    /* 7. 故障紧急关断快车道：故障锁存时立即切断功率级（无硬件 BKIN，软件是
     *    唯一关断路径）；状态机在下一 2 kHz 慢拍对齐记录并清理。 */
    MotorStateMachine_FastFaultStop();

    /* 8. 本拍调度：先核对故障，再选择维护、控制或空闲预载；身份在执行前捕获。
     *    本函数不写 control_mode 或功率使能，功能仅返回结果事实。 */
    uint32_t work_epoch = MotorStateMachine_WorkEpoch();
    bool configuration_work = MotorStateMachine_MaintenanceRunnable();
    work_outcome = run_fast_task(configuration_work);

    /* 9. worker 结果入箱：由 2 kHz 主状态机在下一慢拍消费。 */
    if (configuration_work && s_motor_state.fault_code == MOTOR_FAULT_NONE &&
        (work_outcome.result == MOTOR_TASK_COMPLETED || work_outcome.result == MOTOR_TASK_FINALIZE))
    {
        if (MotorStateMachine_GetMaintenanceKind() == MOTOR_MAINTENANCE_DEFAULTS)
        {
            configure_controllers();
        }
        MotorParameterTransaction_AdvanceMaintenanceRevision(work_epoch);
    }
    MotorStateMachine_PostOutcome(work_outcome, work_epoch);
    MotorStateMachine_FastFaultStop();
}

static void sample_motor_status(MotorStatus *sample)
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
}
MotorState MotorAxis_ReadStatus(MotorStatus *sample,
                                uint8_t *valid_bits,
                                MotorMaintenanceKind *maintenance_kind,
                                MotorControlProgress *control_progress)
{
    uint32_t mask = critical_hw_enter();
    sample_motor_status(sample);
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
    critical_hw_exit(mask);
    return state;
}

void MotorAxis_ServiceBackground(void)
{
    FocBackgroundClaim claim;
    if (!MotorStateMachine_BeginBackground(&claim))
    {
        return;
    }
    bool success = false;
    if (claim.kind == FOC_BACKGROUND_SAVE)
    {
        success = MotorParameterTransaction_SaveRecord(&s_motor_configuration, &claim);
    }
    uint32_t mask = critical_hw_enter();
    bool current = MotorStateMachine_BackgroundValid(&claim);
    MotorStateMachine_FinishBackground(&claim, success);
    if (claim.kind == FOC_BACKGROUND_SAVE)
    {
        MotorParameterTransaction_FinishSave(claim.epoch, current, success);
    }
    critical_hw_exit(mask);
}

/* 外部参数入口保留；事务状态与资格判断仅由参数事务模块推进。 */
ParameterStatus MotorAxis_ReadParameter(const ParameterReadRequest *request, ParameterValue *value)
{
    return MotorParameterTransaction_Read(&s_motor_configuration, request, value);
}

ParameterStatus MotorAxis_WriteParameter(const ParameterWriteRequest *request,
                                         ParameterValue *applied)
{
    uint32_t mask = critical_hw_enter();
    ParameterStatus result = MotorParameterTransaction_Write(
        &s_motor_configuration, &s_motor_state, &s_foc_state, &s_speed_pi, request, applied);
    if (result == PARAMETER_OK)
    {
        configure_controllers();
    }
    critical_hw_exit(mask);
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
