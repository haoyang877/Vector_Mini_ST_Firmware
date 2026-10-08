#include "motor_axis.h"
#include <math.h>
#include "critical_hw.h"
#include "motor_maintenance.h"
#include "motor_hw.h"
#include "motor_sensing.h"
#include "foc_cogging_calibration.h"
#include "power_stage_hw.h"
#include "foc_sensing.h"
#include "motor_parameter_transaction.h"
#include "maintenance_workspace.h"
#include "rtt_telemetry.h"

#include "angle_feedback.h"
#include "foc_algorithm.h"
#include "foc_errhandle.h"
#include "foc_pid.h"
#include "motor_execution.h"
#include "motor_state_machine.h"
#include "foc_sensorless.h"
#include "foc_sensorless_run.h"

/* 齿槽候选与其它维护任务互斥借用同一工作区；增长必须显式更新容量预算。 */
typedef char CoggingCandidate_FitsWorkspace
    [(sizeof(CoggingMapRecord) <= MAINTENANCE_WORKSPACE_CAPACITY_BYTES) ? 1 : -1];

/* 电机与控制对象全局符号供 Watch 观测；运行写入仍由 MotorAxis 组装和调度。 */
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
    return s_motor_state.device.node_id;
}

/* PI运行对象立即反映活动参数；首个有效母线样本前电压额度为零，运行时照常更新。 */
static void configure_controllers(void)
{
    float maximum_voltage =
        isfinite(s_foc_state.bus_voltage_filtered_v) && s_foc_state.bus_voltage_filtered_v > 0.0f
            ? FOC_MaximumVoltage(s_foc_state.bus_voltage_filtered_v)
            : 0.0f;
    PiController_Configure(&s_foc_state.id_pi,
                           s_motor_state.model.id_kp,
                           s_motor_state.model.id_ki,
                           Current_Ts,
                           -maximum_voltage,
                           maximum_voltage);
    PiController_Configure(&s_foc_state.iq_pi,
                           s_motor_state.model.iq_kp,
                           s_motor_state.model.iq_ki,
                           Current_Ts,
                           -maximum_voltage,
                           maximum_voltage);
    PiController_Configure(&s_speed_pi,
                           s_motor_state.application.speed_kp,
                           s_motor_state.application.speed_ki,
                           Speed_Ts,
                           -1.0f,
                           1.0f);
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
    s_motor_state.device.phase_a_offset_counts = NAN;
    s_motor_state.device.phase_b_offset_counts = NAN;
    s_motor_state.device.phase_c_offset_counts = NAN;
    s_foc_state.phase_a_current_a = NAN;
    s_foc_state.phase_b_current_a = NAN;
    s_foc_state.phase_c_current_a = NAN;
}

/* 只能由新鲜JEOS快速拍调用；不启相、不分配、不阻塞、不保存、不自动重试。 */
static void sample_phase_currents(void)
{
    if (s_current_offset_calibration.complete)
    {
        Current_Cal(&s_foc_state, &s_motor_state);
        return;
    }
    if (s_current_offset_calibration.failed)
    {
        return;
    }
    if (!power_stage_hw_phases_disabled() || s_motor_state.control_mode != MOTOR_CONTROL_NONE)
    {
        s_current_offset_calibration.failed = true;
        MotorFault_Report(&s_motor_state, MOTOR_FAULT_CURRENT_OFFSET);
        return;
    }
    uint16_t raw[3];
    for (unsigned phase = 0U; phase < 3U; ++phase)
    {
        raw[phase] = motor_hw_current_sample_raw((MotorHwCurrentPhase)phase);
        if (raw[phase] > 4U * 4095U)
        {
            s_current_offset_calibration.failed = true;
            MotorFault_Report(&s_motor_state, MOTOR_FAULT_CURRENT_OFFSET);
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
    s_motor_state.device.phase_a_offset_counts =
        (float)s_current_offset_calibration.phase_sum_raw[0] * scale;
    s_motor_state.device.phase_b_offset_counts =
        (float)s_current_offset_calibration.phase_sum_raw[1] * scale;
    s_motor_state.device.phase_c_offset_counts =
        (float)s_current_offset_calibration.phase_sum_raw[2] * scale;
    if (!MotorCurrentOffsets_Valid(&s_motor_state))
    {
        s_motor_state.device.phase_a_offset_counts = NAN;
        s_motor_state.device.phase_b_offset_counts = NAN;
        s_motor_state.device.phase_c_offset_counts = NAN;
        s_current_offset_calibration.failed = true;
        MotorFault_Report(&s_motor_state, MOTOR_FAULT_CURRENT_OFFSET);
        return;
    }
    s_current_offset_calibration.complete = true;
    MotorParameterTransaction_AdvanceRevision();
    Current_Cal(&s_foc_state, &s_motor_state);
}

void MotorAxis_Init(void)
{
    MotorParameterTransaction_Init(&s_motor_state, &s_rotor_encoder);
    initialize_current_offsets();
    MotorStateMachine_Init(
        &s_motor_state, &s_foc_state, &s_speed_pi, &s_rotor_encoder, &s_sensorless_startup);
    Encoder_ParamInit(&s_rotor_encoder);

    FluxObserver_ParamInit(&s_flux_observer);
    SensorlessStartup_Reset(&s_sensorless_startup);
    FOC_CurrentController_Reset(&s_foc_state);
    PiController_Reset(&s_speed_pi);
    configure_controllers();

    s_motor_state.application.position_error_window_rad = 0.001f;
    s_motor_state.position_velocity_filtered_rad_s = 0.0f;
    MotorOuterLoop_RequestReset();
    MotorMaintenance_Init();

    /* 未配置轴记录的关节在启动后保持禁用。 */
    /* 上电零点校正和基础自检只在关相状态推进，完成也不自动使能电机。 */
    s_motor_state.control_mode = MOTOR_CONTROL_NONE;
    if (!s_motor_state.application.axis_profile_valid)
    {
        MotorFault_Report(&s_motor_state, MOTOR_FAULT_INVALID_PARAMETERS);
    }
}

MotorControlMode MotorAxis_SupervisorTick(void)
{
    /* 1. 编码器慢估计：多圈/机械角/速度是慢状态唯一写者。 */
    Encoder_UpdateSlowEstimate(&s_motor_state, &s_rotor_encoder);

    /* 2. 外环控制：位置级联/轨迹与速度 PI 直接执行并发布 iq_target_a；
     *    本函数是外环输出唯一写者，快环只读取。 */
    MotorOuterLoop_SlowTick(&s_motor_state, &s_speed_pi, &s_rotor_encoder);

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
        return MotorMaintenance_Step(MotorStateMachine_GetMaintenanceKind(),
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
        &s_foc_state, &s_motor_state, &s_rotor_encoder, &s_flux_observer);
}

void MotorAxis_FastTick(void)
{
    /* 本拍模式 worker 的周期结果，仅由运行状态机消费。 */
    MotorTaskResult work_outcome;
    /* 编码器帧采样是否已在本拍成功发起；false 时稍后走同步兜底读取。 */
    bool encoder_sample_started;

    /* 1. 发起编码器帧采样：只启动传感器总线事务，不在本拍等待结果。
     *    返回 false 表示总线未就绪，由第 4 步同步兜底完成本帧。 */
    encoder_sample_started = Encoder_BeginSample();

    /* 2. 母线电压采样：换算 bus_voltage_v、更新低通滤波值，并推进过压/欠压确认计数；
     *    禁用模式清零计数。保护判定基于本拍电压，必须先于控制输出执行。 */
    Vbus_Update(&s_foc_state, &s_motor_state, MotorStateMachine_OutputExpected());
    update_current_controller_limits();

    /* 3. 先用关相新鲜帧完成上电零点校正，再执行既有电流换算与过流保护。 */
    sample_phase_currents();

    /* 4. 完成本拍角度采样：取回 SPI 帧，做方向/LUT 校正并更新电角度；
     *    多圈/机械角/速度由 2 kHz 慢估计维护。 */
    Encoder_CompleteSample(&s_motor_state, &s_rotor_encoder, encoder_sample_started);

    /* 5. 观测器选择：位置模式（模式 3）由编码器与外环提供角度，跳过观测器；
     *    其余模式在模式分发前更新，首次切进电流/无感模式的这一拍即有估计可用。 */
    if (s_current_offset_calibration.complete &&
        s_motor_state.control_mode != MOTOR_CONTROL_POSITION)
    {
        FluxObserver_Update(&s_foc_state, &s_motor_state, &s_flux_observer);
    }

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
        MotorParameterTransaction_AdvanceMaintenanceRevision(work_epoch);
    }
    MotorStateMachine_PostOutcome(work_outcome, work_epoch);
    MotorStateMachine_FastFaultStop();

    /* 10. 遥测：采样一帧 4 通道 RTT（机械位置/机械转速/iq_a 指令/iq_a 反馈），
     *     非阻塞、有界，不修改控制状态与运行模式。 */
    RTT_Sampling(s_motor_state.control_mode,
                 Encoder_GetMecPos(&s_rotor_encoder),
                 Encoder_GetMecVel(&s_rotor_encoder),
                 s_motor_state.iq_target_a,
                 s_foc_state.iq_a);
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
    if (s_motor_state.control_mode == MOTOR_CONTROL_POSITION)
    {
        sample->position_planned = s_motor_state.position_reference_rad;
        sample->speed_planned = s_motor_state.position_trajectory_speed_rad_s;
    }
    else if (s_motor_state.control_mode == MOTOR_CONTROL_SPEED)
    {
        sample->speed_planned = s_motor_state.speed_ramped_rad_s;
    }
}
MotorState MotorAxis_ReadStatus(MotorStatus *sample,
                                uint8_t *valid_bits,
                                MotorMaintenanceKind *maintenance_kind,
                                MotorControlProgress *control_progress)
{
    uint32_t mask = critical_hw_enter();
    sample_motor_status(sample);
    *valid_bits = 0x54U;
    if (s_motor_state.application.axis_profile_valid &&
        Encoder_GetBadFrameStreak(&s_rotor_encoder) == 0U &&
        Encoder_GetCalibFlag(&s_rotor_encoder) != 0U)
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
    /* 补偿请求只读取稳定配置；维护占用时不运行整表 CRC。 */
    MotorStateMachine_ServiceCompensation();
    if (!MotorStateMachine_BeginBackground(&claim))
    {
        return;
    }
    bool success = false;
    if (claim.kind == FOC_BACKGROUND_SAVE)
    {
        success = MotorParameterTransaction_SaveRecord(&s_motor_state, &s_rotor_encoder, &claim);
    }
    else if (claim.kind == FOC_BACKGROUND_COGGING)
    {
        CoggingMapRecord *candidate = MaintenanceWorkspace_BorrowCogging();
        if (candidate != NULL)
        {
            bool built = FocCogging_BuildCandidate(candidate, &s_motor_state, &s_rotor_encoder);
            uint32_t mask = critical_hw_enter();
            if (built && MotorStateMachine_BackgroundValid(&claim))
            {
                success = FocCogging_PublishCandidate(candidate);
            }
            critical_hw_exit(mask);
            MaintenanceWorkspace_ReleaseCogging(candidate);
        }
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
    return MotorParameterTransaction_Read(&s_motor_state, &s_rotor_encoder, request, value);
}

ParameterStatus MotorAxis_WriteParameter(const ParameterWriteRequest *request,
                                         ParameterValue *applied)
{
    return MotorParameterTransaction_Write(
        &s_motor_state, &s_foc_state, &s_speed_pi, &s_rotor_encoder, request, applied);
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
