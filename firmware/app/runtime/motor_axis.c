#include "motor_axis.h"
#include <math.h>
#include "critical_hw.h"
#include "motor_maintenance.h"
#include "motor_hw.h"
#include "encoder_calibration_task.h"
#include "foc_cogging_calibration.h"
#include "power_stage_hw.h"
#include "foc_sensing.h"
#include "foc_param.h"
#include "param_store.h"
#include "heap.h"
#include "rtt_telemetry.h"

#include "angle_feedback.h"
#include "foc_algorithm.h"
#include "foc_errhandle.h"
#include "friction_identification_task.h"
#include "foc_pid.h"
#include "motor_execution.h"
#include "motor_state_machine.h"
#include "foc_sensorless.h"
#include "foc_sensorless_run.h"

/* 参数与齿槽候选和编码器采样互斥借用同一池；记录增长必须显式更新容量预算。 */
typedef char
    ParamRecord_FitsWorkspace[(sizeof(InterfaceParam) <= HEAP_WORKSPACE_CAPACITY_BYTES) ? 1 : -1];
typedef char CoggingCandidate_FitsWorkspace
    [(sizeof(CoggingMapRecord) <= HEAP_WORKSPACE_CAPACITY_BYTES) ? 1 : -1];

/* 电机与控制对象实例：全部私有定义在此，仅受控调度时传给实际计算调用方。 */
static MotorControlState s_motor_state;
static PiController s_speed_pi;
static EncoderState s_rotor_encoder;
static FluxObserverState s_flux_observer;
static SensorlessStartupState s_sensorless_startup;

static FocState s_foc_state;

/* 活动修订与最近变更身份；不常驻 InterfaceParam 副本，不复用 schema 版本。 */
static volatile uint32_t s_config_revision;
static uint32_t s_revision_work_epoch;
static struct
{
    ParameterReadRequest request;
    ParameterValue value;
    bool valid;
} s_last_read;
static struct
{
    ParameterWriteRequest request;
    ParameterValue applied;
    bool valid;
} s_last_write;
static struct
{
    ParameterSaveRequest request;
    uint32_t ticket, persisted_revision;
    ParameterStatus result;
    bool valid;
} s_save_job;

static void advance_config_revision(void)
{
    if (s_config_revision != UINT32_MAX)
    {
        ++s_config_revision;
    }
}

/* 启动时只借用一份既有大小的堆记录，不额外常驻配置副本。 */
static void load_parameters(void)
{
    InterfaceParam *record = Heap_Allocate(sizeof(*record));
    if (record == NULL || !param_store_read(record, sizeof(*record)))
    {
        Param_Return_Default(&s_motor_state, &s_rotor_encoder);
    }
    else
    {
        Param_Download(record, &s_motor_state, &s_rotor_encoder);
    }
    if (record != NULL)
    {
        Heap_Free(record);
    }
}

uint8_t MotorAxis_NodeId(void)
{
    return s_motor_state.device.node_id;
}

void MotorAxis_Init(void)
{
    s_config_revision = 1U;
    s_revision_work_epoch = UINT32_MAX;
    s_last_read.valid = s_last_write.valid = s_save_job.valid = false;
    load_parameters();
    MotorStateMachine_Init(
        &s_motor_state, &s_foc_state, &s_speed_pi, &s_rotor_encoder, &s_sensorless_startup);
    Encoder_ParamInit(&s_rotor_encoder);

    FluxObserver_ParamInit(&s_flux_observer);
    SensorlessStartup_Reset(&s_sensorless_startup);
    FOC_CurrentController_Reset(&s_foc_state);
    PiController_Reset(&s_speed_pi);

    s_motor_state.application.position_error_window_rad = 0.001f;
    s_motor_state.position_velocity_filtered_rad_s = 0.0f;
    MotorOuterLoop_RequestReset();
    MotorMaintenance_Init();

    /* 未配置轴记录的关节在启动后保持禁用。 */
    /* 启动默认保持禁用：CurrentOffset 标定启动已随 s_foc_state-Calibration 功能移除，
     * 待标定重建后再定启动流程；避免以死模式进入自动使能而空开功率级。 */
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
/* 每拍仅选择一个已获准功能；故障不预载 PWM，空闲预载不改变功率使能。 */
static MotorTaskResult run_fast_task(void)
{
    if (s_motor_state.fault_code != MOTOR_FAULT_NONE)
    {
        return (MotorTaskResult){MOTOR_TASK_RUNNING, MOTOR_FAULT_NONE};
    }
    if (MotorStateMachine_MaintenanceRunnable())
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

    /* 3. 相电流换算：ADC 原始值减零点偏置并折算为 phase_a_current_a/phase_b_current_a/phase_c_current_a（A），
     *    同时校验偏置有效性，并按连续 5 拍确认过流。 */
    Current_Cal(&s_foc_state, &s_motor_state);

    /* 4. 完成本拍角度采样：取回 SPI 帧，做方向/LUT 校正并更新电角度；
     *    多圈/机械角/速度由 2 kHz 慢估计维护。 */
    Encoder_CompleteSample(&s_motor_state, &s_rotor_encoder, encoder_sample_started);

    /* 5. 观测器选择：位置模式（模式 3）由编码器与外环提供角度，跳过观测器；
     *    其余模式在模式分发前更新，首次切进电流/无感模式的这一拍即有估计可用。 */
    if (s_motor_state.control_mode != MOTOR_CONTROL_POSITION)
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
    work_outcome = run_fast_task();

    /* 9. worker 结果入箱：由 2 kHz 主状态机在下一慢拍消费。 */
    if (configuration_work && s_motor_state.fault_code == MOTOR_FAULT_NONE &&
        (work_outcome.result == MOTOR_TASK_COMPLETED ||
         work_outcome.result == MOTOR_TASK_FINALIZE) &&
        work_epoch != s_revision_work_epoch)
    {
        s_revision_work_epoch = work_epoch;
        advance_config_revision();
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
        InterfaceParam *record = Heap_Allocate(sizeof(*record));
        if (record != NULL)
        {
            Param_Upload(record, &s_motor_state, &s_rotor_encoder);
            uint32_t mask = critical_hw_enter();
            if (MotorStateMachine_BackgroundValid(&claim))
            {
                success = param_store_write(record, sizeof(*record));
            }
            critical_hw_exit(mask);
            Heap_Free(record);
        }
    }
    else if (claim.kind == FOC_BACKGROUND_COGGING)
    {
        CoggingMapRecord *candidate = Heap_Allocate(sizeof(*candidate));
        if (candidate != NULL)
        {
            bool built = FocCogging_BuildCandidate(candidate, &s_motor_state, &s_rotor_encoder);
            uint32_t mask = critical_hw_enter();
            if (built && MotorStateMachine_BackgroundValid(&claim))
            {
                success = FocCogging_PublishCandidate(candidate);
            }
            critical_hw_exit(mask);
            Heap_Free(candidate);
        }
    }
    uint32_t mask = critical_hw_enter();
    bool current = MotorStateMachine_BackgroundValid(&claim);
    MotorStateMachine_FinishBackground(&claim, success);
    if (s_save_job.valid && s_save_job.ticket == claim.epoch && claim.kind == FOC_BACKGROUND_SAVE)
    {
        s_save_job.result = !current  ? PARAMETER_CANCELLED
                            : success ? PARAMETER_OK
                                      : PARAMETER_PERSIST_FAILED;
        if (current && success)
        {
            s_save_job.persisted_revision = s_save_job.request.expected_revision;
        }
    }
    critical_hw_exit(mask);
}

/* 比较实际字段而非含 padding 的结构内存；幂等身份包含原始整数位模式。 */
static bool same_write(const ParameterWriteRequest *left, const ParameterWriteRequest *right)
{
    return left->parameter_id == right->parameter_id && left->value_type == right->value_type &&
           left->expected_revision == right->expected_revision &&
           left->raw_value == right->raw_value;
}
static bool stale_sequence(uint8_t source, uint16_t sequence)
{
    /* 单一受权主站的半圈窗口；相同身份重放在调用方先匹配缓存，避免再执行。 */
    if (s_last_write.valid && source == s_last_write.request.source_node &&
        (uint16_t)(sequence - s_last_write.request.sequence) >= 0x8000U)
    {
        return true;
    }
    if (s_save_job.valid && source == s_save_job.request.source_node &&
        (uint16_t)(sequence - s_save_job.request.sequence) >= 0x8000U)
    {
        return true;
    }
    return false;
}

/* 调用方持有临界区；身份冲突：来源与序号命中任一已缓存票据。各入口先各自重放主缓存结果。 */
static bool matches_cached_request(uint8_t source_node, uint16_t sequence)
{
    return (s_last_read.valid && source_node == s_last_read.request.source_node &&
            sequence == s_last_read.request.sequence) ||
           (s_last_write.valid && source_node == s_last_write.request.source_node &&
            sequence == s_last_write.request.sequence) ||
           (s_save_job.valid && source_node == s_save_job.request.source_node &&
            sequence == s_save_job.request.sequence);
}

/* 调用方持有临界区；新请求也收敛旧SAVE，不能依赖主机继续轮询已取消任务。 */
static void refresh_save_result(void)
{
    if (!s_save_job.valid || s_save_job.result != PARAMETER_ACCEPTED)
    {
        return;
    }
    MotorMaintenanceProgress progress;
    if (!MotorStateMachine_ReadMaintenance(s_save_job.ticket, &progress) ||
        progress.result == MOTOR_MAINTENANCE_CANCELLED)
    {
        s_save_job.result = PARAMETER_CANCELLED;
    }
    else if (progress.result == MOTOR_MAINTENANCE_FAILED)
    {
        s_save_job.result = PARAMETER_PERSIST_FAILED;
    }
}

ParameterStatus MotorAxis_ReadParameter(const ParameterReadRequest *request, ParameterValue *value)
{
    if (value == NULL)
    {
        return PARAMETER_INVALID_ARGUMENT;
    }
    *value = (ParameterValue){0};
    if (request == NULL)
    {
        return PARAMETER_INVALID_ARGUMENT;
    }
    uint32_t mask = critical_hw_enter();
    ParameterStatus result;
    if (s_last_read.valid && request->source_node == s_last_read.request.source_node &&
        request->sequence == s_last_read.request.sequence)
    {
        result = request->parameter_id == s_last_read.request.parameter_id
                     ? PARAMETER_OK
                     : PARAMETER_CONFLICTING_RETRY;
        if (result == PARAMETER_OK)
        {
            *value = s_last_read.value;
        }
    }
    else if (matches_cached_request(request->source_node, request->sequence))
    {
        result = PARAMETER_CONFLICTING_RETRY;
    }
    else
    {
        MotorState state = MotorStateMachine_GetState();
        result = state == MOTOR_STATE_MAINTENANCE || state == MOTOR_STATE_STARTING ||
                         state == MOTOR_STATE_STOPPING
                     ? PARAMETER_BUSY
                     : ParameterValue_Read(&s_motor_state,
                                           &s_rotor_encoder,
                                           s_config_revision,
                                           request->parameter_id,
                                           value);
        if (request->parameter_id >= 0x4FU && request->parameter_id <= 0x51U)
        {
            MotorReadiness readiness;
            MotorStateMachine_ReadReadiness(&readiness);
            uint32_t raw = request->parameter_id == 0x4FU   ? readiness.start_blockers
                           : request->parameter_id == 0x50U ? readiness.self_test_checks
                                                            : readiness.startable_modes;
            *value = (ParameterValue){request->parameter_id,
                                      PARAMETER_VALUE_U32,
                                      PARAMETER_ACCESS_READ,
                                      0U,
                                      s_config_revision,
                                      raw};
            result = PARAMETER_OK;
        }
        if (result == PARAMETER_OK)
        {
            if (!MotorStateMachine_ConfigurationAllowed())
            {
                value->access &= (uint8_t)~PARAMETER_ACCESS_WRITE_IDLE;
            }
            s_last_read.request = *request;
            s_last_read.value = *value;
            s_last_read.valid = true;
        }
    }
    critical_hw_exit(mask);
    return result;
}

ParameterStatus MotorAxis_WriteParameter(const ParameterWriteRequest *request,
                                         ParameterValue *applied)
{
    if (applied == NULL)
    {
        return PARAMETER_INVALID_ARGUMENT;
    }
    *applied = (ParameterValue){0};
    if (request == NULL || request->reserved != 0U || request->reserved_sequence != 0U)
    {
        return PARAMETER_INVALID_ARGUMENT;
    }
    uint32_t mask = critical_hw_enter();
    refresh_save_result();
    ParameterStatus result;
    if (s_last_write.valid && request->source_node == s_last_write.request.source_node &&
        request->sequence == s_last_write.request.sequence)
    {
        result =
            same_write(request, &s_last_write.request) ? PARAMETER_OK : PARAMETER_CONFLICTING_RETRY;
        if (result == PARAMETER_OK)
        {
            *applied = s_last_write.applied;
        }
    }
    else if (matches_cached_request(request->source_node, request->sequence))
    {
        result = PARAMETER_CONFLICTING_RETRY;
    }
    else if (stale_sequence(request->source_node, request->sequence))
    {
        result = PARAMETER_STALE_SEQUENCE;
    }
    else if (s_motor_state.fault_code != MOTOR_FAULT_NONE)
    {
        result = PARAMETER_FAULT;
    }
    else if (!MotorStateMachine_ConfigurationAllowed() ||
             (s_save_job.valid && s_save_job.result == PARAMETER_ACCEPTED))
    {
        result = PARAMETER_WRONG_STATE;
    }
    else if (request->expected_revision != s_config_revision || s_config_revision == UINT32_MAX)
    {
        result = PARAMETER_REVISION_CONFLICT;
    }
    else
    {
        result = ParameterValue_Apply(&s_motor_state, request);
        if (result == PARAMETER_OK)
        {
            advance_config_revision();
            /* 原有控制器复位边界，不在这里重写控制数学或增益公式。 */
            MotorOuterLoop_RequestReset();
            FOC_CurrentController_Reset(&s_foc_state);
            PiController_Reset(&s_speed_pi);
            result = ParameterValue_Read(&s_motor_state,
                                         &s_rotor_encoder,
                                         s_config_revision,
                                         request->parameter_id,
                                         applied);
            if (result == PARAMETER_OK)
            {
                s_last_write.request = *request;
                s_last_write.applied = *applied;
                s_last_write.valid = true;
            }
        }
    }
    critical_hw_exit(mask);
    return result;
}

ParameterStatus MotorAxis_RequestSave(const ParameterSaveRequest *request, uint32_t *ticket)
{
    if (ticket == NULL)
    {
        return PARAMETER_INVALID_ARGUMENT;
    }
    *ticket = 0U;
    if (request == NULL || request->reserved != 0U)
    {
        return PARAMETER_INVALID_ARGUMENT;
    }
    uint32_t mask = critical_hw_enter();
    refresh_save_result();
    ParameterStatus result;
    if (s_save_job.valid && request->source_node == s_save_job.request.source_node &&
        request->sequence == s_save_job.request.sequence)
    {
        result = request->expected_revision == s_save_job.request.expected_revision
                     ? s_save_job.result
                     : PARAMETER_CONFLICTING_RETRY;
        if (result != PARAMETER_CONFLICTING_RETRY)
        {
            *ticket = s_save_job.ticket;
        }
    }
    else if (matches_cached_request(request->source_node, request->sequence))
    {
        result = PARAMETER_CONFLICTING_RETRY;
    }
    else if (stale_sequence(request->source_node, request->sequence))
    {
        result = PARAMETER_STALE_SEQUENCE;
    }
    else if (s_save_job.valid && s_save_job.result == PARAMETER_ACCEPTED)
    {
        result = PARAMETER_BUSY;
    }
    else if (request->expected_revision != s_config_revision)
    {
        result = PARAMETER_REVISION_CONFLICT;
    }
    else
    {
        MotorControlResult accepted =
            MotorStateMachine_RequestMaintenance(MOTOR_MAINTENANCE_SAVE, ticket);
        result = accepted == MOTOR_CONTROL_ACCEPTED ? PARAMETER_ACCEPTED
                 : accepted == MOTOR_CONTROL_BUSY   ? PARAMETER_BUSY
                 : accepted == MOTOR_CONTROL_FAULT  ? PARAMETER_FAULT
                                                    : PARAMETER_WRONG_STATE;
        if (result == PARAMETER_ACCEPTED)
        {
            s_save_job.request = *request;
            s_save_job.ticket = *ticket;
            s_save_job.persisted_revision = 0U;
            s_save_job.result = result;
            s_save_job.valid = true;
        }
    }
    critical_hw_exit(mask);
    return result;
}

ParameterStatus MotorAxis_PollSave(uint32_t ticket, uint32_t *persisted_revision)
{
    if (persisted_revision == NULL)
    {
        return PARAMETER_INVALID_ARGUMENT;
    }
    *persisted_revision = 0U;
    uint32_t mask = critical_hw_enter();
    ParameterStatus result = PARAMETER_STALE_TICKET;
    if (s_save_job.valid && ticket != 0U && ticket == s_save_job.ticket)
    {
        refresh_save_result();
        result = s_save_job.result;
        if (result == PARAMETER_OK)
        {
            *persisted_revision = s_save_job.persisted_revision;
        }
    }
    critical_hw_exit(mask);
    return result;
}

bool MotorAxis_SaveIdle(void)
{
    uint32_t mask = critical_hw_enter();
    refresh_save_result();
    bool idle = !s_save_job.valid || s_save_job.result != PARAMETER_ACCEPTED;
    critical_hw_exit(mask);
    return idle;
}

bool MotorAxis_ConfigurationRevisionAvailable(void)
{
    return s_config_revision != 0U && s_config_revision != UINT32_MAX;
}
