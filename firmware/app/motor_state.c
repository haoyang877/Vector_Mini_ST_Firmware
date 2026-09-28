#include "motor_state.h"
#include <math.h>
#include "critical_hw.h"
#include "foc_mode_dispatch.h"
#include "foc_encoder_calibration.h"
#include "foc_cogging_calibration.h"
#include "power_stage_hw.h"
#include "foc_sensing.h"
#include "foc_param.h"
#include "param_comm_bridge.h"
#include "param_store.h"
#include "heap.h"
#include "rtt_telemetry.h"

#include "angle_feedback.h"
#include "foc_algorithm.h"
#include "foc_errhandle.h"
#include "foc_friction_identification.h"
#include "foc_pid.h"
#include "foc_run.h"
#include "foc_run_state.h"
#include "foc_sensorless.h"
#include "foc_sensorless_run.h"

/* 电机与控制对象实例：全部私有定义在此，仅受控调度时传给实际计算调用方。 */
static MotorControl_TypeDef MotorControl;
static PI_Controller_TypeDef PI_Speed;
static Encoder_TypeDef OnBoard_Encoder;
static Fluxobserver_TypeDef Fluxobserver;
static SensorlessStartup_TypeDef SensorlessStartup;

static FOC_TypeDef FOC;

/* 活动修订与最近变更身份；不常驻 InterfaceParam 副本，不复用 schema 版本。 */
static uint32_t config_revision, revision_work_epoch;
static struct
{
    ParameterReadRequest request;
    ParameterValue value;
    bool valid;
} last_read;
static struct
{
    ParameterWriteRequest request;
    ParameterValue applied;
    bool valid;
} last_write;
static struct
{
    ParameterSaveRequest request;
    uint32_t ticket, persisted_revision;
    ParameterStatus result;
    bool valid;
} save_job;

static void advance_config_revision(void)
{
    if (config_revision != UINT32_MAX)
    {
        ++config_revision;
    }
}

/* 启动时只借用一份既有大小的堆记录，不额外常驻配置副本。 */
static void load_parameters(void)
{
    InterfaceParam_TypeDef *record = HEAP_malloc(sizeof(*record));
    if (record == NULL || !param_store_read(record, sizeof(*record)))
    {
        Param_Return_Default(&MotorControl, &OnBoard_Encoder);
    }
    else if (Param_Download(record, &MotorControl, &OnBoard_Encoder))
    {
        Param_Upload(record, &MotorControl, &OnBoard_Encoder);
        record->magic_word = MAGIC_WORD;
        uint32_t mask = critical_hw_enter();
        (void)param_store_write(record, sizeof(*record));
        critical_hw_exit(mask);
    }
    if (record != NULL)
    {
        HEAP_free(record);
    }
}

void MotorControl_Init(void)
{
    config_revision = 1U;
    revision_work_epoch = UINT32_MAX;
    last_read.valid = last_write.valid = save_job.valid = false;
    load_parameters();
    FocRunState_Init(&MotorControl, &FOC, &PI_Speed, &OnBoard_Encoder, &SensorlessStartup);
    Encoder_ParamInit(&OnBoard_Encoder);

    Fluxobserver_ParamInit(&Fluxobserver);
    SensorlessStartup_Reset(&SensorlessStartup);
    FOC_CurrentController_Reset(&FOC);
    PI_Controller_Reset(&PI_Speed);

    MotorControl.pos_error_window = 0.001f;
    MotorControl.pos_vel_filtered = 0.0f;
    Task_Position_Mode_Reset();
    FocFrictionIdentification_Init();

    /* 未配置轴记录的关节在启动后保持禁用。 */
    /* 启动默认保持禁用：CurrentOffset 标定启动已随 FOC-Calibration 功能移除，
     * 待标定重建后再定启动流程；避免以死模式进入自动使能而空开功率级。 */
    MotorControl.ModeNow = Motor_Disable;
    if (!MotorControl.axis_profile_valid)
    {
        MotorFault_Report(&MotorControl, MotorParam_Error);
    }
}

ModeNow_TypeDef MotorControl_SlowTick(void)
{
    /* 1. 编码器慢估计：多圈/机械角/速度是慢状态唯一写者。 */
    Encoder_UpdateSlowEstimate(&MotorControl, &OnBoard_Encoder);

    /* 2. 外环控制：位置级联/轨迹与速度 PI 直接执行并发布 iqRef；
     *    本函数是外环输出唯一写者，快环只读取。 */
    MotorOuterLoop_SlowTick(&MotorControl, &PI_Speed, &OnBoard_Encoder);

    /* 3. 主运行状态机：消费快环 worker 结果，完成故障指示、维护会话、
     *    功率级启停迁移与影子提交；故障的硬件关断由快环紧急快车道先行执行。 */
    FocRunState_Tick();

    /* 4. MCU 温度与 VREFINT 换算属于 2 kHz 监督任务。 */
    Temperature_Update(&FOC, &MotorControl);
    return MotorControl.ModeNow;
}
void MotorControl_FastTick(void)
{
    /* 本拍模式 worker 的周期结果，仅由运行状态机消费。 */
    MotorWorkOutcome_TypeDef work_outcome;
    /* 编码器帧采样是否已在本拍成功发起；false 时稍后走同步兜底读取。 */
    bool encoder_sample_started;

    /* 1. 发起编码器帧采样：只启动传感器总线事务，不在本拍等待结果。
     *    返回 false 表示总线未就绪，由第 4 步同步兜底完成本帧。 */
    encoder_sample_started = Encoder_BeginSample();

    /* 2. 母线电压采样：换算 Vbus、更新低通滤波值，并推进过压/欠压确认计数；
     *    禁用模式清零计数。保护判定基于本拍电压，必须先于控制输出执行。 */
    Vbus_Update(&FOC, &MotorControl);

    /* 3. 相电流换算：ADC 原始值减零点偏置并折算为 Ia/Ib/Ic（A），
     *    同时校验偏置有效性，并按连续 5 拍确认过流。 */
    Current_Cal(&FOC, &MotorControl);

    /* 4. 完成本拍角度采样：取回 SPI 帧，做方向/LUT 校正并更新电角度；
     *    多圈/机械角/速度由 2 kHz 慢估计维护。 */
    Encoder_CompleteSample(&MotorControl, &OnBoard_Encoder, encoder_sample_started);

    /* 5. 观测器选择：位置模式（模式 3）由编码器与外环提供角度，跳过观测器；
     *    其余模式在模式分发前更新，首次切进电流/无感模式的这一拍即有估计可用。 */
    if (MotorControl.ModeNow != Position_Mode)
    {
        Fluxobserver_Update(&FOC, &MotorControl, &Fluxobserver);
    }

    /* 6. 快速环故障检查：编码器离线或轴配置失效时立即置故障并停机；
     *    不清除既有故障，且先于任何控制输出执行。 */
    FocRunState_CheckFastFaults();

    /* 7. 故障紧急关断快车道：故障锁存时立即切断功率级（无硬件 BKIN，软件是
     *    唯一关断路径）；状态机在下一 2 kHz 慢拍对齐记录并清理。 */
    FocRunState_FastFaultStop();

    /* 8. 模式分发：按 ModeNow 执行对应 worker（禁用态零矢量预载、控制、
     *    标定与参数命令），回传周期结果；本函数不写 ModeNow 或功率级。 */
    uint32_t work_epoch = FocRunState_WorkEpoch();
    bool configuration_work = FocRunState_IsMaintenanceActive(MotorControl.ModeNow);
    work_outcome = FocMode_Dispatch(
        &FOC, &MotorControl, &PI_Speed, &OnBoard_Encoder, &Fluxobserver, &SensorlessStartup);

    /* 9. worker 结果入箱：由 2 kHz 主状态机在下一慢拍消费。 */
    if (configuration_work && MotorControl.ErrorNow == No_Error &&
        (work_outcome.result == MOTOR_WORK_SWITCH_MODE || work_outcome.result == MOTOR_WORK_STOP) &&
        work_epoch != revision_work_epoch)
    {
        revision_work_epoch = work_epoch;
        advance_config_revision();
    }
    FocRunState_PostOutcome(work_outcome, work_epoch);

    /* 10. 遥测：采样一帧 4 通道 RTT（机械位置/机械转速/Iq 指令/Iq 反馈），
     *     非阻塞、有界，不修改控制状态与运行模式。 */
    RTT_Sampling(MotorControl.ModeNow,
                 Encoder_GetMecPos(&OnBoard_Encoder),
                 Encoder_GetMecVel(&OnBoard_Encoder),
                 MotorControl.iqRef,
                 FOC.Iq);
}

static void sample_motor_status(MotorStatus *sample)
{
    sample->fault = (uint16_t)MotorControl.ErrorNow;
    sample->mode = (uint16_t)MotorControl.ModeNow;
    sample->position_target = MotorControl.posRef;
    sample->position_feedback = Encoder_GetMecPos(&OnBoard_Encoder);
    sample->speed_target = MotorControl.speedRef;
    sample->speed_feedback =
        (MotorControl.ModeNow == Position_Mode || MotorControl.ModeNow == Position_Impedance_Mode)
            ? MotorControl.pos_vel_filtered
            : Encoder_GetMecVel(&OnBoard_Encoder);
    sample->current_reference = MotorControl.iqRef;
    sample->current_feedback = FOC.Iq;
    sample->temperature = FOC.temp;
    sample->bus_voltage = FOC.Vbus_filt;
    sample->bus_current = FOC.Ibus_filt;
    sample->position_planned = NAN;
    sample->speed_planned = NAN;
    if (MotorControl.ModeNow == Position_Mode)
    {
        sample->position_planned = MotorControl.posShadow;
        sample->speed_planned = MotorControl.pos_trajectory_speed_rad_s;
    }
    else if (MotorControl.ModeNow == Position_Impedance_Mode)
    {
        sample->position_planned = MotorControl.posShadow;
        sample->speed_planned = MotorControl.speedShadow;
    }
    else if (MotorControl.ModeNow == Speed_Mode)
    {
        sample->speed_planned = MotorControl.speedShadow;
    }
}
AppLifecycleState MotorControl_ReadStatus(MotorStatus *sample, uint8_t *valid_bits)
{
    uint32_t mask = critical_hw_enter();
    sample_motor_status(sample);
    *valid_bits = 0x54U;
    if (MotorControl.axis_profile_valid && Encoder_GetBadFrameStreak(&OnBoard_Encoder) == 0U &&
        Encoder_GetCalibFlag(&OnBoard_Encoder) != 0U)
    {
        *valid_bits |= 0x03U;
    }
    if (MotorTemperature_IsFresh())
    {
        *valid_bits |= 0x20U;
    }
    AppLifecycleState state = FocRunState_GetState();
    critical_hw_exit(mask);
    return state;
}

void MotorControl_ServiceBackground(void)
{
    FocBackgroundClaim claim;
    /* 补偿请求只读取稳定配置；维护占用时不运行整表 CRC。 */
    FocRunState_ServiceCompensation();
    if (!FocRunState_BeginBackground(&claim))
    {
        return;
    }
    bool success = false;
    if (claim.kind == FOC_BACKGROUND_SAVE)
    {
        InterfaceParam_TypeDef *record = HEAP_malloc(sizeof(*record));
        if (record != NULL)
        {
            Param_Upload(record, &MotorControl, &OnBoard_Encoder);
            record->magic_word = MAGIC_WORD;
            uint32_t mask = critical_hw_enter();
            if (FocRunState_BackgroundValid(&claim))
            {
                success = param_store_write(record, sizeof(*record));
            }
            critical_hw_exit(mask);
            HEAP_free(record);
        }
    }
    else if (claim.kind == FOC_BACKGROUND_COGGING)
    {
        CoggingMapRecord *candidate = HEAP_malloc(sizeof(*candidate));
        if (candidate != NULL)
        {
            bool built = FocCogging_BuildCandidate(candidate, &MotorControl, &OnBoard_Encoder);
            uint32_t mask = critical_hw_enter();
            if (built && FocRunState_BackgroundValid(&claim))
            {
                success = FocCogging_PublishCandidate(candidate);
            }
            critical_hw_exit(mask);
            HEAP_free(candidate);
        }
    }
    uint32_t mask = critical_hw_enter();
    bool current = FocRunState_BackgroundValid(&claim);
    FocRunState_FinishBackground(&claim, success);
    if (save_job.valid && save_job.ticket == claim.epoch && claim.kind == FOC_BACKGROUND_SAVE)
    {
        save_job.result = !current  ? PARAMETER_CANCELLED
                          : success ? PARAMETER_OK
                                    : PARAMETER_PERSIST_FAILED;
        if (current && success)
        {
            save_job.persisted_revision = save_job.request.expected_revision;
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
    if (last_write.valid && source == last_write.request.source_node &&
        (uint16_t)(sequence - last_write.request.sequence) >= 0x8000U)
    {
        return true;
    }
    if (save_job.valid && source == save_job.request.source_node &&
        (uint16_t)(sequence - save_job.request.sequence) >= 0x8000U)
    {
        return true;
    }
    return false;
}

ParameterStatus MotorControl_ReadParameter(const ParameterReadRequest *request,
                                           ParameterValue *value)
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
    if (last_read.valid && request->source_node == last_read.request.source_node &&
        request->sequence == last_read.request.sequence)
    {
        result = request->parameter_id == last_read.request.parameter_id
                     ? PARAMETER_OK
                     : PARAMETER_CONFLICTING_RETRY;
        if (result == PARAMETER_OK)
        {
            *value = last_read.value;
        }
    }
    else if ((last_write.valid && request->source_node == last_write.request.source_node &&
              request->sequence == last_write.request.sequence) ||
             (save_job.valid && request->source_node == save_job.request.source_node &&
              request->sequence == save_job.request.sequence))
    {
        result = PARAMETER_CONFLICTING_RETRY;
    }
    else
    {
        AppLifecycleState state = FocRunState_GetState();
        result = state == APP_MAINTENANCE || state == APP_STARTING || state == APP_STOPPING
                     ? PARAMETER_BUSY
                     : ParameterValue_Read(&MotorControl,
                                           &OnBoard_Encoder,
                                           config_revision,
                                           Protocol_NodeId_Get(),
                                           request->parameter_id,
                                           value);
        if (result == PARAMETER_OK)
        {
            if (!FocRunState_ConfigurationAllowed())
            {
                value->access &= (uint8_t)~PARAMETER_ACCESS_WRITE_IDLE;
            }
            last_read.request = *request;
            last_read.value = *value;
            last_read.valid = true;
        }
    }
    critical_hw_exit(mask);
    return result;
}

ParameterStatus MotorControl_WriteParameter(const ParameterWriteRequest *request,
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
    ParameterStatus result;
    if (last_write.valid && request->source_node == last_write.request.source_node &&
        request->sequence == last_write.request.sequence)
    {
        result =
            same_write(request, &last_write.request) ? PARAMETER_OK : PARAMETER_CONFLICTING_RETRY;
        if (result == PARAMETER_OK)
        {
            *applied = last_write.applied;
        }
    }
    else if (save_job.valid && request->source_node == save_job.request.source_node &&
             request->sequence == save_job.request.sequence)
    {
        result = PARAMETER_CONFLICTING_RETRY;
    }
    else if (last_read.valid && request->source_node == last_read.request.source_node &&
             request->sequence == last_read.request.sequence)
    {
        result = PARAMETER_CONFLICTING_RETRY;
    }
    else if (stale_sequence(request->source_node, request->sequence))
    {
        result = PARAMETER_STALE_SEQUENCE;
    }
    else if (MotorControl.ErrorNow != No_Error)
    {
        result = PARAMETER_FAULT;
    }
    else if (!FocRunState_ConfigurationAllowed() ||
             (save_job.valid && save_job.result == PARAMETER_ACCEPTED))
    {
        result = PARAMETER_WRONG_STATE;
    }
    else if (request->expected_revision != config_revision || config_revision == UINT32_MAX)
    {
        result = PARAMETER_REVISION_CONFLICT;
    }
    else
    {
        result = ParameterValue_Apply(&MotorControl, Protocol_NodeId_Get(), request);
        if (result == PARAMETER_OK)
        {
            advance_config_revision();
            /* 原有控制器复位边界，不在这里重写控制数学或增益公式。 */
            Task_Position_Mode_Reset();
            FOC_CurrentController_Reset(&FOC);
            PI_Controller_Reset(&PI_Speed);
            result = ParameterValue_Read(&MotorControl,
                                         &OnBoard_Encoder,
                                         config_revision,
                                         Protocol_NodeId_Get(),
                                         request->parameter_id,
                                         applied);
            if (result == PARAMETER_OK)
            {
                last_write.request = *request;
                last_write.applied = *applied;
                last_write.valid = true;
            }
        }
    }
    critical_hw_exit(mask);
    return result;
}

ParameterStatus MotorControl_RequestSave(const ParameterSaveRequest *request, uint32_t *ticket)
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
    ParameterStatus result;
    if (save_job.valid && request->source_node == save_job.request.source_node &&
        request->sequence == save_job.request.sequence)
    {
        result = request->expected_revision == save_job.request.expected_revision
                     ? save_job.result
                     : PARAMETER_CONFLICTING_RETRY;
        if (result != PARAMETER_CONFLICTING_RETRY)
        {
            *ticket = save_job.ticket;
        }
    }
    else if (last_write.valid && request->source_node == last_write.request.source_node &&
             request->sequence == last_write.request.sequence)
    {
        result = PARAMETER_CONFLICTING_RETRY;
    }
    else if (last_read.valid && request->source_node == last_read.request.source_node &&
             request->sequence == last_read.request.sequence)
    {
        result = PARAMETER_CONFLICTING_RETRY;
    }
    else if (stale_sequence(request->source_node, request->sequence))
    {
        result = PARAMETER_STALE_SEQUENCE;
    }
    else if (save_job.valid && save_job.result == PARAMETER_ACCEPTED)
    {
        result = PARAMETER_BUSY;
    }
    else if (request->expected_revision != config_revision)
    {
        result = PARAMETER_REVISION_CONFLICT;
    }
    else
    {
        MotorControlResult accepted =
            FocRunState_RequestMaintenance(MOTOR_MAINTENANCE_SAVE, ticket);
        result = accepted == MOTOR_CONTROL_ACCEPTED ? PARAMETER_ACCEPTED
                 : accepted == MOTOR_CONTROL_BUSY   ? PARAMETER_BUSY
                 : accepted == MOTOR_CONTROL_FAULT  ? PARAMETER_FAULT
                                                    : PARAMETER_WRONG_STATE;
        if (result == PARAMETER_ACCEPTED)
        {
            save_job.request = *request;
            save_job.ticket = *ticket;
            save_job.persisted_revision = 0U;
            save_job.result = result;
            save_job.valid = true;
        }
    }
    critical_hw_exit(mask);
    return result;
}

ParameterStatus MotorControl_PollSave(uint32_t ticket, uint32_t *persisted_revision)
{
    if (persisted_revision == NULL)
    {
        return PARAMETER_INVALID_ARGUMENT;
    }
    *persisted_revision = 0U;
    uint32_t mask = critical_hw_enter();
    ParameterStatus result = PARAMETER_STALE_TICKET;
    if (save_job.valid && ticket != 0U && ticket == save_job.ticket)
    {
        if (save_job.result == PARAMETER_ACCEPTED)
        {
            MotorMaintenanceProgress progress;
            if (!FocRunState_ReadMaintenance(ticket, &progress) ||
                progress.result == APP_OPERATION_CANCELLED)
            {
                save_job.result = PARAMETER_CANCELLED;
            }
            else if (progress.result == APP_OPERATION_FAILED)
            {
                save_job.result = PARAMETER_PERSIST_FAILED;
            }
        }
        result = save_job.result;
        if (result == PARAMETER_OK)
        {
            *persisted_revision = save_job.persisted_revision;
        }
    }
    critical_hw_exit(mask);
    return result;
}

bool MotorControl_SaveIdle(void)
{
    uint32_t mask = critical_hw_enter();
    if (save_job.valid && save_job.result == PARAMETER_ACCEPTED)
    {
        uint32_t persisted_revision;
        (void)MotorControl_PollSave(save_job.ticket, &persisted_revision);
    }
    bool idle = !save_job.valid || save_job.result != PARAMETER_ACCEPTED;
    critical_hw_exit(mask);
    return idle;
}
