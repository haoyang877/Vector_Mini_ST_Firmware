#include "motor_state.h"
#include <math.h>
#include "critical_hw.h"
#include "foc_mode_dispatch.h"
#include "foc_encoder_calibration.h"
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
    work_outcome = FocMode_Dispatch(
        &FOC, &MotorControl, &PI_Speed, &OnBoard_Encoder, &Fluxobserver, &SensorlessStartup);

    /* 9. worker 结果入箱：由 2 kHz 主状态机在下一慢拍消费。 */
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
    FocRunState_FinishBackground(&claim, success);
}
