#include "foc_task.h"

#include "common_inc.h"
#include "SEGGER_RTT.h"
#include "foc_friction_identification.h"
#include "foc_phase_resistance.h"
#include "position_cascade.h"
#include "position_cascade_config.h"
#include "servo_hil.h"
#include "fast_loop_profile.h"
#include "motor_status.h"
#include "motor_load_control.h"
#include "motor_hw.h"
#include "motor_thermal_control.h"
#include "time_hw.h"

#if FOC_FREQ != 20000
#error "Thermal sample window requires the validated 20 kHz motor fast loop"
#endif
#if SPEED_LOOP_DIVIDER != 10
#error "Sensorless phase scheduling requires the validated 2 kHz speed loop"
#endif

MotorControl_TypeDef MotorControl;
PI_Controller_TypeDef PI_Speed;
Encoder_TypeDef OnBoard_Encoder;
Fluxobserver_TypeDef Fluxobserver;
SensorlessStartup_TypeDef SensorlessStartup;

ModeNow_TypeDef  ModeLast  = Motor_Disable;
ErrorNow_TypeDef ErrorLast = No_Error;

FOC_TypeDef FOC;

/* CAN 单字发布，快环独占消费和运行态修改；UINT32_MAX 表示无请求。 */
static volatile uint32_t pending_load_profile = UINT32_MAX;

/**
 * @brief 复位应用运行态并取消未完成标定，保留已提交标定、参数和热状态。
 * @note 仅快环在停机清理或停机切换装配档时调用；不操作 PWM、不写 Flash。
 */
static void task_reset_runtime(void)
{
    EncoderCalibration_Reset();
    if (ModeLast == Calib_Friction)
    {
        FocFrictionIdentification_Abort(&MotorControl, &PI_Speed);
    }
    MotorControl.idRef = 0.0f;
    MotorControl.iqRef = 0.0f;
    MotorControl.vqRef = 0.0f;
    MotorControl.speedRef = 0.0f;
    MotorControl.speedShadow = 0.0f;
    MotorControl.posShadow = 0.0f;
    MotorControl.isReachTargetPos = false;
    MotorControl.pos_vel_filtered = 0.0f;
    Task_Position_Mode_Reset();

    FOC.Id = 0.0f;
    FOC.Iq = 0.0f;
    FOC_CurrentController_Reset(&FOC);
    PI_Controller_Reset(&PI_Speed);
    SensorlessStartup_Reset(&SensorlessStartup);
}

bool MotorControl_SetSpeedFeedforward(const SpeedFeedforwardConfig *config)
{
    if (!SpeedFeedforward_IsValid(config) || MotorControl.ModeNow != Motor_Disable ||
        ModeLast != Motor_Disable || MotorControl.ErrorNow != No_Error ||
        pending_load_profile != UINT32_MAX || !motor_hw_phase_outputs_disabled())
    {
        return false;
    }
    MotorControl.speed_ff_config = *config;
    SpeedFeedforward_Reset(&MotorControl.speed_ff_state);
    return true;
}

bool MotorControl_RequestLoadProfile(uint32_t flags)
{
    if (!MotorLoadProfile_IsValid(flags) || MotorControl.ModeNow != Motor_Disable ||
        ModeLast != Motor_Disable || MotorControl.ErrorNow != No_Error ||
        pending_load_profile != UINT32_MAX || !motor_hw_phase_outputs_disabled())
    {
        return false;
    }
    motor_hw_outer_barrier();
    pending_load_profile = flags;
    return true;
}

int32_t MotorControl_GetLoadProfile(void)
{
    if (pending_load_profile != UINT32_MAX || !MotorControl.load_profile_valid ||
        !MotorLoadProfile_IsValid(MotorControl.load_profile_flags))
    {
        return -1;
    }
    return (int32_t)MotorControl.load_profile_flags;
}

static void MotorControl_ApplyLoadProfile(void)
{
    uint32_t flags = pending_load_profile;
    if (flags == UINT32_MAX)
    {
        return;
    }
    pending_load_profile = UINT32_MAX;
    motor_hw_outer_barrier();
    /* 提交与消费之间可能收到 START；再次检查后才修改控制参数。 */
    if (MotorControl.ModeNow != Motor_Disable || ModeLast != Motor_Disable ||
        MotorControl.ErrorNow != No_Error || !motor_hw_phase_outputs_disabled() ||
        !MotorLoadProfile_IsValid(flags))
    {
        return;
    }
    task_reset_runtime();
    Fluxobserver_ParamInit(&Fluxobserver);
    if (!MotorControl.load_profile_valid ||
        ((MotorControl.load_profile_flags ^ flags) & MOTOR_LOAD_DAMPING_RING) != 0U)
    {
        MotorControl.friction_model_valid = false;
        FocFrictionIdentification_Init();
    }
    MotorControl.load_profile_flags = flags;
    MotorControl.load_profile_valid = true;
    /* 装配切换恢复该档默认精度；测量噪声约±0.025°，阻尼档取±0.08°。 */
    MotorControl.pos_error_window = MotorLoadProfile_PassiveHoldEnabled(flags) ?
        POSITION_SERVO_DAMPED_ERROR_WINDOW_RAD : 0.001f;
}

#define RTT_SPEED_SCALE_COUNTS_PER_RAD_S	(18000.0f / _PI)
#define RTT_POSITION_SCALE_COUNTS_PER_RAD	(18000.0f / _PI)
#define RTT_CURRENT_SCALE_COUNTS_PER_A		1000.0f


static int16_t RTT_EncodeInt16(float value, float scale)
{
	float scaled;

	/* 所有调用点的比例均为有限编译期常量，仅采样值需要检查。 */
	if (!isfinite(value))
	{
		return 0;
	}
	scaled = value * scale;
	if (scaled > 32767.0f)
		return 32767;
	if (scaled < -32768.0f)
		return -32768;

	return (int16_t)scaled;
}

typedef struct
{
    int16_t data0;
    int16_t data1;
    int16_t data2;
    int16_t data3;
    int16_t data4;
    int16_t data5;
    int16_t data6;
    int16_t data7;
    int16_t data8;
    int16_t data9;
#if RTT_TELEMETRY_PROFILE != RTT_TELEMETRY_MANUAL_DIAGNOSTICS
    int16_t data10;
#endif
} RTT_Data_TypeDef;

#if RTT_TELEMETRY_PROFILE == RTT_TELEMETRY_MANUAL_DIAGNOSTICS
typedef char RTT_DataFrame_SizeMustBe20Bytes[
    (sizeof(RTT_Data_TypeDef) == 20U) ? 1 : -1];
/* PendSV 独占的 RTT 环形缓冲丢帧计数；不占用波形通道。 */
static volatile uint32_t rtt_manual_dropped_frames;
#else
typedef char RTT_DataFrame_SizeMustBe22Bytes[
    (sizeof(RTT_Data_TypeDef) == 22U) ? 1 : -1];
#endif

/* 单核快中断生产、PendSV 消费；所有共享访问 volatile 有序。
 * ready 为真时快中断只丢帧，服务编码完局部帧后才释放，不锁中断、不覆盖半帧。 */
typedef struct
{
    float current_reference_a;
    float current_feedback_a;
    float d_current_a;
#if RTT_TELEMETRY_PROFILE == RTT_TELEMETRY_MANUAL_DIAGNOSTICS
    float position_reference_rad;
    float position_feedback_rad;
    float position_error_rad;
    float speed_reference_rad_s;
    float control_speed_rad_s;
    float measured_speed_rad_s;
    float friction_current_a;
#else
    float q_voltage_v;
    float d_voltage_v;
    float encoder_electrical_rad;
    float observer_electrical_rad;
    float phase_a_a;
    float phase_b_a;
    float phase_c_a;
#endif
} RTT_Snapshot;

static volatile RTT_Snapshot rtt_snapshot;
static volatile bool rtt_snapshot_ready;
/* 快中断独占写；总丢帧为本计数加诊断模式的 rtt_manual_dropped_frames。 */
static volatile uint32_t rtt_capture_dropped_frames;

static void RTT_Sampling(void)
{
    /* 仅改变首帧相位：2kHz采样与编码器慢拍、1kHz热更新错开半周期。 */
    static uint32_t rtt_divider_count = RTT_SAMPLE_DIVIDER / 2U;

    if (++rtt_divider_count < RTT_SAMPLE_DIVIDER)
    {
        return;
    }
    rtt_divider_count = 0U;
    if (rtt_snapshot_ready)
    {
        ++rtt_capture_dropped_frames;
        return;
    }

    rtt_snapshot.current_reference_a = MotorControl.iqRef;
    rtt_snapshot.current_feedback_a = FOC.Iq;
    rtt_snapshot.d_current_a = FOC.Id;
#if RTT_TELEMETRY_PROFILE == RTT_TELEMETRY_MANUAL_DIAGNOSTICS
    /* 只读同一快拍及最近外环快照；停机时补偿电流明确清零。 */
    {
        PositionCascadeTelemetry_TypeDef planned;
        float speed_reference_rad_s = 0.0f;
        float control_speed_rad_s = MotorControl.pos_vel_filtered;
        float friction_current_a = 0.0f;

        if (MotorControl.ModeNow == Position_Mode && MotorOuterLoop_GetTelemetry(&planned))
        {
            speed_reference_rad_s = planned.speed_command;
            control_speed_rad_s = planned.speed_feedback;
            friction_current_a = planned.friction_feedforward_current;
        }
        rtt_snapshot.position_reference_rad = MotorControl.posRef;
        rtt_snapshot.position_feedback_rad = OnBoard_Encoder.theta_mech;
        rtt_snapshot.position_error_rad = MotorControl.posRef - OnBoard_Encoder.theta_mech;
        rtt_snapshot.speed_reference_rad_s = speed_reference_rad_s;
        rtt_snapshot.control_speed_rad_s = control_speed_rad_s;
        rtt_snapshot.measured_speed_rad_s = OnBoard_Encoder.vel_mech;
        rtt_snapshot.friction_current_a = friction_current_a;
    }
#else
    rtt_snapshot.q_voltage_v = FOC.Vq;
    rtt_snapshot.d_voltage_v = FOC.Vd;
    rtt_snapshot.encoder_electrical_rad = OnBoard_Encoder.theta_elec;
    rtt_snapshot.observer_electrical_rad = Fluxobserver.theta_e;
    rtt_snapshot.phase_a_a = FOC.Ia;
    rtt_snapshot.phase_b_a = FOC.Ib;
    rtt_snapshot.phase_c_a = FOC.Ic;
#endif
    rtt_snapshot_ready = true;
    motor_hw_outer_schedule();
}

void FOC_TelemetryService(void)
{
    RTT_Data_TypeDef frame;

    if (!rtt_snapshot_ready)
    {
        return;
    }
#if RTT_TELEMETRY_PROFILE == RTT_TELEMETRY_MANUAL_DIAGNOSTICS
    frame.data0 = RTT_EncodeInt16(rtt_snapshot.position_reference_rad,
        RTT_POSITION_SCALE_COUNTS_PER_RAD);
    frame.data1 = RTT_EncodeInt16(rtt_snapshot.position_feedback_rad,
        RTT_POSITION_SCALE_COUNTS_PER_RAD);
    frame.data2 = RTT_EncodeInt16(rtt_snapshot.position_error_rad,
        RTT_POSITION_SCALE_COUNTS_PER_RAD);
    frame.data3 = RTT_EncodeInt16(rtt_snapshot.speed_reference_rad_s,
        RTT_SPEED_SCALE_COUNTS_PER_RAD_S);
    frame.data4 = RTT_EncodeInt16(rtt_snapshot.control_speed_rad_s,
        RTT_SPEED_SCALE_COUNTS_PER_RAD_S);
    frame.data5 = RTT_EncodeInt16(rtt_snapshot.measured_speed_rad_s,
        RTT_SPEED_SCALE_COUNTS_PER_RAD_S);
    frame.data6 = RTT_EncodeInt16(rtt_snapshot.current_reference_a,
        RTT_CURRENT_SCALE_COUNTS_PER_A);
    frame.data7 = RTT_EncodeInt16(rtt_snapshot.current_feedback_a,
        RTT_CURRENT_SCALE_COUNTS_PER_A);
    frame.data8 = RTT_EncodeInt16(rtt_snapshot.d_current_a, RTT_CURRENT_SCALE_COUNTS_PER_A);
    frame.data9 = RTT_EncodeInt16(rtt_snapshot.friction_current_a,
        RTT_CURRENT_SCALE_COUNTS_PER_A);
#else
    const float ang_scale = 32767.0f / (2.0f * _PI);
    float obs = rtt_snapshot.observer_electrical_rad;
    float encoder_angle = rtt_snapshot.encoder_electrical_rad;
    float diff;

    frame.data0 = RTT_EncodeInt16(rtt_snapshot.current_reference_a, RTT_CURRENT_SCALE_COUNTS_PER_A);
    frame.data1 = RTT_EncodeInt16(rtt_snapshot.current_feedback_a, RTT_CURRENT_SCALE_COUNTS_PER_A);
    frame.data2 = RTT_EncodeInt16(rtt_snapshot.d_current_a, RTT_CURRENT_SCALE_COUNTS_PER_A);
    frame.data3 = RTT_EncodeInt16(rtt_snapshot.q_voltage_v, 1000.0f);
    frame.data4 = RTT_EncodeInt16(rtt_snapshot.d_voltage_v, 1000.0f);
    frame.data5 = RTT_EncodeInt16(encoder_angle, ang_scale);
    /* 原始同拍电角只在低优先级编码时归位，比例与旧快环编码一致。 */
    if (obs < 0.0f)
    {
        obs += _2PI;
    }
    frame.data6 = RTT_EncodeInt16(obs, ang_scale);
    diff = obs - encoder_angle;
    if (diff > _PI)
    {
        diff -= _2PI;
    }
    else if (diff < -_PI)
    {
        diff += _2PI;
    }
    frame.data7 = RTT_EncodeInt16(diff, 32767.0f / _PI);
    frame.data8 = RTT_EncodeInt16(rtt_snapshot.phase_a_a, RTT_CURRENT_SCALE_COUNTS_PER_A);
    frame.data9 = RTT_EncodeInt16(rtt_snapshot.phase_b_a, RTT_CURRENT_SCALE_COUNTS_PER_A);
    frame.data10 = RTT_EncodeInt16(rtt_snapshot.phase_c_a, RTT_CURRENT_SCALE_COUNTS_PER_A);
#endif
    /* 局部帧已完整；发送期间快中断可以发布下一帧，不会改写本次发送内容。 */
    rtt_snapshot_ready = false;
#if RTT_TELEMETRY_PROFILE == RTT_TELEMETRY_MANUAL_DIAGNOSTICS
    /* 通道1只有此低优先级服务写；v6.30b SKIP成功返回1，丢帧返回0。 */
    if (SEGGER_RTT_WriteSkipNoLock(1, &frame, sizeof(frame)) == 0U)
    {
        ++rtt_manual_dropped_frames;
    }
#else
    SEGGER_RTT_WriteSkipNoLock(1, &frame, sizeof(frame));
#endif
}

/** Read-only publication from the motor owner. Requested at <=200 Hz; CAN
 * packing and transmission stay in the foreground, never in this fast path. */
static void MotorStatus_Sampling(void)
{
    MotorStatus sample;
    PositionCascadeTelemetry_TypeDef planned;
    if (!MotorStatus_IsRequested()) return;
    sample.fault = (uint16_t)MotorControl.ErrorNow;
    sample.mode = (uint16_t)MotorControl.ModeNow;
    sample.position_target = MotorControl.posRef;
    sample.position_feedback = OnBoard_Encoder.theta_mech;
    sample.speed_target = MotorControl.speedRef;
    sample.speed_feedback = (MotorControl.ModeNow == Position_Mode ||
        MotorControl.ModeNow == Position_Impedance_Mode) ?
        MotorControl.pos_vel_filtered : OnBoard_Encoder.vel_mech;
    sample.current_reference = MotorControl.iqRef;
    sample.current_feedback = FOC.Iq;
    sample.temperature = FOC.temp;
    sample.bus_voltage = FOC.Vbus_filt;
    sample.bus_current = FOC.Ibus_filt;
    sample.position_planned = NAN;
    sample.speed_planned = NAN;
    if (MotorControl.ModeNow == Position_Mode && MotorOuterLoop_GetTelemetry(&planned)) {
        sample.position_planned = planned.position_reference;
        sample.speed_planned = planned.trajectory_speed_reference;
    } else if (MotorControl.ModeNow == Position_Impedance_Mode) {
        sample.position_planned = MotorControl.posShadow;
        sample.speed_planned = MotorControl.speedShadow;
    } else if (MotorControl.ModeNow == Speed_Mode) {
        sample.speed_planned = MotorControl.speedShadow;
    }
    MotorStatus_Publish(&sample);
}

/**
	* @brief  Initialize motor control parameters
 **/
bool MotorControl_IsConfigurationValid(void)
{
	return MotorControl.axis_profile_valid && MotorControl.load_profile_valid;
}

void MotorControl_Init(void)
{
    MotorThermalControl_Init(time_hw_now_ms());
	Encoder_ParamInit(&OnBoard_Encoder);
	
	Fluxobserver_ParamInit(&Fluxobserver);
	SensorlessStartup_Reset(&SensorlessStartup);
	FOC_CurrentController_Reset(&FOC);
	PI_Controller_Reset(&PI_Speed);
	
	MotorControl.pos_error_window = MotorLoadProfile_PassiveHoldEnabled(
		MotorControl.load_profile_flags) ? POSITION_SERVO_DAMPED_ERROR_WINDOW_RAD : 0.001f;
	MotorControl.pos_vel_filtered = 0.0f;
	Task_Position_Mode_Reset();
	FocFrictionIdentification_Init();
	
	/* Unconfigured joint records remain disabled at boot. */
	MotorControl.ModeNow = MotorControl_IsConfigurationValid() ? Calib_CurrentOffset : Motor_Disable;
	if (!MotorControl_IsConfigurationValid())
		Set_ErrorNow(MotorParam_Error);
}

/**
	* @brief  FOC task, motor control related
			  use finite state machine
 **/
static void Task_SetMechanicalZero(MotorControl_TypeDef *MotorControl, Encoder_TypeDef *Encoder)
{
	if (!Encoder_SetMechanicalZero(Encoder))
	{
		Set_ErrorNow(Encoder_Error);
		return;
	}

	Task_Position_Mode_Reset();
	Set_ModeNow(Save_Param);
}

/* 清其他故障不绕过热锁存，清绕组故障必须真正满足冷却资格。 */
static void Task_TryClearError(void)
{
    if (MotorThermalControl_ClearFault() || MotorControl.ErrorNow != MotorThermal_Protection)
    {
        Set_ErrorNow(No_Error);
    }
    Set_ModeNow(Motor_Disable);
}

static bool Encoder_FeedbackRequired(const MotorControl_TypeDef *MotorControl)
{
	if (MotorControl->ModeNow == Current_Mode)
		return !MotorControl->isUseSensorless;

	return MotorControl->ModeNow == Speed_Mode ||
	       MotorControl->ModeNow == Position_Mode ||
	       MotorControl->ModeNow == Position_Impedance_Mode ||
	       MotorControl->ModeNow == Vq_Mode ||
	       MotorControl->ModeNow == Calib_EncoderOffset ||
	       MotorControl->ModeNow == Calib_EncoderObserver ||
	       MotorControl->ModeNow == Calib_EleAngelOffset ||
	       MotorControl->ModeNow == Calib_Friction ||
	       MotorControl->ModeNow == Set_ZeroPosition;
}

/* 快上下文最终准入，前台/调试器改变模式也不能跳过此处。 */
static void Task_ApplyThermalGuard(void)
{
    bool voltage_mode = MotorControl.ModeNow == Voltage_OpenLoop || MotorControl.ModeNow == Vq_Mode ||
        MotorControl.ModeNow == Calib_EncoderOffset || MotorControl.ModeNow == Calib_PhaseResistance;
    bool current_boundary_exceeded = false;
    /* 命令可能刚改配置上限，本拍最终边界必须按新配置重新发布。 */
    MotorControl.thermal_current_limit_a = MotorThermalControl_CurrentLimit(MotorControl.current_limit);
    MotorControl.thermal_limit_active = MotorControl.thermal_current_limit_a < MotorControl.current_limit;
    if (voltage_mode)
    {
        float limit_a = MotorControl.thermal_current_limit_a;
        float measured_vector_squared_a2 = (2.0f / 3.0f) *
            (FOC.Ia * FOC.Ia + FOC.Ib * FOC.Ib + FOC.Ic * FOC.Ic);
        current_boundary_exceeded = !isfinite(measured_vector_squared_a2) ||
            measured_vector_squared_a2 > limit_a * limit_a;
    }
    /* 电压控制无法通过参考限流，实测越过边界时直接退出。 */
    if (!MotorThermalControl_Guard((uint32_t)MotorControl.ModeNow) ||
        current_boundary_exceeded)
    {
        if (MotorControl.ErrorNow == No_Error)
        {
            Set_ErrorNow(MotorThermalControl_ParametersConfirmed() ?
                MotorThermal_Protection : MotorParam_Error);
        }
        MotorControl.idRef = 0.0f;
        MotorControl.iqRef = 0.0f;
        Set_ModeNow(Motor_Disable);
    }
}

void FOC1kHzSupervisor(void)
{
	/* MCU 温度换算及保护由 1 kHz 监督任务执行。 */
	Temperature_Update(&FOC, MotorThermalControl_ProtectionEnabled());
    /* ADC快中断消失时仍能停机；不在此上下文清/改热模型。 */
    if (!MotorThermalControl_CheckWatchdog(time_hw_now_ms(), motor_hw_phase_outputs_disabled()))
    {
        if (MotorControl.ErrorNow == No_Error)
        {
            Set_ErrorNow(MotorThermal_Protection);
        }
        Set_ModeNow(Motor_Disable);
        Stop_PWM_Generate();
    }
}

/* 交接首拍选择一次 PI 相位；后续仍每10拍运行，不改热样本窗。
 * 入参均在本拍采样/编码器完成后读取。返回下一次PI调用前的分频计数。 */
static uint32_t FOC_SelectSensorlessSpeedPhase(uint32_t thermal_remaining_fast_ticks,
    uint32_t encoder_velocity_divider)
{
    uint32_t thermal_phase;
    uint32_t encoder_phase;
    uint32_t first_phase;
    uint32_t second_phase;
    uint32_t first_distance;
    uint32_t second_distance;
    uint32_t selected_phase;

    if (thermal_remaining_fast_ticks == 0U ||
        thermal_remaining_fast_ticks > 2U * SPEED_LOOP_DIVIDER ||
        encoder_velocity_divider >= SPEED_LOOP_DIVIDER)
    {
        return 0U;
    }
    thermal_phase = thermal_remaining_fast_ticks % SPEED_LOOP_DIVIDER;
    encoder_phase = (SPEED_LOOP_DIVIDER - encoder_velocity_divider) % SPEED_LOOP_DIVIDER;
    /* 两个候选距热更新3拍；首拍不叠加PI，遇0向邻拍偏移后仍至少隔2拍。 */
    first_phase = (thermal_phase + 3U) % SPEED_LOOP_DIVIDER;
    second_phase = (thermal_phase + 7U) % SPEED_LOOP_DIVIDER;
    if (first_phase == 0U)
    {
        first_phase = SPEED_LOOP_DIVIDER - 1U;
    }
    if (second_phase == 0U)
    {
        second_phase = 1U;
    }
    first_distance = (first_phase + SPEED_LOOP_DIVIDER - encoder_phase) % SPEED_LOOP_DIVIDER;
    second_distance = (second_phase + SPEED_LOOP_DIVIDER - encoder_phase) % SPEED_LOOP_DIVIDER;
    if (first_distance > SPEED_LOOP_DIVIDER / 2U)
    {
        first_distance = SPEED_LOOP_DIVIDER - first_distance;
    }
    if (second_distance > SPEED_LOOP_DIVIDER / 2U)
    {
        second_distance = SPEED_LOOP_DIVIDER - second_distance;
    }
    selected_phase = first_distance >= second_distance ? first_phase : second_phase;
    /* 调用中的前置++也占一拍；首次PI在交接第2～10拍，随后固定10拍。 */
    return SPEED_LOOP_DIVIDER - 1U - selected_phase;
}

void FOC20kHzIRQHandler(void)
{
	static bool position_start_prepared;
	bool defer_power_start = false;
	bool observer_power_start_ready = false;
	bool defer_optional_telemetry = false;
	bool encoder_sample_started;
    uint8_t encoder_sample_stage;
    bool current_sample_valid;
	FAST_PROFILE_BEGIN(FAST_PROFILE_EMPTY);
	FAST_PROFILE_END(FAST_PROFILE_EMPTY);
	FAST_PROFILE_BEGIN(FAST_PROFILE_ENCODER_REQUEST);
	encoder_sample_started = Encoder_BeginSample();
	FAST_PROFILE_END(FAST_PROFILE_ENCODER_REQUEST);
	FAST_PROFILE_BEGIN(FAST_PROFILE_SENSING);
	Vbus_Update(&FOC, &MotorControl);
	
    current_sample_valid = Current_Cal(&FOC, &MotorControl);
    encoder_sample_stage = Encoder_BeginReadback(encoder_sample_started);
    MotorThermalControl_Sample(current_sample_valid ? FOC.Ia : NAN,
        current_sample_valid ? FOC.Ib : NAN, current_sample_valid ? FOC.Ic : NAN,
        motor_hw_phase_outputs_disabled(), time_hw_now_ms());
    /* 最终热边界在命令处理后的 Task_ApplyThermalGuard 中统一发布。 */
	#if SERVO_HIL_ENABLE
	ServoHil_ObservePhaseCurrents(FOC.Ia, FOC.Ib, FOC.Ic);
	#endif
	FAST_PROFILE_END(FAST_PROFILE_SENSING);

    MotorControl_ApplyLoadProfile();
    /* 观测器只消费本拍相电流，与 SPI 响应传输重叠；编码器仍在同拍控制前完成。 */
    if (!SERVO_HIL_ENABLE && MotorControl.ModeNow != Position_Mode)
    {
        Fluxobserver_Update(&FOC, &MotorControl, &Fluxobserver);
    }
	
	FAST_PROFILE_BEGIN(FAST_PROFILE_ENCODER);
	Encoder_CompleteSample(&MotorControl, &OnBoard_Encoder, encoder_sample_stage);
	FAST_PROFILE_END(FAST_PROFILE_ENCODER);
	FAST_PROFILE_BEGIN(FAST_PROFILE_COMMANDS);
#if SERVO_HIL_ENABLE
	{
		/* Debug mailbox at 10 kHz; count both fast ticks for the watchdog.
		 * ARM occurs here, so the divided servo subsequently falls between polls. */
		static uint8_t hil_divider;
		if (++hil_divider >= 2U) {
			defer_optional_telemetry = true;
			ServoHilCommand command = ServoHil_Poll(Encoder_GetMecPos(&OnBoard_Encoder),
				Encoder_GetMecVelContinuous(&OnBoard_Encoder), FOC.Iq,
				(uint32_t)MotorControl.ModeNow, (uint32_t)MotorControl.ErrorNow, FOC_FREQ, 2U);
			bool accepted = true;
			hil_divider = 0U;
			switch (command.action) {
			case SERVO_HIL_STOP: Set_ModeNow(Motor_Disable); break;
			case SERVO_HIL_ARM: accepted = ModeSwitch_Handle(Position_Mode); break;
			case SERVO_HIL_POSITION: MotorControl.posRef = command.value; break;
			case SERVO_HIL_POSITION_KP: MotorControl.cascade_pos_Kp = command.value; break;
			case SERVO_HIL_POSITION_KD: MotorControl.cascade_pos_Kd = command.value; break;
			case SERVO_HIL_HOLD_FILTER:
				MotorControl.position_hold_filter_bypass = command.value == 0.0f;
				MotorControl.position_hold_filter_half_cutoff = command.value == 2.0f;
				break;
			case SERVO_HIL_VELOCITY_FILTER:
				MotorControl.position_velocity_filter_half_cutoff = command.value == 1.0f;
				break;
			case SERVO_HIL_SPEED_KP: MotorControl.position_speed_Kp = command.value; break;
			case SERVO_HIL_SPEED_KI: MotorControl.position_speed_Ki = command.value; break;
			case SERVO_HIL_MAX_SPEED:
				accepted = command.value <= MotorControl.speed_limit;
				if (accepted) MotorControl.pos_maxspeed = command.value;
				break;
			default: break;
			}
			ServoHil_Complete(accepted);
		}
	}
#endif
	/* HIL 命令使用本拍编码器，须在命令分派后选择观测器，避免 mode 3 入场重排。 */
	if (SERVO_HIL_ENABLE && MotorControl.ModeNow != Position_Mode)
		Fluxobserver_Update(&FOC, &MotorControl, &Fluxobserver);

	if (Encoder_FeedbackRequired(&MotorControl) &&
		OnBoard_Encoder.bad_frame_streak >= ENCODER_BAD_FRAME_OFFLINE_COUNT)
		Set_ErrorNow(Encoder_Error);

	/* Also cover internal mode assignments, not only communication requests. */
	if ((!MotorControl_IsConfigurationValid() ||
         !MotorLoadProfile_IsValid(MotorControl.load_profile_flags)) &&
        MotorControl.ModeNow != Motor_Disable &&
		MotorControl.ModeNow != Save_Param && MotorControl.ModeNow != Clear_Error)
	{
		Set_ErrorNow(MotorParam_Error);
		Set_ModeNow(Motor_Disable);
	}
    Task_ApplyThermalGuard();
	FAST_PROFILE_END(FAST_PROFILE_COMMANDS);
	MotorOuterLoop_FastTick(&MotorControl, &PI_Speed, &OnBoard_Encoder);

    if ((MotorControl.ModeNow == Calib_EncoderObserver ||
         MotorControl.ModeNow == Sensorless_Speed_Mode) &&
        SensorlessStartup.state == SENSORLESS_STARTUP_HANDOFF &&
        SensorlessStartup.state_ticks == 0U)
    {
        SensorlessStartup.speed_loop_ticks = FOC_SelectSensorlessSpeedPhase(
            MotorThermalControl_SamplesUntilUpdate(), OnBoard_Encoder.velocity_divider);
    }
	switch(MotorControl.ModeNow)
	{
		case Motor_Disable:
			PhaseResistanceMode_Cancel(&FOC, &MotorControl);
			/* Outputs stay disabled. Prime an equal-duty zero vector before
			 * the next enable, rather than switching from 100% preload during
			 * the first current-sampling window. Use the existing PWM adapter. */
			Set_A_Duty(0.5f);
			Set_B_Duty(0.5f);
			Set_C_Duty(0.5f);
		break;
		
		case Current_Mode:
			Task_Current_Mode(&FOC, &MotorControl, &OnBoard_Encoder, &Fluxobserver);
		break;
		
		case Speed_Mode:
			FOC_Current(&FOC, &MotorControl, Encoder_GetElePhase(&OnBoard_Encoder),
                Encoder_GetEleVel(&OnBoard_Encoder));
		break;

		case Sensorless_Speed_Mode:
		{
            const MotorCalibrationProfile *profile =
                MotorLoadProfile_Calibration(MotorControl.load_profile_flags);
			Task_Sensorless_Speed_Mode(&FOC, &MotorControl, &PI_Speed, &Fluxobserver,
				&SensorlessStartup, (MotorControl.load_profile_flags & MOTOR_LOAD_DAMPING_RING) != 0U ?
                    &profile->startup : &SensorlessStartup_DefaultConfig);
		}
		break;
		
		case Position_Mode:
			FAST_PROFILE_BEGIN(FAST_PROFILE_POSITION_WITH_CURRENT);
			FOC_Current(&FOC, &MotorControl, Encoder_GetElePhase(&OnBoard_Encoder),
                Encoder_GetEleVel(&OnBoard_Encoder));
			FAST_PROFILE_END(FAST_PROFILE_POSITION_WITH_CURRENT);
		break;

		case Position_Impedance_Mode:
			/* 编号保留；直接调试写入已裁剪模式时也必须走停机路径。 */
			MotorControl.idRef = 0.0f;
			MotorControl.iqRef = 0.0f;
			Set_ErrorNow(MotorParam_Error);
			Set_ModeNow(Motor_Disable);
		break;

		case Calib_Friction:
			FocFrictionIdentification_Task(&FOC, &MotorControl, &PI_Speed, &OnBoard_Encoder,
                MotorThermalControl_ProtectionEnabled());
		break;
		
		case Calib_Motor_R_L_Flux:
			Task_Calib_R_L_Flux(&FOC, &MotorControl);
		break;
		
		case Calib_PhaseResistance:
		{
			PhaseResistanceModeStatus_TypeDef status =
				PhaseResistanceMode_Run(&FOC, &MotorControl);

			if (status == PHASE_RESISTANCE_MODE_DONE)
				Set_ModeNow(Motor_Disable);
			else if (status == PHASE_RESISTANCE_MODE_SETTLE_TIMEOUT)
				Set_ErrorNow(Large_Phase_Resistance);
			else if (status == PHASE_RESISTANCE_MODE_INVALID_RESULT)
				Set_ErrorNow(MotorParam_Error);
			else if (status == PHASE_RESISTANCE_MODE_UNDER_VOLTAGE)
				Set_ErrorNow(Under_Voltage);
			else if (status == PHASE_RESISTANCE_MODE_OVER_VOLTAGE)
				Set_ErrorNow(Over_Voltage);
			break;
		}

		case Calib_EncoderOffset:
			Task_Calib_EncoderOffset(&FOC, &MotorControl, &OnBoard_Encoder, &Fluxobserver);
		break;

		case Calib_EncoderObserver:
			/* 只能使用拍前准备状态；准备完成与PWM启用不能挤在同一拍。 */
			observer_power_start_ready = EncoderObserverCalibration_IsPowerStartReady();
			Task_Calib_EncoderObserver(&FOC, &MotorControl, &PI_Speed, &OnBoard_Encoder,
				&Fluxobserver, &SensorlessStartup);
		break;

		case Calib_EleAngelOffset:
			Task_Calib_EleAngelOffset(&FOC, &MotorControl, &OnBoard_Encoder);
		break;
		
		case Calib_CurrentOffset:
			Task_Calib_CurrentOffset(&FOC, &MotorControl);
		break;
		
		case Voltage_OpenLoop:
			Task_Voltage_Mode(&FOC, &MotorControl);
		break;
		
		case Vq_Mode:
			Task_Vq_Mode(&FOC, &MotorControl, &OnBoard_Encoder);
		break;

		case Set_ZeroPosition:
			Task_SetMechanicalZero(&MotorControl, &OnBoard_Encoder);
		break;
		
		case Default_Param:
			Param_Return_Default();
		
		case Clear_Error:
            Task_TryClearError();
		default:break;
	}
	
	FAST_PROFILE_BEGIN(FAST_PROFILE_POST_CONTROL);
	/*no error*/
	if(MotorControl.ErrorNow == No_Error)
	{
		LED_SetState(0, (uint8_t)MotorControl.ModeNow);
	}
	/*error*/
	else
	{
		/*tolerant when writing paramters to flash*/
		if(MotorControl.ModeNow != Save_Param && MotorControl.ModeNow != Default_Param)
			MotorControl.ModeNow = Motor_Disable;
		
		LED_SetState(1, (uint8_t)MotorControl.ErrorNow);
	}
	
	/* 准备拍尚未更新 ModeLast；STOP 也必须释放待启动标定状态。 */
	if (MotorControl.ModeNow == Motor_Disable &&
		(ModeLast != Motor_Disable || !motor_hw_phase_outputs_disabled() ||
		 EncoderObserverCalibration_IsPowerStartReady()))
	{
		task_reset_runtime();
		Stop_PWM_Generate();
	}
	/* 设零与保存不需要功率输出；包括标定结束转入保存的同一快拍。 */
	if ((MotorControl.ModeNow == Set_ZeroPosition || MotorControl.ModeNow == Save_Param) &&
		!motor_hw_phase_outputs_disabled())
	{
		Stop_PWM_Generate();
	}
	
	if(ModeLast == Motor_Disable && MotorControl.ModeNow != Motor_Disable &&
		MotorControl.ModeNow != Set_ZeroPosition && MotorControl.ModeNow != Save_Param &&
		MotorControl.axis_profile_valid)
	{
		/* The first mode-3 tick validates/initializes the controller while
		 * phase outputs are still off. Enable on the following fast tick,
		 * after the neutral preload and without combining both startup costs. */
		if (MotorControl.ModeNow == Calib_EncoderObserver && !observer_power_start_ready)
		{
			defer_power_start = true;
		}
		else if ((MotorControl.ModeNow == Position_Mode || MotorControl.ModeNow == Speed_Mode) &&
            (!position_start_prepared || !MotorOuterLoop_IsReady()))
		{
			position_start_prepared = true;
			defer_power_start = true;
		}
		else if (MotorControl.ModeNow == Damping_Mode)
			Start_Damping_Brake();
		else
			Start_PWM_Generate();
	}
	if (!defer_power_start)
		position_start_prepared = false;
	
	Detect_Mode_Error_Change();
	
	if (!defer_power_start)
		ModeLast = MotorControl.ModeNow;
	ErrorLast = MotorControl.ErrorNow;
	
	MotorControl.ModeNow_f = MotorControl.ModeNow;
	MotorControl.ErrorNow_f = MotorControl.ErrorNow;

    if (!defer_optional_telemetry) MotorStatus_Sampling();
    RTT_Sampling();

}

