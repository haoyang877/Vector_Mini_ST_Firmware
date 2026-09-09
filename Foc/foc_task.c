#include "foc_task.h"

#include "common_inc.h"
#include "SEGGER_RTT.h"
#include "foc_friction_identification.h"
#include "foc_phase_resistance.h"
#include "position_cascade.h"
#include "servo_hil.h"

MotorControl_TypeDef MotorControl;
PI_Controller_TypeDef PI_Speed;
Encoder_TypeDef OnBoard_Encoder;
Fluxobserver_TypeDef Fluxobserver;
SensorlessStartup_TypeDef SensorlessStartup;

ModeNow_TypeDef  ModeLast  = Motor_Disable;
ErrorNow_TypeDef ErrorLast = No_Error;

FOC_TypeDef FOC;

#define RTT_SPEED_SCALE_COUNTS_PER_RAD_S	10000.0f
#define RTT_POSITION_ERROR_SCALE_COUNTS_PER_RAD	10000.0f
#define RTT_CURRENT_SCALE_COUNTS_PER_A		1000.0f
#define RTT_ANGLE_Q15_SCALE				(32768.0f / _PI)

#define RTT_SERVO_STATUS_PHASE_MASK		0x0003U
#define RTT_SERVO_STATUS_PHASE_INVALID	0x0003U
#define RTT_SERVO_STATUS_TARGET_REACHED	(1U << 2)
#define RTT_SERVO_STATUS_CURRENT_SATURATED	(1U << 3)
#define RTT_SERVO_STATUS_FEEDFORWARD_ACTIVE	(1U << 4)
#define RTT_SERVO_STATUS_FAULT_ACTIVE	(1U << 5)
#define RTT_SERVO_STATUS_PREVIOUS_FRAME_DROPPED	(1U << 6)
#define RTT_SERVO_STATUS_TELEMETRY_VALID	(1U << 7)
#define RTT_SERVO_STATUS_FRAME_VERSION_1	(1U << 8)
#define RTT_SERVO_STATUS_FRICTION_LANDING	(1U << 9)
#define RTT_SERVO_STATUS_SETTLE_RECOVERY	(1U << 10)
#define RTT_SERVO_STATUS_HOLD_CANDIDATE	(1U << 11)
#define RTT_SERVO_STATUS_TRAJECTORY_LIMITED	(1U << 12)
#define RTT_SERVO_STATUS_STICTION_INTEGRATING	(1U << 13)

typedef struct
{
	int16_t trajectory_position;
	int16_t position_feedback;
	int16_t position_error;
	int16_t trajectory_speed;
	int16_t speed_command;
	int16_t speed_feedback;
	int16_t iq_reference;
	int16_t iq_feedback;
	int16_t feedback_current;
	int16_t feedforward_current;
	int16_t hold_current;
	int16_t servo_status;
} RTT_ControlFrame_TypeDef;

typedef char RTT_ControlFrame_SizeMustBe24Bytes[
	(sizeof(RTT_ControlFrame_TypeDef) == 24U) ? 1 : -1];

static int16_t RTT_EncodeInt16(float value, float scale)
{
	float scaled;

	if (!isfinite(value) || !isfinite(scale))
		return 0;
	scaled = value * scale;
	if (scaled > 32767.0f)
		return 32767;
	if (scaled < -32768.0f)
		return -32768;

	return (int16_t)scaled;
}

static int16_t RTT_EncodeAngleQ15(float angle)
{
	if (!isfinite(angle))
		return 0;

	/* Signed Q15 angle: 0 rad -> 0; wrap occurs at +/-pi. */
	angle = normalizeAngle(angle);
	if (angle >= _PI)
		angle -= _2PI;
	return RTT_EncodeInt16(angle, RTT_ANGLE_Q15_SCALE);
}

static void RTT_Sampling(bool defer_encoding)
{
	static uint32_t rtt_divider_count;
	static bool previous_frame_dropped;
	PositionCascadeTelemetry_TypeDef servo_telemetry;
	RTT_ControlFrame_TypeDef frame;
	uint16_t status_flags = RTT_SERVO_STATUS_FRAME_VERSION_1 |
		RTT_SERVO_STATUS_PHASE_INVALID;
	unsigned bytes_written;
	bool servo_telemetry_valid;

	if(++rtt_divider_count < RTT_SAMPLE_DIVIDER)
		return;
	/* Preserve the divider clock while deferring optional encoding away from
	 * the HIL mailbox/diagnostic publication tick as well as the servo tick. */
	if (defer_encoding)
		return;
	if (Encoder_DidUpdateVelocity(&OnBoard_Encoder))
		return;
	/* Do not stack frame encoding on the same IRQ as the 2 kHz servo.
	 * A coincident frame is deferred by one fast tick; subsequent frames keep
	 * their normal cadence. This does not delay the current/PWM update. */
#if CASCADE_POSITION_LOOP_DIVIDER > 1U
	if (MotorControl.ModeNow == Position_Mode && PositionCascade_ShouldDeferTelemetry())
		return;
#endif
	rtt_divider_count = 0;

	/* This fixed frame is intentionally meaningful only in mode 3. */
	frame.trajectory_position = 0;
	frame.position_feedback = 0;
	frame.position_error = 0;
	frame.trajectory_speed = 0;
	frame.speed_command = 0;
	frame.speed_feedback = 0;
	frame.iq_reference = 0;
	frame.iq_feedback = 0;
	frame.feedback_current = 0;
	frame.feedforward_current = 0;
	frame.hold_current = 0;
	frame.servo_status = 0;

	servo_telemetry_valid = MotorControl.ModeNow == Position_Mode &&
		PositionCascade_GetTelemetry(&servo_telemetry);

	if (servo_telemetry_valid)
	{
		status_flags &= (uint16_t)~RTT_SERVO_STATUS_PHASE_MASK;
		status_flags |= (uint16_t)servo_telemetry.phase &
			RTT_SERVO_STATUS_PHASE_MASK;
		status_flags |= RTT_SERVO_STATUS_TELEMETRY_VALID;
		frame.trajectory_position = RTT_EncodeAngleQ15(
			servo_telemetry.position_reference);
		frame.position_feedback = RTT_EncodeAngleQ15(
			OnBoard_Encoder.theta_mech);
		frame.position_error = RTT_EncodeInt16(
			servo_telemetry.position_reference - OnBoard_Encoder.theta_mech,
			RTT_POSITION_ERROR_SCALE_COUNTS_PER_RAD);
		frame.trajectory_speed = RTT_EncodeInt16(
			servo_telemetry.trajectory_speed_reference,
			RTT_SPEED_SCALE_COUNTS_PER_RAD_S);
		frame.speed_command = RTT_EncodeInt16(servo_telemetry.speed_command,
			RTT_SPEED_SCALE_COUNTS_PER_RAD_S);
		frame.speed_feedback = RTT_EncodeInt16(servo_telemetry.speed_feedback,
			RTT_SPEED_SCALE_COUNTS_PER_RAD_S);
		frame.iq_reference = RTT_EncodeInt16(MotorControl.iqRef,
			RTT_CURRENT_SCALE_COUNTS_PER_A);
		frame.iq_feedback = RTT_EncodeInt16(FOC.Iq,
			RTT_CURRENT_SCALE_COUNTS_PER_A);
		frame.feedback_current = RTT_EncodeInt16(
			servo_telemetry.feedback_current,
			RTT_CURRENT_SCALE_COUNTS_PER_A);
		frame.feedforward_current = RTT_EncodeInt16(
			MotorControl.iqRef - servo_telemetry.feedback_current,
			RTT_CURRENT_SCALE_COUNTS_PER_A);
		frame.hold_current = RTT_EncodeInt16(servo_telemetry.hold_current,
			RTT_CURRENT_SCALE_COUNTS_PER_A);
		if (servo_telemetry.target_reached)
			status_flags |= RTT_SERVO_STATUS_TARGET_REACHED;
		if (servo_telemetry.current_saturated)
			status_flags |= RTT_SERVO_STATUS_CURRENT_SATURATED;
		if (servo_telemetry.friction_landing_active)
			status_flags |= RTT_SERVO_STATUS_FRICTION_LANDING;
		if (servo_telemetry.settle_recovery_active)
			status_flags |= RTT_SERVO_STATUS_SETTLE_RECOVERY;
		if (servo_telemetry.hold_candidate_active)
			status_flags |= RTT_SERVO_STATUS_HOLD_CANDIDATE;
		if (servo_telemetry.trajectory_limited)
			status_flags |= RTT_SERVO_STATUS_TRAJECTORY_LIMITED;
		if (servo_telemetry.stiction_integrating)
			status_flags |= RTT_SERVO_STATUS_STICTION_INTEGRATING;
	}
	if (frame.feedforward_current != 0)
		status_flags |= RTT_SERVO_STATUS_FEEDFORWARD_ACTIVE;
	if (MotorControl.ErrorNow != No_Error)
		status_flags |= RTT_SERVO_STATUS_FAULT_ACTIVE;
	if (previous_frame_dropped)
		status_flags |= RTT_SERVO_STATUS_PREVIOUS_FRAME_DROPPED;
	frame.servo_status = (int16_t)status_flags;

	bytes_written = SEGGER_RTT_Write(1, &frame, sizeof(frame));
	previous_frame_dropped = bytes_written != sizeof(frame);
}

/**
	* @brief  Initialize motor control parameters
 **/
bool MotorControl_IsConfigurationValid(void)
{
	return MotorControl.axis_profile_valid;
}

void MotorControl_Init(void)
{	Encoder_ParamInit(&OnBoard_Encoder);
	
	Fluxobserver_ParamInit(&Fluxobserver);
	SensorlessStartup_Reset(&SensorlessStartup);
	FOC_CurrentController_Reset(&FOC);
	PI_Controller_Reset(&PI_Speed);
	
	MotorControl.pos_error_window = 0.001f;
	MotorControl.pos_vel_filtered = 0.0f;
	Task_Position_Mode_Reset();
	FocFrictionIdentification_Init();
	
	/* Unconfigured joint records remain disabled at boot. */
	MotorControl.ModeNow = MotorControl.axis_profile_valid ? Calib_CurrentOffset : Motor_Disable;
	if (!MotorControl.axis_profile_valid)
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

void FOC1kHzSupervisor(void)
{
	/* Temperature conversion includes logf and belongs to the slow supervisor. */
	Temperature_Update(&FOC);
}

void FOC20kHzIRQHandler(void)
{
	static bool position_start_prepared;
	bool defer_position_power_start = false;
	bool defer_optional_telemetry = false;
	Vbus_Update(&FOC, &MotorControl);
	
	Current_Cal(&FOC, &MotorControl);
	#if SERVO_HIL_ENABLE
	ServoHil_ObservePhaseCurrents(FOC.Ia, FOC.Ib, FOC.Ic);
	#endif
	
	Encoder_Update(&MotorControl, &OnBoard_Encoder);
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
			case SERVO_HIL_SPEED_KP: MotorControl.speed_Kp = command.value; break;
			case SERVO_HIL_SPEED_KI: MotorControl.speed_Ki = command.value; break;
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
	/* Select after command dispatch, including the first mode-3 IRQ. */
	if (MotorControl.ModeNow != Position_Mode)
		Fluxobserver_Update(&FOC, &MotorControl, &Fluxobserver);

	if (Encoder_FeedbackRequired(&MotorControl) &&
		OnBoard_Encoder.bad_frame_streak >= ENCODER_BAD_FRAME_OFFLINE_COUNT)
		Set_ErrorNow(Encoder_Error);

	/* Also cover internal mode assignments, not only communication requests. */
	if (!MotorControl.axis_profile_valid && MotorControl.ModeNow != Motor_Disable &&
		MotorControl.ModeNow != Save_Param && MotorControl.ModeNow != Clear_Error)
	{
		Set_ErrorNow(MotorParam_Error);
		Set_ModeNow(Motor_Disable);
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
			Task_Speed_Mode(&FOC, &MotorControl, &PI_Speed, &OnBoard_Encoder);
		break;

		case Sensorless_Speed_Mode:
			Task_Sensorless_Speed_Mode(&FOC, &MotorControl, &PI_Speed, &Fluxobserver,
				&SensorlessStartup, &SensorlessStartup_DefaultConfig);
		break;
		
		case Position_Mode:
			Task_Position_Mode(&FOC, &MotorControl, &OnBoard_Encoder);
		break;

		case Position_Impedance_Mode:
			Task_Position_Impedance_Mode(&FOC, &MotorControl, &OnBoard_Encoder);
		break;

		case Calib_Friction:
			FocFrictionIdentification_Task(&FOC, &MotorControl, &PI_Speed, &OnBoard_Encoder);
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
			Set_ErrorNow(No_Error);
		default:break;
	}
	
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
	
	if(ModeLast != Motor_Disable && MotorControl.ModeNow == Motor_Disable) 
	{
		Clear_RunningData();
		Stop_PWM_Generate();
	}
	
	if(ModeLast == Motor_Disable && MotorControl.ModeNow != Motor_Disable &&
		MotorControl.axis_profile_valid)
	{
		/* The first mode-3 tick validates/initializes the controller while
		 * phase outputs are still off. Enable on the following fast tick,
		 * after the neutral preload and without combining both startup costs. */
		if (MotorControl.ModeNow == Position_Mode && !position_start_prepared)
		{
			position_start_prepared = true;
			defer_position_power_start = true;
		}
		else
			Start_PWM_Generate();
	}
	if (!defer_position_power_start)
		position_start_prepared = false;
	
	Detect_Mode_Error_Change();
	
	if (!defer_position_power_start)
		ModeLast = MotorControl.ModeNow;
	ErrorLast = MotorControl.ErrorNow;
	
	MotorControl.ModeNow_f = MotorControl.ModeNow;
	MotorControl.ErrorNow_f = MotorControl.ErrorNow;
    
    RTT_Sampling(defer_optional_telemetry);
}
