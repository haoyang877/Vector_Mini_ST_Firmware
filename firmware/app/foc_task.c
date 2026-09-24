#include "foc_task.h"

#include "common_inc.h"
#include "SEGGER_RTT.h"
#include "foc_friction_identification.h"
#include "foc_phase_resistance.h"
#include "position_cascade.h"
#include "servo_hil.h"
#include "fast_loop_profile.h"
#include "motor_status.h"

MotorControl_TypeDef MotorControl;
PI_Controller_TypeDef PI_Speed;
Encoder_TypeDef OnBoard_Encoder;
Fluxobserver_TypeDef Fluxobserver;
SensorlessStartup_TypeDef SensorlessStartup;

ModeNow_TypeDef  ModeLast  = Motor_Disable;
ErrorNow_TypeDef ErrorLast = No_Error;

FOC_TypeDef FOC;

#define RTT_SPEED_SCALE_COUNTS_PER_RAD_S	(18000.0f / _PI)
#define RTT_POSITION_SCALE_COUNTS_PER_RAD	(18000.0f / _PI)
#define RTT_CURRENT_SCALE_COUNTS_PER_A		1000.0f


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
	int16_t data10;
} RTT_Data_TypeDef;

typedef char RTT_DataFrame_SizeMustBe22Bytes[
    (sizeof(RTT_Data_TypeDef) == 22U) ? 1 : -1];

static void RTT_Sampling(void)
{
	static uint32_t rtt_divider_count;

	if (++rtt_divider_count < RTT_SAMPLE_DIVIDER)
		return;
	rtt_divider_count = 0U; /* 必须清零，否则达到门限后每次都通过 */

	RTT_Data_TypeDef frame;
	const float ang_scale = 32767.0f / (2.0f * _PI);
	float obs;
	float diff;

	frame.data0 = RTT_EncodeInt16(MotorControl.iqRef, RTT_CURRENT_SCALE_COUNTS_PER_A);
	frame.data1 = RTT_EncodeInt16(FOC.Iq, RTT_CURRENT_SCALE_COUNTS_PER_A);
	frame.data2 = RTT_EncodeInt16(FOC.Id, RTT_CURRENT_SCALE_COUNTS_PER_A);
	frame.data3 = RTT_EncodeInt16(FOC.Vq, 1000.0f);
	frame.data4 = RTT_EncodeInt16(FOC.Vd, 1000.0f);
	frame.data5 = RTT_EncodeInt16(OnBoard_Encoder.theta_elec, ang_scale);
	/* 观测器电角度：归到 [0,2π) 后与编码器电角度同刻度；差值单次有界归位到 ±π
	 * （ISR 中不允许循环等待）。差值按 ±180°↔±32767 归一（0.0055°/计数，不饱和）。 */
	obs = Fluxobserver.theta_e;
	if (obs < 0.0f)
		obs += _2PI;
	frame.data6 = RTT_EncodeInt16(obs, ang_scale);
	diff = obs - OnBoard_Encoder.theta_elec;
	if (diff > _PI)
		diff -= _2PI;
	else if (diff < -_PI)
		diff += _2PI;
	frame.data7 = RTT_EncodeInt16(diff, 32767.0f / _PI);
	frame.data8 = RTT_EncodeInt16(FOC.Ia, RTT_CURRENT_SCALE_COUNTS_PER_A);
	frame.data9 = RTT_EncodeInt16(FOC.Ib, RTT_CURRENT_SCALE_COUNTS_PER_A);
	frame.data10 = RTT_EncodeInt16(FOC.Ic, RTT_CURRENT_SCALE_COUNTS_PER_A);
	SEGGER_RTT_Write(1, &frame, sizeof(frame));
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
	bool encoder_sample_started;
	FAST_PROFILE_BEGIN(FAST_PROFILE_EMPTY);
	FAST_PROFILE_END(FAST_PROFILE_EMPTY);
	FAST_PROFILE_BEGIN(FAST_PROFILE_ENCODER_REQUEST);
	encoder_sample_started = Encoder_BeginSample();
	FAST_PROFILE_END(FAST_PROFILE_ENCODER_REQUEST);
	FAST_PROFILE_BEGIN(FAST_PROFILE_SENSING);
	Vbus_Update(&FOC, &MotorControl);
	
	Current_Cal(&FOC, &MotorControl);
	#if SERVO_HIL_ENABLE
	ServoHil_ObservePhaseCurrents(FOC.Ia, FOC.Ib, FOC.Ic);
	#endif
	FAST_PROFILE_END(FAST_PROFILE_SENSING);
	
	FAST_PROFILE_BEGIN(FAST_PROFILE_ENCODER);
	Encoder_CompleteSample(&MotorControl, &OnBoard_Encoder, encoder_sample_started);
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
	FAST_PROFILE_END(FAST_PROFILE_COMMANDS);
	MotorOuterLoop_FastTick(&MotorControl, &PI_Speed, &OnBoard_Encoder);
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
			Task_Sensorless_Speed_Mode(&FOC, &MotorControl, &PI_Speed, &Fluxobserver,
				&SensorlessStartup, &SensorlessStartup_DefaultConfig);
		break;
		
		case Position_Mode:
			FAST_PROFILE_BEGIN(FAST_PROFILE_POSITION_WITH_CURRENT);
			FOC_Current(&FOC, &MotorControl, Encoder_GetElePhase(&OnBoard_Encoder),
                Encoder_GetEleVel(&OnBoard_Encoder));
			FAST_PROFILE_END(FAST_PROFILE_POSITION_WITH_CURRENT);
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
		if ((MotorControl.ModeNow == Position_Mode || MotorControl.ModeNow == Speed_Mode) &&
            (!position_start_prepared || !MotorOuterLoop_IsReady()))
		{
			position_start_prepared = true;
			defer_position_power_start = true;
		}
		else if (MotorControl.ModeNow == Damping_Mode)
			Start_Damping_Brake();
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

    if (!defer_optional_telemetry) MotorStatus_Sampling();
    RTT_Sampling();

}

