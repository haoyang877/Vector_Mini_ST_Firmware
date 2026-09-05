#include "foc_task.h"

#include "common_inc.h"
#include "SEGGER_RTT.h"
#include "foc_friction_identification.h"
#include "foc_phase_resistance.h"

MotorControl_TypeDef MotorControl;
PI_Controller_TypeDef PI_Speed;
Encoder_TypeDef OnBoard_Encoder;
Fluxobserver_TypeDef Fluxobserver;
SensorlessStartup_TypeDef SensorlessStartup;

ModeNow_TypeDef  ModeLast  = Motor_Disable;
ErrorNow_TypeDef ErrorLast = No_Error;

FOC_TypeDef FOC;

#define RTT_SPEED_SCALE_COUNTS_PER_RAD_S	10000.0f
#define RTT_CURRENT_SCALE_COUNTS_PER_A		1000.0f
#define RTT_ANGLE_Q15_SCALE				(32768.0f / _PI)

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

void RTT_Sampling(void)
{
	static uint32_t rtt_divider_count;

	if(++rtt_divider_count < RTT_SAMPLE_DIVIDER)
		return;
	rtt_divider_count = 0;

    struct {
        int16_t position_ref;
        int16_t position_feedback;
        int16_t speed_ref;
        int16_t speed_feedback;
        int16_t iq_ref;
        int16_t iq_feedback;
    } Rttstru;

	/* Fixed six-channel frame for stable host-side parsing in every motor mode. */
	Rttstru.position_ref = RTT_EncodeAngleQ15(MotorControl.posShadow);
	Rttstru.position_feedback = RTT_EncodeAngleQ15(OnBoard_Encoder.theta_mech);
	Rttstru.speed_ref = RTT_EncodeInt16(MotorControl.speedShadow,
		RTT_SPEED_SCALE_COUNTS_PER_RAD_S);
	Rttstru.speed_feedback = RTT_EncodeInt16(OnBoard_Encoder.vel_mech,
		RTT_SPEED_SCALE_COUNTS_PER_RAD_S);
	Rttstru.iq_ref = RTT_EncodeInt16(MotorControl.iqRef,
		RTT_CURRENT_SCALE_COUNTS_PER_A);
	Rttstru.iq_feedback = RTT_EncodeInt16(FOC.Iq,
		RTT_CURRENT_SCALE_COUNTS_PER_A);
    
    SEGGER_RTT_Write(1, &Rttstru, sizeof(Rttstru));
}

/**
	* @brief  Initialize motor control parameters
 **/
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
	
	/*run current offset calibration automatically at power-up*/
	MotorControl.ModeNow = Calib_CurrentOffset;
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

void FOC20kHzIRQHandler(void)
{
	Vbus_Update(&FOC, &MotorControl);
	
	Current_Cal(&FOC, &MotorControl);
	
	Temperature_Update(&FOC);
	
	Encoder_Update(&MotorControl, &OnBoard_Encoder);
	Fluxobserver_Update(&FOC, &MotorControl, &Fluxobserver);

	if (Encoder_FeedbackRequired(&MotorControl) &&
		OnBoard_Encoder.bad_frame_streak >= ENCODER_BAD_FRAME_OFFLINE_COUNT)
		Set_ErrorNow(Encoder_Error);

	switch(MotorControl.ModeNow)
	{
		case Motor_Disable:
			PhaseResistanceMode_Cancel(&FOC, &MotorControl);
			PWM_TurnOnHighSides();
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
	
	if(ModeLast == Motor_Disable && MotorControl.ModeNow != Motor_Disable)
	{
		Start_PWM_Generate();
	}
	
	Detect_Mode_Error_Change();
	
	ModeLast  = MotorControl.ModeNow;
	ErrorLast = MotorControl.ErrorNow;
	
	MotorControl.ModeNow_f = MotorControl.ModeNow;
	MotorControl.ErrorNow_f = MotorControl.ErrorNow;
    
    RTT_Sampling();
}
