#include "foc_task.h"

#include "common_inc.h"
#include "SEGGER_RTT.h"

MotorControl_TypeDef MotorControl;
PI_Controller_TypeDef PI_Speed;
Encoder_TypeDef External_Encoder;
Fluxobserver_TypeDef Fluxobserver;
SensorlessStartup_TypeDef SensorlessStartup;

ModeNow_TypeDef  ModeLast  = Motor_Disable;
ErrorNow_TypeDef ErrorLast = No_Error;

FOC_TypeDef FOC;

void RTT_Sampling(void)
{
	static uint32_t rtt_divider_count;

	if(++rtt_divider_count < RTT_SAMPLE_DIVIDER)
		return;
	rtt_divider_count = 0;

    struct {
    int16_t data0;
    int16_t data1;
    int16_t data2;
    int16_t data3;
    int16_t data4;
    int16_t data5;
    int16_t data6;
    int16_t data7;
    int16_t data8;
    } Rttstru;

	/*RTT channels: speed x100, current/voltage x1000, encoder and open-loop angles mapped to int16 full scale.*/
	Rttstru.data0 = (int16_t)(MotorControl.speedShadow * 100.0f);
	Rttstru.data1 = (int16_t)(External_Encoder.vel_mech * 100.0f);
	Rttstru.data2 = (int16_t)(MotorControl.iqRef * 1000.0f);
	Rttstru.data3 = (int16_t)(FOC.Iq * 1000.0f);
	Rttstru.data4 = (int16_t)(FOC.Id * 1000.0f);
	Rttstru.data5 = (int16_t)(FOC.mod_q * FOC.Vbus_filt / 1.5f * 1000.0f);
	Rttstru.data6 = (int16_t)(FOC.mod_d * FOC.Vbus_filt / 1.5f * 1000.0f);
	Rttstru.data7 = (int16_t)(normalizeAngle(External_Encoder.theta_elec) * (65536.0f / _2PI) - 32768.0f);
	Rttstru.data8 = (int16_t)(normalizeAngle(MotorControl.ol_theta) * (65536.0f / _2PI) - 32768.0f);
    
    SEGGER_RTT_Write(1, &Rttstru, sizeof(Rttstru));
}

/**
	* @brief  Initialize motor control parameters
 **/
void MotorControl_Init(void)
{	Encoder_ParamInit(&External_Encoder);
	
	Fluxobserver_ParamInit(&Fluxobserver);
	SensorlessStartup_Reset(&SensorlessStartup);
	FOC_CurrentController_Reset(&FOC);
	PI_Controller_Reset(&PI_Speed);
	
	MotorControl.posTrajUpdated = true;
	MotorControl.pos_error_window = 0.001f;
	MotorControl.pos_Kp = 0.5f;
	MotorControl.pos_Kd = 0.1f;
	
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

	MotorControl->posTrajUpdated = true;
	Set_ModeNow(Save_Param);
}

static bool Encoder_FeedbackRequired(const MotorControl_TypeDef *MotorControl)
{
	if (MotorControl->ModeNow == Current_Mode)
		return !MotorControl->isUseSensorless;

	return MotorControl->ModeNow == Speed_Mode ||
	       MotorControl->ModeNow == Position_Mode ||
	       MotorControl->ModeNow == Vq_Mode ||
	       MotorControl->ModeNow == Calib_EncoderOffset ||
	       MotorControl->ModeNow == Calib_EncoderObserver ||
	       MotorControl->ModeNow == Calib_EleAngelOffset ||
	       MotorControl->ModeNow == Set_ZeroPosition;
}

void FOC20kHzIRQHandler(void)
{
	Vbus_Update(&FOC, &MotorControl);
	
	Current_Cal(&FOC, &MotorControl);
	
	Temperature_Update(&FOC);
	
	Encoder_Update(&MotorControl, &External_Encoder);
	Fluxobserver_Update(&FOC, &MotorControl, &Fluxobserver);

	if (Encoder_FeedbackRequired(&MotorControl) &&
		External_Encoder.bad_frame_streak >= ENCODER_BAD_FRAME_OFFLINE_COUNT)
		Set_ErrorNow(Encoder_Error);

	switch(MotorControl.ModeNow)
	{
		case Motor_Disable:
			PWM_TurnOnHighSides();
		break;
		
		case Current_Mode:
			Task_Current_Mode(&FOC, &MotorControl, &External_Encoder, &Fluxobserver);
		break;
		
		case Speed_Mode:
			Task_Speed_Mode(&FOC, &MotorControl, &PI_Speed, &External_Encoder);
		break;

		case Sensorless_Speed_Mode:
			Task_Sensorless_Speed_Mode(&FOC, &MotorControl, &PI_Speed, &Fluxobserver, &SensorlessStartup);
		break;
		
		case Position_Mode:
			Task_Position_Mode(&FOC, &MotorControl, &PI_Speed, &External_Encoder);
		break;
		
		case Calib_Motor_R_L_Flux:
			Task_Calib_R_L_Flux(&FOC, &MotorControl);
		break;
		
		case Calib_EncoderOffset:
			Task_Calib_EncoderOffset(&FOC, &MotorControl, &External_Encoder, &Fluxobserver);
		break;

		case Calib_EncoderObserver:
			Task_Calib_EncoderObserver(&FOC, &MotorControl, &PI_Speed, &External_Encoder,
				&Fluxobserver, &SensorlessStartup);
		break;

		case Calib_EleAngelOffset:
			Task_Calib_EleAngelOffset(&FOC, &MotorControl, &External_Encoder);
		break;
		
		case Calib_CurrentOffset:
			Task_Calib_CurrentOffset(&FOC, &MotorControl);
		break;
		
		case Voltage_OpenLoop:
			Task_Voltage_Mode(&FOC, &MotorControl);
		break;
		
		case Vq_Mode:
			Task_Vq_Mode(&FOC, &MotorControl, &External_Encoder);
		break;

		case Set_ZeroPosition:
			Task_SetMechanicalZero(&MotorControl, &External_Encoder);
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
