#include "foc_run.h"

#include "common_inc.h"

/**
	* @brief  Current mode control task
	* @param  *FOC: FOC struct pointer
	* @param  *MotorControl: MotorControl struct pointer
	* @param  *Encoder: encoder struct pointer
	* @param  *Fluxobserver: flux observer struct pointer
 **/
void Task_Current_Mode(FOC_TypeDef *FOC, MotorControl_TypeDef *MotorControl, Encoder_TypeDef *Encoder, Fluxobserver_TypeDef *Fluxobserver)
{
	float theta_elec;
	float vel_elec;

	if(MotorControl->isUseSensorless == true)
	{
		theta_elec = Observer_GetElePhase(Fluxobserver);
		vel_elec = 0.0f;
	}
	else
	{
		theta_elec = Encoder_GetElePhase(Encoder);
		vel_elec = Encoder_GetEleVel(Encoder);		
	}

	FOC_Current(FOC, MotorControl, theta_elec, vel_elec);
}

/**
	* @brief  Speed mode control task
	* @param  *FOC: FOC struct pointer
	* @param  *MotorControl: MotorControl struct pointer
	* @param  *PID: PID struct pointer
	* @param  *Encoder: encoder struct pointer
 **/
void Task_Speed_Mode(FOC_TypeDef *FOC, MotorControl_TypeDef *MotorControl, PID_TypeDef *PID, Encoder_TypeDef *Encoder)
{
	static int speedloop_count;
	
	float theta_elec;
	float vel_elec,vel_mech;
	
	theta_elec = Encoder_GetElePhase(Encoder);
	vel_elec = Encoder_GetEleVel(Encoder);
	vel_mech = Encoder_GetMecVel(Encoder); 
	
	if(++speedloop_count >= 2)
	{
		if(MotorControl->speedAcc == 0.0f || MotorControl->speedDec == 0.0f)
			MotorControl->isUseSpeedRamp = false;
		else
			MotorControl->isUseSpeedRamp = true;
		
		/*enable speed ramp*/
		if(MotorControl->isUseSpeedRamp == true)
		{
			if(MotorControl->speedRef > MotorControl->speedShadow)
			{
				MotorControl->speedShadow += MotorControl->speedAcc * _2PI * Speed_Ts;
				if(MotorControl->speedShadow > MotorControl->speedRef)
					MotorControl->speedShadow = MotorControl->speedRef;
			}
			else if(MotorControl->speedRef < MotorControl->speedShadow)
			{
					MotorControl->speedShadow -= MotorControl->speedDec * _2PI * Speed_Ts;
					if(MotorControl->speedShadow < MotorControl->speedRef)
						MotorControl->speedShadow = MotorControl->speedRef;			
			}
		}
		else
		{
			MotorControl->speedShadow = MotorControl->speedRef;
		}
		PID->Kp = MotorControl->speed_Kp;
		PID->Ki = MotorControl->speed_Ki;
		PID->fbk_value = vel_mech;
		PID->ref_value  = MotorControl->speedShadow;
		
		MotorControl->idRef = 0.0f;
		MotorControl->iqRef = Speed_PI_Ctrl(PID) * MotorControl->current_limit;

		speedloop_count = 0;
	}
	
	FOC_Current(FOC, MotorControl, theta_elec, vel_elec);
}

/**
	* @brief  Position mode control task
	* @param  *FOC: FOC struct pointer
	* @param  *MotorControl: MotorControl struct pointer
	* @param  *PID: PID struct pointer
	* @param  *Encoder: encoder struct pointer
 **/
void Task_Position_Mode(FOC_TypeDef *FOC, MotorControl_TypeDef *MotorControl, PID_TypeDef *PID, Encoder_TypeDef *Encoder)
{
	static int speedloop_count;
	static int positionloop_count;
	
	float theta_elec,theta_mech;
	float vel_elec,vel_mech;
	
	theta_elec = Encoder_GetElePhase(Encoder);
	theta_mech = Encoder_GetMecPos(Encoder);
	
	vel_elec = Encoder_GetEleVel(Encoder);
	vel_mech = Encoder_GetMecVel(Encoder);
	
	static float pos_ref_last;
	static float pos_err_last;
	
	/*position ref alternated*/
	if(pos_ref_last != MotorControl->posRef)
	{
		MotorControl->posTrajUpdated = true;
		MotorControl->isReachTargetPos = false;
	}
	
	pos_ref_last = MotorControl->posRef;
	
	if(++positionloop_count >= 4)
	{
		if(MotorControl->posTrajUpdated == true)
		{
			/*clear position ref update flag*/
			MotorControl->posTrajUpdated = false;
			
			MotorControl->posShadow = theta_mech;
			
			/*trapezoid profile plan*/
			TRAJ_plan(MotorControl->posRef,
					  MotorControl->posShadow,
					  MotorControl->speedShadow,
					  MotorControl->pos_maxspeed,
					  MotorControl->posAcc, 
					  MotorControl->posDec);
		}
		
		/*position error between allowed value*/
		if(fast_abs(theta_mech - MotorControl->posRef) <= MotorControl->pos_error_window)
			MotorControl->isReachTargetPos = true;
			
		TRAJ_eval();
		MotorControl->posShadow = TRAJ_Get_Y();
		MotorControl->speedShadow = TRAJ_Get_Yd();
		
		PID->ref_value = MotorControl->speedShadow;
		
		float pos_err = MotorControl->posShadow - theta_mech;
		float d_err = (pos_err - pos_err_last) / Position_Ts;
		
		float pos_p_output = MotorControl->pos_Kp * pos_err;
		float pos_d_output = MotorControl->pos_Kd * d_err;
			
		/*anti-disturbtion, enhance robustness during trajectory plan*/
		if(MotorControl->isReachTargetPos == false)
		{
			PID->ref_value  += pos_p_output;
			PID->ref_value  = constrain(PID->ref_value, -MotorControl->pos_maxspeed, MotorControl->pos_maxspeed);
		}
		/* normal position loop without trajectory plan*/
		else
		{
			PID->ref_value = pos_p_output + pos_d_output; 
			PID->ref_value  = constrain(PID->ref_value, -MotorControl->pos_maxspeed, MotorControl->pos_maxspeed);
		}
		
		pos_err_last = pos_err;
		
		positionloop_count = 0;
	}
	
	if(++speedloop_count >= 2)
	{
		PID->Kp = MotorControl->speed_Kp;
		PID->Ki = MotorControl->speed_Ki;
		
		PID->fbk_value = vel_mech;
		PID->error = PID->ref_value - PID->fbk_value;
		PID->error_sum += PID->error * PID->Ki * Speed_Ts;
		PID->error_sum = constrain(PID->error_sum, -1.0f, 1.0f);
		
		PID->output = PID->Kp * PID->error + PID->error_sum;
		PID->output = constrain(PID->output,-1.0f,1.0f);
	
		MotorControl->idRef = 0.0f;
		MotorControl->iqRef = PID->output * MotorControl->current_limit;
	
		speedloop_count = 0;
	}
	
	FOC_Current(FOC, MotorControl, theta_elec, vel_elec);
}

/**
	* @brief  Voltage open-loop mode
	*         rotate electrical angle by open-loop velocity and
	*         apply open-loop voltage on d-axis
	* @param  *FOC: FOC struct pointer
	* @param  *MotorControl: MotorControl struct pointer
 **/
void Task_Voltage_Mode(FOC_TypeDef *FOC, MotorControl_TypeDef *MotorControl)
{
	/*integrate electrical angle from open-loop velocity*/
	MotorControl->ol_theta = normalizeAngle(MotorControl->ol_theta + MotorControl->ol_elec_vel * Current_Ts);
	
	/*open-loop voltage drive on d-axis*/
	FOC_Voltage(FOC, MotorControl->ol_voltage, 0.0f, MotorControl->ol_theta);
}

/**
	* @brief  Q-axis voltage mode using encoder electrical angle
	*         regulate d-axis current to zero and directly command Vq
	* @param  *FOC: FOC struct pointer
	* @param  *MotorControl: MotorControl struct pointer
	* @param  *Encoder: encoder struct pointer
	**/
void Task_Vq_Mode(FOC_TypeDef *FOC, MotorControl_TypeDef *MotorControl, Encoder_TypeDef *Encoder)
{
	if(Encoder->enable != ENCODER_ENABLE)
	{
		Set_ErrorNow(Encoder_Error);
		return;
	}
	if((Encoder->calib_flag & ENC_CALIB_ALL) != ENC_CALIB_ALL)
	{
		Set_ErrorNow(Encoder_NotCalibrated);
		return;
	}

	MotorControl->idRef = 0.0f;
	MotorControl->iqRef = 0.0f;

	FOC_Vq_Mode(FOC,
				MotorControl,
				Encoder_GetElePhase(Encoder),
				Encoder_GetEleVel(Encoder));
}
