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
		vel_elec = Observer_GetEleVel(Fluxobserver);
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
void Task_Speed_Mode(FOC_TypeDef *FOC, MotorControl_TypeDef *MotorControl, PI_Controller_TypeDef *controller, Encoder_TypeDef *Encoder)
{
    static int speedloop_count;
    float theta_elec;
    float vel_elec;
    float vel_mech;

    theta_elec = Encoder_GetElePhase(Encoder);
    vel_elec = Encoder_GetEleVel(Encoder);
    vel_mech = Encoder_GetMecVel(Encoder);

    if (++speedloop_count >= SPEED_LOOP_DIVIDER)
    {
        MotorControl->isUseSpeedRamp = MotorControl->speedAcc > 0.0f && MotorControl->speedDec > 0.0f;

        if (MotorControl->isUseSpeedRamp)
        {
            if (MotorControl->speedRef > MotorControl->speedShadow)
            {
                MotorControl->speedShadow += MotorControl->speedAcc * Speed_Ts;
                if (MotorControl->speedShadow > MotorControl->speedRef)
                {
                    MotorControl->speedShadow = MotorControl->speedRef;
                }
            }
            else if (MotorControl->speedRef < MotorControl->speedShadow)
            {
                MotorControl->speedShadow -= MotorControl->speedDec * Speed_Ts;
                if (MotorControl->speedShadow < MotorControl->speedRef)
                {
                    MotorControl->speedShadow = MotorControl->speedRef;
                }
            }
        }
        else
        {
            MotorControl->speedShadow = MotorControl->speedRef;
        }

        PI_Controller_Configure(controller, MotorControl->speed_Kp, MotorControl->speed_Ki, Speed_Ts, -1.0f, 1.0f);
        MotorControl->idRef = 0.0f;
        MotorControl->iqRef = PI_Controller_Run(controller, MotorControl->speedShadow, vel_mech) * MotorControl->current_limit;
        speedloop_count = 0;
    }

    FOC_Current(FOC, MotorControl, theta_elec, vel_elec);
}

static float Sensorless_AngleDifference(float target, float source)
{
	float difference = target - source;

	if (difference > _PI)
		difference -= _2PI;
	else if (difference < -_PI)
		difference += _2PI;

	return difference;
}

static bool Sensorless_ObserverIsUsable(const Fluxobserver_TypeDef *Fluxobserver)
{
	return Fluxobserver->theta_e == Fluxobserver->theta_e &&
	       Fluxobserver->omega_e == Fluxobserver->omega_e &&
	       fast_abs(Fluxobserver->omega_e) <= SENSORLESS_OBSERVER_MAX_ELEC_VEL_RAD_S;
}

static bool Sensorless_StartupCurrentsAreValid(const MotorControl_TypeDef *MotorControl)
{
	return MotorControl->current_limit >= SENSORLESS_ALIGN_CURRENT_A &&
	       MotorControl->current_limit >= SENSORLESS_STARTUP_IQ_A &&
	       MotorControl->current_limit >= SENSORLESS_STARTUP_ID_A;
}

static void Sensorless_UpdateSpeedReference(MotorControl_TypeDef *MotorControl)
{
	MotorControl->isUseSpeedRamp = MotorControl->speedAcc > 0.0f && MotorControl->speedDec > 0.0f;

	if (MotorControl->isUseSpeedRamp)
	{
		if (MotorControl->speedRef > MotorControl->speedShadow)
		{
			MotorControl->speedShadow += MotorControl->speedAcc * Speed_Ts;
			if (MotorControl->speedShadow > MotorControl->speedRef)
				MotorControl->speedShadow = MotorControl->speedRef;
		}
		else if (MotorControl->speedRef < MotorControl->speedShadow)
		{
			MotorControl->speedShadow -= MotorControl->speedDec * Speed_Ts;
			if (MotorControl->speedShadow < MotorControl->speedRef)
				MotorControl->speedShadow = MotorControl->speedRef;
		}
	}
	else
	{
		MotorControl->speedShadow = MotorControl->speedRef;
	}
}

/**
	* @brief  Sensorless speed control with align, open-loop startup and observer handoff
 **/
void Task_Sensorless_Speed_Mode(FOC_TypeDef *FOC,
						MotorControl_TypeDef *MotorControl,
						PI_Controller_TypeDef *controller,
						Fluxobserver_TypeDef *Fluxobserver,
						SensorlessStartup_TypeDef *Startup)
{
	float pole_pairs = (float)MotorControl->motor_pole_pairs;
	float min_mech_vel;
	float requested_direction;

	if (pole_pairs <= 0.0f || MotorControl->motor_phase_resistance <= 0.0f ||
		MotorControl->motor_d_inductance <= 0.0f || MotorControl->motor_q_inductance <= 0.0f ||
		MotorControl->motor_flux <= 0.0f || MotorControl->current_limit <= 0.0f)
	{
		Set_ErrorNow(MotorParam_Error);
		return;
	}

	if (fast_abs(MotorControl->speedRef) <= 1e-4f)
	{
		SensorlessStartup_Reset(Startup);
		PI_Controller_Reset(controller);
		MotorControl->speedShadow = 0.0f;
		MotorControl->idRef = 0.0f;
		MotorControl->iqRef = 0.0f;
		FOC_Current(FOC, MotorControl, 0.0f, 0.0f);
		return;
	}

	min_mech_vel = SENSORLESS_STARTUP_MIN_ELEC_VEL_RAD_S / pole_pairs;
	if (fast_abs(MotorControl->speedRef) < min_mech_vel)
	{
		Set_ErrorNow(Sensorless_Error);
		return;
	}

	if (!Sensorless_StartupCurrentsAreValid(MotorControl))
	{
		Set_ErrorNow(Sensorless_Error);
		return;
	}

	requested_direction = MotorControl->speedRef >= 0.0f ? 1.0f : -1.0f;

	if (Startup->state == SENSORLESS_STARTUP_IDLE)
	{
		Fluxobserver_ParamInit(Fluxobserver);
		FOC_CurrentController_Reset(FOC);
		PI_Controller_Reset(controller);
		Startup->state = SENSORLESS_STARTUP_ALIGN;
		Startup->state_ticks = 0U;
		Startup->direction = requested_direction;
	}

	switch (Startup->state)
	{
		case SENSORLESS_STARTUP_ALIGN:
			MotorControl->idRef = SENSORLESS_ALIGN_CURRENT_A *
				constrain(((float)Startup->state_ticks + 1.0f) * Current_Ts /
					SENSORLESS_ALIGN_CURRENT_RAMP_TIME_S, 0.0f, 1.0f);
			MotorControl->iqRef = 0.0f;
			FOC_Current(FOC, MotorControl, 0.0f, 0.0f);
			if (++Startup->state_ticks >= (uint32_t)(SENSORLESS_ALIGN_TIME_S / Current_Ts))
			{
				Startup->state = SENSORLESS_STARTUP_OPEN_LOOP;
				Startup->open_loop_theta = 0.0f;
				Startup->open_loop_omega = 0.0f;
				Startup->direction = requested_direction;
				Startup->state_ticks = 0U;
				Startup->lock_ticks = 0U;
			}
		break;

		case SENSORLESS_STARTUP_OPEN_LOOP:
		{
			float iq_ramp_ratio;

			if (requested_direction != Startup->direction)
			{
				SensorlessStartup_Reset(Startup);
				break;
			}

			if (fast_abs(Startup->open_loop_omega) < SENSORLESS_STARTUP_TARGET_ELEC_VEL_RAD_S)
			{
				Startup->open_loop_omega += Startup->direction * SENSORLESS_STARTUP_ELEC_ACCEL_RAD_S2 * Current_Ts;
				if (fast_abs(Startup->open_loop_omega) >= SENSORLESS_STARTUP_TARGET_ELEC_VEL_RAD_S)
					Startup->open_loop_omega = Startup->direction * SENSORLESS_STARTUP_TARGET_ELEC_VEL_RAD_S;
			}

			Startup->open_loop_ticks++;
			iq_ramp_ratio = constrain((float)Startup->open_loop_ticks * Current_Ts /
				SENSORLESS_STARTUP_IQ_RAMP_TIME_S, 0.0f, 1.0f);

			Startup->open_loop_theta = normalizeAngle(Startup->open_loop_theta + Startup->open_loop_omega * Current_Ts);
			MotorControl->idRef = SENSORLESS_STARTUP_ID_A;
			MotorControl->iqRef = Startup->direction * (SENSORLESS_STARTUP_IQ_INITIAL_A +
				(SENSORLESS_STARTUP_IQ_A - SENSORLESS_STARTUP_IQ_INITIAL_A) * iq_ramp_ratio);
			FOC_Current(FOC, MotorControl, Startup->open_loop_theta, Startup->open_loop_omega);

			if (fast_abs(Startup->open_loop_omega) >= SENSORLESS_STARTUP_TARGET_ELEC_VEL_RAD_S)
			{
				Startup->state = SENSORLESS_STARTUP_SPEED_LOCK;
				Startup->state_ticks = 0U;
				Startup->lock_ticks = 0U;
			}
		}
		break;
		case SENSORLESS_STARTUP_SPEED_LOCK:
		{
			float speed_error;
			bool is_observer_locked;

			if (requested_direction != Startup->direction || !Sensorless_ObserverIsUsable(Fluxobserver))
			{
				Set_ErrorNow(Sensorless_Error);
				return;
			}

			Startup->open_loop_theta = normalizeAngle(Startup->open_loop_theta + Startup->open_loop_omega * Current_Ts);
			Startup->state_ticks++;

			MotorControl->idRef = SENSORLESS_STARTUP_ID_A;
			MotorControl->iqRef = Startup->direction * SENSORLESS_STARTUP_IQ_A;
			FOC_Current(FOC, MotorControl, Startup->open_loop_theta, Startup->open_loop_omega);

			speed_error = fast_abs(Observer_GetEleVel(Fluxobserver) - Startup->open_loop_omega);
			is_observer_locked = Observer_GetEleVel(Fluxobserver) * Startup->open_loop_omega > 0.0f &&
				speed_error <= fast_abs(Startup->open_loop_omega) * SENSORLESS_OBSERVER_LOCK_RATIO;

			if (is_observer_locked)
				Startup->lock_ticks++;
			else
				Startup->lock_ticks = 0U;

			if (Startup->lock_ticks >= (uint32_t)(SENSORLESS_STARTUP_SPEED_LOCK_TIME_S / Current_Ts))
			{
				PI_Controller_Reset(controller);
				PI_Controller_Configure(controller, MotorControl->speed_Kp, MotorControl->speed_Ki, Speed_Ts, -1.0f, 1.0f);
				PI_Controller_TrackOutput(controller, MotorControl->iqRef / MotorControl->current_limit);
				Startup->handoff_phase_delta = Sensorless_AngleDifference(Startup->open_loop_theta,
					Observer_GetElePhase(Fluxobserver));
				Startup->state = SENSORLESS_STARTUP_HANDOFF;
				Startup->state_ticks = 0U;
			}

			if (Startup->state_ticks >= (uint32_t)(SENSORLESS_STARTUP_LOCK_TIMEOUT_S / Current_Ts))
			{
				Set_ErrorNow(Sensorless_Error);
				return;
			}
		}
		break;
		case SENSORLESS_STARTUP_HANDOFF:
		{
			float blend;
			float phase;
			float phase_vel;

			if (requested_direction != Startup->direction || !Sensorless_ObserverIsUsable(Fluxobserver))
			{
				Set_ErrorNow(Sensorless_Error);
				return;
			}

			Startup->open_loop_theta = normalizeAngle(Startup->open_loop_theta + Startup->open_loop_omega * Current_Ts);
			blend = constrain((float)(++Startup->state_ticks) * Current_Ts /
				SENSORLESS_ANGLE_HANDOFF_TIME_S, 0.0f, 1.0f);
			phase = normalizeAngle(Observer_GetElePhase(Fluxobserver) +
				(1.0f - blend) * Startup->handoff_phase_delta);
			phase_vel = Startup->open_loop_omega + blend *
				(Observer_GetEleVel(Fluxobserver) - Startup->open_loop_omega);

			MotorControl->idRef = SENSORLESS_STARTUP_ID_A;
			MotorControl->iqRef = Startup->direction * SENSORLESS_STARTUP_IQ_A;
			FOC_Current(FOC, MotorControl, phase, phase_vel);

			if (blend >= 1.0f)
			{
				Startup->state = SENSORLESS_STARTUP_CLOSED_LOOP;
				Startup->state_ticks = 0U;
				Startup->id_ramp_ticks = 0U;
				Startup->loss_ticks = 0U;
				Startup->speed_feedback = Observer_GetEleVel(Fluxobserver) / pole_pairs;
				MotorControl->speedShadow = Startup->speed_feedback;
			}
		}
		break;

		case SENSORLESS_STARTUP_CLOSED_LOOP:
		{
			float observer_vel = Observer_GetEleVel(Fluxobserver);
			float observer_mech_vel = observer_vel / pole_pairs;

			if (!Sensorless_ObserverIsUsable(Fluxobserver))
			{
				Set_ErrorNow(Sensorless_Error);
				return;
			}

			if (requested_direction * observer_vel < 0.0f &&
				fast_abs(observer_vel) < SENSORLESS_STARTUP_MIN_ELEC_VEL_RAD_S)
			{
				SensorlessStartup_Reset(Startup);
				FOC_CurrentController_Reset(FOC);
				PI_Controller_Reset(controller);
				return;
			}

			MotorControl->idRef = SENSORLESS_STARTUP_ID_A *
				(1.0f - constrain((float)Startup->id_ramp_ticks * Current_Ts / SENSORLESS_ID_RAMP_DOWN_TIME_S, 0.0f, 1.0f));
			if (Startup->id_ramp_ticks < (uint32_t)(SENSORLESS_ID_RAMP_DOWN_TIME_S / Current_Ts))
				Startup->id_ramp_ticks++;

			if (++Startup->speed_loop_ticks >= SPEED_LOOP_DIVIDER)
			{
				Startup->speed_feedback += SENSORLESS_SPEED_FEEDBACK_LPF_ALPHA *
					(observer_mech_vel - Startup->speed_feedback);
				Sensorless_UpdateSpeedReference(MotorControl);
				PI_Controller_Configure(controller, MotorControl->speed_Kp, MotorControl->speed_Ki, Speed_Ts, -1.0f, 1.0f);
				MotorControl->iqRef = PI_Controller_Run(controller, MotorControl->speedShadow, Startup->speed_feedback) * MotorControl->current_limit;
				Startup->speed_loop_ticks = 0U;
			}

			if (fast_abs(observer_vel) < SENSORLESS_STARTUP_MIN_ELEC_VEL_RAD_S * 0.5f)
				Startup->loss_ticks++;
			else
				Startup->loss_ticks = 0U;

			if (Startup->loss_ticks >= (uint32_t)(SENSORLESS_OBSERVER_LOSS_TIME_S / Current_Ts))
			{
				Set_ErrorNow(Sensorless_Error);
				return;
			}

			FOC_Current(FOC, MotorControl, Observer_GetElePhase(Fluxobserver), observer_vel);
		}
		break;

		default:
			SensorlessStartup_Reset(Startup);
		break;
	}
}
/**
	* @brief  Position mode control task
	* @param  *FOC: FOC struct pointer
	* @param  *MotorControl: MotorControl struct pointer
	* @param  *PID: PID struct pointer
	* @param  *Encoder: encoder struct pointer
 **/
void Task_Position_Mode(FOC_TypeDef *FOC, MotorControl_TypeDef *MotorControl, PI_Controller_TypeDef *controller, Encoder_TypeDef *Encoder)
{
    static int speedloop_count;
    static int positionloop_count;
    static float pos_ref_last;
    static float pos_err_last;
    float theta_elec;
    float theta_mech;
    float vel_elec;
    float vel_mech;

    theta_elec = Encoder_GetElePhase(Encoder);
    theta_mech = Encoder_GetMecPos(Encoder);
    vel_elec = Encoder_GetEleVel(Encoder);
    vel_mech = Encoder_GetMecVel(Encoder);

    if (pos_ref_last != MotorControl->posRef)
    {
        MotorControl->posTrajUpdated = true;
        MotorControl->isReachTargetPos = false;
    }

    pos_ref_last = MotorControl->posRef;

    if (++positionloop_count >= 4)
    {
        float pos_err;
        float derivative_error;
        float position_p_output;
        float position_d_output;

        if (MotorControl->posTrajUpdated == true)
        {
            MotorControl->posTrajUpdated = false;
            MotorControl->posShadow = theta_mech;
            TRAJ_plan(MotorControl->posRef,
                      MotorControl->posShadow,
                      MotorControl->speedShadow,
                      MotorControl->pos_maxspeed,
                      MotorControl->posAcc,
                      MotorControl->posDec);
        }

        if (fast_abs(theta_mech - MotorControl->posRef) <= MotorControl->pos_error_window)
        {
            MotorControl->isReachTargetPos = true;
        }

        TRAJ_eval();
        MotorControl->posShadow = TRAJ_Get_Y();
        MotorControl->speedShadow = TRAJ_Get_Yd();
        controller->Ref = MotorControl->speedShadow;

        pos_err = MotorControl->posShadow - theta_mech;
        derivative_error = (pos_err - pos_err_last) / Position_Ts;
        position_p_output = MotorControl->pos_Kp * pos_err;
        position_d_output = MotorControl->pos_Kd * derivative_error;

        if (MotorControl->isReachTargetPos == false)
        {
            controller->Ref += position_p_output;
            controller->Ref = constrain(controller->Ref, -MotorControl->pos_maxspeed, MotorControl->pos_maxspeed);
        }
        else
        {
            controller->Ref = position_p_output + position_d_output;
            controller->Ref = constrain(controller->Ref, -MotorControl->pos_maxspeed, MotorControl->pos_maxspeed);
        }

        pos_err_last = pos_err;
        positionloop_count = 0;
    }

    if (++speedloop_count >= SPEED_LOOP_DIVIDER)
    {
        PI_Controller_Configure(controller, MotorControl->speed_Kp, MotorControl->speed_Ki, Speed_Ts, -1.0f, 1.0f);
        MotorControl->idRef = 0.0f;
        MotorControl->iqRef = PI_Controller_Run(controller, controller->Ref, vel_mech) * MotorControl->current_limit;
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
	if(!Encoder_IsOnline(Encoder))
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
