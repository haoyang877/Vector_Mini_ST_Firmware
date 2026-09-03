#include "foc_run.h"

#include "common_inc.h"
#include "foc_param_profile.h"

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
				PI_Controller_Configure(controller, MotorControl->speed_Kp, MotorControl->speed_Ki,
					Speed_Ts, -1.0f, Startup->speed_pi_output_max);
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
	* @param  *Encoder: encoder struct pointer
 **/
void Task_Position_Mode(FOC_TypeDef *FOC, MotorControl_TypeDef *MotorControl, Encoder_TypeDef *Encoder)
{
	float theta_elec = Encoder_GetElePhase(Encoder);
	float theta_mech = Encoder_GetMecPos(Encoder);
	float vel_elec = Encoder_GetEleVel(Encoder);

	if (!MotorControl->pos_impedance_initialized)
	{
		MotorControl->pos_impedance_initialized = true;
		MotorControl->posTrajUpdated = true;
		MotorControl->isReachTargetPos = false;
		MotorControl->pos_integral = 0.0f;
		MotorControl->pos_vel_filtered = 0.0f;
		MotorControl->pos_last_mech = theta_mech;
		MotorControl->posShadow = theta_mech;
		MotorControl->speedShadow = 0.0f;
		MotorControl->pos_ref_last = MotorControl->posRef;
		MotorControl->pos_hold_counter = 0U;
		MotorControl->pos_loop_counter = 0U;
		MotorControl->pos_integral_transport_active = false;
	}

	if (++MotorControl->pos_loop_counter >= POSITION_LOOP_DIVIDER)
	{
		float raw_velocity;
		float velocity_filter_ratio;
		float velocity_filter_alpha;
		float target_error;
		float position_error;
		float velocity_error;
		float proportional_current;
		float damping_current;
		float integral_candidate;
		float current_candidate;
		float output_limit;
		float integral_limit;
		float desired_motion;
		float integral_transport_distance;
		float integral_decay_ratio;
		float integral_decay_step;
		uint32_t hold_ticks;

		MotorControl->pos_loop_counter = 0U;
		if (!isfinite(theta_mech) || !isfinite(MotorControl->posRef) ||
			!isfinite(MotorControl->pos_error_window) || MotorControl->pos_error_window <= 0.0f ||
			!isfinite(MotorControl->posAcc) || MotorControl->posAcc <= 0.0f ||
			!isfinite(MotorControl->posDec) || MotorControl->posDec <= 0.0f ||
			!isfinite(MotorControl->pos_maxspeed) || MotorControl->pos_maxspeed <= 0.0f ||
			MotorControl->pos_maxspeed > POSITION_IMPEDANCE_MAX_SPEED_RPS * _2PI ||
			!isfinite(MotorControl->speed_limit) || MotorControl->speed_limit <= 0.0f ||
			MotorControl->pos_maxspeed > MotorControl->speed_limit ||
			!isfinite(MotorControl->pos_Kp) || MotorControl->pos_Kp < 0.0f ||
			MotorControl->pos_Kp > POSITION_IMPEDANCE_KP_MAX_A_PER_RAD ||
			!isfinite(MotorControl->pos_Kd) || MotorControl->pos_Kd < 0.0f ||
			MotorControl->pos_Kd > POSITION_IMPEDANCE_KD_MAX_A_PER_RAD_S ||
			!isfinite(MotorControl->pos_Ki) || MotorControl->pos_Ki < 0.0f ||
			MotorControl->pos_Ki > POSITION_IMPEDANCE_KI_MAX_A_PER_RAD_S)
		{
			MotorControl->idRef = 0.0f;
			MotorControl->iqRef = 0.0f;
			Set_ErrorNow(MotorParam_Error);
			return;
		}

		if (MotorControl->pos_ref_last != MotorControl->posRef)
		{
			MotorControl->pos_ref_last = MotorControl->posRef;
			MotorControl->posTrajUpdated = true;
			MotorControl->isReachTargetPos = false;
			MotorControl->pos_hold_counter = 0U;
			MotorControl->pos_integral_transport_active = true;
		}

		if (MotorControl->posTrajUpdated)
		{
			MotorControl->posTrajUpdated = false;
			/* Keep the reference state continuous when a target changes mid-trajectory. */
			TRAJ_plan(MotorControl->posRef,
					  MotorControl->posShadow,
					  MotorControl->speedShadow,
					  MotorControl->pos_maxspeed,
					  MotorControl->posAcc,
					  MotorControl->posDec);
		}

		TRAJ_eval();
		MotorControl->posShadow = TRAJ_Get_Y();
		MotorControl->speedShadow = TRAJ_Get_Yd();

		/*
		 * Differentiate the continuous multi-turn position at 1 kHz, then apply
		 * a 20 Hz first-order low-pass. This avoids the encoder estimator's hard
		 * zero/non-zero threshold being converted into damping-current steps.
		 */
		raw_velocity = (theta_mech - MotorControl->pos_last_mech) / Position_Ts;
		MotorControl->pos_last_mech = theta_mech;
		if (!isfinite(raw_velocity))
			raw_velocity = 0.0f;
		if (isfinite(MotorControl->speed_limit) && MotorControl->speed_limit > 0.0f)
			raw_velocity = constrain(raw_velocity,
								 -MotorControl->speed_limit,
								 MotorControl->speed_limit);
		velocity_filter_ratio = _2PI * PARAM_APP_POSITION_VELOCITY_FILTER_HZ * Position_Ts;
		velocity_filter_alpha = velocity_filter_ratio / (1.0f + velocity_filter_ratio);
		MotorControl->pos_vel_filtered += velocity_filter_alpha *
			(raw_velocity - MotorControl->pos_vel_filtered);
		if (!isfinite(MotorControl->pos_vel_filtered))
			MotorControl->pos_vel_filtered = 0.0f;

		target_error = MotorControl->posRef - theta_mech;
		hold_ticks = (uint32_t)(PARAM_APP_POSITION_HOLD_TIME_S / Position_Ts + 0.5f);
		if (hold_ticks < 1U)
			hold_ticks = 1U;

		if (MotorControl->isReachTargetPos)
		{
			if (fast_abs(target_error) > 2.0f * MotorControl->pos_error_window ||
				fast_abs(MotorControl->pos_vel_filtered) > PARAM_APP_POSITION_HOLD_EXIT_SPEED_RAD_S)
			{
				MotorControl->isReachTargetPos = false;
				MotorControl->pos_hold_counter = 0U;
			}
		}
		else if (fast_abs(MotorControl->posRef - MotorControl->posShadow) <=
				 MotorControl->pos_error_window &&
				 fast_abs(MotorControl->speedShadow) <=
				 PARAM_APP_POSITION_INTEGRAL_SPEED_RAD_S &&
				 fast_abs(target_error) <= MotorControl->pos_error_window &&
				 fast_abs(MotorControl->pos_vel_filtered) <= PARAM_APP_POSITION_HOLD_ENTER_SPEED_RAD_S)
		{
			if (MotorControl->pos_hold_counter < hold_ticks)
				MotorControl->pos_hold_counter++;
			if (MotorControl->pos_hold_counter >= hold_ticks)
				MotorControl->isReachTargetPos = true;
		}
		else
		{
			MotorControl->pos_hold_counter = 0U;
		}

		position_error = MotorControl->posShadow - theta_mech;
		velocity_error = MotorControl->speedShadow - MotorControl->pos_vel_filtered;
		proportional_current = MotorControl->pos_Kp * position_error;
		damping_current = MotorControl->pos_Kd * velocity_error;

		output_limit = isfinite(MotorControl->current_limit) && MotorControl->current_limit > 0.0f ?
			MotorControl->current_limit : 0.0f;
		integral_limit = fast_min(PARAM_APP_POSITION_INTEGRAL_LIMIT_A, output_limit);
		if (!isfinite(MotorControl->pos_integral))
			MotorControl->pos_integral = 0.0f;
		MotorControl->pos_integral = constrain(MotorControl->pos_integral,
										 -integral_limit,
										 integral_limit);
		if (MotorControl->pos_integral_transport_active &&
			(MotorControl->isReachTargetPos ||
			 fast_abs(MotorControl->pos_integral) <=
				PARAM_APP_POSITION_INTEGRAL_ZERO_THRESHOLD_A))
		{
			MotorControl->pos_integral_transport_active = false;
		}

		/*
		 * The integral is a holding-current estimate, so carrying it unchanged to
		 * a distant target can oppose the new motion or apply the wrong load bias.
		 * During a move, transport that state smoothly toward zero as the rotor
		 * moves. If the old bias opposes the desired direction, keep unloading it
		 * at a bounded rate until the target window is reached; this also handles a
		 * short move whose trajectory has already ended. Otherwise require both raw
		 * and filtered feedback to confirm motion in the desired direction, which
		 * preserves support current during a stall. Both the fractional and absolute
		 * changes are limited to avoid a step from a large stored integral.
		 */
		if (integral_limit <= 0.0f)
		{
			MotorControl->pos_integral = 0.0f;
			MotorControl->pos_integral_transport_active = false;
		}
		else if (MotorControl->pos_Ki <= 0.0f)
		{
			/* A live Ki disable must not remove as much as 1.5 A in one tick. */
			integral_decay_step = constrain(
				MotorControl->pos_integral *
					PARAM_APP_POSITION_INTEGRAL_OPPOSING_DECAY_RATIO_PER_TICK,
				-PARAM_APP_POSITION_INTEGRAL_DECAY_MAX_STEP_A,
				 PARAM_APP_POSITION_INTEGRAL_DECAY_MAX_STEP_A);
			MotorControl->pos_integral -= integral_decay_step;
			if (fast_abs(MotorControl->pos_integral) <=
				PARAM_APP_POSITION_INTEGRAL_ZERO_THRESHOLD_A)
			{
				MotorControl->pos_integral = 0.0f;
				MotorControl->pos_integral_transport_active = false;
			}
		}
		else
		{
			bool trajectory_is_moving =
				fast_abs(MotorControl->speedShadow) > PARAM_APP_POSITION_INTEGRAL_SPEED_RAD_S;
			bool target_is_unsettled = fast_abs(target_error) > MotorControl->pos_error_window;

			desired_motion = trajectory_is_moving ? MotorControl->speedShadow : target_error;
			bool integral_opposes_motion =
				target_is_unsettled && MotorControl->pos_integral * desired_motion < 0.0f;
			bool feedback_follows_motion =
				target_is_unsettled &&
				fast_abs(MotorControl->pos_vel_filtered) >
					PARAM_APP_POSITION_INTEGRAL_DECAY_MIN_SPEED_RAD_S &&
				desired_motion * MotorControl->pos_vel_filtered > 0.0f &&
				desired_motion * raw_velocity > 0.0f;

			if (MotorControl->pos_integral_transport_active &&
				(trajectory_is_moving || target_is_unsettled) &&
				(integral_opposes_motion || feedback_follows_motion))
			{
				if (integral_opposes_motion)
				{
					integral_decay_ratio =
						PARAM_APP_POSITION_INTEGRAL_OPPOSING_DECAY_RATIO_PER_TICK;
				}
				else
				{
					integral_transport_distance = fast_abs(raw_velocity) * Position_Ts;
					integral_decay_ratio = constrain(
						integral_transport_distance /
							PARAM_APP_POSITION_INTEGRAL_DECAY_DISTANCE_RAD,
						0.0f,
						PARAM_APP_POSITION_INTEGRAL_DECAY_MAX_RATIO_PER_TICK);
				}
				integral_decay_step = constrain(
					MotorControl->pos_integral * integral_decay_ratio,
					-PARAM_APP_POSITION_INTEGRAL_DECAY_MAX_STEP_A,
					 PARAM_APP_POSITION_INTEGRAL_DECAY_MAX_STEP_A);
				MotorControl->pos_integral -= integral_decay_step;
				if (fast_abs(MotorControl->pos_integral) <=
					PARAM_APP_POSITION_INTEGRAL_ZERO_THRESHOLD_A)
				{
					MotorControl->pos_integral = 0.0f;
					MotorControl->pos_integral_transport_active = false;
				}
			}

			/*
			 * Learn the new load bias only near the end of the trajectory. Freeze
			 * inside the position window so encoder dither cannot pump the state,
			 * and use conditional integration for anti-windup.
			 */
			if (fast_abs(MotorControl->speedShadow) <=
					PARAM_APP_POSITION_INTEGRAL_SPEED_RAD_S &&
				fast_abs(MotorControl->pos_vel_filtered) <=
					PARAM_APP_POSITION_INTEGRAL_SPEED_RAD_S &&
				fast_abs(position_error) > MotorControl->pos_error_window &&
				fast_abs(position_error) <= PARAM_APP_POSITION_INTEGRAL_ZONE_RAD)
			{
				integral_candidate = constrain(MotorControl->pos_integral +
					MotorControl->pos_Ki * position_error * Position_Ts,
					-integral_limit,
					integral_limit);
				current_candidate = proportional_current + damping_current + integral_candidate;
				if ((current_candidate >= -output_limit && current_candidate <= output_limit) ||
					(current_candidate > output_limit && position_error < 0.0f) ||
					(current_candidate < -output_limit && position_error > 0.0f))
				{
					MotorControl->pos_integral = integral_candidate;
				}
			}
		}

		current_candidate = proportional_current + damping_current + MotorControl->pos_integral;
		if (!isfinite(current_candidate))
		{
			MotorControl->idRef = 0.0f;
			MotorControl->iqRef = 0.0f;
			Set_ErrorNow(MotorParam_Error);
			return;
		}
		MotorControl->idRef = 0.0f;
		MotorControl->iqRef = constrain(current_candidate, -output_limit, output_limit);
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
