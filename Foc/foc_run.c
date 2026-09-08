#include "foc_run.h"

#include "common_inc.h"
#include "position_cascade.h"
#include "position_cascade_config.h"
#include "position_impedance.h"
#include "position_impedance_config.h"

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

const SensorlessStartupConfig_TypeDef SensorlessStartup_DefaultConfig =
{
	SENSORLESS_ALIGN_CURRENT_RAMP_TIME_S,
	SENSORLESS_ALIGN_HOLD_TIME_S,
	SENSORLESS_ALIGN_CURRENT_A,
	SENSORLESS_STARTUP_IQ_INITIAL_A,
	SENSORLESS_STARTUP_IQ_A,
	SENSORLESS_STARTUP_IQ_RAMP_TIME_S,
	SENSORLESS_STARTUP_ID_A,
	0.0f,
	SENSORLESS_STARTUP_MIN_ELEC_VEL_RAD_S,
	SENSORLESS_STARTUP_TARGET_ELEC_VEL_RAD_S,
	SENSORLESS_STARTUP_RAMP_TIME_S,
	SENSORLESS_STARTUP_SPEED_LOCK_TIME_S,
	SENSORLESS_SPEED_LOCK_FILTER_ALPHA,
	SENSORLESS_OBSERVER_LOCK_RATIO,
	SENSORLESS_ANGLE_HANDOFF_TIME_S,
	SENSORLESS_STARTUP_LOCK_TIMEOUT_S,
	SENSORLESS_ID_RAMP_DOWN_TIME_S,
	SENSORLESS_OBSERVER_LOSS_TIME_S
};

const SensorlessStartupConfig_TypeDef SensorlessStartup_EncoderCalibConfig =
{
	SENSORLESS_ENCODER_CALIB_ALIGN_CURRENT_RAMP_TIME_S,
	SENSORLESS_ENCODER_CALIB_ALIGN_HOLD_TIME_S,
	SENSORLESS_ENCODER_CALIB_ALIGN_CURRENT_A,
	SENSORLESS_ENCODER_CALIB_STARTUP_IQ_INITIAL_A,
	SENSORLESS_ENCODER_CALIB_STARTUP_IQ_A,
	SENSORLESS_ENCODER_CALIB_STARTUP_IQ_RAMP_TIME_S,
	SENSORLESS_ENCODER_CALIB_STARTUP_ID_A,
	SENSORLESS_ENCODER_CALIB_MIN_CURRENT_LIMIT_A,
	SENSORLESS_ENCODER_CALIB_MIN_ELEC_VEL_RAD_S,
	SENSORLESS_ENCODER_CALIB_TARGET_ELEC_VEL_RAD_S,
	SENSORLESS_ENCODER_CALIB_STARTUP_RAMP_TIME_S,
	SENSORLESS_ENCODER_CALIB_SPEED_LOCK_TIME_S,
	SENSORLESS_ENCODER_CALIB_SPEED_LOCK_FILTER_ALPHA,
	SENSORLESS_ENCODER_CALIB_OBSERVER_LOCK_RATIO,
	SENSORLESS_ENCODER_CALIB_ANGLE_HANDOFF_TIME_S,
	SENSORLESS_ENCODER_CALIB_LOCK_TIMEOUT_S,
	SENSORLESS_ENCODER_CALIB_ID_RAMP_DOWN_TIME_S,
	SENSORLESS_ENCODER_CALIB_OBSERVER_LOSS_TIME_S
};

static bool Sensorless_ObserverIsUsable(const Fluxobserver_TypeDef *Fluxobserver)
{
	return Fluxobserver->theta_e == Fluxobserver->theta_e &&
	       Fluxobserver->omega_e == Fluxobserver->omega_e &&
	       fast_abs(Fluxobserver->omega_e) <= SENSORLESS_OBSERVER_MAX_ELEC_VEL_RAD_S;
}

static bool Sensorless_StartupConfigIsValid(const SensorlessStartupConfig_TypeDef *Config)
{
	return Config != NULL &&
		Config->align_current_ramp_time_s > 0.0f && Config->align_hold_time_s >= 0.0f &&
		Config->align_current_a > 0.0f && Config->startup_iq_initial_a >= 0.0f &&
		Config->startup_iq_a >= Config->startup_iq_initial_a &&
		Config->startup_iq_ramp_time_s > 0.0f && Config->startup_id_a >= 0.0f &&
		Config->minimum_current_limit_a >= 0.0f &&
		Config->minimum_electrical_velocity_rad_s > 0.0f &&
		Config->target_electrical_velocity_rad_s >= Config->minimum_electrical_velocity_rad_s &&
		Config->startup_ramp_time_s > 0.0f && Config->speed_lock_time_s > 0.0f &&
		Config->speed_lock_filter_alpha > 0.0f && Config->speed_lock_filter_alpha <= 1.0f &&
		Config->observer_lock_ratio > 0.0f && Config->angle_handoff_time_s > 0.0f &&
		Config->lock_timeout_s >= Config->speed_lock_time_s &&
		Config->id_ramp_down_time_s > 0.0f && Config->observer_loss_time_s > 0.0f;
}

static bool Sensorless_StartupCurrentsAreValid(const MotorControl_TypeDef *MotorControl,
	const SensorlessStartupConfig_TypeDef *Config)
{
	float current_limit_squared = MotorControl->current_limit * MotorControl->current_limit;
	float startup_current_squared = Config->startup_iq_a * Config->startup_iq_a +
		Config->startup_id_a * Config->startup_id_a;

	return MotorControl->current_limit >= Config->minimum_current_limit_a &&
		MotorControl->current_limit >= Config->align_current_a &&
		startup_current_squared <= current_limit_squared;
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
						SensorlessStartup_TypeDef *Startup,
						const SensorlessStartupConfig_TypeDef *Config)
{
	float pole_pairs = (float)MotorControl->motor_pole_pairs;
	float min_mech_vel;
	float requested_direction;

	if (!Sensorless_StartupConfigIsValid(Config) || pole_pairs <= 0.0f ||
		MotorControl->motor_phase_resistance <= 0.0f ||
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

	min_mech_vel = Config->minimum_electrical_velocity_rad_s / pole_pairs;
	if (fast_abs(MotorControl->speedRef) < min_mech_vel)
	{
		Set_ErrorNow(Sensorless_Error);
		return;
	}

	if (!Sensorless_StartupCurrentsAreValid(MotorControl, Config))
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
			MotorControl->idRef = Config->align_current_a *
				constrain(((float)Startup->state_ticks + 1.0f) * Current_Ts /
					Config->align_current_ramp_time_s, 0.0f, 1.0f);
			MotorControl->iqRef = 0.0f;
			FOC_Current(FOC, MotorControl, 0.0f, 0.0f);
			if (++Startup->state_ticks >= (uint32_t)((Config->align_current_ramp_time_s +
				Config->align_hold_time_s) / Current_Ts))
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

			if (fast_abs(Startup->open_loop_omega) < Config->target_electrical_velocity_rad_s)
			{
				Startup->open_loop_omega += Startup->direction *
					(Config->target_electrical_velocity_rad_s / Config->startup_ramp_time_s) * Current_Ts;
				if (fast_abs(Startup->open_loop_omega) >= Config->target_electrical_velocity_rad_s)
					Startup->open_loop_omega = Startup->direction * Config->target_electrical_velocity_rad_s;
			}

			Startup->open_loop_ticks++;
			iq_ramp_ratio = constrain((float)Startup->open_loop_ticks * Current_Ts /
				Config->startup_iq_ramp_time_s, 0.0f, 1.0f);

			Startup->open_loop_theta = normalizeAngle(Startup->open_loop_theta + Startup->open_loop_omega * Current_Ts);
			MotorControl->idRef = Config->startup_id_a;
			MotorControl->iqRef = Startup->direction * (Config->startup_iq_initial_a +
				(Config->startup_iq_a - Config->startup_iq_initial_a) * iq_ramp_ratio);
			FOC_Current(FOC, MotorControl, Startup->open_loop_theta, Startup->open_loop_omega);

			if (fast_abs(Startup->open_loop_omega) >= Config->target_electrical_velocity_rad_s)
			{
				Startup->state = SENSORLESS_STARTUP_SPEED_LOCK;
				Startup->state_ticks = 0U;
				Startup->lock_ticks = 0U;
			}
		}
		break;
		case SENSORLESS_STARTUP_SPEED_LOCK:
		{
			float observer_velocity;
			float speed_error;
			bool is_observer_locked;

			if (requested_direction != Startup->direction || !Sensorless_ObserverIsUsable(Fluxobserver))
			{
				Set_ErrorNow(Sensorless_Error);
				return;
			}

			Startup->open_loop_theta = normalizeAngle(Startup->open_loop_theta + Startup->open_loop_omega * Current_Ts);
			Startup->state_ticks++;

			MotorControl->idRef = Config->startup_id_a;
			MotorControl->iqRef = Startup->direction * Config->startup_iq_a;
			FOC_Current(FOC, MotorControl, Startup->open_loop_theta, Startup->open_loop_omega);

			observer_velocity = Observer_GetEleVel(Fluxobserver);
			if (Startup->state_ticks == 1U)
				Startup->lock_speed_feedback = observer_velocity;
			else
				Startup->lock_speed_feedback += Config->speed_lock_filter_alpha *
					(observer_velocity - Startup->lock_speed_feedback);

			speed_error = fast_abs(Startup->lock_speed_feedback - Startup->open_loop_omega);
			is_observer_locked = Startup->lock_speed_feedback * Startup->open_loop_omega > 0.0f &&
				speed_error <= fast_abs(Startup->open_loop_omega) * Config->observer_lock_ratio;

			if (is_observer_locked)
				Startup->lock_ticks++;
			else
				Startup->lock_ticks = 0U;

			if (Startup->lock_ticks >= (uint32_t)(Config->speed_lock_time_s / Current_Ts))
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

			if (Startup->state_ticks >= (uint32_t)(Config->lock_timeout_s / Current_Ts))
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
				Config->angle_handoff_time_s, 0.0f, 1.0f);
			phase = normalizeAngle(Observer_GetElePhase(Fluxobserver) +
				(1.0f - blend) * Startup->handoff_phase_delta);
			phase_vel = Startup->open_loop_omega + blend *
				(Observer_GetEleVel(Fluxobserver) - Startup->open_loop_omega);

			MotorControl->idRef = Config->startup_id_a;
			MotorControl->iqRef = Startup->direction * Config->startup_iq_a;
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
				fast_abs(observer_vel) < Config->minimum_electrical_velocity_rad_s)
			{
				SensorlessStartup_Reset(Startup);
				FOC_CurrentController_Reset(FOC);
				PI_Controller_Reset(controller);
				return;
			}

			MotorControl->idRef = Config->startup_id_a *
				(1.0f - constrain((float)Startup->id_ramp_ticks * Current_Ts / Config->id_ramp_down_time_s, 0.0f, 1.0f));
			if (Startup->id_ramp_ticks < (uint32_t)(Config->id_ramp_down_time_s / Current_Ts))
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

			if (fast_abs(observer_vel) < Config->minimum_electrical_velocity_rad_s * 0.5f)
				Startup->loss_ticks++;
			else
				Startup->loss_ticks = 0U;

			if (Startup->loss_ticks >= (uint32_t)(Config->observer_loss_time_s / Current_Ts))
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
	* @brief  Mode-3 jerk-limited position-servo control task
 **/
void Task_Position_Mode(FOC_TypeDef *FOC, MotorControl_TypeDef *MotorControl,
	Encoder_TypeDef *Encoder)
{
	/* Every member is assigned below; the core excludes padding from comparison. */
	PositionCascadeConfig_TypeDef config;
	PositionCascadeOutput_TypeDef output;
	float theta_elec = Encoder_GetElePhase(Encoder);
	float theta_mech = Encoder_GetMecPos(Encoder);
	float vel_elec = Encoder_GetEleVel(Encoder);
	float vel_mech = Encoder_GetMecVelContinuous(Encoder);

	config.update_period_s = Cascade_Position_Ts;
	config.call_divider = CASCADE_POSITION_LOOP_DIVIDER;
	config.target_position = MotorControl->posRef;
	config.position_error_window = MotorControl->pos_error_window;
	config.hold_enter_position = POSITION_SERVO_HOLD_ENTER_POSITION_RAD;
	config.hold_exit_position = POSITION_SERVO_HOLD_EXIT_POSITION_RAD;
	config.velocity_filter_hz = POSITION_SERVO_VELOCITY_FILTER_HZ;
	config.following_error_limit = POSITION_SERVO_FOLLOWING_ERROR_LIMIT_RAD;
	config.stiction_integral_rate = POSITION_SERVO_STICTION_INTEGRAL_RATE_A_PER_S;
	config.acceleration = MotorControl->posAcc;
	config.deceleration = MotorControl->posDec;
	if (config.deceleration > POSITION_SERVO_DECELERATION_MAX_RAD_S2)
		config.deceleration = POSITION_SERVO_DECELERATION_MAX_RAD_S2;
	config.maximum_speed = MotorControl->pos_maxspeed;
	config.speed_limit = MotorControl->speed_limit;
	config.jerk_limit = (config.acceleration > config.deceleration ?
		config.acceleration : config.deceleration) /
		POSITION_SERVO_JERK_RAMP_TIME_S;
	config.position_kp = MotorControl->cascade_pos_Kp;
	config.position_kd = MotorControl->cascade_pos_Kd;
	config.speed_kp = MotorControl->speed_Kp;
	config.speed_ki = MotorControl->speed_Ki;
	config.acceleration_feedforward_gain =
		POSITION_SERVO_ACCEL_FF_GAIN_A_PER_RAD_S2;
	config.current_limit = MotorControl->current_limit;
	config.friction_feedforward_enabled =
		MOTOR_DAMPING_FEEDFORWARD == MOTOR_DAMPING_FEEDFORWARD_ENABLED;
	if (MotorControl->friction_model_valid)
	{
		config.friction_coulomb_positive = MotorControl->friction_coulomb_pos_a;
		config.friction_coulomb_negative = MotorControl->friction_coulomb_neg_a;
		config.friction_viscous_positive =
			MotorControl->friction_viscous_pos_a_per_rad_s;
		config.friction_viscous_negative =
			MotorControl->friction_viscous_neg_a_per_rad_s;
	}
	else
	{
		/* Board damping-ring profile used until an identified model is applied. */
		config.friction_coulomb_positive =
			POSITION_IMPEDANCE_FRICTION_POSITIVE_A;
		config.friction_coulomb_negative =
			POSITION_IMPEDANCE_FRICTION_NEGATIVE_A;
		config.friction_viscous_positive = 0.0f;
		config.friction_viscous_negative = 0.0f;
	}
	config.friction_breakaway_ratio =
		POSITION_SERVO_FRICTION_BREAKAWAY_RATIO;
	config.friction_attack_slew_rate =
		POSITION_SERVO_FRICTION_ATTACK_SLEW_A_PER_S;
	config.friction_fast_release_slew_rate =
		POSITION_SERVO_FRICTION_FAST_RELEASE_SLEW_A_PER_S;
	config.friction_release_slew_rate =
		POSITION_SERVO_FRICTION_RELEASE_SLEW_A_PER_S;
	config.friction_reference_speed =
		POSITION_SERVO_FRICTION_REFERENCE_SPEED_RAD_S;
	config.friction_stop_speed = POSITION_SERVO_FRICTION_STOP_SPEED_RAD_S;
	config.friction_move_speed = POSITION_SERVO_FRICTION_MOVE_SPEED_RAD_S;
	config.friction_breakaway_distance =
		POSITION_SERVO_FRICTION_BREAKAWAY_DISTANCE_RAD;
	config.friction_stuck_time = POSITION_SERVO_FRICTION_STUCK_TIME_S;

	if (!PositionCascade_Update(&config, theta_mech, vel_mech, &output))
	{
		MotorControl->idRef = 0.0f;
		MotorControl->iqRef = 0.0f;
		Set_ErrorNow(MotorParam_Error);
		return;
	}

	MotorControl->posShadow = output.position_reference;
	MotorControl->speedShadow = output.speed_reference;
	MotorControl->pos_vel_filtered = output.speed_feedback;
	MotorControl->isReachTargetPos = output.target_reached;
	MotorControl->idRef = 0.0f;
	MotorControl->iqRef = output.iq_reference;
	FOC_Current(FOC, MotorControl, theta_elec, vel_elec);
}

/**
	* @brief  Position impedance control task
	* @param  *FOC: FOC struct pointer
	* @param  *MotorControl: MotorControl struct pointer
	* @param  *Encoder: encoder struct pointer
 **/
void Task_Position_Impedance_Mode(FOC_TypeDef *FOC, MotorControl_TypeDef *MotorControl, Encoder_TypeDef *Encoder)
{
	PositionImpedanceConfig_TypeDef config;
	PositionImpedanceOutput_TypeDef output;
	float theta_elec = Encoder_GetElePhase(Encoder);
	float theta_mech = Encoder_GetMecPos(Encoder);
	float vel_elec = Encoder_GetEleVel(Encoder);

	config.target_position = MotorControl->posRef;
	config.position_error_window = MotorControl->pos_error_window;
	config.acceleration = MotorControl->posAcc;
	config.deceleration = MotorControl->posDec;
	config.maximum_speed = MotorControl->pos_maxspeed;
	config.speed_limit = MotorControl->speed_limit;
	config.kp = MotorControl->pos_Kp;
	config.kd = MotorControl->pos_Kd;
	config.ki = MotorControl->pos_Ki;
	config.integral_limit = MotorControl->pos_integral_limit;
	config.output_limit = MotorControl->current_limit;
	config.friction_feedforward_enabled =
		MOTOR_DAMPING_FEEDFORWARD == MOTOR_DAMPING_FEEDFORWARD_ENABLED;
	config.friction_positive_current = POSITION_IMPEDANCE_FRICTION_POSITIVE_A;
	config.friction_negative_current = POSITION_IMPEDANCE_FRICTION_NEGATIVE_A;
	config.breakaway_positive_current = POSITION_IMPEDANCE_BREAKAWAY_POSITIVE_A;
	config.breakaway_negative_current = POSITION_IMPEDANCE_BREAKAWAY_NEGATIVE_A;
	config.friction_current_slew_rate = POSITION_IMPEDANCE_FRICTION_CURRENT_SLEW_A_PER_S;
	config.friction_position_enter = POSITION_IMPEDANCE_FRICTION_POSITION_ENTER_RAD;
	config.friction_position_exit = POSITION_IMPEDANCE_FRICTION_POSITION_EXIT_RAD;
	config.friction_reference_speed = POSITION_IMPEDANCE_FRICTION_REFERENCE_SPEED_RAD_S;
	config.friction_stop_speed = POSITION_IMPEDANCE_FRICTION_STOP_SPEED_RAD_S;
	config.friction_move_speed = POSITION_IMPEDANCE_FRICTION_MOVE_SPEED_RAD_S;
	config.friction_stuck_time = POSITION_IMPEDANCE_FRICTION_STUCK_TIME_S;
	config.friction_landing_position = POSITION_IMPEDANCE_FRICTION_LANDING_POSITION_RAD;
	config.friction_landing_speed = POSITION_IMPEDANCE_FRICTION_LANDING_SPEED_RAD_S;
	config.friction_recovery_delay = POSITION_IMPEDANCE_FRICTION_RECOVERY_DELAY_S;
	config.friction_recovery_pulse_time = POSITION_IMPEDANCE_FRICTION_RECOVERY_PULSE_S;
	config.friction_recovery_cooldown = POSITION_IMPEDANCE_FRICTION_RECOVERY_COOLDOWN_S;

	if (!PositionImpedance_Update(&config, theta_mech, &output))
	{
		MotorControl->idRef = 0.0f;
		MotorControl->iqRef = 0.0f;
		Set_ErrorNow(MotorParam_Error);
		return;
	}

	MotorControl->posShadow = output.position_reference;
	MotorControl->speedShadow = output.speed_reference;
	MotorControl->pos_vel_filtered = output.velocity_feedback;
	MotorControl->isReachTargetPos = output.target_reached;
	MotorControl->idRef = 0.0f;
	MotorControl->iqRef = output.iq_reference;
	FOC_Current(FOC, MotorControl, theta_elec, vel_elec);
}

void Task_Position_Mode_Reset(void)
{
	PositionCascade_Reset();
	PositionImpedance_Reset();
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
