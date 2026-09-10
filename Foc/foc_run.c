#include "fast_loop_profile.h"
#include "foc_run.h"

#include "common_inc.h"
#include "position_cascade.h"
#include "position_cascade_config.h"
#include "position_impedance.h"
#include "position_impedance_config.h"
#include "../hal/api/motor_hw.h"

static void MotorOuterLoop_RequestReset(void);

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
static void SpeedMode_UpdateControl(MotorControl_TypeDef *MotorControl,
    PI_Controller_TypeDef *controller, float vel_mech)
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
}

/* Calibration retains its serialized divided call path and shares the same PI. */
void Task_Speed_Mode(FOC_TypeDef *FOC, MotorControl_TypeDef *MotorControl,
    PI_Controller_TypeDef *controller, Encoder_TypeDef *Encoder)
{
    static unsigned speedloop_count;
    if (++speedloop_count >= SPEED_LOOP_DIVIDER) {
        SpeedMode_UpdateControl(MotorControl, controller, Encoder_GetMecVel(Encoder));
        speedloop_count = 0U;
    }
    FOC_Current(FOC, MotorControl, Encoder_GetElePhase(Encoder), Encoder_GetEleVel(Encoder));
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
/* Keep the cold configuration frame off the unchanged-tuning fast path.
 * This is a compiler hint only; other C99 compilers retain identical behavior. */
#if defined(__GNUC__) || defined(__clang__) || defined(__CC_ARM)
#define FOC_CONFIG_NOINLINE __attribute__((noinline))
#else
#define FOC_CONFIG_NOINLINE
#endif
static FOC_CONFIG_NOINLINE bool PositionMode_UpdateConfiguration(MotorControl_TypeDef *MotorControl,
    float theta_mech, float vel_mech, PositionCascadeControlOutput_TypeDef *output,
    uint16_t call_divider)
{
    PositionCascadeConfig_TypeDef config;
	config.update_period_s = Cascade_Position_Ts;
	config.call_divider = call_divider;
	config.target_position = MotorControl->posRef;
	config.position_error_window = MotorControl->pos_error_window;
	config.hold_enter_position = POSITION_SERVO_HOLD_ENTER_POSITION_RAD;
	config.hold_exit_position = POSITION_SERVO_HOLD_EXIT_POSITION_RAD;
	config.velocity_filter_hz = POSITION_SERVO_VELOCITY_FILTER_HZ *
		(MotorControl->position_velocity_filter_half_cutoff ? 0.5f : 1.0f);
	config.hold_velocity_filter_hz = MotorControl->position_hold_filter_bypass ?
		0.0f : POSITION_SERVO_HOLD_VELOCITY_FILTER_HZ *
		(MotorControl->position_hold_filter_half_cutoff ? 0.5f : 1.0f);
	config.following_error_limit = POSITION_SERVO_FOLLOWING_ERROR_LIMIT_RAD;
	config.stiction_integral_rate = POSITION_SERVO_STICTION_INTEGRAL_RATE_A_PER_S;
	config.acceleration = MotorControl->posAcc;
	config.deceleration = MotorControl->posDec;
	if (config.deceleration > POSITION_SERVO_DECELERATION_MAX_RAD_S2)
		config.deceleration = POSITION_SERVO_DECELERATION_MAX_RAD_S2;
	config.maximum_speed = MotorControl->pos_maxspeed;
	if (MotorControl->axis_profile.magic != 0U &&
		config.maximum_speed > MotorControl->axis_profile.maximum_speed_rad_s)
		config.maximum_speed = MotorControl->axis_profile.maximum_speed_rad_s;
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

    return PositionCascade_UpdateControl(&config, theta_mech, vel_mech, output);
}

/* Only this adapter supplies this controller's fixed tuning. Compare every
 * live input each fast tick; rebuild/validate on a change without a second RAM
 * cache or a parameter-write generation counter that could miss a writer. */
static bool PositionMode_SameTuningValue(float first, float second)
{
    uint32_t first_bits, second_bits;
    typedef char FloatMustBe32Bits[(sizeof(float) == sizeof(uint32_t)) ? 1 : -1];
    (void)sizeof(FloatMustBe32Bits);
    memcpy(&first_bits, &first, sizeof(first_bits));
    memcpy(&second_bits, &second, sizeof(second_bits));
    return first_bits == second_bits;
}

static bool PositionMode_ConfigurationMatches(const PositionCascadeConfig_TypeDef *config,
    const MotorControl_TypeDef *MotorControl)
{
    float deceleration = MotorControl->posDec;
    float maximum_speed = MotorControl->pos_maxspeed;
    if (config == NULL) return false;
    if (deceleration > POSITION_SERVO_DECELERATION_MAX_RAD_S2)
        deceleration = POSITION_SERVO_DECELERATION_MAX_RAD_S2;
    if (MotorControl->axis_profile.magic != 0U &&
        maximum_speed > MotorControl->axis_profile.maximum_speed_rad_s)
        maximum_speed = MotorControl->axis_profile.maximum_speed_rad_s;
    if (!PositionMode_SameTuningValue(config->velocity_filter_hz,
            POSITION_SERVO_VELOCITY_FILTER_HZ *
                (MotorControl->position_velocity_filter_half_cutoff ? 0.5f : 1.0f)) ||
        !PositionMode_SameTuningValue(config->hold_velocity_filter_hz,
            MotorControl->position_hold_filter_bypass ? 0.0f : POSITION_SERVO_HOLD_VELOCITY_FILTER_HZ *
                (MotorControl->position_hold_filter_half_cutoff ? 0.5f : 1.0f)) ||
        !PositionMode_SameTuningValue(config->position_error_window, MotorControl->pos_error_window) ||
        !PositionMode_SameTuningValue(config->acceleration, MotorControl->posAcc) ||
        !PositionMode_SameTuningValue(config->deceleration, deceleration) ||
        !PositionMode_SameTuningValue(config->maximum_speed, maximum_speed) ||
        !PositionMode_SameTuningValue(config->speed_limit, MotorControl->speed_limit) ||
        !PositionMode_SameTuningValue(config->position_kp, MotorControl->cascade_pos_Kp) ||
        !PositionMode_SameTuningValue(config->position_kd, MotorControl->cascade_pos_Kd) ||
        !PositionMode_SameTuningValue(config->speed_kp, MotorControl->speed_Kp) ||
        !PositionMode_SameTuningValue(config->speed_ki, MotorControl->speed_Ki) ||
        !PositionMode_SameTuningValue(config->current_limit, MotorControl->current_limit))
        return false;
    if (MotorControl->friction_model_valid)
        return PositionMode_SameTuningValue(config->friction_coulomb_positive, MotorControl->friction_coulomb_pos_a) &&
            PositionMode_SameTuningValue(config->friction_coulomb_negative, MotorControl->friction_coulomb_neg_a) &&
            PositionMode_SameTuningValue(config->friction_viscous_positive, MotorControl->friction_viscous_pos_a_per_rad_s) &&
            PositionMode_SameTuningValue(config->friction_viscous_negative, MotorControl->friction_viscous_neg_a_per_rad_s);
    return PositionMode_SameTuningValue(config->friction_coulomb_positive, POSITION_IMPEDANCE_FRICTION_POSITIVE_A) &&
        PositionMode_SameTuningValue(config->friction_coulomb_negative, POSITION_IMPEDANCE_FRICTION_NEGATIVE_A) &&
        PositionMode_SameTuningValue(config->friction_viscous_positive, 0.0f) &&
        PositionMode_SameTuningValue(config->friction_viscous_negative, 0.0f);
}

void Task_Position_Mode(FOC_TypeDef *FOC, MotorControl_TypeDef *MotorControl,
	Encoder_TypeDef *Encoder)
{
	bool updated;
	PositionCascadeControlOutput_TypeDef output;
	FAST_PROFILE_BEGIN(FAST_PROFILE_POSITION_ONLY);
	float theta_elec = Encoder_GetElePhase(Encoder);
	float theta_mech = Encoder_GetMecPos(Encoder);
	float vel_elec = Encoder_GetEleVel(Encoder);
	float vel_mech = Encoder_GetMecVelContinuous(Encoder);
	if (!MotorAxisProfile_AllowsPosition(&MotorControl->axis_profile,
		MotorControl->axis_profile_valid, theta_mech, MotorControl->posRef))
	{
		MotorControl->idRef = 0.0f;
		MotorControl->iqRef = 0.0f;
		Set_ErrorNow(MotorParam_Error);
		FAST_PROFILE_END(FAST_PROFILE_POSITION_ONLY);
		return;
	}

	if (PositionMode_ConfigurationMatches(PositionCascade_GetConfiguration(), MotorControl))
		updated = PositionCascade_UpdateTargetControl(MotorControl->posRef, theta_mech, vel_mech, &output);
	else
		updated = PositionMode_UpdateConfiguration(MotorControl, theta_mech, vel_mech, &output,
            CASCADE_POSITION_LOOP_DIVIDER);

	if (!updated)
	{
		MotorControl->idRef = 0.0f;
		MotorControl->iqRef = 0.0f;
		Set_ErrorNow(MotorParam_Error);
		FAST_PROFILE_END(FAST_PROFILE_POSITION_ONLY);
		return;
	}

	MotorControl->posShadow = output.position_reference;
	MotorControl->speedShadow = output.speed_reference;
	MotorControl->pos_vel_filtered = output.speed_feedback;
	MotorControl->isReachTargetPos = output.target_reached;
	MotorControl->idRef = 0.0f;
	MotorControl->iqRef = output.iq_reference;
	FAST_PROFILE_END(FAST_PROFILE_POSITION_ONLY);
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
	MotorOuterLoop_RequestReset();
	PositionImpedance_Reset();
}

/* OUTER_RUNTIME_BEGIN
 * One immutable request and one completion. Only the fast context may reuse
 * the slot; the worker never writes live MotorControl or live PI_Speed.
 * Reset invalidates the epoch without touching a possibly preempted controller.
 * This mailbox deliberately cannot queue: a missed 500 us release is a fault. */
enum { OUTER_IDLE, OUTER_QUEUED, OUTER_RUNNING, OUTER_DONE };
static struct {
    volatile unsigned status;
    volatile uint32_t epoch;
    uint32_t request_epoch, worker_epoch;
    unsigned divider, age, maximum_age, deadline_misses, completed, discarded;
    ModeNow_TypeDef mode;
    bool ready, valid, telemetry_valid;
    float position, speed;
    MotorControl_TypeDef motor;
    PI_Controller_TypeDef speed_controller;
    PositionCascadeTelemetry_TypeDef result_telemetry, published_telemetry;
} outer;

static void MotorOuterLoop_RequestReset(void)
{
    outer.epoch++;
    outer.ready = false;
    outer.telemetry_valid = false;
}

static bool MotorOuterLoop_SameTuning(const MotorControl_TypeDef *m)
{
    const MotorControl_TypeDef *saved = &outer.motor;
    if (outer.request_epoch != outer.epoch) return false;
#define SAME_INPUT(field) PositionMode_SameTuningValue(m->field, saved->field)
    return SAME_INPUT(current_limit) && SAME_INPUT(speed_Kp) && SAME_INPUT(speed_Ki) &&
        SAME_INPUT(pos_error_window) && SAME_INPUT(posAcc) && SAME_INPUT(posDec) &&
        SAME_INPUT(pos_maxspeed) && SAME_INPUT(speed_limit) &&
        SAME_INPUT(cascade_pos_Kp) && SAME_INPUT(cascade_pos_Kd) &&
        m->friction_model_valid == saved->friction_model_valid &&
        (!m->friction_model_valid ||
         (SAME_INPUT(friction_coulomb_pos_a) && SAME_INPUT(friction_coulomb_neg_a) &&
          SAME_INPUT(friction_viscous_pos_a_per_rad_s) && SAME_INPUT(friction_viscous_neg_a_per_rad_s))) &&
        m->axis_profile.magic == saved->axis_profile.magic &&
        m->axis_profile.maximum_speed_rad_s == saved->axis_profile.maximum_speed_rad_s;
#undef SAME_INPUT
}

/* Fast protection remains independent of the deferred controller. Match the
 * effective clamping used by the configuration adapter, including NaN rejection. */
static bool MotorOuterLoop_InputsValid(const MotorControl_TypeDef *m,
    float position, float speed)
{
    float deceleration, maximum_speed;
    if (!isfinite(speed)) return false;
    if (m->ModeNow == Speed_Mode)
        return isfinite(m->speedRef) && isfinite(m->speedAcc) && isfinite(m->speedDec) &&
            isfinite(m->current_limit) && m->current_limit > 0.0f &&
            isfinite(m->speed_Kp) && m->speed_Kp >= 0.0f &&
            isfinite(m->speed_Ki) && m->speed_Ki >= 0.0f;
    if (!MotorAxisProfile_AllowsPosition(&m->axis_profile,
        m->axis_profile_valid, position, m->posRef)) return false;
    /* The worker changes outputs only; request tuning stays immutable even
     * during preemption. Every live field is compared each fast tick. */
    if (MotorOuterLoop_SameTuning(m)) return true;
    if (!isfinite(m->current_limit) || m->current_limit <= 0.0f ||
        !isfinite(m->speed_Kp) || m->speed_Kp < 0.0f ||
        !isfinite(m->speed_Ki) || m->speed_Ki < 0.0f) return false;
    deceleration = m->posDec;
    maximum_speed = m->pos_maxspeed;
    if (deceleration > POSITION_SERVO_DECELERATION_MAX_RAD_S2)
        deceleration = POSITION_SERVO_DECELERATION_MAX_RAD_S2;
    if (m->axis_profile.magic != 0U && maximum_speed > m->axis_profile.maximum_speed_rad_s)
        maximum_speed = m->axis_profile.maximum_speed_rad_s;
    if (!isfinite(m->pos_error_window) || m->pos_error_window <= 0.0f ||
        m->pos_error_window > POSITION_SERVO_HOLD_ENTER_POSITION_RAD ||
        !isfinite(m->posAcc) || m->posAcc <= 0.0f ||
        !isfinite(deceleration) || deceleration <= 0.0f ||
        !isfinite(maximum_speed) || maximum_speed <= 0.0f ||
        !isfinite(m->speed_limit) || m->speed_limit < maximum_speed ||
        !isfinite((m->posAcc > deceleration ? m->posAcc : deceleration) /
            POSITION_SERVO_JERK_RAMP_TIME_S) ||
        !isfinite(m->cascade_pos_Kp) || m->cascade_pos_Kp < 0.0f ||
        m->cascade_pos_Kp > CASCADE_POSITION_KP_MAX_PER_S ||
        !isfinite(m->cascade_pos_Kd) || m->cascade_pos_Kd < 0.0f ||
        m->cascade_pos_Kd > CASCADE_POSITION_KD_MAX) return false;
    return !m->friction_model_valid ||
        (isfinite(m->friction_coulomb_pos_a) && m->friction_coulomb_pos_a >= 0.0f &&
         isfinite(m->friction_coulomb_neg_a) && m->friction_coulomb_neg_a >= 0.0f &&
         isfinite(m->friction_viscous_pos_a_per_rad_s) && m->friction_viscous_pos_a_per_rad_s >= 0.0f &&
         isfinite(m->friction_viscous_neg_a_per_rad_s) && m->friction_viscous_neg_a_per_rad_s >= 0.0f);
}

void MotorOuterLoop_Service(void)
{
    PositionCascadeControlOutput_TypeDef output;
    if (outer.status != OUTER_QUEUED) return;
    motor_hw_outer_barrier();
    outer.status = OUTER_RUNNING;
    if (outer.worker_epoch != outer.request_epoch) {
        PositionCascade_Reset();
        outer.worker_epoch = outer.request_epoch;
    }
    if (outer.motor.ModeNow == Position_Mode) {
        if (PositionMode_ConfigurationMatches(PositionCascade_GetConfiguration(), &outer.motor))
            outer.valid = PositionCascade_UpdateTargetControl(outer.motor.posRef,
                outer.position, outer.speed, &output);
        else
            outer.valid = PositionMode_UpdateConfiguration(&outer.motor,
                outer.position, outer.speed, &output, 1U);
        outer.valid = outer.valid && isfinite(output.iq_reference);
        if (outer.valid) {
            outer.motor.posShadow = output.position_reference;
            outer.motor.speedShadow = output.speed_reference;
            outer.motor.pos_vel_filtered = output.speed_feedback;
            outer.motor.isReachTargetPos = output.target_reached;
            outer.motor.iqRef = output.iq_reference;
            outer.valid = PositionCascade_GetTelemetry(&outer.result_telemetry);
        }
    } else {
        SpeedMode_UpdateControl(&outer.motor, &outer.speed_controller, outer.speed);
        outer.valid = isfinite(outer.motor.iqRef);
    }
    motor_hw_outer_barrier();
    outer.status = OUTER_DONE;
}

bool MotorOuterLoop_IsReady(void)
{
    return outer.ready;
}

bool MotorOuterLoop_GetTelemetry(PositionCascadeTelemetry_TypeDef *telemetry)
{
    /* Fast-context reader; the worker never modifies this published snapshot. */
    if (!outer.telemetry_valid || telemetry == NULL) return false;
    *telemetry = outer.published_telemetry;
    return true;
}

void MotorOuterLoop_FastTick(MotorControl_TypeDef *m, PI_Controller_TypeDef *pi,
    Encoder_TypeDef *encoder)
{
    bool active = m->ModeNow == Position_Mode || m->ModeNow == Speed_Mode;
    float position = Encoder_GetMecPos(encoder);
    float speed = m->ModeNow == Speed_Mode ? Encoder_GetMecVel(encoder) :
        Encoder_GetMecVelContinuous(encoder);
    if (m->ModeNow != outer.mode) {
        MotorOuterLoop_RequestReset();
        outer.mode = m->ModeNow;
        outer.divider = SPEED_LOOP_DIVIDER - 1U;
        if (active) { m->idRef = 0.0f; m->iqRef = 0.0f; }
    }
    if (active && m->ErrorNow == No_Error &&
        !MotorOuterLoop_InputsValid(m, position, speed)) {
        Set_ErrorNow(MotorParam_Error);
        MotorOuterLoop_RequestReset();
        m->idRef = 0.0f; m->iqRef = 0.0f;
    }
    if (outer.status != OUTER_IDLE) {
        outer.age++;
        if (outer.status == OUTER_DONE) {
            motor_hw_outer_barrier();
            if (active && m->ErrorNow == No_Error && outer.request_epoch == outer.epoch) {
                if (!outer.valid) {
                    Set_ErrorNow(MotorParam_Error);
                    m->idRef = 0.0f; m->iqRef = 0.0f;
                } else {
                    m->idRef = 0.0f;
                    /* A live current-limit reduction is effective immediately. */
                    m->iqRef = fminf(fmaxf(outer.motor.iqRef, -m->current_limit), m->current_limit);
                    m->speedShadow = outer.motor.speedShadow;
                    if (m->ModeNow == Position_Mode) {
                        m->posShadow = outer.motor.posShadow;
                        m->pos_vel_filtered = outer.motor.pos_vel_filtered;
                        m->isReachTargetPos = outer.motor.isReachTargetPos;
                        outer.published_telemetry = outer.result_telemetry;
                        outer.telemetry_valid = true;
                    } else {
                        m->isUseSpeedRamp = outer.motor.isUseSpeedRamp;
                        *pi = outer.speed_controller;
                    }
                    outer.ready = true;
                    outer.completed++;
                }
            } else outer.discarded++;
            if (outer.age > outer.maximum_age) outer.maximum_age = outer.age;
            motor_hw_outer_barrier();
            outer.status = OUTER_IDLE;
        }
    }
    if (!active || m->ErrorNow != No_Error) return;
    if (++outer.divider < SPEED_LOOP_DIVIDER) return;
    outer.divider = 0U;
    if (outer.status != OUTER_IDLE) {
        outer.deadline_misses++;
        Set_ErrorNow(ControlOverrun_Error);
        MotorOuterLoop_RequestReset();
        m->idRef = 0.0f; m->iqRef = 0.0f;
        return;
    }
    outer.motor = *m;
    outer.speed_controller = *pi;
    outer.position = position;
    outer.speed = speed;
    outer.request_epoch = outer.epoch;
    outer.age = 0U;
    motor_hw_outer_barrier();
    outer.status = OUTER_QUEUED;
    motor_hw_outer_schedule();
}
/* OUTER_RUNTIME_END */

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
