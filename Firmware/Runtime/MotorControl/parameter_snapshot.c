#include "parameter_snapshot.h"

#include <string.h>
#include <math.h>
#include "motor_state_runtime.h"
#include "can_configuration_service.h"
#include "fast_math.h"

static ParameterSnapshotContext *ActiveContext;

#define ParameterMotor (ActiveContext->motor)
#define ParameterEncoder (ActiveContext->encoder)
#define ParameterBoardProfile (ActiveContext->board_profile)
#define ParameterMotorProfile (ActiveContext->motor_profile)
#define ParameterEncoderProfile (ActiveContext->encoder_profile)
#define MotorControl    (*ParameterMotor)
#define OnBoard_Encoder (*ParameterEncoder)

bool ParameterSnapshot_Initialize(ParameterSnapshotContext *context,
	MotorControlContext *motor,
	EncoderContext *encoder, const BoardProfile *board_profile,
	const MotorProfile *motor_profile, const EncoderProfile *encoder_profile)
{
	if (context == 0 || motor == 0 || encoder == 0 || board_profile == 0 ||
		motor_profile == 0 || encoder_profile == 0)
		return false;
	context->motor = motor;
	context->encoder = encoder;
	context->board_profile = board_profile;
	context->motor_profile = motor_profile;
	context->encoder_profile = encoder_profile;
	ActiveContext = context;
	return true;
}

static void ParameterSnapshot_ApplyNonPersistentDefaults(void)
{
	MotorControl.configuration.use_sensorless_feedback = false;
	MotorControl.configuration.open_loop_voltage_v =
		ParameterMotorProfile->open_loop_voltage_v;
	MotorControl.configuration.open_loop_electrical_velocity_rad_s =
		ParameterMotorProfile->open_loop_electrical_velocity_rad_s;
	MotorControl.configuration.position_error_window_rad =
		ParameterMotorProfile->position_error_window_rad;
	MotorControl.command.q_axis_voltage_reference_v = 0.0f;
	MotorControl.runtime.open_loop_electrical_angle_rad =
		ParameterMotorProfile->open_loop_initial_theta_rad;
}

void ParameterSnapshot_LoadDefaults(void)
{
	ParameterSnapshot_ApplyNonPersistentDefaults();
	(void)CanConfigurationService_SetNodeId(
		ParameterBoardProfile->default_can_node_id);

	MotorControl.configuration.phase_a_current_offset_adc =
		ParameterBoardProfile->default_current_offset_adc;
	MotorControl.configuration.phase_b_current_offset_adc =
		ParameterBoardProfile->default_current_offset_adc;
	MotorControl.configuration.phase_c_current_offset_adc =
		ParameterBoardProfile->default_current_offset_adc;

	MotorControl.configuration.pole_pairs = ParameterMotorProfile->pole_pairs;
	MotorControl.configuration.phase_resistance_ohm = ParameterMotorProfile->phase_resistance_ohm;
	MotorControl.configuration.d_axis_inductance_h = ParameterMotorProfile->d_axis_inductance_h;
	MotorControl.configuration.q_axis_inductance_h = ParameterMotorProfile->q_axis_inductance_h;
	MotorControl.configuration.flux_weber = ParameterMotorProfile->flux_weber;
	MotorControl.configuration.d_axis_current_kp = MotorControl.configuration.d_axis_inductance_h * ParameterMotorProfile->current_loop_bandwidth_rad_s;
	MotorControl.configuration.q_axis_current_kp = MotorControl.configuration.q_axis_inductance_h * ParameterMotorProfile->current_loop_bandwidth_rad_s;
	MotorControl.configuration.d_axis_current_ki = MotorControl.configuration.phase_resistance_ohm * ParameterMotorProfile->current_loop_bandwidth_rad_s;
	MotorControl.configuration.q_axis_current_ki = MotorControl.configuration.phase_resistance_ohm * ParameterMotorProfile->current_loop_bandwidth_rad_s;

	OnBoard_Encoder.electrical_zero_q15 = ParameterEncoderProfile->default_electrical_zero_q15;
	OnBoard_Encoder.mechanical_zero_q15 = ParameterEncoderProfile->default_mechanical_zero_q15;
	OnBoard_Encoder.calib_flag = ParameterEncoderProfile->default_calibration_flag;
	OnBoard_Encoder.reverse = ParameterEncoderProfile->default_reverse;
	memset(OnBoard_Encoder.linearization_lut_q15, 0, sizeof(OnBoard_Encoder.linearization_lut_q15));

	MotorControl.configuration.calibration_current_a = ParameterMotorProfile->calibration_current_a;
	MotorControl.configuration.current_limit_a = ParameterMotorProfile->current_limit_a;
	MotorControl.configuration.speed_limit_rad_s = ParameterMotorProfile->speed_limit_rps * MATH_TWO_PI;
	MotorControl.configuration.speed_acceleration_rad_s2 = ParameterMotorProfile->speed_acceleration_rps2 * MATH_TWO_PI;
	MotorControl.configuration.speed_deceleration_rad_s2 = ParameterMotorProfile->speed_deceleration_rps2 * MATH_TWO_PI;
	MotorControl.configuration.speed_kp = ParameterMotorProfile->speed_kp;
	MotorControl.configuration.speed_ki = ParameterMotorProfile->speed_ki;
	MotorControl.configuration.position_acceleration_rad_s2 = ParameterMotorProfile->position_acceleration_rps2 * MATH_TWO_PI;
	MotorControl.configuration.position_deceleration_rad_s2 = ParameterMotorProfile->position_deceleration_rps2 * MATH_TWO_PI;
	MotorControl.configuration.position_max_speed_rad_s = ParameterMotorProfile->position_max_speed_rps * MATH_TWO_PI;
	MotorControl.configuration.position_kp_a_per_rad = ParameterMotorProfile->position_kp_a_per_rad;
	MotorControl.configuration.position_kd_a_per_rad_s = ParameterMotorProfile->position_kd_a_per_rad_s;
	MotorControl.configuration.position_ki_a_per_rad_s = ParameterMotorProfile->position_ki_a_per_rad_s;
	MotorControl.configuration.position_integral_limit_a = ParameterMotorProfile->position_integral_limit_a;
	MotorControl.configuration.cascade_position_kp_per_s = ParameterMotorProfile->cascade_position_kp_per_s;
	MotorControl.configuration.cascade_position_kd = ParameterMotorProfile->cascade_position_kd;
	(void)CanConfigurationService_SetHeartbeatMs(
		ParameterBoardProfile->default_can_heartbeat_ms);
}

void ParameterSnapshot_Capture(ParameterSnapshot *param)
{
	uint32_t lut_index;

	memset(param, 0, sizeof(*param));
	param->node_id = (float)CanConfigurationService_GetNodeId();
	param->phase_a_current_offset_adc = (float)MotorControl.configuration.phase_a_current_offset_adc;
	param->phase_b_current_offset_adc = (float)MotorControl.configuration.phase_b_current_offset_adc;
	param->phase_c_current_offset_adc = (float)MotorControl.configuration.phase_c_current_offset_adc;
	param->pole_pairs = (float)MotorControl.configuration.pole_pairs;
	param->phase_resistance_ohm = MotorControl.configuration.phase_resistance_ohm;
	param->d_axis_inductance_h = MotorControl.configuration.d_axis_inductance_h;
	param->q_axis_inductance_h = MotorControl.configuration.q_axis_inductance_h;
	param->flux_weber = MotorControl.configuration.flux_weber;
	param->encoder_electrical_zero_q15 = OnBoard_Encoder.electrical_zero_q15;
	param->encoder_mechanical_zero_q15 = OnBoard_Encoder.mechanical_zero_q15;
	param->encoder_calib_flag = OnBoard_Encoder.calib_flag;
	param->encoder_reverse = OnBoard_Encoder.reverse;
	for (lut_index = 0U; lut_index < ENCODER_OFFSET_LUT_SIZE; ++lut_index)
		param->encoder_linearization_lut_q15[lut_index] = OnBoard_Encoder.linearization_lut_q15[lut_index];
	param->calibration_current_a = MotorControl.configuration.calibration_current_a;
	param->current_limit_a = MotorControl.configuration.current_limit_a;
	param->d_axis_current_kp = MotorControl.configuration.d_axis_current_kp;
	param->d_axis_current_ki = MotorControl.configuration.d_axis_current_ki;
	param->q_axis_current_kp = MotorControl.configuration.q_axis_current_kp;
	param->q_axis_current_ki = MotorControl.configuration.q_axis_current_ki;
	param->speed_limit_rad_s = MotorControl.configuration.speed_limit_rad_s;
	param->speed_acceleration_rad_s2 = MotorControl.configuration.speed_acceleration_rad_s2;
	param->speed_deceleration_rad_s2 = MotorControl.configuration.speed_deceleration_rad_s2;
	param->speed_kp = MotorControl.configuration.speed_kp;
	param->speed_ki = MotorControl.configuration.speed_ki;
	param->position_acceleration_rad_s2 = MotorControl.configuration.position_acceleration_rad_s2;
	param->position_deceleration_rad_s2 = MotorControl.configuration.position_deceleration_rad_s2;
	param->position_max_speed_rad_s = MotorControl.configuration.position_max_speed_rad_s;
	param->position_kp_a_per_rad = MotorControl.configuration.position_kp_a_per_rad;
	param->position_kd_a_per_rad_s = MotorControl.configuration.position_kd_a_per_rad_s;
	param->position_ki_a_per_rad_s = MotorControl.configuration.position_ki_a_per_rad_s;
	param->current_sense_shunt_milliohm =
		ParameterBoardProfile->current_sense_shunt_milliohm;
	param->position_integral_limit_a = MotorControl.configuration.position_integral_limit_a;
	param->cascade_position_kp_per_s = MotorControl.configuration.cascade_position_kp_per_s;
	param->cascade_position_kd = MotorControl.configuration.cascade_position_kd;
	param->can_heartbeat_ms = (float)CanConfigurationService_GetHeartbeatMs();
	param->schema_version = PARAM_SCHEMA_VERSION;
}

void ParameterSnapshot_Apply(const ParameterSnapshot *param)
{
	uint32_t lut_index;
	uint32_t stored_shunt_milliohm;
	bool previous_schema_cascade_parameters;
	bool current_scaling_changed;
	float position_speed_limit;

	if (param->magic_word != PARAMETER_SNAPSHOT_MAGIC ||
		(param->schema_version != PARAM_SCHEMA_VERSION &&
		 param->schema_version != PARAM_SCHEMA_VERSION_PREVIOUS_INTEGRAL_LIMIT &&
		 param->schema_version != PARAM_SCHEMA_VERSION_PREVIOUS_CURRENT_SENSE &&
		 param->schema_version != PARAM_SCHEMA_VERSION_PREVIOUS_IMPEDANCE &&
		 param->schema_version != PARAM_SCHEMA_VERSION_PREVIOUS_CASCADE))
	{
		ParameterSnapshot_LoadDefaults();
		return;
	}
	previous_schema_cascade_parameters = param->schema_version == PARAM_SCHEMA_VERSION_PREVIOUS_CASCADE;
	if (param->schema_version == PARAM_SCHEMA_VERSION_PREVIOUS_CASCADE)
		stored_shunt_milliohm = PARAM_SCHEMA_PREVIOUS_CASCADE_SHUNT_MILLIOHM;
	else if (param->schema_version == PARAM_SCHEMA_VERSION_PREVIOUS_IMPEDANCE)
		stored_shunt_milliohm = PARAM_SCHEMA_PREVIOUS_IMPEDANCE_SHUNT_MILLIOHM;
	else
		stored_shunt_milliohm = param->current_sense_shunt_milliohm;
	current_scaling_changed = stored_shunt_milliohm !=
		ParameterBoardProfile->current_sense_shunt_milliohm;
	if (param->encoder_reverse > 1U)
	{
		ParameterSnapshot_LoadDefaults();
		return;
	}
	ParameterSnapshot_ApplyNonPersistentDefaults();

	(void)CanConfigurationService_SetNodeId((uint8_t)param->node_id);
	MotorControl.configuration.phase_a_current_offset_adc = (uint16_t)param->phase_a_current_offset_adc;
	MotorControl.configuration.phase_b_current_offset_adc = (uint16_t)param->phase_b_current_offset_adc;
	MotorControl.configuration.phase_c_current_offset_adc = (uint16_t)param->phase_c_current_offset_adc;
	MotorControl.configuration.pole_pairs = (int32_t)param->pole_pairs;
	MotorControl.configuration.phase_resistance_ohm = param->phase_resistance_ohm;
	MotorControl.configuration.d_axis_inductance_h = param->d_axis_inductance_h;
	MotorControl.configuration.q_axis_inductance_h = param->q_axis_inductance_h;
	MotorControl.configuration.flux_weber = param->flux_weber;
	OnBoard_Encoder.electrical_zero_q15 = param->encoder_electrical_zero_q15;
	OnBoard_Encoder.mechanical_zero_q15 = param->encoder_mechanical_zero_q15;
	OnBoard_Encoder.calib_flag = param->encoder_calib_flag;
	OnBoard_Encoder.reverse = param->encoder_reverse;
	for (lut_index = 0U; lut_index < ENCODER_OFFSET_LUT_SIZE; ++lut_index)
		OnBoard_Encoder.linearization_lut_q15[lut_index] = param->encoder_linearization_lut_q15[lut_index];
	/* A scaling change invalidates saved current-domain defaults, but not calibration data. */
	if (current_scaling_changed ||
		!isfinite(param->calibration_current_a) || param->calibration_current_a < 0.0f)
		MotorControl.configuration.calibration_current_a =
			ParameterMotorProfile->calibration_current_a;
	else
		MotorControl.configuration.calibration_current_a = FastMath_Clamp(param->calibration_current_a,
			0.0f, ParameterBoardProfile->calibration_current_limit_a);

	if (current_scaling_changed ||
		!isfinite(param->current_limit_a) || param->current_limit_a <= 0.0f)
		MotorControl.configuration.current_limit_a =
			ParameterMotorProfile->current_limit_a;
	else
		MotorControl.configuration.current_limit_a = FastMath_Clamp(param->current_limit_a,
			0.0f, ParameterBoardProfile->current_command_limit_a);
	MotorControl.configuration.d_axis_current_kp = param->d_axis_current_kp;
	MotorControl.configuration.d_axis_current_ki = param->d_axis_current_ki;
	MotorControl.configuration.q_axis_current_kp = param->q_axis_current_kp;
	MotorControl.configuration.q_axis_current_ki = param->q_axis_current_ki;
	MotorControl.configuration.speed_limit_rad_s = isfinite(param->speed_limit_rad_s) && param->speed_limit_rad_s > 0.0f ?
		FastMath_Clamp(param->speed_limit_rad_s, 0.0f,
			ParameterMotorProfile->speed_limit_max_rad_s) :
		ParameterMotorProfile->speed_limit_rps * MATH_TWO_PI;
	position_speed_limit = FastMath_Min(MotorControl.configuration.speed_limit_rad_s,
		ParameterMotorProfile->position_speed_limit_rps * MATH_TWO_PI);
	MotorControl.configuration.speed_acceleration_rad_s2 = param->speed_acceleration_rad_s2;
	MotorControl.configuration.speed_deceleration_rad_s2 = param->speed_deceleration_rad_s2;
	MotorControl.configuration.speed_kp = param->speed_kp;
	MotorControl.configuration.speed_ki = param->speed_ki;
	/*
	 * Schema v4 gains drove the speed PI and have incompatible units. Preserve
	 * motor/encoder calibration, but migrate the position settings to the safe
	 * low-speed impedance defaults.
	 */
	if (previous_schema_cascade_parameters)
	{
		MotorControl.configuration.position_acceleration_rad_s2 = ParameterMotorProfile->position_acceleration_rps2 * MATH_TWO_PI;
		MotorControl.configuration.position_deceleration_rad_s2 = ParameterMotorProfile->position_deceleration_rps2 * MATH_TWO_PI;
		MotorControl.configuration.position_max_speed_rad_s = FastMath_Min(ParameterMotorProfile->position_max_speed_rps * MATH_TWO_PI,
			position_speed_limit);
		MotorControl.configuration.position_kp_a_per_rad = ParameterMotorProfile->position_kp_a_per_rad;
		MotorControl.configuration.position_kd_a_per_rad_s = ParameterMotorProfile->position_kd_a_per_rad_s;
		MotorControl.configuration.position_ki_a_per_rad_s = ParameterMotorProfile->position_ki_a_per_rad_s;
		MotorControl.configuration.position_integral_limit_a = ParameterMotorProfile->position_integral_limit_a;
		MotorControl.configuration.cascade_position_kp_per_s =
			isfinite(param->position_kp_a_per_rad) && param->position_kp_a_per_rad >= 0.0f ?
			FastMath_Clamp(param->position_kp_a_per_rad, 0.0f,
				ParameterMotorProfile->cascade_position_kp_limit_per_s) :
			ParameterMotorProfile->cascade_position_kp_per_s;
		MotorControl.configuration.cascade_position_kd =
			isfinite(param->position_kd_a_per_rad_s) && param->position_kd_a_per_rad_s >= 0.0f ?
			FastMath_Clamp(param->position_kd_a_per_rad_s, 0.0f,
				ParameterMotorProfile->cascade_position_kd_limit) :
			ParameterMotorProfile->cascade_position_kd;
	}
	else
	{
		MotorControl.configuration.position_acceleration_rad_s2 = isfinite(param->position_acceleration_rad_s2) && param->position_acceleration_rad_s2 > 0.0f ?
			FastMath_Clamp(param->position_acceleration_rad_s2, 0.0f,
				ParameterMotorProfile->position_ramp_max_rad_s2) :
			ParameterMotorProfile->position_acceleration_rps2 * MATH_TWO_PI;
		MotorControl.configuration.position_deceleration_rad_s2 = isfinite(param->position_deceleration_rad_s2) && param->position_deceleration_rad_s2 > 0.0f ?
			FastMath_Clamp(param->position_deceleration_rad_s2, 0.0f,
				ParameterMotorProfile->position_ramp_max_rad_s2) :
			ParameterMotorProfile->position_deceleration_rps2 * MATH_TWO_PI;
		MotorControl.configuration.position_max_speed_rad_s = isfinite(param->position_max_speed_rad_s) && param->position_max_speed_rad_s > 0.0f ?
			FastMath_Clamp(param->position_max_speed_rad_s, 0.0f, position_speed_limit) :
			FastMath_Min(ParameterMotorProfile->position_max_speed_rps * MATH_TWO_PI, position_speed_limit);
		MotorControl.configuration.position_kp_a_per_rad = isfinite(param->position_kp_a_per_rad) && param->position_kp_a_per_rad >= 0.0f ?
			FastMath_Clamp(param->position_kp_a_per_rad, 0.0f,
				ParameterMotorProfile->position_kp_limit_a_per_rad) :
			ParameterMotorProfile->position_kp_a_per_rad;
		MotorControl.configuration.position_kd_a_per_rad_s = isfinite(param->position_kd_a_per_rad_s) && param->position_kd_a_per_rad_s >= 0.0f ?
			FastMath_Clamp(param->position_kd_a_per_rad_s, 0.0f,
				ParameterMotorProfile->position_kd_limit_a_per_rad_s) :
			ParameterMotorProfile->position_kd_a_per_rad_s;
		MotorControl.configuration.position_ki_a_per_rad_s = isfinite(param->position_ki_a_per_rad_s) && param->position_ki_a_per_rad_s >= 0.0f ?
			FastMath_Clamp(param->position_ki_a_per_rad_s, 0.0f,
				ParameterMotorProfile->position_ki_limit_a_per_rad_s) :
			ParameterMotorProfile->position_ki_a_per_rad_s;
		MotorControl.configuration.position_integral_limit_a =
			param->schema_version >= PARAM_SCHEMA_VERSION_PREVIOUS_INTEGRAL_LIMIT &&
			isfinite(param->position_integral_limit_a) && param->position_integral_limit_a >= 0.0f ?
			FastMath_Clamp(param->position_integral_limit_a, 0.0f,
				ParameterBoardProfile->current_command_limit_a) :
			ParameterMotorProfile->position_integral_limit_a;
		MotorControl.configuration.cascade_position_kp_per_s =
			param->schema_version == PARAM_SCHEMA_VERSION &&
			isfinite(param->cascade_position_kp_per_s) && param->cascade_position_kp_per_s >= 0.0f ?
			FastMath_Clamp(param->cascade_position_kp_per_s, 0.0f,
				ParameterMotorProfile->cascade_position_kp_limit_per_s) :
			ParameterMotorProfile->cascade_position_kp_per_s;
		MotorControl.configuration.cascade_position_kd =
			param->schema_version == PARAM_SCHEMA_VERSION &&
			isfinite(param->cascade_position_kd) && param->cascade_position_kd >= 0.0f ?
			FastMath_Clamp(param->cascade_position_kd, 0.0f,
				ParameterMotorProfile->cascade_position_kd_limit) :
			ParameterMotorProfile->cascade_position_kd;
	}
	(void)CanConfigurationService_SetHeartbeatMs((uint32_t)param->can_heartbeat_ms);
}
