#include "Core/Application/MotorControl/parameter_snapshot.h"

#include <math.h>
#include <string.h>

#if defined(__CC_ARM)
#pragma O3
#pragma Ospace
#endif

#define ParameterMotor (context->motor)
#define ParameterEncoder (context->encoder)
#define ParameterBoardConfig (context->board_config)
#define ParameterMotorDesign (context->motor_design)
#define ParameterControlConfig (context->control_config)
#define CanConfiguration (context->can_configuration)
#define MotorControl (*ParameterMotor)
#define OnBoard_Encoder (*ParameterEncoder)

bool ParameterSnapshot_Initialize(ParameterSnapshotContext *context,
	MotorControlContext *motor, EncoderContext *encoder,
	const ParameterSnapshotBoardConfig *board_config,
	const ProductMotorDesign *motor_design,
	const ProductControlConfig *control_config,
	CanConfigurationServiceContext *can_configuration)
{
	if (context == 0 || motor == 0 || encoder == 0 || board_config == 0 ||
		motor_design == 0 || control_config == 0 || can_configuration == 0)
		return false;
	context->motor = motor;
	context->encoder = encoder;
	context->board_config = *board_config;
	context->motor_design = motor_design;
	context->control_config = control_config;
	context->can_configuration = can_configuration;
	return true;
}

void ParameterSnapshot_LoadDefaults(ParameterSnapshotContext *context)
{
	MotorControl.configuration.use_sensorless_feedback = false;
	MotorControl.configuration.open_loop_voltage_v = ParameterControlConfig->open_loop_voltage_v;
	MotorControl.configuration.open_loop_electrical_velocity_rad_s = ParameterControlConfig->open_loop_electrical_velocity_rad_s;
	MotorControl.configuration.position_error_window_rad = ParameterControlConfig->position_error_window_rad;
	MotorControl.command.q_axis_voltage_reference_v = 0.0f;
	MotorControl.runtime.open_loop_electrical_angle_rad = ParameterControlConfig->open_loop_initial_theta_rad;
	(void)CanConfigurationService_SetNodeId(CanConfiguration,
		ParameterBoardConfig.default_can_node_id);
	(void)CanConfigurationService_SetHeartbeatMs(CanConfiguration,
		ParameterBoardConfig.default_can_heartbeat_ms);

	MotorControl.configuration.phase_a_current_offset_adc =
		ParameterBoardConfig.default_current_offset_adc[0];
	MotorControl.configuration.phase_b_current_offset_adc =
		ParameterBoardConfig.default_current_offset_adc[1];
	MotorControl.configuration.phase_c_current_offset_adc =
		ParameterBoardConfig.default_current_offset_adc[2];
	MotorControl.configuration.pole_pairs = ParameterMotorDesign->pole_pairs;
	MotorControl.configuration.phase_resistance_ohm = ParameterMotorDesign->phase_resistance_ohm;
	MotorControl.configuration.d_axis_inductance_h = ParameterMotorDesign->d_axis_inductance_h;
	MotorControl.configuration.q_axis_inductance_h = ParameterMotorDesign->q_axis_inductance_h;
	MotorControl.configuration.flux_weber = ParameterMotorDesign->flux_weber;
	MotorControl.configuration.d_axis_current_kp = ParameterMotorDesign->d_axis_inductance_h * ParameterControlConfig->current_loop_bandwidth_rad_s;
	MotorControl.configuration.q_axis_current_kp = ParameterMotorDesign->q_axis_inductance_h * ParameterControlConfig->current_loop_bandwidth_rad_s;
	MotorControl.configuration.d_axis_current_ki = ParameterMotorDesign->phase_resistance_ohm * ParameterControlConfig->current_loop_bandwidth_rad_s;
	MotorControl.configuration.q_axis_current_ki = MotorControl.configuration.d_axis_current_ki;
	MotorControl.configuration.calibration_current_a = ParameterMotorDesign->calibration_current_a;
	MotorControl.configuration.current_limit_a = ParameterMotorDesign->current_limit_a;
	MotorControl.configuration.speed_limit_rad_s = ParameterControlConfig->default_speed_limit_rad_s;
	MotorControl.configuration.speed_acceleration_rad_s2 = ParameterControlConfig->speed_acceleration_rad_s2;
	MotorControl.configuration.speed_deceleration_rad_s2 = ParameterControlConfig->speed_deceleration_rad_s2;
	MotorControl.configuration.speed_kp = ParameterControlConfig->speed_kp;
	MotorControl.configuration.speed_ki = ParameterControlConfig->speed_ki;
	MotorControl.configuration.position_acceleration_rad_s2 = ParameterControlConfig->position_acceleration_rad_s2;
	MotorControl.configuration.position_deceleration_rad_s2 = ParameterControlConfig->position_deceleration_rad_s2;
	MotorControl.configuration.position_max_speed_rad_s = ParameterControlConfig->default_position_max_speed_rad_s;
	MotorControl.configuration.position_kp_a_per_rad = ParameterControlConfig->position_kp_a_per_rad;
	MotorControl.configuration.position_kd_a_per_rad_s = ParameterControlConfig->position_kd_a_per_rad_s;
	MotorControl.configuration.position_ki_a_per_rad_s = ParameterControlConfig->position_ki_a_per_rad_s;
	MotorControl.configuration.position_integral_limit_a = ParameterControlConfig->position_integral_limit_a;
	MotorControl.configuration.cascade_position_kp_per_s = ParameterControlConfig->cascade_position_kp_per_s;
	MotorControl.configuration.cascade_position_kd = ParameterControlConfig->cascade_position_kd;
	MotorControl.configuration.friction_coulomb_pos_a = 0.0f;
	MotorControl.configuration.friction_coulomb_neg_a = 0.0f;
	MotorControl.configuration.friction_viscous_pos_a_per_rad_s = 0.0f;
	MotorControl.configuration.friction_viscous_neg_a_per_rad_s = 0.0f;
	MotorControl.configuration.friction_model_valid = false;

	/* A catalog entry always boots uncalibrated; per-unit values are restored
	 * only from a compatible ParameterSnapshot. */
	OnBoard_Encoder.electrical_zero_q15 = 0U;
	OnBoard_Encoder.mechanical_zero_q15 = 0U;
	OnBoard_Encoder.calib_flag = 0U;
	OnBoard_Encoder.reverse = 0U;
	memset(OnBoard_Encoder.linearization_lut_q15, 0,
		sizeof(OnBoard_Encoder.linearization_lut_q15));
	memset(OnBoard_Encoder.cogging_compensation_map_ma, 0,
		sizeof(OnBoard_Encoder.cogging_compensation_map_ma));
}

void ParameterSnapshot_Capture(const ParameterSnapshotContext *context,
	ParameterSnapshot *param)
{
	uint32_t index;
	memset(param, 0, sizeof(*param));
	/* Only module-specific calibration is persisted. All design values are
	 * deliberately reloaded from typed profiles on each boot. */
	param->phase_a_current_offset_adc = (float)MotorControl.configuration.phase_a_current_offset_adc;
	param->phase_b_current_offset_adc = (float)MotorControl.configuration.phase_b_current_offset_adc;
	param->phase_c_current_offset_adc = (float)MotorControl.configuration.phase_c_current_offset_adc;
	param->encoder_electrical_zero_q15 = OnBoard_Encoder.electrical_zero_q15;
	param->encoder_mechanical_zero_q15 = OnBoard_Encoder.mechanical_zero_q15;
	param->encoder_calib_flag = OnBoard_Encoder.calib_flag;
	param->encoder_reverse = OnBoard_Encoder.reverse;
	for (index = 0U; index < ENCODER_OFFSET_LUT_SIZE; ++index)
		param->encoder_linearization_lut_q15[index] = OnBoard_Encoder.linearization_lut_q15[index];
	param->current_sense_shunt_milliohm =
		ParameterBoardConfig.current_sense_shunt_milliohm;
	param->friction_coulomb_pos_a = MotorControl.configuration.friction_coulomb_pos_a;
	param->friction_coulomb_neg_a = MotorControl.configuration.friction_coulomb_neg_a;
	param->friction_viscous_pos_a_per_rad_s = MotorControl.configuration.friction_viscous_pos_a_per_rad_s;
	param->friction_viscous_neg_a_per_rad_s = MotorControl.configuration.friction_viscous_neg_a_per_rad_s;
	param->friction_model_valid = MotorControl.configuration.friction_model_valid ? 1U : 0U;
	for (index = 0U; index < ENCODER_COGGING_MAP_SIZE; ++index)
		param->cogging_compensation_map_ma[index] = OnBoard_Encoder.cogging_compensation_map_ma[index];
	param->schema_version = PARAM_SCHEMA_VERSION;
}

void ParameterSnapshot_Apply(ParameterSnapshotContext *context,
	const ParameterSnapshot *param)
{
	uint32_t index;
	uint8_t allowed_flags = ENC_CALIB_LINEARIZED | ENC_CALIB_ELECTRICAL_ZERO |
		ENC_CALIB_MECHANICAL_ZERO | ENC_CALIB_COGGING;
	bool offsets_valid;
	bool friction_valid;

	ParameterSnapshot_LoadDefaults(context);
	if (param == 0 || param->magic_word != PARAMETER_SNAPSHOT_MAGIC ||
		param->schema_version < PARAM_SCHEMA_VERSION_PREVIOUS_CASCADE ||
		param->schema_version > PARAM_SCHEMA_VERSION || param->encoder_reverse > 1U)
		return;
	offsets_valid = param->phase_a_current_offset_adc >=
		ParameterBoardConfig.minimum_current_offset_adc[0] &&
		param->phase_a_current_offset_adc <=
		ParameterBoardConfig.maximum_current_offset_adc[0] &&
		param->phase_b_current_offset_adc >=
		ParameterBoardConfig.minimum_current_offset_adc[1] &&
		param->phase_b_current_offset_adc <=
		ParameterBoardConfig.maximum_current_offset_adc[1] &&
		param->phase_c_current_offset_adc >=
		ParameterBoardConfig.minimum_current_offset_adc[2] &&
		param->phase_c_current_offset_adc <=
		ParameterBoardConfig.maximum_current_offset_adc[2];
	if (offsets_valid)
	{
		MotorControl.configuration.phase_a_current_offset_adc = (uint16_t)param->phase_a_current_offset_adc;
		MotorControl.configuration.phase_b_current_offset_adc = (uint16_t)param->phase_b_current_offset_adc;
		MotorControl.configuration.phase_c_current_offset_adc = (uint16_t)param->phase_c_current_offset_adc;
	}
	OnBoard_Encoder.reverse = param->encoder_reverse;
	OnBoard_Encoder.electrical_zero_q15 = param->encoder_electrical_zero_q15;
	OnBoard_Encoder.mechanical_zero_q15 = param->encoder_mechanical_zero_q15;
	OnBoard_Encoder.calib_flag = param->encoder_calib_flag & allowed_flags;
	for (index = 0U; index < ENCODER_OFFSET_LUT_SIZE; ++index)
		OnBoard_Encoder.linearization_lut_q15[index] = param->encoder_linearization_lut_q15[index];
	friction_valid = param->schema_version >= PARAM_SCHEMA_VERSION_PREVIOUS_COGGING &&
		param->friction_model_valid == 1U && isfinite(param->friction_coulomb_pos_a) &&
		param->friction_coulomb_pos_a >= 0.0f && isfinite(param->friction_coulomb_neg_a) &&
		param->friction_coulomb_neg_a >= 0.0f && isfinite(param->friction_viscous_pos_a_per_rad_s) &&
		param->friction_viscous_pos_a_per_rad_s >= 0.0f &&
		isfinite(param->friction_viscous_neg_a_per_rad_s) &&
		param->friction_viscous_neg_a_per_rad_s >= 0.0f;
	if (friction_valid)
	{
		MotorControl.configuration.friction_coulomb_pos_a = param->friction_coulomb_pos_a;
		MotorControl.configuration.friction_coulomb_neg_a = param->friction_coulomb_neg_a;
		MotorControl.configuration.friction_viscous_pos_a_per_rad_s = param->friction_viscous_pos_a_per_rad_s;
		MotorControl.configuration.friction_viscous_neg_a_per_rad_s = param->friction_viscous_neg_a_per_rad_s;
		MotorControl.configuration.friction_model_valid = true;
	}
	if (param->schema_version == PARAM_SCHEMA_VERSION &&
		(OnBoard_Encoder.calib_flag & ENC_CALIB_COGGING) != 0U)
	{
		for (index = 0U; index < ENCODER_COGGING_MAP_SIZE; ++index)
			OnBoard_Encoder.cogging_compensation_map_ma[index] = param->cogging_compensation_map_ma[index];
	}
	else
		OnBoard_Encoder.calib_flag &= (uint8_t)~ENC_CALIB_COGGING;
}
