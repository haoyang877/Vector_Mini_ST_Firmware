#ifndef CORE_APPLICATION_MOTOR_CONTROL_PARAMETER_SNAPSHOT_H
#define CORE_APPLICATION_MOTOR_CONTROL_PARAMETER_SNAPSHOT_H

#include <stddef.h>
#include <stdint.h>
#include "motor_control_types.h"
#include "Core/Config/product_config.h"
#include "encoder.h"
#include "Core/Infrastructure/Parameters/parameter_schema.h"
#include "Core/Application/Communication/can_configuration_service.h"

#define PARAMETER_SNAPSHOT_MAGIC ((uint32_t)0x454E4332U)

typedef struct
{
	uint16_t default_current_offset_adc[3];
	uint16_t minimum_current_offset_adc[3];
	uint16_t maximum_current_offset_adc[3];
	uint16_t current_sense_shunt_milliohm;
	uint8_t default_can_node_id;
	uint32_t default_can_heartbeat_ms;
} ParameterSnapshotBoardConfig;

typedef struct
{
	float node_id;
	float phase_a_current_offset_adc;
	float phase_b_current_offset_adc;
	float phase_c_current_offset_adc;
	float pole_pairs;
	float phase_resistance_ohm;
	float d_axis_inductance_h;
	float q_axis_inductance_h;
	float flux_weber;
	uint16_t encoder_electrical_zero_q15;
	uint16_t encoder_mechanical_zero_q15;
	uint8_t encoder_calib_flag;
	uint8_t encoder_reverse;
	uint8_t encoder_reserved[2];
	int16_t encoder_linearization_lut_q15[ENCODER_OFFSET_LUT_SIZE];
	float d_axis_current_kp;
	float d_axis_current_ki;
	float q_axis_current_kp;
	float q_axis_current_ki;
	float speed_acceleration_rad_s2;
	float speed_deceleration_rad_s2;
	float speed_kp;
	float speed_ki;
	float position_acceleration_rad_s2;
	float position_deceleration_rad_s2;
	float position_max_speed_rad_s;
	float position_kp_a_per_rad;
	float position_kd_a_per_rad_s;
	float calibration_current_a;
	float current_limit_a;
	float speed_limit_rad_s;
	float can_heartbeat_ms;
	uint32_t schema_version;
	uint32_t magic_word;
	/* Appended in schema v5 so the v4 schema/magic offsets remain readable. */
	float position_ki_a_per_rad_s;
	/* Appended in schema v6; identifies the current-sense scaling in Flash. */
	uint32_t current_sense_shunt_milliohm;
	/* Appended in schema v7; position-integrator output limit in amperes. */
	float position_integral_limit_a;
	/* Appended in schema v8; previous-schema cascade outer-loop gains. */
	float cascade_position_kp_per_s;
	float cascade_position_kd;
	/* Appended in schema v9; current-domain friction model. */
	float friction_coulomb_pos_a;
	float friction_coulomb_neg_a;
	float friction_viscous_pos_a_per_rad_s;
	float friction_viscous_neg_a_per_rad_s;
	uint32_t friction_model_valid;
	/* Appended in schema v10. Only calibration fields are meaningful from v10. */
	int16_t cogging_compensation_map_ma[ENCODER_COGGING_MAP_SIZE];
} ParameterSnapshot;

/* Flash ABI guardrails. Changing any of these values requires an explicit
 * schema migration; the v10 payload is already deployed in the field. */
typedef char ParameterSnapshot_SchemaMustRemainV10[
	(PARAM_SCHEMA_VERSION == 10U) ? 1 : -1];
typedef char ParameterSnapshot_SizeMustRemain2464[
	(sizeof(ParameterSnapshot) == 2464U) ? 1 : -1];
typedef char ParameterSnapshot_ShuntOffsetMustRemain2172[
	(offsetof(ParameterSnapshot, current_sense_shunt_milliohm) == 2172U) ? 1 : -1];
typedef char ParameterSnapshot_FrictionOffsetMustRemain2188[
	(offsetof(ParameterSnapshot, friction_coulomb_pos_a) == 2188U) ? 1 : -1];
typedef char ParameterSnapshot_CoggingOffsetMustRemain2208[
	(offsetof(ParameterSnapshot, cogging_compensation_map_ma) == 2208U) ? 1 : -1];

typedef struct
{
	MotorControlContext *motor;
	EncoderContext *encoder;
	ParameterSnapshotBoardConfig board_config;
	const ProductMotorDesign *motor_design;
	const ProductControlConfig *control_config;
	CanConfigurationServiceContext *can_configuration;
} ParameterSnapshotContext;

void ParameterSnapshot_LoadDefaults(ParameterSnapshotContext *context);
void ParameterSnapshot_Capture(const ParameterSnapshotContext *context,
	ParameterSnapshot *snapshot);
void ParameterSnapshot_Apply(ParameterSnapshotContext *context,
	const ParameterSnapshot *snapshot);
bool ParameterSnapshot_Initialize(ParameterSnapshotContext *context,
	MotorControlContext *motor,
	EncoderContext *encoder,
	const ParameterSnapshotBoardConfig *board_config,
	const ProductMotorDesign *motor_design,
	const ProductControlConfig *control_config,
	CanConfigurationServiceContext *can_configuration);

#endif
