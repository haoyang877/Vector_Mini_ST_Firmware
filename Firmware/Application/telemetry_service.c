#include "telemetry_service.h"

static TelemetryServiceContext *ActiveContext;

#define TelemetryBuffers (ActiveContext->buffers)
#define TelemetrySequence (ActiveContext->sequence)
#define PublishedTelemetryBuffer (ActiveContext->published_buffer)
#define TelemetryAvailable (ActiveContext != 0 && ActiveContext->is_available)

bool TelemetryService_Initialize(TelemetryServiceContext *context)
{
	if (context == 0)
		return false;
	context->sequence[0] = 0U;
	context->sequence[1] = 0U;
	context->published_buffer = 0U;
	context->is_available = false;
	ActiveContext = context;
	return true;
}

void TelemetryService_Publish(const MotorTelemetrySnapshot *snapshot)
{
	uint8_t next_buffer;

	if (snapshot == 0 || ActiveContext == 0)
		return;

	next_buffer = PublishedTelemetryBuffer == 0U ? 1U : 0U;
	TelemetrySequence[next_buffer]++;
	TelemetryBuffers[next_buffer] = *snapshot;
	TelemetrySequence[next_buffer]++;
	PublishedTelemetryBuffer = next_buffer;
	ActiveContext->is_available = true;
}

bool TelemetryService_ReadSnapshot(MotorTelemetrySnapshot *snapshot)
{
	uint8_t published_buffer;
	uint32_t sequence_before;
	uint32_t sequence_after;
	uint8_t attempt;

	if (snapshot == 0 || !TelemetryAvailable)
		return false;

	/* A bounded sequence-lock retry prevents an ISR publish tearing a copy. */
	for (attempt = 0U; attempt < 3U; attempt++)
	{
		published_buffer = PublishedTelemetryBuffer;
		sequence_before = TelemetrySequence[published_buffer];
		if ((sequence_before & 1U) != 0U)
			continue;
		*snapshot = TelemetryBuffers[published_buffer];
		sequence_after = TelemetrySequence[published_buffer];
		if (sequence_before == sequence_after &&
			(sequence_after & 1U) == 0U &&
			published_buffer == PublishedTelemetryBuffer)
			return true;
	}
	return false;
}

bool TelemetryService_ReadValue(MotorTelemetryId telemetry, float *value)
{
	MotorTelemetrySnapshot snapshot;

	if (value == 0 || !TelemetryService_ReadSnapshot(&snapshot))
		return false;

	switch (telemetry)
	{
		case MOTOR_TELEMETRY_MODE: *value = (float)snapshot.mode; break;
		case MOTOR_TELEMETRY_PRIMARY_ERROR: *value = (float)snapshot.primary_error; break;
		case MOTOR_TELEMETRY_ACTIVE_FAULTS: *value = (float)snapshot.active_faults; break;
		case MOTOR_TELEMETRY_LATCHED_FAULTS: *value = (float)snapshot.latched_faults; break;
		case MOTOR_TELEMETRY_CURRENT_REFERENCE_A: *value = snapshot.current_reference_a; break;
		case MOTOR_TELEMETRY_SPEED_REFERENCE_RAD_S: *value = snapshot.speed_reference_rad_s; break;
		case MOTOR_TELEMETRY_POSITION_REFERENCE_RAD: *value = snapshot.position_reference_rad; break;
		case MOTOR_TELEMETRY_BUS_VOLTAGE_V: *value = snapshot.bus_voltage_v; break;
		case MOTOR_TELEMETRY_BUS_CURRENT_A: *value = snapshot.bus_current_a; break;
		case MOTOR_TELEMETRY_PHASE_A_CURRENT_A: *value = snapshot.phase_a_current_a; break;
		case MOTOR_TELEMETRY_PHASE_B_CURRENT_A: *value = snapshot.phase_b_current_a; break;
		case MOTOR_TELEMETRY_PHASE_C_CURRENT_A: *value = snapshot.phase_c_current_a; break;
		case MOTOR_TELEMETRY_D_AXIS_CURRENT_A: *value = snapshot.d_axis_current_a; break;
		case MOTOR_TELEMETRY_Q_AXIS_CURRENT_A: *value = snapshot.q_axis_current_a; break;
		case MOTOR_TELEMETRY_D_AXIS_CURRENT_FILTERED_A: *value = snapshot.d_axis_current_filtered_a; break;
		case MOTOR_TELEMETRY_Q_AXIS_CURRENT_FILTERED_A: *value = snapshot.q_axis_current_filtered_a; break;
		case MOTOR_TELEMETRY_MECHANICAL_SPEED_RAD_S: *value = snapshot.mechanical_speed_rad_s; break;
		case MOTOR_TELEMETRY_MECHANICAL_POSITION_RAD: *value = snapshot.mechanical_position_rad; break;
		case MOTOR_TELEMETRY_TEMPERATURE_C: *value = snapshot.temperature_c; break;
		case MOTOR_TELEMETRY_ENCODER_ONLINE: *value = (float)snapshot.encoder_online; break;
		case MOTOR_TELEMETRY_ENCODER_REVERSED: *value = (float)snapshot.encoder_reversed; break;
		case MOTOR_TELEMETRY_POLE_PAIRS: *value = snapshot.pole_pairs; break;
		case MOTOR_TELEMETRY_CALIBRATION_CURRENT_A: *value = snapshot.calibration_current_a; break;
		case MOTOR_TELEMETRY_CURRENT_LIMIT_A: *value = snapshot.current_limit_a; break;
		case MOTOR_TELEMETRY_SPEED_LIMIT_RAD_S: *value = snapshot.speed_limit_rad_s; break;
		case MOTOR_TELEMETRY_SPEED_ACCELERATION_RAD_S2: *value = snapshot.speed_acceleration_rad_s2; break;
		case MOTOR_TELEMETRY_SPEED_DECELERATION_RAD_S2: *value = snapshot.speed_deceleration_rad_s2; break;
		case MOTOR_TELEMETRY_SPEED_KP: *value = snapshot.speed_kp; break;
		case MOTOR_TELEMETRY_SPEED_KI: *value = snapshot.speed_ki; break;
		case MOTOR_TELEMETRY_POSITION_ACCELERATION_RAD_S2: *value = snapshot.position_acceleration_rad_s2; break;
		case MOTOR_TELEMETRY_POSITION_DECELERATION_RAD_S2: *value = snapshot.position_deceleration_rad_s2; break;
		case MOTOR_TELEMETRY_POSITION_MAX_SPEED_RAD_S: *value = snapshot.position_max_speed_rad_s; break;
		case MOTOR_TELEMETRY_POSITION_KP_A_PER_RAD: *value = snapshot.position_kp_a_per_rad; break;
		case MOTOR_TELEMETRY_POSITION_KD_A_PER_RAD_S: *value = snapshot.position_kd_a_per_rad_s; break;
		case MOTOR_TELEMETRY_POSITION_KI_A_PER_RAD_S: *value = snapshot.position_ki_a_per_rad_s; break;
		case MOTOR_TELEMETRY_POSITION_INTEGRAL_LIMIT_A: *value = snapshot.position_integral_limit_a; break;
		case MOTOR_TELEMETRY_CASCADE_POSITION_KP_PER_S: *value = snapshot.cascade_position_kp_per_s; break;
		case MOTOR_TELEMETRY_CASCADE_POSITION_KD: *value = snapshot.cascade_position_kd; break;
		case MOTOR_TELEMETRY_PHASE_RESISTANCE_OHM: *value = snapshot.phase_resistance_ohm; break;
		case MOTOR_TELEMETRY_D_AXIS_INDUCTANCE_H: *value = snapshot.d_axis_inductance_h; break;
		case MOTOR_TELEMETRY_Q_AXIS_INDUCTANCE_H: *value = snapshot.q_axis_inductance_h; break;
		case MOTOR_TELEMETRY_FLUX_WEBER: *value = snapshot.flux_weber; break;
		default: return false;
	}

	return true;
}
