#include "Core/Infrastructure/Telemetry/telemetry_service.h"

#include <stddef.h>

#if defined(__CC_ARM) || defined(__GNUC__) || defined(__clang__)
#define TELEMETRY_NOINLINE __attribute__((noinline))
#elif defined(_MSC_VER)
#define TELEMETRY_NOINLINE __declspec(noinline)
#else
#define TELEMETRY_NOINLINE
#endif

#define TELEMETRY_SNAPSHOT_WORD_COUNT \
	(sizeof(MotorTelemetrySnapshot) / sizeof(uint32_t))

typedef char TelemetrySnapshotSizeMustBeWordAligned[
	(sizeof(MotorTelemetrySnapshot) % sizeof(uint32_t)) == 0U ? 1 : -1];

#define TelemetryBuffers (context->buffers)
#define TelemetrySequence (context->sequence)
#define PublishedTelemetryBuffer (context->published_buffer)
#define TelemetryAvailable (context != 0 && context->is_available)

/* Keep the copy count a runtime argument and these helpers out of line.  ARMCC
 * otherwise expands each volatile structure assignment into hundreds of
 * bytes.  Word accesses retain the volatile source/destination semantics used
 * by the sequence lock.  MotorTelemetrySnapshot is word-aligned and its size
 * is guarded above. */
static TELEMETRY_NOINLINE void TelemetryService_CopyToVolatileWords(
	volatile uint32_t *destination, const uint32_t *source, size_t word_count)
{
	while (word_count != 0U)
	{
		*destination++ = *source++;
		word_count--;
	}
}

static TELEMETRY_NOINLINE void TelemetryService_CopyFromVolatileWords(
	uint32_t *destination, const volatile uint32_t *source, size_t word_count)
{
	while (word_count != 0U)
	{
		*destination++ = *source++;
		word_count--;
	}
}

bool TelemetryService_Initialize(TelemetryServiceContext *context)
{
	if (context == 0)
		return false;
	context->sequence[0] = 0U;
	context->sequence[1] = 0U;
	context->published_buffer = 0U;
	context->is_available = false;
	return true;
}

void TelemetryService_Publish(TelemetryServiceContext *context,
	const MotorTelemetrySnapshot *snapshot)
{
	uint8_t next_buffer;

	if (snapshot == 0 || context == 0)
		return;

	next_buffer = PublishedTelemetryBuffer == 0U ? 1U : 0U;
	TelemetrySequence[next_buffer]++;
	TelemetryService_CopyToVolatileWords(
		(volatile uint32_t *)(void *)&TelemetryBuffers[next_buffer],
		(const uint32_t *)(const void *)snapshot,
		TELEMETRY_SNAPSHOT_WORD_COUNT);
	TelemetrySequence[next_buffer]++;
	PublishedTelemetryBuffer = next_buffer;
	context->is_available = true;
}

bool TelemetryService_ReadSnapshot(const TelemetryServiceContext *context,
	MotorTelemetrySnapshot *snapshot)
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
		TelemetryService_CopyFromVolatileWords(
			(uint32_t *)(void *)snapshot,
			(const volatile uint32_t *)(const volatile void *)
				&TelemetryBuffers[published_buffer],
			TELEMETRY_SNAPSHOT_WORD_COUNT);
		sequence_after = TelemetrySequence[published_buffer];
		if (sequence_before == sequence_after &&
			(sequence_after & 1U) == 0U &&
			published_buffer == PublishedTelemetryBuffer)
			return true;
	}
	return false;
}

bool TelemetryService_ReadValue(const TelemetryServiceContext *context,
	MotorTelemetryId telemetry, float *value)
{
	MotorTelemetrySnapshot snapshot;

	if (value == 0 || !TelemetryService_ReadSnapshot(context, &snapshot))
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
		case MOTOR_TELEMETRY_COMMISSIONING_STAGE:
			*value = (float)snapshot.commissioning_stage; break;
		case MOTOR_TELEMETRY_COMMISSIONING_PROGRESS_PERCENT:
			*value = (float)snapshot.commissioning_progress_percent; break;
		case MOTOR_TELEMETRY_PHASE_RESISTANCE_SPREAD_PERCENT:
			*value = snapshot.phase_resistance_spread_percent; break;
		case MOTOR_TELEMETRY_PHASE_RESISTANCE_DESIGN_ERROR_PERCENT:
			*value = snapshot.phase_resistance_design_error_percent; break;
		default: return false;
	}

	return true;
}
