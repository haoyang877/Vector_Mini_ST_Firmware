#ifndef CORE_SERVICES_MEASUREMENT_MEASUREMENT_MODEL_H
#define CORE_SERVICES_MEASUREMENT_MEASUREMENT_MODEL_H

#include <stdbool.h>
#include <stdint.h>

#include "phase_current_strategy.h"

#define MEASUREMENT_MODEL_PHASE_COUNT              3U
#define MEASUREMENT_MODEL_MAX_CURRENT_OBSERVATIONS 3U

typedef uint8_t MeasurementCurrentChannelRole;

enum
{
	MEASUREMENT_CURRENT_CHANNEL_ROLE_INVALID = 0,
	MEASUREMENT_CURRENT_CHANNEL_ROLE_PHASE_A,
	MEASUREMENT_CURRENT_CHANNEL_ROLE_PHASE_B,
	MEASUREMENT_CURRENT_CHANNEL_ROLE_PHASE_C,
	MEASUREMENT_CURRENT_CHANNEL_ROLE_DC_LINK
};

/*
 * Describes one physical current channel. acquisition_index is the endpoint's
 * stable slot in the BSP raw array and is independent of the selected physical
 * channel count (a two-shunt product may legitimately select slots 1 and 2).
 * For DC-link single-shunt, the index is retained for binding/calibration;
 * control observations are the two ordered PWM-window samples in slots 0/1.
 * The signed scale includes the analogue-chain polarity convention.
 */
typedef struct
{
	uint8_t acquisition_index;
	MeasurementCurrentChannelRole role;
	uint16_t offset_adc;
	uint16_t minimum_valid_offset_adc;
	uint16_t maximum_valid_offset_adc;
	float current_a_per_count;
} MeasurementCurrentChannelConfig;

typedef struct
{
	PhaseCurrentTopology topology;
	uint8_t physical_channel_count;
	MeasurementCurrentChannelConfig
		channels[MEASUREMENT_MODEL_MAX_CURRENT_OBSERVATIONS];
} MeasurementCurrentSenseConfig;

/*
 * Raw observations are always in acquisition order, never canonical phase
 * order. Direct 3/2-shunt frames use sample_count 3/2 and valid_phase_mask to
 * identify their physical phase observations. A 1-shunt frame uses exactly
 * two ordered observations, valid_phase_mask NONE, sector 1..6, and FIRST /
 * SECOND_ACTIVE_VECTOR window identifiers.
 */
typedef struct
{
	uint32_t raw_observation_adc[
		MEASUREMENT_MODEL_MAX_CURRENT_OBSERVATIONS];
	uint32_t sequence;
	uint8_t sample_count;
	uint8_t valid_phase_mask;
	uint8_t saturated_observation_mask;
	uint8_t sector;
	PhaseCurrentDcLinkWindowId
		windows[MEASUREMENT_MODEL_MAX_CURRENT_OBSERVATIONS];
} MeasurementCurrentAcquisitionInput;

typedef struct
{
	MeasurementCurrentAcquisitionInput current;
	uint32_t bus_voltage_adc;
	bool protection_is_active;
} MeasurementModelAcquisitionInput;

typedef enum
{
    MEASUREMENT_FAULT_NONE = 0U,
    MEASUREMENT_FAULT_OVER_VOLTAGE = 1U << 0,
    MEASUREMENT_FAULT_UNDER_VOLTAGE = 1U << 1,
    MEASUREMENT_FAULT_CURRENT_OFFSET = 1U << 2,
    MEASUREMENT_FAULT_OVER_CURRENT = 1U << 3,
	/* Invalid, duplicated, or stale acquisition: callers must fail closed. */
	MEASUREMENT_FAULT_CURRENT_ACQUISITION = 1U << 4
} MeasurementFaultFlags;

typedef struct
{
	uint32_t phase_a_adc;
	uint32_t phase_b_adc;
	uint32_t phase_c_adc;
	uint32_t bus_voltage_adc;
    bool protection_is_active;
} MeasurementModelInput;

typedef struct
{
	int16_t phase_a_offset_adc;
	int16_t phase_b_offset_adc;
	int16_t phase_c_offset_adc;
	/* Canonical phase-indexed design values. phase_channel_index maps the
	 * hardware acquisition order to A/B/C without relying on endpoint order. */
	uint8_t phase_channel_index[3];
	uint16_t minimum_valid_offset_adc[3];
	uint16_t maximum_valid_offset_adc[3];
	/* Signed scale; ProductConfig polarity is projected into the sign. */
	float current_a_per_count[3];
    float bus_voltage_v_per_count;
    float bus_voltage_filter_alpha;
    float overcurrent_trip_a;
    float overvoltage_trip_v;
    float undervoltage_trip_v;
	uint16_t overcurrent_confirm_cycles;
    uint16_t voltage_confirm_cycles;
	/*
	 * Formal 3/2/1-shunt configuration. topology INVALID selects the legacy
	 * phase_a/b/c compatibility path above; new integrations must populate
	 * this block and call MeasurementModel_UpdateAcquisition().
	 */
	MeasurementCurrentSenseConfig current_sense;
} MeasurementModelConfig;

typedef struct
{
    float phase_a_current_a;
    float phase_b_current_a;
    float phase_c_current_a;
    float bus_voltage_v;
    float bus_voltage_filtered_v;
    MeasurementFaultFlags faults;
	PhaseCurrentQuality phase_current_quality;
	PhaseCurrentStatus phase_current_status;
	uint32_t current_sequence;
} MeasurementModelOutput;

typedef struct
{
    MeasurementModelConfig config;
    MeasurementModelOutput output;
    uint16_t overvoltage_count;
    uint16_t undervoltage_count;
    uint16_t overcurrent_count;
	PhaseCurrentStrategy phase_current_strategy;
	uint32_t last_acquisition_sequence;
	bool has_last_acquisition_sequence;
	bool uses_acquisition_pipeline;
    bool is_configured;
} MeasurementModelContext;

void MeasurementModel_Reset(MeasurementModelContext *context);
bool MeasurementModel_Configure(MeasurementModelContext *context,
    const MeasurementModelConfig *config);
bool MeasurementModel_Update(MeasurementModelContext *context,
    const MeasurementModelInput *input, MeasurementModelOutput *output);

/*
 * Returns PHASE_CURRENT_STATUS_OK when voltage/current processing completed.
 * Any other status has already invalidated the phase currents and set
 * MEASUREMENT_FAULT_CURRENT_ACQUISITION in output. Every observed sequence,
 * including one carrying an invalid payload, is consumed before validation.
 */
PhaseCurrentStatus MeasurementModel_UpdateAcquisition(
	MeasurementModelContext *context,
	const MeasurementModelAcquisitionInput *input,
	MeasurementModelOutput *output);

#endif
