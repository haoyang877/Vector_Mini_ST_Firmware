#ifndef CORE_APPLICATION_CONTRACTS_FRICTION_IDENTIFICATION_PORT_H
#define CORE_APPLICATION_CONTRACTS_FRICTION_IDENTIFICATION_PORT_H

#include <stdbool.h>
#include <stdint.h>

#define FRICTION_IDENTIFICATION_PORT_MAX_SAMPLES 8U

typedef struct
{
	uint8_t state;
	uint8_t reason;
	uint8_t point_index;
	uint8_t sample_count;
	float progress_percent;
	float candidate_coulomb_pos_a;
	float candidate_coulomb_neg_a;
	float candidate_viscous_pos_a_per_rad_s;
	float candidate_viscous_neg_a_per_rad_s;
	float candidate_rmse_pos_a;
	float candidate_rmse_neg_a;
	bool candidate_valid;
	float active_coulomb_pos_a;
	float active_coulomb_neg_a;
	float active_viscous_pos_a_per_rad_s;
	float active_viscous_neg_a_per_rad_s;
	bool active_model_valid;
} FrictionIdentificationPortStatus;

typedef struct
{
	float target_speed_rad_s;
	float mean_speed_rad_s;
	float mean_iq_a;
	uint32_t sample_count;
} FrictionIdentificationPortSample;

typedef struct
{
	void *context;
	bool (*read_status)(void *context, FrictionIdentificationPortStatus *status);
	bool (*read_sample)(void *context, uint8_t index,
		FrictionIdentificationPortSample *sample);
	bool (*apply_candidate)(void *context);
} FrictionIdentificationPort;

#endif
