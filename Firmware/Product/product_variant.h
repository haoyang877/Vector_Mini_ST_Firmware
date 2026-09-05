#ifndef PRODUCT_PRODUCT_VARIANT_H
#define PRODUCT_PRODUCT_VARIANT_H

#include <stdbool.h>
#include <stdint.h>

#include "board_profile.h"
#include "control_tuning_profile.h"
#include "encoder_profiles.h"
#include "mechanical_load_profiles.h"
#include "memory_layout_profile.h"
#include "motor_profiles.h"
#include "product_manifest.h"
#include "vector_mini_st_profile.h"

/* Changes whenever any persisted-parameter compatibility dimension changes. */
#define PRODUCT_CONFIGURATION_FINGERPRINT (0x8C310000UL ^ \
	((uint32_t)ACTIVE_BOARD_PROFILE << 0U) ^ \
	((uint32_t)ACTIVE_MOTOR_PROFILE << 4U) ^ \
	((uint32_t)ACTIVE_ENCODER_PROFILE << 8U) ^ \
	((uint32_t)ACTIVE_MECHANICAL_LOAD_PROFILE << 12U) ^ \
	((uint32_t)CONTROL_TUNING_PROFILE_HT8115_4_VECTOR_MINI_ST << 16U) ^ \
	((uint32_t)CURRENT_SENSE_SHUNT_MILLIOHM << 20U) ^ \
	((uint32_t)MEMORY_LAYOUT_PROFILE_VECTOR_MINI_ST << 28U))

typedef struct
{
	const ProductManifest *identity;
	const BoardProfile *board;
	const MotorProfile *motor;
	const EncoderProfile *encoder;
	const MechanicalLoadProfile *mechanical_load;
	const ControlTuningProfile *control_tuning;
	const MemoryLayoutProfile *memory_layout;
	uint32_t configuration_fingerprint;
	bool allow_legacy_parameter_migration;
} ProductVariant;

bool ProductVariant_GetActive(ProductVariant *variant);
bool ProductVariant_Validate(const ProductVariant *variant);

#endif
