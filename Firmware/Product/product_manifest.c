#include "product_manifest.h"

#include "vector_mini_st_profile.h"
#include "parameter_schema.h"
#include "image_contract.h"
#include "encoder_profiles.h"
#include "control_tuning_profile.h"
#include "product_catalog.h"

static const ProductManifest VectorMiniStManifest =
{
	.product_id = PRODUCT_ID_VECTOR_MINI_ST,
	.mcu_id = PRODUCT_MCU_ID_STM32G431,
	.hardware_revision = PRODUCT_HARDWARE_REVISION,
	.hardware_profile_id = ACTIVE_BOARD_PROFILE,
	.motor_profile_id = ACTIVE_MOTOR_PROFILE,
	.encoder_profile_id = ACTIVE_ENCODER_PROFILE,
	.mechanical_load_profile_id = ACTIVE_MECHANICAL_LOAD_PROFILE,
	.control_tuning_profile_id =
		CONTROL_TUNING_PROFILE_HT8115_4_VECTOR_MINI_ST,
	.memory_layout_profile_id = PRODUCT_STORAGE_LAYOUT_COMPATIBILITY_ID,
	.parameter_schema_version = PARAM_SCHEMA_VERSION,
	.boot_image_contract_version = BOOT_IMAGE_CONTRACT_VERSION,
	.firmware_version_major = FIRMWARE_VERSION_MAJOR,
	.firmware_version_minor = FIRMWARE_VERSION_MINOR,
	.firmware_version_patch = FIRMWARE_VERSION_PATCH,
	.is_production_release = FIRMWARE_IS_PRODUCTION_RELEASE,
	.build_number = FIRMWARE_BUILD_NUMBER,
	.configuration_fingerprint = PRODUCT_CATALOG_CONFIGURATION_FINGERPRINT
};

const ProductManifest *ProductManifest_Get(void)
{
	return &VectorMiniStManifest;
}
