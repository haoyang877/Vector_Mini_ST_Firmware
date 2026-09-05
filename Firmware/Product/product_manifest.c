#include "product_manifest.h"

#include "vector_mini_st_profile.h"
#include "parameter_schema.h"
#include "image_contract.h"

static const ProductManifest VectorMiniStManifest =
{
	PRODUCT_ID_VECTOR_MINI_ST,
	PRODUCT_MCU_ID_STM32G431,
	PRODUCT_HARDWARE_REVISION,
	ACTIVE_BOARD_PROFILE,
	ACTIVE_MOTOR_PROFILE,
	PARAM_SCHEMA_VERSION,
	BOOT_IMAGE_CONTRACT_VERSION,
	FIRMWARE_VERSION_MAJOR,
	FIRMWARE_VERSION_MINOR,
	FIRMWARE_VERSION_PATCH,
	FIRMWARE_IS_PRODUCTION_RELEASE,
	FIRMWARE_BUILD_NUMBER
};

const ProductManifest *ProductManifest_Get(void)
{
	return &VectorMiniStManifest;
}
