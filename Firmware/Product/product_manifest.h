#ifndef PRODUCT_VECTOR_MINI_ST_PRODUCT_MANIFEST_H
#define PRODUCT_VECTOR_MINI_ST_PRODUCT_MANIFEST_H

#include <stdint.h>

#define PRODUCT_ID_VECTOR_MINI_ST 0x564D5354UL
#define PRODUCT_MCU_ID_STM32G431 0x00000431UL

#ifndef FIRMWARE_VERSION_MAJOR
#define FIRMWARE_VERSION_MAJOR 0U
#endif
#ifndef FIRMWARE_VERSION_MINOR
#define FIRMWARE_VERSION_MINOR 0U
#endif
#ifndef FIRMWARE_VERSION_PATCH
#define FIRMWARE_VERSION_PATCH 0U
#endif
#ifndef FIRMWARE_BUILD_NUMBER
#define FIRMWARE_BUILD_NUMBER 0UL
#endif
#ifndef PRODUCT_HARDWARE_REVISION
#define PRODUCT_HARDWARE_REVISION 0U
#endif
#ifndef FIRMWARE_IS_PRODUCTION_RELEASE
#define FIRMWARE_IS_PRODUCTION_RELEASE 0U
#endif

typedef struct
{
	uint32_t product_id;
	uint32_t mcu_id;
	uint16_t hardware_revision;
	uint16_t hardware_profile_id;
	uint16_t motor_profile_id;
	uint16_t parameter_schema_version;
	uint16_t boot_image_contract_version;
	uint8_t firmware_version_major;
	uint8_t firmware_version_minor;
	uint8_t firmware_version_patch;
	uint8_t is_production_release;
	uint32_t build_number;
} ProductManifest;

const ProductManifest *ProductManifest_Get(void);

#endif
