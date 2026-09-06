#ifndef FIRMWARE_CORE_CONFIG_PRODUCT_CATALOG_H
#define FIRMWARE_CORE_CONFIG_PRODUCT_CATALOG_H

#include "product_config.h"
#include "Core/Config/product_manifest.h"

/* Stable generic identifiers; concrete platform bindings live below Core. */
#define PRODUCT_CATALOG_PLATFORM_CURRENT_TARGET       0x00000431UL
#define PRODUCT_CATALOG_BOARD_CURRENT                 0x564D0001UL
#define PRODUCT_CATALOG_MOTOR_CURRENT                 0x48540001UL
#define PRODUCT_CATALOG_LOAD_NO_DAMPER                 0x4C4F0000UL
#define PRODUCT_CATALOG_LOAD_DAMPED                   0x4C4F0001UL
#define PRODUCT_CATALOG_ANGLE_TLE5012B                0x414E0001UL
#define PRODUCT_CATALOG_TEMPERATURE_INTERNAL          0x544D0001UL
#define PRODUCT_CATALOG_PRODUCT_CURRENT               0x564D5354UL
#define PRODUCT_CATALOG_VARIANT_NO_DAMPER             0x00010000UL
#define PRODUCT_CATALOG_VARIANT_DAMPED                0x00010001UL
#define PRODUCT_CATALOG_FINGERPRINT_NO_DAMPER         0x9C501110UL
#define PRODUCT_CATALOG_FINGERPRINT_DAMPED            0x9C501111UL
#define PRODUCT_CATALOG_BSP_BINDING_FINGERPRINT       0x564D53A1UL

/* Stable persistence compatibility keys. They identify the already deployed
 * Flash tuple; they are not runtime profile selectors. */
#define PRODUCT_CATALOG_HARDWARE_COMPATIBILITY_ID      1U
#define PRODUCT_CATALOG_MOTOR_COMPATIBILITY_ID         1U
#define PRODUCT_CATALOG_ENCODER_COMPATIBILITY_ID       1U
#define PRODUCT_CATALOG_LOAD_NO_DAMPER_COMPATIBILITY_ID 0U
#define PRODUCT_CATALOG_LOAD_DAMPED_COMPATIBILITY_ID   1U
#define PRODUCT_CATALOG_CONTROL_COMPATIBILITY_ID       1U
#define PRODUCT_CATALOG_STORAGE_COMPATIBILITY_ID       1U

#ifndef PRODUCT_CATALOG_ACTIVE_VARIANT
#define PRODUCT_CATALOG_ACTIVE_VARIANT PRODUCT_CATALOG_VARIANT_DAMPED
#endif

#if PRODUCT_CATALOG_ACTIVE_VARIANT != PRODUCT_CATALOG_VARIANT_NO_DAMPER && \
	PRODUCT_CATALOG_ACTIVE_VARIANT != PRODUCT_CATALOG_VARIANT_DAMPED
#error "Unsupported PRODUCT_CATALOG_ACTIVE_VARIANT"
#endif

#define PRODUCT_CATALOG_ENDPOINT_MOTOR_DRIVE          0x0100U
#define PRODUCT_CATALOG_ENDPOINT_CURRENT_PHASE_A      0x0101U
#define PRODUCT_CATALOG_ENDPOINT_CURRENT_PHASE_B      0x0102U
#define PRODUCT_CATALOG_ENDPOINT_CURRENT_PHASE_C      0x0103U
#define PRODUCT_CATALOG_ENDPOINT_ANGLE_PRIMARY        0x0201U
#define PRODUCT_CATALOG_ENDPOINT_TEMPERATURE_INTERNAL 0x0301U
#define PRODUCT_CATALOG_ENDPOINT_CAN_CONTROL          0x0401U
#define PRODUCT_CATALOG_ENDPOINT_SERVICE_STREAM       0x0402U

typedef struct
{
	uint16_t hardware_compatibility_id;
	uint16_t motor_compatibility_id;
	uint16_t encoder_compatibility_id;
	uint16_t mechanical_load_compatibility_id;
	uint16_t control_compatibility_id;
	uint16_t storage_layout_compatibility_id;
	bool allow_erased_fingerprint_migration;
} ProductPersistenceCompatibility;

typedef struct
{
	const ProductConfig *config;
	ProductPersistenceCompatibility persistence;
	ProductManifest manifest;
} ProductCatalogEntry;

extern const ProductBoardDesign ProductCatalog_VectorMiniStBoard;
extern const ProductMotorDesign ProductCatalog_Ht8115_4Motor;
extern const ProductAngleSensorDesign
	ProductCatalog_Tle5012bAngleSensor;
extern const ProductTemperatureSensorDesign
	ProductCatalog_InternalTemperatureSensor;

#if defined(PRODUCT_CATALOG_INCLUDE_ALL) || \
	PRODUCT_CATALOG_ACTIVE_VARIANT == PRODUCT_CATALOG_VARIANT_NO_DAMPER
extern const ProductCatalogEntry ProductCatalog_VectorMiniStHt8115NoDamper;
#endif
#if defined(PRODUCT_CATALOG_INCLUDE_ALL) || \
	PRODUCT_CATALOG_ACTIVE_VARIANT == PRODUCT_CATALOG_VARIANT_DAMPED
extern const ProductCatalogEntry ProductCatalog_VectorMiniStHt8115Damped;
#endif

const ProductCatalogEntry *ProductCatalog_GetCurrent(void);
const ProductCatalogEntry *ProductCatalog_GetByVariant(
	ProductComponentId variant_id);

#endif
