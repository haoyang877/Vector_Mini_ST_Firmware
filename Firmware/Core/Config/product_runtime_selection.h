#ifndef FIRMWARE_CORE_CONFIG_PRODUCT_RUNTIME_SELECTION_H
#define FIRMWARE_CORE_CONFIG_PRODUCT_RUNTIME_SELECTION_H

#include <stdint.h>

/*
 * Compact, immutable identity linked by the flash-constrained migration
 * target. The complete ProductConfig remains the host-validated product-line
 * model until legacy runtime consumers have migrated to it.
 */
typedef struct
{
	uint32_t product_id;
	uint32_t platform_id;
	uint32_t board_design_id;
	uint32_t configuration_fingerprint;
	uint32_t bsp_binding_fingerprint;
	uint16_t hardware_revision;
	uint16_t configuration_schema_version;
} ProductRuntimeSelection;

extern const ProductRuntimeSelection ProductCatalog_CurrentRuntimeSelection;

#endif
