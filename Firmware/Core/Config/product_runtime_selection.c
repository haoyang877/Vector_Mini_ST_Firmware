#include "product_runtime_selection.h"

#include "product_catalog.h"

const ProductRuntimeSelection ProductCatalog_CurrentRuntimeSelection =
{
	.product_id = PRODUCT_CATALOG_PRODUCT_CURRENT,
	.platform_id = PRODUCT_CATALOG_PLATFORM_CURRENT_TARGET,
	.board_design_id = PRODUCT_CATALOG_BOARD_CURRENT,
	.configuration_fingerprint =
		PRODUCT_CATALOG_CONFIGURATION_FINGERPRINT,
	.bsp_binding_fingerprint = PRODUCT_CATALOG_BSP_BINDING_FINGERPRINT,
	.hardware_revision = 0U,
	.configuration_schema_version = PRODUCT_CONFIG_SCHEMA_VERSION
};
