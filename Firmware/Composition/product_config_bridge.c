#include "product_config_bridge.h"

#if defined(__CC_ARM)
#pragma O3
#pragma Ospace
#endif

bool ProductConfigBridge_ValidateRuntime(const ProductRuntimeSelection *selection,
	const BspBoardRuntimeIdentity *board_identity,
	uint32_t runtime_configuration_fingerprint,
	ProductConfigBridgeStatus *bridge_status)
{
	if (bridge_status != 0)
		*bridge_status = PRODUCT_CONFIG_BRIDGE_OK;
	if (selection == 0)
	{
		if (bridge_status != 0)
			*bridge_status = PRODUCT_CONFIG_BRIDGE_PRODUCT_INVALID;
		return false;
	}
	if (selection->product_id == 0U || selection->platform_id == 0U ||
		selection->board_design_id == 0U ||
		selection->configuration_schema_version != PRODUCT_CONFIG_SCHEMA_VERSION ||
		selection->configuration_fingerprint == 0U)
	{
		if (bridge_status != 0)
			*bridge_status = PRODUCT_CONFIG_BRIDGE_PRODUCT_INVALID;
		return false;
	}
	if (runtime_configuration_fingerprint == 0U)
	{
		if (bridge_status != 0)
			*bridge_status = PRODUCT_CONFIG_BRIDGE_RUNTIME_CONFIG_INVALID;
		return false;
	}
	if (selection->configuration_fingerprint !=
		runtime_configuration_fingerprint)
	{
		if (bridge_status != 0)
			*bridge_status = PRODUCT_CONFIG_BRIDGE_RUNTIME_CONFIG_MISMATCH;
		return false;
	}
	if (board_identity == 0 || board_identity->board_id == 0U ||
		board_identity->binding_fingerprint == 0U)
	{
		if (bridge_status != 0)
			*bridge_status = PRODUCT_CONFIG_BRIDGE_BOARD_INVALID;
		return false;
	}
	if (selection->board_design_id != board_identity->board_id ||
		selection->bsp_binding_fingerprint !=
			board_identity->binding_fingerprint)
	{
		if (bridge_status != 0)
			*bridge_status = PRODUCT_CONFIG_BRIDGE_BINDING_MISMATCH;
		return false;
	}
	return true;
}
