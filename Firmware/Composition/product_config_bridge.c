#include "product_config_bridge.h"

#include <stddef.h>
#include <string.h>

#include "product_catalog.h"

#if defined(__CC_ARM)
#pragma O3
#pragma Ospace
#endif

static bool ProductConfigBridge_IsSupportedCan(const ProductConfig *config)
{
	const ProductCanConfig *can = &config->can;
	uint32_t nominal_bitrate_bps;
	uint32_t data_bitrate_bps;

	if (can->endpoint != PRODUCT_CATALOG_ENDPOINT_CAN_CONTROL ||
		can->maximum_payload_bytes != 8U || can->default_node_id > 7U ||
		can->nominal_bitrate_kbps == 0U ||
		can->nominal_bitrate_kbps > UINT32_MAX / 1000U ||
		can->minimum_heartbeat_ms > can->maximum_heartbeat_ms ||
		(can->heartbeat_ms != 0U &&
		 (can->heartbeat_ms < can->minimum_heartbeat_ms ||
		  can->heartbeat_ms > can->maximum_heartbeat_ms)))
		return false;
	nominal_bitrate_bps = can->nominal_bitrate_kbps * 1000U;
	if (can->mode == PRODUCT_CAN_MODE_CLASSIC)
		return config->board->classic_can_supported &&
			!can->bit_rate_switching && can->data_bitrate_kbps == 0U &&
			nominal_bitrate_bps <= 1000000U;
	if (can->mode != PRODUCT_CAN_MODE_FD || !config->board->can_fd_supported ||
		can->data_bitrate_kbps == 0U ||
		can->data_bitrate_kbps > UINT32_MAX / 1000U ||
		(can->bit_rate_switching && !config->board->can_brs_supported) ||
		(!can->bit_rate_switching &&
		 can->data_bitrate_kbps != can->nominal_bitrate_kbps))
		return false;
	data_bitrate_bps = can->data_bitrate_kbps * 1000U;
	return nominal_bitrate_bps <= 1000000U && data_bitrate_bps <= 5000000U;
}

static bool ProductConfigBridge_IsSupportedRuntime(const ProductConfig *config)
{
	/* Everything before CAN describes the currently implemented board, motor,
	 * sensor and commissioning graph. Exact comparison is intentional: a new
	 * topology must add its runtime strategies before it can pass startup. */
	return memcmp(config, &ProductCatalog_CurrentConfig,
			offsetof(ProductConfig, can)) == 0 &&
		memcmp(&config->service_stream,
			&ProductCatalog_CurrentConfig.service_stream,
			sizeof(config->service_stream)) == 0 &&
		ProductConfigBridge_IsSupportedCan(config);
}

bool ProductConfigBridge_ValidateRuntime(const ProductConfig *config,
	const BspBoardRuntimeIdentity *board_identity,
	ProductConfigBridgeStatus *bridge_status)
{
	if (bridge_status != 0)
		*bridge_status = PRODUCT_CONFIG_BRIDGE_OK;
	if (config == 0 || config->board == 0 || config->motor == 0 ||
		config->load == 0 || config->identity.product_id == 0U ||
		config->identity.configuration_fingerprint !=
			PRODUCT_CATALOG_CONFIGURATION_FINGERPRINT ||
		config->identity.configuration_schema_version !=
			PRODUCT_CONFIG_SCHEMA_VERSION)
	{
		if (bridge_status != 0)
			*bridge_status = PRODUCT_CONFIG_BRIDGE_PRODUCT_INVALID;
		return false;
	}
	if (board_identity == 0 || board_identity->board_id == 0U ||
		board_identity->binding_fingerprint == 0U)
	{
		if (bridge_status != 0)
			*bridge_status = PRODUCT_CONFIG_BRIDGE_BOARD_INVALID;
		return false;
	}
	if (config->board->design_id != board_identity->board_id ||
		config->board->bsp_binding_fingerprint !=
			board_identity->binding_fingerprint)
	{
		if (bridge_status != 0)
			*bridge_status = PRODUCT_CONFIG_BRIDGE_BINDING_MISMATCH;
		return false;
	}
	if (!ProductConfigBridge_IsSupportedRuntime(config))
	{
		if (bridge_status != 0)
			*bridge_status = PRODUCT_CONFIG_BRIDGE_RUNTIME_UNSUPPORTED;
		return false;
	}
	return true;
}
