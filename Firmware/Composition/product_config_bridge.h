#ifndef FIRMWARE_COMPOSITION_PRODUCT_CONFIG_BRIDGE_H
#define FIRMWARE_COMPOSITION_PRODUCT_CONFIG_BRIDGE_H

#include "bsp_board.h"
#include "product_config.h"
#include "product_runtime_selection.h"

typedef enum
{
	PRODUCT_CONFIG_BRIDGE_OK = 0,
	PRODUCT_CONFIG_BRIDGE_PRODUCT_INVALID,
	PRODUCT_CONFIG_BRIDGE_BOARD_INVALID,
	PRODUCT_CONFIG_BRIDGE_BINDING_MISMATCH,
	PRODUCT_CONFIG_BRIDGE_RUNTIME_CONFIG_INVALID,
	PRODUCT_CONFIG_BRIDGE_RUNTIME_CONFIG_MISMATCH
} ProductConfigBridgeStatus;

/* Compact startup guard used by the flash-constrained production target. */
bool ProductConfigBridge_ValidateRuntime(const ProductRuntimeSelection *selection,
	const BspBoardRuntimeIdentity *board_identity,
	uint32_t runtime_configuration_fingerprint,
	ProductConfigBridgeStatus *bridge_status);

#endif
