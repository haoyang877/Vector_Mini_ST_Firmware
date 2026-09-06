#ifndef FIRMWARE_COMPOSITION_PRODUCT_CONFIG_BRIDGE_H
#define FIRMWARE_COMPOSITION_PRODUCT_CONFIG_BRIDGE_H

#include "bsp_board.h"
#include "product_config.h"

typedef enum
{
	PRODUCT_CONFIG_BRIDGE_OK = 0,
	PRODUCT_CONFIG_BRIDGE_PRODUCT_INVALID,
	PRODUCT_CONFIG_BRIDGE_BOARD_INVALID,
	PRODUCT_CONFIG_BRIDGE_BINDING_MISMATCH,
	PRODUCT_CONFIG_BRIDGE_RUNTIME_UNSUPPORTED
} ProductConfigBridgeStatus;

/* Validates the selected product directly against the linked BSP identity and
 * the set of device/runtime strategies that are actually implemented. */
bool ProductConfigBridge_ValidateRuntime(const ProductConfig *config,
	const BspBoardRuntimeIdentity *board_identity,
	ProductConfigBridgeStatus *bridge_status);

#endif
