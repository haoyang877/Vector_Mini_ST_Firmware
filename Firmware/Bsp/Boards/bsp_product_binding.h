#ifndef FIRMWARE_BSP_BOARDS_BSP_PRODUCT_BINDING_H
#define FIRMWARE_BSP_BOARDS_BSP_PRODUCT_BINDING_H

#include "bsp_board.h"
#include "product_config.h"

/* Full host/build-gate validation of one product against one physical board. */
bool BspProductBinding_Validate(const ProductConfig *config,
	const BspBoardCapabilities *board_capabilities,
	ProductConfigValidationResult *product_result,
	BspBoardValidationResult *board_result);

#endif
