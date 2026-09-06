#ifndef FIRMWARE_BSP_BOARDS_VECTOR_MINI_ST_ANGLE_SERIAL_H
#define FIRMWARE_BSP_BOARDS_VECTOR_MINI_ST_ANGLE_SERIAL_H

#include "angle_serial_stm32g431.h"
#include "vector_mini_st_bsp.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Returns NULL when the board endpoint has no synchronous-serial binding. */
const AngleSerialStm32g431ResourceConfig *
	BspVectorMiniSt_FindAngleSerialResources(BspEndpointId endpoint_id);

#ifdef __cplusplus
}
#endif

#endif
