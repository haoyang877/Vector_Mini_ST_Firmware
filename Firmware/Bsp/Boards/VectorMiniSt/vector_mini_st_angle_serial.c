#include "vector_mini_st_angle_serial.h"

#include "main.h"
#include "spi.h"

#define VECTOR_MINI_ST_ANGLE_SERIAL_SPIN_LIMIT 340U

static const AngleSerialStm32g431ResourceConfig OnboardAngleSerialResources =
{
	&hspi2,
	BRD_ENC_CS_GPIO_Port,
	GPIOB,
	VECTOR_MINI_ST_ANGLE_SERIAL_SPIN_LIMIT,
	BRD_ENC_CS_Pin,
	15U,
	5U,
	2U,
	true
};

static const AngleSerialStm32g431ResourceConfig ExternalAngleSerialResources =
{
	&hspi2,
	EXT_ENC_CS_GPIO_Port,
	GPIOB,
	VECTOR_MINI_ST_ANGLE_SERIAL_SPIN_LIMIT,
	EXT_ENC_CS_Pin,
	15U,
	5U,
	2U,
	true
};

const AngleSerialStm32g431ResourceConfig *
	BspVectorMiniSt_FindAngleSerialResources(BspEndpointId endpoint_id)
{
	if (endpoint_id == BSP_VECTOR_MINI_ST_ANGLE_ENDPOINT_ONBOARD)
		return &OnboardAngleSerialResources;
	if (endpoint_id == BSP_VECTOR_MINI_ST_ANGLE_ENDPOINT_EXTERNAL)
		return &ExternalAngleSerialResources;
	return 0;
}
