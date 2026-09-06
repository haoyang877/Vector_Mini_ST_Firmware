#ifndef FIRMWARE_CORE_CONFIG_PRODUCT_CATALOG_H
#define FIRMWARE_CORE_CONFIG_PRODUCT_CATALOG_H

#include "product_config.h"

/* Stable generic identifiers; concrete platform bindings live below Core. */
#define PRODUCT_CATALOG_PLATFORM_CURRENT_TARGET       0x00000431UL
#define PRODUCT_CATALOG_BOARD_CURRENT                 0x564D0001UL
#define PRODUCT_CATALOG_MOTOR_CURRENT                 0x48540001UL
#define PRODUCT_CATALOG_LOAD_DAMPED                   0x4C4F0001UL
#define PRODUCT_CATALOG_ANGLE_ABSOLUTE_SERIAL_16BIT   0x414E0001UL
#define PRODUCT_CATALOG_TEMPERATURE_INTERNAL          0x544D0001UL
#define PRODUCT_CATALOG_PRODUCT_CURRENT               0x564D5354UL
#define PRODUCT_CATALOG_VARIANT_CURRENT               0x00010001UL
#define PRODUCT_CATALOG_CONFIGURATION_FINGERPRINT     0x9C501111UL
#define PRODUCT_CATALOG_BSP_BINDING_FINGERPRINT       0x564D53A1UL

#define PRODUCT_CATALOG_ENDPOINT_CURRENT_PHASE_A      0x0101U
#define PRODUCT_CATALOG_ENDPOINT_CURRENT_PHASE_B      0x0102U
#define PRODUCT_CATALOG_ENDPOINT_CURRENT_PHASE_C      0x0103U
#define PRODUCT_CATALOG_ENDPOINT_ANGLE_PRIMARY        0x0201U
#define PRODUCT_CATALOG_ENDPOINT_TEMPERATURE_INTERNAL 0x0301U
#define PRODUCT_CATALOG_ENDPOINT_CAN_CONTROL          0x0401U
#define PRODUCT_CATALOG_ENDPOINT_SERVICE_STREAM       0x0402U

extern const ProductBoardDesign ProductCatalog_CurrentBoard;
extern const ProductMotorDesign ProductCatalog_CurrentMotor;
extern const ProductLoadDesign ProductCatalog_CurrentLoad;
extern const ProductAngleSensorDesign
	ProductCatalog_AbsoluteSerial16BitAngleSensor;
extern const ProductTemperatureSensorDesign
	ProductCatalog_InternalTemperatureSensor;
extern const ProductConfig ProductCatalog_CurrentConfig;

const ProductConfig *ProductCatalog_GetCurrent(void);

#endif
