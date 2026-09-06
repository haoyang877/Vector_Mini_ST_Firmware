#ifndef BSP_BOARDS_VECTOR_MINI_ST_BOOTSTRAP_PRODUCT_CONFIG_BRIDGE_H
#define BSP_BOARDS_VECTOR_MINI_ST_BOOTSTRAP_PRODUCT_CONFIG_BRIDGE_H

#include "bsp_board.h"
#include "measurement_model.h"
#include "product_catalog.h"
#include "Core/Application/motor_commissioning_workflow.h"
#include "Core/Application/MotorControl/rotor_feedback_runtime.h"

typedef enum
{
	PRODUCT_CONFIG_BRIDGE_OK = 0,
	PRODUCT_CONFIG_BRIDGE_PRODUCT_INVALID,
	PRODUCT_CONFIG_BRIDGE_BOARD_INVALID,
	PRODUCT_CONFIG_BRIDGE_BINDING_MISMATCH,
	PRODUCT_CONFIG_BRIDGE_RUNTIME_UNSUPPORTED
} ProductConfigBridgeStatus;

/*
 * Runtime projection of the single Product current-sense source of truth.
 * Flash keeps its deployed three-slot ABI: phase A/B/C use slots 0/1/2,
 * DC-link uses slot 0, and every unused slot remains the verifiable 0/0/0
 * default/range tuple.
 */
typedef struct
{
	MeasurementCurrentSenseConfig measurement;
	uint16_t default_offset_adc[MEASUREMENT_MODEL_PHASE_COUNT];
	uint16_t minimum_offset_adc[MEASUREMENT_MODEL_PHASE_COUNT];
	uint16_t maximum_offset_adc[MEASUREMENT_MODEL_PHASE_COUNT];
} ProductCurrentSenseProjection;

#define PRODUCT_CONFIG_BRIDGE_TEMPERATURE_SENSOR_NONE UINT8_C(0xFF)

typedef struct
{
	uint8_t sensor_index;
	bool supervision_enabled;
	bool protection_enabled;
} ProductTemperatureRuntimeProjection;

/* Fast target startup check for catalog/manifest/BSP identity consistency and
 * bounded configuration shape. Rich Product/BSP compatibility is enforced by
 * host/build gates; concrete Bootstrap factories still resolve every endpoint
 * and fail closed when a selected leaf is unavailable. */
bool ProductConfigBridge_ValidateRuntime(const ProductCatalogEntry *entry,
	const BspBoardRuntimeIdentity *board_identity,
	const BspBoardCapabilities *board_capabilities,
	ProductConfigBridgeStatus *bridge_status);

bool ProductConfigBridge_ProjectCurrentSense(
	const ProductCurrentSenseConfig *current_sense,
	const BspMotorDriveEndpointCapabilities *motor_drive_capabilities,
	ProductCurrentSenseProjection *projection);

/* Translate the declarative Product feedback graph into the fixed, bounded
 * runtime slots.  Only the primary motor-rotor sensor owns the persisted
 * EncoderContext; a second sensor is admitted only as lightweight output
 * position feedback. */
bool ProductConfigBridge_ProjectFeedback(const ProductConfig *config,
	RotorFeedbackRuntimeConfig *projection);

/* Resolve feature policy into the only control modes that the projected
 * feedback topology can execute safely. */
bool ProductConfigBridge_ProjectControlModes(const ProductConfig *config,
	const RotorFeedbackRuntimeConfig *feedback,
	MotorControlModeMask *projection);

/* Select the single temperature channel supported by this target. Optional
 * monitoring may resolve to disabled when the physical endpoint is absent. */
bool ProductConfigBridge_ProjectTemperature(const ProductConfig *config,
	const BspBoardCapabilities *board_capabilities,
	ProductTemperatureRuntimeProjection *projection);

/* Map Product commissioning policy onto the fixed production workflow.  The
 * mask may remove stages, but MotorCommissioningWorkflow remains the sole
 * owner of their execution order. */
bool ProductConfigBridge_ProjectCommissioning(const ProductConfig *config,
	MotorCommissioningStageMask *projection);

#endif
