#ifndef FIRMWARE_BSP_BOARDS_BSP_BOARD_H
#define FIRMWARE_BSP_BOARDS_BSP_BOARD_H

#include "../Api/bsp_angle_sensor.h"
#include "../Api/bsp_communication.h"
#include "../Api/bsp_motor_drive.h"
#include "../Api/bsp_system.h"
#include "../Api/bsp_temperature.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BSP_BOARD_MAX_ANGLE_BINDING_COUNT 4U
#define BSP_BOARD_MAX_TEMPERATURE_BINDING_COUNT 4U
#define BSP_BOARD_MAX_COMMUNICATION_BINDING_COUNT 4U

typedef struct
{
	uint32_t board_id;
	uint32_t binding_fingerprint;
} BspBoardRuntimeIdentity;

typedef struct
{
	const char *board_name;
	uint32_t board_id;
	uint32_t binding_fingerprint;
	const BspMotorDriveEndpointCapabilities *motor_drive_endpoints;
	size_t motor_drive_endpoint_count;
	const BspAngleSensorEndpointCapabilities *angle_sensor_endpoints;
	size_t angle_sensor_endpoint_count;
	const BspTemperatureEndpointCapabilities *temperature_endpoints;
	size_t temperature_endpoint_count;
	const BspCommunicationEndpointCapabilities *communication_endpoints;
	size_t communication_endpoint_count;
	BspSystemCapabilities system;
} BspBoardCapabilities;

typedef struct
{
	BspEndpointId endpoint_id;
	/* UNSPECIFIED accepts any endpoint kind supported by the device adapter. */
	BspAngleEndpointKind required_endpoint_kind;
} BspAngleEndpointBindingRequest;

typedef struct
{
	BspEndpointId endpoint_id;
	BspTemperatureSourceKind required_source_kind;
	BspTemperatureLocation required_location;
} BspTemperatureEndpointBindingRequest;

typedef struct
{
	BspEndpointId endpoint_id;
	/* UNSPECIFIED accepts either framed CAN or a byte stream. */
	BspCommunicationKind required_kind;
	BspCommunicationFeatureSet required_features;
	uint16_t required_payload_bytes;
	/* Requested rates are in bits per second; zero means not applicable. */
	uint32_t required_nominal_bit_rate;
	uint32_t required_data_bit_rate;
} BspCommunicationEndpointBindingRequest;

/*
 * Composition creates this bounded, transient projection from ProductConfig.
 * It is not a second configuration source and must never be persisted or
 * edited by a user. Product roles and policies remain exclusively in
 * ProductConfig; this request contains only physical binding requirements.
 */
typedef struct
{
	BspEndpointId motor_drive_endpoint;
	BspCurrentSenseTopology current_sense_topology;
	uint8_t current_sensor_binding_count;
	BspEndpointId
		current_sensor_endpoints[BSP_MOTOR_MAX_CURRENT_SENSOR_COUNT];
	bool require_synchronized_current_sampling;
	bool require_hardware_shutdown;
	uint8_t angle_binding_count;
	BspAngleEndpointBindingRequest
		angle_bindings[BSP_BOARD_MAX_ANGLE_BINDING_COUNT];
	uint8_t temperature_binding_count;
	BspTemperatureEndpointBindingRequest
		temperature_bindings[BSP_BOARD_MAX_TEMPERATURE_BINDING_COUNT];
	uint8_t communication_binding_count;
	BspCommunicationEndpointBindingRequest
		communication_bindings[BSP_BOARD_MAX_COMMUNICATION_BINDING_COUNT];
	BspSystemFeatureSet required_system_features;
} BspBoardBindingRequest;

typedef enum
{
	BSP_BOARD_RESOURCE_NONE = 0,
	BSP_BOARD_RESOURCE_MOTOR_DRIVE,
	BSP_BOARD_RESOURCE_ANGLE_SENSOR,
	BSP_BOARD_RESOURCE_TEMPERATURE,
	BSP_BOARD_RESOURCE_COMMUNICATION,
	BSP_BOARD_RESOURCE_SYSTEM
} BspBoardResourceKind;

typedef enum
{
	BSP_BOARD_VALIDATION_OK = 0,
	BSP_BOARD_VALIDATION_NULL_ARGUMENT,
	BSP_BOARD_VALIDATION_INVALID_DESCRIPTOR,
	BSP_BOARD_VALIDATION_DUPLICATE_ENDPOINT_ID,
	BSP_BOARD_VALIDATION_BINDING_COUNT_EXCEEDED,
	BSP_BOARD_VALIDATION_ENDPOINT_NOT_FOUND,
	BSP_BOARD_VALIDATION_ENDPOINT_UNAVAILABLE,
	BSP_BOARD_VALIDATION_DUPLICATE_BINDING,
	BSP_BOARD_VALIDATION_CURRENT_SENSE_TOPOLOGY_INVALID,
	BSP_BOARD_VALIDATION_CURRENT_SENSE_TOPOLOGY_UNSUPPORTED,
	BSP_BOARD_VALIDATION_CURRENT_SENSOR_CAPACITY_INSUFFICIENT,
	BSP_BOARD_VALIDATION_CURRENT_SENSOR_BINDING_MISMATCH,
	BSP_BOARD_VALIDATION_SYNCHRONIZED_SAMPLING_UNSUPPORTED,
	BSP_BOARD_VALIDATION_HARDWARE_SHUTDOWN_UNSUPPORTED,
	BSP_BOARD_VALIDATION_ENDPOINT_KIND_MISMATCH,
	BSP_BOARD_VALIDATION_ENDPOINT_FEATURE_UNSUPPORTED,
	BSP_BOARD_VALIDATION_COMMUNICATION_PAYLOAD_UNSUPPORTED,
	BSP_BOARD_VALIDATION_COMMUNICATION_BIT_RATE_UNSUPPORTED,
	BSP_BOARD_VALIDATION_SYSTEM_FEATURE_UNSUPPORTED
} BspBoardValidationCode;

#define BSP_BOARD_VALIDATION_NO_BINDING_INDEX ((size_t)-1)

typedef struct
{
	BspBoardValidationCode code;
	BspBoardResourceKind resource;
	size_t binding_index;
	BspEndpointId endpoint_id;
} BspBoardValidationResult;

/* Both validators are deterministic, allocation-free, and perform no I/O. */
BspBoardValidationResult BspBoard_ValidateCapabilities(
	const BspBoardCapabilities *capabilities);
BspBoardValidationResult BspBoard_ValidateBindingRequest(
	const BspBoardCapabilities *capabilities,
	const BspBoardBindingRequest *request);
const BspCommunicationEndpointCapabilities *
	BspBoard_FindCommunicationEndpoint(
		const BspBoardCapabilities *capabilities, BspEndpointId endpoint_id);

#ifdef __cplusplus
}
#endif

#endif
