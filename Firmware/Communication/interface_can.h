#ifndef COMMUNICATION_CAN_INTERFACE_H
#define COMMUNICATION_CAN_INTERFACE_H

#include <stdbool.h>
#include <stdint.h>
#include "can_transport_port.h"
#include "Core/Application/Contracts/can_configuration_port.h"
#include "Core/Application/Contracts/can_response_port.h"
#include "can_protocol_v1.h"
#include "can_command_router.h"
#include "communication_watchdog_service.h"
#include "telemetry_service.h"
#include "control_authority_service.h"

#define CAN_INTERFACE_RX_QUEUE_CAPACITY 8U

typedef struct
{
	CanParameterId parameter;
	float value;
} CanReceivedCommand;

typedef struct
{
	volatile uint8_t node_id;
	uint32_t baudrate;
	uint32_t configured_bitrate;
	CanParameterId tx_parameter_id;
	float tx_value;
	volatile bool received_once;
	bool transmit_pending;
	bool heartbeat_enabled;
	bool disconnect_reported;
	uint32_t heartbeat_timeout_ms;
	uint32_t minimum_heartbeat_ms;
	uint32_t maximum_heartbeat_ms;
	volatile uint32_t heartbeat_elapsed_ms;
	volatile uint32_t receive_overflow_count;
	CanReceivedCommand receive_queue[CAN_INTERFACE_RX_QUEUE_CAPACITY];
	volatile uint8_t receive_write_index;
	volatile uint8_t receive_read_index;
	volatile uint8_t disconnect_clear_pending;
	CanTransportPort transport;
	bool transport_is_initialized;
	ControlAuthorityServiceContext *control_authority;
} CanInterfaceContext;

void CanInterface_ApplyConfiguredBitrate(CanInterfaceContext *context,
	CommunicationWatchdogServiceContext *watchdog);
bool CanInterface_Initialize(CanInterfaceContext *context,
	const CanTransportPort *transport,
	ControlAuthorityServiceContext *control_authority,
	uint32_t default_bitrate_kbps, uint32_t minimum_heartbeat_ms,
	uint32_t maximum_heartbeat_ms);
CanConfigurationPort CanInterface_CreateConfigurationPort(
	CanInterfaceContext *context);
CanResponsePort CanInterface_CreateResponsePort(CanInterfaceContext *context);
void CanInterface_UpdateWatchdog(CanInterfaceContext *context,
	const TelemetryServiceContext *telemetry,
	CommunicationWatchdogServiceContext *watchdog);
void CanInterface_ApplyPendingBitrate(CanInterfaceContext *context,
	CommunicationWatchdogServiceContext *watchdog);
void CanInterface_OnReceiveInterrupt(CanInterfaceContext *context);
void CanInterface_ProcessReceivedFrames(CanInterfaceContext *context,
	CanCommandRouterContext *router,
	CommunicationWatchdogServiceContext *watchdog);
void CanInterface_FlushTransmit(CanInterfaceContext *context);
#endif
