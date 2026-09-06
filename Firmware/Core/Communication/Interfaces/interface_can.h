#ifndef CORE_COMMUNICATION_INTERFACES_CAN_INTERFACE_H
#define CORE_COMMUNICATION_INTERFACES_CAN_INTERFACE_H

#include <stdbool.h>
#include <stdint.h>

#include "Bsp/Api/bsp_communication.h"
#include "Core/Application/Contracts/can_configuration_port.h"
#include "Core/Communication/Contracts/can_response_port.h"
#include "Core/Application/communication_watchdog_service.h"
#include "Core/Application/control_authority_service.h"
#include "Core/Communication/Router/can_command_router.h"
#include "Core/Communication/Protocol/can_protocol_v1.h"

#define CAN_INTERFACE_DECODED_QUEUE_COUNT 4U

#if ((CAN_INTERFACE_DECODED_QUEUE_COUNT & \
	(CAN_INTERFACE_DECODED_QUEUE_COUNT - 1U)) != 0U) || \
	(CAN_INTERFACE_DECODED_QUEUE_COUNT > 127U)
#error "CAN decoded queue count must be a power of two no greater than 127"
#endif

typedef struct
{
	CanProtocolV1Command command;
	uint8_t configuration_generation;
} CanInterfaceDecodedCommand;

typedef struct
{
	/* Active protocol identity. Updated in background while RX is gated. */
	volatile uint8_t node_id;
	uint8_t requested_node_id;
	volatile uint8_t configuration_generation;
	uint32_t baudrate;
	uint32_t configured_bitrate;
	uint32_t nominal_bitrate_kbps;
	BspCanFrame transmit_frame;
	CanInterfaceDecodedCommand decoded_queue[CAN_INTERFACE_DECODED_QUEUE_COUNT];
	volatile uint8_t decoded_read_sequence;
	volatile uint8_t decoded_write_sequence;
	volatile uint32_t last_valid_rx_ms;
	volatile uint32_t rx_generation;
	uint32_t supervised_rx_generation;
	volatile bool received_once;
	bool transmit_pending;
	bool heartbeat_enabled;
	bool disconnect_reported;
	bool enable_fd;
	bool enable_brs;
	volatile bool transport_started;
	volatile uint32_t heartbeat_timeout_ms;
	uint32_t minimum_heartbeat_ms;
	uint32_t maximum_heartbeat_ms;
	uint32_t heartbeat_reference_ms;
	volatile BspCommunicationFaultSet rx_pipeline_faults;
	/* Sticky until this context is reinitialized. */
	volatile BspCommunicationFaultSet observed_transport_faults;
	BspCanPort transport;
	BspMonotonicClockPort clock;
	BspCriticalSectionPort critical_section;
	ControlAuthorityServiceContext *control_authority;
} CanInterfaceContext;

void CanInterface_ApplyConfiguredBitrate(CanInterfaceContext *context,
	CommunicationWatchdogServiceContext *watchdog);
bool CanInterface_Initialize(CanInterfaceContext *context,
	const BspCanPort *transport,
	const BspMonotonicClockPort *clock,
	const BspCriticalSectionPort *critical_section,
	ControlAuthorityServiceContext *control_authority,
	bool enable_fd, bool enable_brs, uint32_t default_nominal_bitrate_kbps,
	uint32_t default_data_bitrate_kbps,
	uint32_t minimum_heartbeat_ms, uint32_t maximum_heartbeat_ms);
CanConfigurationPort CanInterface_CreateConfigurationPort(
	CanInterfaceContext *context);
CanResponsePort CanInterface_CreateResponsePort(CanInterfaceContext *context);
/* Called only from the FDCAN RX ISR; it drains the BSP receive endpoint. */
void CanInterface_OnReceiveInterrupt(CanInterfaceContext *context);
/* Bounded 1 kHz supervision; never stops or reconfigures the CAN peripheral. */
void CanInterface_Supervise1kHz(CanInterfaceContext *context,
	bool can_motion_mode_active,
	CommunicationWatchdogServiceContext *watchdog);
/* The only live-reconfiguration and Router/Application execution path. */
void CanInterface_RunBackground(CanInterfaceContext *context,
	CanCommandRouterContext *router,
	CommunicationWatchdogServiceContext *watchdog);
BspCommunicationFaultSet CanInterface_GetObservedTransportFaults(
	const CanInterfaceContext *context);

#endif
