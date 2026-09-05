#include "interface_can.h"
#include "telemetry_service.h"
#include "can_protocol_v1.h"
#include "can_command_router.h"
#include "communication_watchdog_service.h"

#include <string.h>

#define PROTOCOL_MODE_CURRENT              1
#define PROTOCOL_MODE_SPEED                2
#define PROTOCOL_MODE_POSITION             3
#define PROTOCOL_MODE_POSITION_IMPEDANCE  18

#define CANContext (*context)
#define CANRxQueue (context->receive_queue)
#define CANRxQueueWriteIndex (context->receive_write_index)
#define CANRxQueueReadIndex (context->receive_read_index)
#define CANDisconnectClearPending (context->disconnect_clear_pending)
#define CANTransport (context->transport)
#define CANTransportInitialized (context->transport_is_initialized)
#define CAN_RX_QUEUE_CAPACITY CAN_INTERFACE_RX_QUEUE_CAPACITY
#define CANReceivedCommand CanReceivedCommand

static bool CanInterface_SetNodeId(void *raw_context, uint8_t node_id)
{
	CanInterfaceContext *context = (CanInterfaceContext *)raw_context;
	if (context == 0)
		return false;
	if (node_id > 7U)
		return false;
	if (CANTransportInitialized && CANContext.node_id != node_id &&
		!CANTransport.configure_node_id(CANTransport.context, node_id))
		return false;
	CANContext.node_id = node_id;
	return true;
}

static uint8_t CanInterface_GetNodeId(void *raw_context)
{
	CanInterfaceContext *context = (CanInterfaceContext *)raw_context;
	if (context == 0)
		return 0U;
	return CANContext.node_id;
}

static bool CanInterface_SetBitrateKbps(void *raw_context,
	uint32_t bitrate_kbps)
{
	CanInterfaceContext *context = (CanInterfaceContext *)raw_context;
	if (context == 0)
		return false;
	if (CANTransportInitialized && CANTransport.maximum_bitrate_kbps != 0U &&
		bitrate_kbps > CANTransport.maximum_bitrate_kbps)
		return false;
	switch (bitrate_kbps)
	{
		case 100U:
		case 125U:
		case 200U:
		case 250U:
		case 500U:
		case 1000U:
		case 2000U:
		case 2500U:
		case 5000U:
			CANContext.baudrate = bitrate_kbps;
			return true;
		default:
			return false;
	}
}

static uint32_t CanInterface_GetBitrateKbps(void *raw_context)
{
	CanInterfaceContext *context = (CanInterfaceContext *)raw_context;
	if (context == 0)
		return 0U;
	return CANContext.baudrate;
}

static bool CanInterface_SetHeartbeatMs(void *raw_context,
	uint32_t heartbeat_ms)
{
	CanInterfaceContext *context = (CanInterfaceContext *)raw_context;
	if (context == 0)
		return false;
	if (heartbeat_ms != 0U &&
		(heartbeat_ms < CANContext.minimum_heartbeat_ms ||
		 heartbeat_ms > CANContext.maximum_heartbeat_ms))
		return false;
	CANContext.heartbeat_timeout_ms = heartbeat_ms;
	if (heartbeat_ms == 0U)
	{
		CANContext.heartbeat_enabled = false;
		CANContext.heartbeat_elapsed_ms = 0U;
		CANContext.disconnect_reported = false;
	}
	return true;
}

static uint32_t CanInterface_GetHeartbeatMs(void *raw_context)
{
	CanInterfaceContext *context = (CanInterfaceContext *)raw_context;
	if (context == 0)
		return 0U;
	return CANContext.heartbeat_timeout_ms;
}

CanConfigurationPort CanInterface_CreateConfigurationPort(
	CanInterfaceContext *context)
{
	CanConfigurationPort port;

	port.context = context;
	port.set_node_id = CanInterface_SetNodeId;
	port.get_node_id = CanInterface_GetNodeId;
	port.set_bitrate_kbps = CanInterface_SetBitrateKbps;
	port.get_bitrate_kbps = CanInterface_GetBitrateKbps;
	port.set_heartbeat_ms = CanInterface_SetHeartbeatMs;
	port.get_heartbeat_ms = CanInterface_GetHeartbeatMs;
	return port;
}





/**
	* @brief  FDCAN1 Filter Init
			  Standard ID, Range Mode
 **/
void CanInterface_ApplyConfiguredBitrate(CanInterfaceContext *context,
	CommunicationWatchdogServiceContext *watchdog)
{
	if (context == 0 || !CANTransportInitialized ||
		!CANTransport.initialize(CANTransport.context, CANContext.node_id))
		(void)CommunicationWatchdogService_ReportDisconnected(watchdog);
}

bool CanInterface_Initialize(CanInterfaceContext *context,
	const CanTransportPort *transport,
	ControlAuthorityServiceContext *control_authority,
	uint32_t default_bitrate_kbps, uint32_t minimum_heartbeat_ms,
	uint32_t maximum_heartbeat_ms)
{
	if (context == 0 || transport == 0 || transport->initialize == 0 ||
		transport->configure_node_id == 0 ||
		transport->configure_bitrate_kbps == 0 || transport->receive == 0 ||
		transport->transmit == 0 || control_authority == 0 ||
		default_bitrate_kbps == 0U ||
		minimum_heartbeat_ms > maximum_heartbeat_ms)
		return false;
	memset(context, 0, sizeof(*context));
	CANTransport = *transport;
	CANTransportInitialized = true;
	CANContext.control_authority = control_authority;
	CANContext.baudrate = default_bitrate_kbps;
	CANContext.configured_bitrate = default_bitrate_kbps;
	CANContext.minimum_heartbeat_ms = minimum_heartbeat_ms;
	CANContext.maximum_heartbeat_ms = maximum_heartbeat_ms;
	return true;
}

/**
	* @brief  Handle CAN heartbeat disconnect protection
 **/
void CanInterface_UpdateWatchdog(CanInterfaceContext *context,
	const TelemetryServiceContext *telemetry,
	CommunicationWatchdogServiceContext *watchdog)
{
	float mode_value = 0.0f;
	int mode;

	if (context == 0)
		return;
	(void)TelemetryService_ReadValue(telemetry, MOTOR_TELEMETRY_MODE,
		&mode_value);
	mode = (int)mode_value;
	CANContext.heartbeat_enabled = CANContext.heartbeat_timeout_ms != 0U &&
		ControlAuthorityService_IsOwner(CANContext.control_authority,
			CONTROL_AUTHORITY_CAN) &&
		(mode == PROTOCOL_MODE_CURRENT || mode == PROTOCOL_MODE_SPEED ||
		 mode == PROTOCOL_MODE_POSITION ||
		 mode == PROTOCOL_MODE_POSITION_IMPEDANCE);

	if(CANContext.received_once && CANContext.heartbeat_enabled)
	{
		/*timeout protect*/
		if (CANContext.heartbeat_elapsed_ms <
			CANContext.heartbeat_timeout_ms)
			CANContext.heartbeat_elapsed_ms++;
		if (CANContext.heartbeat_elapsed_ms >=
			CANContext.heartbeat_timeout_ms &&
			!CANContext.disconnect_reported)
		{
			(void)CommunicationWatchdogService_ReportDisconnected(watchdog);
			CANContext.disconnect_reported = true;
		}
	}
}

/**
	* @brief  Set encoder state from CAN parameter value
	* @param  data: encoded encoder state value
 **/


/**
	* @brief  Switch CAN baudrate when baudrate setting changes
 **/
void CanInterface_ApplyPendingBitrate(CanInterfaceContext *context,
	CommunicationWatchdogServiceContext *watchdog)
{
	if (context == 0)
		return;
	if(CANContext.configured_bitrate != CANContext.baudrate)
    {
		if (!CANTransport.configure_bitrate_kbps(CANTransport.context,
			CANContext.baudrate))
			(void)CommunicationWatchdogService_ReportDisconnected(watchdog);
		else
			CANContext.configured_bitrate = CANContext.baudrate;
	}
}

/**
	* @brief  Get encoded encoder state
	* @retval encoded encoder state value
 **/


/**
	* @brief  Handle received CAN message
			  update motor control paramters
    * @param  param_id: CAN parameter id
    * @param  data: CAN parameter data
 **/


/**
	* @brief  Update CAN transmit message data
	* @param  param_id: CAN parameter id
	* @param  data: CAN transmit data
 **/
static bool CanInterface_QueueResponse(void *raw_context,
	uint8_t parameter_id, float data)
{
	CanInterfaceContext *context = (CanInterfaceContext *)raw_context;
	CanTransportFrame frame;
	CanParameterId param_id = (CanParameterId)parameter_id;

	if (context == 0)
		return false;

	if (!CanProtocolV1_Encode(CANContext.node_id, (uint8_t)param_id,
		data, &frame))
		return false;
	CANContext.tx_parameter_id = param_id;
	CANContext.tx_value = data;
	CANContext.transmit_pending = true;
	return true;
}

CanResponsePort CanInterface_CreateResponsePort(CanInterfaceContext *context)
{
	CanResponsePort port;

	port.context = context;
	port.queue_response = CanInterface_QueueResponse;
	return port;
}

/**
	* @brief  CAN Rx interrupt Handle
			  extract param id and data from mail box
 **/
void CanInterface_OnReceiveInterrupt(CanInterfaceContext *context)
{
	CanTransportFrame frame;
	CanProtocolV1Command decoded;
	uint8_t next_write_index;

	if (context == 0 || !CANTransportInitialized ||
		!CANTransport.receive(CANTransport.context, &frame))
		return;
	if (!CanProtocolV1_Decode(&frame, CANContext.node_id, &decoded))
		return;

	CANContext.received_once = true;
	CANContext.heartbeat_elapsed_ms = 0U;
	CANContext.disconnect_reported = false;
	CANDisconnectClearPending = 1U;
	next_write_index = (uint8_t)((CANRxQueueWriteIndex + 1U) %
		CAN_RX_QUEUE_CAPACITY);
	if (next_write_index == CANRxQueueReadIndex)
	{
		CANContext.receive_overflow_count++;
	}
	else
	{
		CANRxQueue[CANRxQueueWriteIndex].parameter =
			(CanParameterId)decoded.parameter_id;
		CANRxQueue[CANRxQueueWriteIndex].value = decoded.value;
		CANRxQueueWriteIndex = next_write_index;
	}
}

void CanInterface_ProcessReceivedFrames(CanInterfaceContext *context,
	CanCommandRouterContext *router,
	CommunicationWatchdogServiceContext *watchdog)
{
	CANReceivedCommand command;
	if (context == 0 || router == 0)
		return;

	if (CANDisconnectClearPending != 0U)
	{
		CANDisconnectClearPending = 0U;
		(void)CommunicationWatchdogService_ReportFrameReceived(watchdog);
	}

	if (CANRxQueueReadIndex == CANRxQueueWriteIndex)
		return;

	command = CANRxQueue[CANRxQueueReadIndex];
	CANRxQueueReadIndex = (uint8_t)((CANRxQueueReadIndex + 1U) %
		CAN_RX_QUEUE_CAPACITY);
	CanCommandRouter_Handle(router, command.parameter, command.value);
}

/**
	* @brief  CAN Tx function
			  use ExtId, DLC length 4
 **/
void CanInterface_FlushTransmit(CanInterfaceContext *context)
{
	CanTransportFrame frame;

	if (context == 0 || !CANContext.transmit_pending)
		return;
	if (!CANTransportInitialized)
		return;
	if (!CanProtocolV1_Encode(CANContext.node_id,
		(uint8_t)CANContext.tx_parameter_id, CANContext.tx_value, &frame))
		return;
	if (CANTransport.transmit(CANTransport.context, &frame))
		CANContext.transmit_pending = false;

}
