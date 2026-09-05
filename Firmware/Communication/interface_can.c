#include "interface_can.h"
#include "telemetry_service.h"
#include "can_protocol_v1.h"
#include "can_command_router.h"
#include "communication_watchdog_service.h"

#define PROTOCOL_MODE_CURRENT              1
#define PROTOCOL_MODE_SPEED                2
#define PROTOCOL_MODE_POSITION             3
#define PROTOCOL_MODE_POSITION_IMPEDANCE  18

static CanInterfaceContext *ActiveContext;
#define CANContext (*ActiveContext)
#define CANRxQueue (ActiveContext->receive_queue)
#define CANRxQueueWriteIndex (ActiveContext->receive_write_index)
#define CANRxQueueReadIndex (ActiveContext->receive_read_index)
#define CANDisconnectClearPending (ActiveContext->disconnect_clear_pending)
#define CANTransport (ActiveContext->transport)
#define CANTransportInitialized (ActiveContext->transport_is_initialized)
#define CAN_RX_QUEUE_CAPACITY CAN_INTERFACE_RX_QUEUE_CAPACITY
#define CANReceivedCommand CanReceivedCommand

static bool CanInterface_SetNodeId(void *context, uint8_t node_id)
{
	(void)context;
	if (node_id > 7U)
		return false;
	CANContext.node_id = node_id;
	return true;
}

static uint8_t CanInterface_GetNodeId(void *context)
{
	(void)context;
	return CANContext.node_id;
}

static bool CanInterface_SetBitrateKbps(void *context,
	uint32_t bitrate_kbps)
{
	(void)context;
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

static uint32_t CanInterface_GetBitrateKbps(void *context)
{
	(void)context;
	return CANContext.baudrate;
}

static bool CanInterface_SetHeartbeatMs(void *context,
	uint32_t heartbeat_ms)
{
	(void)context;
	if (heartbeat_ms != 0U &&
		(heartbeat_ms < 500U || heartbeat_ms > 1000U))
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

static uint32_t CanInterface_GetHeartbeatMs(void *context)
{
	(void)context;
	return CANContext.heartbeat_timeout_ms;
}

CanConfigurationPort CanInterface_CreateConfigurationPort(
	CanInterfaceContext *context)
{
	CanConfigurationPort port;

	ActiveContext = context;
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
void CanInterface_ApplyConfiguredBitrate(void)
{
	if (!CANTransportInitialized ||
		!CANTransport.initialize(CANTransport.context, CANContext.node_id))
		(void)CommunicationWatchdogService_ReportDisconnected();
}

bool CanInterface_Initialize(CanInterfaceContext *context,
	const CanTransportPort *transport)
{
	if (context == 0 || transport == 0 || transport->initialize == 0 ||
		transport->configure_bitrate_kbps == 0 || transport->receive == 0 ||
		transport->transmit == 0)
		return false;
	ActiveContext = context;
	CANTransport = *transport;
	CANTransportInitialized = true;
	CANContext.baudrate = 1000U;
	CANContext.configured_bitrate = 1000U;
	return true;
}

/**
	* @brief  Handle CAN heartbeat disconnect protection
 **/
void CanInterface_UpdateWatchdog(void)
{
	float mode_value = 0.0f;
	int mode;

	(void)TelemetryService_ReadValue(MOTOR_TELEMETRY_MODE, &mode_value);
	mode = (int)mode_value;
	CANContext.heartbeat_enabled = CANContext.heartbeat_timeout_ms != 0U &&
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
			(void)CommunicationWatchdogService_ReportDisconnected();
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
void CanInterface_ApplyPendingBitrate(void)
{
	if(CANContext.configured_bitrate != CANContext.baudrate)
    {
		if (!CANTransport.configure_bitrate_kbps(CANTransport.context,
			CANContext.baudrate))
			(void)CommunicationWatchdogService_ReportDisconnected();
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
static bool CanInterface_QueueResponse(void *context, uint8_t parameter_id, float data)
{
	CanTransportFrame frame;
	CanParameterId param_id = (CanParameterId)parameter_id;

	(void)context;

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

	ActiveContext = context;
	port.context = context;
	port.queue_response = CanInterface_QueueResponse;
	return port;
}

/**
	* @brief  CAN Rx interrupt Handle
			  extract param id and data from mail box
 **/
void CanInterface_OnReceiveInterrupt(void)
{
	CanTransportFrame frame;
	CanProtocolV1Command decoded;
	uint8_t next_write_index;

	if (!CANTransportInitialized ||
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

void CanInterface_ProcessReceivedFrames(void)
{
	CANReceivedCommand command;

	if (CANDisconnectClearPending != 0U)
	{
		CANDisconnectClearPending = 0U;
		(void)CommunicationWatchdogService_ReportFrameReceived();
	}

	if (CANRxQueueReadIndex == CANRxQueueWriteIndex)
		return;

	command = CANRxQueue[CANRxQueueReadIndex];
	CANRxQueueReadIndex = (uint8_t)((CANRxQueueReadIndex + 1U) %
		CAN_RX_QUEUE_CAPACITY);
	CanCommandRouter_Handle(command.parameter, command.value);
}

/**
	* @brief  CAN Tx function
			  use ExtId, DLC length 4
 **/
void CanInterface_FlushTransmit(void)
{
	CanTransportFrame frame;

	if(!CANContext.transmit_pending)
		return;
	if (!CANTransportInitialized)
		return;
	if (!CanProtocolV1_Encode(CANContext.node_id,
		(uint8_t)CANContext.tx_parameter_id, CANContext.tx_value, &frame))
		return;
	if (CANTransport.transmit(CANTransport.context, &frame))
		CANContext.transmit_pending = false;

}
