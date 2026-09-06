#include "interface_can.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#if defined(__CC_ARM)
#pragma O3
#pragma Ospace
#endif

#define PROTOCOL_MODE_CURRENT              1
#define PROTOCOL_MODE_SPEED                2
#define PROTOCOL_MODE_POSITION             3
#define PROTOCOL_MODE_POSITION_IMPEDANCE  18

static bool CanInterface_IsSupportedBitrate(uint32_t bitrate_kbps)
{
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
			return true;
		default:
			return false;
	}
}

static uint32_t CanInterface_MaximumBitrateKbps(
	const CanInterfaceContext *context)
{
	uint32_t maximum_bit_rate;

	if (context == NULL || context->transport.capabilities == NULL)
		return 0U;
	maximum_bit_rate = context->enable_fd && context->enable_brs ?
		context->transport.capabilities->maximum_data_bit_rate :
		context->transport.capabilities->maximum_nominal_bit_rate;
	return maximum_bit_rate / 1000U;
}

static bool CanInterface_BuildTransportConfiguration(
	const CanInterfaceContext *context, uint8_t node_id,
	uint32_t bitrate_kbps, BspCanAcceptanceFilter *filter,
	BspCanConfiguration *configuration)
{
	if (context == NULL || filter == NULL || configuration == NULL ||
		node_id > 7U || !CanInterface_IsSupportedBitrate(bitrate_kbps) ||
		bitrate_kbps > CanInterface_MaximumBitrateKbps(context))
	{
		return false;
	}

	memset(configuration, 0, sizeof(*configuration));
	filter->filter_index = 0U;
	filter->identifier_kind = BSP_CAN_IDENTIFIER_STANDARD;
	filter->filter_kind = BSP_CAN_FILTER_RANGE;
	filter->identifier_a = (uint32_t)node_id << 8;
	filter->identifier_b = filter->identifier_a + 0xFFU;
	configuration->enable_fd = context->enable_fd;
	configuration->enable_brs = context->enable_brs;
	configuration->listen_only = false;
	configuration->acceptance_filters = filter;
	configuration->acceptance_filter_count = 1U;
	configuration->unmatched_standard_policy = BSP_CAN_UNMATCHED_REJECT;
	configuration->unmatched_extended_policy = BSP_CAN_UNMATCHED_REJECT;
	configuration->remote_standard_policy = BSP_CAN_REMOTE_REJECT;
	configuration->remote_extended_policy = BSP_CAN_REMOTE_REJECT;
	if (context->enable_fd && context->enable_brs)
	{
		configuration->nominal_bit_rate =
			context->nominal_bitrate_kbps * 1000U;
		configuration->data_bit_rate = bitrate_kbps * 1000U;
	}
	else
	{
		configuration->nominal_bit_rate = bitrate_kbps * 1000U;
		configuration->data_bit_rate = context->enable_fd ?
			configuration->nominal_bit_rate : 0U;
	}
	return true;
}

static bool CanInterface_ConfigureAndStart(CanInterfaceContext *context,
	uint8_t node_id, uint32_t bitrate_kbps)
{
	BspCanAcceptanceFilter filter;
	BspCanConfiguration configuration;

	if (!CanInterface_BuildTransportConfiguration(context, node_id,
		bitrate_kbps, &filter, &configuration))
	{
		return false;
	}
	if (context->transport.configure(context->transport.context,
		&configuration) != BSP_RESULT_OK)
	{
		return false;
	}
	return context->transport.start(context->transport.context) == BSP_RESULT_OK;
}

static bool CanInterface_ApplyTransportConfiguration(
	CanInterfaceContext *context, uint8_t node_id, uint32_t bitrate_kbps)
{
	bool was_started;

	if (context == NULL || !context->transport_is_initialized)
		return false;
	was_started = context->transport_started;
	if (was_started &&
		context->transport.stop(context->transport.context) != BSP_RESULT_OK)
	{
		return false;
	}
	context->transport_started = false;
	if (CanInterface_ConfigureAndStart(context, node_id, bitrate_kbps))
	{
		context->transport_started = true;
		return true;
	}

	/* A failed live reconfiguration must not silently change the active
	 * protocol identity. Best-effort rollback restores the last applied pair. */
	if (was_started && CanInterface_ConfigureAndStart(context,
		context->node_id, context->configured_bitrate))
	{
		context->transport_started = true;
	}
	return false;
}

static bool CanInterface_SetNodeId(void *raw_context, uint8_t node_id)
{
	CanInterfaceContext *context = (CanInterfaceContext *)raw_context;

	if (context == NULL || node_id > 7U)
		return false;
	if (context->node_id == node_id)
		return true;
	if (context->transport_started &&
		!CanInterface_ApplyTransportConfiguration(context, node_id,
			context->configured_bitrate))
	{
		return false;
	}
	context->node_id = node_id;
	return true;
}

static uint8_t CanInterface_GetNodeId(void *raw_context)
{
	CanInterfaceContext *context = (CanInterfaceContext *)raw_context;
	return context != NULL ? context->node_id : 0U;
}

static bool CanInterface_SetBitrateKbps(void *raw_context,
	uint32_t bitrate_kbps)
{
	CanInterfaceContext *context = (CanInterfaceContext *)raw_context;

	if (context == NULL || !CanInterface_IsSupportedBitrate(bitrate_kbps) ||
		bitrate_kbps > CanInterface_MaximumBitrateKbps(context))
	{
		return false;
	}
	context->baudrate = bitrate_kbps;
	return true;
}

static uint32_t CanInterface_GetBitrateKbps(void *raw_context)
{
	CanInterfaceContext *context = (CanInterfaceContext *)raw_context;
	return context != NULL ? context->baudrate : 0U;
}

static bool CanInterface_SetHeartbeatMs(void *raw_context,
	uint32_t heartbeat_ms)
{
	CanInterfaceContext *context = (CanInterfaceContext *)raw_context;

	if (context == NULL || (heartbeat_ms != 0U &&
		(heartbeat_ms < context->minimum_heartbeat_ms ||
		 heartbeat_ms > context->maximum_heartbeat_ms)))
	{
		return false;
	}
	context->heartbeat_timeout_ms = heartbeat_ms;
	if (heartbeat_ms == 0U)
	{
		context->heartbeat_enabled = false;
		context->heartbeat_elapsed_ms = 0U;
		context->disconnect_reported = false;
	}
	return true;
}

static uint32_t CanInterface_GetHeartbeatMs(void *raw_context)
{
	CanInterfaceContext *context = (CanInterfaceContext *)raw_context;
	return context != NULL ? context->heartbeat_timeout_ms : 0U;
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

void CanInterface_ApplyConfiguredBitrate(CanInterfaceContext *context,
	CommunicationWatchdogServiceContext *watchdog)
{
	if (context == NULL ||
		!CanInterface_ApplyTransportConfiguration(context, context->node_id,
			context->baudrate))
	{
		(void)CommunicationWatchdogService_ReportDisconnected(watchdog);
		return;
	}
	context->configured_bitrate = context->baudrate;
}

bool CanInterface_Initialize(CanInterfaceContext *context,
	const BspCanPort *transport,
	ControlAuthorityServiceContext *control_authority,
	bool enable_fd, bool enable_brs, uint32_t default_nominal_bitrate_kbps,
	uint32_t default_data_bitrate_kbps,
	uint32_t minimum_heartbeat_ms, uint32_t maximum_heartbeat_ms)
{
	BspCommunicationFeatureSet required_features;
	uint32_t active_bitrate_kbps;

	if (context == NULL || transport == NULL || transport->capabilities == NULL ||
		transport->capabilities->kind != BSP_COMMUNICATION_CAN ||
		transport->configure == NULL || transport->start == NULL ||
		transport->stop == NULL || transport->try_receive == NULL ||
		transport->try_transmit == NULL || transport->read_faults == NULL ||
		control_authority == NULL || (enable_brs && !enable_fd) ||
		!CanInterface_IsSupportedBitrate(default_nominal_bitrate_kbps) ||
		(!enable_fd && default_data_bitrate_kbps != 0U) ||
		(enable_fd && (!CanInterface_IsSupportedBitrate(
			default_data_bitrate_kbps) ||
			(!enable_brs && default_data_bitrate_kbps !=
				default_nominal_bitrate_kbps))) ||
		minimum_heartbeat_ms > maximum_heartbeat_ms)
	{
		return false;
	}
	required_features = enable_fd ? BSP_COMMUNICATION_FEATURE_CAN_FD :
		BSP_COMMUNICATION_FEATURE_CAN_CLASSIC;
	if (enable_brs)
		required_features |= BSP_COMMUNICATION_FEATURE_CAN_BRS;
	if ((transport->capabilities->features & required_features) !=
		required_features)
	{
		return false;
	}
	memset(context, 0, sizeof(*context));
	context->transport = *transport;
	context->transport_is_initialized = true;
	context->control_authority = control_authority;
	context->enable_fd = enable_fd;
	context->enable_brs = enable_brs;
	context->nominal_bitrate_kbps = default_nominal_bitrate_kbps;
	active_bitrate_kbps = enable_fd ? default_data_bitrate_kbps :
		default_nominal_bitrate_kbps;
	context->baudrate = active_bitrate_kbps;
	context->configured_bitrate = active_bitrate_kbps;
	context->minimum_heartbeat_ms = minimum_heartbeat_ms;
	context->maximum_heartbeat_ms = maximum_heartbeat_ms;
	return default_nominal_bitrate_kbps <=
			transport->capabilities->maximum_nominal_bit_rate / 1000U &&
		active_bitrate_kbps <= CanInterface_MaximumBitrateKbps(context);
}

void CanInterface_UpdateWatchdog(CanInterfaceContext *context,
	const TelemetryServiceContext *telemetry,
	CommunicationWatchdogServiceContext *watchdog)
{
	BspCommunicationFaultSet transport_faults;
	float mode_value = 0.0f;
	int mode;

	if (context == NULL)
		return;
	transport_faults = context->transport.read_faults(
		context->transport.context);
	context->observed_transport_faults |= transport_faults;
	if (context->observed_transport_faults != 0U)
	{
		if (!context->disconnect_reported)
		{
			(void)CommunicationWatchdogService_ReportDisconnected(watchdog);
			context->disconnect_reported = true;
		}
		return;
	}
	(void)TelemetryService_ReadValue(telemetry, MOTOR_TELEMETRY_MODE,
		&mode_value);
	mode = (int)mode_value;
	context->heartbeat_enabled = context->heartbeat_timeout_ms != 0U &&
		ControlAuthorityService_IsOwner(context->control_authority,
			CONTROL_AUTHORITY_CAN) &&
		(mode == PROTOCOL_MODE_CURRENT || mode == PROTOCOL_MODE_SPEED ||
		 mode == PROTOCOL_MODE_POSITION ||
		 mode == PROTOCOL_MODE_POSITION_IMPEDANCE);
	if (context->received_once && context->heartbeat_enabled)
	{
		if (context->heartbeat_elapsed_ms < context->heartbeat_timeout_ms)
			context->heartbeat_elapsed_ms++;
		if (context->heartbeat_elapsed_ms >= context->heartbeat_timeout_ms &&
			!context->disconnect_reported)
		{
			(void)CommunicationWatchdogService_ReportDisconnected(watchdog);
			context->disconnect_reported = true;
		}
	}
}

void CanInterface_ApplyPendingBitrate(CanInterfaceContext *context,
	CommunicationWatchdogServiceContext *watchdog)
{
	if (context == NULL || context->configured_bitrate == context->baudrate)
		return;
	if (!CanInterface_ApplyTransportConfiguration(context, context->node_id,
		context->baudrate))
	{
		(void)CommunicationWatchdogService_ReportDisconnected(watchdog);
		return;
	}
	context->configured_bitrate = context->baudrate;
}

static bool CanInterface_QueueResponse(void *raw_context,
	uint8_t parameter_id, float data)
{
	CanInterfaceContext *context = (CanInterfaceContext *)raw_context;

	if (context == NULL || context->transmit_pending || !isfinite(data))
	{
		return false;
	}
	context->tx_parameter_id = (CanParameterId)parameter_id;
	context->tx_value = data;
	context->transmit_pending = true;
	return true;
}

CanResponsePort CanInterface_CreateResponsePort(CanInterfaceContext *context)
{
	CanResponsePort port;

	port.context = context;
	port.queue_response = CanInterface_QueueResponse;
	return port;
}

void CanInterface_ProcessReceivedFrames(CanInterfaceContext *context,
	CanCommandRouterContext *router,
	CommunicationWatchdogServiceContext *watchdog)
{
	BspCanFrame frame;
	CanProtocolV1Command decoded;
	BspResult result;

	if (context == NULL || router == NULL || !context->transport_started ||
		context->transmit_pending)
		return;
	result = context->transport.try_receive(context->transport.context, &frame);
	if (result == BSP_RESULT_NOT_READY || result == BSP_RESULT_BUSY)
		return;
	if (result != BSP_RESULT_OK ||
		!CanProtocolV1_Decode(&frame, context->node_id, &decoded))
	{
		return;
	}
	context->received_once = true;
	context->heartbeat_elapsed_ms = 0U;
	/* A valid frame recovers only heartbeat loss. Transport faults are sticky. */
	if (context->observed_transport_faults == 0U)
	{
		context->disconnect_reported = false;
		(void)CommunicationWatchdogService_ReportFrameReceived(watchdog);
	}
	CanCommandRouter_Handle(router, (CanParameterId)decoded.parameter_id,
		decoded.value);
}

void CanInterface_FlushTransmit(CanInterfaceContext *context)
{
	BspCanFrame frame;
	BspResult result;

	if (context == NULL || !context->transmit_pending ||
		!context->transport_started ||
		!CanProtocolV1_Encode(context->node_id,
			(uint8_t)context->tx_parameter_id, context->tx_value, &frame))
	{
		return;
	}
	if (context->enable_fd)
		frame.flags |= BSP_CAN_FRAME_FD;
	if (context->enable_brs)
		frame.flags |= BSP_CAN_FRAME_BRS;
	result = context->transport.try_transmit(context->transport.context, &frame);
	if (result == BSP_RESULT_OK)
		context->transmit_pending = false;
}

BspCommunicationFaultSet CanInterface_GetObservedTransportFaults(
	const CanInterfaceContext *context)
{
	return context != NULL ? context->observed_transport_faults :
		BSP_COMMUNICATION_FAULT_IO;
}
