#include "Core/Communication/Interfaces/interface_can.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#if defined(__CC_ARM)
#pragma O3
#pragma Ospace
#endif

#define CAN_INTERFACE_RX_ISR_DRAIN_LIMIT 3U

static void CanInterface_LatchIoFault(CanInterfaceContext *context)
{
	BspCriticalSectionToken token;

	token = context->critical_section.enter(context->critical_section.context);
	context->rx_pipeline_faults |= BSP_COMMUNICATION_FAULT_IO;
	context->critical_section.exit(context->critical_section.context, token);
}

static void CanInterface_ResetReceivePipeline(CanInterfaceContext *context,
	bool begin_generation)
{
	BspCriticalSectionToken token;

	token = context->critical_section.enter(context->critical_section.context);
	if (begin_generation)
	{
		context->configuration_generation++;
		context->supervised_rx_generation = context->rx_generation;
	}
	context->decoded_read_sequence = context->decoded_write_sequence;
	context->critical_section.exit(context->critical_section.context, token);
}

static bool CanInterface_HasStickyFault(const CanInterfaceContext *context)
{
	return context->observed_transport_faults != 0U ||
		context->rx_pipeline_faults != 0U;
}

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
	uint32_t maximum_bit_rate = context->enable_fd && context->enable_brs ?
		context->transport.capabilities->maximum_data_bit_rate :
		context->transport.capabilities->maximum_nominal_bit_rate;
	return maximum_bit_rate / 1000U;
}

static void CanInterface_BuildTransportConfiguration(
	const CanInterfaceContext *context, uint8_t node_id,
	uint32_t bitrate_kbps, BspCanAcceptanceFilter *filter,
	BspCanConfiguration *configuration)
{
	memset(configuration, 0, sizeof(*configuration));
	filter->filter_index = 0U;
	filter->identifier_kind = BSP_CAN_IDENTIFIER_STANDARD;
	filter->filter_kind = BSP_CAN_FILTER_RANGE;
	filter->identifier_a = (uint32_t)node_id << 8;
	filter->identifier_b = filter->identifier_a + 0xFFU;
	configuration->enable_fd = context->enable_fd;
	configuration->enable_brs = context->enable_brs;
	configuration->acceptance_filters = filter;
	configuration->acceptance_filter_count = 1U;
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
}

static bool CanInterface_ConfigureAndStart(CanInterfaceContext *context,
	uint8_t node_id, uint32_t bitrate_kbps)
{
	BspCanAcceptanceFilter filter;
	BspCanConfiguration configuration;

	CanInterface_BuildTransportConfiguration(context, node_id,
		bitrate_kbps, &filter, &configuration);
	if (context->transport.configure(context->transport.context,
		&configuration) != BSP_RESULT_OK)
	{
		CanInterface_LatchIoFault(context);
		return false;
	}
	/* Publish the decoder identity before enabling RX interrupts. */
	context->node_id = node_id;
	context->transport_started = true;
	if (context->transport.start(context->transport.context) != BSP_RESULT_OK)
	{
		context->transport_started = false;
		CanInterface_LatchIoFault(context);
		return false;
	}
	return true;
}

static bool CanInterface_ApplyTransportConfiguration(
	CanInterfaceContext *context, uint8_t node_id, uint32_t bitrate_kbps)
{
	bool was_started;
	uint8_t previous_node_id;

	previous_node_id = context->node_id;
	was_started = context->transport_started;
	/* Gate the ISR before the BSP disables its interrupt source. */
	context->transport_started = false;
	if (was_started &&
		context->transport.stop(context->transport.context) != BSP_RESULT_OK)
	{
		context->transport_started = true;
		CanInterface_LatchIoFault(context);
		CanInterface_ResetReceivePipeline(context, false);
		return false;
	}
	/* Commands decoded under an old node/rate must never cross this boundary. */
	CanInterface_ResetReceivePipeline(context, true);
	if (CanInterface_ConfigureAndStart(context, node_id, bitrate_kbps))
	{
		context->heartbeat_reference_ms =
			context->clock.read_ms(context->clock.context);
		return true;
	}

	/* A failed live reconfiguration must not silently change the active
	 * protocol identity. Best-effort rollback restores the last applied pair. */
	if (was_started && CanInterface_ConfigureAndStart(context,
		previous_node_id, context->configured_bitrate))
		return false;
	context->node_id = previous_node_id;
	context->transport_started = false;
	CanInterface_LatchIoFault(context);
	return false;
}

static bool CanInterface_SetNodeId(void *raw_context, uint8_t node_id)
{
	CanInterfaceContext *context = (CanInterfaceContext *)raw_context;

	if (context == NULL || node_id > 7U)
		return false;
	/* The request is committed only by CanInterface_RunBackground(). */
	context->requested_node_id = node_id;
	return true;
}

static uint8_t CanInterface_GetNodeId(void *raw_context)
{
	CanInterfaceContext *context = (CanInterfaceContext *)raw_context;
	return context != NULL ? context->requested_node_id : 0U;
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
	uint32_t target_bitrate;
	uint8_t target_node_id;

	if (context == NULL)
		return;
	target_bitrate = context->baudrate;
	target_node_id = context->requested_node_id;
	if (context->transmit_pending || CanInterface_HasStickyFault(context))
		return;
	if (!CanInterface_ApplyTransportConfiguration(context, target_node_id,
		target_bitrate))
	{
		context->baudrate = context->configured_bitrate;
		context->requested_node_id = context->node_id;
		context->disconnect_reported = true;
		(void)CommunicationWatchdogService_ReportDisconnected(watchdog);
		return;
	}
	context->configured_bitrate = target_bitrate;
	context->requested_node_id = target_node_id;
}

bool CanInterface_Initialize(CanInterfaceContext *context,
	const BspCanPort *transport,
	const BspMonotonicClockPort *clock,
	const BspCriticalSectionPort *critical_section,
	ControlAuthorityServiceContext *control_authority,
	bool enable_fd, bool enable_brs, uint32_t default_nominal_bitrate_kbps,
	uint32_t default_data_bitrate_kbps,
	uint32_t minimum_heartbeat_ms, uint32_t maximum_heartbeat_ms)
{
	BspCommunicationFeatureSet required_features;
	uint32_t active_bitrate_kbps;

	if (context == NULL || transport == NULL || transport->capabilities == NULL ||
		transport->capabilities->availability != BSP_ENDPOINT_AVAILABLE ||
		transport->capabilities->kind != BSP_COMMUNICATION_CAN ||
		transport->capabilities->maximum_payload_bytes < 4U ||
		transport->configure == NULL || transport->start == NULL ||
		transport->stop == NULL || transport->try_receive == NULL ||
		transport->try_transmit == NULL || transport->read_faults == NULL ||
		clock == NULL || clock->read_ms == NULL || critical_section == NULL ||
		critical_section->enter == NULL || critical_section->exit == NULL ||
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
		required_features || default_nominal_bitrate_kbps >
		transport->capabilities->maximum_nominal_bit_rate / 1000U ||
		(enable_fd && default_data_bitrate_kbps >
		 transport->capabilities->maximum_data_bit_rate / 1000U))
	{
		return false;
	}
	memset(context, 0, sizeof(*context));
	context->transport = *transport;
	context->clock = *clock;
	context->critical_section = *critical_section;
	context->control_authority = control_authority;
	context->enable_fd = enable_fd;
	context->enable_brs = enable_brs;
	context->requested_node_id = 0U;
	context->nominal_bitrate_kbps = default_nominal_bitrate_kbps;
	active_bitrate_kbps = enable_fd ? default_data_bitrate_kbps :
		default_nominal_bitrate_kbps;
	context->baudrate = active_bitrate_kbps;
	context->configured_bitrate = active_bitrate_kbps;
	context->minimum_heartbeat_ms = minimum_heartbeat_ms;
	context->maximum_heartbeat_ms = maximum_heartbeat_ms;
	return true;
}

void CanInterface_OnReceiveInterrupt(CanInterfaceContext *context)
{
	uint32_t arrival_ms;
	uint8_t drained;

	if (context == NULL || !context->transport_started)
	{
		return;
	}
	arrival_ms = context->clock.read_ms(context->clock.context);
	for (drained = 0U; drained < CAN_INTERFACE_RX_ISR_DRAIN_LIMIT; ++drained)
	{
		BspCanFrame frame;
		CanProtocolV1Command decoded;
		BspResult result = context->transport.try_receive(
			context->transport.context, &frame);

		if (result == BSP_RESULT_NOT_READY || result == BSP_RESULT_BUSY)
			break;
		if (result != BSP_RESULT_OK)
		{
			CanInterface_LatchIoFault(context);
			break;
		}
		if (!CanProtocolV1_Decode(&frame, context->node_id, &decoded))
			continue;

		{
			BspCriticalSectionToken token = context->critical_section.enter(
				context->critical_section.context);
			CanInterfaceDecodedCommand *queued;

			/* Publish arrival independently of background Router progress. */
			context->last_valid_rx_ms = arrival_ms;
			context->received_once = true;
			context->rx_generation++;
			if ((uint8_t)(context->decoded_write_sequence -
				context->decoded_read_sequence) >=
				CAN_INTERFACE_DECODED_QUEUE_COUNT)
			{
				context->rx_pipeline_faults |=
					BSP_COMMUNICATION_FAULT_RX_OVERFLOW;
				context->critical_section.exit(
					context->critical_section.context, token);
				continue;
			}
			queued = &context->decoded_queue[context->decoded_write_sequence &
				(CAN_INTERFACE_DECODED_QUEUE_COUNT - 1U)];
			queued->command = decoded;
			queued->configuration_generation =
				context->configuration_generation;
			/* Publish last: the consumer cannot see a partial command. */
			context->decoded_write_sequence++;
			context->critical_section.exit(
				context->critical_section.context, token);
		}
	}
}

void CanInterface_Supervise1kHz(CanInterfaceContext *context,
	bool can_motion_mode_active,
	CommunicationWatchdogServiceContext *watchdog)
{
	BspCommunicationFaultSet transport_faults;
	BspCommunicationFaultSet pipeline_faults;
	BspCriticalSectionToken token;
	uint32_t last_valid_rx_ms;
	uint32_t rx_generation;
	uint32_t now_ms;
	uint32_t timeout_ms;
	bool received_once;
	bool new_frame;
	bool enable_heartbeat;

	if (context == NULL || watchdog == NULL)
		return;
	/* Fault snapshots remain valid while stopped and expose failed rollback. */
	transport_faults = context->transport.read_faults(
		context->transport.context);
	token = context->critical_section.enter(
		context->critical_section.context);
	pipeline_faults = context->rx_pipeline_faults;
	last_valid_rx_ms = context->last_valid_rx_ms;
	rx_generation = context->rx_generation;
	received_once = context->received_once;
	now_ms = context->clock.read_ms(context->clock.context);
	context->critical_section.exit(context->critical_section.context, token);
	context->observed_transport_faults |=
		transport_faults | pipeline_faults;
	if (context->observed_transport_faults != 0U)
	{
		/* A sticky source is level-triggered: a generic clear-fault command must
		 * never make an active transport failure disappear. */
		(void)CommunicationWatchdogService_ReportDisconnected(watchdog);
		context->disconnect_reported = true;
		return;
	}
	new_frame = rx_generation != context->supervised_rx_generation;
	if (new_frame)
	{
		context->heartbeat_reference_ms = last_valid_rx_ms;
		context->supervised_rx_generation = rx_generation;
		context->disconnect_reported = false;
		(void)CommunicationWatchdogService_ReportFrameReceived(watchdog);
	}

	timeout_ms = context->heartbeat_timeout_ms;
	enable_heartbeat = timeout_ms != 0U && can_motion_mode_active &&
		ControlAuthorityService_IsOwner(context->control_authority,
			CONTROL_AUTHORITY_CAN);
	if (!enable_heartbeat)
	{
		context->heartbeat_enabled = false;
		return;
	}
	if (!context->heartbeat_enabled)
	{
		/* Enabling supervision starts a fresh deadline, as the old counter did. */
		context->heartbeat_enabled = true;
		context->heartbeat_reference_ms = new_frame ?
			last_valid_rx_ms : now_ms;
	}
	else if (received_once &&
		(uint32_t)(now_ms - context->heartbeat_reference_ms) >= timeout_ms &&
		!new_frame)
	{
		uint32_t confirmed_generation;
		uint32_t confirmed_timestamp;
		BspCommunicationFaultSet confirmed_pipeline_faults;
		bool arrival_after_snapshot;

		/* FDCAN has higher priority than TIM7. Recheck after the first snapshot
		 * so an arrival in that preemption window cannot cause a false trip. */
		token = context->critical_section.enter(
			context->critical_section.context);
		confirmed_generation = context->rx_generation;
		confirmed_timestamp = context->last_valid_rx_ms;
		confirmed_pipeline_faults = context->rx_pipeline_faults;
		arrival_after_snapshot = confirmed_generation != rx_generation;
		context->critical_section.exit(context->critical_section.context, token);

		context->observed_transport_faults |= confirmed_pipeline_faults |
			context->transport.read_faults(context->transport.context);
		if (context->observed_transport_faults != 0U)
		{
			(void)CommunicationWatchdogService_ReportDisconnected(watchdog);
			context->disconnect_reported = true;
		}
		else if (arrival_after_snapshot)
		{
			context->heartbeat_reference_ms = confirmed_timestamp;
			context->supervised_rx_generation = confirmed_generation;
			context->disconnect_reported = false;
			(void)CommunicationWatchdogService_ReportFrameReceived(watchdog);
		}
		else
		{
			/* Timeout is also level-triggered until a valid frame arrives. */
			(void)CommunicationWatchdogService_ReportDisconnected(watchdog);
			context->disconnect_reported = true;
		}
	}
}

static bool CanInterface_QueueResponse(void *raw_context,
	uint8_t parameter_id, float data)
{
	CanInterfaceContext *context = (CanInterfaceContext *)raw_context;

	if (context == NULL || context->transmit_pending || !isfinite(data) ||
		!CanProtocolV1_Encode(context->node_id, parameter_id, data,
			&context->transmit_frame))
	{
		return false;
	}
	if (context->enable_fd)
		context->transmit_frame.flags |= BSP_CAN_FRAME_FD;
	if (context->enable_brs)
		context->transmit_frame.flags |= BSP_CAN_FRAME_BRS;
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

static bool CanInterface_TryPopDecodedCommand(CanInterfaceContext *context,
	CanProtocolV1Command *decoded)
{
	CanInterfaceDecodedCommand queued;
	BspCriticalSectionToken token;
	bool available;

	token = context->critical_section.enter(context->critical_section.context);
	available = context->decoded_read_sequence !=
		context->decoded_write_sequence;
	if (available)
	{
		queued = context->decoded_queue[context->decoded_read_sequence &
			(CAN_INTERFACE_DECODED_QUEUE_COUNT - 1U)];
		context->decoded_read_sequence++;
	}
	context->critical_section.exit(context->critical_section.context, token);
	if (!available || queued.configuration_generation !=
		context->configuration_generation)
	{
		return false;
	}
	*decoded = queued.command;
	return true;
}

static void CanInterface_FlushTransmit(CanInterfaceContext *context)
{
	BspResult result;

	if (!context->transmit_pending || !context->transport_started)
	{
		return;
	}
	result = context->transport.try_transmit(context->transport.context,
		&context->transmit_frame);
	if (result == BSP_RESULT_OK)
		context->transmit_pending = false;
	else if (result != BSP_RESULT_BUSY && result != BSP_RESULT_NOT_READY)
		CanInterface_LatchIoFault(context);
}

static void CanInterface_AcknowledgeReceiveBeforeRouting(
	CanInterfaceContext *context,
	CommunicationWatchdogServiceContext *watchdog)
{
	BspCriticalSectionToken token;
	BspCommunicationFaultSet pipeline_faults;
	uint32_t last_valid_rx_ms;
	uint32_t rx_generation;

	/* A command response must observe recovery caused by that same valid frame.
	 * The 1 kHz supervisor remains the timeout owner, while this background
	 * boundary only acknowledges ISR-published arrivals before Router execution. */
	token = context->critical_section.enter(context->critical_section.context);
	pipeline_faults = context->rx_pipeline_faults;
	last_valid_rx_ms = context->last_valid_rx_ms;
	rx_generation = context->rx_generation;
	context->critical_section.exit(context->critical_section.context, token);
	if (rx_generation == context->supervised_rx_generation)
		return;
	context->observed_transport_faults |= pipeline_faults |
		context->transport.read_faults(context->transport.context);
	if (context->observed_transport_faults != 0U)
		return;
	context->heartbeat_reference_ms = last_valid_rx_ms;
	context->supervised_rx_generation = rx_generation;
	context->disconnect_reported = false;
	(void)CommunicationWatchdogService_ReportFrameReceived(watchdog);
}

void CanInterface_RunBackground(CanInterfaceContext *context,
	CanCommandRouterContext *router,
	CommunicationWatchdogServiceContext *watchdog)
{
	CanProtocolV1Command decoded;

	if (context == NULL || router == NULL || watchdog == NULL)
		return;
	CanInterface_AcknowledgeReceiveBeforeRouting(context, watchdog);
	if (CanInterface_HasStickyFault(context))
	{
		CanInterface_ResetReceivePipeline(context, false);
		return;
	}
	/* Stop/configure/start is owned exclusively by this background path and
	 * waits until the complete response accepted before the request is applied. */
	if (context->requested_node_id != context->node_id ||
		context->configured_bitrate != context->baudrate)
	{
		if (context->transmit_pending)
			CanInterface_FlushTransmit(context);
		else
			CanInterface_ApplyConfiguredBitrate(context, watchdog);
		return;
	}
	if (context->transmit_pending)
	{
		CanInterface_FlushTransmit(context);
		return;
	}
	if (CanInterface_TryPopDecodedCommand(context, &decoded))
	{
		CanCommandRouter_Handle(router, (CanParameterId)decoded.parameter_id,
			decoded.value);
	}
	CanInterface_FlushTransmit(context);
	/* A command may have staged a configuration request; it is deliberately
	 * committed on the next background turn, after this routing call returns. */
}

BspCommunicationFaultSet CanInterface_GetObservedTransportFaults(
	const CanInterfaceContext *context)
{
	return context != NULL ? context->observed_transport_faults :
		BSP_COMMUNICATION_FAULT_IO;
}
