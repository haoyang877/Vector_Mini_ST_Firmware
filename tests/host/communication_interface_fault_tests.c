#include "Core/Communication/Interfaces/interface_can.h"

#include <string.h>

#define FAKE_CAN_QUEUE_COUNT 8U

typedef struct
{
	BspCommunicationFaultSet faults;
	BspCanFrame frames[FAKE_CAN_QUEUE_COUNT];
	uint8_t read_sequence;
	uint8_t write_sequence;
	BspResult transmit_result;
	uint32_t configure_calls;
	uint32_t start_calls;
	uint32_t stop_calls;
	uint32_t transmit_calls;
	uint32_t configure_failures_remaining;
	uint32_t start_failures_remaining;
	uint32_t stop_failures_remaining;
	uint8_t configured_node_id;
	BspCanFrame last_transmitted_frame;
	bool started;
} FakeCanTransport;

typedef struct
{
	uint32_t raised;
	uint32_t cleared;
} FakeFaultSink;

typedef struct
{
	uint32_t now_ms;
} FakeClock;

typedef struct
{
	uint32_t enter_calls;
	uint32_t exit_calls;
	uint32_t depth;
	void (*exit_hook)(void *context);
	void *exit_hook_context;
} FakeCriticalSection;

typedef struct
{
	CanInterfaceContext *interface_context;
	FakeCanTransport *transport;
	uint8_t node_id;
	CanParameterId parameter;
	float value;
	bool pushed;
} FakeReceiveInjection;

static const BspCommunicationEndpointCapabilities FakeCanCapabilities = {
	1U,
	BSP_ENDPOINT_AVAILABLE,
	BSP_COMMUNICATION_CAN,
	BSP_COMMUNICATION_FEATURE_CAN_CLASSIC |
		BSP_COMMUNICATION_FEATURE_CAN_FD |
		BSP_COMMUNICATION_FEATURE_CAN_BRS,
	8U,
	1000000UL,
	5000000UL
};

static uint32_t RoutedFrameCount;
static CanParameterId RoutedParameters[32];

#define CHECK(expression) do { if (!(expression)) return __LINE__; } while (0)

static BspResult FakeCan_Configure(void *raw_context,
	const BspCanConfiguration *configuration)
{
	FakeCanTransport *transport = (FakeCanTransport *)raw_context;

	transport->configure_calls++;
	if (transport->configure_failures_remaining != 0U)
	{
		transport->configure_failures_remaining--;
		return BSP_RESULT_IO_ERROR;
	}
	if (configuration == NULL ||
		configuration->acceptance_filter_count != 1U ||
		configuration->acceptance_filters == NULL)
	{
		return BSP_RESULT_INVALID_ARGUMENT;
	}
	transport->configured_node_id = (uint8_t)
		(configuration->acceptance_filters[0].identifier_a >> 8);
	return BSP_RESULT_OK;
}

static BspResult FakeCan_Start(void *raw_context)
{
	FakeCanTransport *transport = (FakeCanTransport *)raw_context;
	transport->start_calls++;
	if (transport->start_failures_remaining != 0U)
	{
		transport->start_failures_remaining--;
		return BSP_RESULT_IO_ERROR;
	}
	transport->started = true;
	return BSP_RESULT_OK;
}

static BspResult FakeCan_Stop(void *raw_context)
{
	FakeCanTransport *transport = (FakeCanTransport *)raw_context;
	transport->stop_calls++;
	if (transport->stop_failures_remaining != 0U)
	{
		transport->stop_failures_remaining--;
		return BSP_RESULT_IO_ERROR;
	}
	transport->started = false;
	return BSP_RESULT_OK;
}

static BspCommunicationFaultSet FakeCan_ReadFaults(void *raw_context)
{
	return ((FakeCanTransport *)raw_context)->faults;
}

static BspResult FakeCan_TryReceive(void *raw_context, BspCanFrame *frame)
{
	FakeCanTransport *transport = (FakeCanTransport *)raw_context;

	if (!transport->started || transport->read_sequence ==
		transport->write_sequence)
	{
		return BSP_RESULT_NOT_READY;
	}
	*frame = transport->frames[transport->read_sequence &
		(FAKE_CAN_QUEUE_COUNT - 1U)];
	transport->read_sequence++;
	return BSP_RESULT_OK;
}

static BspResult FakeCan_TryTransmit(void *raw_context,
	const BspCanFrame *frame)
{
	FakeCanTransport *transport = (FakeCanTransport *)raw_context;

	if (frame == NULL)
		return BSP_RESULT_INVALID_ARGUMENT;
	transport->transmit_calls++;
	transport->last_transmitted_frame = *frame;
	return transport->transmit_result;
}

static bool FakeCan_PushFrame(FakeCanTransport *transport,
	const BspCanFrame *frame)
{
	if (transport == NULL || frame == NULL ||
		(uint8_t)(transport->write_sequence - transport->read_sequence) >=
		FAKE_CAN_QUEUE_COUNT)
	{
		return false;
	}
	transport->frames[transport->write_sequence &
		(FAKE_CAN_QUEUE_COUNT - 1U)] = *frame;
	transport->write_sequence++;
	return true;
}

static bool FakeCan_PushEncoded(FakeCanTransport *transport, uint8_t node_id,
	CanParameterId parameter, float value)
{
	BspCanFrame frame;
	return CanProtocolV1_Encode(node_id, (uint8_t)parameter, value, &frame) &&
		FakeCan_PushFrame(transport, &frame);
}

static uint32_t FakeClock_ReadMs(void *raw_context)
{
	return ((FakeClock *)raw_context)->now_ms;
}

static BspCriticalSectionToken FakeCritical_Enter(void *raw_context)
{
	FakeCriticalSection *critical = (FakeCriticalSection *)raw_context;
	BspCriticalSectionToken previous = critical->depth;
	critical->enter_calls++;
	critical->depth++;
	return previous;
}

static void FakeCritical_Exit(void *raw_context,
	BspCriticalSectionToken token)
{
	FakeCriticalSection *critical = (FakeCriticalSection *)raw_context;
	void (*hook)(void *context);
	void *hook_context;

	critical->exit_calls++;
	critical->depth = (uint32_t)token;
	hook = critical->exit_hook;
	hook_context = critical->exit_hook_context;
	critical->exit_hook = NULL;
	critical->exit_hook_context = NULL;
	if (hook != NULL)
		hook(hook_context);
}

static void FakeReceive_Inject(void *raw_context)
{
	FakeReceiveInjection *injection = (FakeReceiveInjection *)raw_context;

	injection->pushed = FakeCan_PushEncoded(injection->transport,
		injection->node_id, injection->parameter, injection->value);
	if (injection->pushed)
		CanInterface_OnReceiveInterrupt(injection->interface_context);
}

static bool FakeFault_Raise(void *raw_context, uint32_t fault_code)
{
	FakeFaultSink *sink = (FakeFaultSink *)raw_context;
	(void)fault_code;
	sink->raised++;
	return true;
}

static bool FakeFault_Clear(void *raw_context, uint32_t fault_code)
{
	FakeFaultSink *sink = (FakeFaultSink *)raw_context;
	(void)fault_code;
	sink->cleared++;
	return true;
}

void CanCommandRouter_Handle(CanCommandRouterContext *context,
	CanParameterId parameter_id, float value)
{
	(void)context;
	(void)value;
	if (RoutedFrameCount <
		(uint32_t)(sizeof(RoutedParameters) / sizeof(RoutedParameters[0])))
	{
		RoutedParameters[RoutedFrameCount] = parameter_id;
	}
	RoutedFrameCount++;
}

static bool FakeCan_SetupInterface(CanInterfaceContext *interface_context,
	FakeCanTransport *transport, FakeFaultSink *fault_sink,
	FakeClock *clock, FakeCriticalSection *critical,
	ControlAuthorityServiceContext *authority,
	CommunicationWatchdogServiceContext *watchdog, BspCanPort *can_port,
	BspMonotonicClockPort *clock_port,
	BspCriticalSectionPort *critical_port,
	CanConfigurationPort *configuration_port, bool start_transport)
{
	FaultCommandPort fault_port;

	(void)memset(interface_context, 0, sizeof(*interface_context));
	(void)memset(transport, 0, sizeof(*transport));
	(void)memset(fault_sink, 0, sizeof(*fault_sink));
	(void)memset(clock, 0, sizeof(*clock));
	(void)memset(critical, 0, sizeof(*critical));
	transport->transmit_result = BSP_RESULT_OK;
	can_port->context = transport;
	can_port->capabilities = &FakeCanCapabilities;
	can_port->configure = FakeCan_Configure;
	can_port->start = FakeCan_Start;
	can_port->stop = FakeCan_Stop;
	can_port->try_receive = FakeCan_TryReceive;
	can_port->try_transmit = FakeCan_TryTransmit;
	can_port->read_faults = FakeCan_ReadFaults;
	clock_port->context = clock;
	clock_port->read_ms = FakeClock_ReadMs;
	critical_port->context = critical;
	critical_port->enter = FakeCritical_Enter;
	critical_port->exit = FakeCritical_Exit;
	fault_port.context = fault_sink;
	fault_port.raise = FakeFault_Raise;
	fault_port.clear = FakeFault_Clear;
	if (!ControlAuthorityService_Initialize(authority) ||
		!CommunicationWatchdogService_Initialize(watchdog, &fault_port, 7U) ||
		!CanInterface_Initialize(interface_context, can_port, clock_port,
			critical_port, authority, false, false, 1000U, 0U, 500U, 1000U))
	{
		return false;
	}
	*configuration_port = CanInterface_CreateConfigurationPort(
		interface_context);
	if (!configuration_port->set_node_id(configuration_port->context, 3U))
		return false;
	if (start_transport)
	{
		CanInterface_ApplyConfiguredBitrate(interface_context, watchdog);
		if (!interface_context->transport_started || !transport->started)
			return false;
	}
	return true;
}

int CommunicationInterfaceFault_RunHostTests(void)
{
	CanInterfaceContext interface_context;
	CanInterfaceContext invalid_interface;
	CanCommandRouterContext router;
	CommunicationWatchdogServiceContext watchdog;
	ControlAuthorityServiceContext authority;
	FakeCanTransport transport;
	FakeFaultSink fault_sink;
	FakeClock fake_clock;
	FakeCriticalSection fake_critical;
	BspCanPort can_port;
	BspMonotonicClockPort clock_port;
	BspCriticalSectionPort critical_port;
	CanConfigurationPort configuration_port;
	CanResponsePort response_port;
	FakeReceiveInjection injection;
	BspCanPort invalid_port;
	BspCommunicationEndpointCapabilities invalid_capabilities;
	BspCanFrame invalid_frame;
	uint32_t valid_timestamp;
	uint32_t routed_before;
	uint32_t configure_before;
	uint32_t clear_before;
	uint32_t raised_before;
	uint8_t generation_before;

	(void)memset(&router, 0, sizeof(router));
	CHECK(FakeCan_SetupInterface(&interface_context, &transport, &fault_sink,
		&fake_clock, &fake_critical, &authority, &watchdog, &can_port,
		&clock_port, &critical_port, &configuration_port, true));
	CHECK(interface_context.transport_started);
	CHECK(transport.configure_calls == 1U && transport.start_calls == 1U);
	CHECK(transport.configured_node_id == 3U);
	CHECK(configuration_port.set_heartbeat_ms(configuration_port.context, 500U));
	ControlAuthorityService_Claim(&authority, CONTROL_AUTHORITY_CAN);

	/* A valid ISR arrival refreshes heartbeat before background Router work. */
	fake_clock.now_ms = 10U;
	CHECK(FakeCan_PushEncoded(&transport, 3U, CAN_SET_CURRENT, 1.25f));
	RoutedFrameCount = 0U;
	CanInterface_OnReceiveInterrupt(&interface_context);
	CHECK(RoutedFrameCount == 0U);
	CHECK(interface_context.rx_generation == 1U);
	CanInterface_Supervise1kHz(&interface_context, true, &watchdog);
	CHECK(fault_sink.cleared == 1U);
	fake_clock.now_ms = 509U;
	CanInterface_Supervise1kHz(&interface_context, true, &watchdog);
	CHECK(fault_sink.raised == 0U);
	fake_clock.now_ms = 510U;
	CanInterface_Supervise1kHz(&interface_context, true, &watchdog);
	CHECK(fault_sink.raised == 1U && interface_context.disconnect_reported);
	CHECK(RoutedFrameCount == 0U);
	CanInterface_RunBackground(&interface_context, &router, &watchdog);
	CHECK(RoutedFrameCount == 1U && RoutedParameters[0] == CAN_SET_CURRENT);

	/* A later valid frame recovers heartbeat loss when no sticky fault exists. */
	fake_clock.now_ms = 511U;
	CHECK(FakeCan_PushEncoded(&transport, 3U, CAN_SET_SPEED, 2.0f));
	CanInterface_OnReceiveInterrupt(&interface_context);
	CanInterface_Supervise1kHz(&interface_context, true, &watchdog);
	CHECK(fault_sink.cleared == 2U && !interface_context.disconnect_reported);
	CanInterface_RunBackground(&interface_context, &router, &watchdog);
	CHECK(RoutedFrameCount == 2U && RoutedParameters[1] == CAN_SET_SPEED);

	/* Wrong node, bad DLC and NaN are not heartbeat-valid or routed. */
	valid_timestamp = interface_context.last_valid_rx_ms;
	CHECK(FakeCan_PushEncoded(&transport, 2U, CAN_SET_CURRENT, 3.0f));
	CHECK(CanProtocolV1_Encode(3U, CAN_SET_CURRENT, 3.0f, &invalid_frame));
	invalid_frame.length = 3U;
	CHECK(FakeCan_PushFrame(&transport, &invalid_frame));
	CHECK(CanProtocolV1_Encode(3U, CAN_SET_CURRENT, 3.0f, &invalid_frame));
	invalid_frame.data[0] = 0x7FU;
	invalid_frame.data[1] = 0xC0U;
	invalid_frame.data[2] = 0x00U;
	invalid_frame.data[3] = 0x00U;
	CHECK(FakeCan_PushFrame(&transport, &invalid_frame));
	CanInterface_OnReceiveInterrupt(&interface_context);
	CHECK(interface_context.last_valid_rx_ms == valid_timestamp);
	CHECK(RoutedFrameCount == 2U);
	fake_clock.now_ms = 1011U;
	CanInterface_Supervise1kHz(&interface_context, true, &watchdog);
	CHECK(fault_sink.raised == 2U);

	/* The arrival deadline remains correct across the uint32_t ms wrap. */
	fake_clock.now_ms = UINT32_MAX - 200U;
	CHECK(FakeCan_PushEncoded(&transport, 3U, CAN_SET_CURRENT, 7.0f));
	CanInterface_OnReceiveInterrupt(&interface_context);
	CanInterface_Supervise1kHz(&interface_context, true, &watchdog);
	CHECK(fault_sink.cleared == 3U && !interface_context.disconnect_reported);
	fake_clock.now_ms = 298U;
	CanInterface_Supervise1kHz(&interface_context, true, &watchdog);
	CHECK(fault_sink.raised == 2U);
	fake_clock.now_ms = 299U;
	CanInterface_Supervise1kHz(&interface_context, true, &watchdog);
	CHECK(fault_sink.raised == 3U && interface_context.disconnect_reported);
	CanInterface_RunBackground(&interface_context, &router, &watchdog);

	/* A fifth queued command faults sticky; no queued mutation may run after it. */
	fake_clock.now_ms = 1100U;
	CHECK(FakeCan_PushEncoded(&transport, 3U, CAN_SET_MODE, 1.0f));
	CanInterface_OnReceiveInterrupt(&interface_context);
	CanInterface_Supervise1kHz(&interface_context, true, &watchdog);
	CanInterface_RunBackground(&interface_context, &router, &watchdog);
	CHECK(FakeCan_PushEncoded(&transport, 3U, CAN_SET_CURRENT, 1.0f));
	CHECK(FakeCan_PushEncoded(&transport, 3U, CAN_SET_SPEED, 2.0f));
	CHECK(FakeCan_PushEncoded(&transport, 3U, CAN_SET_POS, 3.0f));
	CHECK(FakeCan_PushEncoded(&transport, 3U, CAN_SET_MODE, 4.0f));
	CanInterface_OnReceiveInterrupt(&interface_context);
	CanInterface_OnReceiveInterrupt(&interface_context);
	routed_before = RoutedFrameCount;
	CHECK(FakeCan_PushEncoded(&transport, 3U, CAN_SET_CURRENT, 5.0f));
	CanInterface_OnReceiveInterrupt(&interface_context);
	CHECK((interface_context.rx_pipeline_faults &
		BSP_COMMUNICATION_FAULT_RX_OVERFLOW) != 0U);
	CanInterface_Supervise1kHz(&interface_context, true, &watchdog);
	CHECK((CanInterface_GetObservedTransportFaults(&interface_context) &
		BSP_COMMUNICATION_FAULT_RX_OVERFLOW) != 0U);
	raised_before = fault_sink.raised;
	CHECK(raised_before == 4U);
	CanInterface_RunBackground(&interface_context, &router, &watchdog);
	CanInterface_RunBackground(&interface_context, &router, &watchdog);
	CanInterface_RunBackground(&interface_context, &router, &watchdog);
	CanInterface_RunBackground(&interface_context, &router, &watchdog);
	CHECK(RoutedFrameCount == routed_before);
	CanInterface_Supervise1kHz(&interface_context, true, &watchdog);
	CHECK(fault_sink.raised == raised_before + 1U);
	clear_before = fault_sink.cleared;
	CHECK(FakeCan_PushEncoded(&transport, 3U, CAN_SET_SPEED, 6.0f));
	CanInterface_OnReceiveInterrupt(&interface_context);
	CanInterface_Supervise1kHz(&interface_context, true, &watchdog);
	CHECK(fault_sink.cleared == clear_before);
	CHECK(fault_sink.raised == raised_before + 2U);
	CanInterface_RunBackground(&interface_context, &router, &watchdog);
	CHECK(RoutedFrameCount == routed_before);
	transport.faults |= BSP_COMMUNICATION_FAULT_BUS_OFF;
	CanInterface_Supervise1kHz(&interface_context, true, &watchdog);
	CHECK((CanInterface_GetObservedTransportFaults(&interface_context) &
		BSP_COMMUNICATION_FAULT_BUS_OFF) != 0U);

	/* Reconfiguration waits for a frozen response and runs only in background. */
	CHECK(FakeCan_SetupInterface(&interface_context, &transport, &fault_sink,
		&fake_clock, &fake_critical, &authority, &watchdog, &can_port,
		&clock_port, &critical_port, &configuration_port, true));
	RoutedFrameCount = 0U;
	response_port = CanInterface_CreateResponsePort(&interface_context);
	CHECK(response_port.queue_response(response_port.context, CAN_GET_MODE,
		2.0f));
	CHECK(!response_port.queue_response(response_port.context,
		CAN_GET_CURRENT_SET, 3.0f));
	CHECK(configuration_port.set_node_id(configuration_port.context, 4U));
	CHECK(configuration_port.get_node_id(configuration_port.context) == 4U);
	CHECK(interface_context.node_id == 3U);
	configure_before = transport.configure_calls;
	generation_before = interface_context.configuration_generation;
	transport.transmit_result = BSP_RESULT_BUSY;
	CanInterface_RunBackground(&interface_context, &router, &watchdog);
	CHECK(interface_context.transmit_pending && transport.transmit_calls == 1U);
	CHECK(transport.configure_calls == configure_before);
	CHECK((uint8_t)(transport.last_transmitted_frame.identifier >> 8) == 3U);
	transport.transmit_result = BSP_RESULT_OK;
	CanInterface_RunBackground(&interface_context, &router, &watchdog);
	CHECK(!interface_context.transmit_pending && transport.transmit_calls == 2U);
	CHECK(transport.configure_calls == configure_before);
	CanInterface_RunBackground(&interface_context, &router, &watchdog);
	CHECK(transport.configure_calls == configure_before + 1U);
	CHECK(transport.stop_calls == 1U && transport.start_calls == 2U);
	CHECK(interface_context.node_id == 4U &&
		transport.configured_node_id == 4U);
	CHECK(interface_context.configuration_generation ==
		(uint8_t)(generation_before + 1U));
	CHECK((uint8_t)(transport.last_transmitted_frame.identifier >> 8) == 3U);

	/* Commands decoded before a node/rate boundary are discarded by generation. */
	CHECK(FakeCan_PushEncoded(&transport, 4U, CAN_SET_CURRENT, 5.0f));
	CanInterface_OnReceiveInterrupt(&interface_context);
	CHECK(configuration_port.set_node_id(configuration_port.context, 5U));
	CanInterface_RunBackground(&interface_context, &router, &watchdog);
	CanInterface_RunBackground(&interface_context, &router, &watchdog);
	CHECK(interface_context.node_id == 5U && RoutedFrameCount == 0U);
	CHECK(FakeCan_PushEncoded(&transport, 5U, CAN_SET_SPEED, 3.0f));
	CanInterface_OnReceiveInterrupt(&interface_context);
	configure_before = transport.configure_calls;
	CHECK(configuration_port.set_bitrate_kbps(configuration_port.context, 500U));
	CanInterface_Supervise1kHz(&interface_context, false, &watchdog);
	CHECK(transport.configure_calls == configure_before);
	CanInterface_RunBackground(&interface_context, &router, &watchdog);
	CanInterface_RunBackground(&interface_context, &router, &watchdog);
	CHECK(interface_context.configured_bitrate == 500U &&
		RoutedFrameCount == 0U);

	/* A target configure failure rolls back identity but latches a sticky fault. */
	CHECK(FakeCan_SetupInterface(&interface_context, &transport, &fault_sink,
		&fake_clock, &fake_critical, &authority, &watchdog, &can_port,
		&clock_port, &critical_port, &configuration_port, true));
	transport.configure_failures_remaining = 1U;
	CHECK(configuration_port.set_node_id(configuration_port.context, 4U));
	CanInterface_RunBackground(&interface_context, &router, &watchdog);
	CHECK(configuration_port.get_node_id(configuration_port.context) == 3U);
	CHECK(interface_context.node_id == 3U &&
		transport.configured_node_id == 3U && interface_context.transport_started);
	CHECK((interface_context.rx_pipeline_faults &
		BSP_COMMUNICATION_FAULT_IO) != 0U);
	CanInterface_Supervise1kHz(&interface_context, false, &watchdog);
	CHECK((CanInterface_GetObservedTransportFaults(&interface_context) &
		BSP_COMMUNICATION_FAULT_IO) != 0U && fault_sink.raised >= 2U);

	/* Stop and double-start rollback failures are sticky and fail closed. */
	CHECK(FakeCan_SetupInterface(&interface_context, &transport, &fault_sink,
		&fake_clock, &fake_critical, &authority, &watchdog, &can_port,
		&clock_port, &critical_port, &configuration_port, true));
	transport.stop_failures_remaining = 1U;
	CHECK(configuration_port.set_node_id(configuration_port.context, 4U));
	CanInterface_RunBackground(&interface_context, &router, &watchdog);
	CHECK(interface_context.node_id == 3U && interface_context.transport_started);
	CHECK((interface_context.rx_pipeline_faults &
		BSP_COMMUNICATION_FAULT_IO) != 0U);

	CHECK(FakeCan_SetupInterface(&interface_context, &transport, &fault_sink,
		&fake_clock, &fake_critical, &authority, &watchdog, &can_port,
		&clock_port, &critical_port, &configuration_port, true));
	transport.start_failures_remaining = 2U;
	CHECK(configuration_port.set_node_id(configuration_port.context, 4U));
	CanInterface_RunBackground(&interface_context, &router, &watchdog);
	CHECK(!interface_context.transport_started && !transport.started);
	CanInterface_Supervise1kHz(&interface_context, false, &watchdog);
	CHECK((CanInterface_GetObservedTransportFaults(&interface_context) &
		BSP_COMMUNICATION_FAULT_IO) != 0U && fault_sink.raised >= 2U);

	/* Initial configuration failure is visible even though transport is stopped. */
	CHECK(FakeCan_SetupInterface(&interface_context, &transport, &fault_sink,
		&fake_clock, &fake_critical, &authority, &watchdog, &can_port,
		&clock_port, &critical_port, &configuration_port, false));
	transport.configure_failures_remaining = 1U;
	CanInterface_ApplyConfiguredBitrate(&interface_context, &watchdog);
	CHECK(!interface_context.transport_started);
	CanInterface_Supervise1kHz(&interface_context, false, &watchdog);
	CHECK((CanInterface_GetObservedTransportFaults(&interface_context) &
		BSP_COMMUNICATION_FAULT_IO) != 0U && fault_sink.raised >= 2U);

	/* A valid FDCAN arrival between heartbeat snapshots prevents false timeout. */
	CHECK(FakeCan_SetupInterface(&interface_context, &transport, &fault_sink,
		&fake_clock, &fake_critical, &authority, &watchdog, &can_port,
		&clock_port, &critical_port, &configuration_port, true));
	CHECK(configuration_port.set_heartbeat_ms(configuration_port.context, 500U));
	ControlAuthorityService_Claim(&authority, CONTROL_AUTHORITY_CAN);
	fake_clock.now_ms = 10U;
	CHECK(FakeCan_PushEncoded(&transport, 3U, CAN_SET_CURRENT, 1.0f));
	CanInterface_OnReceiveInterrupt(&interface_context);
	CanInterface_Supervise1kHz(&interface_context, true, &watchdog);
	fake_clock.now_ms = 510U;
	(void)memset(&injection, 0, sizeof(injection));
	injection.interface_context = &interface_context;
	injection.transport = &transport;
	injection.node_id = 3U;
	injection.parameter = CAN_SET_SPEED;
	injection.value = 2.0f;
	fake_critical.exit_hook = FakeReceive_Inject;
	fake_critical.exit_hook_context = &injection;
	CanInterface_Supervise1kHz(&interface_context, true, &watchdog);
	CHECK(injection.pushed && fault_sink.raised == 0U &&
		fault_sink.cleared == 2U && !interface_context.disconnect_reported);

	/* Invalid port contracts fail initialization instead of dereferencing later. */
	invalid_port = can_port;
	invalid_port.try_receive = NULL;
	CHECK(!CanInterface_Initialize(&invalid_interface, &invalid_port, &clock_port,
		&critical_port, &authority, false, false, 1000U, 0U, 500U, 1000U));
	invalid_port = can_port;
	invalid_capabilities = FakeCanCapabilities;
	invalid_capabilities.features = 0U;
	invalid_port.capabilities = &invalid_capabilities;
	CHECK(!CanInterface_Initialize(&invalid_interface, &invalid_port, &clock_port,
		&critical_port, &authority, false, false, 1000U, 0U, 500U, 1000U));
	CHECK(fake_critical.enter_calls == fake_critical.exit_calls);
	CHECK(fake_critical.depth == 0U);
	return 0;
}
