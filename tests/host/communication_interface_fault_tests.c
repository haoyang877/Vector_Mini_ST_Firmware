#include "Communication/interface_can.h"

#include <string.h>

typedef struct
{
	BspCommunicationFaultSet faults;
	BspCanFrame frame;
	bool frame_ready;
	BspResult transmit_result;
	uint32_t transmit_calls;
} FakeCanTransport;

typedef struct
{
	uint32_t raised;
	uint32_t cleared;
} FakeFaultSink;

static uint32_t RoutedFrameCount;

#define CHECK(expression) do { if (!(expression)) return __LINE__; } while (0)

static BspCommunicationFaultSet FakeCan_ReadFaults(void *raw_context)
{
	return ((FakeCanTransport *)raw_context)->faults;
}

static BspResult FakeCan_TryReceive(void *raw_context, BspCanFrame *frame)
{
	FakeCanTransport *transport = (FakeCanTransport *)raw_context;

	if (!transport->frame_ready)
		return BSP_RESULT_NOT_READY;
	*frame = transport->frame;
	transport->frame_ready = false;
	return BSP_RESULT_OK;
}

static BspResult FakeCan_TryTransmit(void *raw_context,
	const BspCanFrame *frame)
{
	FakeCanTransport *transport = (FakeCanTransport *)raw_context;

	if (frame == NULL)
		return BSP_RESULT_INVALID_ARGUMENT;
	transport->transmit_calls++;
	return transport->transmit_result;
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
	(void)parameter_id;
	(void)value;
	RoutedFrameCount++;
}

int CommunicationInterfaceFault_RunHostTests(void)
{
	CanInterfaceContext interface_context;
	CanCommandRouterContext router;
	CommunicationWatchdogServiceContext watchdog;
	FakeCanTransport transport;
	FakeFaultSink fault_sink;
	FaultCommandPort fault_port;
	CanResponsePort response_port;

	(void)memset(&interface_context, 0, sizeof(interface_context));
	(void)memset(&router, 0, sizeof(router));
	(void)memset(&transport, 0, sizeof(transport));
	(void)memset(&fault_sink, 0, sizeof(fault_sink));
	fault_port.context = &fault_sink;
	fault_port.raise = FakeFault_Raise;
	fault_port.clear = FakeFault_Clear;
	CHECK(CommunicationWatchdogService_Initialize(&watchdog, &fault_port, 7U));

	interface_context.node_id = 3U;
	interface_context.transport.context = &transport;
	interface_context.transport.read_faults = FakeCan_ReadFaults;
	interface_context.transport.try_receive = FakeCan_TryReceive;
	interface_context.transport.try_transmit = FakeCan_TryTransmit;
	interface_context.transport_started = true;
	transport.transmit_result = BSP_RESULT_OK;
	CHECK(CanProtocolV1_Encode(interface_context.node_id, CAN_SET_CURRENT,
		1.25f, &transport.frame));
	transport.frame_ready = true;
	RoutedFrameCount = 0U;
	CanInterface_ProcessReceivedFrames(&interface_context, &router, &watchdog);
	CHECK(fault_sink.cleared == 1U);
	CHECK(RoutedFrameCount == 1U);

	transport.faults = BSP_COMMUNICATION_FAULT_RX_OVERFLOW;
	CanInterface_UpdateWatchdog(&interface_context, NULL, &watchdog);
	CHECK(fault_sink.raised == 1U);
	CHECK(interface_context.disconnect_reported);
	CHECK(CanInterface_GetObservedTransportFaults(&interface_context) ==
		BSP_COMMUNICATION_FAULT_RX_OVERFLOW);

	transport.frame_ready = true;
	CanInterface_ProcessReceivedFrames(&interface_context, &router, &watchdog);
	CHECK(fault_sink.cleared == 1U);
	CHECK(interface_context.disconnect_reported);
	CHECK(RoutedFrameCount == 2U);

	response_port = CanInterface_CreateResponsePort(&interface_context);
	CHECK(response_port.queue_response(response_port.context, CAN_GET_MODE,
		2.0f));
	CHECK(!response_port.queue_response(response_port.context,
		CAN_GET_CURRENT_SET, 3.0f));
	CHECK(transport.frame_ready == false);
	CHECK(CanProtocolV1_Encode(interface_context.node_id, CAN_SET_CURRENT,
		2.5f, &transport.frame));
	transport.frame_ready = true;
	CanInterface_ProcessReceivedFrames(&interface_context, &router, &watchdog);
	CHECK(transport.frame_ready);
	CHECK(RoutedFrameCount == 2U);
	transport.transmit_result = BSP_RESULT_BUSY;
	CanInterface_FlushTransmit(&interface_context);
	CHECK(interface_context.transmit_pending);
	CHECK(transport.transmit_calls == 1U);
	transport.transmit_result = BSP_RESULT_OK;
	CanInterface_FlushTransmit(&interface_context);
	CHECK(!interface_context.transmit_pending);
	CHECK(transport.transmit_calls == 2U);
	CanInterface_ProcessReceivedFrames(&interface_context, &router, &watchdog);
	CHECK(!transport.frame_ready);
	CHECK(RoutedFrameCount == 3U);

	transport.faults |= BSP_COMMUNICATION_FAULT_BUS_OFF;
	CanInterface_UpdateWatchdog(&interface_context, NULL, &watchdog);
	CHECK(fault_sink.raised == 1U);
	CHECK(CanInterface_GetObservedTransportFaults(&interface_context) ==
		(BSP_COMMUNICATION_FAULT_RX_OVERFLOW |
		 BSP_COMMUNICATION_FAULT_BUS_OFF));
	return 0;
}
