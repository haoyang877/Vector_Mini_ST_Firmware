#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define UsbInterface_Initialize UsbInterface_TestInitialize
#define UsbInterface_ProcessReceivedCommands UsbInterface_TestProcessReceivedCommands
#define UsbInterface_FlushTransmit UsbInterface_TestFlushTransmit
#define UsbInterface_UpdateTelemetryStream UsbInterface_TestUpdateTelemetryStream
#define UsbInterface_GetObservedTransportFaults UsbInterface_TestGetObservedTransportFaults
#define UsbCommandRouter_Handle UsbCommandRouter_InterfaceTestHandle
#include "../../Firmware/Core/Communication/Interfaces/interface_usb.c"
#undef UsbCommandRouter_Handle
#undef UsbInterface_GetObservedTransportFaults
#undef UsbInterface_UpdateTelemetryStream
#undef UsbInterface_FlushTransmit
#undef UsbInterface_ProcessReceivedCommands
#undef UsbInterface_Initialize

#define CHECK(expression_) do { if (!(expression_)) return __LINE__; } while (0)
#define CLOSE(a_, b_) (fabsf((a_) - (b_)) <= 0.0001f)

typedef struct
{
	uint8_t node_id;
	uint32_t bitrate_kbps;
	uint32_t heartbeat_ms;
} UsbInterfaceFakeCanConfiguration;

UsbCommandError UsbCommandRouter_InterfaceTestHandle(
	UsbCommandRouterContext *context, const UsbProtocolV1Command *command,
	const UsbCommandRouterState *state, UsbCommandRouterResponse *response)
{
	(void)context;
	(void)command;
	(void)state;
	(void)response;
	return USB_UNKNOWNED_PARAM;
}

static bool UsbInterfaceFakeSetNodeId(void *context, uint8_t node_id)
{
	((UsbInterfaceFakeCanConfiguration *)context)->node_id = node_id;
	return true;
}

static uint8_t UsbInterfaceFakeGetNodeId(void *context)
{
	return ((UsbInterfaceFakeCanConfiguration *)context)->node_id;
}

static bool UsbInterfaceFakeSetBitrate(void *context, uint32_t bitrate_kbps)
{
	((UsbInterfaceFakeCanConfiguration *)context)->bitrate_kbps = bitrate_kbps;
	return true;
}

static uint32_t UsbInterfaceFakeGetBitrate(void *context)
{
	return ((UsbInterfaceFakeCanConfiguration *)context)->bitrate_kbps;
}

static bool UsbInterfaceFakeSetHeartbeat(void *context, uint32_t heartbeat_ms)
{
	((UsbInterfaceFakeCanConfiguration *)context)->heartbeat_ms = heartbeat_ms;
	return true;
}

static uint32_t UsbInterfaceFakeGetHeartbeat(void *context)
{
	return ((UsbInterfaceFakeCanConfiguration *)context)->heartbeat_ms;
}

static float UsbInterfaceReadSource(UsbCommandRouterPrintSource source,
	float scale, const MotorTelemetrySnapshot *snapshot,
	const CanConfigurationServiceContext *can_configuration)
{
	UsbPrintChannel channel;

	channel.source = source;
	channel.scale = scale;
	return UsbInterface_ReadPrintValue(&channel, snapshot, can_configuration);
}

int UsbInterface_RunHostTests(void)
{
	UsbInterfaceFakeCanConfiguration fake_can;
	CanConfigurationPort can_port;
	CanConfigurationServiceContext can_configuration;
	TelemetryServiceContext telemetry;
	MotorTelemetrySnapshot snapshot;
	UsbInterfaceContext interface_context;
	unsigned int index;

	(void)memset(&fake_can, 0, sizeof(fake_can));
	(void)memset(&can_port, 0, sizeof(can_port));
	(void)memset(&snapshot, 0, sizeof(snapshot));
	(void)memset(&interface_context, 0, sizeof(interface_context));
	fake_can.node_id = 6U;
	fake_can.bitrate_kbps = 1000U;
	fake_can.heartbeat_ms = 250U;
	can_port.context = &fake_can;
	can_port.set_node_id = UsbInterfaceFakeSetNodeId;
	can_port.get_node_id = UsbInterfaceFakeGetNodeId;
	can_port.set_bitrate_kbps = UsbInterfaceFakeSetBitrate;
	can_port.get_bitrate_kbps = UsbInterfaceFakeGetBitrate;
	can_port.set_heartbeat_ms = UsbInterfaceFakeSetHeartbeat;
	can_port.get_heartbeat_ms = UsbInterfaceFakeGetHeartbeat;
	CHECK(CanConfigurationService_Initialize(&can_configuration, &can_port));

	snapshot.mode = 3U;
	snapshot.primary_error = 9U;
	snapshot.current_reference_a = 1.25f;
	snapshot.bus_voltage_v = 28.0f;
	snapshot.d_axis_current_filtered_a = 4.5f;
	snapshot.encoder_online = 1U;
	snapshot.pole_pairs = 7.0f;
	snapshot.phase_resistance_design_error_percent = 12.5f;
	CHECK(CLOSE(UsbInterfaceReadSource(MOTOR_TELEMETRY_MODE |
		USB_COMMAND_ROUTER_PRINT_INTEGER_FLAG, 1.0f, &snapshot,
		&can_configuration), 3.0f));
	CHECK(CLOSE(UsbInterfaceReadSource(MOTOR_TELEMETRY_PRIMARY_ERROR |
		USB_COMMAND_ROUTER_PRINT_INTEGER_FLAG, 1.0f, &snapshot,
		&can_configuration), 9.0f));
	CHECK(CLOSE(UsbInterfaceReadSource(MOTOR_TELEMETRY_CURRENT_REFERENCE_A,
		2.0f, &snapshot, &can_configuration), 2.5f));
	CHECK(CLOSE(UsbInterfaceReadSource(MOTOR_TELEMETRY_BUS_VOLTAGE_V,
		1.0f, &snapshot, &can_configuration), 28.0f));
	CHECK(CLOSE(UsbInterfaceReadSource(
		MOTOR_TELEMETRY_D_AXIS_CURRENT_FILTERED_A, 1.0f, &snapshot,
		&can_configuration), 4.5f));
	CHECK(CLOSE(UsbInterfaceReadSource(MOTOR_TELEMETRY_ENCODER_ONLINE |
		USB_COMMAND_ROUTER_PRINT_INTEGER_FLAG, 1.0f, &snapshot,
		&can_configuration), 1.0f));
	CHECK(CLOSE(UsbInterfaceReadSource(MOTOR_TELEMETRY_POLE_PAIRS,
		1.0f, &snapshot, &can_configuration), 7.0f));
	CHECK(CLOSE(UsbInterfaceReadSource(
		MOTOR_TELEMETRY_PHASE_RESISTANCE_DESIGN_ERROR_PERCENT, 1.0f,
		&snapshot, &can_configuration), 12.5f));
	CHECK(CLOSE(UsbInterfaceReadSource(USB_COMMAND_ROUTER_PRINT_CAN_NODE_ID,
		1.0f, &snapshot, &can_configuration), 6.0f));
	CHECK(CLOSE(UsbInterfaceReadSource(USB_COMMAND_ROUTER_PRINT_CAN_BITRATE,
		1.0f, &snapshot, &can_configuration), 1000.0f));
	CHECK(CLOSE(UsbInterfaceReadSource(USB_COMMAND_ROUTER_PRINT_CAN_HEARTBEAT,
		1.0f, &snapshot, &can_configuration), 250.0f));
	CHECK(CLOSE(UsbInterfaceReadSource(USB_COMMAND_ROUTER_PRINT_ZERO,
		5.0f, &snapshot, &can_configuration), 0.0f));

	CHECK(TelemetryService_Initialize(&telemetry));
	TelemetryService_Publish(&telemetry, &snapshot);
	interface_context.print_enabled = 1U;
	interface_context.print_channels[0].source = MOTOR_TELEMETRY_MODE |
		USB_COMMAND_ROUTER_PRINT_INTEGER_FLAG;
	interface_context.print_channels[1].source =
		MOTOR_TELEMETRY_CURRENT_REFERENCE_A;
	interface_context.print_channels[2].source =
		MOTOR_TELEMETRY_D_AXIS_CURRENT_FILTERED_A;
	interface_context.print_channels[3].source =
		USB_COMMAND_ROUTER_PRINT_CAN_NODE_ID;
	interface_context.print_channels[4].source = USB_COMMAND_ROUTER_PRINT_ZERO;
	for (index = 0U; index < 5U; ++index)
		interface_context.print_channels[index].scale = 1.0f;
	UsbInterface_TestUpdateTelemetryStream(&interface_context, &telemetry,
		&can_configuration);
	CHECK(interface_context.enabled_channel_count == 5U);
	CHECK(interface_context.print_pending == 1U);
	CHECK(CLOSE(FloatBits_Decode(interface_context.print_array[0]), 3.0f));
	CHECK(CLOSE(FloatBits_Decode(interface_context.print_array[1]), 1.25f));
	CHECK(CLOSE(FloatBits_Decode(interface_context.print_array[2]), 4.5f));
	CHECK(CLOSE(FloatBits_Decode(interface_context.print_array[3]), 6.0f));
	CHECK(CLOSE(FloatBits_Decode(interface_context.print_array[4]), 0.0f));
	CHECK(interface_context.print_array[5] == 0x7F800000UL);
	return 0;
}
