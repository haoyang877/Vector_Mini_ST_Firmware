#include "Core/Communication/Interfaces/interface_usb.h"
#include "Core/Infrastructure/Telemetry/telemetry_service.h"
#include "Core/Application/rotor_calibration_service.h"
#include "Core/Communication/Protocol/usb_protocol_v1.h"
#include "Core/Communication/Router/usb_command_router.h"
#include "Core/Application/Communication/can_configuration_service.h"
#include "Core/Application/friction_identification_service.h"
#include "Core/Communication/Formatting/text_writer.h"

#include <stddef.h>
#include <string.h>

#include "fast_math.h"
#include "Core/Communication/Transport/byte_ring_buffer.h"

#if defined(__CC_ARM)
#pragma O3
#pragma Ospace
#endif

#define USBContext (*context)
#define USBRxOverflow (context->receive_overflow)
#define USBTransport (context->transport)
#define USBTransportInitialized (context->transport_is_initialized)
#define USBRxQueue (context->receive_queue)
#define tx_en transmit_enabled
#define tx_str transmit_text
#define tx_buffer transmit_buffer
#define lut_export_en lut_export_enabled
#define friction_export_en friction_export_enabled
#define print_en print_enabled
#define en_channel_num enabled_channel_count
#define p_var print_channels
#define USB_TRANSMIT_TIMEOUT_MS 250U

#define USB_SNAPSHOT_WORD_OFFSET(member_) \
	(offsetof(MotorTelemetrySnapshot, member_) / sizeof(uint32_t))

typedef char UsbPrintSourceSpecialRangeMustRemainReserved[
	MOTOR_TELEMETRY_PHASE_RESISTANCE_DESIGN_ERROR_PERCENT <
		USB_COMMAND_ROUTER_PRINT_ZERO ? 1 : -1];
typedef char UsbSnapshotScalarWidthMustRemainOneWord[
	sizeof(float) == sizeof(uint32_t) &&
	sizeof(((MotorTelemetrySnapshot *)0)->mode) == sizeof(uint32_t) ? 1 : -1];
typedef char UsbSnapshotStatusFieldsMustRemainContiguous[
	USB_SNAPSHOT_WORD_OFFSET(latched_faults) ==
		USB_SNAPSHOT_WORD_OFFSET(mode) + 3U ? 1 : -1];
typedef char UsbSnapshotReferenceFieldsMustRemainContiguous[
	USB_SNAPSHOT_WORD_OFFSET(position_reference_rad) ==
		USB_SNAPSHOT_WORD_OFFSET(current_reference_a) + 2U ? 1 : -1];
typedef char UsbSnapshotMeasurementFieldsMustRemainContiguous[
	USB_SNAPSHOT_WORD_OFFSET(encoder_reversed) ==
		USB_SNAPSHOT_WORD_OFFSET(bus_voltage_v) +
		(MOTOR_TELEMETRY_ENCODER_REVERSED -
			MOTOR_TELEMETRY_BUS_VOLTAGE_V) ? 1 : -1];
typedef char UsbSnapshotConfigurationFieldsMustRemainContiguous[
	USB_SNAPSHOT_WORD_OFFSET(phase_resistance_design_error_percent) ==
		USB_SNAPSHOT_WORD_OFFSET(pole_pairs) +
		(MOTOR_TELEMETRY_PHASE_RESISTANCE_DESIGN_ERROR_PERCENT -
			MOTOR_TELEMETRY_POLE_PAIRS) ? 1 : -1];

bool UsbInterface_Initialize(UsbInterfaceContext *context,
	const BspByteStreamPort *transport, const BspMonotonicClockPort *clock)
{
	if (context == 0 || transport == 0 || transport->capabilities == 0 ||
		transport->capabilities->kind != BSP_COMMUNICATION_BYTE_STREAM ||
		transport->start == 0 || transport->stop == 0 ||
		transport->try_read == 0 || transport->try_write == 0 ||
		transport->cancel_write == 0 || transport->read_faults == 0 ||
		clock == 0 || clock->read_ms == 0)
		return false;
	memset(&USBContext, 0, sizeof(USBContext));
	USBTransport = *transport;
	USBContext.clock = *clock;
	USBRxOverflow = 0U;
	ByteRingBuffer_Initialize(&USBRxQueue);
	if (USBTransport.start(USBTransport.context) != BSP_RESULT_OK)
		return false;
	USBTransportInitialized = true;
	return true;
}

/**
	* @brief  USB ring buffer data analyze
			  use finite state machine
    * @retval USBRxError
 **/
#define USB_COMMAND_LINE_CAPACITY 64U

static UsbCommandError UsbInterface_ReadCommandLine(UsbInterfaceContext *context,
	uint8_t *line,
    uint16_t capacity, uint16_t *line_length)
{
    uint16_t available;
    uint16_t index;

	if (line == NULL || line_length == NULL ||
		capacity < USB_COMMAND_MIN_LENGTH)
        return USB_SYNTAX_ERROR;

    available = ByteRingBuffer_GetLength(&USBRxQueue);
    while (available > 0U && ByteRingBuffer_Peek(&USBRxQueue, 0U) != '\\')
    {
        ByteRingBuffer_Consume(&USBRxQueue, 1U);
        available--;
    }
	if (available < USB_COMMAND_MIN_LENGTH)
        return USB_INCOMPLETE_DATA;

    for (index = 1U; index < available && index < capacity; ++index)
    {
        if (ByteRingBuffer_Peek(&USBRxQueue, index - 1U) == '\r' &&
			ByteRingBuffer_Peek(&USBRxQueue, index) == '\n')
        {
            uint16_t copy_index;
            *line_length = index + 1U;
            for (copy_index = 0U; copy_index < *line_length; ++copy_index)
				line[copy_index] = ByteRingBuffer_Peek(&USBRxQueue, copy_index);
			ByteRingBuffer_Consume(&USBRxQueue, *line_length);
            return USB_NO_ERROR;
        }
    }

    if (available >= capacity)
    {
		ByteRingBuffer_Consume(&USBRxQueue, 1U);
        return USB_SYNTAX_ERROR;
    }
    return USB_INCOMPLETE_DATA;
}

static UsbCommandError UsbInterface_DecodeNextCommand(
	UsbInterfaceContext *context, UsbProtocolV1Command *command)
{
    uint8_t line[USB_COMMAND_LINE_CAPACITY];
    uint16_t line_length = 0U;
    UsbProtocolV1DecodeResult result;
    UsbCommandError read_result = UsbInterface_ReadCommandLine(context, line,
        USB_COMMAND_LINE_CAPACITY, &line_length);

    if (read_result != USB_NO_ERROR)
        return read_result;
    result = UsbProtocolV1_Decode(line, line_length, command);
    if (result == USB_PROTOCOL_V1_DECODE_SYNTAX_ERROR)
        return USB_SYNTAX_ERROR;
    if (result == USB_PROTOCOL_V1_DECODE_INVALID_NUMBER)
        return USB_DATA_INVALID;
    return USB_NO_ERROR;
}

static void UsbInterface_QueueText(UsbInterfaceContext *context,
	const char *text)
{
	TextWriter writer;

	if (text == NULL)
		return;
	TextWriter_Initialize(&writer, USBContext.tx_str,
		sizeof(USBContext.tx_str));
	(void)TextWriter_AppendLiteral(&writer, text);
	USBContext.tx_en = 1U;
}

static void UsbInterface_PollReceive(UsbInterfaceContext *context)
{
	BspCommunicationFaultSet faults;
	BspCommunicationFaultSet new_faults;
	uint8_t received[USB_COMMAND_LINE_CAPACITY];
	size_t read_count = 0U;
	BspResult result;

	if (!USBTransportInitialized)
		return;
	faults = USBTransport.read_faults(USBTransport.context);
	new_faults = faults & ~USBContext.observed_transport_faults;
	USBContext.observed_transport_faults |= faults;
	if ((new_faults & BSP_COMMUNICATION_FAULT_RX_OVERFLOW) != 0U)
		USBRxOverflow = 1U;
	result = USBTransport.try_read(USBTransport.context, received,
		sizeof(received), &read_count);
	if (result == BSP_RESULT_NOT_READY || result == BSP_RESULT_BUSY)
		return;
	if (result != BSP_RESULT_OK || read_count > UINT16_MAX ||
		(read_count != 0U && ByteRingBuffer_Write(&USBRxQueue, received,
			(uint16_t)read_count) != read_count))
	{
		USBRxOverflow = 1U;
	}
}

void UsbInterface_ProcessReceivedCommands(UsbInterfaceContext *context,
	UsbCommandRouterContext *router)
{
	UsbCommandError USBRXError;
	UsbProtocolV1Command command;
	UsbCommandRouterState router_state;
	UsbCommandRouterResponse response;

	if (context == 0 || router == 0)
		return;
	UsbInterface_PollReceive(context);
	if (USBContext.transmit_pending || USBContext.tx_en != 0U ||
		USBContext.lut_export_en != 0U || USBContext.friction_export_en != 0U)
		return;

	if (USBRxOverflow != 0U)
	{
		USBRxOverflow = 0U;
		UsbInterface_QueueText(context, "Receive buffer overflow!\r\n");
		return;
	}

	if (ByteRingBuffer_GetLength(&USBRxQueue) == 0U)
		return;

	USBRXError = UsbInterface_DecodeNextCommand(context, &command);
	if (USBRXError == USB_INCOMPLETE_DATA)
		return;
	if (USBRXError == USB_SYNTAX_ERROR)
	{
		UsbInterface_QueueText(context, "Syntax error!\r\n");
		return;		
	}
	if (USBRXError == USB_DATA_INVALID)
	{
		UsbInterface_QueueText(context, "Data invalid!\r\n");
		return;
	}

	router_state.print_active = USBContext.print_en != 0U;
	router_state.lut_export_active = USBContext.lut_export_en != 0U;
	router_state.friction_export_active = USBContext.friction_export_en != 0U;
	USBRXError = UsbCommandRouter_Handle(router, &command, &router_state,
		&response);
	
	switch(USBRXError)
	{
		case USB_WRITE_INVALID:
			UsbInterface_QueueText(context, "Write invalid!\r\n");
			return;
		case USB_UNKNOWNED_PARAM:
			UsbInterface_QueueText(context, "Unknowned parameter!\r\n");
			return;
		case USB_DATA_INVALID:
			UsbInterface_QueueText(context, "Data invalid!\r\n");
			return;
		case USB_DATA_OUT_OF_RANGE:
			UsbInterface_QueueText(context, "Data out of range!\r\n");
			return;
		case USB_CYCLIC_OVERFLOW:
			UsbInterface_QueueText(context, "Receive buffer overflow!\r\n");
			return;
		default:
			if (response.action == USB_COMMAND_ROUTER_ACTION_CONFIGURE_PRINT)
			{
				USBContext.p_var[response.print_channel].source =
					response.print_source;
				USBContext.p_var[response.print_channel].scale =
					response.print_scale;
				USBContext.print_en = 1U;
			}
			else if (response.action == USB_COMMAND_ROUTER_ACTION_BEGIN_LUT_EXPORT)
			{
				USBContext.lut_export_index = 0U;
				USBContext.lut_export_en = 1U;
				UsbInterface_QueueText(context, response.text);
			}
			else if (response.action == USB_COMMAND_ROUTER_ACTION_BEGIN_FRICTION_EXPORT)
			{
				USBContext.friction_export_index = 0U;
				USBContext.friction_export_en = 1U;
				UsbInterface_QueueText(context, response.text);
			}
			else if (response.action == USB_COMMAND_ROUTER_ACTION_SEND_TEXT)
				UsbInterface_QueueText(context, response.text);
			break;
	}
}

static bool UsbInterface_BeginTransmit(UsbInterfaceContext *context,
	const void *data, uint16_t length)
{
	if (!USBTransportInitialized || data == NULL || length == 0U ||
		length > sizeof(USBContext.tx_buffer) || USBContext.transmit_pending)
	{
		return false;
	}
	memcpy(USBContext.tx_buffer, data, length);
	USBContext.transmit_length = length;
	USBContext.transmit_offset = 0U;
	USBContext.transmit_wait_active = false;
	USBContext.transmit_pending = true;
	return true;
}

static void UsbInterface_ContinueTransmit(UsbInterfaceContext *context)
{
	size_t accepted_count = 0U;
	size_t remaining;
	BspResult result;
	uint32_t now_ms;

	if (!USBTransportInitialized || !USBContext.transmit_pending)
		return;
	remaining = (size_t)USBContext.transmit_length -
		USBContext.transmit_offset;
	result = USBTransport.try_write(USBTransport.context,
		&USBContext.tx_buffer[USBContext.transmit_offset], remaining,
		&accepted_count);
	if (result == BSP_RESULT_OK)
	{
		if (accepted_count > remaining)
			accepted_count = 0U;
		USBContext.transmit_offset += (uint16_t)accepted_count;
		if (USBContext.transmit_offset == USBContext.transmit_length)
		{
			USBContext.transmit_pending = false;
			USBContext.transmit_wait_active = false;
			return;
		}
	}
	now_ms = USBContext.clock.read_ms(USBContext.clock.context);
	if (!USBContext.transmit_wait_active)
	{
		USBContext.transmit_started_ms = now_ms;
		USBContext.transmit_wait_active = true;
		return;
	}
	if ((uint32_t)(now_ms - USBContext.transmit_started_ms) >=
		USB_TRANSMIT_TIMEOUT_MS)
	{
		BspResult cancel_result = USBTransport.cancel_write(
			USBTransport.context);

		/* A byte stream cannot resume a logical response after an accepted
		 * prefix is aborted: doing so would splice a suffix onto whatever the
		 * host receives next.  Drop the complete logical message and expose a
		 * sticky TX fault.  A later producer may start a fresh message. */
		USBContext.observed_transport_faults |=
			BSP_COMMUNICATION_FAULT_TX_OVERFLOW;
		if (cancel_result != BSP_RESULT_OK)
		{
			USBContext.observed_transport_faults |=
				BSP_COMMUNICATION_FAULT_IO;
		}
		USBContext.transmit_pending = false;
		USBContext.transmit_length = 0U;
		USBContext.transmit_offset = 0U;
		USBContext.transmit_wait_active = false;
	}
}

void UsbInterface_FlushTransmit(UsbInterfaceContext *context,
	const FrictionIdentificationServiceContext *friction,
	const RotorCalibrationServiceContext *rotor_calibration)
{
	if (context == 0 || friction == 0 || rotor_calibration == 0)
		return;
	if (USBContext.transmit_pending)
	{
		UsbInterface_ContinueTransmit(context);
		return;
	}

	if (USBContext.tx_en != 0U)
	{
		uint16_t tx_length = (uint16_t)strlen(USBContext.tx_str);

		if (!UsbInterface_BeginTransmit(context, USBContext.tx_str, tx_length))
			return;
		USBContext.tx_en = 0U;
		UsbInterface_ContinueTransmit(context);
		return;
	}

	if (USBContext.print_pending != 0U)
	{
		uint16_t tx_length = (uint16_t)(4U *
			(USBContext.en_channel_num + 1U));
		if (!UsbInterface_BeginTransmit(context, USBContext.print_array,
			tx_length))
			return;
		USBContext.print_pending = 0U;
		UsbInterface_ContinueTransmit(context);
		return;
	}

	if (USBContext.lut_export_en == 0U)
	{
		FrictionIdentificationPortSample sample;
		FrictionIdentificationPortStatus status;
		if (USBContext.friction_export_en == 0U) return;
		if (!FrictionIdentificationService_ReadStatus(friction, &status))
		{
			USBContext.friction_export_en = 0U;
			UsbInterface_QueueText(context, "friction_error\r\n");
			return;
		}
		if (USBContext.friction_export_index < status.sample_count)
		{
			TextWriter writer;

			if (!FrictionIdentificationService_ReadSample(friction,
					USBContext.friction_export_index, &sample))
			{
				USBContext.friction_export_en = 0U;
				UsbInterface_QueueText(context, "friction_error\r\n");
				return;
			}
			TextWriter_Initialize(&writer, USBContext.tx_str,
				sizeof(USBContext.tx_str));
			(void)TextWriter_AppendLiteral(&writer, "friction=");
			(void)TextWriter_AppendU32(&writer,
				USBContext.friction_export_index);
			(void)TextWriter_AppendLiteral(&writer, ",target=");
			(void)TextWriter_AppendFixedF32(&writer,
				sample.target_speed_rad_s * 0.15915494309f, 6U);
			(void)TextWriter_AppendLiteral(&writer, ",speed=");
			(void)TextWriter_AppendFixedF32(&writer,
				sample.mean_speed_rad_s * 0.15915494309f, 6U);
			(void)TextWriter_AppendLiteral(&writer, ",iq=");
			(void)TextWriter_AppendFixedF32(&writer, sample.mean_iq_a, 6U);
			(void)TextWriter_AppendLiteral(&writer, ",n=");
			(void)TextWriter_AppendU32(&writer, sample.sample_count);
			(void)TextWriter_AppendLiteral(&writer, "\r\n");
			USBContext.friction_export_index++;
			USBContext.tx_en = 1U;
			return;
		}
		USBContext.friction_export_en = 0U;
		UsbInterface_QueueText(context, "friction_end\r\n");
		return;
	}

	if (USBContext.lut_export_index <
		RotorCalibrationService_GetEntryCount(rotor_calibration))
	{
		RotorCalibrationEntry entry;
		TextWriter writer;
		uint32_t counts_per_revolution =
			RotorCalibrationService_GetCountsPerRevolution(rotor_calibration);

		if (!RotorCalibrationService_ReadEntry(rotor_calibration,
			USBContext.lut_export_index, &entry) ||
			counts_per_revolution == 0U)
		{
			USBContext.lut_export_en = 0U;
			UsbInterface_QueueText(context, "lut_error\r\n");
			return;
		}

		TextWriter_Initialize(&writer, USBContext.tx_str,
			sizeof(USBContext.tx_str));
		(void)TextWriter_AppendLiteral(&writer, "lut=");
		(void)TextWriter_AppendU32(&writer, USBContext.lut_export_index);
		(void)TextWriter_AppendLiteral(&writer, ",raw_deg=");
		(void)TextWriter_AppendFixedF32(&writer,
			(float)entry.raw_angle_q15 *
				(360.0f / (float)counts_per_revolution), 4U);
		(void)TextWriter_AppendLiteral(&writer, ",err_deg=");
		(void)TextWriter_AppendFixedF32(&writer,
			(float)entry.error_q15 *
				(360.0f / (float)counts_per_revolution), 5U);
		(void)TextWriter_AppendLiteral(&writer, "\r\n");
		USBContext.lut_export_index++;
		USBContext.tx_en = 1U;
		return;
	}

	USBContext.lut_export_en = 0U;
	UsbInterface_QueueText(context, "lut_end\r\n");
}

BspCommunicationFaultSet UsbInterface_GetObservedTransportFaults(
	const UsbInterfaceContext *context)
{
	return context != NULL ? context->observed_transport_faults :
		BSP_COMMUNICATION_FAULT_IO;
}

/**
	* @brief  Get scaled value for USB print profile
	* @param  *var: variable information pointer
	* @retval scaled variable value
 **/
static float UsbInterface_ReadPrintValue(const UsbPrintChannel *channel,
	const MotorTelemetrySnapshot *snapshot,
	const CanConfigurationServiceContext *can_configuration)
{
	UsbCommandRouterPrintSource source = channel->source;
	uint8_t telemetry;
	uint8_t word_offset;
	uint32_t raw_value;
	float value;

	if (source == USB_COMMAND_ROUTER_PRINT_ZERO)
		return 0.0f;
	if (source == USB_COMMAND_ROUTER_PRINT_CAN_NODE_ID)
		value = (float)CanConfigurationService_GetNodeId(can_configuration);
	else if (source == USB_COMMAND_ROUTER_PRINT_CAN_BITRATE)
		value = (float)CanConfigurationService_GetBitrateKbps(can_configuration);
	else if (source == USB_COMMAND_ROUTER_PRINT_CAN_HEARTBEAT)
		value = (float)CanConfigurationService_GetHeartbeatMs(can_configuration);
	else
	{
		telemetry = source & USB_COMMAND_ROUTER_PRINT_TELEMETRY_MASK;
		if (telemetry > MOTOR_TELEMETRY_PHASE_RESISTANCE_DESIGN_ERROR_PERCENT)
			return 0.0f;
		if (telemetry <= MOTOR_TELEMETRY_LATCHED_FAULTS)
			word_offset = (uint8_t)(USB_SNAPSHOT_WORD_OFFSET(mode) + telemetry);
		else if (telemetry <= MOTOR_TELEMETRY_POSITION_REFERENCE_RAD)
			word_offset = (uint8_t)(USB_SNAPSHOT_WORD_OFFSET(current_reference_a) +
				telemetry -
				MOTOR_TELEMETRY_CURRENT_REFERENCE_A);
		else if (telemetry <= MOTOR_TELEMETRY_ENCODER_REVERSED)
			word_offset = (uint8_t)(USB_SNAPSHOT_WORD_OFFSET(bus_voltage_v) +
				telemetry -
				MOTOR_TELEMETRY_BUS_VOLTAGE_V);
		else
			word_offset = (uint8_t)(USB_SNAPSHOT_WORD_OFFSET(pole_pairs) +
				telemetry -
				MOTOR_TELEMETRY_POLE_PAIRS);

		(void)memcpy(&raw_value,
			(const uint8_t *)(const void *)snapshot +
				((size_t)word_offset * sizeof(uint32_t)),
			sizeof(raw_value));
		if ((source & USB_COMMAND_ROUTER_PRINT_INTEGER_FLAG) != 0U)
			value = (float)raw_value;
		else
			(void)memcpy(&value, &raw_value, sizeof(value));
	}
	return value * channel->scale;
}

/**
	* @brief  Send USB print profile data
 **/
void UsbInterface_UpdateTelemetryStream(UsbInterfaceContext *context,
	const TelemetryServiceContext *telemetry,
	const CanConfigurationServiceContext *can_configuration)
{
	MotorTelemetrySnapshot snapshot;

	if (context == 0 || telemetry == 0 || can_configuration == 0 ||
		!USBContext.print_en || USBContext.print_pending != 0U)
		return;
	if (!TelemetryService_ReadSnapshot(telemetry, &snapshot))
		return;
	
	USBContext.en_channel_num = 5;

	for(int i = 0; i < USBContext.en_channel_num; i++)
	{
		float var_temp = UsbInterface_ReadPrintValue(&USBContext.p_var[i], &snapshot,
			can_configuration);
		USBContext.print_array[i] = FloatBits_Encode(var_temp);
	}
	
	USBContext.print_array[USBContext.en_channel_num] = 0x7F800000;
	/* USB driver access is deferred to the background flush routine. */
	USBContext.print_pending = 1U;
}
