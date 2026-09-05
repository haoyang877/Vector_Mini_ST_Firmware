#include "interface_usb.h"
#include "telemetry_service.h"
#include "rotor_calibration_service.h"
#include "usb_protocol_v1.h"
#include "usb_command_router.h"
#include "can_configuration_service.h"
#include "friction_identification_service.h"

#include <stdio.h>
#include <string.h>

#include "fast_math.h"
#include "byte_ring_buffer.h"

static UsbInterfaceContext *ActiveContext;
#define USBContext (*ActiveContext)
#define USBRxOverflow (ActiveContext->receive_overflow)
#define USBTransport (ActiveContext->transport)
#define USBTransportInitialized (ActiveContext->transport_is_initialized)
#define USBRxQueue (ActiveContext->receive_queue)
#define tx_en transmit_enabled
#define tx_busy transmit_busy
#define tx_str transmit_text
#define tx_buffer transmit_buffer
#define lut_export_en lut_export_enabled
#define friction_export_en friction_export_enabled
#define print_en print_enabled
#define en_channel_num enabled_channel_count
#define p_var print_channels
#define USB_TRANSMIT_TIMEOUT_MS 250U

bool UsbInterface_Initialize(UsbInterfaceContext *context,
	const ByteTransportPort *transport, const MonotonicClockPort *clock)
{
	if (context == 0 || transport == 0 || transport->transmit == 0 ||
		clock == 0 || clock->read_ms == 0)
		return false;
	ActiveContext = context;
	memset(&USBContext, 0, sizeof(USBContext));
	USBTransport = *transport;
	USBContext.clock = *clock;
	USBRxOverflow = 0U;
	ByteRingBuffer_Initialize(&USBRxQueue);
	USBTransportInitialized = true;
	return true;
}

/**
	* @brief  USB ring buffer data analyze
			  use finite state machine
    * @retval USBRxError
 **/
#define USB_COMMAND_LINE_CAPACITY 64U

static UsbCommandError UsbInterface_ReadCommandLine(uint8_t *line,
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

static UsbCommandError UsbInterface_DecodeNextCommand(UsbProtocolV1Command *command)
{
    uint8_t line[USB_COMMAND_LINE_CAPACITY];
    uint16_t line_length = 0U;
	UsbProtocolV1DecodeResult result;
    UsbCommandError read_result = UsbInterface_ReadCommandLine(line,
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

static void UsbInterface_QueueText(const char *text)
{
	if (text == NULL)
		return;
	(void)snprintf(USBContext.tx_str, sizeof(USBContext.tx_str), "%s", text);
	USBContext.tx_en = 1U;
}

/**
	* @brief  USB receive interrupt handler
	* @param  *data: received data buffer pointer
	* @param  length: received data length
 **/
void UsbInterface_OnReceiveInterrupt(const uint8_t *data, uint32_t length)
{
	if (data == 0 || length == 0U || length > UINT16_MAX)
		return;

	/* IRQ work is bounded to a byte copy; parsing and formatting run in main. */
	if (ByteRingBuffer_Write(&USBRxQueue, data, (uint16_t)length) != length)
		USBRxOverflow = 1U;
}

void UsbInterface_ProcessReceivedCommands(void)
{
	UsbCommandError USBRXError;
	UsbProtocolV1Command command;
	UsbCommandRouterState router_state;
	UsbCommandRouterResponse response;

	if (USBContext.tx_busy != 0U || USBContext.tx_en != 0U ||
		USBContext.lut_export_en != 0U || USBContext.friction_export_en != 0U)
		return;

	if (USBRxOverflow != 0U)
	{
		USBRxOverflow = 0U;
		UsbInterface_QueueText("Receive buffer overflow!\r\n");
		return;
	}

	if (ByteRingBuffer_GetLength(&USBRxQueue) == 0U)
		return;

	USBRXError = UsbInterface_DecodeNextCommand(&command);
	if (USBRXError == USB_INCOMPLETE_DATA)
		return;
	if (USBRXError == USB_SYNTAX_ERROR)
	{
		UsbInterface_QueueText("Syntax error!\r\n");
		return;		
	}
	if (USBRXError == USB_DATA_INVALID)
	{
		UsbInterface_QueueText("Data invalid!\r\n");
		return;
	}

	router_state.print_active = USBContext.print_en != 0U;
	router_state.lut_export_active = USBContext.lut_export_en != 0U;
	router_state.friction_export_active = USBContext.friction_export_en != 0U;
	USBRXError = UsbCommandRouter_Handle(&command, &router_state, &response);
	
	switch(USBRXError)
	{
		case USB_WRITE_INVALID:
		{
				UsbInterface_QueueText("Write invalid!\r\n");
			return;		
		}
		case USB_UNKNOWNED_PARAM:
		{
				UsbInterface_QueueText("Unknowned parameter!\r\n");
			return;
		}
		case USB_DATA_INVALID:
		{
				UsbInterface_QueueText("Data invalid!\r\n");
			return;
		}
		case USB_DATA_OUT_OF_RANGE:
		{
				UsbInterface_QueueText("Data out of range!\r\n");
			return;
		}
		case USB_CYCLIC_OVERFLOW:
		{
				UsbInterface_QueueText("Receive buffer overflow!\r\n");
			return;
		}
		default:
		{
				if (response.action == USB_COMMAND_ROUTER_ACTION_CONFIGURE_PRINT)
				{
					USBContext.p_var[response.print_channel].parameter =
						response.print_parameter;
					USBContext.p_var[response.print_channel].scale =
						response.print_scale;
					USBContext.print_en = 1U;
				}
				else if (response.action == USB_COMMAND_ROUTER_ACTION_BEGIN_LUT_EXPORT)
				{
					USBContext.lut_export_index = 0U;
					USBContext.lut_export_en = 1U;
					UsbInterface_QueueText(response.text);
				}
				else if (response.action == USB_COMMAND_ROUTER_ACTION_BEGIN_FRICTION_EXPORT)
				{
					USBContext.friction_export_index = 0U;
					USBContext.friction_export_en = 1U;
					UsbInterface_QueueText(response.text);
				}
				else if (response.action == USB_COMMAND_ROUTER_ACTION_SEND_TEXT)
				{
					UsbInterface_QueueText(response.text);
				}
		}
		break;
	}
}

/**
	* @brief  Send USB response message
 **/
void UsbInterface_OnTransmitCompleteInterrupt(void)
{
	if (ActiveContext != 0)
		USBContext.tx_busy = 0U;
}

static bool UsbInterface_StartTransmit(const uint8_t *data, uint16_t length)
{
	if (!USBTransportInitialized || data == 0 || length == 0U)
		return false;

	/* Mark busy first: a fast completion IRQ must not be overwritten here. */
	USBContext.tx_busy = 1U;
	USBContext.transmit_started_ms = USBContext.clock.read_ms(
		USBContext.clock.context);
	if (!USBTransport.transmit(USBTransport.context, data, length))
	{
		USBContext.tx_busy = 0U;
		return false;
	}
	return true;
}

static void UsbInterface_RecoverTimedOutTransmit(void)
{
	uint32_t now_ms;
	if (USBContext.tx_busy == 0U)
		return;
	now_ms = USBContext.clock.read_ms(USBContext.clock.context);
	if ((uint32_t)(now_ms - USBContext.transmit_started_ms) <
		USB_TRANSMIT_TIMEOUT_MS)
		return;

	if (USBTransport.cancel_transmit != 0)
		(void)USBTransport.cancel_transmit(USBTransport.context);
	USBContext.tx_busy = 0U;
}

void UsbInterface_FlushTransmit(void)
{
	UsbInterface_RecoverTimedOutTransmit();
	if (USBContext.tx_busy != 0U)
		return;

	if (USBContext.tx_en != 0U)
	{
		uint16_t tx_length = (uint16_t)strlen(USBContext.tx_str);

		memcpy(USBContext.tx_buffer, USBContext.tx_str, tx_length + 1U);
		if (!UsbInterface_StartTransmit((uint8_t *)USBContext.tx_buffer,
			tx_length))
			return;
		USBContext.tx_en = 0U;
		return;
	}

	if (USBContext.print_pending != 0U)
	{
		uint16_t tx_length = (uint16_t)(4U *
			(USBContext.en_channel_num + 1U));
		memcpy(USBContext.tx_buffer, USBContext.print_array, tx_length);
		if (!UsbInterface_StartTransmit((uint8_t *)USBContext.tx_buffer,
			tx_length))
			return;
		USBContext.print_pending = 0U;
		return;
	}

	if (USBContext.lut_export_en == 0U)
	{
		FrictionIdentificationPortSample sample;
		FrictionIdentificationPortStatus status;
		if (USBContext.friction_export_en == 0U) return;
		if (!FrictionIdentificationService_ReadStatus(&status))
		{
			USBContext.friction_export_en = 0U;
			UsbInterface_QueueText("friction_error\r\n");
			return;
		}
		if (USBContext.friction_export_index < status.sample_count)
		{
			if (!FrictionIdentificationService_ReadSample(
					USBContext.friction_export_index, &sample))
			{
				USBContext.friction_export_en = 0U;
				UsbInterface_QueueText("friction_error\r\n");
				return;
			}
			(void)snprintf(USBContext.tx_str, sizeof(USBContext.tx_str),
				"friction=%u,target=%.6f,speed=%.6f,iq=%.6f,n=%lu\r\n",
				(unsigned int)USBContext.friction_export_index,
				sample.target_speed_rad_s * 0.15915494309f,
				sample.mean_speed_rad_s * 0.15915494309f,
				sample.mean_iq_a, (unsigned long)sample.sample_count);
			USBContext.friction_export_index++;
			USBContext.tx_en = 1U;
			return;
		}
		USBContext.friction_export_en = 0U;
		UsbInterface_QueueText("friction_end\r\n");
		return;
	}

	if (USBContext.lut_export_index < RotorCalibrationService_GetEntryCount())
	{
		RotorCalibrationEntry entry;
		uint32_t counts_per_revolution =
			RotorCalibrationService_GetCountsPerRevolution();

		if (!RotorCalibrationService_ReadEntry(USBContext.lut_export_index, &entry) ||
			counts_per_revolution == 0U)
		{
			USBContext.lut_export_en = 0U;
			UsbInterface_QueueText("lut_error\r\n");
			return;
		}

		(void)snprintf(USBContext.tx_str, sizeof(USBContext.tx_str),
			"lut=%u,raw_deg=%.4f,err_deg=%.5f\r\n",
			(unsigned int)USBContext.lut_export_index,
			(float)entry.raw_angle_q15 * (360.0f / (float)counts_per_revolution),
			(float)entry.error_q15 * (360.0f / (float)counts_per_revolution));
		USBContext.lut_export_index++;
		USBContext.tx_en = 1U;
		return;
	}

	USBContext.lut_export_en = 0U;
	UsbInterface_QueueText("lut_end\r\n");
}

/**
	* @brief  Get scaled value for USB print profile
	* @param  *var: variable information pointer
	* @retval scaled variable value
 **/
static float print_get_value(const UsbPrintChannel *var,
	const MotorTelemetrySnapshot *snapshot)
{
	float value;

	switch(var->parameter)
	{
		case USB_MODE: value = (float)snapshot->mode; break;
		case USB_CURRENT_SET: value = snapshot->current_reference_a; break;
		case USB_SPEED_SET: value = snapshot->speed_reference_rad_s; break;
		case USB_POS_SET: value = snapshot->position_reference_rad; break;
		case USB_NODE_ID: value = (float)CanConfigurationService_GetNodeId(); break;
		case USB_POLEPARIS: value = snapshot->pole_pairs; break;
		case USB_ENCODER_STATE: value = (float)snapshot->encoder_online; break;
		case USB_ENCODER_REVERSE: value = (float)snapshot->encoder_reversed; break;
		case USB_CURRENT_CAL: value = snapshot->calibration_current_a; break;
		case USB_CURRENT_LIMIT: value = snapshot->current_limit_a; break;
		case USB_SPEED_LIMIT: value = snapshot->speed_limit_rad_s; break;
		case USB_SPEED_ACC: value = snapshot->speed_acceleration_rad_s2; break;
		case USB_SPEED_DEC: value = snapshot->speed_deceleration_rad_s2; break;
		case USB_SPEED_KP: value = snapshot->speed_kp; break;
		case USB_SPEED_KI: value = snapshot->speed_ki; break;
		case USB_POS_ACC: value = snapshot->position_acceleration_rad_s2; break;
		case USB_POS_DEC: value = snapshot->position_deceleration_rad_s2; break;
		case USB_POS_MAXSPEED: value = snapshot->position_max_speed_rad_s; break;
		case USB_POS_KP: value = snapshot->position_kp_a_per_rad; break;
		case USB_POS_KD: value = snapshot->position_kd_a_per_rad_s; break;
		case USB_POS_KI: value = snapshot->position_ki_a_per_rad_s; break;
		case USB_POS_INTEGRAL_LIMIT: value = snapshot->position_integral_limit_a; break;
		case USB_CASCADE_POS_KP: value = snapshot->cascade_position_kp_per_s; break;
		case USB_CASCADE_POS_KD: value = snapshot->cascade_position_kd; break;
		case USB_CAN_BR: value = (float)CanConfigurationService_GetBitrateKbps(); break;
		case USB_CAN_HB: value = (float)CanConfigurationService_GetHeartbeatMs(); break;
		case USB_VBUS: value = snapshot->bus_voltage_v; break;
		case USB_IBUS: value = snapshot->bus_current_a; break;
		case USB_IA: value = snapshot->phase_a_current_a; break;
		case USB_IB: value = snapshot->phase_b_current_a; break;
		case USB_IC: value = snapshot->phase_c_current_a; break;
		case USB_ID: value = snapshot->d_axis_current_filtered_a; break;
		case USB_IQ: value = snapshot->q_axis_current_filtered_a; break;
		case USB_SPEED2_FILT: value = snapshot->mechanical_speed_rad_s; break;
		case USB_POS2_FILT: value = snapshot->mechanical_position_rad; break;
		case USB_TEMP: value = snapshot->temperature_c; break;
		case USB_RS: value = snapshot->phase_resistance_ohm; break;
		case USB_LD: value = snapshot->d_axis_inductance_h; break;
		case USB_LQ: value = snapshot->q_axis_inductance_h; break;
		case USB_FLUX: value = snapshot->flux_weber; break;
		case USB_ERROR: value = (float)snapshot->primary_error; break;
		default: value = 0.0f; break;
	}

	return value * var->scale;
}

/**
	* @brief  Send USB print profile data
 **/
void UsbInterface_UpdateTelemetryStream(void)
{
	MotorTelemetrySnapshot snapshot;

	if (!USBContext.print_en || USBContext.print_pending != 0U)
		return;
	if (!TelemetryService_ReadSnapshot(&snapshot))
		return;
	
	USBContext.en_channel_num = 5;

	for(int i = 0; i < USBContext.en_channel_num; i++)
	{
		float var_temp = print_get_value(&USBContext.p_var[i], &snapshot);
		USBContext.print_array[i] = FloatBits_Encode(var_temp);
	}
	
	USBContext.print_array[USBContext.en_channel_num] = 0x7F800000;
	/* USB driver access is deferred to the background flush routine. */
	USBContext.print_pending = 1U;
}
