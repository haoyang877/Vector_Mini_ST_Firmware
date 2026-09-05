#ifndef COMMUNICATION_INTERFACE_USB_H
#define COMMUNICATION_INTERFACE_USB_H

#include <stdbool.h>
#include <stdint.h>
#include "byte_transport_port.h"
#include "monotonic_clock_port.h"
#include "usb_protocol_contract.h"
#include "usb_protocol_v1.h"
#include "byte_ring_buffer.h"

#define USB_COMMAND_MIN_LENGTH 8U
#define USB_INTERFACE_TEXT_CAPACITY 80U

typedef struct
{
	UsbParameterId parameter;
	float scale;
} UsbPrintChannel;

typedef struct
{
	uint8_t transmit_enabled;
	volatile uint8_t transmit_busy;
	char transmit_text[USB_INTERFACE_TEXT_CAPACITY];
	char transmit_buffer[USB_INTERFACE_TEXT_CAPACITY];
	uint8_t lut_export_enabled;
	uint16_t lut_export_index;
	uint8_t friction_export_enabled;
	uint8_t friction_export_index;
	uint8_t print_enabled;
	volatile uint8_t print_pending;
	uint8_t enabled_channel_count;
	UsbPrintChannel print_channels[5];
	uint32_t print_array[6];
	volatile uint8_t receive_overflow;
	ByteTransportPort transport;
	MonotonicClockPort clock;
	bool transport_is_initialized;
	uint32_t transmit_started_ms;
	ByteRingBufferContext receive_queue;
} UsbInterfaceContext;

bool UsbInterface_Initialize(UsbInterfaceContext *context,
	const ByteTransportPort *transport, const MonotonicClockPort *clock);
void UsbInterface_OnReceiveInterrupt(const uint8_t *data, uint32_t length);
void UsbInterface_ProcessReceivedCommands(void);
void UsbInterface_OnTransmitCompleteInterrupt(void);
void UsbInterface_FlushTransmit(void);
void UsbInterface_UpdateTelemetryStream(void);

#endif
