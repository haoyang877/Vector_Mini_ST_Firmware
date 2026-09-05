#ifndef COMMUNICATION_INTERFACE_USB_H
#define COMMUNICATION_INTERFACE_USB_H

#include <stdbool.h>
#include <stdint.h>
#include "byte_transport_port.h"
#include "monotonic_clock_port.h"
#include "usb_protocol_contract.h"
#include "usb_protocol_v1.h"
#include "byte_ring_buffer.h"
#include "can_configuration_service.h"
#include "friction_identification_service.h"
#include "rotor_calibration_service.h"
#include "telemetry_service.h"
#include "usb_command_router.h"

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
void UsbInterface_OnReceiveInterrupt(UsbInterfaceContext *context,
	const uint8_t *data, uint32_t length);
void UsbInterface_ProcessReceivedCommands(UsbInterfaceContext *context,
	UsbCommandRouterContext *router);
void UsbInterface_OnTransmitCompleteInterrupt(UsbInterfaceContext *context);
void UsbInterface_FlushTransmit(UsbInterfaceContext *context,
	const FrictionIdentificationServiceContext *friction,
	const RotorCalibrationServiceContext *rotor_calibration);
void UsbInterface_UpdateTelemetryStream(UsbInterfaceContext *context,
	const TelemetryServiceContext *telemetry,
	const CanConfigurationServiceContext *can_configuration);

#endif
