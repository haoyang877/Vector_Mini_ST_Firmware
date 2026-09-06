#ifndef COMMUNICATION_INTERFACE_USB_H
#define COMMUNICATION_INTERFACE_USB_H

#include <stdbool.h>
#include <stdint.h>

#include "Bsp/Api/bsp_communication.h"
#include "Bsp/Api/bsp_system.h"
#include "Core/Application/friction_identification_service.h"
#include "Core/Application/rotor_calibration_service.h"
#include "Core/Infrastructure/Telemetry/telemetry_service.h"
#include "byte_ring_buffer.h"
#include "can_configuration_service.h"
#include "usb_command_router.h"
#include "usb_protocol_contract.h"
#include "usb_protocol_v1.h"

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
	char transmit_text[USB_INTERFACE_TEXT_CAPACITY];
	uint8_t transmit_buffer[USB_INTERFACE_TEXT_CAPACITY];
	uint16_t transmit_length;
	uint16_t transmit_offset;
	bool transmit_pending;
	bool transmit_wait_active;
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
	/* Sticky until this context is reinitialized. */
	BspCommunicationFaultSet observed_transport_faults;
	BspByteStreamPort transport;
	BspMonotonicClockPort clock;
	bool transport_is_initialized;
	uint32_t transmit_started_ms;
	ByteRingBufferContext receive_queue;
} UsbInterfaceContext;

bool UsbInterface_Initialize(UsbInterfaceContext *context,
	const BspByteStreamPort *transport, const BspMonotonicClockPort *clock);
void UsbInterface_ProcessReceivedCommands(UsbInterfaceContext *context,
	UsbCommandRouterContext *router);
void UsbInterface_FlushTransmit(UsbInterfaceContext *context,
	const FrictionIdentificationServiceContext *friction,
	const RotorCalibrationServiceContext *rotor_calibration);
void UsbInterface_UpdateTelemetryStream(UsbInterfaceContext *context,
	const TelemetryServiceContext *telemetry,
	const CanConfigurationServiceContext *can_configuration);
BspCommunicationFaultSet UsbInterface_GetObservedTransportFaults(
	const UsbInterfaceContext *context);

#endif
