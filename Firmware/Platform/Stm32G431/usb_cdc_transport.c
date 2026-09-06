#include "usb_cdc_transport.h"

#include <stddef.h>
#include <string.h>

#include "usbd_cdc_if.h"

#if defined(__CC_ARM)
#pragma O3
#pragma Ospace
#endif

/* Monotonic 8-bit sequences make all 64 slots usable without a sentinel. */
#define USB_CDC_TRANSPORT_RX_STORAGE_CAPACITY 64U
#define USB_CDC_TRANSPORT_TX_STORAGE_CAPACITY 64U

typedef struct
{
	volatile BspCommunicationFaultSet interrupt_faults;
	BspCommunicationFaultSet foreground_faults;
	uint8_t receive_storage[USB_CDC_TRANSPORT_RX_STORAGE_CAPACITY];
	uint8_t transmit_storage[USB_CDC_TRANSPORT_TX_STORAGE_CAPACITY];
	volatile uint8_t receive_read_sequence;
	volatile uint8_t receive_write_sequence;
	volatile bool transmit_busy;
	volatile bool started;
} UsbCdcTransportContext;

static UsbCdcTransportContext UsbCdcTransportState;

static BspResult UsbCdcTransport_Start(void *context)
{
	UsbCdcTransportContext *transport = (UsbCdcTransportContext *)context;

	if (transport == NULL)
		return BSP_RESULT_INVALID_ARGUMENT;
	if (transport->started)
		return BSP_RESULT_OK;
	transport->receive_read_sequence = 0U;
	transport->receive_write_sequence = 0U;
	transport->transmit_busy = false;
	transport->started = true;
	return BSP_RESULT_OK;
}

static BspResult UsbCdcTransport_Stop(void *context)
{
	UsbCdcTransportContext *transport = (UsbCdcTransportContext *)context;

	if (transport == NULL)
		return BSP_RESULT_INVALID_ARGUMENT;
	if (!transport->started)
		return BSP_RESULT_OK;
	transport->started = false;
	if (transport->transmit_busy)
	{
		if (CDC_AbortTransmit_FS() != USBD_OK)
		{
			transport->started = true;
			transport->foreground_faults |= BSP_COMMUNICATION_FAULT_IO;
			return BSP_RESULT_IO_ERROR;
		}
		transport->transmit_busy = false;
	}
	transport->receive_read_sequence = 0U;
	transport->receive_write_sequence = 0U;
	return BSP_RESULT_OK;
}

static BspResult UsbCdcTransport_TryRead(void *context, uint8_t *buffer,
	size_t capacity, size_t *read_count)
{
	UsbCdcTransportContext *transport = (UsbCdcTransportContext *)context;
	uint8_t available;
	uint8_t count;
	uint8_t index;

	if (transport == NULL || read_count == NULL ||
		(buffer == NULL && capacity != 0U))
	{
		return BSP_RESULT_INVALID_ARGUMENT;
	}
	*read_count = 0U;
	if (!transport->started)
		return BSP_RESULT_NOT_READY;
	if (capacity == 0U)
		return BSP_RESULT_OK;

	available = (uint8_t)(transport->receive_write_sequence -
		transport->receive_read_sequence);
	if (available == 0U)
		return BSP_RESULT_NOT_READY;
	count = available;
	if ((size_t)count > capacity)
		count = (uint8_t)capacity;
	for (index = 0U; index < count; ++index)
	{
		buffer[index] = transport->receive_storage[
			(uint8_t)(transport->receive_read_sequence + index) &
			(USB_CDC_TRANSPORT_RX_STORAGE_CAPACITY - 1U)];
	}
	transport->receive_read_sequence =
		(uint8_t)(transport->receive_read_sequence + count);
	*read_count = count;
	return BSP_RESULT_OK;
}

static BspResult UsbCdcTransport_TryWrite(void *context,
	const uint8_t *data, size_t length, size_t *accepted_count)
{
	UsbCdcTransportContext *transport = (UsbCdcTransportContext *)context;
	size_t count;
	uint8_t status;

	if (transport == NULL || accepted_count == NULL ||
		(data == NULL && length != 0U))
	{
		return BSP_RESULT_INVALID_ARGUMENT;
	}
	*accepted_count = 0U;
	if (!transport->started)
		return BSP_RESULT_NOT_READY;
	if (transport->transmit_busy)
		return BSP_RESULT_BUSY;
	if (length == 0U)
		return BSP_RESULT_OK;

	count = length;
	if (count > USB_CDC_TRANSPORT_TX_STORAGE_CAPACITY)
		count = USB_CDC_TRANSPORT_TX_STORAGE_CAPACITY;
	(void)memcpy(transport->transmit_storage, data, count);
	/* Publish busy before handing the buffer to USB: completion may be fast. */
	transport->transmit_busy = true;
	status = CDC_Transmit_FS(transport->transmit_storage, (uint16_t)count);
	if (status == USBD_OK)
	{
		*accepted_count = count;
		return BSP_RESULT_OK;
	}
	transport->transmit_busy = false;
	if (status == USBD_BUSY)
		return BSP_RESULT_BUSY;
	transport->foreground_faults |= BSP_COMMUNICATION_FAULT_IO;
	return BSP_RESULT_IO_ERROR;
}

static BspResult UsbCdcTransport_CancelWrite(void *context)
{
	UsbCdcTransportContext *transport = (UsbCdcTransportContext *)context;

	if (transport == NULL)
		return BSP_RESULT_INVALID_ARGUMENT;
	if (!transport->transmit_busy)
		return BSP_RESULT_OK;
	if (CDC_AbortTransmit_FS() != USBD_OK)
	{
		transport->foreground_faults |= BSP_COMMUNICATION_FAULT_IO;
		return BSP_RESULT_IO_ERROR;
	}
	transport->transmit_busy = false;
	return BSP_RESULT_OK;
}

static BspCommunicationFaultSet UsbCdcTransport_ReadFaults(void *context)
{
	const UsbCdcTransportContext *transport =
		(const UsbCdcTransportContext *)context;

	if (transport == NULL)
		return BSP_COMMUNICATION_FAULT_IO;
	return transport->foreground_faults | transport->interrupt_faults;
}

void UsbCdcTransport_OnReceiveInterrupt(const uint8_t *data,
	uint32_t length)
{
	UsbCdcTransportContext *transport = &UsbCdcTransportState;
	uint8_t index;

	if (!transport->started || data == NULL || length == 0U)
		return;
	if (length > (uint8_t)(USB_CDC_TRANSPORT_RX_STORAGE_CAPACITY -
		(uint8_t)(transport->receive_write_sequence -
			transport->receive_read_sequence)))
	{
		transport->interrupt_faults |= BSP_COMMUNICATION_FAULT_RX_OVERFLOW;
		return;
	}

	for (index = 0U; index < (uint8_t)length; ++index)
	{
		transport->receive_storage[
			(uint8_t)(transport->receive_write_sequence + index) &
			(USB_CDC_TRANSPORT_RX_STORAGE_CAPACITY - 1U)] = data[index];
	}
	/* Publish the new index last so the reader only sees complete bytes. */
	transport->receive_write_sequence =
		(uint8_t)(transport->receive_write_sequence + length);
}

void UsbCdcTransport_OnTransmitCompleteInterrupt(void)
{
	UsbCdcTransportState.transmit_busy = false;
}

bool UsbCdcTransport_CreatePort(
	const BspCommunicationEndpointCapabilities *capabilities,
	BspByteStreamPort *port)
{
	const BspCommunicationFeatureSet features =
		BSP_COMMUNICATION_FEATURE_BYTE_STREAM |
		BSP_COMMUNICATION_FEATURE_FULL_DUPLEX;

	if (port == NULL || capabilities == NULL ||
		capabilities->endpoint_id == BSP_ENDPOINT_ID_NONE ||
		capabilities->availability != BSP_ENDPOINT_AVAILABLE ||
		capabilities->kind != BSP_COMMUNICATION_BYTE_STREAM ||
		capabilities->features != features ||
		capabilities->maximum_payload_bytes !=
			USB_CDC_TRANSPORT_TX_STORAGE_CAPACITY ||
		capabilities->maximum_nominal_bit_rate != 0U ||
		capabilities->maximum_data_bit_rate != 0U)
	{
		return false;
	}
	(void)memset(&UsbCdcTransportState, 0, sizeof(UsbCdcTransportState));
	port->context = &UsbCdcTransportState;
	port->capabilities = capabilities;
	port->start = UsbCdcTransport_Start;
	port->stop = UsbCdcTransport_Stop;
	port->try_read = UsbCdcTransport_TryRead;
	port->try_write = UsbCdcTransport_TryWrite;
	port->cancel_write = UsbCdcTransport_CancelWrite;
	port->read_faults = UsbCdcTransport_ReadFaults;
	return true;
}
