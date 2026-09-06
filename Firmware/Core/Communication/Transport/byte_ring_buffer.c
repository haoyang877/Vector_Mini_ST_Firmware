#include "Core/Communication/Transport/byte_ring_buffer.h"

#include <stddef.h>
#include <string.h>

void ByteRingBuffer_Initialize(ByteRingBufferContext *context)
{
	if (context == NULL)
		return;
	context->read_index = 0U;
	context->write_index = 0U;
}

void ByteRingBuffer_Consume(ByteRingBufferContext *context, uint16_t length)
{
	uint16_t available;

	if (context == NULL)
		return;
	available = ByteRingBuffer_GetLength(context);
	if (length > available)
		length = available;
	context->read_index = (uint16_t)((context->read_index + length) %
		BYTE_RING_BUFFER_CAPACITY);
}

uint8_t ByteRingBuffer_Peek(const ByteRingBufferContext *context,
	uint16_t offset)
{
	if (context == NULL || offset >= ByteRingBuffer_GetLength(context))
		return 0U;
	return context->storage[(context->read_index + offset) %
		BYTE_RING_BUFFER_CAPACITY];
}

uint16_t ByteRingBuffer_GetLength(const ByteRingBufferContext *context)
{
	if (context == NULL)
		return 0U;
	return (uint16_t)((context->write_index - context->read_index +
		BYTE_RING_BUFFER_CAPACITY) % BYTE_RING_BUFFER_CAPACITY);
}

uint16_t ByteRingBuffer_GetRemaining(const ByteRingBufferContext *context)
{
	return (uint16_t)((BYTE_RING_BUFFER_CAPACITY - 1U) -
		ByteRingBuffer_GetLength(context));
}

uint16_t ByteRingBuffer_Write(ByteRingBufferContext *context,
	const uint8_t *data, uint16_t length)
{
	uint16_t first_length;

	if (context == NULL || data == NULL || length == 0U ||
		ByteRingBuffer_GetRemaining(context) < length)
		return 0U;

	first_length = (uint16_t)(BYTE_RING_BUFFER_CAPACITY -
		context->write_index);
	if (first_length > length)
		first_length = length;
	memcpy(&context->storage[context->write_index], data, first_length);
	if (length > first_length)
		memcpy(context->storage, data + first_length, length - first_length);
	context->write_index = (uint16_t)((context->write_index + length) %
		BYTE_RING_BUFFER_CAPACITY);
	return length;
}
