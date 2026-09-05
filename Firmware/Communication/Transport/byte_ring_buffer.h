#ifndef COMMUNICATION_BYTE_RING_BUFFER_H
#define COMMUNICATION_BYTE_RING_BUFFER_H

#include <stdint.h>

#define BYTE_RING_BUFFER_CAPACITY 256U

typedef struct
{
	volatile uint16_t read_index;
	volatile uint16_t write_index;
	uint8_t storage[BYTE_RING_BUFFER_CAPACITY];
} ByteRingBufferContext;

void ByteRingBuffer_Initialize(ByteRingBufferContext *context);
void ByteRingBuffer_Consume(ByteRingBufferContext *context, uint16_t length);
uint8_t ByteRingBuffer_Peek(const ByteRingBufferContext *context,
	uint16_t offset);
uint16_t ByteRingBuffer_GetLength(const ByteRingBufferContext *context);
uint16_t ByteRingBuffer_GetRemaining(const ByteRingBufferContext *context);
uint16_t ByteRingBuffer_Write(ByteRingBufferContext *context,
	const uint8_t *data, uint16_t length);

#endif
