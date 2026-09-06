#ifndef CORE_COMMUNICATION_FORMATTING_TEXT_WRITER_H
#define CORE_COMMUNICATION_FORMATTING_TEXT_WRITER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * Bounded, allocation-free text construction for embedded protocol replies.
 *
 * A writer always keeps a non-empty destination NUL terminated.  When output
 * does not fit, the prefix that fits is retained and WasTruncated() becomes
 * true.  AppendFixedF32() implements decimal round-half-up for precisions 1..6
 * without depending on stdio or the floating-point printf runtime.
 */
typedef struct
{
	char *buffer;
	size_t capacity;
	size_t length;
	bool truncated;
} TextWriter;

void TextWriter_Initialize(TextWriter *writer, char *buffer, size_t capacity);
bool TextWriter_AppendLiteral(TextWriter *writer, const char *literal);
bool TextWriter_AppendU32(TextWriter *writer, uint32_t value);
bool TextWriter_AppendI32(TextWriter *writer, int32_t value);
bool TextWriter_AppendFixedF32(TextWriter *writer, float value,
	uint8_t precision);
size_t TextWriter_GetLength(const TextWriter *writer);
bool TextWriter_WasTruncated(const TextWriter *writer);

#endif
