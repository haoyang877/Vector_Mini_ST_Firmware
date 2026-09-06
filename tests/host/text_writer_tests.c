#include "Core/Communication/Formatting/text_writer.h"

#include <float.h>
#include <stdint.h>
#include <string.h>

#define CHECK(condition) do { if (!(condition)) return __LINE__; } while (0)

static int TextWriter_CheckFixed(float value, uint8_t precision,
	const char *expected)
{
	char buffer[64];
	TextWriter writer;

	TextWriter_Initialize(&writer, buffer, sizeof(buffer));
	CHECK(TextWriter_AppendFixedF32(&writer, value, precision));
	CHECK(!TextWriter_WasTruncated(&writer));
	CHECK(TextWriter_GetLength(&writer) == strlen(expected));
	CHECK(strcmp(buffer, expected) == 0);
	return 0;
}

int TextWriter_RunHostTests(void)
{
	char buffer[64];
	char short_buffer[5];
	TextWriter writer;
	int result;
	volatile float positive_infinity = 1.0f / 0.0f;
	volatile float not_a_number = 0.0f / 0.0f;

	TextWriter_Initialize(&writer, buffer, sizeof(buffer));
	CHECK(TextWriter_AppendLiteral(&writer, "value="));
	CHECK(TextWriter_AppendU32(&writer, UINT32_MAX));
	CHECK(TextWriter_AppendLiteral(&writer, ",signed="));
	CHECK(TextWriter_AppendI32(&writer, INT32_MIN));
	CHECK(strcmp(buffer, "value=4294967295,signed=-2147483648") == 0);

	result = TextWriter_CheckFixed(123.456f, 2U, "123.46");
	CHECK(result == 0);
	result = TextWriter_CheckFixed(1.25f, 1U, "1.3");
	CHECK(result == 0);
	result = TextWriter_CheckFixed(-1.25f, 1U, "-1.3");
	CHECK(result == 0);
	result = TextWriter_CheckFixed(-0.0f, 2U, "-0.00");
	CHECK(result == 0);
	result = TextWriter_CheckFixed(0.125f, 2U, "0.13");
	CHECK(result == 0);
	result = TextWriter_CheckFixed(9.9996f, 3U, "10.000");
	CHECK(result == 0);
	result = TextWriter_CheckFixed(0.03125f, 4U, "0.0313");
	CHECK(result == 0);
	result = TextWriter_CheckFixed(0.015625f, 5U, "0.01563");
	CHECK(result == 0);
	result = TextWriter_CheckFixed(0.0078125f, 6U, "0.007813");
	CHECK(result == 0);
	result = TextWriter_CheckFixed(FLT_MAX, 1U,
		"340282346638528859811704183484516925440.0");
	CHECK(result == 0);
	result = TextWriter_CheckFixed(FLT_MIN, 6U, "0.000000");
	CHECK(result == 0);
	result = TextWriter_CheckFixed(-positive_infinity, 2U, "-inf");
	CHECK(result == 0);
	result = TextWriter_CheckFixed(not_a_number, 2U, "nan");
	CHECK(result == 0);

	TextWriter_Initialize(&writer, buffer, sizeof(buffer));
	CHECK(!TextWriter_AppendFixedF32(&writer, 1.0f, 0U));
	CHECK(!TextWriter_AppendFixedF32(&writer, 1.0f, 7U));
	CHECK(strcmp(buffer, "") == 0);

	TextWriter_Initialize(&writer, short_buffer, sizeof(short_buffer));
	CHECK(!TextWriter_AppendLiteral(&writer, "abcdef"));
	CHECK(strcmp(short_buffer, "abcd") == 0);
	CHECK(TextWriter_GetLength(&writer) == 4U);
	CHECK(TextWriter_WasTruncated(&writer));
	CHECK(!TextWriter_AppendU32(&writer, 42U));
	CHECK(strcmp(short_buffer, "abcd") == 0);

	TextWriter_Initialize(&writer, NULL, 0U);
	CHECK(!TextWriter_AppendLiteral(&writer, "x"));
	CHECK(TextWriter_GetLength(&writer) == 0U);
	CHECK(TextWriter_WasTruncated(&writer));
	return 0;
}

#ifdef TEXT_WRITER_STANDALONE
int main(void)
{
	return TextWriter_RunHostTests();
}
#endif
