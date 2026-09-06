#include "text_writer.h"

#include <float.h>

#if FLT_RADIX != 2 || FLT_MANT_DIG != 24 || FLT_MAX_EXP != 128
#error "TextWriter_AppendFixedF32 requires an IEEE-754 binary32 float"
#endif

#define TEXT_WRITER_FLOAT_EXPONENT_MASK 0x7F800000UL
#define TEXT_WRITER_FLOAT_FRACTION_MASK 0x007FFFFFUL
#define TEXT_WRITER_FLOAT_SIGN_MASK     0x80000000UL
#define TEXT_WRITER_FLOAT_EXPONENT_SHIFT 23U
#define TEXT_WRITER_FLOAT_EXPONENT_BIAS 127
#define TEXT_WRITER_FLOAT_MANTISSA_BITS 23
#define TEXT_WRITER_BIG_UINT_LIMBS      5U
#define TEXT_WRITER_MAX_DECIMAL_DIGITS  45U

typedef union
{
	float value;
	uint32_t bits;
} TextWriterFloatBits;

typedef char TextWriterFloatMustBe32Bits[
	(sizeof(float) == sizeof(uint32_t)) ? 1 : -1];

typedef struct
{
	uint32_t limb[TEXT_WRITER_BIG_UINT_LIMBS];
} TextWriterBigUInt;

static bool TextWriter_AppendCharacter(TextWriter *writer, char character)
{
	if (writer == NULL)
		return false;
	if (writer->buffer == NULL || writer->capacity == 0U)
	{
		writer->truncated = true;
		return false;
	}

	if (writer->length < writer->capacity - 1U)
	{
		writer->buffer[writer->length++] = character;
		writer->buffer[writer->length] = '\0';
		return !writer->truncated;
	}

	writer->truncated = true;
	writer->buffer[writer->length] = '\0';
	return false;
}

void TextWriter_Initialize(TextWriter *writer, char *buffer, size_t capacity)
{
	if (writer == NULL)
		return;

	writer->buffer = buffer;
	writer->capacity = buffer != NULL ? capacity : 0U;
	writer->length = 0U;
	writer->truncated = false;
	if (writer->capacity > 0U)
		writer->buffer[0] = '\0';
}

bool TextWriter_AppendLiteral(TextWriter *writer, const char *literal)
{
	bool complete = true;

	if (writer == NULL || literal == NULL)
		return false;
	while (*literal != '\0')
	{
		if (!TextWriter_AppendCharacter(writer, *literal))
			complete = false;
		++literal;
	}
	return complete && !writer->truncated;
}

bool TextWriter_AppendU32(TextWriter *writer, uint32_t value)
{
	char reversed[10];
	size_t count = 0U;
	bool complete = true;

	do
	{
		reversed[count++] = (char)('0' + value % 10U);
		value /= 10U;
	} while (value != 0U);

	while (count > 0U)
	{
		if (!TextWriter_AppendCharacter(writer, reversed[--count]))
			complete = false;
	}
	return complete;
}

bool TextWriter_AppendI32(TextWriter *writer, int32_t value)
{
	uint32_t magnitude;
	bool complete = true;

	if (value < 0)
	{
		complete = TextWriter_AppendCharacter(writer, '-');
		/* This form is defined for INT32_MIN; direct negation is not. */
		magnitude = (uint32_t)(-(value + 1)) + 1U;
	}
	else
	{
		magnitude = (uint32_t)value;
	}
	return TextWriter_AppendU32(writer, magnitude) && complete;
}

static void TextWriterBigUInt_Initialize(TextWriterBigUInt *value,
	uint64_t initial)
{
	size_t index;

	value->limb[0] = (uint32_t)initial;
	value->limb[1] = (uint32_t)(initial >> 32U);
	for (index = 2U; index < TEXT_WRITER_BIG_UINT_LIMBS; ++index)
		value->limb[index] = 0U;
}

static void TextWriterBigUInt_ShiftLeft(TextWriterBigUInt *value,
	uint32_t bit_count)
{
	uint32_t bit;

	for (bit = 0U; bit < bit_count; ++bit)
	{
		uint32_t carry = 0U;
		size_t index;

		for (index = 0U; index < TEXT_WRITER_BIG_UINT_LIMBS; ++index)
		{
			uint32_t next_carry = value->limb[index] >> 31U;
			value->limb[index] = (value->limb[index] << 1U) | carry;
			carry = next_carry;
		}
	}
}

static bool TextWriterBigUInt_IsZero(const TextWriterBigUInt *value)
{
	size_t index;

	for (index = 0U; index < TEXT_WRITER_BIG_UINT_LIMBS; ++index)
	{
		if (value->limb[index] != 0U)
			return false;
	}
	return true;
}

static uint32_t TextWriterBigUInt_DivideBy10(TextWriterBigUInt *value)
{
	uint32_t remainder = 0U;
	size_t index = TEXT_WRITER_BIG_UINT_LIMBS;

	while (index > 0U)
	{
		uint32_t dividend;
		uint32_t quotient_high;
		uint32_t quotient_low;

		--index;
		/* Two base-2^16 steps avoid pulling a 64-bit division runtime. */
		dividend = (remainder << 16U) | (value->limb[index] >> 16U);
		quotient_high = dividend / 10U;
		remainder = dividend % 10U;
		dividend = (remainder << 16U) | (value->limb[index] & 0xFFFFU);
		quotient_low = dividend / 10U;
		remainder = dividend % 10U;
		value->limb[index] = (quotient_high << 16U) | quotient_low;
	}
	return remainder;
}

static uint64_t TextWriter_ScaleAndRoundSmall(uint32_t mantissa,
	uint32_t decimal_scale, uint32_t right_shift)
{
	uint64_t scaled = (uint64_t)mantissa * decimal_scale;

	if (right_shift == 0U)
		return scaled;
	if (right_shift >= 64U)
		return 0U;
	return (scaled + (UINT64_C(1) << (right_shift - 1U))) >> right_shift;
}

static bool TextWriter_AppendScaledDecimal(TextWriter *writer,
	TextWriterBigUInt scaled, uint8_t precision)
{
	char reversed[TEXT_WRITER_MAX_DECIMAL_DIGITS];
	size_t count = 0U;
	bool complete = true;

	do
	{
		reversed[count++] = (char)('0' +
			TextWriterBigUInt_DivideBy10(&scaled));
	} while (!TextWriterBigUInt_IsZero(&scaled) &&
		count < sizeof(reversed));

	while (count <= precision)
		reversed[count++] = '0';

	while (count > 0U)
	{
		if (count == precision &&
			!TextWriter_AppendCharacter(writer, '.'))
			complete = false;
		if (!TextWriter_AppendCharacter(writer, reversed[--count]))
			complete = false;
	}
	return complete;
}

bool TextWriter_AppendFixedF32(TextWriter *writer, float value,
	uint8_t precision)
{
	static const uint32_t DecimalScales[7] =
	{
		1U, 10U, 100U, 1000U, 10000U, 100000U, 1000000U
	};
	TextWriterFloatBits representation;
	TextWriterBigUInt scaled_value;
	uint32_t exponent_bits;
	uint32_t fraction_bits;
	uint32_t mantissa;
	uint64_t initial;
	int32_t binary_shift;
	bool complete = true;

	if (writer == NULL || precision < 1U || precision > 6U)
		return false;

	representation.value = value;
	exponent_bits = (representation.bits & TEXT_WRITER_FLOAT_EXPONENT_MASK) >>
		TEXT_WRITER_FLOAT_EXPONENT_SHIFT;
	fraction_bits = representation.bits & TEXT_WRITER_FLOAT_FRACTION_MASK;
	if (exponent_bits == 0xFFU)
	{
		if (fraction_bits != 0U)
			return TextWriter_AppendLiteral(writer, "nan");
		if ((representation.bits & TEXT_WRITER_FLOAT_SIGN_MASK) != 0U)
			complete = TextWriter_AppendCharacter(writer, '-');
		return TextWriter_AppendLiteral(writer, "inf") && complete;
	}

	if ((representation.bits & TEXT_WRITER_FLOAT_SIGN_MASK) != 0U)
		complete = TextWriter_AppendCharacter(writer, '-');

	if (exponent_bits == 0U)
	{
		mantissa = fraction_bits;
		binary_shift = -149;
	}
	else
	{
		mantissa = fraction_bits | (UINT32_C(1) <<
			TEXT_WRITER_FLOAT_MANTISSA_BITS);
		binary_shift = (int32_t)exponent_bits -
			TEXT_WRITER_FLOAT_EXPONENT_BIAS -
			TEXT_WRITER_FLOAT_MANTISSA_BITS;
	}

	if (binary_shift < 0)
	{
		initial = TextWriter_ScaleAndRoundSmall(mantissa,
			DecimalScales[precision], (uint32_t)(-binary_shift));
		TextWriterBigUInt_Initialize(&scaled_value, initial);
	}
	else
	{
		initial = (uint64_t)mantissa * DecimalScales[precision];
		TextWriterBigUInt_Initialize(&scaled_value, initial);
		TextWriterBigUInt_ShiftLeft(&scaled_value, (uint32_t)binary_shift);
	}

	return TextWriter_AppendScaledDecimal(writer, scaled_value, precision) &&
		complete;
}

size_t TextWriter_GetLength(const TextWriter *writer)
{
	return writer != NULL ? writer->length : 0U;
}

bool TextWriter_WasTruncated(const TextWriter *writer)
{
	return writer != NULL && writer->truncated;
}
