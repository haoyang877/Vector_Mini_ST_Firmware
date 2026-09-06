#include <math.h>
#include <stdint.h>
#include <string.h>

#include "fast_math.h"

#define FAST_MATH_TEST_LUT_SIZE 1024U

static uint32_t FastMathTest_FloatBits(float value)
{
	uint32_t bits;

	(void)memcpy(&bits, &value, sizeof(bits));
	return bits;
}

int FastMath_RunHostTests(void)
{
	uint32_t index;

	/* Exercise every lookup bucket. The retained literals are samples of a
	 * 1023-interval turn; this catches a reversed-index or endpoint error in
	 * the half-wave reconstruction without duplicating the table in tests. */
	for (index = 0U; index < FAST_MATH_TEST_LUT_SIZE; index++)
	{
		float theta = ((float)index + 0.25f) / FAST_MATH_LUT_SCALE;
		float expected = sinf(MATH_TWO_PI * (float)index /
			(float)(FAST_MATH_TEST_LUT_SIZE - 1U));
		float actual = FastMath_Sin(theta);

		if (fabsf(actual - expected) > 2.0e-6f)
			return __LINE__;
	}

	/* The original final entry was positive zero; keep the bit-level result
	 * stable because telemetry and regression captures may inspect it. */
	if (FastMathTest_FloatBits(FastMath_Sin(
		(1023.25f / FAST_MATH_LUT_SCALE))) != 0U)
		return __LINE__;

	for (index = 0U; index < FAST_MATH_TEST_LUT_SIZE; index += 7U)
	{
		float theta = ((float)index + 0.25f) / FAST_MATH_LUT_SCALE;
		float sine;
		float cosine;

		FastMath_SinCos(theta, &sine, &cosine);
		if (FastMathTest_FloatBits(sine) !=
			FastMathTest_FloatBits(FastMath_Sin(theta)) ||
			FastMathTest_FloatBits(cosine) !=
			FastMathTest_FloatBits(FastMath_Cos(theta)))
		{
			return __LINE__;
		}
	}

	return 0;
}
