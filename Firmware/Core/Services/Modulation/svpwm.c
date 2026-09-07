#include "svpwm.h"

#include <stddef.h>

#if defined(__CC_ARM)
#pragma O3
#pragma Otime
#endif

#define SVPWM_ONE_BY_SQRT_3 0.57735026919f
#define SVPWM_TWO_BY_SQRT_3 1.15470053838f

static bool Svpwm_IsValidDuty(float duty)
{
	return duty == duty && duty >= 0.0f && duty <= 1.0f;
}

bool Svpwm_Calculate(float alpha, float beta, SvpwmOutput *output)
{
	float phase_a;
	float phase_b;
	float phase_c;
	uint8_t sector;

	if (output == NULL || alpha != alpha || beta != beta)
		return false;

	if (beta >= 0.0f)
	{
		if (alpha >= 0.0f)
			sector = SVPWM_ONE_BY_SQRT_3 * beta > alpha ? 2U : 1U;
		else
			sector = -SVPWM_ONE_BY_SQRT_3 * beta > alpha ? 3U : 2U;
	}
	else
	{
		if (alpha >= 0.0f)
			sector = -SVPWM_ONE_BY_SQRT_3 * beta > alpha ? 5U : 6U;
		else
			sector = SVPWM_ONE_BY_SQRT_3 * beta > alpha ? 4U : 5U;
	}

	switch (sector)
	{
		case 1U:
		{
			float t1 = alpha - SVPWM_ONE_BY_SQRT_3 * beta;
			float t2 = SVPWM_TWO_BY_SQRT_3 * beta;
			phase_a = (1.0f - t1 - t2) * 0.5f;
			phase_b = phase_a + t1;
			phase_c = phase_b + t2;
			break;
		}
		case 2U:
		{
			float t2 = alpha + SVPWM_ONE_BY_SQRT_3 * beta;
			float t3 = -alpha + SVPWM_ONE_BY_SQRT_3 * beta;
			phase_b = (1.0f - t2 - t3) * 0.5f;
			phase_a = phase_b + t3;
			phase_c = phase_a + t2;
			break;
		}
		case 3U:
		{
			float t3 = SVPWM_TWO_BY_SQRT_3 * beta;
			float t4 = -alpha - SVPWM_ONE_BY_SQRT_3 * beta;
			phase_b = (1.0f - t3 - t4) * 0.5f;
			phase_c = phase_b + t3;
			phase_a = phase_c + t4;
			break;
		}
		case 4U:
		{
			float t4 = -alpha + SVPWM_ONE_BY_SQRT_3 * beta;
			float t5 = -SVPWM_TWO_BY_SQRT_3 * beta;
			phase_c = (1.0f - t4 - t5) * 0.5f;
			phase_b = phase_c + t5;
			phase_a = phase_b + t4;
			break;
		}
		case 5U:
		{
			float t5 = -alpha - SVPWM_ONE_BY_SQRT_3 * beta;
			float t6 = alpha - SVPWM_ONE_BY_SQRT_3 * beta;
			phase_c = (1.0f - t5 - t6) * 0.5f;
			phase_a = phase_c + t5;
			phase_b = phase_a + t6;
			break;
		}
		default:
		{
			float t6 = -SVPWM_TWO_BY_SQRT_3 * beta;
			float t1 = alpha + SVPWM_ONE_BY_SQRT_3 * beta;
			phase_a = (1.0f - t6 - t1) * 0.5f;
			phase_c = phase_a + t1;
			phase_b = phase_c + t6;
			break;
		}
	}

	if (!Svpwm_IsValidDuty(phase_a) || !Svpwm_IsValidDuty(phase_b) ||
		!Svpwm_IsValidDuty(phase_c))
		return false;

	output->phase_a_duty = phase_a;
	output->phase_b_duty = phase_b;
	output->phase_c_duty = phase_c;
	output->sector = sector;
	return true;
}
