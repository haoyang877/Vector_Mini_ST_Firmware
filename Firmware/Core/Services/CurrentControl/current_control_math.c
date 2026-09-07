#include "current_control_math.h"

#include <stddef.h>

#if defined(__CC_ARM)
#pragma O3
#pragma Otime
#endif

#define CURRENT_CONTROL_ONE_BY_SQRT_3 0.57735026919f
#define CURRENT_CONTROL_SQRT_3_BY_2    0.86602540378f

void CurrentControl_Clarke(float phase_a, float phase_b, float phase_c,
    float *alpha, float *beta)
{
    if (alpha == NULL || beta == NULL)
        return;
    *alpha = phase_a;
    *beta = (phase_b - phase_c) * CURRENT_CONTROL_ONE_BY_SQRT_3;
}

void CurrentControl_InverseClarke(float alpha, float beta,
    float *phase_a, float *phase_b, float *phase_c)
{
    if (phase_a == NULL || phase_b == NULL || phase_c == NULL)
        return;
    *phase_a = alpha;
    *phase_b = -0.5f * alpha + CURRENT_CONTROL_SQRT_3_BY_2 * beta;
    *phase_c = -0.5f * alpha - CURRENT_CONTROL_SQRT_3_BY_2 * beta;
}

void CurrentControl_Park(float alpha, float beta, float sine, float cosine,
    float *direct, float *quadrature)
{
    if (direct == NULL || quadrature == NULL)
        return;
    *direct = alpha * cosine + beta * sine;
    *quadrature = -alpha * sine + beta * cosine;
}

void CurrentControl_InversePark(float direct, float quadrature,
    float sine, float cosine, float *alpha, float *beta)
{
    if (alpha == NULL || beta == NULL)
        return;
    *alpha = direct * cosine - quadrature * sine;
    *beta = direct * sine + quadrature * cosine;
}
