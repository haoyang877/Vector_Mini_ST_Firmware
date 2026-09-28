/* 生产正余弦查表的独立精度参考；仅主机测试使用标准库三角函数。 */
#include <assert.h>
#include <float.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "utils.h"
#include "foc_algorithm.h"

static double max_error;
static double max_norm_error;
static double max_rotation_error;
static double max_rotation_drift;
static unsigned samples;

static void check(float angle, double tolerance)
{
    float s, c;
    Math_SinCos(angle, &s, &c);
    double error = fmax(fabs((double)s - sin((double)angle)), fabs((double)c - cos((double)angle)));
    double norm_error = fabs((double)s * s + (double)c * c - 1.0);
    max_error = fmax(max_error, error);
    max_norm_error = fmax(max_norm_error, norm_error);
    assert(error <= tolerance);
    assert(norm_error <= 1e-6);
    assert(fabsf(s) <= 1.000001f && fabsf(c) <= 1.000001f);
    ++samples;
}

static void check_rotations(float step)
{
    float s, c;
    float d = 2.0f, q = 3.0f;
    Math_SinCos(step, &s, &c);
    for (unsigned i = 1; i <= 10000; ++i)
    {
        float next_d = d * c - q * s;
        q = d * s + q * c;
        d = next_d;
        double angle = (double)step * i;
        double expected_d = 2.0 * cos(angle) - 3.0 * sin(angle);
        double expected_q = 2.0 * sin(angle) + 3.0 * cos(angle);
        double error = hypot((double)d - expected_d, (double)q - expected_q);
        double drift = fabs(hypot((double)d, (double)q) / sqrt(13.0) - 1.0);
        max_rotation_error = fmax(max_rotation_error, error);
        max_rotation_drift = fmax(max_rotation_drift, drift);
        if (error > 0.002 || drift > 0.001)
        {
            fprintf(stderr,
                    "rotation step=%.9g count=%u error=%.9g drift=%.9g sinerr=%.9g coserr=%.9g\n",
                    (double)step,
                    i,
                    error,
                    drift,
                    (double)s - sin((double)step),
                    (double)c - cos((double)step));
        }
        assert(error <= 0.002);
        assert(drift <= 0.001);
    }
}

int main(void)
{
    const double pi = 3.14159265358979323846;
    /* 全圈密扫覆盖每个表区间和象限，真值直接由输入 float 的 double 值计算。 */
    for (int i = -524288; i <= 524288; ++i)
    {
        float angle = (float)(i * (2.0 * pi / 524288.0));
        check(angle, 1e-6);
        float d, q, alpha, beta;
        Park_Transform(2.0f, -3.0f, angle, &d, &q);
        Inverse_Park_Transform(d, q, angle, &alpha, &beta);
        assert(fabsf(alpha - 2.0f) < 1e-5f && fabsf(beta + 3.0f) < 1e-5f);
    }
    double single_error = max_error;
    for (int i = -4096; i <= 4096; ++i)
    {
        float knot = (float)(i * (2.0 * pi / 1024.0));
        check(knot, 3e-6);
        check(nextafterf(knot, INFINITY), 3e-6);
        check(nextafterf(knot, -INFINITY), 3e-6);
        check(knot + (float)(pi / 1024.0), 3e-6);
    }
    for (int exponent = -30; exponent <= -2; ++exponent)
    {
        float tiny = powf(10.0f, (float)exponent);
        float s, c;
        Math_SinCos(tiny, &s, &c);
        assert(s > 0.0f && fabs((double)s / sin((double)tiny) - 1.0) < 1e-6);
        Math_SinCos(-tiny, &s, &c);
        assert(s < 0.0f && fabs((double)s / sin(-(double)tiny) - 1.0) < 1e-6);
        check(tiny, 1e-6);
        check(-tiny, 1e-6);
    }
    const float steps[] = {1e-8f, 1e-6f, 1e-5f, 1e-4f, 1e-3f, 1e-2f, .05f, .1f};
    for (unsigned i = 0; i < sizeof(steps) / sizeof(steps[0]); ++i)
    {
        check_rotations(steps[i]);
        check_rotations(-steps[i]);
    }
    const float invalid[] = {NAN, INFINITY, -INFINITY};
    for (unsigned i = 0; i < sizeof(invalid) / sizeof(invalid[0]); ++i)
    {
        float s, c;
        Math_SinCos(invalid[i], &s, &c);
        assert(isnan(s) && isnan(c));
    }
    float s, c;
    Math_SinCos(-0.0f, &s, &c);
    assert(signbit(s) && c == 1.0f);
    Math_SinCos(0.0f, &s, &c);
    assert(!signbit(s) && c == 1.0f);
    /* 跨全部指数的有限输入检查索引安全与单位圆；大角度不要求真实周期精度。 */
    for (uint32_t bits = 1U; bits < 0x7f800000U; bits += 65537U)
    {
        float angle;
        memcpy(&angle, &bits, sizeof(angle));
        for (unsigned sign = 0; sign < 2; ++sign)
        {
            Math_SinCos(sign ? -angle : angle, &s, &c);
            assert(isfinite(s) && isfinite(c));
            assert(fabs((double)s * s + (double)c * c - 1.0) < 1e-6);
        }
    }
    Math_SinCos(FLT_MAX, &s, &c);
    assert(isfinite(s) && isfinite(c));
    printf("PASS sincos: %u angles, single error %.9g, multi error %.9g, norm error %.9g; "
           "rotation vector error %.9g, relative drift %.9g; Park roundtrip and invalids checked\n",
           samples,
           single_error,
           max_error,
           max_norm_error,
           max_rotation_error,
           max_rotation_drift);
    return 0;
}
