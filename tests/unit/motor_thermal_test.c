/** @file
 * @brief 无硬件热模型逻辑验证；固定输入不是实机热标定。
 */
#include "motor_thermal.h"
#include <assert.h>
#include <float.h>
#include <stdio.h>
#include <string.h>

static MotorThermalConfig config(void)
{
    MotorThermalConfig c = {
        3.0f, 2.121320344f, 7.0f, 65.98486785f, 120.0f, 0.001f, 20U, 0.8f, 0.7f, 1.0f, 0.5f};
    return c;
}

static void dc(MotorThermalState *s, float a, float b, float c)
{
    unsigned i;
    for (i = 0; i < 20U; ++i)
    {
        assert(MotorThermal_Accumulate(s, a, b, c));
    }
    assert(MotorThermal_Update(s));
}

static void sine(MotorThermalState *s, float peak)
{
    unsigned i;
    for (i = 0; i < 20U; ++i)
    {
        float angle = (float)i * 6.28318530718f / 20.0f;
        assert(MotorThermal_Accumulate(s,
                                       peak * sinf(angle),
                                       peak * sinf(angle - 2.09439510239f),
                                       peak * sinf(angle + 2.09439510239f)));
    }
    assert(MotorThermal_Update(s));
}

static void analytical_cases(void)
{
    const float currents[] = {3, 5, 6, 7, 8};
    unsigned k, n;
    MotorThermalConfig c = config();
    for (k = 0; k < sizeof(currents) / sizeof(currents[0]); ++k)
    {
        MotorThermalState s;
        double expected;
        double trip_time;
        assert(MotorThermal_Init(&s, &c, 0));
        for (n = 0; n < 10000U; ++n)
        {
            sine(&s, currents[k]);
        }
        expected = currents[k] * currents[k] / 9.0 * (1.0 - exp(-10.0 / c.heating_tau_s));
        assert(fabs(s.heat[0] - expected) < .0002);
        assert(fabs(s.heat[1] - s.heat[0]) < .0002);
        if (currents[k] > 3)
        {
            trip_time = -c.heating_tau_s * log(1.0 - 9.0 / (currents[k] * currents[k]));
            assert(MotorThermal_Init(&s, &c, 0));
            for (n = 0; !s.fault_latched && n < 40000U; ++n)
            {
                sine(&s, currents[k]);
            }
            assert(fabs(n * .001 - trip_time) < .025);
            printf("reference %.0f A overall trip %.3f s (no feedback derating)\n",
                   currents[k],
                   n * .001);
        }
        else
        {
            assert(!s.fault_latched);
        }
    }
}

static void hotspot_and_cooling(void)
{
    MotorThermalState s;
    MotorThermalConfig c = config();
    unsigned n;
    float stored;
    assert(MotorThermal_Init(&s, &c, 0));
    for (n = 0; !s.fault_latched && n < 6000U; ++n)
    {
        dc(&s, 8, -4, -4);
    }
    assert(n > 4780U && n < 4840U);
    assert(s.heat[1] > s.heat[0] && s.available_a == 0);
    stored = s.max_heat;
    assert(!MotorThermal_ClearFault(&s) && s.max_heat == stored);
    for (n = 0; n < 6000U; ++n)
    {
        dc(&s, 8, -4, -4);
    }
    assert(s.max_heat > 1.5f); /* 故障显示不能丢弃超限热量。 */
    for (n = 0; s.max_heat > .5f && n < 400000U; ++n)
    {
        dc(&s, 0, 0, 0);
    }
    assert(s.max_heat <= .5f && s.fault_latched && s.available_a == 0);
    stored = s.max_heat;
    assert(MotorThermal_ClearFault(&s) && !s.fault_latched && s.max_heat == stored);
    assert(s.available_a == 7);
}

static void hysteresis_and_pulses(void)
{
    MotorThermalState warm, cold;
    MotorThermalConfig c = config();
    unsigned n;
    assert(MotorThermal_Init(&warm, &c, .81f));
    assert(warm.derating && warm.available_a < 7);
    while (warm.max_heat > .75f)
    {
        dc(&warm, 0, 0, 0);
    }
    assert(warm.derating);
    while (warm.max_heat > .70f)
    {
        dc(&warm, 0, 0, 0);
    }
    assert(!warm.derating);
    assert(MotorThermal_Init(&warm, &c, 0));
    for (n = 0; n < 1000U; ++n)
    {
        dc(&warm, 7, -3.5f, -3.5f);
    }
    for (n = 0; n < 100U; ++n)
    {
        dc(&warm, 0, 0, 0);
    }
    assert(MotorThermal_Init(&cold, &c, 0));
    for (n = 0; n < 1000U; ++n)
    {
        dc(&warm, 7, -3.5f, -3.5f);
        dc(&cold, 7, -3.5f, -3.5f);
    }
    assert(warm.max_heat > cold.max_heat + .1f);
}

static void invalid_and_vector(void)
{
    MotorThermalState s;
    MotorThermalConfig c = config();
    float d = 6, q = 8;
    unsigned i;
    assert(MotorThermal_LimitVector(5, &d, &q));
    assert(fabsf(d - 3) < .00001f && fabsf(q - 4) < .00001f);
    assert(MotorThermal_ResolveLimit(7, 2, true) == 2);
    assert(MotorThermal_ResolveLimit(7, 2, false) == 7);
    assert(MotorThermal_ResolveLimit(NAN, 2, false) == 0);
    d = NAN;
    q = 5;
    assert(!MotorThermal_LimitVector(5, &d, &q) && d == 0 && q == 0);
    d = 1;
    q = 2;
    assert(MotorThermal_LimitVector(0, &d, &q) && d == 0 && q == 0);
    assert(MotorThermal_Init(&s, &c, .25f));
    assert(!MotorThermal_Update(&s) && !s.valid && s.available_a == 0 && s.heat[0] == .25f);
    assert(!MotorThermal_ClearFault(&s));
    assert(MotorThermal_Init(&s, &c, .25f));
    assert(!MotorThermal_Accumulate(&s, NAN, 0, 0) && !s.valid && s.max_heat == .25f);
    assert(MotorThermal_Init(&s, &c, 0));
    for (i = 0; i < 20U; ++i)
    {
        assert(MotorThermal_Accumulate(&s, 0, 0, 0));
    }
    assert(!MotorThermal_Accumulate(&s, 0, 0, 0) && !s.valid);
    c.continuous_vector_a = FLT_MIN;
    assert(!MotorThermal_Init(&s, &c, 0));
    c = config();
    c.cooling_tau_s = 1;
    assert(!MotorThermal_Init(&s, &c, 0));
    assert(!MotorThermal_Init(NULL, &c, 0));
}

int main(void)
{
    analytical_cases();
    hotspot_and_cooling();
    hysteresis_and_pulses();
    invalid_and_vector();
    puts("PASS thermal: analytic curves, signed-square windows, phase hotspot, retained heat, "
         "cooling latch, hysteresis, invalid inputs and d/q vector cap");
    return 0;
}
