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

/* 固定三相展开前的独立基准；保留失败相之前的逐相累加顺序。 */
static bool legacy_accumulate(MotorThermalState *state, float ia_a, float ib_a, float ic_a)
{
    float current[3];
    unsigned i;
    if (state == NULL || !state->valid)
    {
        return false;
    }
    current[0] = ia_a;
    current[1] = ib_a;
    current[2] = ic_a;
    if (state->sample_count >= state->config.samples_per_update)
    {
        MotorThermal_Invalidate(state);
        return false;
    }
    for (i = 0U; i < 3U; ++i)
    {
        if (!(fabsf(current[i]) <= 100.0f))
        {
            MotorThermal_Invalidate(state);
            return false;
        }
        state->square_sum_a2[i] += current[i] * current[i];
    }
    ++state->sample_count;
    return true;
}

static void optimized_accumulate_equivalence(void)
{
    const uint32_t bits[] = {0U, 0x80000000U, 1U, 0x80000001U, 0x007fffffU,
        0x00800000U, 0x42c7ffffU, 0x42c80000U, 0x42c80001U,
        0xc2c7ffffU, 0xc2c80000U, 0xc2c80001U, 0x7f7fffffU, 0xff7fffffU,
        0x7f800000U, 0xff800000U, 0x7f800001U, 0xff800001U, 0x7fc12345U, 0xffc12345U};
    MotorThermalConfig c = config();
    unsigned checked = 0U;
    for (unsigned channel = 0U; channel < 3U; ++channel)
    {
        for (unsigned value = 0U; value < sizeof(bits) / sizeof(bits[0]); ++value)
        {
            for (unsigned admission = 0U; admission < 3U; ++admission)
            {
                MotorThermalState actual, before;
                float currents[3] = {1.25f, -2.5f, 3.75f};
                assert(MotorThermal_Init(&actual, &c, .81f));
                actual.sample_count = admission == 1U ? c.samples_per_update : 7U;
                actual.valid = admission != 2U;
                actual.square_sum_a2[0] = .125f;
                actual.square_sum_a2[1] = 4.0f;
                actual.square_sum_a2[2] = 8.0f;
                memcpy(&currents[channel], &bits[value], sizeof(float));
                before = actual;
                assert(MotorThermal_Accumulate(&actual, currents[0], currents[1], currents[2]) ==
                    legacy_accumulate(&before, currents[0], currents[1], currents[2]));
                assert(memcmp(&actual, &before, sizeof(actual)) == 0);
                ++checked;
            }
        }
    }
    uint32_t random = 0x13579bdfU;
    for (unsigned scenario = 0U; scenario < 100000U; ++scenario)
    {
        MotorThermalState actual, before;
        float currents[3];
        assert(MotorThermal_Init(&actual, &c, .81f));
        actual.sample_count = scenario % 22U;
        for (unsigned channel = 0U; channel < 3U; ++channel)
        {
            random = random * 1664525U + 1013904223U;
            memcpy(&currents[channel], &random, sizeof(float));
            if (scenario % 4U != channel)
            {
                currents[channel] = (float)(int32_t)(random % 20001U) * .01f - 100.0f;
            }
            actual.square_sum_a2[channel] = (float)(random % 10000U) * .1f;
        }
        before = actual;
        assert(MotorThermal_Accumulate(&actual, currents[0], currents[1], currents[2]) ==
            legacy_accumulate(&before, currents[0], currents[1], currents[2]));
        assert(memcmp(&actual, &before, sizeof(actual)) == 0);
        ++checked;
    }
    assert(!MotorThermal_Accumulate(NULL, 1.0f, NAN, INFINITY));
    assert(!legacy_accumulate(NULL, 1.0f, NAN, INFINITY));
    printf("PASS thermal accumulate bitwise equivalence: %u states, per-phase invalid partial sums, "
        "signed zero/subnormal/100A boundary/sNaN/qNaN/Inf, full windows and invalid/null states\n", checked);
}

/* 优化前的更新/降额路径：保持独立表达式作为逐位基准，不参与产品编译。 */
static void legacy_limit(MotorThermalState *state)
{
    float sustainable_a, weight;
    if (!state->valid || state->fault_latched)
    {
        state->available_a = 0.0f;
        return;
    }
    if (state->max_heat >= state->config.derating_enter_heat)
    {
        state->derating = true;
    }
    else if (state->max_heat <= state->config.derating_exit_heat)
    {
        state->derating = false;
    }
    if (!state->derating)
    {
        state->available_a = state->config.peak_a;
        return;
    }
    sustainable_a = state->config.continuous_vector_a;
    if (fmaxf(state->heat[1], fmaxf(state->heat[2], state->heat[3])) >= state->heat[0])
    {
        sustainable_a = fminf(sustainable_a, state->config.continuous_phase_rms_a);
    }
    weight = fminf(1.0f, fmaxf(0.0f, (state->max_heat - state->config.derating_enter_heat) /
        (state->config.trip_heat - state->config.derating_enter_heat)));
    state->available_a = state->config.peak_a + weight * (sustainable_a - state->config.peak_a);
}

static bool legacy_update(MotorThermalState *state)
{
    float q[4], phase_denominator, samples;
    unsigned i;
    if (state == NULL || !state->valid || state->sample_count != state->config.samples_per_update)
    {
        MotorThermal_Invalidate(state);
        return false;
    }
    samples = (float)state->sample_count;
    phase_denominator = samples * state->config.continuous_phase_rms_a *
        state->config.continuous_phase_rms_a;
    q[0] = (2.0f / 3.0f) *
        (state->square_sum_a2[0] + state->square_sum_a2[1] + state->square_sum_a2[2]) /
        (samples * state->config.continuous_vector_a * state->config.continuous_vector_a);
    state->max_heat = 0.0f;
    for (i = 0U; i < 3U; ++i)
    {
        q[i + 1U] = state->square_sum_a2[i] / phase_denominator;
    }
    for (i = 0U; i < 4U; ++i)
    {
        float alpha = q[i] >= state->heat[i] ? state->heating_alpha : state->cooling_alpha;
        state->heat[i] += alpha * (q[i] - state->heat[i]);
        if (!isfinite(state->heat[i]) || state->heat[i] < 0.0f)
        {
            MotorThermal_Invalidate(state);
            return false;
        }
        if (state->heat[i] > state->max_heat)
        {
            state->max_heat = state->heat[i];
        }
    }
    memset(state->square_sum_a2, 0, sizeof(state->square_sum_a2));
    state->sample_count = 0U;
    if (state->max_heat >= state->config.trip_heat)
    {
        state->fault_latched = true;
    }
    legacy_limit(state);
    return true;
}

static void optimized_update_equivalence(void)
{
    const float vectors[] = {.01f, .1f, 1.0f, 3.0f, 10.0f, 99.0f};
    const float taus[] = {.1f, 1.0f, 65.98486785f, 86400.0f};
    const float initial[] = {0.0f, .7f, .8f, .81f, .99f, 1.0f, 2.0f};
    unsigned checked = 0U;
    for (unsigned scenario = 0U; scenario < 48U; ++scenario)
    {
        MotorThermalState actual, before;
        MotorThermalConfig c = config();
        c.continuous_vector_a = vectors[scenario % 6U];
        c.continuous_phase_rms_a = c.continuous_vector_a * .707106781f;
        c.peak_a = 100.0f;
        c.heating_tau_s = taus[scenario % 4U];
        c.cooling_tau_s = fmaxf(120.0f, c.heating_tau_s);
        c.samples_per_update = scenario % 3U == 0U ? 1U : 20U;
        assert(MotorThermal_Init(&actual, &c, initial[scenario % 7U]));
        before = actual;
        for (unsigned tick = 0U; tick < 6000U; ++tick)
        {
            float a = tick % 3000U < 1000U ? 8.0f : 0.0f;
            float b = scenario & 1U ? -a : -.5f * a;
            float d = scenario & 1U ? 0.0f : -.5f * a;
            for (unsigned sample = 0U; sample < c.samples_per_update; ++sample)
            {
                assert(MotorThermal_Accumulate(&actual, a, b, d));
                assert(legacy_accumulate(&before, a, b, d));
            }
            assert(MotorThermal_Update(&actual) == legacy_update(&before));
            assert(memcmp(&actual, &before, sizeof(actual)) == 0);
            if (actual.max_heat <= c.clear_heat)
            {
                assert(MotorThermal_ClearFault(&actual));
                before.fault_latched = false;
                legacy_limit(&before);
                assert(memcmp(&actual, &before, sizeof(actual)) == 0);
            }
            ++checked;
        }
    }
    /* 每个通道非有限/负值时，中止位置、部分max_heat与故障输出必须保持原样。 */
    const float invalid[] = {NAN, INFINITY, -INFINITY, -1.0f, FLT_MAX};
    for (unsigned field = 0U; field < 7U; ++field)
    {
        for (unsigned bad = 0U; bad < sizeof(invalid) / sizeof(invalid[0]); ++bad)
        {
            MotorThermalState actual, before;
            MotorThermalConfig c = config();
            assert(MotorThermal_Init(&actual, &c, .81f));
            actual.sample_count = c.samples_per_update;
            actual.square_sum_a2[0] = actual.square_sum_a2[1] = actual.square_sum_a2[2] = 1.0f;
            if (field < 4U)
            {
                actual.heat[field] = invalid[bad];
            }
            else
            {
                actual.square_sum_a2[field - 4U] = invalid[bad];
            }
            before = actual;
            assert(MotorThermal_Update(&actual) == legacy_update(&before));
            assert(memcmp(&actual, &before, sizeof(actual)) == 0);
            ++checked;
        }
    }
    printf("PASS thermal update bitwise equivalence: %u windows, heating/cooling, hotspot, "
        "derating/trip/clear thresholds and invalid partial updates\n", checked);
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

/* 覆盖最小/最大时间常数，验证节省指数库后的离散系数精度。 */
static void alpha_precision(void)
{
    unsigned i;
    for (i = 0U; i <= 100000U; ++i)
    {
        float tau_s = (float)exp(log(0.1) + (log(86400.0) - log(0.1)) * i / 100000.0);
        float actual = MotorThermal_Alpha(0.001f, tau_s);
        double expected = -expm1(-(double)0.001f / (double)tau_s);
        assert(actual > 0.0f && fabs((double)actual - expected) / expected < 2.0e-7);
    }
}

int main(void)
{
    optimized_accumulate_equivalence();
    optimized_update_equivalence();
    alpha_precision();
    analytical_cases();
    hotspot_and_cooling();
    hysteresis_and_pulses();
    invalid_and_vector();
    puts("PASS thermal: analytic curves, signed-square windows, phase hotspot, retained heat, "
         "cooling latch, hysteresis, invalid inputs and d/q vector cap");
    return 0;
}
