/* 真实RL回路仿真；断相与固定器件压降独立于测量算法。 */
/* PHASE_RESISTANCE_IMPLEMENTATION */
#include "foc.h"
#include "control_math.h"
/* SVM_IMPLEMENTATION */
#include <assert.h>
#include <math.h>
#include <stdio.h>

static PhaseResistance
simulate(float a, float b, float c, unsigned open, float drop, unsigned motion)
{
    PhaseResistance state;
    PhaseResistance_Init(&state, 1.905f, 0.001f, 0.0f);
    float resistance[3] = {a, b, c};
    float current[3] = {0};
    unsigned previous = 3U;
    unsigned last_excitation = 0U, motion_ticks = 0U;
    for (unsigned tick = 0; tick < 800000U && !state.done; ++tick)
    {
        float duty[3];
        if (state.excitation != last_excitation)
        {
            last_excitation = state.excitation;
            motion_ticks = 0U;
        }
        float elapsed = (float)motion_ticks++ * 0.00005f;
        float turns = motion == 0U ? 0.0f
                                   : 0.01f * sinf(20.0f * elapsed) *
                                         (motion == 1U ? expf(-4.0f * elapsed) : 1.0f);
        unsigned mask = PhaseResistance_Step(&state, current, 28.0f, turns, 0.00005f, duty);
        if (motion != 0U && elapsed < 0.5f)
        {
            assert(state.samples == 0U);
        }
        if (mask != previous)
        {
            assert(fabsf(current[0]) + fabsf(current[1]) + fabsf(current[2]) < 0.1f);
            previous = mask;
        }
        unsigned first = mask == 3U ? 0U : mask == 6U ? 1U : 2U;
        unsigned second = (first + 1U) % 3U;
        unsigned third = 3U - first - second;
        assert(duty[third] == 0.5f);
        float voltage = (duty[second] - duty[first]) * 28.0f;
        float amperes = current[first];
        if (open & mask)
        {
            amperes = 0.0f;
        }
        else
        {
            float direction = amperes > 0.001f ? 1.0f : amperes < -0.001f ? -1.0f : 0.0f;
            amperes +=
                (voltage - direction * drop - (resistance[first] + resistance[second]) * amperes) *
                0.00005f / 0.002f;
        }
        current[first] = amperes;
        current[second] = -amperes;
        current[third] = 0.0f;
    }
    assert(state.done);
    return state;
}

int main(void)
{
    /* 起始对齐保持慢升流，只有第二档及撤流采用较快斜坡。 */
    PhaseResistance ramps;
    float ramp_duty[3], zero_current[3] = {0};
    PhaseResistance_Init(&ramps, 1.905f, 0.001f, 0.0f);
    PhaseResistance_Step(&ramps, zero_current, 28.0f, 0.0f, 0.00005f, ramp_duty);
    assert(fabsf(ramps.target_a - 0.00010f) < 1e-8f);
    ramps.level = 1U;
    ramps.target_a = 1.0f;
    PhaseResistance_Step(&ramps, zero_current, 28.0f, 0.0f, 0.00005f, ramp_duty);
    assert(fabsf(ramps.target_a - 1.00025f) < 1e-7f);
    ramps.gap = true;
    ramps.target_a = 2.0f;
    PhaseResistance_Step(&ramps, zero_current, 28.0f, 0.0f, 0.00005f, ramp_duty);
    assert(fabsf(ramps.target_a - 1.99950f) < 1e-7f);
    /* 精确二进制边界验证50%与150%均包含，不能只验证中间值。 */
    PhaseResistance edges = {.report = {.nominal_ohm = 2.0f, .line_ohm = {3.0f, 5.0f, 4.0f}}};
    finish(&edges);
    assert(edges.report.result == 1U && edges.report.phase_ohm[0] == 1.0f &&
           edges.report.phase_ohm[2] == 3.0f);

    PhaseResistance normal = simulate(1.905f, 1.905f, 1.905f, 0U, 0.2f, 0U);
    printf("normal result=%u failure=%u A=%f B=%f C=%f\n",
           normal.report.result,
           normal.report.failure,
           normal.report.phase_ohm[0],
           normal.report.phase_ohm[1],
           normal.report.phase_ohm[2]);
    assert(normal.report.result == 1U && normal.report.valid == 63U);
    for (unsigned excitation = 0U; excitation < 6U; ++excitation)
    {
        assert(fabsf(normal.measured_a[excitation][0] - 1.0f) < 0.01f);
        assert(fabsf(normal.measured_a[excitation][1] - 2.0f) < 0.01f);
    }
    /* 最坏正常线阻在2A下仍有电压余量，不把有效高阻误报为超时。 */
    PhaseResistance upper = simulate(2.8f, 2.8f, 2.8f, 0U, 0.3f, 0U);
    assert(upper.report.result == 1U);

    for (unsigned i = 0U; i < 3U; ++i)
    {
        assert(fabsf(normal.report.phase_ohm[i] - 1.905f) < 0.05f);
        PhaseResistance missing = simulate(1.905f, 1.905f, 1.905f, 1U << i, 0.0f, 0U);
        assert(missing.report.result == 2U && missing.report.open_phases == (1U << i));
    }
    PhaseResistance all = simulate(1.905f, 1.905f, 1.905f, 7U, 0.0f, 0U);
    assert(all.report.result == 2U && all.report.open_phases == 7U);
    PhaseResistance low = simulate(0.7f, 1.905f, 1.905f, 0U, 0.2f, 0U);
    assert(low.report.result == 3U && low.report.low_phases == 1U);
    PhaseResistance high = simulate(3.0f, 1.905f, 1.905f, 0U, 0.0f, 0U);
    assert(high.report.failure == 10U && high.report.high_phases == 1U);
    PhaseResistance moving = simulate(1.905f, 1.905f, 1.905f, 0U, 0.2f, 1U);
    assert(moving.report.result == 1U);
    PhaseResistance oscillating = simulate(1.905f, 1.905f, 1.905f, 0U, 0.0f, 2U);
    assert(oscillating.report.failure == 3U);
    float duty[3], current[3] = {7.1f, -7.1f, 0.0f};
    PhaseResistance_Init(&normal, 1.905f, 0.001f, 0.0f);
    PhaseResistance_Step(&normal, current, 28.0f, 0.0f, 0.00005f, duty);
    assert(normal.done && normal.report.failure == 1U && duty[0] == 0.5f);
    current[0] = current[1] = 0.0f;
    PhaseResistance_Init(&normal, 1.905f, 0.001f, 0.0f);
    PhaseResistance_Step(&normal, current, 28.0f, 0.2f, 0.00005f, duty);
    assert(!normal.done && normal.report.failure == 0U);
    PhaseResistance_Init(&normal, 1.905f, 0.001f, 0.0f);
    PhaseResistance_Step(&normal, current, NAN, 0.0f, 0.00005f, duty);
    assert(normal.done && normal.report.failure == 4U);
    PhaseResistance_Init(&ramps, 1.905f, 0.001f, 0.0f);
    for (unsigned tick = 0U; tick < 10000U; ++tick)
    {
        PhaseResistance_Step(&ramps, zero_current, 10.0f, 0.0f, 0.00005f, ramp_duty);
        for (unsigned phase = 0U; phase < 3U; ++phase)
        {
            assert(ramp_duty[phase] >= 0.0f && ramp_duty[phase] <= 1.0f);
        }
        assert(ramps.voltage_v <= 4.5f);
    }
    /* 六个测试方向与既有生产SVM同向，防止仿真和实现共同采用错误极性。 */
    for (unsigned excitation = 0U; excitation < 6U; ++excitation)
    {
        PhaseResistance_Init(&normal, 1.905f, 0.001f, 0.0f);
        normal.excitation = (uint8_t)excitation;
        unsigned first = s_first[excitation / 2U], second = s_second[excitation / 2U];
        float abc[3] = {0};
        float sign = (excitation & 1U) == 0U ? 1.0f : -1.0f;
        abc[first] = 0.01f * sign;
        abc[second] = -0.01f * sign;
        AlphaBetaVector vector = {abc[0], (abc[1] - abc[2]) * ONE_BY_SQRT_3};
        ABCVect svm;
        Svm_SectorJudge(&vector, &svm);
        float reference[3] = {svm.a, svm.b, svm.c};
        PhaseResistance_Step(&normal, current, 28.0f, 0.0f, 0.00005f, duty);
        assert((duty[first] - duty[second]) * (reference[first] - reference[second]) > 0.0f);
    }
    puts("PASS resistance normal, fixed drop, open A/B/C/all, low, high, 7A protection, free rotor "
         "settling, oscillation timeout, SVM polarity");
    return 0;
}
