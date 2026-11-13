/* 真实RL回路仿真；断相与固定器件压降独立于测量算法。 */
/* PHASE_RESISTANCE_IMPLEMENTATION */
#include <assert.h>
#include <math.h>
#include <stdio.h>

static PhaseResistance simulate(float a, float b, float c, unsigned open, float drop)
{
    PhaseResistance state;
    PhaseResistance_Init(&state, 1.905f, 0.001f, 0.0f);
    float resistance[3] = {a, b, c};
    float current[3] = {0};
    unsigned previous = 3U;
    for (unsigned tick = 0; tick < 40000U && !state.done; ++tick)
    {
        float duty[3];
        unsigned mask = PhaseResistance_Step(&state, current, 28.0f, 0.0f, 0.00005f, duty);
        if (mask != previous)
        {
            assert(fabsf(current[0]) + fabsf(current[1]) + fabsf(current[2]) < 0.1f);
            previous = mask;
        }
        unsigned first = mask == 3U ? 0U : mask == 6U ? 1U : 2U;
        unsigned second = (first + 1U) % 3U;
        unsigned third = 3U - first - second;
        assert(duty[third] == 0.5f);
        float voltage = (duty[first] - duty[second]) * 28.0f;
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
    /* 精确二进制边界验证50%与150%均包含，不能只验证中间值。 */
    PhaseResistance edges = {.report = {.nominal_ohm = 2.0f, .line_ohm = {3.0f, 5.0f, 4.0f}}};
    finish(&edges);
    assert(edges.report.result == 1U && edges.report.phase_ohm[0] == 1.0f &&
           edges.report.phase_ohm[2] == 3.0f);

    PhaseResistance normal = simulate(1.905f, 1.905f, 1.905f, 0U, 0.2f);
    printf("normal result=%u failure=%u A=%f B=%f C=%f\n",
           normal.report.result,
           normal.report.failure,
           normal.report.phase_ohm[0],
           normal.report.phase_ohm[1],
           normal.report.phase_ohm[2]);
    assert(normal.report.result == 1U && normal.report.valid == 63U);
    for (unsigned i = 0U; i < 3U; ++i)
    {
        assert(fabsf(normal.report.phase_ohm[i] - 1.905f) < 0.05f);
        PhaseResistance missing = simulate(1.905f, 1.905f, 1.905f, 1U << i, 0.0f);
        assert(missing.report.result == 2U && missing.report.open_phases == (1U << i));
    }
    PhaseResistance all = simulate(1.905f, 1.905f, 1.905f, 7U, 0.0f);
    assert(all.report.result == 2U && all.report.open_phases == 7U);
    PhaseResistance low = simulate(0.7f, 1.905f, 1.905f, 0U, 0.2f);
    assert(low.report.result == 3U && low.report.low_phases == 1U);
    PhaseResistance high = simulate(3.0f, 1.905f, 1.905f, 0U, 0.0f);
    assert(high.report.failure == 10U && high.report.high_phases == 1U);
    float duty[3], current[3] = {1.6f, -1.6f, 0.0f};
    PhaseResistance_Init(&normal, 1.905f, 0.001f, 0.0f);
    PhaseResistance_Step(&normal, current, 28.0f, 0.0f, 0.00005f, duty);
    assert(normal.done && normal.report.failure == 1U && duty[0] == 0.5f);
    current[0] = current[1] = 0.0f;
    PhaseResistance_Init(&normal, 1.905f, 0.001f, 0.0f);
    PhaseResistance_Step(&normal, current, 28.0f, 0.002f, 0.00005f, duty);
    assert(normal.done && normal.report.failure == 2U);
    PhaseResistance_Init(&normal, 1.905f, 0.001f, 0.0f);
    PhaseResistance_Step(&normal, current, 26.9f, 0.0f, 0.00005f, duty);
    assert(normal.done && normal.report.failure == 7U);
    puts("PASS resistance normal, fixed drop, open A/B/C/all, low, high, current, motion, supply");
    return 0;
}
