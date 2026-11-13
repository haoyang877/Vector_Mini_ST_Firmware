#include "phase_resistance.h"
#include <math.h>
#include <string.h>

static const uint8_t s_first[3] = {0U, 1U, 2U};
static const uint8_t s_second[3] = {1U, 2U, 0U};

/* 多条失败路径共享结算，避免快拍代码重复展开。 */
static __attribute__((noinline)) void finish(PhaseResistance *state)
{
    PhaseResistanceReport *report = &state->report;
    state->done = true;
    if (report->failure != 0U)
    {
        return;
    }
    if (report->open_lines != 0U)
    {
        report->result = 2U;
        for (unsigned i = 0U; i < 3U; ++i)
        {
            unsigned adjacent = (1U << i) | (1U << ((i + 2U) % 3U));
            if ((report->open_lines & adjacent) == adjacent)
            {
                report->open_phases |= (uint8_t)(1U << i);
            }
        }
        return;
    }
    for (unsigned i = 0U; i < 3U; ++i)
    {
        float resistance = 0.5f * (report->line_ohm[i] + report->line_ohm[(i + 2U) % 3U] -
                                   report->line_ohm[(i + 1U) % 3U]);
        report->phase_ohm[i] = resistance;
        report->valid |= (uint8_t)(1U << i);
        if (resistance < 0.5f * report->nominal_ohm)
        {
            report->low_phases |= (uint8_t)(1U << i);
        }
        if (resistance > 1.5f * report->nominal_ohm)
        {
            report->high_phases |= (uint8_t)(1U << i);
        }
    }
    report->result = report->low_phases != 0U ? 3U : (report->high_phases == 0U ? 1U : 0U);
    report->failure = report->high_phases != 0U ? 10U : 0U;
}

void PhaseResistance_Init(PhaseResistance *state,
                          float nominal_ohm,
                          float inductance_h,
                          float turns)
{
    memset(state, 0, sizeof(*state));
    state->report.nominal_ohm = nominal_ohm;
    state->nominal_inductance_h = inductance_h;
    state->origin_turns = turns;
}

#if defined(__clang__)
__attribute__((minsize))
#endif
uint8_t PhaseResistance_Step(PhaseResistance *state,
                             const float current_a[3],
                             float bus_v,
                             float turns,
                             float period_s,
                             float duty[3])
{
    unsigned line = state->excitation / 2U;
    uint8_t first = s_first[line], second = s_second[line];
    uint8_t mask = (uint8_t)((1U << first) | (1U << second));
    float maximum_a = 0.0f;
    duty[0] = duty[1] = duty[2] = 0.5f;
    if (state->done)
    {
        return mask;
    }
    for (unsigned i = 0U; i < 3U; ++i)
    {
        if (!isfinite(current_a[i]))
        {
            state->report.failure = 4U;
        }
        maximum_a = fmaxf(maximum_a, fabsf(current_a[i]));
    }
    float displacement = turns - state->origin_turns;
    displacement -= roundf(displacement);
    if (!isfinite(turns))
    {
        state->report.failure = 4U;
    }
    else if (fabsf(displacement) > 0.5f / 360.0f)
    {
        state->report.failure = 2U;
    }
    if (!(bus_v >= 27.0f && bus_v <= 29.0f))
    {
        state->report.failure = 7U;
    }
    if (maximum_a > 1.5f)
    {
        state->report.failure = 1U;
    }
    if (state->report.failure != 0U)
    {
        finish(state);
        return mask;
    }
    ++state->ticks;
    if (state->gap)
    {
        state->stable_ticks = maximum_a < 0.03f ? state->stable_ticks + 1U : 0U;
        if ((float)state->stable_ticks * period_s >= 0.002f)
        {
            if (++state->excitation == 6U)
            {
                /* 终态后不再索引激励表。 */
                state->excitation = 5U;
                finish(state);
            }
            else
            {
                state->gap = false;
                state->ticks = state->stable_ticks = state->samples = 0U;
                state->level = 0U;
                state->integral_v = state->target_a = state->voltage_v = 0.0f;
                state->sum_v = state->sum_a = 0.0f;
            }
        }
        else if ((float)state->ticks * period_s > 0.030f)
        {
            state->report.failure = 3U;
            finish(state);
        }
        return mask;
    }
    float sign = (state->excitation & 1U) == 0U ? 1.0f : -1.0f;
    float measured_a = sign * 0.5f * (current_a[first] - current_a[second]);
    unsigned third = 3U - first - second;
    if (fabsf(current_a[third]) > 0.15f || fabsf(current_a[first] + current_a[second]) > 0.2f)
    {
        state->report.failure = 6U;
        finish(state);
        return mask;
    }
    float target_a = state->level == 0U ? 0.5f : 1.0f;
    state->target_a = fminf(target_a, state->target_a + 100.0f * period_s);
    float error_a = state->target_a - measured_a;
    float resistance = 2.0f * state->report.nominal_ohm;
    float proportional = 2.0f * state->nominal_inductance_h / 0.002f * error_a;
    float output = resistance * state->target_a + proportional + state->integral_v;
    if ((output < 6.5f || error_a < 0.0f) && (output > 0.0f || error_a > 0.0f))
    {
        state->integral_v += resistance / 0.002f * error_a * period_s;
    }
    bool stable = state->target_a == target_a && fabsf(error_a) < 0.035f;
    if (!stable)
    {
        state->stable_ticks = state->samples = 0U;
        state->sum_v = state->sum_a = 0.0f;
    }
    else if ((float)++state->stable_ticks * period_s > 0.010f)
    {
        /* 电流与上一拍实际下发电压配对，不用尚未生效的新命令。 */
        state->sum_v += state->voltage_v;
        state->sum_a += measured_a;
        if ((float)++state->samples * period_s >= 0.008f)
        {
            state->points_v[state->level] = state->sum_v / (float)state->samples;
            state->points_a[state->level] = state->sum_a / (float)state->samples;
            if (state->level == 0U)
            {
                state->level = 1U;
                state->ticks = state->stable_ticks = state->samples = 0U;
                state->sum_v = state->sum_a = 0.0f;
            }
            else
            {
                float delta_a = state->points_a[1] - state->points_a[0];
                float ohm = (state->points_v[1] - state->points_v[0]) / delta_a;
                if (!(delta_a > 0.2f) || !isfinite(ohm) || ohm < 0.0f)
                {
                    state->report.failure = 4U;
                    finish(state);
                }
                else if ((state->excitation & 1U) == 0U)
                {
                    state->positive_ohm = ohm;
                }
                else
                {
                    state->report.line_ohm[line] = 0.5f * (state->positive_ohm + ohm);
                    state->report.valid |= (uint8_t)(8U << line);
                    if (fabsf(state->positive_ohm - ohm) > 0.5f * state->report.nominal_ohm)
                    {
                        state->report.failure = 9U;
                        finish(state);
                    }
                }
                state->gap = true;
                state->ticks = state->stable_ticks = 0U;
            }
        }
    }
    if ((float)state->ticks * period_s > 0.100f && !state->gap)
    {
        if (measured_a < 0.1f && state->voltage_v > 6.3f)
        {
            state->report.open_lines |= (uint8_t)(1U << line);
            state->excitation |= 1U;
            state->gap = true;
            state->ticks = state->stable_ticks = 0U;
        }
        else
        {
            state->report.failure = 3U;
            finish(state);
        }
    }
    state->voltage_v = state->gap || state->done ? 0.0f : fminf(6.5f, fmaxf(0.0f, output));
    duty[first] += sign * state->voltage_v / (2.0f * bus_v);
    duty[second] -= sign * state->voltage_v / (2.0f * bus_v);
    return mask;
}
