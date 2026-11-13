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

static void reset_window(PhaseResistance *state)
{
    state->window_position = state->window_min = state->window_max = 0.0f;
    state->window_ticks = state->stable_ticks = state->samples = 0U;
    state->sum_v = state->sum_a = 0.0f;
    state->window_current_stable = true;
}

void PhaseResistance_Init(PhaseResistance *state,
                          float nominal_ohm,
                          float inductance_h,
                          float turns)
{
    memset(state, 0, sizeof(*state));
    state->report.nominal_ohm = nominal_ohm;
    state->nominal_inductance_h = inductance_h;
    state->last_turns = turns;
    reset_window(state);
}

/* 电流与转子共同稳定后才形成采样证据；允许此前的任意对齐位移。 */
static bool
ready_to_measure(PhaseResistance *state, float delta_turns, bool current_stable, float period_s)
{
    state->window_position += delta_turns;
    state->window_min = fminf(state->window_min, state->window_position);
    state->window_max = fmaxf(state->window_max, state->window_position);
    state->window_current_stable = state->window_current_stable && current_stable;
    ++state->window_ticks;
    bool still = state->window_max - state->window_min <= 0.1f / 360.0f;
    if (!current_stable || !still)
    {
        state->stable_ticks = state->samples = 0U;
        state->sum_v = state->sum_a = 0.0f;
    }
    if ((float)state->window_ticks * period_s >= 0.100f)
    {
        state->stable_ticks =
            still && state->window_current_stable ? state->stable_ticks + state->window_ticks : 0U;
        state->window_ticks = 0U;
        state->window_position = state->window_min = state->window_max = 0.0f;
        state->window_current_stable = true;
    }
    return current_stable && still && (float)state->stable_ticks * period_s >= 0.200f;
}

static void complete_point(PhaseResistance *state)
{
    unsigned level = state->level;
    unsigned excitation = state->excitation;
    state->points_v[level] = state->sum_v / (float)state->samples;
    state->points_a[level] = state->sum_a / (float)state->samples;
    state->measured_v[excitation][level] = state->points_v[level];
    state->measured_a[excitation][level] = state->points_a[level];
    if (level == 0U)
    {
        state->level = 1U;
        reset_window(state);
        return;
    }
    float delta_a = state->points_a[1] - state->points_a[0];
    float ohm = (state->points_v[1] - state->points_v[0]) / delta_a;
    if (!(delta_a > 0.2f) || !isfinite(ohm) || ohm < 0.0f)
    {
        state->report.failure = 4U;
        finish(state);
        return;
    }
    if ((excitation & 1U) == 0U)
    {
        state->positive_ohm = ohm;
    }
    else
    {
        state->report.line_ohm[excitation / 2U] = 0.5f * (state->positive_ohm + ohm);
        state->report.valid |= (uint8_t)(8U << (excitation / 2U));
    }
    state->gap = true;
    state->zero_ticks = 0U;
}

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
    duty[0] = duty[1] = duty[2] = 0.5f;
    if (state->done)
    {
        return mask;
    }
    float maximum_a = 0.0f;
    for (unsigned i = 0U; i < 3U; ++i)
    {
        if (!isfinite(current_a[i]))
        {
            state->report.failure = 4U;
        }
        maximum_a = fmaxf(maximum_a, fabsf(current_a[i]));
    }
    state->peak_current_a = fmaxf(state->peak_current_a, maximum_a);
    if (!isfinite(turns) || !isfinite(bus_v) || !(bus_v > 0.0f))
    {
        state->report.failure = 4U;
    }
    if (maximum_a > 7.0f)
    {
        state->report.failure = 1U;
    }
    if (state->report.failure != 0U)
    {
        state->voltage_v = 0.0f;
        finish(state);
        return mask;
    }
    float delta_turns = turns - state->last_turns;
    delta_turns -= roundf(delta_turns);
    state->last_turns = turns;
    ++state->ticks;
    float sign = (state->excitation & 1U) == 0U ? 1.0f : -1.0f;
    float measured_a = sign * 0.5f * (current_a[first] - current_a[second]);
    float desired_a = state->gap ? 0.0f : (state->level == 0U ? 0.5f : 1.0f);
    /* 新磁场缓升流对齐；已对齐后的同向升档和撤流加快，仍保留稳定判定。 */
    float ramp_a = (state->gap || state->level == 1U ? 5.0f : 1.0f) * period_s;
    if (state->target_a < desired_a)
    {
        state->target_a = fminf(desired_a, state->target_a + ramp_a);
    }
    else
    {
        state->target_a = fmaxf(desired_a, state->target_a - ramp_a);
    }
    float error_a = state->target_a - measured_a;
    float resistance = 2.0f * state->report.nominal_ohm;
    float output = resistance * state->target_a +
                   2.0f * state->nominal_inductance_h / 0.002f * error_a + state->integral_v;
    if ((output < 6.5f || error_a < 0.0f) && (output > 0.0f || error_a > 0.0f))
    {
        state->integral_v += resistance / 0.002f * error_a * period_s;
    }
    if (state->gap)
    {
        state->zero_ticks =
            state->target_a == 0.0f && maximum_a < 0.05f ? state->zero_ticks + 1U : 0U;
        if ((float)state->zero_ticks * period_s >= 0.002f)
        {
            output = 0.0f;
            if (state->excitation == 5U)
            {
                finish(state);
            }
            else
            {
                ++state->excitation;
                state->gap = false;
                state->level = 0U;
                state->ticks = state->zero_ticks = 0U;
                state->target_a = state->integral_v = 0.0f;
                reset_window(state);
            }
        }
    }
    else if (ready_to_measure(state,
                              delta_turns,
                              state->target_a == desired_a && fabsf(error_a) <= 0.05f,
                              period_s))
    {
        /* 实测电流对应上一拍实际施加的电压。 */
        state->sum_v += state->voltage_v;
        state->sum_a += measured_a;
        if ((float)++state->samples * period_s >= 0.050f)
        {
            complete_point(state);
        }
    }
    if ((float)state->ticks * period_s > 4.0f)
    {
        if (!state->gap && fabsf(measured_a) < 0.1f && state->voltage_v > 6.3f)
        {
            state->report.open_lines |= (uint8_t)(1U << line);
            state->excitation |= 1U;
            state->gap = true;
            state->ticks = state->zero_ticks = 0U;
        }
        else
        {
            state->report.failure = 3U;
            finish(state);
        }
    }
    state->voltage_v = state->done ? 0.0f : fminf(6.5f, fmaxf(0.0f, output));
    /* 与既有SVM和三相采样符号一致：正相电压对应较低CCR占空比。 */
    duty[first] -= sign * state->voltage_v / (2.0f * bus_v);
    duty[second] += sign * state->voltage_v / (2.0f * bus_v);
    return mask;
}
