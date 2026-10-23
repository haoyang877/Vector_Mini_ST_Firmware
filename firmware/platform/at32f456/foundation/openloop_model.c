#include "openloop_model.h"

#include <math.h>
#include <string.h>
#include "../../../motor/math/control_math.h"
#include "../../config/boards/r40_analog.h"

static void midpoint(R40OlModel *model)
{
    for (unsigned i = 0U; i < 3U; ++i)
    {
        model->duty[i] = 0.5f;
    }
}

void R40Ol_Init(R40OlModel *model)
{
    memset(model, 0, sizeof(*model));
    for (unsigned i = 0U; i < 3U; ++i)
    {
        model->low[i] = UINT16_MAX;
    }
    midpoint(model);
}

void R40Ol_Stop(R40OlModel *model, R40OlFault fault)
{
    if (model->state != R40_OL_FAULT)
    {
        model->fault = fault;
    }
    model->state = R40_OL_FAULT;
    midpoint(model);
}

bool R40Ol_Begin(R40OlModel *model, uint32_t now_ms)
{
    if (model->state != R40_OL_READY)
    {
        return false;
    }
    model->state = R40_OL_ARMING;
    model->started_ms = model->heartbeat_ms = now_ms;
    model->running_samples = 0U;
    model->quiet_samples = 0U;
    model->phase_rad = 0.0f;
    return true;
}

void R40Ol_Heartbeat(R40OlModel *model, uint32_t now_ms)
{
    model->heartbeat_ms = now_ms;
}

void R40Ol_Step(
    R40OlModel *model, const uint16_t raw[4], bool adc_ok, bool encoder_ok, uint32_t now_ms)
{
    if (model->state == R40_OL_FAULT)
    {
        return;
    }
    if (!adc_ok || raw == NULL)
    {
        R40Ol_Stop(model, R40_OL_ADC_ERROR);
        return;
    }
    if (!encoder_ok)
    {
        R40Ol_Stop(model, R40_OL_ENCODER_ERROR);
        return;
    }
    for (unsigned i = 0U; i < 4U; ++i)
    {
        if (raw[i] > R40_ADC_MAX_COUNTS * R40_ADC_ACCUMULATIONS)
        {
            R40Ol_Stop(model, R40_OL_ADC_ERROR);
            return;
        }
    }
    const float volts_per_count =
        R40_ADC_REFERENCE_V / (R40_ADC_MAX_COUNTS * R40_ADC_ACCUMULATIONS);
    model->bus_v = raw[3] * volts_per_count * (R40_VBUS_TOP_KOHM + R40_VBUS_BOTTOM_KOHM) /
                   R40_VBUS_BOTTOM_KOHM;
    if (!(model->bus_v >= 20.0f && model->bus_v <= 30.0f))
    {
        R40Ol_Stop(model, R40_OL_BUS_VOLTAGE);
        return;
    }
    if (model->state == R40_OL_CALIBRATING)
    {
        /* 实测初期存在共模零漂，先稳定1秒；此期间仍不允许开始或输出。 */
        if (model->settling_samples < R40_OL_SETTLE_SAMPLES)
        {
            ++model->settling_samples;
            return;
        }
        for (unsigned i = 0U; i < 3U; ++i)
        {
            if (raw[i] < 1638U || raw[i] > 14742U)
            {
                R40Ol_Stop(model, R40_OL_OFFSET_ERROR);
                return;
            }
            model->sums[i] += raw[i];
            if (raw[i] < model->low[i])
            {
                model->low[i] = raw[i];
            }
            if (raw[i] > model->high[i])
            {
                model->high[i] = raw[i];
            }
            if (model->high[i] - model->low[i] > 40U)
            {
                R40Ol_Stop(model, R40_OL_OFFSET_ERROR);
                return;
            }
        }
        if (++model->calibration_samples == R40_OL_CALIBRATION_SAMPLES)
        {
            for (unsigned i = 0U; i < 3U; ++i)
            {
                model->offsets[i] = (float)model->sums[i] / R40_OL_CALIBRATION_SAMPLES;
            }
            model->state = R40_OL_READY;
        }
        return;
    }
    for (unsigned i = 0U; i < 3U; ++i)
    {
        model->current_a[i] = (raw[i] - model->offsets[i]) * volts_per_count /
                              (R40_PHASE_SHUNT_OHM * R40_CURRENT_GAIN);
        float magnitude = fabsf(model->current_a[i]);
        if (magnitude > model->peak_current_a)
        {
            model->peak_current_a = magnitude;
        }
    }
    for (unsigned i = 0U; i < 3U; ++i)
    {
        if (fabsf(model->current_a[i]) > R40_OL_CURRENT_LIMIT_A)
        {
            R40Ol_Stop(model, R40_OL_OVERCURRENT);
            return;
        }
    }
    if (model->state != R40_OL_RUNNING && model->state != R40_OL_ARMING)
    {
        return;
    }
    if ((uint32_t)(now_ms - model->heartbeat_ms) >= R40_OL_LEASE_MS)
    {
        R40Ol_Stop(model, R40_OL_LEASE_EXPIRED);
        return;
    }
    if (model->state == R40_OL_ARMING)
    {
        bool quiet = true;
        for (unsigned i = 0U; i < 3U; ++i)
        {
            quiet = quiet && fabsf(model->current_a[i]) <= 0.10f;
        }
        model->quiet_samples = quiet ? model->quiet_samples + 1U : 0U;
        if (model->quiet_samples >= 200U)
        {
            model->state = R40_OL_RUNNING;
            model->started_ms = now_ms;
        }
        else if ((uint32_t)(now_ms - model->started_ms) >= 500U)
        {
            R40Ol_Stop(model, R40_OL_OFFSET_ERROR);
        }
        return;
    }
    uint32_t elapsed_ms = (uint32_t)(now_ms - model->started_ms);
    if (elapsed_ms >= R40_OL_DURATION_MS || model->running_samples >= R40_OL_SAMPLE_LIMIT)
    {
        model->state = R40_OL_DONE;
        midpoint(model);
        return;
    }
    /* 双重时限：毫秒时基与采样次数任一到期即结束，不能靠停止SysTick延长输出。 */
    ++model->running_samples;
#if defined(R40_CLOSEDLOOP_TEST) && R40_CLOSEDLOOP_TEST
    /* 闭环目标复用准入和停机保护；其PWM由原FOC生成，禁止再计算无用开环波形。 */
    return;
#endif
    float gain = fminf(1.0f, (float)elapsed_ms / R40_OL_RAMP_MS);
    gain = fminf(gain, (float)(R40_OL_DURATION_MS - elapsed_ms) / R40_OL_RAMP_MS);
    model->phase_rad += _2PI * R40_OL_ELECTRICAL_HZ / 20000.0f;
    if (model->phase_rad >= _2PI)
    {
        model->phase_rad -= _2PI;
    }
    float sine, cosine;
    Math_SinCos(model->phase_rad, &sine, &cosine);
    float scale = gain * R40_OL_VOLTAGE_V / model->bus_v;
    model->duty[0] = 0.5f + scale * cosine;
    model->duty[1] = 0.5f + scale * (-0.5f * cosine + SQRT_3_BY_2 * sine);
    model->duty[2] = 0.5f + scale * (-0.5f * cosine - SQRT_3_BY_2 * sine);
    for (unsigned i = 0U; i < 3U; ++i)
    {
        if (!isfinite(model->duty[i]) || model->duty[i] < 0.05f || model->duty[i] > 0.95f)
        {
            R40Ol_Stop(model, R40_OL_OUTPUT_ERROR);
            return;
        }
    }
}
