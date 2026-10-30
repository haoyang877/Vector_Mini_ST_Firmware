/* 验证开环固定时限、零流准入及故障锁存；纯模型测试不访问硬件。 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "../../bench/at32f456/openloop_model.h"

static void ready(AT32OlModel *model)
{
    const uint16_t raw[4] = {7631U, 7635U, 7633U, 4120U};
    AT32Ol_Init(model);
    assert(!AT32Ol_Begin(model, 0U));
    for (unsigned i = 0U; i < AT32_OL_SETTLE_SAMPLES + AT32_OL_CALIBRATION_SAMPLES; ++i)
    {
        AT32Ol_Step(model, raw, true, true, i / 20U);
        assert(model->duty[0] == 0.5f);
    }
    assert(model->state == AT32_OL_READY && model->offsets[0] == 7631.0f);
}

int main(void)
{
    AT32OlModel model;
    uint16_t raw[4] = {7631U, 7635U, 7633U, 4120U};
    ready(&model);
    assert(AT32Ol_Begin(&model, 200U));
    raw[0] += 44U;
    AT32Ol_Step(&model, raw, true, true, 200U);
    assert(model.state == AT32_OL_ARMING && model.quiet_samples == 0U);
    raw[0] -= 44U;
    for (unsigned i = 0U; i < 200U; ++i)
    {
        AT32Ol_Step(&model, raw, true, true, 200U);
    }
    assert(model.state == AT32_OL_RUNNING);
    assert(!AT32Ol_Begin(&model, 200U));
    float maximum = 0.0f;
    double phase_advance = 0.0;
    uint32_t phase_samples = 0U;
    for (unsigned tick = 0U; tick <= AT32_OL_SAMPLE_LIMIT; ++tick)
    {
        uint32_t now = 200U + tick / 20U;
        AT32Ol_Heartbeat(&model, now);
        float previous_phase = model.phase_rad;
        AT32Ol_Step(&model, raw, true, true, now);
        if (model.state == AT32_OL_RUNNING)
        {
            double advance = (double)model.phase_rad - previous_phase;
            phase_advance += advance < 0.0 ? advance + 6.283185307179586 : advance;
            ++phase_samples;
        }
        assert(fabsf(model.duty[0] + model.duty[1] + model.duty[2] - 1.5f) < 1e-6f);
        for (unsigned phase = 0U; phase < 3U; ++phase)
        {
            float volts = fabsf((model.duty[phase] - 0.5f) * model.bus_v);
            assert(volts <= AT32_OL_VOLTAGE_V + 1e-5f);
            if (volts > maximum)
            {
                maximum = volts;
            }
        }
    }
    double measured_hz = phase_advance * 20000.0 / (phase_samples * 6.283185307179586);
    assert(fabs(measured_hz / AT32_OL_ELECTRICAL_HZ - 1.0) < 0.001);
    assert(maximum > AT32_OL_VOLTAGE_V * 0.99f && model.state == AT32_OL_DONE);
    assert(model.duty[0] == 0.5f && !AT32Ol_Begin(&model, 1201U));
    ready(&model);
    assert(AT32Ol_Begin(&model, UINT32_MAX - 99U));
    AT32Ol_Step(&model, raw, true, true, 150U);
    assert(model.fault == AT32_OL_LEASE_EXPIRED);
    ready(&model);
    /* 独立标度向量：4次累加、3.3V、4mOhm和20倍增益，约397.09计数/A。 */
    uint16_t allowed_counts = (uint16_t)floorf(AT32_OL_CURRENT_LIMIT_A * 0.8f * 397.090909f);
    raw[1] += allowed_counts;
    AT32Ol_Step(&model, raw, true, true, 0U);
    assert(model.state == AT32_OL_READY && model.fault == AT32_OL_OK);
    raw[1] -= allowed_counts;
    assert(AT32Ol_Begin(&model, 0U));
    uint16_t trip_counts = (uint16_t)ceilf((AT32_OL_CURRENT_LIMIT_A + 0.01f) * 397.090909f);
    raw[1] += trip_counts;
    AT32Ol_Step(&model, raw, true, true, 1U);
    assert(model.fault == AT32_OL_OVERCURRENT && model.duty[1] == 0.5f);
    raw[1] -= trip_counts;
    ready(&model);
    AT32Ol_Step(&model, raw, true, false, 1U);
    assert(model.fault == AT32_OL_ENCODER_ERROR);
    ready(&model);
    AT32Ol_Step(&model, raw, false, true, 1U);
    assert(model.fault == AT32_OL_ADC_ERROR);
    ready(&model);
    raw[3] = 1000U;
    AT32Ol_Step(&model, raw, true, true, 1U);
    assert(model.fault == AT32_OL_BUS_VOLTAGE);
    raw[3] = 4120U;
    AT32Ol_Init(&model);
    for (unsigned i = 0U; i < AT32_OL_SETTLE_SAMPLES; ++i)
    {
        raw[0] = 7500U + (i % 100U);
        AT32Ol_Step(&model, raw, true, true, i / 20U);
        assert(!AT32Ol_Begin(&model, i / 20U));
    }
    raw[0] = 7631U;
    AT32Ol_Step(&model, raw, true, true, 1U);
    raw[0] += 41U;
    AT32Ol_Step(&model, raw, true, true, 2U);
    assert(model.fault == AT32_OL_OFFSET_ERROR);
    raw[0] -= 41U;
    ready(&model);
    assert(AT32Ol_Begin(&model, 0U));
    for (unsigned i = 0U; i <= AT32_OL_SAMPLE_LIMIT + 200U; ++i)
    {
        AT32Ol_Step(&model, raw, true, true, 0U);
    }
    assert(model.state == AT32_OL_DONE);
    ready(&model);
    assert(AT32Ol_Begin(&model, 0U));
    AT32Ol_Stop(&model, AT32_OL_USER_STOP);
    AT32Ol_Stop(&model, AT32_OL_ADC_ERROR);
    assert(model.fault == AT32_OL_USER_STOP && !AT32Ol_Begin(&model, 1U));
    puts("PASS open-loop: measured offsets, voltage envelope, duration/sample cap, lease wrap, "
         "current/ADC/encoder/voltage faults, latched stop, no restart");
    return 0;
}
