/* 使用真实开环保护模型验证Watch开关，测试不连接硬件。 */
#include <assert.h>
#include <stdio.h>
#include "openloop_keil_control.h"

static const uint16_t s_raw[4] = {8192U, 8192U, 8192U, 4704U};

static void ready(AT32OlModel *model, AT32KeilControl *control)
{
    AT32Ol_Init(model);
    AT32KeilControl_Init(control);
    g_openloop_enable = 1U;
    assert(!AT32KeilControl_Poll(control, model, g_openloop_enable, 0U));
    assert(g_openloop_enable == 0U && g_openloop_rejected_starts == 1U);
    for (uint32_t i = 0U; i < 22048U; ++i)
    {
        AT32Ol_Step(model, s_raw, true, true, i / 20U);
    }
    assert(model->state == AT32_OL_READY);
    g_openloop_enable = 2U;
    assert(!AT32KeilControl_Poll(control, model, g_openloop_enable, 1500U));
    assert(g_openloop_enable == 0U && model->state == AT32_OL_READY);
}

int main(void)
{
    AT32OlModel model;
    AT32KeilControl control;
    ready(&model, &control);
    g_openloop_enable = 1U;
    assert(!AT32KeilControl_Poll(&control, &model, g_openloop_enable, 2000U));
    assert(control.active && model.state == AT32_OL_ARMING);
    for (uint32_t i = 0U; i < 201000U; ++i)
    {
        uint32_t now = 2000U + i / 20U;
        if (i % 20U == 0U)
        {
            AT32KeilControl_Poll(&control, &model, g_openloop_enable, now);
        }
        AT32Ol_Step(&model, s_raw, true, true, now);
    }
    assert(model.state == AT32_OL_DONE && model.fault == AT32_OL_OK);
    assert(g_openloop_enable == 0U && !control.active);
    g_openloop_enable = 1U;
    assert(!AT32KeilControl_Poll(&control, &model, g_openloop_enable, 13000U));
    assert(g_openloop_enable == 0U && model.state == AT32_OL_DONE);

    ready(&model, &control);
    g_openloop_enable = 1U;
    AT32KeilControl_Poll(&control, &model, g_openloop_enable, 2000U);
    for (unsigned i = 0U; i < 200U; ++i)
    {
        AT32Ol_Step(&model, s_raw, true, true, 2000U);
    }
    assert(model.state == AT32_OL_RUNNING);
    g_openloop_enable = 0U;
    assert(AT32KeilControl_Poll(&control, &model, g_openloop_enable, 2001U));
    assert(model.state == AT32_OL_FAULT && model.fault == AT32_OL_USER_STOP);
    g_openloop_enable = 1U;
    AT32KeilControl_Poll(&control, &model, g_openloop_enable, 2002U);
    assert(g_openloop_enable == 0U && model.fault == AT32_OL_USER_STOP);

    ready(&model, &control);
    g_openloop_enable = 1U;
    AT32KeilControl_Poll(&control, &model, g_openloop_enable, 2000U);
    AT32Ol_Step(&model, s_raw, true, true, 2300U);
    assert(model.fault == AT32_OL_LEASE_EXPIRED);
    assert(AT32KeilControl_Poll(&control, &model, g_openloop_enable, 2301U));
    assert(g_openloop_enable == 0U);

    ready(&model, &control);
    g_openloop_enable = 1U;
    AT32KeilControl_Poll(&control, &model, g_openloop_enable, 2000U);
    const uint16_t high[4] = {9500U, 8192U, 8192U, 4704U};
    AT32Ol_Step(&model, high, true, true, 2001U);
    assert(model.fault == AT32_OL_OVERCURRENT);
    assert(AT32KeilControl_Poll(&control, &model, g_openloop_enable, 2002U));

    ready(&model, &control);
    assert(AT32Ol_Begin(&model, 2000U));
    g_openloop_enable = 1U;
    assert(!AT32KeilControl_Poll(&control, &model, g_openloop_enable, 2001U));
    assert(!control.active && model.state == AT32_OL_ARMING && g_openloop_enable == 0U);
    AT32Ol_Step(&model, s_raw, true, true, 2300U);
    assert(model.fault == AT32_OL_LEASE_EXPIRED);
    puts("PASS Keil switch: readiness, single 10s run, stop, latched faults and host ownership");
    return 0;
}
