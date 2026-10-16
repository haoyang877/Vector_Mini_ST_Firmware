#include "motor_thermal_control.h"

#include <stddef.h>

/* 快环拥有模型；监督上下文只发布时钟失效事件，不清零热量。 */
static MotorThermalDiagnostics s_thermal;
static volatile bool s_clock_fault;
static volatile uint32_t s_published_update_ms;

bool MotorThermalControl_Init(const MotorWindingThermalConfig *config, uint32_t now_ms)
{
    MotorThermalConfig derived;
    if (!MotorWindingThermal_Derive(config, &derived))
    {
        return false;
    }
    if (s_thermal.initialized)
    {
        return true;
    }
    if (!MotorThermal_Init(&s_thermal.model, &derived, 0.0f))
    {
        return false;
    }
    s_thermal.last_update_ms = s_published_update_ms = now_ms;
    s_thermal.initialized = true;
    return true;
}

bool MotorThermalControl_Configure(const MotorWindingThermalConfig *config, bool outputs_disabled)
{
    MotorThermalConfig derived;
    if (!outputs_disabled || !s_thermal.initialized || s_clock_fault ||
        !MotorWindingThermal_Derive(config, &derived))
    {
        return false;
    }
    return MotorThermal_Reconfigure(
        &s_thermal.model,
        &derived,
        MotorThermal_Alpha(derived.update_period_s, derived.heating_tau_s),
        MotorThermal_Alpha(derived.update_period_s, derived.cooling_tau_s));
}

void MotorThermalControl_Sample(
    float ia_a, float ib_a, float ic_a, bool outputs_disabled, uint32_t now_ms)
{
    MotorThermalState *model = &s_thermal.model;
    if (!s_thermal.initialized)
    {
        return;
    }
    if (s_clock_fault)
    {
        MotorThermal_Invalidate(model);
        return;
    }
    if (!MotorThermal_Accumulate(model, ia_a, ib_a, ic_a))
    {
        return;
    }
    if (model->sample_count == model->config.samples_per_update)
    {
        if (!outputs_disabled && (uint32_t)(now_ms - s_thermal.last_update_ms) > 2U)
        {
            MotorThermal_Invalidate(model);
            return;
        }
        (void)MotorThermal_Update(model);
        s_thermal.last_update_ms = s_published_update_ms = now_ms;
        ++s_thermal.updates;
    }
}

bool MotorThermalControl_CheckWatchdog(uint32_t now_ms, bool outputs_disabled)
{
    if (outputs_disabled)
    {
        return true;
    }
    if (!s_thermal.initialized || s_clock_fault || (uint32_t)(now_ms - s_published_update_ms) > 2U)
    {
        s_clock_fault = true;
        return false;
    }
    return true;
}

bool MotorThermalControl_CanRun(bool derating_allowed)
{
    return s_thermal.initialized && !s_clock_fault && s_thermal.model.valid &&
           !s_thermal.model.fault_latched && (derating_allowed || !s_thermal.model.derating);
}

bool MotorThermalControl_Guard(bool derating_allowed)
{
    if (MotorThermalControl_CanRun(derating_allowed))
    {
        return true;
    }
    if (!derating_allowed && s_thermal.initialized && s_thermal.model.derating)
    {
        s_thermal.model.fault_latched = true;
        s_thermal.model.available_a = 0.0f;
    }
    return false;
}

float MotorThermalControl_CurrentLimit(float configured_a)
{
    float available_a =
        s_thermal.initialized && !s_clock_fault ? s_thermal.model.available_a : 0.0f;
    return MotorThermal_ResolveLimit(configured_a, available_a, true);
}

bool MotorThermalControl_CanClearFault(void)
{
    return s_thermal.initialized && !s_clock_fault && s_thermal.model.valid &&
           (!s_thermal.model.fault_latched ||
            s_thermal.model.max_heat <= s_thermal.model.config.clear_heat);
}

bool MotorThermalControl_ClearFault(void)
{
    return MotorThermalControl_CanClearFault() &&
           (!s_thermal.model.fault_latched || MotorThermal_ClearFault(&s_thermal.model));
}

bool MotorThermalControl_GetDiagnostics(MotorThermalDiagnostics *output)
{
    if (output == NULL || !s_thermal.initialized)
    {
        return false;
    }
    *output = s_thermal;
    return true;
}
