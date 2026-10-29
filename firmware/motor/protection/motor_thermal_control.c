#include "motor_thermal_control.h"

#include <stddef.h>
#include "motor_fault.h"
#include "control_config.h"

#if APP_ENABLED_WINDING_THERMAL_MODEL

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

void MotorThermalControl_UpdateProtection(const MotorConfiguration *configuration,
                                          const FocState *foc,
                                          MotorControlState *motor,
                                          bool samples_ready,
                                          bool outputs_disabled,
                                          bool output_expected,
                                          bool derating_allowed)
{
    /* 快环独占模型，低优先级监督不能在本次调用中改写时钟故障。
     * 只取一次资格快照，避免CurrentLimit/CanClear/CanRun/Guard重复展开同一组判断。 */
    MotorThermalState *model = &s_thermal.model;
    bool available = s_thermal.initialized && !s_clock_fault;
    bool valid = available && model->valid;
    bool healthy = valid && !model->fault_latched;
    motor->thermal_current_limit_a = MotorThermal_ResolveLimit(
        configuration->application.current_limit_a, available ? model->available_a : 0.0f, true);
    motor->thermal_limit_active =
        motor->thermal_current_limit_a < configuration->application.current_limit_a;
    motor->thermal_clear_ready =
        valid && (!model->fault_latched || model->max_heat <= model->config.clear_heat);
    if (!samples_ready)
    {
        return;
    }
    if (!healthy)
    {
        MotorFault_Report(motor, MOTOR_FAULT_WINDING_THERMAL);
    }
    else if (output_expected && !derating_allowed && model->derating)
    {
        /* 与原Guard相同：先保留本拍限流/恢复快照，再锁存不允许降额的策略故障。 */
        model->fault_latched = true;
        model->available_a = 0.0f;
        MotorFault_Report(motor, MOTOR_FAULT_WINDING_THERMAL);
    }
    if (!outputs_disabled && MotorControlMode_IsVoltage(motor->control_mode))
    {
        float vector_squared_a2 = (2.0f / 3.0f) * (foc->phase_a_current_a * foc->phase_a_current_a +
                                                   foc->phase_b_current_a * foc->phase_b_current_a +
                                                   foc->phase_c_current_a * foc->phase_c_current_a);
        float limit_a = motor->thermal_current_limit_a;
        if (!(vector_squared_a2 <= limit_a * limit_a))
        {
            MotorFault_Report(motor, MOTOR_FAULT_WINDING_THERMAL);
        }
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

#else

/* 限时台架关闭热估算；不伪造模型有效或冷却状态，仍使用配置的硬电流限值。 */
static bool s_limit_initialized;

bool MotorThermalControl_Init(const MotorWindingThermalConfig *config, uint32_t now_ms)
{
    (void)now_ms;
    if (!MotorWindingThermal_Valid(config))
    {
        return false;
    }
    s_limit_initialized = true;
    return true;
}

bool MotorThermalControl_Configure(const MotorWindingThermalConfig *config, bool outputs_disabled)
{
    return s_limit_initialized && outputs_disabled && MotorWindingThermal_Valid(config);
}

void MotorThermalControl_Sample(
    float ia_a, float ib_a, float ic_a, bool outputs_disabled, uint32_t now_ms)
{
    (void)ia_a;
    (void)ib_a;
    (void)ic_a;
    (void)outputs_disabled;
    (void)now_ms;
}

void MotorThermalControl_UpdateProtection(const MotorConfiguration *configuration,
                                          const FocState *foc,
                                          MotorControlState *motor,
                                          bool samples_ready,
                                          bool outputs_disabled,
                                          bool output_expected,
                                          bool derating_allowed)
{
    (void)output_expected;
    (void)derating_allowed;
    float limit_a = MotorThermalControl_CurrentLimit(configuration->application.current_limit_a);
    motor->thermal_current_limit_a = limit_a;
    motor->thermal_limit_active = false;
    motor->thermal_clear_ready = s_limit_initialized;
    /* 开环电压模式的瞬时电流边界属于限流，不能随热估算一起关闭。 */
    if (samples_ready && !outputs_disabled && MotorControlMode_IsVoltage(motor->control_mode))
    {
        float vector_squared_a2 = (2.0f / 3.0f) * (foc->phase_a_current_a * foc->phase_a_current_a +
                                                   foc->phase_b_current_a * foc->phase_b_current_a +
                                                   foc->phase_c_current_a * foc->phase_c_current_a);
        if (!(vector_squared_a2 <= limit_a * limit_a))
        {
            MotorFault_Report(motor, MOTOR_FAULT_WINDING_THERMAL);
        }
    }
}

bool MotorThermalControl_CheckWatchdog(uint32_t now_ms, bool outputs_disabled)
{
    (void)now_ms;
    return outputs_disabled || s_limit_initialized;
}

bool MotorThermalControl_CanRun(bool derating_allowed)
{
    (void)derating_allowed;
    return s_limit_initialized;
}

bool MotorThermalControl_Guard(bool derating_allowed)
{
    return MotorThermalControl_CanRun(derating_allowed);
}

float MotorThermalControl_CurrentLimit(float configured_a)
{
    return s_limit_initialized ? MotorThermal_ResolveLimit(configured_a, 0.0f, false) : 0.0f;
}

bool MotorThermalControl_CanClearFault(void)
{
    return s_limit_initialized;
}

bool MotorThermalControl_ClearFault(void)
{
    return s_limit_initialized;
}

bool MotorThermalControl_GetDiagnostics(MotorThermalDiagnostics *output)
{
    (void)output;
    return false;
}

#endif
