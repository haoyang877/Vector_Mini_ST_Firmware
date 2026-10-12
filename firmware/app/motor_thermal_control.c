/** @file
 * @brief 热模型单一所有者；冷态参数未确认时拒绝出力，不提供运行时绕过开关。
 */
#include "motor_thermal_control.h"
#include "motor_thermal_config.h"
#include "data_type.h"
#include <stddef.h>

/* 只读调试符号；外部不得写入以获得冷态预算。 */
static MotorThermalDiagnostics thermal_diagnostics;
/* 监督上下文只发布事件；模型失效及累计器仍由快上下文处理。 */
static volatile bool pending_clock_fault;
static volatile uint32_t published_update_ms;

static bool power_mode(uint32_t mode)
{
    switch (mode)
    {
    case Motor_Disable:
    case Set_ZeroPosition:
    case Default_Param:
    case Save_Param:
    case Clear_Error:
    case Calib_CurrentOffset:
        return false;
    default:
        return true;
    }
}

static bool abort_on_derating(uint32_t mode)
{
    switch (mode)
    {
    case Current_Mode:
    case Speed_Mode:
    case Position_Mode:
    case Sensorless_Speed_Mode:
    case Position_Impedance_Mode:
        return false;
    default:
        return power_mode(mode);
    }
}

void MotorThermalControl_Init(uint32_t now_ms)
{
    MotorThermalConfig config;
    if (thermal_diagnostics.initialized)
    {
        return;
    }
    config.continuous_vector_a = MOTOR_THERMAL_CONTINUOUS_VECTOR_A;
    config.continuous_phase_rms_a = MOTOR_THERMAL_CONTINUOUS_PHASE_RMS_A;
    config.peak_a = MOTOR_THERMAL_PEAK_A;
    config.heating_tau_s = MOTOR_THERMAL_HEATING_TAU_S;
    config.cooling_tau_s = MOTOR_THERMAL_COOLING_TAU_S;
    config.update_period_s = MOTOR_THERMAL_UPDATE_PERIOD_S;
    config.samples_per_update = MOTOR_THERMAL_FAST_SAMPLES;
    /* 用户指定上电按冷态开始；MCU复位丢失热历史，重复初始化仍保留热量。 */
    MotorThermal_Init(&thermal_diagnostics.model, &config, 0.0f);
    thermal_diagnostics.last_update_ms = now_ms;
    published_update_ms = now_ms;
    thermal_diagnostics.parameters_confirmed = MOTOR_THERMAL_PARAMETERS_CONFIRMED == 1U;
    thermal_diagnostics.initialized = true;
}

void MotorThermalControl_Sample(
    float ia_a, float ib_a, float ic_a, bool outputs_disabled, uint32_t now_ms)
{
    MotorThermalState *state = &thermal_diagnostics.model;
    if (!thermal_diagnostics.initialized)
    {
        return;
    }
    if (pending_clock_fault)
    {
        MotorThermal_Invalidate(state);
        return;
    }
    if (!MotorThermal_Accumulate(state, ia_a, ib_a, ic_a))
    {
        return;
    }
    if (state->sample_count == state->config.samples_per_update)
    {
        /* 时钟量化允许0/1/2 ms；超出窗口不能当成只消耗1 ms热量。 */
        if (!outputs_disabled && (uint32_t)(now_ms - thermal_diagnostics.last_update_ms) > 2U)
        {
            MotorThermal_Invalidate(state);
            return;
        }
        MotorThermal_Update(state);
        thermal_diagnostics.last_update_ms = now_ms;
        published_update_ms = now_ms;
        ++thermal_diagnostics.updates;
    }
}

bool MotorThermalControl_CheckWatchdog(uint32_t now_ms, bool outputs_disabled)
{
    if (outputs_disabled)
    {
        return true;
    }
    if (!thermal_diagnostics.initialized || pending_clock_fault ||
        (uint32_t)(now_ms - published_update_ms) > 2U)
    {
        pending_clock_fault = true;
        return false;
    }
    return true;
}

bool MotorThermalControl_CanRunMode(uint32_t mode)
{
    const MotorThermalState *state = &thermal_diagnostics.model;
    if (mode >= MODE_NUM || mode == Calib_Anticogging)
    {
        return false;
    }
    if (!power_mode(mode))
    {
        return true;
    }
    return !pending_clock_fault && thermal_diagnostics.initialized &&
           thermal_diagnostics.parameters_confirmed && state->valid && !state->fault_latched &&
           !(state->derating && abort_on_derating(mode));
}

bool MotorThermalControl_Guard(uint32_t mode)
{
    if (MotorThermalControl_CanRunMode(mode))
    {
        return true;
    }
    if (thermal_diagnostics.parameters_confirmed && abort_on_derating(mode) &&
        thermal_diagnostics.model.derating)
    {
        thermal_diagnostics.model.fault_latched = true;
        thermal_diagnostics.model.available_a = 0.0f;
    }
    return false;
}

float MotorThermalControl_CurrentLimit(float configured_a)
{
    float available_a = !pending_clock_fault && thermal_diagnostics.initialized &&
                                thermal_diagnostics.parameters_confirmed
                            ? thermal_diagnostics.model.available_a
                            : 0.0f;
    return MotorThermal_ResolveLimit(configured_a, available_a, true);
}

bool MotorThermalControl_CanClearFault(void)
{
    return !pending_clock_fault && thermal_diagnostics.initialized &&
           thermal_diagnostics.parameters_confirmed && thermal_diagnostics.model.valid &&
           thermal_diagnostics.model.max_heat <= 0.5f;
}

bool MotorThermalControl_ClearFault(void)
{
    return MotorThermalControl_CanClearFault() &&
           MotorThermal_ClearFault(&thermal_diagnostics.model);
}

bool MotorThermalControl_ParametersConfirmed(void)
{
    return thermal_diagnostics.parameters_confirmed;
}

bool MotorThermalControl_GetDiagnostics(MotorThermalDiagnostics *output)
{
    if (output == NULL || !thermal_diagnostics.initialized)
    {
        return false;
    }
    *output = thermal_diagnostics;
    output->watchdog_fault_pending = pending_clock_fault;
    return true;
}
