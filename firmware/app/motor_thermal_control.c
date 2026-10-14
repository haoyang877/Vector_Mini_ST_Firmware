/** @file
 * @brief 热模型单一所有者；冷态参数未确认时拒绝出力，不提供运行时绕过开关。
 */
#include "motor_thermal_control.h"
#include "motor_thermal_config.h"
#include "data_type.h"
#include "motor_hw.h"
#include <stddef.h>

/* 只读调试符号；外部不得写入以获得冷态预算。 */
static MotorThermalDiagnostics thermal_diagnostics;
/* 监督上下文只发布事件；模型失效及累计器仍由快上下文处理。 */
static volatile bool pending_clock_fault;
static volatile uint32_t published_update_ms;

static const MotorThermalConfig default_config = {MOTOR_THERMAL_CONTINUOUS_VECTOR_A,
                                                  MOTOR_THERMAL_CONTINUOUS_PHASE_RMS_A,
                                                  MOTOR_THERMAL_PEAK_A,
                                                  MOTOR_THERMAL_HEATING_TAU_S,
                                                  MOTOR_THERMAL_COOLING_TAU_S,
                                                  MOTOR_THERMAL_UPDATE_PERIOD_S,
                                                  MOTOR_THERMAL_FAST_SAMPLES,
                                                  MOTOR_THERMAL_DERATING_ENTER_HEAT,
                                                  MOTOR_THERMAL_DERATING_EXIT_HEAT,
                                                  MOTOR_THERMAL_TRIP_HEAT,
                                                  MOTOR_THERMAL_CLEAR_HEAT};
static MotorThermalConfig startup_config;
static bool startup_config_set;
/* 单一通信生产者；ready置位后不得覆盖，快环消费完成才释放槽位。 */
static struct
{
    MotorThermalConfig config;
    float heating_alpha;
    float cooling_alpha;
} pending_config;
static volatile bool pending_config_ready;

static MotorThermalConfig current_config(void)
{
    if (pending_config_ready)
    {
        motor_hw_outer_barrier();
        return pending_config.config;
    }
    if (thermal_diagnostics.initialized)
    {
        return thermal_diagnostics.model.config;
    }
    return startup_config_set ? startup_config : default_config;
}

static float *parameter_value(MotorThermalConfig *config, MotorThermalParameter parameter)
{
    switch (parameter)
    {
    case MOTOR_THERMAL_CONTINUOUS_VECTOR:
        return &config->continuous_vector_a;
    case MOTOR_THERMAL_PEAK:
        return &config->peak_a;
    case MOTOR_THERMAL_HEATING_TAU:
        return &config->heating_tau_s;
    case MOTOR_THERMAL_DERATING_ENTER:
        return &config->derating_enter_heat;
    case MOTOR_THERMAL_DERATING_EXIT:
        return &config->derating_exit_heat;
    case MOTOR_THERMAL_TRIP:
        return &config->trip_heat;
    case MOTOR_THERMAL_CLEAR:
        return &config->clear_heat;
    default:
        return NULL;
    }
}

static void derive_config(MotorThermalConfig *config)
{
    config->continuous_phase_rms_a = config->continuous_vector_a * 0.707106781f;
    config->cooling_tau_s = fmaxf(MOTOR_THERMAL_COOLING_MIN_TAU_S, config->heating_tau_s);
}

bool MotorThermalControl_LoadParameters(const float *values)
{
    MotorThermalConfig config = default_config;
    unsigned i;
    if (thermal_diagnostics.initialized)
    {
        return false;
    }
    if (values != NULL)
    {
        for (i = 0U; i < MOTOR_THERMAL_PARAMETER_COUNT; ++i)
        {
            *parameter_value(&config, (MotorThermalParameter)i) = values[i];
        }
        derive_config(&config);
        if (!MotorThermal_ConfigValid(&config))
        {
            return false;
        }
        startup_config = config;
    }
    startup_config_set = values != NULL;
    return true;
}

bool MotorThermalControl_GetParameter(MotorThermalParameter parameter, float *value)
{
    MotorThermalConfig config = current_config();
    float *field = parameter_value(&config, parameter);
    if (value == NULL || field == NULL)
    {
        return false;
    }
    *value = *field;
    return true;
}

bool MotorThermalControl_RequestParameter(MotorThermalParameter parameter,
                                          float value,
                                          uint32_t mode,
                                          bool outputs_disabled)
{
    MotorThermalConfig config;
    float *field;
    if (mode != Motor_Disable || !outputs_disabled || pending_config_ready ||
        !thermal_diagnostics.initialized || !thermal_diagnostics.model.valid || !isfinite(value))
    {
        return false;
    }
    config = current_config();
    field = parameter_value(&config, parameter);
    if (field == NULL)
    {
        return false;
    }
    *field = value;
    derive_config(&config);
    if (!MotorThermal_ConfigValid(&config))
    {
        return false;
    }
    pending_config.config = config;
    pending_config.heating_alpha = -expm1f(-config.update_period_s / config.heating_tau_s);
    pending_config.cooling_alpha = -expm1f(-config.update_period_s / config.cooling_tau_s);
    motor_hw_outer_barrier();
    pending_config_ready = true;
    return true;
}

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
    config = current_config();
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
    if (pending_config_ready && outputs_disabled)
    {
        motor_hw_outer_barrier();
        if (!MotorThermal_Reconfigure(state,
                                      &pending_config.config,
                                      pending_config.heating_alpha,
                                      pending_config.cooling_alpha))
        {
            MotorThermal_Invalidate(state);
        }
        motor_hw_outer_barrier();
        pending_config_ready = false;
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
    return !pending_clock_fault && !pending_config_ready && thermal_diagnostics.initialized &&
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
    float available_a = !pending_clock_fault && !pending_config_ready &&
                                thermal_diagnostics.initialized &&
                                thermal_diagnostics.parameters_confirmed
                            ? thermal_diagnostics.model.available_a
                            : 0.0f;
    return MotorThermal_ResolveLimit(configured_a, available_a, true);
}

bool MotorThermalControl_CanClearFault(void)
{
    return !pending_clock_fault && thermal_diagnostics.initialized &&
           thermal_diagnostics.parameters_confirmed && thermal_diagnostics.model.valid &&
           !pending_config_ready &&
           thermal_diagnostics.model.max_heat <= thermal_diagnostics.model.config.clear_heat;
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
