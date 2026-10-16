#include "motor_thermal_config.h"

/* 物理配置派生固定采样时序；默认峰值不覆盖应用限流，配置更新不含热历史。 */

#include <stddef.h>

void MotorWindingThermal_Default(MotorWindingThermalConfig *config)
{
    *config = (MotorWindingThermalConfig){3.0f, 7.0f, 65.98486785f, 0.8f, 0.7f, 1.0f, 0.5f};
}

bool MotorWindingThermal_Derive(const MotorWindingThermalConfig *config,
                                MotorThermalConfig *derived)
{
    if (config == NULL || derived == NULL)
    {
        return false;
    }
    MotorThermalConfig candidate = {config->continuous_vector_a,
                                    config->continuous_vector_a * 0.707106781f,
                                    config->peak_a,
                                    config->heating_tau_s,
                                    config->heating_tau_s > 120.0f ? config->heating_tau_s : 120.0f,
                                    0.001f,
                                    20U,
                                    config->derating_enter_heat,
                                    config->derating_exit_heat,
                                    config->trip_heat,
                                    config->clear_heat};
    if (!MotorThermal_ConfigValid(&candidate))
    {
        return false;
    }
    *derived = candidate;
    return true;
}

bool MotorWindingThermal_Valid(const MotorWindingThermalConfig *config)
{
    MotorThermalConfig derived;
    return MotorWindingThermal_Derive(config, &derived);
}
