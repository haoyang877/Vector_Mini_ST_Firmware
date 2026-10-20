#include "motor_thermal_config.h"
#include "control_config.h"

/* 物理配置派生固定采样时序；默认峰值不覆盖应用限流，配置更新不含热历史。 */

#include <stddef.h>

/* 热模型保持每毫秒更新，窗口样本数随快环时基派生。 */
#define WINDING_THERMAL_UPDATE_HZ 1000U
#define WINDING_THERMAL_UPDATE_PERIOD_S (1.0f / (float)WINDING_THERMAL_UPDATE_HZ)
#define WINDING_THERMAL_SAMPLES_PER_UPDATE (FOC_FREQ / WINDING_THERMAL_UPDATE_HZ)

#if FOC_FREQ < WINDING_THERMAL_UPDATE_HZ || (FOC_FREQ % WINDING_THERMAL_UPDATE_HZ) != 0U
#error "FOC_FREQ must be an integer multiple of WINDING_THERMAL_UPDATE_HZ"
#endif

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
                                    WINDING_THERMAL_UPDATE_PERIOD_S,
                                    WINDING_THERMAL_SAMPLES_PER_UPDATE,
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
