#include "app_feature_requirements.h"
/* 使用生产默认值装配和配置校验，检查两板首次上电参数可通过自检。 */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "parameter_record.h"
#include "parameter_values.h"
#include "application_profile_defaults.h"
#include "motor_profile_defaults.h"
#include "device_profile_defaults.h"
#include "angle_feedback.h"
#include "control_config.h"
#include "bsp_config.h"

#include "app_defaults_subject.inc"

int main(void)
{
    MotorConfiguration configuration = {0};
    Param_Return_Default(&configuration);
    assert(configuration.application.current_limit_a == EXPECTED_DEFAULT_A);
    assert(configuration.application.current_limit_a <= CURRENT_SENSE_PROFILE_COMMAND_LIMIT_MAX_A);
    assert(ParameterValue_ConfigurationValid(&configuration));
    assert(!configuration.configuration_confirmed);
    assert(configuration.device.encoder.calib_flag == 0U);
    /* 独立数值锁住配置迁移前行为；不能用被测宏自行生成期望值。 */
    assert(configuration.application.speed_kp == EXPECTED_SPEED_KP);
    assert(configuration.application.speed_ki == EXPECTED_SPEED_KI);
    assert(ENCODER_PLL_OMEGA_N_HZ == EXPECTED_PLL_HZ);
    assert(CURRENT_SENSE_PROFILE_OVERCURRENT_TRIP_A == EXPECTED_TRIP_A);
    assert(PARAM_MOTOR_POLE_PAIRS == EXPECTED_POLE_PAIRS);
    assert(PARAM_MOTOR_PHASE_RESISTANCE_OHM == EXPECTED_PHASE_R_OHM);
    assert(BSP_ENABLED_BUS_CURRENT == EXPECTED_BUS_SENSOR);
    assert(BSP_REQUIRED_MCU_TEMPERATURE == !EXPECTED_BUS_SENSOR);
    assert(FOC_FREQ == 20000U && SUPERVISOR_FREQ == 2000U && SPEED_LOOP_FREQ == 2000U);
    assert(PARAM_MOTOR_CURRENT_LOOP_BANDWIDTH_HZ == 1000.0f);
    if (EXPECTED_BUS_SENSOR)
    {
        assert(PARAM_MOTOR_THERMAL_HEATING_TAU_S == 3600.0f);
        assert(PARAM_MOTOR_THERMAL_PEAK_A == 3.0f);
    }
    else
    {
        assert(PARAM_MOTOR_THERMAL_HEATING_TAU_S == 65.98486785f);
        assert(PARAM_MOTOR_THERMAL_PEAK_A == 7.0f);
    }
    configuration.application.current_limit_a = CURRENT_SENSE_PROFILE_COMMAND_LIMIT_MAX_A + 1.0f;
    assert(!ParameterValue_ConfigurationValid(&configuration));
    puts("PASS real APP defaults satisfy board current limit without bypassing configuration "
         "guards");
    return 0;
}
