/* 自定义配置只验证派生值及策略隔离；实际ADC位宽另由MCU驱动拒绝。 */
#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "board_config.h"
#include "application_config.h"
#include "board_config.h"
#include "application_config.h"
#include "application_config.h"

int main(void)
{
    assert(PARAM_MOTOR_POLE_PAIRS == 14);
    assert(fabsf(PARAM_MOTOR_PHASE_RESISTANCE_OHM - 0.75f) < 1e-7f);
    assert(fabsf(PARAM_MOTOR_DEFAULT_ID_KP - 0.0002f * 6283.185307f) < 1e-5f);
    assert(fabsf(PARAM_MOTOR_DEFAULT_IQ_KP - PARAM_MOTOR_DEFAULT_ID_KP) < 1e-7f);
    assert(fabsf(PARAM_MOTOR_DEFAULT_ID_KI - 0.75f * 6283.185307f) < 1e-2f);
    assert(SENSORLESS_ENCODER_CALIB_TARGET_ELEC_VEL_RAD_S == 350.0f);
#if BSP_BOARD == BSP_BOARD_STM32G4
    assert(CURRENT_SENSE_SHUNT_MILLIOHM == 2U);
    assert(CURRENT_SENSE_PROFILE_ADC_MAX_COUNTS == 4095U);
    assert(CURRENT_SENSE_PROFILE_COMMAND_LIMIT_MAX_A == 10.0f);
    assert(CURRENT_SENSE_PROFILE_OVERCURRENT_TRIP_A == 18.0f);
    assert(fabsf(MOTOR_SENSING_CURRENT_A_PER_COUNT - 3.3f / 4095.0f / 5.0f / 0.002f) < 1e-7f);
#else
    assert(CURRENT_SENSE_SHUNT_MILLIOHM == 5U);
    assert(CURRENT_SENSE_PROFILE_ADC_MAX_COUNTS == 1023U);
    assert(R40_ADC_MAX_COUNTS == 1023U);
    assert(fabsf(R40_PHASE_SHUNT_OHM - 0.005f) < 1e-7f);
    assert(CURRENT_SENSE_PROFILE_COMMAND_LIMIT_MAX_A == 3.0f);
    assert(fabsf(MOTOR_SENSING_CURRENT_A_PER_COUNT - 3.3f / 1023.0f / 10.0f / 0.005f) < 1e-7f);
    assert(PARAM_MOTOR_THERMAL_PEAK_A == 7.0f);
#endif
    puts("PASS custom board and motor inputs derive without selector edits");
    return 0;
}
