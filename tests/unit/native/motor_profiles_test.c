#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "motor_profile_selection.h"
#include "motor_startup_profile.h"

/* 期望值由独立的型号矩阵传入，不从被测配置回读生成。 */
static void near_value(float actual, float expected)
{
    assert(isfinite(actual));
    assert(fabsf(actual - expected) <= fmaxf(0.000001f, fabsf(expected) * 0.00001f));
}

int main(int argc, char **argv)
{
    assert(argc == 11);
    assert(strcmp(PARAM_MOTOR_MODEL_NAME, argv[1]) == 0);
    assert(strcmp(PARAM_MOTOR_CONFIGURATION_NAME, argv[2]) == 0);
    assert(PARAM_MOTOR_POLE_PAIRS == atoi(argv[3]));
    float resistance = strtof(argv[4], NULL);
    float inductance = strtof(argv[5], NULL);
    near_value(PARAM_MOTOR_PHASE_RESISTANCE_OHM, resistance);
    near_value(PARAM_MOTOR_D_INDUCTANCE_H, inductance);
    near_value(PARAM_MOTOR_Q_INDUCTANCE_H, inductance);
    near_value(PARAM_MOTOR_GEAR_RATIO, strtof(argv[6], NULL));
    near_value(PARAM_MOTOR_MAX_SPEED_RPM, strtof(argv[7], NULL));
    near_value(PARAM_MOTOR_RATED_VECTOR_A, strtof(argv[8], NULL));
    assert(PARAM_MOTOR_APP_DEFAULTS_READY == (unsigned)atoi(argv[9]));
    assert(PARAM_MOTOR_DATA_PROVISIONAL == (unsigned)atoi(argv[10]));
    near_value(PARAM_MOTOR_CURRENT_LOOP_BANDWIDTH_HZ, 1000.0f);
    near_value(PARAM_MOTOR_DEFAULT_ID_KP, inductance * 6283.185307f);
    near_value(PARAM_MOTOR_DEFAULT_IQ_KP, inductance * 6283.185307f);
    near_value(PARAM_MOTOR_DEFAULT_ID_KI, resistance * 6283.185307f);
    near_value(PARAM_MOTOR_DEFAULT_IQ_KI, resistance * 6283.185307f);
    /* 用独立数值检查Kt/磁链单位及电机轴惯量；避免额外乘减速比或e-4。 */
#if FOC_ACTIVE_MOTOR_PROFILE == FOC_MOTOR_PROFILE_YQ_5708 ||                                       \
    FOC_ACTIVE_MOTOR_PROFILE == FOC_MOTOR_PROFILE_YQ_5740
    near_value(PARAM_MOTOR_TORQUE_CONSTANT_NM_PER_A, 0.0801f);
    near_value(PARAM_MOTOR_FLUX_WB, 0.003814285714f);
    assert(PARAM_MOTOR_TORQUE_CONSTANT_VALID == 1U);
    assert(PARAM_MOTOR_ROTOR_INERTIA_VALID == 1U);
    assert(fabsf(PARAM_MOTOR_ROTOR_INERTIA_KG_M2 - 0.0005f) < 1.0e-10f);
    near_value(1.5f * 14.0f * PARAM_MOTOR_FLUX_WB * 2.0f, 0.1602f);
#else
    assert(PARAM_MOTOR_ROTOR_INERTIA_VALID == 0U);
    assert(PARAM_MOTOR_ROTOR_INERTIA_KG_M2 == 0.0f);
    near_value(PARAM_MOTOR_TORQUE_CONSTANT_NM_PER_A, 0.55132875f);
#if FOC_ACTIVE_MOTOR_PROFILE == FOC_MOTOR_PROFILE_HT8115_4
    assert(PARAM_MOTOR_TORQUE_CONSTANT_VALID == 1U);
#else
    assert(PARAM_MOTOR_TORQUE_CONSTANT_VALID == 0U);
#endif
#endif
#if FOC_ACTIVE_MOTOR_PROFILE == FOC_MOTOR_PROFILE_HT8115_4
    near_value(PARAM_MOTOR_PEAK_VECTOR_A, 7.0f);
    near_value(SENSORLESS_ENCODER_CALIB_ALIGN_CURRENT_A, MOTOR_HAS_DAMPING_RING ? 3.5f : 2.0f);
    near_value(SENSORLESS_ENCODER_CALIB_TARGET_ELEC_VEL_RAD_S,
               MOTOR_HAS_DAMPING_RING ? 420.0f : 315.0f);
    near_value(SENSORLESS_ENCODER_CALIB_SPEED_MEC_RAD_S, MOTOR_HAS_DAMPING_RING ? 20.0f : 15.0f);
#elif FOC_ACTIVE_MOTOR_PROFILE == FOC_MOTOR_PROFILE_YQ_5708 ||                                     \
    FOC_ACTIVE_MOTOR_PROFILE == FOC_MOTOR_PROFILE_YQ_5740
    near_value(PARAM_MOTOR_PEAK_VECTOR_A, 10.60660172f);
#endif
    printf("PASS %s: 1kHz, model inputs, gear ratio, calibration variant and readiness\n",
           PARAM_MOTOR_CONFIGURATION_NAME);
    return 0;
}
