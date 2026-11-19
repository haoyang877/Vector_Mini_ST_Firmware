/* 由 generate_hardware_limits.py 生成，修改版本化JSON契约。 */
#ifndef HARDWARE_CHECK_LIMITS_H
#define HARDWARE_CHECK_LIMITS_H
#include <stdint.h>
enum
{
    CHECK_PHASE_RMS_COUNTS,
    CHECK_BUS_RMS_COUNTS,
    CHECK_CURRENT_BIAS_COUNTS,
    CHECK_BUS_MIN_MV,
    CHECK_BUS_MAX_MV,
    CHECK_TEMPERATURE_MIN_OFFSET_DECI_C,
    CHECK_TEMPERATURE_MAX_OFFSET_DECI_C,
    CHECK_TEMPERATURE_MAX_AGE_MS,
    CHECK_ROTOR_STATIC_MAX_COUNTS,
    CHECK_HALL_RMS_COUNTS,
    CHECK_HALL_RAW_MIN,
    CHECK_HALL_RAW_MAX,
    CHECK_MOTOR_TREND_PER_3000,
    CHECK_HALL_TREND_PERMILLE,
    CHECK_RESISTANCE_MIN_PERMILLE,
    CHECK_RESISTANCE_MAX_PERMILLE,
    CHECK_LIMIT_COUNT
};
#define CHECK_DEFAULT_CURRENT_BIAS_COUNTS 300U
#define CHECK_LIMIT_DEFAULTS                                                                       \
    {10U,                                                                                          \
     50U,                                                                                          \
     CHECK_DEFAULT_CURRENT_BIAS_COUNTS,                                                            \
     27000U,                                                                                       \
     29000U,                                                                                       \
     1000U,                                                                                        \
     1600U,                                                                                        \
     30U,                                                                                          \
     58U,                                                                                          \
     10U,                                                                                          \
     95U,                                                                                          \
     4000U,                                                                                        \
     2000U,                                                                                        \
     400U,                                                                                         \
     500U,                                                                                         \
     1500U}
#define CHECK_LIMIT_MINIMUMS                                                                       \
    {1U, 1U, 1U, 1000U, 1000U, 600U, 600U, 1U, 1U, 1U, 0U, 1U, 1U, 0U, 1U, 1000U}
#define CHECK_LIMIT_MAXIMUMS                                                                       \
    {1000U,                                                                                        \
     1000U,                                                                                        \
     1000U,                                                                                        \
     60000U,                                                                                       \
     60000U,                                                                                       \
     2200U,                                                                                        \
     2200U,                                                                                        \
     1000U,                                                                                        \
     5800U,                                                                                        \
     1000U,                                                                                        \
     4094U,                                                                                        \
     4095U,                                                                                        \
     3000U,                                                                                        \
     999U,                                                                                         \
     1000U,                                                                                        \
     5000U}
#endif
