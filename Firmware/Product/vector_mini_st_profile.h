#ifndef PRODUCT_VECTOR_MINI_ST_PROFILE_H
#define PRODUCT_VECTOR_MINI_ST_PROFILE_H

#include "current_sense_profile.h"
#include "mechanical_load_profiles.h"

/*
 * Compile-time parameter profile selection.
 * Add a new profile ID and a corresponding #elif block for each motor or board.
 * The active profile may also be selected from the compiler command line.
 */
#define MOTOR_PROFILE_HT8115_4           1U
#define BOARD_PROFILE_VECTOR_MINI_ST     1U

#ifndef ACTIVE_MOTOR_PROFILE
#define ACTIVE_MOTOR_PROFILE             MOTOR_PROFILE_HT8115_4
#endif

#ifndef ACTIVE_BOARD_PROFILE
#define ACTIVE_BOARD_PROFILE             BOARD_PROFILE_VECTOR_MINI_ST
#endif

#ifndef ACTIVE_MECHANICAL_LOAD_PROFILE
#define ACTIVE_MECHANICAL_LOAD_PROFILE   MECHANICAL_LOAD_PROFILE_DAMPING_RING_1P5NM
#endif

/* Motor and motor-control defaults. */
#if ACTIVE_MOTOR_PROFILE == MOTOR_PROFILE_HT8115_4

#define PARAM_MOTOR_POLE_PAIRS                    21
#define PARAM_MOTOR_PHASE_RESISTANCE_OHM          (3.81f * 0.5f)
#define PARAM_MOTOR_D_INDUCTANCE_H                (3.27e-3f * 0.5f)
#define PARAM_MOTOR_Q_INDUCTANCE_H                (3.27e-3f * 0.5f)
/* Kv = 15 rpm/V: psi = 60 / (sqrt(3) * 2pi * pole_pairs * Kv). */
#define PARAM_MOTOR_FLUX_WB                       0.0175025f
#define PARAM_MOTOR_PHASE_RESISTANCE_MIN_OHM      0.0001f
#define PARAM_MOTOR_PHASE_RESISTANCE_MAX_OHM      5.0f
#define PARAM_MOTOR_INDUCTANCE_MIN_H              1.0e-6f
#define PARAM_MOTOR_INDUCTANCE_MAX_H              5.0e-3f
#define PARAM_MOTOR_FLUX_MIN_WB                    1.0e-5f
#define PARAM_MOTOR_FLUX_MAX_WB                    1.0f

/* Encoder electrical-angle alignment current. */
#define PARAM_MOTOR_CALIB_CURRENT_A               CURRENT_SENSE_PROFILE_DEFAULT_CALIB_A
#define PARAM_MOTOR_CURRENT_LIMIT_A               CURRENT_SENSE_PROFILE_DEFAULT_LIMIT_A
#if ACTIVE_MECHANICAL_LOAD_PROFILE == MECHANICAL_LOAD_PROFILE_DAMPING_RING_1P5NM
#define PARAM_MOTOR_SPEED_LIMIT_RPS               0.50f
#else
#define PARAM_MOTOR_SPEED_LIMIT_RPS               (372.0f / 60.0f)
#endif
#define PARAM_MOTOR_CURRENT_LOOP_BANDWIDTH_RAD_S  (500.0f * 6.283185307f)

/* Per-motor profile for encoder-independent phase-resistance identification. */
#define PARAM_MOTOR_PHASE_RESISTANCE_TEST_CURRENT_LOW_A   1.0f
#define PARAM_MOTOR_PHASE_RESISTANCE_TEST_CURRENT_HIGH_A  2.0f
#define PARAM_MOTOR_PHASE_RESISTANCE_TEST_CURRENT_MAX_A   3.0f
#define PARAM_MOTOR_PHASE_RESISTANCE_TEST_CURRENT_MIN_A   0.5f
#define PARAM_MOTOR_PHASE_RESISTANCE_CURRENT_TOLERANCE_A  0.10f
#define PARAM_MOTOR_PHASE_RESISTANCE_Q_CURRENT_TOLERANCE_A 0.10f
#define PARAM_MOTOR_PHASE_RESISTANCE_VOLTAGE_TOLERANCE_V  0.01f
#define PARAM_MOTOR_PHASE_RESISTANCE_VOLTAGE_MIN_DELTA_V  0.005f
#define PARAM_MOTOR_PHASE_RESISTANCE_VOLTAGE_FILTER_ALPHA 0.02f
#define PARAM_MOTOR_PHASE_RESISTANCE_RAMP_TIME_MS         200U
#define PARAM_MOTOR_PHASE_RESISTANCE_SETTLE_TIME_MS       200U
#define PARAM_MOTOR_PHASE_RESISTANCE_SAMPLE_TIME_MS       100U
#define PARAM_MOTOR_PHASE_RESISTANCE_PAUSE_TIME_MS        100U
#define PARAM_MOTOR_PHASE_RESISTANCE_TIMEOUT_MS           3000U
#define PARAM_MOTOR_PHASE_RESISTANCE_BALANCE_WARNING_PCT  3.0f
#define PARAM_MOTOR_PHASE_RESISTANCE_BALANCE_FAULT_PCT    5.0f

#else
#error "Unsupported ACTIVE_MOTOR_PROFILE"
#endif

/* Hardware-dependent defaults. Encoder symbols are resolved at macro use. */
#if ACTIVE_BOARD_PROFILE == BOARD_PROFILE_VECTOR_MINI_ST

#define PARAM_HW_CURRENT_OFFSET_A_COUNTS          2048U
#define PARAM_HW_CURRENT_OFFSET_B_COUNTS          2048U
#define PARAM_HW_CURRENT_OFFSET_C_COUNTS          2048U
#define PARAM_HW_PHASE_RESISTANCE_PATH_COMPENSATION_OHM CURRENT_SENSE_PROFILE_PATH_COMPENSATION_OHM

#define PARAM_HW_CAN_NODE_ID                      0x00U
#define PARAM_HW_CAN_HEARTBEAT_MS                 500

#ifdef HARDWARE_VALIDATION_SKIP_TEMPERATURE_PROTECTION
#define PARAM_HW_TEMPERATURE_PROTECTION_ENABLED   0U
#else
#define PARAM_HW_TEMPERATURE_PROTECTION_ENABLED   1U
#endif

#else
#error "Unsupported ACTIVE_BOARD_PROFILE"
#endif

/* Application defaults shared by the selected motor and hardware profiles. */
#define PARAM_APP_ENCODER_ELECTRICAL_ZERO_Q15     0U
#define PARAM_APP_ENCODER_MECHANICAL_ZERO_Q15     0U
#define PARAM_APP_ENCODER_CALIB_FLAG              0U
#define PARAM_APP_ENCODER_REVERSE                 0U

#define PARAM_APP_OPEN_LOOP_VOLTAGE_V             1.0f
#define PARAM_APP_OPEN_LOOP_ELEC_VEL_RAD_S        12.0f
#define PARAM_APP_OPEN_LOOP_THETA_RAD             0.0f

#define PARAM_APP_SPEED_ACCEL_RPS2                50.0f
#define PARAM_APP_SPEED_DECEL_RPS2                50.0f
#define PARAM_APP_SPEED_KP                        0.05f
#define PARAM_APP_SPEED_KI                        0.5f

/* Low-speed (8 s/rev) position-impedance defaults. */
#define PARAM_APP_POSITION_ACCEL_RPS2             0.125f
#define PARAM_APP_POSITION_DECEL_RPS2             0.125f
#if ACTIVE_MECHANICAL_LOAD_PROFILE == MECHANICAL_LOAD_PROFILE_DAMPING_RING_1P5NM
#define PARAM_APP_POSITION_MAX_SPEED_RPS          0.50f
#else
#define PARAM_APP_POSITION_MAX_SPEED_RPS          0.125f
#endif
/* Iq = Kp * position_error + Kd * velocity_error + integral_current. */
#define PARAM_APP_POSITION_KP                     8.0f   /* A/rad */
#define PARAM_APP_POSITION_KD                     0.50f  /* A/(rad/s) */
#define PARAM_APP_POSITION_KI                     10.0f  /* A/(rad*s) */

#define PARAM_APP_POSITION_INTEGRAL_LIMIT_A       5.0f
#define PARAM_APP_POSITION_ERROR_WINDOW_RAD       0.001f

/* Cascade position -> speed -> current cascade outer-loop gains. */
#define PARAM_APP_CASCADE_POSITION_KP             0.05f
#define PARAM_APP_CASCADE_POSITION_KD             0.50f

#endif
