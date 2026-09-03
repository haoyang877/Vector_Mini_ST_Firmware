#ifndef __FOC_PARAM_PROFILE_H__
#define __FOC_PARAM_PROFILE_H__

#include "current_sense_profile.h"

/*
 * Compile-time parameter profile selection.
 * Add a new profile ID and a corresponding #elif block for each motor or board.
 * The active profile may also be selected from the compiler command line.
 */
#define FOC_MOTOR_PROFILE_HT8115_4       1U
#define FOC_HW_PROFILE_VECTOR_MINI_ST    1U

#ifndef FOC_ACTIVE_MOTOR_PROFILE
#define FOC_ACTIVE_MOTOR_PROFILE         FOC_MOTOR_PROFILE_HT8115_4
#endif

#ifndef FOC_ACTIVE_HW_PROFILE
#define FOC_ACTIVE_HW_PROFILE            FOC_HW_PROFILE_VECTOR_MINI_ST
#endif

/* Motor and motor-control defaults. */
#if FOC_ACTIVE_MOTOR_PROFILE == FOC_MOTOR_PROFILE_HT8115_4

#define PARAM_MOTOR_POLE_PAIRS                    21
#define PARAM_MOTOR_PHASE_RESISTANCE_OHM          (3.81f * 0.5f)
#define PARAM_MOTOR_D_INDUCTANCE_H                (3.27e-3f * 0.5f)
#define PARAM_MOTOR_Q_INDUCTANCE_H                (3.27e-3f * 0.5f)
/* Kv = 15 rpm/V: psi = 60 / (sqrt(3) * 2pi * pole_pairs * Kv). */
#define PARAM_MOTOR_FLUX_WB                       0.0175025f

/* Encoder electrical-angle alignment current. */
#define PARAM_MOTOR_CALIB_CURRENT_A               CURRENT_SENSE_PROFILE_DEFAULT_CALIB_A
#define PARAM_MOTOR_CURRENT_LIMIT_A               CURRENT_SENSE_PROFILE_DEFAULT_LIMIT_A
#define PARAM_MOTOR_SPEED_LIMIT_RPS               (372.0f / 60.0f)
#define PARAM_MOTOR_CURRENT_LOOP_BANDWIDTH_RAD_S  (500.0f * 6.283185307f)

/* Per-motor profile for encoder-independent phase-resistance identification. */
#define PARAM_MOTOR_PHASE_RESISTANCE_TEST_CURRENT_LOW_A   1.0f
#define PARAM_MOTOR_PHASE_RESISTANCE_TEST_CURRENT_HIGH_A  2.0f
#define PARAM_MOTOR_PHASE_RESISTANCE_TEST_CURRENT_MAX_A   3.0f
#define PARAM_MOTOR_PHASE_RESISTANCE_RAMP_TIME_MS         200U
#define PARAM_MOTOR_PHASE_RESISTANCE_SETTLE_TIME_MS       200U
#define PARAM_MOTOR_PHASE_RESISTANCE_SAMPLE_TIME_MS       100U
#define PARAM_MOTOR_PHASE_RESISTANCE_PAUSE_TIME_MS        100U
#define PARAM_MOTOR_PHASE_RESISTANCE_TIMEOUT_MS           3000U
#define PARAM_MOTOR_PHASE_RESISTANCE_BALANCE_WARNING_PCT  3.0f
#define PARAM_MOTOR_PHASE_RESISTANCE_BALANCE_FAULT_PCT    5.0f

#else
#error "Unsupported FOC_ACTIVE_MOTOR_PROFILE"
#endif

/* Hardware-dependent defaults. Encoder symbols are resolved at macro use. */
#if FOC_ACTIVE_HW_PROFILE == FOC_HW_PROFILE_VECTOR_MINI_ST

#define PARAM_HW_CURRENT_OFFSET_A_COUNTS          2048U
#define PARAM_HW_CURRENT_OFFSET_B_COUNTS          2048U
#define PARAM_HW_CURRENT_OFFSET_C_COUNTS          2048U
#define PARAM_HW_PHASE_RESISTANCE_PATH_COMPENSATION_OHM CURRENT_SENSE_PROFILE_PATH_COMPENSATION_OHM

#define PARAM_HW_CAN_NODE_ID                      0x00U
#define PARAM_HW_CAN_HEARTBEAT_MS                 500

#else
#error "Unsupported FOC_ACTIVE_HW_PROFILE"
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
#define PARAM_APP_POSITION_MAX_SPEED_RPS          0.125f
/* Iq = Kp * position_error + Kd * velocity_error + integral_current. */
#define PARAM_APP_POSITION_KP                     8.0f   /* A/rad */
#define PARAM_APP_POSITION_KD                     0.50f  /* A/(rad/s) */
#define PARAM_APP_POSITION_KI                     10.0f  /* A/(rad*s) */

/* Compile-time shaping for the low-speed impedance controller. */
#define PARAM_APP_POSITION_VELOCITY_FILTER_HZ     20.0f
#define PARAM_APP_POSITION_INTEGRAL_LIMIT_A       1.5f
#define PARAM_APP_POSITION_INTEGRAL_ZONE_RAD      0.08726646f /* 5 deg */
#define PARAM_APP_POSITION_INTEGRAL_SPEED_RAD_S   0.03f
/*
 * Forget position-dependent holding current smoothly as the rotor moves.
 * The decay distance is an e-fold distance: after 5 deg, 36.8% remains.
 */
#define PARAM_APP_POSITION_INTEGRAL_DECAY_DISTANCE_RAD       0.08726646f /* 5 deg */
#define PARAM_APP_POSITION_INTEGRAL_DECAY_MIN_SPEED_RAD_S    0.02f
#define PARAM_APP_POSITION_INTEGRAL_OPPOSING_DECAY_RATIO_PER_TICK 0.01f
#define PARAM_APP_POSITION_INTEGRAL_DECAY_MAX_RATIO_PER_TICK 0.01f
#define PARAM_APP_POSITION_INTEGRAL_DECAY_MAX_STEP_A         0.005f
#define PARAM_APP_POSITION_INTEGRAL_ZERO_THRESHOLD_A         0.0005f
#define PARAM_APP_POSITION_HOLD_ENTER_SPEED_RAD_S 0.03f
#define PARAM_APP_POSITION_HOLD_EXIT_SPEED_RAD_S  0.08f
#define PARAM_APP_POSITION_HOLD_TIME_S            0.05f

#endif
