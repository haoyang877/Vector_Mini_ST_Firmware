#ifndef __FOC_PARAM_PROFILE_H__
#define __FOC_PARAM_PROFILE_H__

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

/* Encoder open-loop calibration applies approximately 1.5x this current. */
#define PARAM_MOTOR_CALIB_CURRENT_A               1.4f
#define PARAM_MOTOR_CURRENT_LIMIT_A               6.0f
#define PARAM_MOTOR_SPEED_LIMIT_RPS               (372.0f / 60.0f)
#define PARAM_MOTOR_CURRENT_LOOP_BANDWIDTH_RAD_S  200.0f

#else
#error "Unsupported FOC_ACTIVE_MOTOR_PROFILE"
#endif

/* Hardware-dependent defaults. Encoder symbols are resolved at macro use. */
#if FOC_ACTIVE_HW_PROFILE == FOC_HW_PROFILE_VECTOR_MINI_ST

#define PARAM_HW_CURRENT_OFFSET_A_COUNTS          2048U
#define PARAM_HW_CURRENT_OFFSET_B_COUNTS          2048U
#define PARAM_HW_CURRENT_OFFSET_C_COUNTS          2048U

#define PARAM_HW_ONBOARD_ENCODER_TYPE             TLE5012B
#define PARAM_HW_ONBOARD_ENCODER_ENABLE           ENCODER_DISABLE
#define PARAM_HW_ONBOARD_ENCODER_DIR              1
#define PARAM_HW_EXTERNAL_ENCODER_TYPE            MT6701
#define PARAM_HW_EXTERNAL_ENCODER_ENABLE          ENCODER_ENABLE
#define PARAM_HW_EXTERNAL_ENCODER_DIR             1

#define PARAM_HW_CAN_NODE_ID                      0x00U
#define PARAM_HW_CAN_HEARTBEAT_MS                 500

#else
#error "Unsupported FOC_ACTIVE_HW_PROFILE"
#endif

/* Application defaults shared by the selected motor and hardware profiles. */
#define PARAM_APP_ENCODER_OFFSET_COUNTS           0
#define PARAM_APP_ENCODER_ZERO_COUNT              0
#define PARAM_APP_ENCODER_CALIB_FLAG              0

#define PARAM_APP_OPEN_LOOP_VOLTAGE_V             1.0f
#define PARAM_APP_OPEN_LOOP_ELEC_VEL_RAD_S        12.0f
#define PARAM_APP_OPEN_LOOP_THETA_RAD             0.0f

#define PARAM_APP_SPEED_ACCEL_RPS2                50.0f
#define PARAM_APP_SPEED_DECEL_RPS2                50.0f
#define PARAM_APP_SPEED_KP                        0.05f
#define PARAM_APP_SPEED_KI                        0.5f

#define PARAM_APP_POSITION_ACCEL_RPS2             10.0f
#define PARAM_APP_POSITION_DECEL_RPS2             10.0f
#define PARAM_APP_POSITION_MAX_SPEED_RPS          5.0f
#define PARAM_APP_POSITION_KP                     0.05f
#define PARAM_APP_POSITION_KD                     0.5f

#endif
