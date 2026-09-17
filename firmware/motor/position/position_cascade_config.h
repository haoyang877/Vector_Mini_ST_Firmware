#ifndef __POSITION_CASCADE_CONFIG_H__
#define __POSITION_CASCADE_CONFIG_H__

#define CASCADE_POSITION_KP_MAX_PER_S  50.0f
#define CASCADE_POSITION_KD_MAX        10.0f

/*
 * Mode-3 position-servo reference shaping. The acceleration feedforward gain
 * remains zero until J/Kt has been identified for the complete mechanism.
 */
#define POSITION_SERVO_ACCEL_RAMP_TIME_S                 0.10f
/* Jerk shaping is independent of the trajectory speed-error response time. */
#define POSITION_SERVO_JERK_RAMP_TIME_S                  0.20f
/* Mode-3 soft-stop tuning ceiling; preserve a lower commanded deceleration. */
#define POSITION_SERVO_DECELERATION_MAX_RAD_S2           0.523598776f /* 30 deg/s2 */
#define POSITION_SERVO_ACCEL_FF_GAIN_A_PER_RAD_S2        0.0f

/* Software tuning defaults, to validate on the direct-drive mechanism. */
#ifndef POSITION_SERVO_VELOCITY_FILTER_HZ
/* One shared MOVE/SETTLE/HOLD low-pass. Roll motion/position verified;
 * loaded-axis/full-speed validation remains target-specific. */
#define POSITION_SERVO_VELOCITY_FILTER_HZ                10.0f
#endif
/* Optional extra HOLD stage retained for comparative tests. Disabled by
 * default: all phases use the same base-filtered velocity without cascading
 * another low-pass on entry to HOLD. */
#ifndef POSITION_SERVO_HOLD_VELOCITY_FILTER_HZ
#define POSITION_SERVO_HOLD_VELOCITY_FILTER_HZ            0.0f
#endif
/* Reserve bounded speed correction above planned cruise speed. */
#define POSITION_SERVO_SPEED_CORRECTION_HEADROOM_RATIO   1.50f
/* Soft reference governor for point-to-point moves; 0 disables it. Not a fault limit. */
#define POSITION_SERVO_FOLLOWING_ERROR_LIMIT_RAD          0.0f /* Baseline: governor off. */
/* Software tuning rate, not an identified torque or a new current limit. */
#define POSITION_SERVO_STICTION_INTEGRAL_RATE_A_PER_S     0.30f

/* Motion/settling/hold hysteresis. */
#define POSITION_SERVO_HOLD_ENTER_SPEED_RAD_S            0.03f
#define POSITION_SERVO_HOLD_EXIT_SPEED_RAD_S             0.08f
#define POSITION_SERVO_HOLD_ENTER_POSITION_RAD           0.003141593f /* 0.18 deg */
#define POSITION_SERVO_HOLD_EXIT_POSITION_RAD            0.004537856f /* 0.26 deg */
#define POSITION_SERVO_HOLD_CONFIRM_TIME_S               0.05f

/* Identified friction model shaping and static breakaway assistance. */
#define POSITION_SERVO_FRICTION_REFERENCE_SPEED_RAD_S    0.03f
#define POSITION_SERVO_FRICTION_STOP_SPEED_RAD_S         0.02f
#define POSITION_SERVO_FRICTION_MOVE_SPEED_RAD_S         0.05f
#define POSITION_SERVO_FRICTION_BREAKAWAY_DISTANCE_RAD   0.003f
#define POSITION_SERVO_FRICTION_STUCK_TIME_S             0.05f
#define POSITION_SERVO_FRICTION_BREAKAWAY_RATIO          1.20f
/* Keep some approach torque until capture; zero-torque distance is inside it. */
#define POSITION_SERVO_FRICTION_LANDING_ZERO_RATIO       0.50f
#define POSITION_SERVO_FRICTION_ATTACK_SLEW_A_PER_S      200.0f
#define POSITION_SERVO_FRICTION_FAST_RELEASE_SLEW_A_PER_S 200.0f
#define POSITION_SERVO_FRICTION_CAPTURE_RELEASE_SLEW_A_PER_S 30.0f
#define POSITION_SERVO_FRICTION_RELEASE_SLEW_A_PER_S     15.0f

/* Unload reversal/overshoot integral; preserve learned load during braking. */
#define POSITION_SERVO_INTEGRAL_OPPOSING_DECAY_RATE_PER_S 10.0f
#define POSITION_SERVO_INTEGRAL_OPPOSING_MAX_SLEW_A_PER_S 5.0f
#define POSITION_SERVO_INTEGRAL_ZERO_THRESHOLD_A          0.001f

#endif
