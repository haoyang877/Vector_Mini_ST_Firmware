#ifndef __POSITION_CASCADE_CONFIG_H__
#define __POSITION_CASCADE_CONFIG_H__

#define CASCADE_POSITION_KP_MAX_PER_S  50.0f
#define CASCADE_POSITION_KD_MAX        10.0f

/* Mode-3参考整形；惯量前馈由MotorControl的J/Kt运行参数提供，默认0。 */
#define POSITION_SERVO_ACCEL_RAMP_TIME_S                 0.10f
/* Jerk shaping is independent of the trajectory speed-error response time. */
#define POSITION_SERVO_JERK_RAMP_TIME_S                  0.20f
/* Mode-3 soft-stop tuning ceiling; preserve a lower commanded deceleration. */
#define POSITION_SERVO_DECELERATION_MAX_RAD_S2           0.523598776f /* 30 deg/s2 */

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

/* 带阻尼环的停靠范围独立于精确到位，电流只在卸载过渡中保留。 */
#define POSITION_SERVO_DAMPED_ERROR_WINDOW_RAD           0.0013962634f /* 0.08 deg */
#define POSITION_SERVO_PASSIVE_ENTER_POSITION_RAD        0.008726646f /* 0.5 deg；用户指定零保持电流窗口 */
#define POSITION_SERVO_PASSIVE_EXIT_POSITION_RAD         0.017453293f /* 1 deg；原始与资格位置均越界50 ms才纠偏 */
#define POSITION_SERVO_PASSIVE_CAPTURE_SPEED_RAD_S       0.003f /* 静止确认，避免运动末段过早撤力 */
#define POSITION_SERVO_PASSIVE_RELEASE_A_PER_S           2.0f
/* 阻尼轴近目标禁积分；退出滞环仅控制积分策略，不扩大停靠或到位窗口。 */
#define POSITION_SERVO_NO_INTEGRAL_ENTER_RAD              0.008726646f /* 0.5 deg */
#define POSITION_SERVO_NO_INTEGRAL_EXIT_RAD               0.012217305f /* 0.7 deg */
/* 低速不代表停滞：每推进0.06°续期，500 ms无有效推进才卸旧积分。 */
#define POSITION_SERVO_NEAR_PROGRESS_RAD                  0.001047198f /* 0.06 deg */
#define POSITION_SERVO_NEAR_STUCK_TIME_S                  0.50f

/* Identified friction model shaping and static breakaway assistance.
 * 2026-09-22 experiment on the damping-ring motor: the breakaway pulse is a
 * torque step into a low-inertia rotor and produced 1.5-3 deg lurches with
 * 14-15 rpm spikes (see docs/reports/2026-09/cascade_smoothness_20260922.md).
 * Ratio 1.0 removes the step so the feedforward is a single ramped value, and
 * the attack/release slews are slowed so no step is injected at all. */
#define POSITION_SERVO_FRICTION_REFERENCE_SPEED_RAD_S    0.03f
#define POSITION_SERVO_FRICTION_STOP_SPEED_RAD_S         0.02f
#define POSITION_SERVO_FRICTION_MOVE_SPEED_RAD_S         0.05f
#define POSITION_SERVO_FRICTION_BREAKAWAY_DISTANCE_RAD   0.003f
#define POSITION_SERVO_FRICTION_STUCK_TIME_S             0.05f
#define POSITION_SERVO_FRICTION_BREAKAWAY_RATIO          1.00f
/* Keep some approach torque until capture; zero-torque distance is inside it. */
#define POSITION_SERVO_FRICTION_LANDING_ZERO_RATIO       0.50f
/* 缓升轨迹起步时避免模型力矩先加满，使轴冲到参考前方后又被PI制动。 */
#define POSITION_SERVO_FRICTION_ATTACK_SLEW_A_PER_S       2.0f
#define POSITION_SERVO_FRICTION_FAST_RELEASE_SLEW_A_PER_S 30.0f
#define POSITION_SERVO_FRICTION_CAPTURE_RELEASE_SLEW_A_PER_S 30.0f
#define POSITION_SERVO_FRICTION_RELEASE_SLEW_A_PER_S     15.0f

/* Unload reversal/overshoot integral; preserve learned load during braking. */
#define POSITION_SERVO_INTEGRAL_OPPOSING_DECAY_RATE_PER_S 10.0f
#define POSITION_SERVO_INTEGRAL_OPPOSING_MAX_SLEW_A_PER_S 5.0f
#define POSITION_SERVO_INTEGRAL_ZERO_THRESHOLD_A          0.001f

#endif
