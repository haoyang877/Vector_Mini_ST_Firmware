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
/* 2026-09-23 台架实测（无阻尼环轮）：mode 17 三相电阻 2.1488/2.1402/2.1439 Ω、
 * 开环电压阶跃拟合 R=2.1350 Ω、τ=374 µs、L=0.799 mH；扫频复验 L≈0.9 mH。
 * 取 R=2.144 Ω、Ld=Lq=0.9 mH（用户相间实测 1.76–2.6 mH ⇒ 每相 0.88–1.30 mH，取低端）。 */
#define PARAM_MOTOR_PHASE_RESISTANCE_OHM          2.144f
#define PARAM_MOTOR_D_INDUCTANCE_H                0.9e-3f
#define PARAM_MOTOR_Q_INDUCTANCE_H                0.9e-3f
/* Kv = 15 rpm/V: psi = 60 / (sqrt(3) * 2pi * pole_pairs * Kv). */
#define PARAM_MOTOR_FLUX_WB                       0.0175025f

/* Motor current defaults do not increase when a lower-resistance shunt is fitted.
 * Mode-specific alignment minima remain defined in hw_conf.h. */
/* This is the no-damping-ring profile on the 6 mOhm board: 3 A calibration and
 * a 6 A software limit, both below the 6 mOhm command ceiling (10 A) and trip
 * level (18 A). The 2.5 Nm friction-shaft values (4.5 A / 8 A) belong to the
 * damping-ring build only. */
#define PARAM_MOTOR_CALIB_CURRENT_A               3.0f
#define PARAM_MOTOR_CURRENT_LIMIT_A               6.0f
#define PARAM_MOTOR_SPEED_LIMIT_RPS               0.50f
/* 电流环设计带宽：Kp = L·ω_c、Ki = Rs·ω_c（foc_param.c）。
 * 2026-09-23 台架实测：500 Hz 设计实测穿越 580 Hz；提到 1000 Hz 后 3 A d 轴阶跃
 * 超调 40.8%（P 冲击 17 V 触电压饱和）；退回 750 Hz 重测超调 0.39%、闭环 −3 dB 1400 Hz。
 * 配合上面的 R/L 取值，本带宽生成已验证增益 Kp≈4.241、Ki≈10103。 */
#define PARAM_MOTOR_CURRENT_LOOP_BANDWIDTH_RAD_S  (750.0f * 6.283185307f)

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

/* Direct-drive chassis wheels: node 1 = left, node 2 = right.
 * omega_max = v_max / radius = 2 * 1 m/s / 0.10 m = 20 rad/s.
 * Other nodes retain the selected motor profile's ceiling. */
#define PARAM_WHEEL_DIAMETER_M                    0.10f
#define PARAM_WHEEL_MAX_LINEAR_SPEED_M_S          1.0f
#define PARAM_WHEEL_SPEED_LIMIT_RAD_S \
    (2.0f * PARAM_WHEEL_MAX_LINEAR_SPEED_M_S / PARAM_WHEEL_DIAMETER_M)

static inline float Param_SpeedLimitRadS(unsigned int node_id)
{
    return (node_id == 1U || node_id == 2U) ? PARAM_WHEEL_SPEED_LIMIT_RAD_S :
        PARAM_MOTOR_SPEED_LIMIT_RPS * 6.2831853072f;
}

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
/* 速度环默认增益：2026-09-24 台架在 20 kHz PLL 测速下重扫（无阻尼环轮、6 mΩ、6 A、32.9 V）。
 * 关键结论：限制速度环带宽的是**反馈路径相位滞后（PLL 二阶滞后为主），不是噪声**。
 *   PLL ωn=150 Hz → Kp=0.04 即失稳；ωn=300 Hz → 边界 ~0.07；ωn=600 Hz → 边界 ~0.11。
 * 当前取 Kp=0.06/Ki=0.6 配合 ENCODER_PLL_OMEGA_N_HZ=600：
 *   3 rad/s std 4.0%（Kp=0.06 是纹波最优点）；全速域 1/5/10/15/20 rad/s 误差 -0.8/+1.1/+1.3/+0.4/+0.1%；
 *   稳定余量到 Kp=0.10（0.12 失稳）。
 * 代价：高转速段纹波偏高（10–20 rad/s 为 4–6%，因 ωn 高让周期误差进入反馈）。
 * 后续若要两头都占，应做**按速调度 ωn**（低速用 600 换增益余量、高速用 300 滤周期误差）。 */
#define PARAM_APP_SPEED_KP                        0.06f
#define PARAM_APP_SPEED_KI                        0.6f

/* Low-speed (8 s/rev) position-impedance defaults. */
#define PARAM_APP_POSITION_ACCEL_RPS2             0.125f
#define PARAM_APP_POSITION_DECEL_RPS2             0.125f
#define PARAM_APP_POSITION_MAX_SPEED_RPS          0.125f
/* Iq = Kp * position_error + Kd * velocity_error + integral_current. */
#define PARAM_APP_POSITION_KP                     8.0f   /* A/rad */
#define PARAM_APP_POSITION_KD                     0.50f  /* A/(rad/s) */
#define PARAM_APP_POSITION_KI                     10.0f  /* A/(rad*s) */

#define PARAM_APP_POSITION_INTEGRAL_LIMIT_A       5.0f

/* Legacy position -> speed -> current cascade outer-loop gains. */
#define PARAM_APP_CASCADE_POSITION_KP             0.05f
#define PARAM_APP_CASCADE_POSITION_KD             0.50f

/* Unloaded, continuous-rotation friction identification profile. */
#define PARAM_FRICTION_IDENT_SPEED_0_RPS           0.10f
#define PARAM_FRICTION_IDENT_SPEED_1_RPS           0.20f
#define PARAM_FRICTION_IDENT_SPEED_2_RPS           0.40f
#define PARAM_FRICTION_IDENT_SPEED_3_RPS           0.50f
#define PARAM_FRICTION_IDENT_SPEED_POINT_COUNT     4U
#define PARAM_FRICTION_IDENT_STABLE_TIME_S         0.75f
#define PARAM_FRICTION_IDENT_TRACK_TIMEOUT_S       8.0f
#define PARAM_FRICTION_IDENT_SAMPLE_TIMEOUT_S      15.0f
#define PARAM_FRICTION_IDENT_STOP_HOLD_TIME_S      0.30f
#define PARAM_FRICTION_IDENT_STOP_TIMEOUT_S        5.0f
#define PARAM_FRICTION_IDENT_SPEED_TOLERANCE_RATIO 0.05f
#define PARAM_FRICTION_IDENT_MIN_SPEED_TOL_RAD_S   0.08f
#define PARAM_FRICTION_IDENT_STOP_SPEED_RAD_S      0.12f
#define PARAM_FRICTION_IDENT_SAMPLE_TURNS          1.0f
#define PARAM_FRICTION_IDENT_MIN_SAMPLE_TIME_S     0.50f
#define PARAM_FRICTION_IDENT_CURRENT_RATIO_MAX     0.90f
#define PARAM_FRICTION_IDENT_SATURATION_TIME_S     0.25f
#define PARAM_FRICTION_IDENT_RMSE_FLOOR_A          0.05f
#define PARAM_FRICTION_IDENT_RMSE_RATIO_MAX        0.25f

#endif
