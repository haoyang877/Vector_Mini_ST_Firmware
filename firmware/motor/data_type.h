#ifndef __DATA_TYPE_H__
#define __DATA_TYPE_H__

#include <stdbool.h>
#include <stdint.h>
#include "motor_axis_profile.h"

#include "motor_control_mode.h"

typedef enum
{
    No_Error = 0,
    CurrentOffset_Error,
    Encoder_Error,
    PolePairs_Error,
    CAN_DisConnect,
    Large_Phase_Resistance,
    Large_Phase_Inductance,
    Over_Current,
    Over_Voltage,
    Under_Voltage,
    High_Temprature,
    Encoder_NotCalibrated,
    MotorParam_Error,
    Sensorless_Error,
    FrictionIdentification_Error,
    ControlOverrun_Error,
    CoggingCalibration_Error,
    TemperatureSensor_Error,
} ErrorNow_TypeDef;

typedef struct
{
    MotorControlMode control_mode;
    ErrorNow_TypeDef ErrorNow;

    float ErrorNow_f;

    float A_Offset; /* fractional 12-bit ADC counts, same units as stored parameters */
    float B_Offset;
    float C_Offset;

    int32_t motor_pole_pairs;
    float motor_phase_resistance;
    /* Raw resistance from the 0/120/240 degree test vectors. */
    float phase_resistance_vector[3];
    float phase_resistance_a;
    float phase_resistance_b;
    float phase_resistance_c;
    float phase_resistance_spread_pct;
    float motor_d_inductance;
    float motor_q_inductance;
    float motor_flux;
    bool phase_resistance_valid;
    bool phase_resistance_warning;
    bool phase_resistance_balanced;

    bool isUseSensorless;

    float calib_current;
    float current_limit;
    float speed_limit;

    float idRef;
    float id_Kp;
    float id_Ki;
    float iqRef;
    float iq_Kp;
    float iq_Ki;
    /*q-axis voltage command (V), used by MOTOR_CONTROL_VQ*/
    float vqRef;

    bool isUseSpeedRamp;
    float speedAcc;
    float speedDec;
    float speedRef;
    float speedShadow;
    float speed_Kp;
    float speed_Ki;

    bool isReachTargetPos;
    float pos_error_window;
    float posAcc;
    float posDec;
    float pos_maxspeed;
    float posRef;
    float posShadow;
    float cascade_pos_Kp;
    float cascade_pos_Kd;

    /* Applied friction model in current domain; identification does not enable compensation. */
    float friction_coulomb_pos_a;
    float friction_coulomb_neg_a;
    float friction_viscous_pos_a_per_rad_s;
    float friction_viscous_neg_a_per_rad_s;
    bool friction_model_valid;

    /* 位置级联遥测；控制器状态由所属模块私有持有。 */
    float pos_vel_filtered;

    /*voltage open-loop mode*/

    /* Append to preserve existing diagnostic member offsets. */
    MotorAxisProfile axis_profile;
    bool axis_profile_valid;
    /* RAM-only mode-3 A/B request, copied with the motor configuration snapshot. */
    bool position_hold_filter_bypass;
    bool position_hold_filter_half_cutoff;     /* RAM-only comparison against default cutoff. */
    bool position_velocity_filter_half_cutoff; /* RAM-only base velocity filter trial. */
    /* 模式 3 规划轨迹速度参考（rad/s）；由外环发布，遥测无效窗口为 NaN。 */
    float pos_trajectory_speed_rad_s;
} MotorControl_TypeDef;

#endif
