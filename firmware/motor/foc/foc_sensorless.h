#ifndef FOC_SENSORLESS_H
#define FOC_SENSORLESS_H

#include "motor_config.h"
#include "motor_types.h"
#include "foc_algorithm.h"

typedef struct
{
    float voltage_alpha_v, voltage_beta_v;
    float current_alpha_a, current_beta_a;

    float sin, cos;

    float gamma;

    float y1_last, y2_last;
    float etax1, etax2;
    float flux_error_wb2;
    float x1_last, x2_last;
    float x1, x2;
    float electrical_angle_rad, electrical_velocity_rad_s;
    float previous_electrical_angle_rad, previous_electrical_velocity_rad_s;
    float electrical_position_rad;
    uint32_t position_epoch;
} FluxObserverState;

typedef enum
{
    SENSORLESS_STARTUP_IDLE = 0,
    SENSORLESS_STARTUP_ALIGN,
    SENSORLESS_STARTUP_OPEN_LOOP,
    SENSORLESS_STARTUP_SPEED_LOCK,
    SENSORLESS_STARTUP_HANDOFF,
    SENSORLESS_STARTUP_CLOSED_LOOP
} SensorlessStartupPhase;

typedef struct
{
    float align_current_ramp_time_s;
    float align_hold_time_s;
    float align_current_a;
    float startup_iq_initial_a;
    float startup_iq_a;
    float startup_iq_ramp_time_s;
    float startup_id_a;
    float minimum_current_limit_a;
    float minimum_electrical_velocity_rad_s;
    float target_electrical_velocity_rad_s;
    float startup_ramp_time_s;
    float speed_lock_time_s;
    float speed_lock_filter_alpha;
    float observer_lock_ratio;
    float angle_handoff_time_s;
    float lock_timeout_s;
    float id_ramp_down_time_s;
    float observer_loss_time_s;
} SensorlessStartupConfig;

typedef struct
{
    SensorlessStartupPhase state;
    float open_loop_angle_rad;
    float open_loop_velocity_rad_s;
    float handoff_phase_delta_rad;
    float handoff_id_reference_a;
    float speed_feedback_rad_s;
    float lock_speed_feedback_rad_s;
    float direction;
    float speed_pi_output_max;
    uint32_t state_ticks;
    uint32_t speed_loop_ticks;
    uint32_t open_loop_ticks;
    uint32_t lock_ticks;
    uint32_t id_ramp_ticks;
    uint32_t loss_ticks;
} SensorlessStartupState;

/**
 * @brief  初始化磁链观测器参数。
 * @param  flux_observer 观测器状态指针。
 */
void FluxObserver_ParamInit(FluxObserverState *flux_observer);

/**
 * @brief  执行一次磁链观测器更新。
 * @param configuration 活动配置只读视图；本次调用期间须保持稳定。
 * @param  foc foc 状态指针。
 * @param  flux_observer 观测器状态指针。
 */
void FluxObserver_Update(const MotorConfiguration *configuration,
                         FocState *foc,
                         FluxObserverState *flux_observer);

/**
 * @brief  读取观测器电角度。
 * @param  flux_observer 观测器状态指针。
 * @return 电角度，单位 rad。
 */
float Observer_GetElePhase(FluxObserverState *flux_observer);

/**
 * @brief  读取观测器电角速度。
 * @param  flux_observer 观测器状态指针。
 * @return 电角速度，单位 rad/s。
 */
float Observer_GetEleVel(FluxObserverState *flux_observer);

/**
 * @brief  读取观测器连续电角度（未归一化）。
 * @param  flux_observer 观测器状态指针。
 * @return 连续电角度，单位 rad。
 */
float Observer_GetElePosition(FluxObserverState *flux_observer);

/**
 * @brief  读取观测器位置纪元计数。
 * @param  flux_observer 观测器状态指针。
 * @return 位置纪元计数，用于判定样本是否更新。
 */
uint32_t Observer_GetPositionEpoch(FluxObserverState *flux_observer);

#endif
