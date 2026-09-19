#ifndef __FOC_SENSORLESS_H__
#define __FOC_SENSORLESS_H__

#include "data_type.h"
#include "foc_algorithm.h"

typedef struct
{
    float Ualpha, Ubeta;
    float Ialpha, Ibeta;

    float sin, cos;

    float gamma;

    float y1_last, y2_last;
    float etax1, etax2;
    float phi_err;
    float x1_last, x2_last;
    float x1, x2;
    float theta_e, omega_e;
    float theta_last, omega_last;
    float theta_e_unwrapped;
    uint32_t position_epoch;
} Fluxobserver_TypeDef;

typedef enum
{
    SENSORLESS_STARTUP_IDLE = 0,
    SENSORLESS_STARTUP_ALIGN,
    SENSORLESS_STARTUP_OPEN_LOOP,
    SENSORLESS_STARTUP_SPEED_LOCK,
    SENSORLESS_STARTUP_HANDOFF,
    SENSORLESS_STARTUP_CLOSED_LOOP
} SensorlessStartupState_TypeDef;

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
} SensorlessStartupConfig_TypeDef;

typedef struct
{
    SensorlessStartupState_TypeDef state;
    float open_loop_theta;
    float open_loop_omega;
    float handoff_phase_delta;
    float handoff_id_reference;
    float speed_feedback;
    float lock_speed_feedback;
    float direction;
    float speed_pi_output_max;
    uint32_t state_ticks;
    uint32_t speed_loop_ticks;
    uint32_t open_loop_ticks;
    uint32_t lock_ticks;
    uint32_t id_ramp_ticks;
    uint32_t loss_ticks;
} SensorlessStartup_TypeDef;

/**
 * @brief  初始化磁链观测器参数。
 * @param  Fluxobserver 观测器状态指针。
 */
void Fluxobserver_ParamInit(Fluxobserver_TypeDef *Fluxobserver);

/**
 * @brief  执行一次磁链观测器更新。
 * @param  FOC FOC 状态指针。
 * @param  MotorControl 电机控制状态指针。
 * @param  Fluxobserver 观测器状态指针。
 */
void Fluxobserver_Update(FOC_TypeDef *FOC,
                         MotorControl_TypeDef *MotorControl,
                         Fluxobserver_TypeDef *Fluxobserver);

/**
 * @brief  读取观测器电角度。
 * @param  Fluxobserver 观测器状态指针。
 * @return 电角度，单位 rad。
 */
float Observer_GetElePhase(Fluxobserver_TypeDef *Fluxobserver);

/**
 * @brief  读取观测器电角速度。
 * @param  Fluxobserver 观测器状态指针。
 * @return 电角速度，单位 rad/s。
 */
float Observer_GetEleVel(Fluxobserver_TypeDef *Fluxobserver);

/**
 * @brief  读取观测器连续电角度（未归一化）。
 * @param  Fluxobserver 观测器状态指针。
 * @return 连续电角度，单位 rad。
 */
float Observer_GetElePosition(Fluxobserver_TypeDef *Fluxobserver);

/**
 * @brief  读取观测器位置纪元计数。
 * @param  Fluxobserver 观测器状态指针。
 * @return 位置纪元计数，用于判定样本是否更新。
 */
uint32_t Observer_GetPositionEpoch(Fluxobserver_TypeDef *Fluxobserver);

#endif
