#ifndef POSITION_CASCADE_H
#define POSITION_CASCADE_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    POSITION_SERVO_PHASE_MOVE = 0,
    POSITION_SERVO_PHASE_SETTLE,
    POSITION_SERVO_PHASE_HOLD
} PositionServoPhase;

typedef struct
{
    float update_period_s;
    uint16_t call_divider;
    float target_position;
    float position_error_window;   /* Trajectory finish tolerance only, rad. */
    float hold_enter_position;     /* Position qualification window, rad. */
    float hold_exit_position;      /* Wider hold/recovery hysteresis, rad. */
    float velocity_filter_hz;      /* 0 bypasses the base filter; HOLD shaping is separate. */
    float hold_velocity_filter_hz; /* 0 bypasses HOLD shaping; runtime tuning in Hz. */
    float following_error_limit;   /* Soft governor range in rad; 0 disables. */
    float stiction_integral_rate;  /* A/s added to PI only after confirmed stall; 0 disables. */
    float acceleration;
    float deceleration;
    float maximum_speed;
    float speed_limit_rad_s;
    float jerk_limit;
    float position_kp;
    float position_kd;
    float speed_kp;
    float speed_ki;
    float acceleration_feedforward_gain;
    float current_limit_a;
    bool friction_feedforward_enabled;
    float friction_coulomb_positive;
    float friction_coulomb_negative;
    float friction_viscous_positive;
    float friction_viscous_negative;
    float friction_breakaway_ratio;
    float friction_attack_slew_rate;
    float friction_fast_release_slew_rate;
    float friction_release_slew_rate;
    float friction_reference_speed;
    float friction_stop_speed;
    float friction_move_speed;
    float friction_breakaway_distance;
    float friction_stuck_time;
} PositionCascadeConfig;

typedef struct
{
    float position_reference;
    float speed_reference;
    float acceleration_reference;
    float speed_feedback_rad_s;
    float iq_reference;
    float feedback_current;
    float acceleration_feedforward_current;
    float friction_feedforward_current;
    float hold_current;
    PositionServoPhase phase;
    bool target_reached;
} PositionCascadeOutput;

typedef struct
{
    float position_reference;
    float trajectory_speed_reference;
    float speed_command;
    float speed_feedback_rad_s; /* Actual control velocity, including optional HOLD filter.
                           * HOLD exit uses the original feedback before that filter. */
    float acceleration_reference;
    float feedback_current;
    float acceleration_feedforward_current;
    float friction_feedforward_current;
    float hold_current;
    PositionServoPhase phase;
    bool target_reached;
    bool current_saturated;
    bool friction_landing_active;
    bool settle_recovery_active;
    bool hold_candidate_active;
    bool trajectory_limited;
    bool stiction_integrating; /* Bounded static-error integral applied this update. */
} PositionCascadeTelemetry;

/* Minimal current-command/feedback view for the high-rate motor adapter.
 * Rich diagnostics remain available through GetTelemetry / the existing API. */
typedef struct
{
    float position_reference;
    float speed_reference;
    float speed_feedback_rad_s;
    float iq_reference;
    bool target_reached;
} PositionCascadeControlOutput;

/**
 * @brief 验证完整配置并执行一个位置级联调用，保持既有分频。
 * @param config 只读完整配置，位置 rad、速度 rad/s、时间 s、电流 A；内部保存验证值。
 * @param measured_position 有限机械位置，单位 rad。
 * @param measured_speed 有限机械速度，单位 rad/s。
 * @param output 调用方非空结果空间；只在计算成功时填充。
 * @return 配置、反馈和计算有效返回 true，否则返回 false。
 * @note 所有入口共享一个私有控制器，须由同一调度所有者串行调用；无硬件访问。
 */
bool PositionCascade_UpdateControl(const PositionCascadeConfig *config,
                                   float measured_position,
                                   float measured_speed,
                                   PositionCascadeControlOutput *output);
/**
 * @brief 复用已验证调参，仅更新目标并执行位置级联调用。
 * @param target_position 有限目标位置，单位 rad。
 * @param measured_position 有限机械位置，单位 rad。
 * @param measured_speed 有限机械速度，单位 rad/s。
 * @param output 调用方非空结果空间；只在计算成功时填充。
 * @return 已有有效配置且本次计算有效返回 true，否则返回 false。
 * @note 所有入口共享一个私有控制器，须由同一调度所有者串行调用；无硬件访问。调参变化必须先通过完整 Update 入口验证。
 */
bool PositionCascade_UpdateTargetControl(float target_position,
                                         float measured_position,
                                         float measured_speed,
                                         PositionCascadeControlOutput *output);

/**
 * @brief 清零私有控制与轨迹状态，撤销配置有效性。
 * @note 所有入口共享一个私有控制器，须由同一调度所有者串行调用；无硬件访问。
 */
void PositionCascade_Reset(void);
/**
 * @brief 验证完整配置并执行一个位置级联调用，保持既有分频。
 * @param config 只读完整配置，位置 rad、速度 rad/s、时间 s、电流 A；内部保存验证值。
 * @param measured_position 有限机械位置，单位 rad。
 * @param measured_speed 有限机械速度，单位 rad/s。
 * @param output 调用方非空结果空间；只在计算成功时填充。
 * @return 配置、反馈和计算有效返回 true，否则返回 false。
 * @note 所有入口共享一个私有控制器，须由同一调度所有者串行调用；无硬件访问。
 */
bool PositionCascade_Update(const PositionCascadeConfig *config,
                            float measured_position,
                            float measured_speed,
                            PositionCascadeOutput *output);
/**
 * @brief 借用最近验证成功的只读配置。
 * @return 已配置时返回借用地址；复位后或未配置时返回 NULL。
 * @note 所有入口共享一个私有控制器，须由同一调度所有者串行调用；无硬件访问。地址内容仅在下一 Update/Reset 前有效，不可修改。
 */
const PositionCascadeConfig *PositionCascade_GetConfiguration(void);
/**
 * @brief 复用已验证调参，仅更新目标并执行位置级联调用。
 * @param target_position 有限目标位置，单位 rad。
 * @param measured_position 有限机械位置，单位 rad。
 * @param measured_speed 有限机械速度，单位 rad/s。
 * @param output 调用方非空结果空间；只在计算成功时填充。
 * @return 已有有效配置且本次计算有效返回 true，否则返回 false。
 * @note 所有入口共享一个私有控制器，须由同一调度所有者串行调用；无硬件访问。调参变化必须先通过完整 Update 入口验证。
 */
bool PositionCascade_UpdateTarget(float target_position,
                                  float measured_position,
                                  float measured_speed,
                                  PositionCascadeOutput *output);
/**
 * @brief 复制一致的位置级联诊断快照。
 * @param telemetry 调用方非空输出空间；位置 rad、速度 rad/s、电流 A。
 * @return 参数非空且控制器已初始化时返回 true，否则返回 false。
 * @note 所有入口共享一个私有控制器，须由同一调度所有者串行调用；无硬件访问。
 */
bool PositionCascade_GetTelemetry(PositionCascadeTelemetry *telemetry);
/**
 * @brief 查询本周期是否应推迟可选遥测。
 * @return 完整配置校验或控制更新占用当前周期时返回 true。
 * @note 所有入口共享一个私有控制器，须由同一调度所有者串行调用；无硬件访问。只读提示，不负责调度遥测。
 */
bool PositionCascade_ShouldDeferTelemetry(void);

#endif
