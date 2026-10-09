#ifndef POSITION_CASCADE_H
#define POSITION_CASCADE_H

#include <stdbool.h>
#include <stdint.h>

/* 基础串级配置：位置 rad、速度 rad/s、时间 s、电流 A；旧速度增益仍乘电流上限。 */
typedef struct
{
    float update_period_s;
    uint16_t call_divider;
    float target_position;
    float position_error_window;
    float velocity_filter_hz;
    float maximum_speed;
    float speed_limit_rad_s;
    float position_kp;
    float position_kd;
    float speed_kp;
    float speed_ki;
    float current_limit_a;
} PositionCascadeConfig;

/* 仅执行结果；position_reference 等于目标，没有内部运动轨迹。 */
typedef struct
{
    float position_reference;
    float speed_reference;
    float speed_feedback_rad_s;
    float iq_reference;
    bool target_reached;
} PositionCascadeControlOutput;

/**
 * @brief 验证基础配置并执行一次直接位置串级调用，保留分频。
 * @param config 非空只读配置；位置 rad、速度 rad/s、时间 s、电流 A。
 * @param measured_position 有限机械位置，单位 rad。
 * @param measured_speed 有限机械速度，单位 rad/s。
 * @param output 非空结果空间；仅在成功时填充。
 * @return 输入及计算有效返回 true，否则返回 false，不发布新结果。
 * @note 单实例串行调用，无硬件访问；下调限流立即生效，配置更新不清支撑积分。
 */
bool PositionCascade_UpdateControl(const PositionCascadeConfig *config,
                                   float measured_position,
                                   float measured_speed,
                                   PositionCascadeControlOutput *output);

/**
 * @brief 复用有效基础配置，仅更新目标并执行直接位置串级调用。
 * @param target_position 有限目标位置，单位 rad。
 * @param measured_position 有限机械位置，单位 rad。
 * @param measured_speed 有限机械速度，单位 rad/s。
 * @param output 非空结果空间；仅在成功时填充。
 * @return 已配置且输入/计算有效返回 true，否则返回 false。
 * @note 单实例串行调用；调参须先通过 UpdateControl，目标改变不复位 PI 或速度滤波。
 */
bool PositionCascade_UpdateTargetControl(float target_position,
                                         float measured_position,
                                         float measured_speed,
                                         PositionCascadeControlOutput *output);

/**
 * @brief 清零 PI、滤波和分频状态，撤销配置有效性。
 * @note 单实例串行调用；用于启动/模式重置，不执行硬件操作。
 */
void PositionCascade_Reset(void);

/**
 * @brief 借用最近有效的基础配置。
 * @return 已配置时返回只读借用地址，否则返回 NULL。
 * @note 地址内容仅在下一更新或 Reset 前有效，不可修改；单实例串行调用。
 */
const PositionCascadeConfig *PositionCascade_GetConfiguration(void);

#endif
