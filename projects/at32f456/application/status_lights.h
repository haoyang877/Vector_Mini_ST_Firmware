#ifndef STATUS_LIGHTS_H
#define STATUS_LIGHTS_H

#include <stdbool.h>
#include "motor_state.h"

/** @brief 单轴灯语状态；零初始化对应两灯熄灭，仅由监督拍持有。 */
typedef struct
{
    MotorState state;
    bool checking;
    unsigned phase_ticks;
    bool red;
    bool blue;
} StatusLights;

/** @brief 按监督时基推进一拍，并给出这一拍的逻辑亮灭。
 * @param lights 本轴灯语状态。
 * @param state 已提交的轴状态。
 * @param fault 是否有当前锁存故障。
 * @param checking 板级硬件检查是否正在采集。
 * @return 两路输出中至少一路发生变化时为 true。
 */
bool StatusLights_Step(StatusLights *lights, MotorState state, bool fault, bool checking);

#endif
