#ifndef MOTOR_LOAD_CONTROL_H
#define MOTOR_LOAD_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

/** @brief 提交装配配置，实际生效由下一次电机快中断处理。
 * @param flags 配置 0=wheel，1=roll 前馈关，3=roll 前馈开。
 * @return 合法且当前已停机、关相、无待处理请求时为 true。
 * @note CAN 上下文单生产者；不保存 Flash、不启动电机，提交后必须查询生效值。
 */
bool MotorControl_RequestLoadProfile(uint32_t flags);

/** @brief 查询快环已生效的装配配置。
 * @return 配置位；损坏或尚未应用的配置返回 -1。
 * @note 只读，不访问 Flash 或触发运动。
 */
int32_t MotorControl_GetLoadProfile(void);

#endif
