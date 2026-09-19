#ifndef CAN_STATUS_SOURCE_H
#define CAN_STATUS_SOURCE_H
#include <stdbool.h>
#include "motor_status.h"

/* 遥测采集层：通信侧唯一的“读电机状态”出口。
 * 快照在前台按需组装；心跳可见性判定只读当前模式，不修改任何状态。 */

/**
 * @brief 组装当前电机与控制状态的快照（前台按需调用）。
 * @param sample 调用方提供的输出快照；成功时写入全部载荷字段。
 * @note 只读电机/控制/编码器状态，不做 I/O、不阻塞、不分配；
 *       必须在主循环上下文调用，不得在快速环或中断中调用。
 */
void CanStatus_BuildSnapshot(MotorStatus *sample);
/**
 * @brief 判断当前运行模式是否需要心跳看门狗保护。
 * @return 处于电流/速度/位置/阻抗模式返回 true；其余模式返回 false。
 * @note 只读 MotorControl.ModeNow，不修改任何状态；中断与前台均可调用。
 */
bool CanStatus_HeartbeatArmed(void);
#endif
