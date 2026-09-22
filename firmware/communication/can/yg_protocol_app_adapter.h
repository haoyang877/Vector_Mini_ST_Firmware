#ifndef YG_PROTOCOL_APP_ADAPTER_H
#define YG_PROTOCOL_APP_ADAPTER_H

#include "yg_protocol_motor.h"
#include "yg_protocol_motor_status.h"

/**
 * @brief 初始化协议与现有电机运行所有者之间的窄适配。
 * @return 停机服务、状态源和协议绑定均成功返回 true。
 * @note 只绑定显式接口，不在此处访问 CAN 硬件。
 */
bool YgProtocolAppAdapter_Init(void);

/**
 * @brief 刷新一次电机状态快照供 CAN FD 查询使用。
 * @note 由 2 kHz 监督上下文调用；只采样，不编码、不访问总线。
 */
void YgProtocolAppAdapter_RefreshStatus(void);

#endif
