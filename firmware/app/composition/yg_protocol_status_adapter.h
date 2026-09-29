#ifndef YG_PROTOCOL_STATUS_ADAPTER_H
#define YG_PROTOCOL_STATUS_ADAPTER_H

#include <stdbool.h>

/**
 * @brief 初始化 CAN FD 电机与本次启动身份查询源。
 * @return 协议端点已就绪并完成状态源绑定返回 true。
 * @note 不启动硬件；调用前必须完成 `YgProtocolLink_Init`。
 */
bool YgProtocolStatusAdapter_Init(void);

/**
 * @brief 从电机运行态和真实启动交接刷新稳定值快照。
 * @note 由主循环在 `YgProtocolLink_Service` 前调用；函数不编码、不发送、不分配。
 */
void YgProtocolStatusAdapter_Refresh(void);

#endif
