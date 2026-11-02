#ifndef APP_PROTOCOL_H
#define APP_PROTOCOL_H
#include <stdbool.h>

/**
 * @brief 绑定状态、启停和三类标定协议服务。
 * @return 所有绑定成功时返回true，否则保留失能策略。
 * @note 通信链路初始化后、采样启动前调用一次；不启动电机或执行标定。
 */
bool AppProtocol_Init(void);

/**
 * @brief 更新协议可见的状态、模式、故障与已有测量的单位换算。
 * @note 仅前台串行调用；不执行控制、硬件操作或总线发送。
 */
void AppProtocol_Refresh(void);
#endif
