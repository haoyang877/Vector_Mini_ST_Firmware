#ifndef APP_PROTOCOL_H
#define APP_PROTOCOL_H

/**
 * @brief 更新协议可见的状态、模式、故障与已有测量的单位换算。
 * @note 仅前台串行调用；不执行控制、硬件操作或总线发送。
 */
void AppProtocol_Refresh(void);
#endif
