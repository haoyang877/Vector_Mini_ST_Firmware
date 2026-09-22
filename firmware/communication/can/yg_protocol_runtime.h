#ifndef YG_PROTOCOL_RUNTIME_H
#define YG_PROTOCOL_RUNTIME_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief 初始化唯一 CAN FD 协议运行时。
 * @param node_id 本机节点号；不得使用广播地址。
 * @return 协议端点初始化成功返回 true。
 * @note 先初始化协议端点，再启动 CAN FD 硬件；失败时不启动业务调度。
 */
bool YgProtocolRuntime_Init(uint8_t node_id);

/**
 * @brief 推进 CAN FD 协议并按秒限频恢复 bus-off。
 * @param now_ms 单调毫秒时钟，允许 uint32_t 回绕。
 * @note 只在前台调用；一次最多处理 8 个输入和 8 个输出，无等待。
 */
void YgProtocolRuntime_Service(uint32_t now_ms);

#endif
