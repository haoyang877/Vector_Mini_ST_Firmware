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
 * @brief 读取当前协议节点号。
 * @return 已生效的节点号。
 */
uint8_t YgProtocolRuntime_NodeId(void);

/**
 * @brief 写入协议节点号运行态值。
 * @param node_id 目标节点号。
 * @note 只更新参数运行态，不在运行中重配硬件；下次启动时生效。
 */
void YgProtocolRuntime_SetNodeId(uint8_t node_id);

/**
 * @brief 读取保存在参数 ABI 中的通信超时值。
 * @return 超时时间，单位 ms；当前 CAN FD 协议阶段不启用旧心跳看门狗。
 */
uint32_t YgProtocolRuntime_HeartbeatMs(void);

/**
 * @brief 更新参数 ABI 中的通信超时值。
 * @param heartbeat_ms 超时时间，单位 ms。
 * @note 保留参数存储兼容性，不触发旧协议看门狗或总线重配置。
 */
void YgProtocolRuntime_SetHeartbeatMs(uint32_t heartbeat_ms);

#endif
