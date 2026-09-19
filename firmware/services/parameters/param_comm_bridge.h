#ifndef PARAM_COMM_BRIDGE_H
#define PARAM_COMM_BRIDGE_H

#include <stdint.h>

/* 参数持久化与通信运行态之间的窄桥接口。
 * 参数模块（services）不能包含通信头；因此由消费方声明本桥接口，
 * 通信层提供实现并拥有存储（interface_can.c 内的 CANMsg）。接口只用于
 * 参数装载/导出时同步节点身份与心跳超时，不改变任何线协议或编号。 */

/**
 * @brief 读取当前 CAN 节点身份。
 * @return 已生效的节点编号。
 * @note 只读；存储与所有权位于通信层。
 */
uint8_t CAN_NodeId_Get(void);

/**
 * @brief 写入 CAN 节点身份。
 * @param node_id 目标节点编号；调用方保证取值范围合法。
 * @note 只更新运行态存储，不触发保存、不重配总线滤波。
 */
void CAN_NodeId_Set(uint8_t node_id);

/**
 * @brief 读取心跳超时设定。
 * @return 心跳超时，单位 ms；0 表示禁用。
 * @note 只读；存储与所有权位于通信层。
 */
uint32_t CAN_HeartbeatMs_Get(void);

/**
 * @brief 写入心跳超时设定。
 * @param heartbeat_ms 心跳超时，单位 ms；0 表示禁用。
 * @note 只更新运行态存储，不触发保存。
 */
void CAN_HeartbeatMs_Set(uint32_t heartbeat_ms);

#endif
