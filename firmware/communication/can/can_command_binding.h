#ifndef CAN_COMMAND_BINDING_H
#define CAN_COMMAND_BINDING_H
#include "interface_can.h"

/* 命令入口层：接收中断取帧、帧级校验、线路解码并路由到写路径/读路径。
 * 本层不直接改电机状态，也不生成应答内容。 */

/**
 * @brief 处理一帧已解码的 CAN 命令：前导握手后路由到写路径与读路径。
 * @param param_id 线路参数 ID，取值来自 CAN_PARAM_ID。
 * @param data 已按线路编码还原的 SI 值。
 * @note 接收中断上下文；非有限值与越界索引在路由前被拒收，且不产生副作用。
 */
void CAN_ReceiveMessage_Update(CAN_PARAM_ID param_id, float data);
/**
 * @brief FDCAN RX FIFO0 中断入口：取帧、校验并派发本节点命令，同时刷新心跳。
 * @note 中断上下文；帧级校验失败或非本节点帧直接丢弃，不产生副作用。
 */
void CANRxIRQHandler(void);
#endif
