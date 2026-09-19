#ifndef CAN_COMMAND_BINDING_H
#define CAN_COMMAND_BINDING_H
#include "interface_can.h"

/* 命令入口层：接收中断取帧、帧级校验、线路解码并入接收队列；2 kHz 服务
 * 从队列派发到写路径/读路径。本层不直接改电机状态，也不生成应答内容。 */

/**
 * @brief 处理一帧已解码的 CAN 命令：前导握手后路由到写路径与读路径。
 * @param param_id 线路参数 ID，取值来自 CAN_PARAM_ID。
 * @param data 已按线路编码还原的 SI 值。
 * @note 仅由 2 kHz 服务从接收队列派发；写路径在短临界区内应用以恢复对 20 kHz
 *       快环的原子性，非有限值与越界索引在路由前被拒收。
 */
void CAN_ReceiveMessage_Update(CAN_PARAM_ID param_id, float data);
/**
 * @brief 2 kHz 接收服务：排空接收队列并逐条派发。
 * @note 仅由 2 kHz 监督 tick 调用；有界（每拍最多环容量条）、不等待、不分配。
 */
void CanCommand_ServiceRx(void);
/**
 * @brief FDCAN RX FIFO0 中断入口：取帧、校验、解码并刷新心跳，命令入接收队列。
 * @note 中断上下文；帧级校验失败或非本节点帧直接丢弃，不产生副作用。
 */
void CANRxIRQHandler(void);
#endif
