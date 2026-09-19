#ifndef CAN_BINDING_COMMANDS_H
#define CAN_BINDING_COMMANDS_H
#include "interface_can.h"

/* 命令绑定的写路径：把线路写命令映射为电机/服务状态更新。
 * 本层不做线路编解码，也不发送应答。 */

/**
 * @brief 执行写路径命令：按参数 ID 更新电机/服务状态。
 * @param param_id 线路参数 ID，取值来自 CAN_PARAM_ID。
 * @param data 已解码的有限 SI 值。
 * @param data_int data 截断为 int 的结果，供整数型命令判定取值范围。
 * @note 接收中断上下文；未匹配的 ID 不产生任何副作用，也不发送应答。
 */
void CanBinding_ApplyCommand(CAN_PARAM_ID param_id, float data, int data_int);
#endif
