#ifndef CAN_BINDING_QUERIES_H
#define CAN_BINDING_QUERIES_H
#include "interface_can.h"

/* 命令绑定的读路径：把线路读命令映射为应答暂存。
 * 本层只读状态并生成应答，不修改电机或服务状态。 */

/**
 * @brief 执行读路径命令：按参数 ID 暂存一帧应答。
 * @param param_id 线路参数 ID，取值来自 CAN_PARAM_ID。
 * @param data 已解码的有限 SI 值。
 * @param data_int data 截断为 int 的结果，供索引型查询使用。
 * @note 接收中断上下文；只写应答暂存区，未匹配的 ID 不产生任何副作用。
 */
void CanBinding_ApplyQuery(CAN_PARAM_ID param_id, float data, int data_int);
#endif
