#ifndef __INTERFACE_CAN_H__
#define __INTERFACE_CAN_H__

#include <stdbool.h>
#include <stdint.h>
#include "data_type.h"
#include "can_parameter_wire.h"

typedef struct
{
    /*node ID*/
    uint8_t node_id;
    /*parameter ID*/
    CAN_PARAM_ID rx_param_id, tx_param_id;
    float rx_data, tx_data;
    uint8_t rx_data_u8[4], tx_data_u8[4];
    /** @brief 当前待回复参数的线路载荷长度，电流为 2，其他为 4。 */
    uint8_t tx_data_len;
    bool can_rx_en;
    bool can_tx_en;
    bool can_hb_en;
    /*heart beat timeout setting (ms)*/
    uint32_t can_hb_set;
    uint32_t can_hb_count;
} CANMsg_TypeDef;

/** @brief CAN 运行态：节点身份、收发暂存与心跳状态；唯一写者是通信模块自身。 */
extern CANMsg_TypeDef CANMsg;

/**
 * @brief 初始化 FDCAN1 接收通道：装载状态流默认配置，按当前节点配置范围滤波并启动。
 * @note 由 Board_Init 在参数装载之后调用；初始化失败按致命处理，不返回。
 */
void FDCAN1_Param_Init(void);
/**
 * @brief 心跳看门狗：按当前运行模式与心跳设置判定断连并置 CAN_DisConnect 故障。
 * @note 每个 1 kHz 监督时基调用一次；仅在电流/速度/位置/阻抗模式下武装，
 *       已锁存的故障保留，本函数不做清除。
 */
void CAN_DisConnect_Handle(void);
/**
 * @brief 检测波特率设置变化并切换 FDCAN1 波特率。
 * @note 由 1 kHz 任务调用；设置未变化时不访问硬件。
 */
void CAN_BaudRateSwitching(void);
/**
 * @brief 处理一帧已解码的 CAN 命令：按参数 ID 更新电机/服务状态或生成应答。
 * @param param_id 线路参数 ID，取值来自 CAN_PARAM_ID。
 * @param data 已按线路编码还原的 SI 值；非有限值由各命令的接受条件决定是否生效。
 * @note 接收中断上下文；只做有界解码与下发，不等待、不分配、不访问 Flash。
 */
void CAN_ReceiveMessage_Update(CAN_PARAM_ID param_id, float data);
/**
 * @brief 按回复编码规则暂存一帧应答载荷，等待前台发送。
 * @param param_id 回复参数 ID，决定定点编码与负载长度。
 * @param data 待回复的 SI 值；非有限值按哨兵编码。
 * @note 接收中断上下文；只写暂存区并置发送使能，不直接访问硬件。
 */
void CAN_SendMessage_Update(CAN_PARAM_ID param_id, float data);
/**
 * @brief 只读判断控制心跳是否仍然存活（未被看门狗判为断连）。
 * @return 已启用心跳且计数未达超时阈值返回 true；心跳未启用或已超时返回 false。
 * @note 前台与中断均可调用；只读运行态，不修改计数、不触发故障。
 */
bool CAN_IsHeartbeatAlive(void);
/**
 * @brief FDCAN RX FIFO0 中断入口：取帧、校验并派发本节点命令，同时刷新心跳。
 * @note 中断上下文；帧级校验失败或非本节点帧直接丢弃，不产生副作用。
 */
void CANRxIRQHandler(void);
/**
 * @brief 前台发送调度：优先发送暂存应答，空闲时按周期提交 48 字节状态流。
 * @note 主循环调用；发送队列忙时丢帧而不等待，状态流周期由协议层决定。
 */
void CAN_SendMessage(void);
#endif
