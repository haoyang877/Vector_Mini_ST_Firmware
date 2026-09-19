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
    /*最近一帧已解码命令（诊断镜像；派发经 can_transport 定长环）*/
    CAN_PARAM_ID rx_param_id;
    float rx_data;
    bool can_rx_en;
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
 * @note 每个 2 kHz 监督 tick 调用一次；仅在电流/速度/位置/阻抗模式下武装，
 *       已锁存的故障保留，本函数不做清除。
 */
void CAN_DisConnect_Handle(void);
/**
 * @brief 检测波特率设置变化并切换 FDCAN1 波特率。
 * @note 由 2 kHz 监督 tick 调用；设置未变化时不访问硬件。
 */
void CAN_BaudRateSwitching(void);
/**
 * @brief 处理一帧已解码的 CAN 命令：按参数 ID 更新电机/服务状态或生成应答。
 * @param param_id 线路参数 ID，取值来自 CAN_PARAM_ID。
 * @param data 已按线路编码还原的 SI 值；非有限值由各命令的接受条件决定是否生效。
 * @note 仅由 2 kHz 服务（CAN_Service）从接收队列派发；写路径在短临界区内应用
 *       以恢复对 20 kHz 快环的原子性，读路径只生成应答、不屏蔽中断。
 */
void CAN_ReceiveMessage_Update(CAN_PARAM_ID param_id, float data);
/**
 * @brief 按回复编码规则编码一帧应答并排入发送队列（FIFO）。
 * @param param_id 回复参数 ID，决定定点编码与负载长度。
 * @param data 待回复的 SI 值；非有限值按哨兵编码。
 * @note 仅 2 kHz 派发上下文调用；队列满时丢弃最新并计数，不等待、不访问硬件。
 */
void CAN_SendMessage_Update(CAN_PARAM_ID param_id, float data);
/**
 * @brief 只读判断控制心跳是否仍然存活（未被看门狗判为断连）。
 * @return 已启用心跳且计数未达超时阈值返回 true；心跳未启用或已超时返回 false。
 * @note 前台与中断均可调用；只读运行态，不修改计数、不触发故障。
 */
bool CAN_IsHeartbeatAlive(void);
/**
 * @brief 2 kHz 通信服务：排空接收队列并派发，再排空发送队列，最后按周期提交状态流。
 * @note 仅由 2 kHz 监督 tick 调用；有界、不等待、不分配。应答优先于状态流：
 *       发送队列非空时不提交 48 字节状态帧。
 */
void CAN_Service(void);
/**
 * @brief 读取接收队列丢帧计数。
 * @return 自启动以来接收队列满导致的累计丢帧数；只增不减。
 */
uint32_t CAN_GetRxDropCount(void);
/**
 * @brief 读取发送队列丢帧计数。
 * @return 自启动以来发送队列满导致的累计丢帧数；只增不减。
 */
uint32_t CAN_GetTxDropCount(void);
/**
 * @brief FDCAN RX FIFO0 中断入口：取帧、校验、解码并刷新心跳，命令入接收队列。
 * @note 中断上下文；帧级校验失败或非本节点帧直接丢弃，不产生副作用。
 */
void CANRxIRQHandler(void);
#endif
