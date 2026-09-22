#ifndef YG_PROTOCOL_LINK_H
#define YG_PROTOCOL_LINK_H

#include <stdbool.h>
#include <stdint.h>

#include "comm_hw.h"
#include "yg_protocol_motor.h"
#include "yg_protocol_motor_status.h"
#include "yg_protocol_wire_types.h"

/**
 * @brief 初始化 CAN FD 链路端点和独占队列。
 * @param node_id 本机节点号，不能为广播地址。
 * @return 初始化成功返回 true；参数或静态资源不满足返回 false。
 * @note 注册信息、能力、状态查询及停机路由，不绑定生产电机服务。必须在 CAN RX 启动前调用；
 *       运行中重置须先停止 RX 和后台服务。
 */
bool YgProtocolLink_Init(uint8_t node_id);

/**
 * @brief 绑定电机业务服务。
 * @param service 业务所有者提供的服务；传入 NULL 表示解除绑定。
 * @return 已初始化且绑定成功返回 true，否则返回 false。
 * @note 仅由后台调用且不得与 Service 并发；只复制指针和上下文，未绑定时停机返回 UNSUPPORTED。
 */
bool YgProtocolLink_BindMotorService(const yg_protocol_motor_service_t *service);

/**
 * @brief 绑定只读电机状态快照源。
 * @param source 调用方持有的稳定快照；传入 NULL 表示恢复为无新鲜数据状态。
 * @return 已初始化且绑定成功返回 true，否则返回 false。
 * @note 仅由后台调用且不得与 Service 并发；端点借用指针，调用方须在查询期间提供不可变快照。
 */
bool YgProtocolLink_BindMotorStatusSource(const yg_protocol_motor_status_source_t *source);

/**
 * @brief 把硬件接收的扩展 CAN FD 帧放入协议 RX 队列。
 * @param frame 硬件端口提供的完整帧，函数返回后不保存指针。
 * @return 接收并入队返回 true；帧格式非法或队列满返回 false。
 * @note 由唯一 FDCAN RX 入口调用；不执行 CRC、路由或业务处理。
 */
bool YgProtocolLink_OnRxFrame(const CommHwCanFrame *frame);

/**
 * @brief 在主循环后台推进协议收发和只读响应。
 * @param now_ms 单调毫秒时间，用于分片重组截止时间。
 * @note 有界、不等待；一次调用最多处理 8 个输入和 8 个发送帧。不得在 CAN ISR 调用。
 */
void YgProtocolLink_Service(uint32_t now_ms);

/**
 * @brief 查询链路端点是否已成功初始化。
 * @return 已初始化返回 true，否则返回 false。
 */
bool YgProtocolLink_IsReady(void);

#endif
