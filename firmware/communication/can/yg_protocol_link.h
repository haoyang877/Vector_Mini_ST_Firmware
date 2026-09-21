#ifndef YG_PROTOCOL_LINK_H
#define YG_PROTOCOL_LINK_H

#include <stdbool.h>
#include <stdint.h>

#include "comm_hw.h"
#include "yg_protocol_motor.h"
#include "yg_protocol_wire_types.h"

/**
 * @brief 初始化只读 CAN FD 链路诊断端点。
 * @param node_id 本机节点号，不能为广播地址。
 * @return 初始化成功返回 true；参数或静态资源不满足返回 false。
 * @note 只注册 GET_INFO/GET_CAPS，不使能电机、不修改参数；应在板级 CAN 启动后调用。
 */
bool YgProtocolLink_Init(uint8_t node_id);

/**
 * @brief 绑定电机业务服务。
 * @param service 业务所有者提供的服务；传入 NULL 表示解除绑定。
 * @return 已初始化且绑定成功返回 true，否则返回 false。
 * @note 只复制服务指针和上下文，不访问硬件；未绑定时 STOP/DISABLE 返回 UNSUPPORTED。
 */
bool YgProtocolLink_BindMotorService(const yg_protocol_motor_service_t *service);

/**
 * @brief 把硬件接收的扩展 CAN FD 帧放入协议 RX 队列。
 * @param frame 硬件端口提供的完整帧，函数返回后不保存指针。
 * @return 接收并入队返回 true；帧格式非法或队列满返回 false。
 * @note 由唯一 FDCAN RX 入口调用；不执行 CRC、路由或业务处理。
 */
bool YgProtocolLink_OnRxFrame(const CommHwCanFrame *frame);

/**
 * @brief 在监督上下文推进协议收发和只读响应。
 * @param now_ms 单调毫秒时间，用于分片重组截止时间。
 * @note 有界、不等待、不访问电机状态；一次调用最多处理 4 个输入和 4 个发送帧。
 */
void YgProtocolLink_Service(uint32_t now_ms);

/**
 * @brief 查询链路端点是否已成功初始化。
 * @return 已初始化返回 true，否则返回 false。
 */
bool YgProtocolLink_IsReady(void);

#endif
