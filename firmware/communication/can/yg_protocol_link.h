#ifndef YG_PROTOCOL_LINK_H
#define YG_PROTOCOL_LINK_H

#include <stdbool.h>
#include <stdint.h>

#include "comm_hw.h"
#include "loader_entry.h"
#include "yg_protocol_job.h"
#include "yg_protocol_motor.h"
#include "yg_protocol_motor_status.h"
#include "yg_protocol_parameter.h"
#include "yg_protocol_readonly_payload.h"
#include "yg_protocol_wire_types.h"

/**
 * @brief 初始化 CAN FD 链路端点和独占队列。
 * @param node_id 本机节点号，不能为广播地址。
 * @return 初始化成功返回 true；参数或静态资源不满足返回 false。
 * @note 注册信息、能力、状态、101/107/110/116、120～122与160～162路由，不自动绑定业务服务。
 *       必须在CAN RX启动前调用；
 *       link 独占路由、队列、去重历史与待发回复；初始化会撤销旧业务和快照绑定。
 *       运行中重置须先停止 RX 和后台服务；runtime 管理硬件启动，不由 link 启动外设。
 */
bool YgProtocolLink_Init(uint8_t node_id);

/**
 * @brief 设置由产品组合层确认的真实设备身份和已实现能力。
 * @param info 身份和版本值；与capabilities同时为NULL时撤销描述。
 * @param capabilities 真实可用能力值，不以候选Type或编码器存在推断功能。
 * @return 已初始化且两个输入同时有效或同时为NULL时返回true，否则false。
 * @note 后台串行调用，不与Service并发；复制现有值缓冲，不保留输入指针。
 *       初始化或撤销后1/2查询返回DATA_UNAVAILABLE，不提供占位版本或虚构能力。
 *       不访问UID寄存器，不探测硬件；组合层从平台端口和业务所有者收集数据。
 */
bool YgProtocolLink_SetDeviceDescription(const yg_protocol_readonly_info_page0_t *info,
                                         const yg_protocol_readonly_caps_page0_t *capabilities);

/**
 * @brief 绑定Type3所需的真实启动交接快照。
 * @param source APP所有者的稳定只读值，NULL恢复APP角色且epoch不可用的快照。
 * @return 已初始化且绑定成功返回true，否则false。
 * @note 后台串行调用，不与Service并发；借用指针，调用方须保持生命周期与查询期间不可变。
 *       不自行分配epoch，不把硬件UID或u32计数升级为u64启动身份。
 */
bool YgProtocolLink_BindDeviceStatusSource(const yg_protocol_readonly_device_status_t *source);

/**
 * @brief 绑定电机业务服务。
 * @param service 业务所有者提供的服务；传入 NULL 表示解除绑定。
 * @return 已初始化且绑定成功返回 true，否则返回 false。
 * @note 仅由后台调用且不得与 Service 并发；只复制指针和上下文，未绑定时停机返回 UNSUPPORTED。
 */
bool YgProtocolLink_BindMotorService(const yg_protocol_motor_service_t *service);

/**
 * @brief 绑定活动参数、版本及异步保存的业务所有者。
 * @param service 调用方持有的服务绑定，传入NULL解除绑定。
 * @return 链路已初始化且绑定成功时返回true，否则false。
 * @note 后台串行调用，只复制绑定值；不复制参数记录，不在通信调用栈擦写Flash。
 *       所有者须按完整请求身份实现写/保存幂等，未绑定明确返回UNSUPPORTED。
 */
bool YgProtocolLink_BindParameterService(const yg_protocol_parameter_service_t *service);

/**
 * @brief 绑定标定任务所有者，通信只传递任务类别、请求身份及原始任务号。
 * @param service 产品提供的任务绑定，NULL撤销绑定。
 * @return 已初始化且绑定成功返回true，否则false。
 * @note 后台串行调用；START重试须返回原ticket，取消须核对ticket，不能全局取消新任务。
 *       不复制标定工作区、不执行算法或Flash，未绑定返回UNSUPPORTED。
 */
bool YgProtocolLink_BindJobService(const yg_protocol_job_service_t *service);

/**
 * @brief 在静默配置阶段绑定真实APP进入Loader所有者并开放56。
 * @param service 共享业务契约，request与tx_complete必须都有实现；NULL撤销。
 * @return 已初始化、队列无在途回复且绑定有效返回true，否则false且保持旧绑定。
 * @note 只在RX启动前或停止RX并清空队列后调用，不与Service并发。
 *       有在途票据不能撤销或替换；不探测Loader，也不将回调存在当成已安装能力。
 *       物理发送完成由comm_hw tracked接口提供，普通提交成功不能触发复位。
 */
bool YgProtocolLink_BindBootService(const BootEntryService *service);

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
 * @note 有界、不等待；一次调用最多处理 8 个输入和 8 个发送帧。116 回复与 124 反馈
 *       成对预留发送空间。不得在 CAN ISR 调用。
 */
void YgProtocolLink_Service(uint32_t now_ms);

/**
 * @brief 查询链路端点是否已成功初始化。
 * @return 已初始化返回 true，否则返回 false。
 */
bool YgProtocolLink_IsReady(void);

#endif
