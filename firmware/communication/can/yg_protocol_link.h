#ifndef YG_PROTOCOL_LINK_H
#define YG_PROTOCOL_LINK_H

#include <stdbool.h>
#include <stdint.h>
#include "bsp_can.h"
#include "loader_entry.h"
#include "yg_protocol_job.h"
#include "yg_protocol_motor.h"
#include "yg_protocol_parameter.h"
#include "yg_protocol_readonly.h"
#include "yg_protocol_common.h"

/**
 * @brief 唯一 CAN FD RX FIFO0 中断入口。
 * @note 单次最多取 3 帧，只复制扩展 FD+BRS 帧并入队，不执行 CRC、分片或电机业务。
 */
void YgProtocolCanfd_RxIrqHandler(void);

/**
 * @brief 初始化唯一CAN FD协议的软件状态和冻结节点配置，不启动硬件。
 * @param node_id 本机节点号；不得使用广播地址。
 * @return 协议端点初始化成功返回 true。
 * @note 仅在前台启动阶段调用一次；成功后由APP绑定真实业务，再调用Start。
 *       已成功初始化后再次调用返回 false，不清除队列或重新启动硬件。
 *       参数桥保存的是配置值；启动后修改参数桥不会改变 link 已冻结的地址。
 */
bool YgProtocolRuntime_Init(uint8_t node_id);

/**
 * @brief 在全部业务绑定完成后启动CAN FD硬件接收。
 * @return 软件已初始化且平台启动正常返回时返回true；未初始化或link不可用返回false。
 * @note 仅前台启动阶段调用；已启动时幂等返回true，不重新配置外设或清队列。
 *       平台启动失败由既有平台错误处理负责；本函数不恢复电机、不绑定默认业务。
 */
bool YgProtocolRuntime_Start(void);

/**
 * @brief 推进 CAN FD 协议并按秒限频恢复 bus-off。
 * @param now_ms 单调毫秒时钟，允许 uint32_t 回绕。
 * @note 只在前台调用；未Start不处理；一次最多处理8个输入和8个输出，无等待。
 */
void YgProtocolRuntime_Service(uint32_t now_ms);

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
 *       初始化或撤销后1/2查询回复4B result6/detail0，不提供占位版本或虚构能力。
 *       不访问UID寄存器，不探测硬件；组合层从平台端口和业务所有者收集数据。
 */
bool YgProtocolLink_SetDeviceDescription(const yg_protocol_readonly_info_page0_t *info,
                                         const yg_protocol_readonly_caps_page0_t *capabilities);
/**
 * @brief 发布GET_INFO维护页快照。
 * @param page 1为生产身份与启动状态，2为目标与Flash布局。
 * @param payload 组合层编码的32B只读页，返回后不保留指针。
 * @return 端点已初始化且输入非空返回true。
 * @note 前台串行调用；通信层不访问Flash或参数所有者。
 */
bool YgProtocolLink_SetMaintenanceInfoPage(uint16_t page, const uint8_t payload[32]);

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
 *       物理发送完成由BspCan跟踪接口提供，普通提交成功不能触发复位。
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
 * @note 由唯一CAN RX IRQ生产；前台消费端负责互斥，生产端允许电机ISR抢占。
 *   不得从第二个并发生产者调用；不执行CRC、路由或业务处理。
 */
bool YgProtocolLink_OnRxFrame(const BspCanFrame *frame);

/**
 * @brief 在主循环后台推进协议收发和只读响应。
 * @param now_ms 单调毫秒时间，用于分片重组截止时间。
 * @note 有界、不等待；一次调用最多处理 8 个输入和 8 个发送帧。116 回复与 124 反馈
 *       成对预留发送空间；受理请求的原始 124 等待所有者实际终态，期间仍处理 STOP/CLEAR。
 *       新目标在旧终态发布前返回 BUSY，重复请求不再次执行。不得在 CAN ISR 调用。
 */
void YgProtocolLink_Service(uint32_t now_ms);

/**
 * @brief 查询链路端点是否已成功初始化。
 * @return 已初始化返回 true，否则返回 false。
 */
bool YgProtocolLink_IsReady(void);

#endif
