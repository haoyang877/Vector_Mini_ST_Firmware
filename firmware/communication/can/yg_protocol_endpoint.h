#ifndef YG_PROTOCOL_ENDPOINT_H
#define YG_PROTOCOL_ENDPOINT_H

#include "yg_protocol_common.h"
#include "yg_protocol_canfd.h"
#include "yg_protocol_fragment.h"

/** 处理函数只提交业务请求，不等待电机或 Flash 操作完成。
 * 返回 OK 表示已处理请求；业务成功/拒绝原因写入 response_payload。
 * 默认立即回复；reply_ready=false 时由业务保存后续工作，稍后通过 endpoint_send 发出。
 * request 及 payload 只在调用期间有效，禁止保存借用指针。
 */
typedef yg_protocol_result_t (*yg_protocol_command_handler_t)(void *context,
                                                              const yg_protocol_message_t *request,
                                                              yg_protocol_service_result_t *result);

/** CMD ID、分片许可和执行函数在同一处登记。handler=NULL 表示仅用于发送。 */
typedef struct
{
    uint16_t message_type;
    bool allow_fragment;
    yg_protocol_command_handler_t handler;
    void *context;
} yg_protocol_command_t;

typedef struct
{
    const yg_protocol_command_t *entries;
    size_t count;
} yg_protocol_commands_t;

/**
 * @brief 一次注册静态命令表，不复制、不分配内存。
 * @param commands 输出表视图；失败时保留原绑定。
 * @param entries 静态只读表，使用期间必须保持有效；空表可传 NULL。
 * @param count 表项数量。
 * @return 注册成功或重复编号、参数错误。
 * @note 重新注册仅允许在收发停止且无待处理请求时进行。
 */
yg_protocol_result_t yg_protocol_commands_register(yg_protocol_commands_t *commands,
                                                   const yg_protocol_command_t *entries,
                                                   size_t count);

/**
 * @brief 按 CMD ID 查找命令，不执行处理函数。
 * @param commands 已注册的命令表。
 * @param message_type 请求或发送消息的 CMD ID。
 * @return 找到返回只读表项，未找到返回 NULL。
 */
const yg_protocol_command_t *yg_protocol_commands_find(const yg_protocol_commands_t *commands,
                                                       uint16_t message_type);

/**
 * @brief 执行已查到的命令并初始化回复元数据，不再扫描命令表。
 * @param command 已找到的表项；NULL 或仅发送项不执行业务。
 * @param request 完成帧校验的请求，仅在本次调用期间借用。
 * @param result 输出回复；每次执行前清空，默认立即回复。
 * @return 处理函数结果，或命令未找到、参数错误。
 */
yg_protocol_result_t yg_protocol_commands_execute(const yg_protocol_command_t *command,
                                                  const yg_protocol_message_t *request,
                                                  yg_protocol_service_result_t *result);

/**
 * @brief 将已准备好的结果转换为响应消息。
 * @param request 原请求，提供协议版本。
 * @param result 已完成的服务结果，发送入队前必须保持有效。
 * @param response 输出消息视图，借用 result 中的 payload。
 * @return 转换成功或服务结果、地址、长度错误。
 */
yg_protocol_result_t yg_protocol_commands_build_response(const yg_protocol_message_t *request,
                                                         const yg_protocol_service_result_t *result,
                                                         yg_protocol_message_t *response);

#define YG_PROTOCOL_MAX_MESSAGE_PAYLOAD 1767U

/** @brief 组合层提供的端点资源；队列、命令表、重组存储均需覆盖端点生命周期。 */
typedef struct
{
    uint8_t local_node;
    uint8_t reply_priority;
    uint32_t fragment_timeout_ms;
    yg_protocol_transfer_queue_t *rx;
    yg_protocol_transfer_queue_t *tx;
    const yg_protocol_commands_t *commands;
    uint8_t *fragment_storage;
    size_t fragment_capacity;
} yg_protocol_endpoint_config_t;

/** @brief 单上下文端点；忙时保留一个应答，禁止 ISR 与任务并发调用。 */
typedef struct
{
    yg_protocol_endpoint_config_t config;
    yg_protocol_fragment_state_t fragment;
    yg_protocol_service_result_t service_result;
    yg_protocol_message_t pending_response;
    bool response_pending;
    bool initialized;
} yg_protocol_endpoint_t;

/**
 * @brief 绑定已初始化的独占 RX/TX 队列、命令表和重组存储。
 * @param endpoint 输出端点，初始化失败保持不可用。
 * @param config 配置值；RX/TX 必须为不同队列及不同存储，不允许与 endpoint 重叠。
 * @return 初始化结果。
 * @note 不读取硬件、不启动外设；组装代码负责本地节点、优先级和同步。
 */
yg_protocol_result_t yg_protocol_endpoint_init(yg_protocol_endpoint_t *endpoint,
                                               const yg_protocol_endpoint_config_t *config);

/**
 * @brief 将完整消息编码并复制到 TX 队列，长消息切为每片最多 46B。
 * @param endpoint 已初始化端点。
 * @param message 完整单消息视图，禁止预置分片位；source 必须为本机。
 * @param priority CAN 优先级 0..7，由消息登记/调用方策略确定。
 * @return 入队成功或错误；队列容量不足时不入队任何片段。
 * @note 单次最多编码 39 帧；只在前台串行调用，payload 在返回后可释放。
 *       QUEUED 等价于 OK，不表示发送/执行成功。仅 allow_fragment 的消息可分片。
 */
yg_protocol_result_t yg_protocol_endpoint_send(yg_protocol_endpoint_t *endpoint,
                                               const yg_protocol_message_t *message,
                                               uint8_t priority);

/**
 * @brief 最多处理一个 RX 帧，或重试上次因 TX 满而保留的应答。
 * @param endpoint 已初始化端点。
 * @param now_ms 单调毫秒时间，供分片重组使用。
 * @return 路由/分片/排队结果；无输入返回 QUEUE_EMPTY。
 * @note 仅 reply_ready 且 ACK_REQ、非 RESPONSE 的成功处理会自动构造响应；处理失败不会发送成功应答。
 *       应答最多 46B；更长业务输出须经 send 显式分片。忙时不重复调用业务处理器。
 */
yg_protocol_result_t yg_protocol_endpoint_process_one(yg_protocol_endpoint_t *endpoint,
                                                      uint32_t now_ms);

/**
 * @brief 在后台处理调用方已安全出队的一帧，或推进待发应答。
 * @param endpoint 已初始化的单上下文端点。
 * @param frame 完整帧副本；传入 NULL 仅重试待发应答，不读取 RX 队列。
 * @param now_ms 单调毫秒时间。
 * @return 处理结果；存在待发应答且 frame 非空时返回 QUEUE_FULL，输入未消费。
 * @note 有待发应答时调用方须先传 NULL；帧及 payload 只在本次调用内借用。
 */
yg_protocol_result_t yg_protocol_endpoint_process_frame(yg_protocol_endpoint_t *endpoint,
                                                        const yg_protocol_transfer_frame_t *frame,
                                                        uint32_t now_ms);

/**
 * @brief 丢弃未发送/未处理帧、重组和待发应答，保留绑定及丢帧累计值。
 * @param endpoint 已初始化的独占端点。
 * @return 重置结果。
 * @note 在 bus-off/会话撤销时由组合层调用；不复位硬件或自动恢复电机控制。
 */
yg_protocol_result_t yg_protocol_endpoint_reset(yg_protocol_endpoint_t *endpoint);

#endif
