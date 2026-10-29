#ifndef YG_PROTOCOL_PARAMETER_H
#define YG_PROTOCOL_PARAMETER_H

#include "yg_protocol_service.h"
#include "yg_protocol_common.h"

#define YG_PROTOCOL_PARAMETER_TYPE_READ 120U
#define YG_PROTOCOL_PARAMETER_TYPE_WRITE 121U
#define YG_PROTOCOL_PARAMETER_TYPE_SAVE 122U
#define YG_PROTOCOL_PARAMETER_VALUE_I32 1U
#define YG_PROTOCOL_PARAMETER_VALUE_U32 2U
#define YG_PROTOCOL_PARAMETER_ACCESS_READ 0x01U
#define YG_PROTOCOL_PARAMETER_ACCESS_WRITE 0x02U
#define YG_PROTOCOL_PARAMETER_ACCESS_CALIBRATION 0x04U
/* 参数业务错误的detail，不是C枚举或参数ID；外层result沿用既有结果码。 */
#define YG_PROTOCOL_PARAMETER_DETAIL_UNSUPPORTED_ID 1U
#define YG_PROTOCOL_PARAMETER_DETAIL_READ_ONLY 2U
#define YG_PROTOCOL_PARAMETER_DETAIL_REVISION_CONFLICT 3U
#define YG_PROTOCOL_PARAMETER_DETAIL_PERSIST_FAILED 4U
#define YG_PROTOCOL_PARAMETER_DETAIL_WRONG_STATE 5U
#define YG_PROTOCOL_PARAMETER_DETAIL_BAD_FIELD 6U
#define YG_PROTOCOL_PARAMETER_DETAIL_STALE_SEQUENCE 7U
#define YG_PROTOCOL_PARAMETER_DETAIL_CONFLICTING_RETRY 8U

/** @brief parameter 内部操作，不分配线上 type，不等同于电机模式枚举。 */
typedef enum
{
    YG_PROTOCOL_PARAMETER_READ = 0,
    YG_PROTOCOL_PARAMETER_WRITE,
    YG_PROTOCOL_PARAMETER_SAVE,
    YG_PROTOCOL_PARAMETER_RESTORE_DEFAULTS,
} yg_protocol_parameter_operation_t;

/** @brief value 的单位和表示由参数描述符定义；保存由后端受控调度，不能在调用栈里擦写 Flash。 */
typedef struct
{
    yg_protocol_parameter_operation_t operation;
    uint16_t parameter_id;
    int32_t value;
    uint32_t expected_revision;
    uint8_t value_type;
    uint8_t source_node;
    uint16_t sequence;
} yg_protocol_parameter_request_t;

/**
 * @brief 解码120/121/122标量请求，小端参数ID在线路上仅占一个字节。
 * @param message 已通过统一帧校验的请求。
 * @param request 成功输出值请求；失败不改变输出。
 * @return 字段、长度或消息类型检查结果。
 * @note 不校验运行状态、产品范围、revision或写权限；这些由参数所有者执行。
 */
yg_protocol_result_t yg_protocol_parameter_decode(const yg_protocol_message_t *message,
                                                  yg_protocol_parameter_request_t *request);

/**
 * @brief 编码参数服务的标量回复或4字节明确错误。
 * @param request 已解码的请求，决定READ/WRITE/SAVE结果布局。
 * @param reply 所有者提供的实际值、修订号、类型、权限和结果。
 * @param payload 输出缓冲区。
 * @param capacity 输出容量；成功标量需要16字节，SAVE需要8字节。
 * @param written 成功时返回实际长度，不允许为空。
 * @return 编码结果；无效服务元数据不得伪装为成功。
 * @note SAVE的OK只允许在持久化提交验证完成后产生，ACCEPTED不是保存完成。
 */
yg_protocol_result_t
yg_protocol_parameter_encode_reply(const yg_protocol_parameter_request_t *request,
                                   const yg_protocol_service_reply_t *reply,
                                   uint8_t *payload,
                                   size_t capacity,
                                   size_t *written);

/**
 * @brief 业务所有者实现的非阻塞处理入口。
 * @param context 后端拥有的状态，生命周期覆盖调用。
 * @param request 只读请求；返回后不可保留其指针。
 * @param reply 后端填充的值结果，必须设置 status。
 * @note 前台串行调用；权限、租约、状态和范围由后端校验，不得直接把线上输入写入控制变量。
 */
typedef void (*yg_protocol_parameter_handler_t)(void *context,
                                                const yg_protocol_parameter_request_t *request,
                                                yg_protocol_service_reply_t *reply);

/** @brief 由组合层注入的后端绑定；handler 为 NULL 表示该能力未实现。 */
typedef struct
{
    void *context;
    yg_protocol_parameter_handler_t handler;
} yg_protocol_parameter_service_t;

/**
 * @brief 向已绑定后端提交一个内部请求，未绑定返回 UNSUPPORTED。
 * @param service 后端绑定对象。
 * @param request 内部值请求。
 * @param reply 输出执行结果，不代表 CAN 帧已经发送或物理动作已完成。
 * @return 与 reply.status 相同的内部结果；参数非法时返回 INVALID_ARGUMENT。
 * @note 只调用一次后端，不等待、不重试、不分配。异步操作返回 ACCEPTED 和 token。
 */
yg_protocol_service_status_t
yg_protocol_parameter_call(const yg_protocol_parameter_service_t *service,
                           const yg_protocol_parameter_request_t *request,
                           yg_protocol_service_reply_t *reply);

/**
 * @brief 处理本模块命令并生成回复内容；不直接操作收发队列或硬件。
 * @param context 本模块的上下文或服务绑定，由初始化时注入。
 * @param request 已完成外层校验的消息。
 * @param result 输出回复内容，实际发送由统一端点完成。
 * @return 解析处理结果；业务拒绝通过回复中的结果码报告。
 */
yg_protocol_result_t yg_protocol_parameter_handle(void *context,
                                                  const yg_protocol_message_t *request,
                                                  yg_protocol_service_result_t *result);

#endif
