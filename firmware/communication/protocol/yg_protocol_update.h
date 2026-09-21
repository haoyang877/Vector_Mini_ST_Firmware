#ifndef YG_PROTOCOL_UPDATE_H
#define YG_PROTOCOL_UPDATE_H

#include "yg_protocol_service.h"

/** @brief update 内部操作，不分配线上 type，不等同于电机模式枚举。 */
typedef enum
{
    YG_PROTOCOL_UPDATE_OPEN = 0,
    YG_PROTOCOL_UPDATE_MANIFEST,
    YG_PROTOCOL_UPDATE_BEGIN,
    YG_PROTOCOL_UPDATE_WRITE,
    YG_PROTOCOL_UPDATE_QUERY,
    YG_PROTOCOL_UPDATE_VERIFY,
    YG_PROTOCOL_UPDATE_ACTIVATE,
    YG_PROTOCOL_UPDATE_ABORT,
} yg_protocol_update_operation_t;

/** @brief image_offset 为镜像相对字节偏移；data 仅在调用期间有效，异步后端须复制，通信不直接写 Flash。 */
typedef struct
{
    yg_protocol_update_operation_t operation;
    uint32_t session_id;
    uint32_t image_offset;
    const uint8_t *data;
    size_t data_length;
} yg_protocol_update_request_t;

/**
 * @brief 业务所有者实现的非阻塞处理入口。
 * @param context 后端拥有的状态，生命周期覆盖调用。
 * @param request 只读请求；返回后不可保留其指针。
 * @param reply 后端填充的值结果，必须设置 status。
 * @note 前台串行调用；权限、租约、状态和范围由后端校验，不得直接把线上输入写入控制变量。
 */
typedef void (*yg_protocol_update_handler_t)(void *context,
                                             const yg_protocol_update_request_t *request,
                                             yg_protocol_service_reply_t *reply);

/** @brief 由组合层注入的后端绑定；handler 为 NULL 表示该能力未实现。 */
typedef struct
{
    void *context;
    yg_protocol_update_handler_t handler;
} yg_protocol_update_service_t;

/**
 * @brief 向已绑定后端提交一个内部请求，未绑定返回 UNSUPPORTED。
 * @param service 后端绑定对象。
 * @param request 内部值请求。
 * @param reply 输出执行结果，不代表 CAN 帧已经发送或物理动作已完成。
 * @return 与 reply.status 相同的内部结果；参数非法时返回 INVALID_ARGUMENT。
 * @note 只调用一次后端，不等待、不重试、不分配。异步操作返回 ACCEPTED 和 token。
 */
yg_protocol_service_status_t yg_protocol_update_call(const yg_protocol_update_service_t *service,
                                                     const yg_protocol_update_request_t *request,
                                                     yg_protocol_service_reply_t *reply);

#endif
