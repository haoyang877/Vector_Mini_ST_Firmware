#ifndef YG_PROTOCOL_JOB_H
#define YG_PROTOCOL_JOB_H

#include "yg_protocol_service.h"

/** @brief job 内部操作，不分配线上 type，不等同于电机模式枚举。 */
typedef enum
{
    YG_PROTOCOL_JOB_START = 0,
    YG_PROTOCOL_JOB_CANCEL,
    YG_PROTOCOL_JOB_QUERY,
    YG_PROTOCOL_JOB_READ_RESULT,
} yg_protocol_job_operation_t;

/** @brief kind 表示标定/辨识等业务类别；data 仅在调用期间有效，异步后端须复制到有界存储。 */
typedef struct
{
    yg_protocol_job_operation_t operation;
    uint16_t kind;
    uint32_t job_id;
    const uint8_t *data;
    size_t data_length;
} yg_protocol_job_request_t;

/**
 * @brief 业务所有者实现的非阻塞处理入口。
 * @param context 后端拥有的状态，生命周期覆盖调用。
 * @param request 只读请求；返回后不可保留其指针。
 * @param reply 后端填充的值结果，必须设置 status。
 * @note 前台串行调用；权限、租约、状态和范围由后端校验，不得直接把线上输入写入控制变量。
 */
typedef void (*yg_protocol_job_handler_t)(void *context,
                                          const yg_protocol_job_request_t *request,
                                          yg_protocol_service_reply_t *reply);

/** @brief 由组合层注入的后端绑定；handler 为 NULL 表示该能力未实现。 */
typedef struct
{
    void *context;
    yg_protocol_job_handler_t handler;
} yg_protocol_job_service_t;

/**
 * @brief 向已绑定后端提交一个内部请求，未绑定返回 UNSUPPORTED。
 * @param service 后端绑定对象。
 * @param request 内部值请求。
 * @param reply 输出执行结果，不代表 CAN 帧已经发送或物理动作已完成。
 * @return 与 reply.status 相同的内部结果；参数非法时返回 INVALID_ARGUMENT。
 * @note 只调用一次后端，不等待、不重试、不分配。异步操作返回 ACCEPTED 和 token。
 */
yg_protocol_service_status_t yg_protocol_job_call(const yg_protocol_job_service_t *service,
                                                  const yg_protocol_job_request_t *request,
                                                  yg_protocol_service_reply_t *reply);

#endif
