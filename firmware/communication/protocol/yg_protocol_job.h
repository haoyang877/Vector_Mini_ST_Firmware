#ifndef YG_PROTOCOL_JOB_H
#define YG_PROTOCOL_JOB_H

#include "yg_protocol_service.h"
#include "yg_protocol_wire_types.h"

#define YG_PROTOCOL_JOB_TYPE_START 160U
#define YG_PROTOCOL_JOB_TYPE_STATUS 161U
#define YG_PROTOCOL_JOB_TYPE_CANCEL 162U
#define YG_PROTOCOL_JOB_REQUEST_SIZE 4U
#define YG_PROTOCOL_JOB_REPLY_SIZE 12U
#define YG_PROTOCOL_JOB_KIND_ENCODER_LUT 1U
#define YG_PROTOCOL_JOB_KIND_ELECTRICAL_ZERO 2U
#define YG_PROTOCOL_JOB_KIND_COGGING 3U
#define YG_PROTOCOL_JOB_KIND_PHASE_RESISTANCE 4U
#define YG_PROTOCOL_JOB_KIND_FRICTION 5U
#define YG_PROTOCOL_JOB_PHASE_NONE 0U
#define YG_PROTOCOL_JOB_PHASE_IN_PROGRESS 1U
#define YG_PROTOCOL_JOB_PHASE_COMPLETED 2U
#define YG_PROTOCOL_JOB_PHASE_CANCEL_REQUESTED 3U
#define YG_PROTOCOL_JOB_PHASE_CANCELLED 4U
#define YG_PROTOCOL_JOB_PHASE_FAILED 5U
#define YG_PROTOCOL_JOB_EFFECT_UNKNOWN 0U
#define YG_PROTOCOL_JOB_EFFECT_UNCOMMITTED 1U
#define YG_PROTOCOL_JOB_EFFECT_COMMITTED 2U
#define YG_PROTOCOL_JOB_DETAIL_UNSUPPORTED_KIND 1U
#define YG_PROTOCOL_JOB_DETAIL_STALE_TOKEN 2U
#define YG_PROTOCOL_JOB_DETAIL_WRONG_STATE 3U
#define YG_PROTOCOL_JOB_DETAIL_CONFLICTING_RETRY 4U
#define YG_PROTOCOL_JOB_DETAIL_STALE_SEQUENCE 5U
#define YG_PROTOCOL_JOB_DETAIL_BAD_FIELD 6U
#define YG_PROTOCOL_JOB_DETAIL_OPERATION_FAILED 7U

/** @brief job 内部操作，不分配线上 type，不等同于电机模式枚举。 */
typedef enum
{
    YG_PROTOCOL_JOB_START = 0,
    YG_PROTOCOL_JOB_CANCEL,
    YG_PROTOCOL_JOB_QUERY,
} yg_protocol_job_operation_t;

/** @brief 任务值请求；kind独立于内部模式，job_id隔离取消与查询，不含工作区指针。 */
typedef struct
{
    yg_protocol_job_operation_t operation;
    uint16_t kind;
    uint32_t job_id;
    uint16_t sequence;
    uint8_t source_node;
} yg_protocol_job_request_t;

/** @brief 任务专用结果；phase/effects为固定线路值，不复用参数revision/value字段。 */
typedef struct
{
    yg_protocol_service_status_t status;
    uint32_t token;
    uint16_t kind;
    uint16_t detail;
    uint8_t phase;
    uint8_t effects;
} yg_protocol_job_reply_t;

/**
 * @brief 解码单帧160～162的任务类别或非零任务号。
 * @param message 已完成帧校验的消息，元数据仍由产品路由校验。
 * @param request 成功时写入值请求，失败保持原内容。
 * @return 4B字段合法返回OK，否则返回参数、Type、长度或字段错误。
 * @note 不解释旧Mode编号、不启动任务；未知非零kind由业务所有者拒绝。
 */
yg_protocol_result_t yg_protocol_job_decode(const yg_protocol_message_t *message,
                                            yg_protocol_job_request_t *request);

/**
 * @brief 编码12B任务结果，查询完成不等于标定成功或已持久化。
 * @param reply 真实任务所有者填充的结果；受理/成功须有非零token与有效kind/phase。
 * @param payload 输出缓冲，不得与reply重叠。
 * @param capacity 输出容量，至少12B。
 * @param written 成功时写入12，可为NULL。
 * @return 编码结果；非法元数据不会写输出或written。
 * @note 数值显式小端，失败可无任务元数据；COMMITTED只由持久所有者确认。
 */
yg_protocol_result_t yg_protocol_job_encode_reply(const yg_protocol_job_reply_t *reply,
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
typedef void (*yg_protocol_job_handler_t)(void *context,
                                          const yg_protocol_job_request_t *request,
                                          yg_protocol_job_reply_t *reply);

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
                                                  yg_protocol_job_reply_t *reply);

#endif
