#ifndef YG_PROTOCOL_MOTOR_H
#define YG_PROTOCOL_MOTOR_H

#include "yg_protocol_service.h"
#include "yg_protocol_wire_types.h"

/* 项目候选 type；公司电机 100～199 登记完成后再冻结线上编号。 */
#define YG_PROTOCOL_MOTOR_TYPE_STOP 110U
#define YG_PROTOCOL_MOTOR_TYPE_DISABLE 111U
#define YG_PROTOCOL_MOTOR_REPLY_SIZE 16U

/** @brief motor 内部操作，不分配线上 type，不等同于电机模式枚举。 */
typedef enum
{
    YG_PROTOCOL_MOTOR_STOP = 0,
    YG_PROTOCOL_MOTOR_DISABLE,
    YG_PROTOCOL_MOTOR_ENABLE,
    YG_PROTOCOL_MOTOR_SET_MODE,
    YG_PROTOCOL_MOTOR_SET_TARGET,
} yg_protocol_motor_operation_t;

/** @brief 位置、速度、电流使用 mrad、mrad/s、mA；绝对执行时间由本地同步时钟解释。 */
typedef struct
{
    yg_protocol_motor_operation_t operation;
    uint8_t source_node;
    uint16_t sequence;
    uint8_t mode;
    int32_t position_mrad;
    int32_t speed_mrad_s;
    int32_t iq_mA;
    uint32_t execute_at_us;
    uint32_t lease_ms;
} yg_protocol_motor_request_t;

/**
 * @brief 业务所有者实现的非阻塞处理入口。
 * @param context 后端拥有的状态，生命周期覆盖调用。
 * @param request 只读请求；返回后不可保留其指针。
 * @param reply 后端填充的值结果，必须设置 status。
 * @note 前台串行调用；权限、租约、状态和范围由后端校验，不得直接把线上输入写入控制变量。
 */
typedef void (*yg_protocol_motor_handler_t)(void *context,
                                            const yg_protocol_motor_request_t *request,
                                            yg_protocol_service_reply_t *reply);

/** @brief 由组合层注入的后端绑定；handler 为 NULL 表示该能力未实现。 */
typedef struct
{
    void *context;
    yg_protocol_motor_handler_t handler;
} yg_protocol_motor_service_t;

/**
 * @brief 向已绑定后端提交一个内部请求，未绑定返回 UNSUPPORTED。
 * @param service 后端绑定对象。
 * @param request 内部值请求。
 * @param reply 输出执行结果，不代表 CAN 帧已经发送或物理动作已完成。
 * @return 与 reply.status 相同的内部结果；参数非法时返回 INVALID_ARGUMENT。
 * @note 只调用一次后端，不等待、不重试、不分配。异步操作返回 ACCEPTED 和 token。
 */
yg_protocol_service_status_t yg_protocol_motor_call(const yg_protocol_motor_service_t *service,
                                                    const yg_protocol_motor_request_t *request,
                                                    yg_protocol_service_reply_t *reply);

/**
 * @brief 编码电机业务结果响应。
 * @param reply 内部服务结果。
 * @param payload 输出的小端 payload 缓冲区。
 * @param capacity 输出容量，至少 16 字节。
 * @param written 成功时写入长度，可为 NULL。
 * @return 编码结果。
 * @note 该 payload 是项目候选格式；内部服务枚举不直接暴露给电机控制模块。
 */
yg_protocol_result_t yg_protocol_motor_encode_reply(const yg_protocol_service_reply_t *reply,
                                                     uint8_t *payload,
                                                     size_t capacity,
                                                     size_t *written);

#endif
