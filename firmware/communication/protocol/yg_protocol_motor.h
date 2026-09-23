#ifndef YG_PROTOCOL_MOTOR_H
#define YG_PROTOCOL_MOTOR_H

#include "yg_protocol_service.h"
#include "yg_protocol_wire_types.h"

/* 已按项目决策冻结的首批电机 Type；公司消息表待同步登记。 */
#define YG_PROTOCOL_MOTOR_TYPE_STOP 110U
#define YG_PROTOCOL_MOTOR_TYPE_ENABLE 101U
#define YG_PROTOCOL_MOTOR_TYPE_CONTROL 116U
#define YG_PROTOCOL_MOTOR_ENABLE_REQUEST_SIZE 4U
#define YG_PROTOCOL_MOTOR_CONTROL_REQUEST_SIZE 20U
#define YG_PROTOCOL_MOTOR_REPLY_SIZE 4U

/** @brief motor 内部操作，不分配线上 type，不等同于电机模式枚举。 */
typedef enum
{
    YG_PROTOCOL_MOTOR_STOP = 0,
    YG_PROTOCOL_MOTOR_DISABLE,
    YG_PROTOCOL_MOTOR_ENABLE,
    YG_PROTOCOL_MOTOR_SET_CONTROL,
} yg_protocol_motor_operation_t;

/** @brief 单轴控制值对象；位置、速度、电流、电压单位依次为 mrad、mrad/s、mA、mV。 */
typedef struct
{
    yg_protocol_motor_operation_t operation;
    uint8_t source_node;
    uint16_t sequence;
    uint8_t mode;
    int32_t position_mrad;
    int32_t speed_mrad_s;
    int32_t iq_mA;
    int32_t v_q_mV;
} yg_protocol_motor_request_t;

/**
 * @brief 解码 101 使能请求；只解析字段，不执行功率级动作。
 * @param message 已通过统一帧 CRC 校验的请求。
 * @param enabled 成功时输出 0/1 使能值；失败保持不变。
 * @return 长度、字段或参数检查结果。
 */
yg_protocol_result_t yg_protocol_motor_decode_enable(const yg_protocol_message_t *message,
                                                     bool *enabled);

/**
 * @brief 解码 116 模式与目标，按模式检查未使用字段必须为零。
 * @param message 已通过统一帧 CRC 校验的请求。
 * @param request 成功时输出值请求；失败保持不变。
 * @return 长度、字段或参数检查结果。
 * @note 不判断实时位置限位、功率状态或控制权限，这些归业务所有者。
 */
yg_protocol_result_t yg_protocol_motor_decode_control(const yg_protocol_message_t *message,
                                                      yg_protocol_motor_request_t *request);

/**
 * @brief 把内部执行状态映射到已冻结的电机线路结果码。
 * @param status 电机业务服务状态。
 * @return 线路结果码，未知状态返回 8（内部失败）。
 */
uint16_t yg_protocol_motor_result_code(yg_protocol_service_status_t status);

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
 * @param capacity 输出容量，至少 4 字节。
 * @param written 成功时写入长度，可为 NULL。
 * @return 编码结果。
 * @note payload 为 result:u16 + detail:u16；内部服务状态须显式映射到线路结果码。
 */
yg_protocol_result_t yg_protocol_motor_encode_reply(const yg_protocol_service_reply_t *reply,
                                                    uint8_t *payload,
                                                    size_t capacity,
                                                    size_t *written);

#endif
