#ifndef YG_PROTOCOL_MOTOR_H
#define YG_PROTOCOL_MOTOR_H

#include <stdbool.h>
#include <stddef.h>
#include "motor_status_mailbox.h"
#include "motor_control_command.h"
#include "yg_protocol_common.h"
#include "yg_protocol_service.h"

#define YG_PROTOCOL_MOTOR_MEASUREMENT_MASK 0x77U
#define YG_PROTOCOL_MOTOR_FEEDBACK_PAYLOAD_SIZE 45U
#define YG_PROTOCOL_MOTOR_FEEDBACK_VALID_MASK 0x0FFFU

/**
 * @brief 调用方拥有的只读快照与线上元数据，不含硬件句柄。
 * @note sample 与 control_progress 来自同一短临界区；state/mode/faults 必须由所有者映射到线上枚举，
 *       不能直接复制 sample.mode/fault。measurement_valid_bits 使用详细状态的位 0/1/2/4/5。
 *       sample_available 由调度者检查新鲜度后设置；本模块不读时钟或消费 MotorStatus 邮箱。
 */
typedef struct
{
    MotorStatus sample;
    MotorControlProgress control_progress;
    uint32_t faults;
    uint8_t state;
    uint8_t mode;
    uint8_t measurement_valid_bits;
    bool sample_available;
} yg_protocol_motor_status_source_t;

/**
 * @brief 飞书状态反馈字段的固定单位值对象；结构体内存布局不是线路布局。
 * @note result/关联序号由控制事务提供，valid_bits 的位 0～11 对应十二个测量量。
 *       无效测量由所有者填入对应整数最小值，编码器只写入这些显式值。
 */
typedef struct
{
    uint16_t result;
    uint16_t correlated_seq;
    uint32_t fault_code;
    int32_t reference_position_mrad;
    int32_t actual_position_mrad;
    int32_t reference_speed_mrad_s;
    int32_t actual_speed_mrad_s;
    uint16_t valid_bits;
    int16_t bus_voltage_cV;
    int16_t bus_current_mA;
    int16_t reference_iq_mA;
    int16_t actual_iq_mA;
    int16_t mcu_temperature_centi_c;
    int16_t motor_temperature_centi_c;
    int16_t v_q_mV;
    int16_t v_d_mV;
    uint8_t node_id;
    uint8_t motor_state;
    uint8_t control_mode;
} yg_protocol_motor_feedback_t;

/**
 * @brief 将飞书全量状态值编码为 45B 的 124 反馈 payload。
 * @param feedback 调用方提供的稳定、已缩放状态值；返回后不保留指针。
 * @param payload 输出缓冲区，失败保持不变。
 * @param capacity 缓冲区容量，至少 45 字节。
 * @return 编码结果。
 * @note 小端逐字段编码；本函数不调度也不发送帧。具体单位见反馈契约 v0.2。
 */
yg_protocol_result_t yg_protocol_motor_feedback_encode(const yg_protocol_motor_feedback_t *feedback,
                                                       uint8_t *payload,
                                                       size_t capacity);

/**
 * @brief 从同一稳定电机快照生成 124 全量反馈值对象。
 * @param source 可为空；缺失或无新鲜样本时所有测量字段标为无效。
 * @param node_id 本机电机节点 ID。
 * @param result 本条 116 的线路结果码。
 * @param correlated_seq 本条 116 的公司帧序号。
 * @param feedback 输出反馈值；参数错误时保持不变。
 * @return 成功或节点/参数错误。
 * @note 不读取硬件或控制目标；无电机温度、vq_v/vd_v 测量源时相应位保持无效。
 */
yg_protocol_result_t
yg_protocol_motor_feedback_from_source(const yg_protocol_motor_status_source_t *source,
                                       uint8_t node_id,
                                       uint16_t result,
                                       uint16_t correlated_seq,
                                       yg_protocol_motor_feedback_t *feedback);

/**
 * @brief 为只读服务提供空请求的 108 全量状态响应。
 * @param context 指向稳定的 yg_protocol_motor_status_source_t，调用期间不得被并发写入。
 * @param request 已校验的单播查询；非空 payload 在回复中报长度错误。
 * @param payload 输出与 124 同布局的 45 字节状态；失败保持不变。
 * @param payload_length 输入容量，成功输出 45 字节。
 * @return 成功表示已生成业务响应；本地失败禁止发送。
 * @note 非空请求 result=3；无新鲜样本时 result=6 且测量无效；correlated_seq=0，事务由帧头 seq 关联。
 *       不请求样本、不刷新运动看门狗、不读全局电机状态。
 */
yg_protocol_result_t yg_protocol_motor_status_provider(void *context,
                                                       const yg_protocol_message_t *request,
                                                       uint8_t *payload,
                                                       uint32_t *payload_length);

/* 已按项目决策冻结的首批电机 Type；公司消息表待同步登记。 */
#define YG_PROTOCOL_MOTOR_TYPE_STOP 110U
#define YG_PROTOCOL_MOTOR_TYPE_ENABLE 101U
#define YG_PROTOCOL_MOTOR_TYPE_CLEAR_FAULT 107U
#define YG_PROTOCOL_MOTOR_TYPE_CONTROL 116U
#define YG_PROTOCOL_MOTOR_TYPE_START 117U
#define YG_PROTOCOL_MOTOR_ENABLE_REQUEST_SIZE 4U
#define YG_PROTOCOL_MOTOR_START_REQUEST_SIZE 4U
#define YG_PROTOCOL_MOTOR_CONTROL_REQUEST_SIZE 20U
#define YG_PROTOCOL_MOTOR_OPEN_LOOP_REQUEST_SIZE 32U
#define YG_PROTOCOL_MOTOR_REPLY_SIZE 4U

/** @brief motor 内部操作，不分配线上 type，不等同于电机模式枚举。 */
typedef enum
{
    YG_PROTOCOL_MOTOR_STOP = 0,
    YG_PROTOCOL_MOTOR_DISABLE,
    YG_PROTOCOL_MOTOR_ENABLE,
    YG_PROTOCOL_MOTOR_START_CONTROL,
    YG_PROTOCOL_MOTOR_SET_CONTROL,
    YG_PROTOCOL_MOTOR_CLEAR_FAULT,
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
    int32_t electrical_frequency_mHz, id_mA, vd_mV;
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
 * @brief 解码候选 117 普通启动；仅解析 4B 模式和零保留字段，不使能功率。
 * @param message 已通过统一帧校验的请求。
 * @param request 成功时写入来源、序号和模式；失败保持不变。
 * @return 长度、字段或类型检查结果。
 */
yg_protocol_result_t yg_protocol_motor_decode_start(const yg_protocol_message_t *message,
                                                    yg_protocol_motor_request_t *request);

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

#define YG_PROTOCOL_MOTOR_HISTORY_SIZE 32U
typedef struct
{
    uint16_t sequence;
    uint16_t request_crc;
    uint16_t result;
    /* result=1 时复用为该请求的最终反馈结果；ACK 的 detail 仍为 0。 */
    uint16_t detail;
    bool valid;
} yg_protocol_motor_history_t;

/** @brief 电机通信状态；不拥有电机运行对象，仅引用应用提供的值快照。 */
typedef struct
{
    uint8_t node_id;
    yg_protocol_motor_service_t motor_service;
    yg_protocol_motor_status_source_t empty_motor_status;
    const yg_protocol_motor_status_source_t *motor_status_source;
    yg_protocol_motor_history_t control_history[YG_PROTOCOL_MOTOR_HISTORY_SIZE];
    uint16_t last_control_sequence;
    uint16_t feedback_sequence;
    bool control_sequence_seen;
    bool feedback_pending;
    bool control_completion_pending;
    uint8_t control_completion_source;
    uint16_t control_completion_sequence;
    uint8_t feedback_destination;
    uint8_t feedback_payload[YG_PROTOCOL_MOTOR_FEEDBACK_PAYLOAD_SIZE];
} yg_protocol_motor_context_t;
/**
 * @brief 处理本模块命令并生成回复内容；不直接操作收发队列或硬件。
 * @param context 本模块的上下文或服务绑定，由初始化时注入。
 * @param request 已完成外层校验的消息。
 * @param result 输出回复内容，实际发送由统一端点完成。
 * @return 解析处理结果；业务拒绝通过回复中的结果码报告。
 */
yg_protocol_result_t yg_protocol_motor_handle_action(void *context,
                                                     const yg_protocol_message_t *request,
                                                     yg_protocol_service_result_t *result);
/**
 * @brief 处理本模块命令并生成回复内容；不直接操作收发队列或硬件。
 * @param context 本模块的上下文或服务绑定，由初始化时注入。
 * @param request 已完成外层校验的消息。
 * @param result 输出回复内容，实际发送由统一端点完成。
 * @return 解析处理结果；业务拒绝通过回复中的结果码报告。
 */
yg_protocol_result_t yg_protocol_motor_handle_start(void *context,
                                                    const yg_protocol_message_t *request,
                                                    yg_protocol_service_result_t *result);
/**
 * @brief 处理本模块命令并生成回复内容；不直接操作收发队列或硬件。
 * @param context 本模块的上下文或服务绑定，由初始化时注入。
 * @param request 已完成外层校验的消息。
 * @param result 输出回复内容，实际发送由统一端点完成。
 * @return 解析处理结果；业务拒绝通过回复中的结果码报告。
 */
yg_protocol_result_t yg_protocol_motor_handle_control(void *context,
                                                      const yg_protocol_message_t *request,
                                                      yg_protocol_service_result_t *result);
/**
 * @brief 处理本模块命令并生成回复内容；不直接操作收发队列或硬件。
 * @param context 本模块的上下文或服务绑定，由初始化时注入。
 * @param request 已完成外层校验的消息。
 * @param result 输出回复内容，实际发送由统一端点完成。
 * @return 解析处理结果；业务拒绝通过回复中的结果码报告。
 */
yg_protocol_result_t yg_protocol_motor_handle_status(void *context,
                                                     const yg_protocol_message_t *request,
                                                     yg_protocol_service_result_t *result);

/**
 * @brief 尝试发送待发反馈；队列忙时保留内容，下次调用重试。
 * @param motor 电机通信上下文；状态源必须已绑定。
 * @param send 将消息复制入发送队列的函数。
 * @param send_context 发送接口上下文。
 * @return 无待发消息或入队成功返回 OK，否则返回发送错误。
 */
yg_protocol_result_t yg_protocol_motor_flush_feedback(yg_protocol_motor_context_t *motor,
                                                      yg_protocol_send_t send,
                                                      void *send_context);

#endif
