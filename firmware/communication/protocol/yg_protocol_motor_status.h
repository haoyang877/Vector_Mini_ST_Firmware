#ifndef YG_PROTOCOL_MOTOR_STATUS_H
#define YG_PROTOCOL_MOTOR_STATUS_H

#include <stdbool.h>
#include <stddef.h>

#include "motor_status.h"
#include "yg_protocol_readonly_payload.h"

#define YG_PROTOCOL_MOTOR_MEASUREMENT_MASK 0x77U
#define YG_PROTOCOL_MOTOR_RESPONSE_SIZE 46U
#define YG_PROTOCOL_MOTOR_FEEDBACK_PAYLOAD_SIZE 45U
#define YG_PROTOCOL_MOTOR_FEEDBACK_VALID_MASK 0x0FFFU

/**
 * @brief 调用方拥有的只读快照与线上元数据，不含硬件句柄。
 * @note sample 的 SI 测量来自同一次采样；state/mode/faults 必须由所有者映射到线上枚举，
 *       不能直接复制 sample.mode/fault。measurement_valid_bits 使用详细状态的位 0/1/2/4/5。
 *       sample_available 由调度者检查新鲜度后设置；本模块不读时钟或消费 MotorStatus 邮箱。
 *       boot_id、sample_counter、target_applied 和执行序号必须与样本关联，不能使用查询次数。
 */
typedef struct
{
    MotorStatus sample;
    uint32_t boot_id;
    uint32_t sample_counter;
    uint32_t faults;
    uint8_t state;
    uint8_t mode;
    uint8_t last_applied_sequence;
    uint8_t measurement_valid_bits;
    bool target_applied;
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
 * @brief 将现有 SI 测量副本转换为详细状态值对象。
 * @param source 调用方提供的稳定快照及已映射的线上元数据。
 * @param state 输出状态，失败保持不变。
 * @return 成功、无新鲜样本或参数错误。
 * @note 半单位远离零取整；NaN/Inf/超范围置无效哨兵并清有效位，不饱和。仅在前台串行调用。
 */
yg_protocol_result_t
yg_protocol_motor_status_convert(const yg_protocol_motor_status_source_t *source,
                                 yg_protocol_readonly_motor_state_t *state);

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
 * @note 不读取硬件或控制目标；无电机温度、Vq/Vd 测量源时相应位保持无效。
 */
yg_protocol_result_t
yg_protocol_motor_feedback_from_source(const yg_protocol_motor_status_source_t *source,
                                       uint8_t node_id,
                                       uint16_t result,
                                       uint16_t correlated_seq,
                                       yg_protocol_motor_feedback_t *feedback);

/**
 * @brief 为只读服务提供 GET_MOTOR_STATE 的完整 R+32B 响应。
 * @param context 指向稳定的 yg_protocol_motor_status_source_t，调用期间不得被并发写入。
 * @param request 已校验的单播查询；仅支持 session=0，request_id 非零。
 * @param payload 输出完整管理响应；失败保持不变。
 * @param payload_length 输入容量，成功输出 44 或 12 字节。
 * @return 成功表示已生成业务响应，业务错误在 R.result；本地失败禁止发送。
 * @note 无新鲜样本生成 BUSY；不请求样本、不刷新运动看门狗、不读全局电机状态。
 */
yg_protocol_result_t yg_protocol_motor_status_provider(void *context,
                                                       const yg_protocol_message_t *request,
                                                       uint8_t *payload,
                                                       uint32_t *payload_length);

#endif
