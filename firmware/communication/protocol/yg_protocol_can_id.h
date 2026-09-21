#ifndef YG_PROTOCOL_CAN_ID_H
#define YG_PROTOCOL_CAN_ID_H

#include <stdint.h>

#include "yg_protocol_wire_types.h"

#define YG_PROTOCOL_CAN_ID_PF 0xEFU

/** @brief 公司 CAN ID 的项目候选字段。PF 和保留位不由调用方修改。 */
typedef struct
{
    uint8_t priority;
    uint8_t source_node;
    uint8_t destination_node;
} yg_protocol_can_id_fields_t;

/**
 * @brief 按项目候选布局编码 29 位扩展 CAN ID。
 * @param fields priority、源节点和目的节点。
 * @param identifier 输出的 29 位 CAN ID。
 * @return 编码结果。
 * @note 当前 PF=0xEF、R=0、DP=0；该映射仍需公司协议登记确认。
 */
yg_protocol_result_t yg_protocol_can_id_encode(const yg_protocol_can_id_fields_t *fields,
                                               uint32_t *identifier);

/**
 * @brief 解码并校验项目候选布局的 29 位扩展 CAN ID。
 * @param identifier 输入的 29 位 CAN ID。
 * @param fields 输出字段。
 * @return 解码结果。
 * @note 拒绝超出 29 位、PF 不匹配或保留位非零的标识符。
 */
yg_protocol_result_t yg_protocol_can_id_decode(uint32_t identifier,
                                               yg_protocol_can_id_fields_t *fields);

#endif
