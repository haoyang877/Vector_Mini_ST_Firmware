/* 项目候选 CAN ID 的纯编码逻辑；PF 和保留位由本模块统一约束。 */
#include "yg_protocol_can_id.h"

#define YG_PROTOCOL_CAN_ID_MAX 0x1FFFFFFFU
#define YG_PROTOCOL_CAN_ID_RESERVED_MASK (1UL << 25U)
#define YG_PROTOCOL_CAN_ID_DP_MASK (1UL << 24U)

yg_protocol_result_t yg_protocol_can_id_encode(const yg_protocol_can_id_fields_t *fields,
                                               uint32_t *identifier)
{
    if (fields == NULL || identifier == NULL)
    {
        return YG_PROTOCOL_INVALID_ARGUMENT;
    }
    if (fields->priority > 7U)
    {
        return YG_PROTOCOL_INVALID_NODE_ID;
    }

    *identifier = ((uint32_t)fields->priority << 26U) | ((uint32_t)YG_PROTOCOL_CAN_ID_PF << 16U) |
                  ((uint32_t)fields->destination_node << 8U) | fields->source_node;
    return YG_PROTOCOL_OK;
}

yg_protocol_result_t yg_protocol_can_id_decode(uint32_t identifier,
                                               yg_protocol_can_id_fields_t *fields)
{
    if (fields == NULL)
    {
        return YG_PROTOCOL_INVALID_ARGUMENT;
    }
    if ((identifier & ~YG_PROTOCOL_CAN_ID_MAX) != 0U ||
        (identifier & (YG_PROTOCOL_CAN_ID_RESERVED_MASK | YG_PROTOCOL_CAN_ID_DP_MASK)) != 0U ||
        ((identifier >> 16U) & 0xFFU) != YG_PROTOCOL_CAN_ID_PF)
    {
        return YG_PROTOCOL_INVALID_NODE_ID;
    }

    fields->priority = (uint8_t)((identifier >> 26U) & 0x07U);
    fields->destination_node = (uint8_t)((identifier >> 8U) & 0xFFU);
    fields->source_node = (uint8_t)(identifier & 0xFFU);
    return YG_PROTOCOL_OK;
}
