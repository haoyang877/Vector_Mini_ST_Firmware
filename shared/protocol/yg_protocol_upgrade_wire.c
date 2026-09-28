/* 升级载荷v1：只做有界字段转换，镜像真实性、存储及会话由产品所有者校验。 */
#include "yg_protocol_upgrade_wire.h"

static uint16_t read_le16(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
}

static uint32_t read_le32(const uint8_t *data)
{
    return (uint32_t)data[0] | ((uint32_t)data[1] << 8U) | ((uint32_t)data[2] << 16U) |
           ((uint32_t)data[3] << 24U);
}

static uint64_t read_le64(const uint8_t *data)
{
    return (uint64_t)read_le32(data) | ((uint64_t)read_le32(data + 4U) << 32U);
}

static void write_le16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8U);
}

static void write_le32(uint8_t *data, uint32_t value)
{
    for (uint8_t i = 0U; i < 4U; ++i)
    {
        data[i] = (uint8_t)(value >> (8U * i));
    }
}

static void write_le64(uint8_t *data, uint64_t value)
{
    write_le32(data, (uint32_t)value);
    write_le32(data + 4U, (uint32_t)(value >> 32U));
}

static bool valid_envelope(const yg_protocol_message_t *message)
{
    return message->version == YG_PROTOCOL_VERSION && message->reserved == 0U &&
           message->source_node != YG_PROTOCOL_BROADCAST_NODE_ID &&
           message->destination_node != YG_PROTOCOL_BROADCAST_NODE_ID &&
           (message->flags == YG_PROTOCOL_FLAGS_ACK_REQUEST ||
            message->flags == (YG_PROTOCOL_FLAGS_ACK_REQUEST | YG_PROTOCOL_FLAGS_RETRY));
}

static bool valid_result(uint16_t result)
{
    switch (result)
    {
    case YG_PROTOCOL_UPGRADE_RESULT_OK:
    case YG_PROTOCOL_UPGRADE_RESULT_ACCEPTED:
    case YG_PROTOCOL_UPGRADE_RESULT_UNSUPPORTED:
    case YG_PROTOCOL_UPGRADE_RESULT_BAD_LENGTH:
    case YG_PROTOCOL_UPGRADE_RESULT_BAD_FIELD:
    case YG_PROTOCOL_UPGRADE_RESULT_DENIED:
    case YG_PROTOCOL_UPGRADE_RESULT_BUSY:
    case YG_PROTOCOL_UPGRADE_RESULT_UNAUTHORIZED:
    case YG_PROTOCOL_UPGRADE_RESULT_FAILED:
    case YG_PROTOCOL_UPGRADE_RESULT_OUT_OF_RANGE:
    case YG_PROTOCOL_UPGRADE_RESULT_FAULT:
        return true;
    default:
        return false;
    }
}

yg_protocol_result_t YgProtocolUpgrade_DecodeRequest(const yg_protocol_message_t *message,
                                                     YgProtocolUpgradeRequest *request)
{
    YgProtocolUpgradeRequest candidate = {0};
    uint32_t expected_bytes;

    if (message == NULL || request == NULL ||
        (message->payload_length != 0U && message->payload == NULL))
    {
        return YG_PROTOCOL_INVALID_ARGUMENT;
    }
    if (!valid_envelope(message))
    {
        return YG_PROTOCOL_INVALID_FIELD;
    }
    if (message->message_type < YG_PROTOCOL_UPGRADE_TYPE_BEGIN ||
        message->message_type > YG_PROTOCOL_UPGRADE_TYPE_ABORT)
    {
        return YG_PROTOCOL_INVALID_MESSAGE_TYPE;
    }
    candidate.message_type = message->message_type;
    candidate.source_node = message->source_node;
    candidate.sequence = message->sequence;
    if (message->message_type == YG_PROTOCOL_UPGRADE_TYPE_STATUS && message->payload_length == 0U)
    {
        candidate.discover = true;
        *request = candidate;
        return YG_PROTOCOL_OK;
    }
    if (message->message_type == YG_PROTOCOL_UPGRADE_TYPE_BEGIN)
    {
        expected_bytes = YG_PROTOCOL_UPGRADE_BEGIN_BYTES;
    }
    else if (message->message_type == YG_PROTOCOL_UPGRADE_TYPE_WRITE)
    {
        if (message->payload_length < YG_PROTOCOL_UPGRADE_WRITE_PREFIX_BYTES)
        {
            return YG_PROTOCOL_INVALID_LENGTH;
        }
        candidate.data_bytes = read_le16(message->payload + 16U);
        expected_bytes = YG_PROTOCOL_UPGRADE_WRITE_PREFIX_BYTES + candidate.data_bytes;
    }
    else
    {
        expected_bytes = YG_PROTOCOL_UPGRADE_BINDING_BYTES;
    }
    if (message->payload_length != expected_bytes)
    {
        return YG_PROTOCOL_INVALID_LENGTH;
    }
    candidate.boot_epoch = read_le64(message->payload);
    if (candidate.boot_epoch == 0U)
    {
        return YG_PROTOCOL_INVALID_FIELD;
    }
    if (message->message_type == YG_PROTOCOL_UPGRADE_TYPE_BEGIN)
    {
        candidate.data = message->payload + 8U;
        candidate.data_bytes = YG_PROTOCOL_UPGRADE_MANIFEST_BYTES;
    }
    else
    {
        candidate.session_id = read_le32(message->payload + 8U);
        if (candidate.session_id == 0U)
        {
            return YG_PROTOCOL_INVALID_FIELD;
        }
    }
    if (message->message_type == YG_PROTOCOL_UPGRADE_TYPE_WRITE)
    {
        candidate.image_offset_bytes = read_le32(message->payload + 12U);
        if (candidate.data_bytes < YG_PROTOCOL_UPGRADE_ALIGNMENT_BYTES ||
            candidate.data_bytes > YG_PROTOCOL_UPGRADE_MAX_BLOCK_BYTES ||
            candidate.data_bytes % YG_PROTOCOL_UPGRADE_ALIGNMENT_BYTES != 0U ||
            candidate.image_offset_bytes % YG_PROTOCOL_UPGRADE_ALIGNMENT_BYTES != 0U ||
            candidate.image_offset_bytes > UINT32_MAX - candidate.data_bytes)
        {
            return YG_PROTOCOL_INVALID_FIELD;
        }
        candidate.data = message->payload + YG_PROTOCOL_UPGRADE_WRITE_PREFIX_BYTES;
    }
    *request = candidate;
    return YG_PROTOCOL_OK;
}

yg_protocol_result_t YgProtocolUpgrade_EncodeReply(const YgProtocolUpgradeReply *reply,
                                                   uint8_t *payload,
                                                   size_t capacity,
                                                   size_t *written)
{
    const uint8_t flag_mask =
        YG_PROTOCOL_UPGRADE_FLAG_ACTIVATION_PENDING | YG_PROTOCOL_UPGRADE_FLAG_INTEGRITY_ONLY;
    if (reply == NULL || payload == NULL)
    {
        return YG_PROTOCOL_INVALID_ARGUMENT;
    }
    if (!valid_result(reply->result) || reply->phase > YG_PROTOCOL_UPGRADE_PHASE_ERROR ||
        (reply->flags & (uint8_t)~flag_mask) != 0U ||
        (reply->flags & YG_PROTOCOL_UPGRADE_FLAG_INTEGRITY_ONLY) == 0U ||
        ((reply->flags & YG_PROTOCOL_UPGRADE_FLAG_ACTIVATION_PENDING) != 0U &&
         reply->phase != YG_PROTOCOL_UPGRADE_PHASE_VERIFIED) ||
        reply->max_block_bytes < YG_PROTOCOL_UPGRADE_ALIGNMENT_BYTES ||
        reply->max_block_bytes > YG_PROTOCOL_UPGRADE_MAX_BLOCK_BYTES ||
        reply->max_block_bytes % YG_PROTOCOL_UPGRADE_ALIGNMENT_BYTES != 0U ||
        reply->image_bytes % YG_PROTOCOL_UPGRADE_ALIGNMENT_BYTES != 0U ||
        reply->next_offset_bytes % YG_PROTOCOL_UPGRADE_ALIGNMENT_BYTES != 0U ||
        reply->next_offset_bytes > reply->image_bytes)
    {
        return YG_PROTOCOL_INVALID_FIELD;
    }
    if (capacity < YG_PROTOCOL_UPGRADE_REPLY_BYTES)
    {
        return YG_PROTOCOL_BUFFER_TOO_SMALL;
    }
    write_le16(payload, reply->result);
    write_le16(payload + 2U, reply->detail);
    write_le64(payload + 4U, reply->boot_epoch);
    write_le32(payload + 12U, reply->session_id);
    payload[16] = reply->phase;
    payload[17] = reply->flags;
    write_le16(payload + 18U, reply->max_block_bytes);
    write_le32(payload + 20U, reply->next_offset_bytes);
    write_le32(payload + 24U, reply->image_bytes);
    write_le32(payload + 28U, reply->image_token);
    write_le32(payload + 32U, reply->image_version);
    write_le32(payload + 36U, reply->image_role);
    if (written != NULL)
    {
        *written = YG_PROTOCOL_UPGRADE_REPLY_BYTES;
    }
    return YG_PROTOCOL_OK;
}
