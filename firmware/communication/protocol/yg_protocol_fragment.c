/* 管理、日志和升级消息的固定缓冲区分片重组；实时消息由调用方禁止分片。
 * 规则依据 motor_protocol_v1.md 第 12.1 节：单会话、46B 非尾片、重复片忽略、
 * 片间超时与首片起 2000ms 绝对超时；不提供 ACK 关联或整消息重传。 */
#include "yg_protocol_fragment.h"

#include <string.h>

/* 片间/绝对超时判定：到达 deadline 即超时，差值按 uint32 模运算自然回绕。 */
static bool deadline_reached(uint32_t now_ms, uint32_t start_ms, uint32_t timeout_ms)
{
    return (uint32_t)(now_ms - start_ms) >= timeout_ms;
}

static bool
session_expired(const yg_protocol_fragment_state_t *state, uint32_t now_ms, uint32_t timeout_ms)
{
    return deadline_reached(
               now_ms, state->first_activity_ms, YG_PROTOCOL_FRAGMENT_ABSOLUTE_TIMEOUT_MS) ||
           deadline_reached(now_ms, state->last_activity_ms, timeout_ms);
}

/* 仅 src/dst/type 相同才算同一逻辑通道；其余帧不混入也不破坏活动会话。 */
static bool same_key(const yg_protocol_fragment_state_t *state,
                     const yg_protocol_message_t *message)
{
    return state->source_node == message->source_node &&
           state->destination_node == message->destination_node &&
           state->message_type == message->message_type;
}

/* 尾片可额外携带 ACK_REQ，其余非分片标志必须一致。 */
static bool same_metadata(const yg_protocol_fragment_state_t *state,
                          const yg_protocol_message_t *message)
{
    return same_key(state, message) && state->version == message->version &&
           state->base_flags == (message->flags & (uint8_t)~(YG_PROTOCOL_FLAGS_FRAGMENT_MASK |
                                                             YG_PROTOCOL_FLAGS_ACK_REQUEST)) &&
           state->reserved == message->reserved;
}

/* 上一片（首片或中片）payload 恒为 46B，直接与已存尾部分比较，零额外复制。 */
static bool duplicate_of_previous(const yg_protocol_fragment_state_t *state,
                                  const yg_protocol_message_t *message,
                                  uint8_t fragment_kind)
{
    const uint8_t *tail;
    if (fragment_kind != YG_PROTOCOL_FLAGS_FRAGMENT_MIDDLE || state->next_sequence < 2U)
    {
        return false;
    }
    if (message->sequence != (uint16_t)(state->next_sequence - 1U) ||
        state->length < YG_PROTOCOL_MAX_PAYLOAD_SIZE)
    {
        return false;
    }
    tail = state->storage + state->length - YG_PROTOCOL_MAX_PAYLOAD_SIZE;
    return memcmp(tail, message->payload, YG_PROTOCOL_MAX_PAYLOAD_SIZE) == 0;
}

/* 首片始终位于 storage 起始处，用已有缓冲识别“完全一致重传”。 */
static bool duplicate_of_first(const yg_protocol_fragment_state_t *state,
                               const yg_protocol_message_t *message)
{
    if (message->sequence != 0U || state->length < YG_PROTOCOL_MAX_PAYLOAD_SIZE)
    {
        return false;
    }
    return memcmp(state->storage, message->payload, YG_PROTOCOL_MAX_PAYLOAD_SIZE) == 0;
}

static yg_protocol_result_t append_payload(yg_protocol_fragment_state_t *state,
                                           const yg_protocol_message_t *message)
{
    if (message->payload_length > state->capacity - state->length)
    {
        return YG_PROTOCOL_BUFFER_TOO_SMALL;
    }
    if (message->payload_length > 0U)
    {
        memcpy(&state->storage[state->length], message->payload, message->payload_length);
    }
    state->length += message->payload_length;
    return YG_PROTOCOL_OK;
}

yg_protocol_result_t
yg_protocol_fragment_init(yg_protocol_fragment_state_t *state, uint8_t *storage, size_t capacity)
{
    if (state == NULL || storage == NULL || capacity == 0U)
    {
        return YG_PROTOCOL_INVALID_ARGUMENT;
    }
    state->storage = storage;
    state->capacity = capacity;
    state->length = 0U;
    state->version = 0U;
    state->base_flags = 0U;
    state->reserved = 0U;
    state->source_node = 0U;
    state->destination_node = 0U;
    state->message_type = 0U;
    state->next_sequence = 0U;
    state->first_activity_ms = 0U;
    state->last_activity_ms = 0U;
    state->active = false;
    return YG_PROTOCOL_OK;
}

yg_protocol_result_t yg_protocol_fragment_reset(yg_protocol_fragment_state_t *state)
{
    if (state == NULL)
    {
        return YG_PROTOCOL_INVALID_ARGUMENT;
    }
    state->length = 0U;
    state->active = false;
    state->next_sequence = 0U;
    state->first_activity_ms = 0U;
    state->last_activity_ms = 0U;
    return YG_PROTOCOL_OK;
}

yg_protocol_result_t
yg_protocol_fragment_poll(yg_protocol_fragment_state_t *state, uint32_t now_ms, uint32_t timeout_ms)
{
    if (state == NULL || timeout_ms == 0U)
    {
        return YG_PROTOCOL_INVALID_ARGUMENT;
    }
    if (!state->active)
    {
        return YG_PROTOCOL_OK;
    }
    if (session_expired(state, now_ms, timeout_ms))
    {
        (void)yg_protocol_fragment_reset(state);
        return YG_PROTOCOL_FRAGMENT_TIMEOUT;
    }
    return YG_PROTOCOL_FRAGMENT_IN_PROGRESS;
}

yg_protocol_result_t yg_protocol_fragment_accept(yg_protocol_fragment_state_t *state,
                                                 const yg_protocol_message_t *message,
                                                 uint32_t now_ms,
                                                 uint32_t timeout_ms,
                                                 yg_protocol_message_t *completed)
{
    uint8_t fragment_kind;
    yg_protocol_result_t result;

    if (state == NULL || message == NULL || completed == NULL || timeout_ms == 0U ||
        (message->payload_length > 0U && message->payload == NULL))
    {
        return YG_PROTOCOL_INVALID_ARGUMENT;
    }

    fragment_kind = message->flags & YG_PROTOCOL_FLAGS_FRAGMENT_MASK;
    if (fragment_kind == YG_PROTOCOL_FLAGS_FRAGMENT_SINGLE)
    {
        /* 单帧直接交付，不与任何活动会话发生关系。 */
        *completed = *message;
        return YG_PROTOCOL_OK;
    }

    /* 片长规则先于会话变更校验：非尾片恰 46B，尾片 1..46B。 */
    if (fragment_kind == YG_PROTOCOL_FLAGS_FRAGMENT_FIRST ||
        fragment_kind == YG_PROTOCOL_FLAGS_FRAGMENT_MIDDLE)
    {
        if (message->payload_length != YG_PROTOCOL_MAX_PAYLOAD_SIZE)
        {
            return YG_PROTOCOL_INVALID_LENGTH;
        }
    }
    else if (fragment_kind == YG_PROTOCOL_FLAGS_FRAGMENT_LAST)
    {
        if (message->payload_length == 0U || message->payload_length > YG_PROTOCOL_MAX_PAYLOAD_SIZE)
        {
            return YG_PROTOCOL_INVALID_LENGTH;
        }
    }
    else
    {
        return YG_PROTOCOL_INVALID_LENGTH;
    }

    if ((message->flags & YG_PROTOCOL_FLAGS_ACK_REQUEST) != 0U &&
        (fragment_kind != YG_PROTOCOL_FLAGS_FRAGMENT_LAST ||
         (message->flags & YG_PROTOCOL_FLAGS_RESPONSE) != 0U))
    {
        if (state->active && same_key(state, message))
        {
            (void)yg_protocol_fragment_reset(state);
        }
        return YG_PROTOCOL_FRAGMENT_STATE_ERROR;
    }

    if (state->active && session_expired(state, now_ms, timeout_ms))
    {
        (void)yg_protocol_fragment_reset(state);
        return YG_PROTOCOL_FRAGMENT_TIMEOUT;
    }

    if (fragment_kind == YG_PROTOCOL_FLAGS_FRAGMENT_FIRST)
    {
        if (message->sequence != 0U)
        {
            (void)yg_protocol_fragment_reset(state);
            return YG_PROTOCOL_FRAGMENT_SEQUENCE_ERROR;
        }
        if (state->active)
        {
            if (same_metadata(state, message) && duplicate_of_first(state, message))
            {
                /* 完全一致的首片重传：忽略，不累加也不刷新超时。 */
                return YG_PROTOCOL_FRAGMENT_IN_PROGRESS;
            }
            /* 新首片与在途会话冲突：旧会话和新首片一并丢弃。 */
            (void)yg_protocol_fragment_reset(state);
            return YG_PROTOCOL_FRAGMENT_STATE_ERROR;
        }
        state->version = message->version;
        state->base_flags = message->flags & (uint8_t)~YG_PROTOCOL_FLAGS_FRAGMENT_MASK;
        state->reserved = message->reserved;
        state->source_node = message->source_node;
        state->destination_node = message->destination_node;
        state->message_type = message->message_type;
        state->length = 0U;
        state->active = true;
        state->next_sequence = 1U;
        state->first_activity_ms = now_ms;
        state->last_activity_ms = now_ms;
        result = append_payload(state, message);
        if (result != YG_PROTOCOL_OK)
        {
            (void)yg_protocol_fragment_reset(state);
            return result;
        }
        return YG_PROTOCOL_FRAGMENT_IN_PROGRESS;
    }

    /* 中片/尾片必须续接已开始的会话。 */
    if (!state->active)
    {
        return YG_PROTOCOL_FRAGMENT_STATE_ERROR;
    }
    if (!same_key(state, message))
    {
        /* 不相关分片既不混入也不破坏在途会话。 */
        return YG_PROTOCOL_FRAGMENT_STATE_ERROR;
    }
    if (!same_metadata(state, message))
    {
        /* 同一通道但头部不一致属于内容冲突，终止重组。 */
        (void)yg_protocol_fragment_reset(state);
        return YG_PROTOCOL_FRAGMENT_STATE_ERROR;
    }
    if (message->sequence == state->next_sequence)
    {
        result = append_payload(state, message);
        if (result != YG_PROTOCOL_OK)
        {
            (void)yg_protocol_fragment_reset(state);
            return result;
        }
        state->last_activity_ms = now_ms;
        state->next_sequence = (uint16_t)(state->next_sequence + 1U);
        if (fragment_kind == YG_PROTOCOL_FLAGS_FRAGMENT_MIDDLE)
        {
            return YG_PROTOCOL_FRAGMENT_IN_PROGRESS;
        }
        completed->version = state->version;
        completed->flags = state->base_flags | (message->flags & YG_PROTOCOL_FLAGS_ACK_REQUEST);
        completed->source_node = state->source_node;
        completed->destination_node = state->destination_node;
        completed->message_type = state->message_type;
        completed->sequence = message->sequence;
        completed->payload_length = (uint32_t)state->length;
        completed->reserved = state->reserved;
        completed->payload = state->storage;
        (void)yg_protocol_fragment_reset(state);
        return YG_PROTOCOL_OK;
    }
    if (duplicate_of_previous(state, message, fragment_kind))
    {
        /* 上一片完全一致的重复：不累加也不刷新超时。 */
        return YG_PROTOCOL_FRAGMENT_IN_PROGRESS;
    }
    /* 跳号或同序号内容冲突：终止重组，不留下可被后包续接的状态。 */
    (void)yg_protocol_fragment_reset(state);
    return YG_PROTOCOL_FRAGMENT_SEQUENCE_ERROR;
}
