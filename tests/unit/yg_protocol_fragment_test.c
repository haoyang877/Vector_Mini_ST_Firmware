/* 验证 yg_protocol 分片重组的 12.1 可靠性边界；纯 C 逻辑，不访问硬件。 */
#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "yg_protocol_fragment.h"

static uint8_t pattern_a[YG_PROTOCOL_MAX_PAYLOAD_SIZE];
static uint8_t pattern_b[YG_PROTOCOL_MAX_PAYLOAD_SIZE];
static uint8_t pattern_c[YG_PROTOCOL_MAX_PAYLOAD_SIZE];

static void fill(uint8_t *buffer, size_t length, uint8_t seed)
{
    for (size_t index = 0U; index < length; ++index)
    {
        buffer[index] = (uint8_t)(seed + index);
    }
}

static yg_protocol_message_t make(uint8_t version,
                                  uint8_t flags,
                                  uint8_t source_node,
                                  uint8_t destination_node,
                                  uint16_t message_type,
                                  uint16_t sequence,
                                  uint32_t payload_length,
                                  uint8_t reserved,
                                  const uint8_t *payload)
{
    yg_protocol_message_t message = {version,
                                     flags,
                                     source_node,
                                     destination_node,
                                     message_type,
                                     sequence,
                                     payload_length,
                                     reserved,
                                     payload};
    return message;
}

static void assert_unchanged(const yg_protocol_message_t *before,
                             const yg_protocol_message_t *after)
{
    assert(before->version == after->version);
    assert(before->flags == after->flags);
    assert(before->source_node == after->source_node);
    assert(before->destination_node == after->destination_node);
    assert(before->message_type == after->message_type);
    assert(before->sequence == after->sequence);
    assert(before->payload_length == after->payload_length);
    assert(before->reserved == after->reserved);
    assert(before->payload == after->payload);
}

static yg_protocol_message_t middle_of(uint16_t sequence, uint16_t type, const uint8_t *payload)
{
    return make(YG_PROTOCOL_VERSION,
                YG_PROTOCOL_FLAGS_FRAGMENT_MIDDLE,
                1U,
                2U,
                type,
                sequence,
                YG_PROTOCOL_MAX_PAYLOAD_SIZE,
                0U,
                payload);
}

static void argument_validation(void)
{
    uint8_t storage[128];
    yg_protocol_fragment_state_t state;
    yg_protocol_message_t first = make(YG_PROTOCOL_VERSION,
                                       YG_PROTOCOL_FLAGS_FRAGMENT_FIRST,
                                       1U,
                                       2U,
                                       300U,
                                       0U,
                                       46U,
                                       0U,
                                       pattern_a);
    yg_protocol_message_t last = make(
        YG_PROTOCOL_VERSION, YG_PROTOCOL_FLAGS_FRAGMENT_LAST, 1U, 2U, 300U, 1U, 2U, 0U, pattern_a);
    yg_protocol_message_t completed;

    assert(yg_protocol_fragment_init(NULL, storage, sizeof(storage)) ==
           YG_PROTOCOL_INVALID_ARGUMENT);
    assert(yg_protocol_fragment_init(&state, NULL, sizeof(storage)) ==
           YG_PROTOCOL_INVALID_ARGUMENT);
    assert(yg_protocol_fragment_init(&state, storage, 0U) == YG_PROTOCOL_INVALID_ARGUMENT);
    assert(yg_protocol_fragment_init(&state, storage, sizeof(storage)) == YG_PROTOCOL_OK);
    assert(yg_protocol_fragment_reset(NULL) == YG_PROTOCOL_INVALID_ARGUMENT);
    assert(yg_protocol_fragment_poll(NULL, 0U, 10U) == YG_PROTOCOL_INVALID_ARGUMENT);
    assert(yg_protocol_fragment_poll(&state, 0U, 0U) == YG_PROTOCOL_INVALID_ARGUMENT);
    assert(yg_protocol_fragment_accept(&state, NULL, 0U, 10U, &completed) ==
           YG_PROTOCOL_INVALID_ARGUMENT);
    assert(yg_protocol_fragment_accept(&state, &first, 0U, 0U, &completed) ==
           YG_PROTOCOL_INVALID_ARGUMENT);
    assert(yg_protocol_fragment_accept(&state, &first, 0U, 10U, NULL) ==
           YG_PROTOCOL_INVALID_ARGUMENT);
    first.payload = NULL;
    assert(yg_protocol_fragment_accept(&state, &first, 0U, 10U, &completed) ==
           YG_PROTOCOL_INVALID_ARGUMENT);
    assert(yg_protocol_fragment_init(&state, storage, sizeof(storage)) == YG_PROTOCOL_OK);
    assert(yg_protocol_fragment_accept(&state, &last, 0U, 10U, &completed) ==
           YG_PROTOCOL_FRAGMENT_STATE_ERROR);
}

static void fragment_lengths(void)
{
    uint8_t storage[256];
    yg_protocol_fragment_state_t state;
    yg_protocol_message_t completed;
    yg_protocol_message_t first;
    yg_protocol_message_t message;

    assert(yg_protocol_fragment_init(&state, storage, sizeof(storage)) == YG_PROTOCOL_OK);
    first = make(YG_PROTOCOL_VERSION,
                 YG_PROTOCOL_FLAGS_FRAGMENT_FIRST,
                 1U,
                 2U,
                 300U,
                 0U,
                 45U,
                 0U,
                 pattern_a);
    assert(yg_protocol_fragment_accept(&state, &first, 0U, 200U, &completed) ==
           YG_PROTOCOL_INVALID_LENGTH);
    assert(!state.active);
    message = middle_of(1U, 300U, pattern_a);
    assert(yg_protocol_fragment_accept(&state, &message, 0U, 200U, &completed) ==
           YG_PROTOCOL_FRAGMENT_STATE_ERROR);
    assert(yg_protocol_fragment_accept(&state, &message, 0U, 200U, &completed) ==
           YG_PROTOCOL_FRAGMENT_STATE_ERROR);

    first = make(YG_PROTOCOL_VERSION,
                 YG_PROTOCOL_FLAGS_FRAGMENT_FIRST,
                 1U,
                 2U,
                 300U,
                 0U,
                 46U,
                 0U,
                 pattern_a);
    assert(yg_protocol_fragment_accept(&state, &first, 0U, 200U, &completed) ==
           YG_PROTOCOL_FRAGMENT_IN_PROGRESS);
    message = middle_of(1U, 300U, pattern_a);
    message.payload_length = 45U;
    assert(yg_protocol_fragment_accept(&state, &message, 10U, 200U, &completed) ==
           YG_PROTOCOL_INVALID_LENGTH);
    message.payload_length = 46U;
    assert(yg_protocol_fragment_accept(&state, &message, 10U, 200U, &completed) ==
           YG_PROTOCOL_FRAGMENT_IN_PROGRESS);
    message = make(
        YG_PROTOCOL_VERSION, YG_PROTOCOL_FLAGS_FRAGMENT_LAST, 1U, 2U, 300U, 2U, 0U, 0U, pattern_b);
    assert(yg_protocol_fragment_accept(&state, &message, 20U, 200U, &completed) ==
           YG_PROTOCOL_INVALID_LENGTH);
    assert(state.active);
    message.payload_length = 47U;
    assert(yg_protocol_fragment_accept(&state, &message, 20U, 200U, &completed) ==
           YG_PROTOCOL_INVALID_LENGTH);
    message.payload_length = 1U;
    assert(yg_protocol_fragment_accept(&state, &message, 20U, 200U, &completed) == YG_PROTOCOL_OK);
    assert(completed.payload_length == 93U);
}

static void first_sequence_must_be_zero(void)
{
    uint8_t storage[256];
    yg_protocol_fragment_state_t state;
    yg_protocol_message_t completed;
    yg_protocol_message_t first = make(YG_PROTOCOL_VERSION,
                                       YG_PROTOCOL_FLAGS_FRAGMENT_FIRST,
                                       1U,
                                       2U,
                                       300U,
                                       1U,
                                       46U,
                                       0U,
                                       pattern_a);

    assert(yg_protocol_fragment_init(&state, storage, sizeof(storage)) == YG_PROTOCOL_OK);
    assert(yg_protocol_fragment_accept(&state, &first, 0U, 200U, &completed) ==
           YG_PROTOCOL_FRAGMENT_SEQUENCE_ERROR);
    assert(!state.active);
    assert(yg_protocol_fragment_accept(&state, &first, 0U, 200U, &completed) ==
           YG_PROTOCOL_FRAGMENT_SEQUENCE_ERROR);
}

static void metadata_conflict_terminates(void)
{
    uint8_t storage[256];
    yg_protocol_fragment_state_t state;
    yg_protocol_message_t completed;
    yg_protocol_message_t first = make(YG_PROTOCOL_VERSION,
                                       YG_PROTOCOL_FLAGS_FRAGMENT_FIRST,
                                       1U,
                                       2U,
                                       300U,
                                       0U,
                                       46U,
                                       0U,
                                       pattern_a);
    yg_protocol_message_t conflict;

    assert(yg_protocol_fragment_init(&state, storage, sizeof(storage)) == YG_PROTOCOL_OK);
    assert(yg_protocol_fragment_accept(&state, &first, 0U, 200U, &completed) ==
           YG_PROTOCOL_FRAGMENT_IN_PROGRESS);
    conflict = middle_of(1U, 300U, pattern_b);
    conflict.version = (uint8_t)(YG_PROTOCOL_VERSION + 1U);
    assert(yg_protocol_fragment_accept(&state, &conflict, 10U, 200U, &completed) ==
           YG_PROTOCOL_FRAGMENT_STATE_ERROR);
    assert(!state.active);
    assert(yg_protocol_fragment_accept(&state, &conflict, 10U, 200U, &completed) ==
           YG_PROTOCOL_FRAGMENT_STATE_ERROR);

    assert(yg_protocol_fragment_accept(&state, &first, 100U, 200U, &completed) ==
           YG_PROTOCOL_FRAGMENT_IN_PROGRESS);
    conflict = middle_of(1U, 300U, pattern_b);
    conflict.flags |= YG_PROTOCOL_FLAGS_RETRY;
    assert(yg_protocol_fragment_accept(&state, &conflict, 110U, 200U, &completed) ==
           YG_PROTOCOL_FRAGMENT_STATE_ERROR);
    assert(!state.active);

    assert(yg_protocol_fragment_accept(&state, &first, 200U, 200U, &completed) ==
           YG_PROTOCOL_FRAGMENT_IN_PROGRESS);
    conflict = middle_of(1U, 300U, pattern_b);
    conflict.reserved = 0x5AU;
    assert(yg_protocol_fragment_accept(&state, &conflict, 210U, 200U, &completed) ==
           YG_PROTOCOL_FRAGMENT_STATE_ERROR);
    assert(!state.active);
}

static void unrelated_fragment_does_not_mix(void)
{
    uint8_t storage[256];
    yg_protocol_fragment_state_t state;
    yg_protocol_message_t completed;
    yg_protocol_message_t before = {0x11U, 0x22U, 0x33U, 0x44U, 0x55U, 0x66U, 0U, 0x77U, NULL};
    yg_protocol_message_t first = make(YG_PROTOCOL_VERSION,
                                       YG_PROTOCOL_FLAGS_FRAGMENT_FIRST,
                                       1U,
                                       2U,
                                       300U,
                                       0U,
                                       46U,
                                       0U,
                                       pattern_a);
    yg_protocol_message_t unrelated;
    yg_protocol_message_t message;

    assert(yg_protocol_fragment_init(&state, storage, sizeof(storage)) == YG_PROTOCOL_OK);
    completed = before;
    assert(yg_protocol_fragment_accept(&state, &first, 0U, 200U, &completed) ==
           YG_PROTOCOL_FRAGMENT_IN_PROGRESS);
    assert_unchanged(&before, &completed);
    unrelated = make(YG_PROTOCOL_VERSION,
                     YG_PROTOCOL_FLAGS_FRAGMENT_MIDDLE,
                     1U,
                     2U,
                     301U,
                     1U,
                     46U,
                     0U,
                     pattern_b);
    assert(yg_protocol_fragment_accept(&state, &unrelated, 10U, 200U, &completed) ==
           YG_PROTOCOL_FRAGMENT_STATE_ERROR);
    assert(state.active && state.message_type == 300U);
    assert_unchanged(&before, &completed);
    unrelated = make(YG_PROTOCOL_VERSION,
                     YG_PROTOCOL_FLAGS_FRAGMENT_MIDDLE,
                     9U,
                     2U,
                     300U,
                     1U,
                     46U,
                     0U,
                     pattern_b);
    assert(yg_protocol_fragment_accept(&state, &unrelated, 20U, 200U, &completed) ==
           YG_PROTOCOL_FRAGMENT_STATE_ERROR);
    assert(state.active);
    message = middle_of(1U, 300U, pattern_b);
    assert(yg_protocol_fragment_accept(&state, &message, 30U, 200U, &completed) ==
           YG_PROTOCOL_FRAGMENT_IN_PROGRESS);
    message = make(
        YG_PROTOCOL_VERSION, YG_PROTOCOL_FLAGS_FRAGMENT_LAST, 1U, 2U, 300U, 2U, 3U, 0U, pattern_c);
    assert(yg_protocol_fragment_accept(&state, &message, 40U, 200U, &completed) == YG_PROTOCOL_OK);
    assert(completed.payload_length == 95U);
}

static void sequence_gap_terminates(void)
{
    uint8_t storage[256];
    yg_protocol_fragment_state_t state;
    yg_protocol_message_t completed;
    yg_protocol_message_t first = make(YG_PROTOCOL_VERSION,
                                       YG_PROTOCOL_FLAGS_FRAGMENT_FIRST,
                                       1U,
                                       2U,
                                       300U,
                                       0U,
                                       46U,
                                       0U,
                                       pattern_a);
    yg_protocol_message_t message;

    assert(yg_protocol_fragment_init(&state, storage, sizeof(storage)) == YG_PROTOCOL_OK);
    assert(yg_protocol_fragment_accept(&state, &first, 0U, 200U, &completed) ==
           YG_PROTOCOL_FRAGMENT_IN_PROGRESS);
    message = middle_of(1U, 300U, pattern_b);
    assert(yg_protocol_fragment_accept(&state, &message, 10U, 200U, &completed) ==
           YG_PROTOCOL_FRAGMENT_IN_PROGRESS);
    message = middle_of(3U, 300U, pattern_c);
    assert(yg_protocol_fragment_accept(&state, &message, 20U, 200U, &completed) ==
           YG_PROTOCOL_FRAGMENT_SEQUENCE_ERROR);
    assert(!state.active);
    /* 缺片后到达的“下一片”不能续接已终止的重组。 */
    message = middle_of(2U, 300U, pattern_c);
    assert(yg_protocol_fragment_accept(&state, &message, 30U, 200U, &completed) ==
           YG_PROTOCOL_FRAGMENT_STATE_ERROR);
    /* 期望序号 1 的首个中片必须接收；尚无同序号上一片可供重复比较。 */
    assert(yg_protocol_fragment_accept(&state, &first, 40U, 200U, &completed) ==
           YG_PROTOCOL_FRAGMENT_IN_PROGRESS);
    message = middle_of(1U, 300U, pattern_c);
    assert(yg_protocol_fragment_accept(&state, &message, 50U, 200U, &completed) ==
           YG_PROTOCOL_FRAGMENT_IN_PROGRESS);
    assert(state.active);
}

static void duplicate_previous_ignored_no_refresh(void)
{
    uint8_t storage[256];
    yg_protocol_fragment_state_t state;
    yg_protocol_message_t completed;
    yg_protocol_message_t first = make(YG_PROTOCOL_VERSION,
                                       YG_PROTOCOL_FLAGS_FRAGMENT_FIRST,
                                       1U,
                                       2U,
                                       300U,
                                       0U,
                                       46U,
                                       0U,
                                       pattern_a);
    yg_protocol_message_t middle = middle_of(1U, 300U, pattern_b);

    assert(yg_protocol_fragment_init(&state, storage, sizeof(storage)) == YG_PROTOCOL_OK);
    assert(yg_protocol_fragment_accept(&state, &first, 0U, 200U, &completed) ==
           YG_PROTOCOL_FRAGMENT_IN_PROGRESS);
    assert(yg_protocol_fragment_accept(&state, &middle, 100U, 200U, &completed) ==
           YG_PROTOCOL_FRAGMENT_IN_PROGRESS);
    assert(state.length == 92U);
    /* 相同上一片在片间超时内重复出现：不累加、不刷新 last_activity。 */
    assert(yg_protocol_fragment_accept(&state, &middle, 250U, 200U, &completed) ==
           YG_PROTOCOL_FRAGMENT_IN_PROGRESS);
    assert(state.length == 92U && state.last_activity_ms == 100U);
    /* 若重复片刷新了超时，此片会被接受；实际应按原 100ms 基准超时。 */
    assert(yg_protocol_fragment_accept(&state, &middle, 350U, 200U, &completed) ==
           YG_PROTOCOL_FRAGMENT_TIMEOUT);
    assert(!state.active);

    assert(yg_protocol_fragment_accept(&state, &first, 1000U, 200U, &completed) ==
           YG_PROTOCOL_FRAGMENT_IN_PROGRESS);
    assert(yg_protocol_fragment_accept(&state, &middle, 1100U, 200U, &completed) ==
           YG_PROTOCOL_FRAGMENT_IN_PROGRESS);
    middle = middle_of(2U, 300U, pattern_c);
    assert(yg_protocol_fragment_accept(&state, &middle, 1290U, 200U, &completed) ==
           YG_PROTOCOL_FRAGMENT_IN_PROGRESS);
}

static void first_duplicate_and_conflict(void)
{
    uint8_t storage[256];
    yg_protocol_fragment_state_t state;
    yg_protocol_message_t completed;
    yg_protocol_message_t first = make(YG_PROTOCOL_VERSION,
                                       YG_PROTOCOL_FLAGS_FRAGMENT_FIRST,
                                       1U,
                                       2U,
                                       300U,
                                       0U,
                                       46U,
                                       0U,
                                       pattern_a);
    yg_protocol_message_t message;

    assert(yg_protocol_fragment_init(&state, storage, sizeof(storage)) == YG_PROTOCOL_OK);
    assert(yg_protocol_fragment_accept(&state, &first, 0U, 200U, &completed) ==
           YG_PROTOCOL_FRAGMENT_IN_PROGRESS);
    /* 完全一致的首片重传被忽略，长度不变。 */
    assert(yg_protocol_fragment_accept(&state, &first, 50U, 200U, &completed) ==
           YG_PROTOCOL_FRAGMENT_IN_PROGRESS);
    assert(state.length == 46U && state.last_activity_ms == 0U);
    /* 已推进到中片后，原始首片的完全一致重传仍被忽略。 */
    message = middle_of(1U, 300U, pattern_b);
    assert(yg_protocol_fragment_accept(&state, &message, 60U, 200U, &completed) ==
           YG_PROTOCOL_FRAGMENT_IN_PROGRESS);
    assert(yg_protocol_fragment_accept(&state, &first, 70U, 200U, &completed) ==
           YG_PROTOCOL_FRAGMENT_IN_PROGRESS);
    assert(state.length == 92U);
    /* 内容不同的新首片与会话冲突：旧会话和新首片都被丢弃。 */
    first.payload = pattern_c;
    assert(yg_protocol_fragment_accept(&state, &first, 80U, 200U, &completed) ==
           YG_PROTOCOL_FRAGMENT_STATE_ERROR);
    assert(!state.active);
    message = middle_of(2U, 300U, pattern_c);
    assert(yg_protocol_fragment_accept(&state, &message, 90U, 200U, &completed) ==
           YG_PROTOCOL_FRAGMENT_STATE_ERROR);
    /* 冲突后可以用新首片重新开始。 */
    assert(yg_protocol_fragment_accept(&state, &first, 100U, 200U, &completed) ==
           YG_PROTOCOL_FRAGMENT_IN_PROGRESS);
    message = make(
        YG_PROTOCOL_VERSION, YG_PROTOCOL_FLAGS_FRAGMENT_LAST, 1U, 2U, 300U, 1U, 3U, 0U, pattern_b);
    assert(yg_protocol_fragment_accept(&state, &message, 110U, 200U, &completed) == YG_PROTOCOL_OK);
    assert(completed.payload_length == 49U && memcmp(completed.payload, pattern_c, 46U) == 0);
}

static void per_fragment_timeout_exact(void)
{
    uint8_t storage[256];
    yg_protocol_fragment_state_t state;
    yg_protocol_message_t completed;
    yg_protocol_message_t first = make(YG_PROTOCOL_VERSION,
                                       YG_PROTOCOL_FLAGS_FRAGMENT_FIRST,
                                       1U,
                                       2U,
                                       300U,
                                       0U,
                                       46U,
                                       0U,
                                       pattern_a);
    yg_protocol_message_t message;

    assert(yg_protocol_fragment_init(&state, storage, sizeof(storage)) == YG_PROTOCOL_OK);
    assert(yg_protocol_fragment_accept(&state, &first, 0U, 200U, &completed) ==
           YG_PROTOCOL_FRAGMENT_IN_PROGRESS);
    /* deadline 前一毫秒仍可续接。 */
    message = middle_of(1U, 300U, pattern_b);
    assert(yg_protocol_fragment_accept(&state, &message, 199U, 200U, &completed) ==
           YG_PROTOCOL_FRAGMENT_IN_PROGRESS);
    /* 恰好到达 deadline 即超时。 */
    message = middle_of(2U, 300U, pattern_c);
    assert(yg_protocol_fragment_accept(&state, &message, 399U, 200U, &completed) ==
           YG_PROTOCOL_FRAGMENT_TIMEOUT);
    assert(!state.active);
}

static void absolute_timeout_from_first(void)
{
    uint8_t storage[256];
    yg_protocol_fragment_state_t state;
    yg_protocol_message_t completed;
    yg_protocol_message_t first = make(YG_PROTOCOL_VERSION,
                                       YG_PROTOCOL_FLAGS_FRAGMENT_FIRST,
                                       1U,
                                       2U,
                                       300U,
                                       0U,
                                       46U,
                                       0U,
                                       pattern_a);
    yg_protocol_message_t message;

    assert(yg_protocol_fragment_init(&state, storage, sizeof(storage)) == YG_PROTOCOL_OK);
    assert(yg_protocol_fragment_accept(&state, &first, 0U, 5000U, &completed) ==
           YG_PROTOCOL_FRAGMENT_IN_PROGRESS);
    message = middle_of(1U, 300U, pattern_b);
    assert(yg_protocol_fragment_accept(&state, &message, 1500U, 5000U, &completed) ==
           YG_PROTOCOL_FRAGMENT_IN_PROGRESS);
    /* 片间超时未到，但距首片已 2500ms，绝对超时生效且不被新片延长。 */
    message = middle_of(2U, 300U, pattern_c);
    assert(yg_protocol_fragment_accept(&state, &message, 2500U, 5000U, &completed) ==
           YG_PROTOCOL_FRAGMENT_TIMEOUT);
    assert(!state.active);

    /* 恰好 2000ms 即绝对超时。 */
    assert(yg_protocol_fragment_accept(&state, &first, 10000U, 5000U, &completed) ==
           YG_PROTOCOL_FRAGMENT_IN_PROGRESS);
    message = middle_of(1U, 300U, pattern_b);
    assert(yg_protocol_fragment_accept(&state, &message, 12000U, 5000U, &completed) ==
           YG_PROTOCOL_FRAGMENT_TIMEOUT);
    assert(!state.active);
}

static void timeout_wraparound(void)
{
    uint8_t storage[256];
    yg_protocol_fragment_state_t state;
    yg_protocol_message_t completed;
    yg_protocol_message_t first = make(YG_PROTOCOL_VERSION,
                                       YG_PROTOCOL_FLAGS_FRAGMENT_FIRST,
                                       1U,
                                       2U,
                                       300U,
                                       0U,
                                       46U,
                                       0U,
                                       pattern_a);
    yg_protocol_message_t message;

    assert(yg_protocol_fragment_init(&state, storage, sizeof(storage)) == YG_PROTOCOL_OK);
    assert(yg_protocol_fragment_accept(&state, &first, 0xFFFFFF00UL, 5000U, &completed) ==
           YG_PROTOCOL_FRAGMENT_IN_PROGRESS);
    message = middle_of(1U, 300U, pattern_b);
    assert(yg_protocol_fragment_accept(&state, &message, 0x00000064UL, 5000U, &completed) ==
           YG_PROTOCOL_FRAGMENT_IN_PROGRESS);
    /* 回绕后距首片 0x900=2304ms，超过 2000ms 绝对超时。 */
    message = middle_of(2U, 300U, pattern_c);
    assert(yg_protocol_fragment_accept(&state, &message, 0x00000800UL, 5000U, &completed) ==
           YG_PROTOCOL_FRAGMENT_TIMEOUT);
    assert(!state.active);
}

static void completed_unchanged_on_error(void)
{
    uint8_t storage[256];
    yg_protocol_fragment_state_t state;
    uint8_t sentinel[4] = {0xDEU, 0xADU, 0xBEU, 0xEFU};
    yg_protocol_message_t before = {
        0x77U, 0x88U, 0x99U, 0xAAU, 0xBBBBU, 0xCCCCU, 4U, 0xDDU, sentinel};
    yg_protocol_message_t completed = before;
    yg_protocol_message_t first = make(YG_PROTOCOL_VERSION,
                                       YG_PROTOCOL_FLAGS_FRAGMENT_FIRST,
                                       1U,
                                       2U,
                                       300U,
                                       0U,
                                       46U,
                                       0U,
                                       pattern_a);
    yg_protocol_message_t message = middle_of(5U, 300U, pattern_c);

    assert(yg_protocol_fragment_init(&state, storage, sizeof(storage)) == YG_PROTOCOL_OK);
    assert(yg_protocol_fragment_accept(&state, &first, 0U, 200U, &completed) ==
           YG_PROTOCOL_FRAGMENT_IN_PROGRESS);
    assert_unchanged(&before, &completed);
    assert(yg_protocol_fragment_accept(&state, &message, 10U, 200U, &completed) ==
           YG_PROTOCOL_FRAGMENT_SEQUENCE_ERROR);
    assert_unchanged(&before, &completed);
    assert(yg_protocol_fragment_accept(&state, &first, 20U, 200U, &completed) ==
           YG_PROTOCOL_FRAGMENT_IN_PROGRESS);
    message = middle_of(1U, 300U, pattern_b);
    assert(yg_protocol_fragment_accept(&state, &message, 30U, 200U, &completed) ==
           YG_PROTOCOL_FRAGMENT_IN_PROGRESS);
    assert_unchanged(&before, &completed);
    assert(yg_protocol_fragment_accept(&state, &message, 40U, 200U, &completed) ==
           YG_PROTOCOL_FRAGMENT_IN_PROGRESS);
    assert_unchanged(&before, &completed);
}

static void single_insertion(void)
{
    uint8_t storage[256];
    yg_protocol_fragment_state_t state;
    uint8_t single_payload[3] = {1U, 2U, 3U};
    yg_protocol_message_t completed;
    yg_protocol_message_t first = make(YG_PROTOCOL_VERSION,
                                       YG_PROTOCOL_FLAGS_FRAGMENT_FIRST,
                                       1U,
                                       2U,
                                       300U,
                                       0U,
                                       46U,
                                       0U,
                                       pattern_a);
    yg_protocol_message_t message;
    yg_protocol_message_t single =
        make(YG_PROTOCOL_VERSION, 0U, 1U, 2U, 124U, 9U, 3U, 0U, single_payload);

    assert(yg_protocol_fragment_init(&state, storage, sizeof(storage)) == YG_PROTOCOL_OK);
    assert(yg_protocol_fragment_accept(&state, &first, 0U, 200U, &completed) ==
           YG_PROTOCOL_FRAGMENT_IN_PROGRESS);
    assert(yg_protocol_fragment_accept(&state, &single, 5U, 200U, &completed) == YG_PROTOCOL_OK);
    assert(completed.message_type == 124U && completed.payload_length == 3U &&
           completed.payload == single_payload);
    assert(state.active && state.message_type == 300U);
    message = middle_of(1U, 300U, pattern_b);
    assert(yg_protocol_fragment_accept(&state, &message, 10U, 200U, &completed) ==
           YG_PROTOCOL_FRAGMENT_IN_PROGRESS);
    message = make(
        YG_PROTOCOL_VERSION, YG_PROTOCOL_FLAGS_FRAGMENT_LAST, 1U, 2U, 300U, 2U, 2U, 0U, pattern_c);
    assert(yg_protocol_fragment_accept(&state, &message, 20U, 200U, &completed) == YG_PROTOCOL_OK);
    assert(completed.payload_length == 94U && memcmp(completed.payload, pattern_a, 46U) == 0 &&
           memcmp(completed.payload + 46U, pattern_b, 46U) == 0 &&
           memcmp(completed.payload + 92U, pattern_c, 2U) == 0);

    /* 零长度单帧不携带 payload，也不影响会话。 */
    assert(yg_protocol_fragment_init(&state, storage, sizeof(storage)) == YG_PROTOCOL_OK);
    single.payload_length = 0U;
    single.payload = NULL;
    assert(yg_protocol_fragment_accept(&state, &single, 0U, 200U, &completed) == YG_PROTOCOL_OK);
    assert(!state.active);
}

static void poll_semantics(void)
{
    uint8_t storage[256];
    yg_protocol_fragment_state_t state;
    yg_protocol_message_t completed;
    yg_protocol_message_t first = make(YG_PROTOCOL_VERSION,
                                       YG_PROTOCOL_FLAGS_FRAGMENT_FIRST,
                                       1U,
                                       2U,
                                       300U,
                                       0U,
                                       46U,
                                       0U,
                                       pattern_a);

    assert(yg_protocol_fragment_init(&state, storage, sizeof(storage)) == YG_PROTOCOL_OK);
    assert(yg_protocol_fragment_poll(&state, 0U, 200U) == YG_PROTOCOL_OK);
    assert(yg_protocol_fragment_accept(&state, &first, 1000U, 200U, &completed) ==
           YG_PROTOCOL_FRAGMENT_IN_PROGRESS);
    assert(yg_protocol_fragment_poll(&state, 1199U, 200U) == YG_PROTOCOL_FRAGMENT_IN_PROGRESS);
    assert(yg_protocol_fragment_poll(&state, 1200U, 200U) == YG_PROTOCOL_FRAGMENT_TIMEOUT);
    assert(!state.active);
    assert(yg_protocol_fragment_poll(&state, 1200U, 200U) == YG_PROTOCOL_OK);
}

static void capacity_bounded(void)
{
    uint8_t storage[100];
    yg_protocol_fragment_state_t state;
    yg_protocol_message_t completed;
    yg_protocol_message_t first = make(YG_PROTOCOL_VERSION,
                                       YG_PROTOCOL_FLAGS_FRAGMENT_FIRST,
                                       1U,
                                       2U,
                                       300U,
                                       0U,
                                       46U,
                                       0U,
                                       pattern_a);
    yg_protocol_message_t message;

    assert(yg_protocol_fragment_init(&state, storage, sizeof(storage)) == YG_PROTOCOL_OK);
    assert(yg_protocol_fragment_accept(&state, &first, 0U, 200U, &completed) ==
           YG_PROTOCOL_FRAGMENT_IN_PROGRESS);
    message = middle_of(1U, 300U, pattern_b);
    assert(yg_protocol_fragment_accept(&state, &message, 10U, 200U, &completed) ==
           YG_PROTOCOL_FRAGMENT_IN_PROGRESS);
    message = middle_of(2U, 300U, pattern_c);
    assert(yg_protocol_fragment_accept(&state, &message, 20U, 200U, &completed) ==
           YG_PROTOCOL_BUFFER_TOO_SMALL);
    assert(!state.active);
    assert(yg_protocol_fragment_accept(&state, &message, 30U, 200U, &completed) ==
           YG_PROTOCOL_FRAGMENT_STATE_ERROR);
}

static void golden_reassembly(void)
{
    uint8_t storage[256];
    yg_protocol_fragment_state_t state;
    yg_protocol_message_t completed;
    yg_protocol_message_t first = make(YG_PROTOCOL_VERSION,
                                       YG_PROTOCOL_FLAGS_FRAGMENT_FIRST,
                                       1U,
                                       2U,
                                       300U,
                                       0U,
                                       46U,
                                       0U,
                                       pattern_a);
    yg_protocol_message_t message;

    /* 46 + 46 + 3：尾片保留 1..46B 且 completed 清除分片位。 */
    assert(yg_protocol_fragment_init(&state, storage, sizeof(storage)) == YG_PROTOCOL_OK);
    assert(yg_protocol_fragment_accept(&state, &first, 0U, 200U, &completed) ==
           YG_PROTOCOL_FRAGMENT_IN_PROGRESS);
    message = middle_of(1U, 300U, pattern_b);
    assert(yg_protocol_fragment_accept(&state, &message, 10U, 200U, &completed) ==
           YG_PROTOCOL_FRAGMENT_IN_PROGRESS);
    message = make(
        YG_PROTOCOL_VERSION, YG_PROTOCOL_FLAGS_FRAGMENT_LAST, 1U, 2U, 300U, 2U, 3U, 0U, pattern_c);
    assert(yg_protocol_fragment_accept(&state, &message, 20U, 200U, &completed) == YG_PROTOCOL_OK);
    assert(completed.payload_length == 95U);
    assert((completed.flags & YG_PROTOCOL_FLAGS_FRAGMENT_MASK) == 0U);
    assert(memcmp(completed.payload, pattern_a, 46U) == 0);
    assert(memcmp(completed.payload + 46U, pattern_b, 46U) == 0);
    assert(memcmp(completed.payload + 92U, pattern_c, 3U) == 0);

    /* payload 恰为 46 的整数倍：尾片直接标记为 LAST，不补空片。 */
    assert(yg_protocol_fragment_init(&state, storage, sizeof(storage)) == YG_PROTOCOL_OK);
    assert(yg_protocol_fragment_accept(&state, &first, 0U, 200U, &completed) ==
           YG_PROTOCOL_FRAGMENT_IN_PROGRESS);
    message = make(
        YG_PROTOCOL_VERSION, YG_PROTOCOL_FLAGS_FRAGMENT_LAST, 1U, 2U, 300U, 1U, 46U, 0U, pattern_b);
    assert(yg_protocol_fragment_accept(&state, &message, 10U, 200U, &completed) == YG_PROTOCOL_OK);
    assert(completed.payload_length == 92U && completed.sequence == 1U);
    assert(memcmp(completed.payload, pattern_a, 46U) == 0);
    assert(memcmp(completed.payload + 46U, pattern_b, 46U) == 0);
}

static void ack_only_on_request_tail(void)
{
    uint8_t storage[92];
    yg_protocol_fragment_state_t state;
    yg_protocol_message_t first = make(YG_PROTOCOL_VERSION,
                                       YG_PROTOCOL_FLAGS_FRAGMENT_FIRST,
                                       1U,
                                       2U,
                                       300U,
                                       0U,
                                       46U,
                                       0U,
                                       pattern_a);
    yg_protocol_message_t last =
        make(YG_PROTOCOL_VERSION,
             YG_PROTOCOL_FLAGS_FRAGMENT_LAST | YG_PROTOCOL_FLAGS_ACK_REQUEST,
             1U,
             2U,
             300U,
             1U,
             46U,
             0U,
             pattern_b);
    yg_protocol_message_t complete;
    assert(yg_protocol_fragment_init(&state, storage, sizeof(storage)) == YG_PROTOCOL_OK);
    assert(yg_protocol_fragment_accept(&state, &first, 0U, 200U, &complete) ==
           YG_PROTOCOL_FRAGMENT_IN_PROGRESS);
    assert(yg_protocol_fragment_accept(&state, &last, 1U, 200U, &complete) == YG_PROTOCOL_OK);
    assert(complete.flags == YG_PROTOCOL_FLAGS_ACK_REQUEST && complete.payload_length == 92U);
    first.flags |= YG_PROTOCOL_FLAGS_ACK_REQUEST;
    assert(yg_protocol_fragment_accept(&state, &first, 2U, 200U, &complete) ==
           YG_PROTOCOL_FRAGMENT_STATE_ERROR);
    first.flags = YG_PROTOCOL_FLAGS_FRAGMENT_FIRST | YG_PROTOCOL_FLAGS_RESPONSE;
    assert(yg_protocol_fragment_accept(&state, &first, 3U, 200U, &complete) ==
           YG_PROTOCOL_FRAGMENT_IN_PROGRESS);
    last.flags |= YG_PROTOCOL_FLAGS_RESPONSE;
    assert(yg_protocol_fragment_accept(&state, &last, 4U, 200U, &complete) ==
           YG_PROTOCOL_FRAGMENT_STATE_ERROR);
    assert(!state.active);
}

int main(void)
{
    fill(pattern_a, sizeof(pattern_a), 0x10U);
    fill(pattern_b, sizeof(pattern_b), 0x40U);
    fill(pattern_c, sizeof(pattern_c), 0x80U);

    argument_validation();
    fragment_lengths();
    first_sequence_must_be_zero();
    metadata_conflict_terminates();
    unrelated_fragment_does_not_mix();
    sequence_gap_terminates();
    duplicate_previous_ignored_no_refresh();
    first_duplicate_and_conflict();
    per_fragment_timeout_exact();
    absolute_timeout_from_first();
    timeout_wraparound();
    completed_unchanged_on_error();
    single_insertion();
    poll_semantics();
    capacity_bounded();
    golden_reassembly();
    ack_only_on_request_tail();
    puts("PASS yg_protocol fragment 12.1: lengths, seq0, metadata, duplicates, timeouts, poll");
    return 0;
}
