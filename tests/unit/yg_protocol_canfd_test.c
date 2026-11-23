/* 在离线平台替身上验证 CAN FD 承载与完整收发链；不会访问设备。 */
#include <assert.h>
#include <string.h>
#include "yg_protocol.h"

static BspCanFrame sent;
static bool send_ok;
static unsigned send_calls;

bool BspCan_TrySend(const BspCanFrame *frame)
{
    sent = *frame;
    ++send_calls;
    return send_ok;
}

static void lengths_and_errors(void)
{
    uint8_t payload[47];
    yg_protocol_message_t message = {1U, 0U, 2U, 3U, 105U, 2U, 0U, 0U, payload};
    yg_protocol_message_t decoded = {0};
    BspCanFrame frame;
    BspCanFrame changed;
    memset(payload, 0xA6, sizeof(payload));
    for (unsigned n = 0U; n <= 46U; ++n)
    {
        unsigned length = n + 18U;
        unsigned physical = length <= 20U   ? 20U
                            : length <= 24U ? 24U
                            : length <= 32U ? 32U
                            : length <= 48U ? 48U
                                            : 64U;
        message.payload_length = n;
        assert(encode_can_frame(&message, 2U, &frame) == YG_PROTOCOL_OK);
        assert(frame.identifier == 0x08EF0302U && frame.length == physical);
        assert(frame.extended && frame.fd && frame.bitrate_switch && !frame.remote);
        assert(decode_can_frame(&frame, 3U, &decoded) == YG_PROTOCOL_OK);
        assert(decoded.payload == &frame.data[16] && decoded.payload_length == n);
        assert(memcmp(decoded.payload, payload, n) == 0);
        for (unsigned i = length; i < physical; ++i)
        {
            assert(frame.data[i] == 0U);
        }
        changed = frame;
        changed.data[length - 1U] ^= 1U;
        decoded.sequence = 0xCAFEU;
        assert(decode_can_frame(&changed, 3U, &decoded) == YG_PROTOCOL_INVALID_FRAME_CRC);
        assert(decoded.sequence == 0xCAFEU);
        if (length < physical)
        {
            changed = frame;
            changed.data[length] = 1U;
            assert(decode_can_frame(&changed, 3U, &decoded) == YG_PROTOCOL_INVALID_PADDING);
        }
    }
    message.payload_length = 47U;
    changed = frame;
    assert(encode_can_frame(&message, 2U, &frame) == YG_PROTOCOL_INVALID_LENGTH);
    assert(memcmp(&changed, &frame, sizeof(frame)) == 0);
    message.payload_length = 8U;
    assert(encode_can_frame(&message, 8U, &frame) == YG_PROTOCOL_INVALID_CAN_FRAME);
    assert(encode_can_frame(&message, 2U, &frame) == YG_PROTOCOL_OK);
    assert(decode_can_frame(&frame, 4U, &decoded) == YG_PROTOCOL_INVALID_NODE_ID);
    for (unsigned n = 0U; n < 10U; ++n)
    {
        changed = frame;
        switch (n)
        {
        case 0:
            changed.extended = false;
            break;
        case 1:
            changed.fd = false;
            break;
        case 2:
            changed.remote = true;
            break;
        case 3:
            changed.bitrate_switch = false;
            break;
        case 4:
            changed.length = 65U;
            break;
        case 5:
            changed.length = 31U;
            break;
        case 6:
            changed.identifier |= 1UL << 29U;
            break;
        case 7:
            changed.identifier |= 1UL << 25U;
            break;
        case 8:
            changed.identifier |= 1UL << 24U;
            break;
        default:
            changed.identifier ^= 1UL << 16U;
            break;
        }
        assert(decode_can_frame(&changed, 3U, &decoded) == YG_PROTOCOL_INVALID_CAN_FRAME);
    }
    changed = frame;
    changed.identifier ^= 1U;
    assert(decode_can_frame(&changed, 3U, &decoded) == YG_PROTOCOL_INVALID_NODE_ID);
    changed = frame;
    changed.identifier ^= 1U << 8U;
    assert(decode_can_frame(&changed, 3U, &decoded) == YG_PROTOCOL_INVALID_NODE_ID);
    changed = frame;
    memset(&changed.data[10], 0xFF, 4U);
    assert(decode_can_frame(&changed, 3U, &decoded) == YG_PROTOCOL_INVALID_LENGTH);
    changed = frame;
    changed.data[15] ^= 1U;
    assert(decode_can_frame(&changed, 3U, &decoded) == YG_PROTOCOL_INVALID_HEADER_CRC);
    changed = frame;
    changed.length = 48U;
    assert(decode_can_frame(&changed, 3U, &decoded) == YG_PROTOCOL_INVALID_LENGTH);
    message.destination_node = 0xFFU;
    assert(encode_can_frame(&message, 2U, &frame) == YG_PROTOCOL_OK);
    assert(decode_can_frame(&frame, 3U, &decoded) == YG_PROTOCOL_OK);
    message.source_node = 0xFFU;
    assert(encode_can_frame(&message, 2U, &frame) == YG_PROTOCOL_INVALID_CAN_FRAME);
}

static void queued_round_trip(void)
{
    /* 冻结已有离线示例字节，避免 pack/unpack 的相同错误互相抵消。 */
    static const uint8_t golden[32] = {0x5A, 0xA5, 0x01, 0x00, 0x02, 0x03, 0x69, 0x00,
                                       0x02, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0xD1,
                                       0x69, 0x02, 0x34, 0x12, 0xE8, 0x03, 0x00, 0x00,
                                       0xD2, 0xE1, 0,    0,    0,    0,    0,    0};
    BspCanFrame tx_storage[2], rx_storage[2], frame, next, received;
    CanQueue tx, rx;
    yg_protocol_message_t message = {1U, 0U, 2U, 3U, 105U, 2U, 8U, 0U, &golden[16]};
    yg_protocol_message_t decoded;
    tx = (CanQueue){.storage = tx_storage, .capacity = 2U};
    rx = (CanQueue){.storage = rx_storage, .capacity = 2U};
    assert(encode_can_frame(&message, 2U, &frame) == YG_PROTOCOL_OK);
    assert(memcmp(frame.data, golden, sizeof(golden)) == 0);
    assert(ProtocolBoot_SendNext(&endpoint, &tx) == YG_PROTOCOL_QUEUE_EMPTY && send_calls == 0U);
    assert(queue_push(&tx, &frame));
    message.sequence = 3U;
    assert(encode_can_frame(&message, 2U, &next) == YG_PROTOCOL_OK);
    assert(queue_push(&tx, &next));
    assert(ProtocolBoot_SendNext(&endpoint, &tx) == YG_PROTOCOL_TRANSPORT_BUSY);
    assert(ProtocolBoot_SendNext(&endpoint, &tx) == YG_PROTOCOL_TRANSPORT_BUSY);
    assert(queue_count(&tx) == 2U && send_calls == 2U);
    assert(memcmp(sent.data, golden, 32U) == 0);
    send_ok = true;
    assert(ProtocolBoot_SendNext(&endpoint, &tx) == YG_PROTOCOL_OK);
    assert(queue_count(&tx) == 1U);
    assert(queue_push(&rx, &sent));
    assert(queue_push(&rx, &sent));
    assert(!queue_push(&rx, &sent));
    assert(rx.dropped == 1U);
    sent.data[0] = 0U;
    assert(queue_pop(&rx, &received));
    assert(decode_can_frame(&received, 3U, &decoded) == YG_PROTOCOL_OK);
    assert(decoded.sequence == 2U && decoded.payload_length == 8U);
    assert(ProtocolBoot_SendNext(&endpoint, &tx) == YG_PROTOCOL_OK);
    assert(sent.data[8] == 3U && queue_count(&tx) == 0U);
    assert(ProtocolBoot_SendNext(&endpoint, &tx) == YG_PROTOCOL_QUEUE_EMPTY && send_calls == 4U);
    sent.extended = false;
    /* 接收队列只复制数据，非法格式统一由前台解包拒绝。 */
    assert(queue_push(&rx, &sent));
    assert(queue_count(&rx) == 2U);
    assert(queue_pop(&rx, &received));
    assert(queue_pop(&rx, &received));
    assert(decode_can_frame(&received, 3U, &decoded) == YG_PROTOCOL_INVALID_CAN_FRAME);
}

static void reserved_front_slot(void)
{
    BspCanFrame storage[8], input = {0}, snapshot, copied;
    CanQueue queue;
    for (size_t capacity = 1U; capacity <= 8U; ++capacity)
    {
        queue = (CanQueue){.storage = storage, .capacity = capacity};
        for (uint32_t pass = 0U; pass < 1000U; ++pass)
        {
            input.identifier = pass;
            assert(queue_push(&queue, &input));
            assert(queue_peek(&queue, &snapshot));
            size_t reserved = queue.tail;
            /* 生产者可在复制任意两个字节间运行；未消费槽位不可被覆盖。 */
            for (size_t i = 1U; i < capacity; ++i)
            {
                input.identifier = pass + (uint32_t)i;
                assert(queue_push(&queue, &input));
                assert(memcmp(&storage[reserved], &snapshot, sizeof(snapshot)) == 0);
            }
            assert(!queue_push(&queue, &input));
            assert(memcmp(&storage[reserved], &snapshot, sizeof(snapshot)) == 0);
            assert(queue_drop(&queue));
            for (size_t i = 1U; i < capacity; ++i)
            {
                assert(queue_pop(&queue, &copied));
                assert(copied.identifier == pass + i);
            }
            assert(!queue_drop(&queue));
            assert(queue_count(&queue) == 0U);
        }
    }
}

static void queue_sequence_wrap(void)
{
    BspCanFrame storage[3], input = {0}, copied;
    CanQueue queue;
    queue = (CanQueue){.storage = storage, .capacity = 3U};
    queue.produced = queue.consumed = UINT32_MAX - 2U;
    for (unsigned pass = 0U; pass < 12U; ++pass)
    {
        for (unsigned frame = 0U; frame < 3U; ++frame)
        {
            input.identifier = pass * 3U + frame;
            assert(queue_push(&queue, &input));
        }
        assert(queue_count(&queue) == 3U);
        assert(!queue_push(&queue, &input));
        for (unsigned frame = 0U; frame < 3U; ++frame)
        {
            assert(queue_pop(&queue, &copied));
            assert(copied.identifier == pass * 3U + frame);
        }
        assert(queue_count(&queue) == 0U);
    }
}

void yg_protocol_canfd_test(void)
{
    lengths_and_errors();
    queued_round_trip();
    reserved_front_slot();
    queue_sequence_wrap();
}
