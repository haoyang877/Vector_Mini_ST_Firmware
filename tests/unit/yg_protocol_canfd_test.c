/* 在离线平台替身上验证 CAN FD 承载与完整收发链；不会访问设备。 */
#include <assert.h>
#include <string.h>
#include "yg_protocol_canfd.h"
#include "yg_protocol_router.h"

static CommHwCanFrame sent;
static bool send_ok;
static unsigned send_calls;
static unsigned handler_calls;

bool comm_hw_can_try_send_frame(const CommHwCanFrame *frame)
{
    sent = *frame;
    ++send_calls;
    return send_ok;
}

static yg_protocol_result_t handle_test(void *context,
                                        const yg_protocol_message_t *message,
                                        yg_protocol_service_result_t *result)
{
    (void)context;
    ++handler_calls;
    assert(message->payload_length == 8U);
    result->applied_sequence = message->sequence;
    return YG_PROTOCOL_OK;
}

static void lengths_and_errors(void)
{
    uint8_t payload[47];
    yg_protocol_message_t message = {1U, 0U, 2U, 3U, 105U, 2U, 0U, 0U, payload};
    yg_protocol_message_t decoded = {0};
    yg_protocol_transfer_frame_t frame;
    yg_protocol_transfer_frame_t changed;
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
        assert(yg_protocol_canfd_pack(&message, 2U, &frame) == YG_PROTOCOL_OK);
        assert(frame.identifier == 0x08EF0302U && frame.length == physical);
        assert(frame.extended && frame.fd && frame.bitrate_switch && !frame.remote);
        assert(yg_protocol_canfd_unpack(&frame, 3U, &decoded) == YG_PROTOCOL_OK);
        assert(decoded.payload == &frame.data[16] && decoded.payload_length == n);
        assert(memcmp(decoded.payload, payload, n) == 0);
        for (unsigned i = length; i < physical; ++i)
        {
            assert(frame.data[i] == 0U);
        }
        changed = frame;
        changed.data[length - 1U] ^= 1U;
        decoded.sequence = 0xCAFEU;
        assert(yg_protocol_canfd_unpack(&changed, 3U, &decoded) == YG_PROTOCOL_INVALID_FRAME_CRC);
        assert(decoded.sequence == 0xCAFEU);
        if (length < physical)
        {
            changed = frame;
            changed.data[length] = 1U;
            assert(yg_protocol_canfd_unpack(&changed, 3U, &decoded) == YG_PROTOCOL_INVALID_PADDING);
        }
    }
    message.payload_length = 47U;
    changed = frame;
    assert(yg_protocol_canfd_pack(&message, 2U, &frame) == YG_PROTOCOL_INVALID_LENGTH);
    assert(memcmp(&changed, &frame, sizeof(frame)) == 0);
    message.payload_length = 8U;
    assert(yg_protocol_canfd_pack(&message, 8U, &frame) == YG_PROTOCOL_INVALID_CAN_FRAME);
    assert(yg_protocol_canfd_pack(&message, 2U, &frame) == YG_PROTOCOL_OK);
    assert(yg_protocol_canfd_unpack(&frame, 4U, &decoded) == YG_PROTOCOL_INVALID_NODE_ID);
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
        assert(yg_protocol_canfd_unpack(&changed, 3U, &decoded) == YG_PROTOCOL_INVALID_CAN_FRAME);
    }
    changed = frame;
    changed.identifier ^= 1U;
    assert(yg_protocol_canfd_unpack(&changed, 3U, &decoded) == YG_PROTOCOL_INVALID_NODE_ID);
    changed = frame;
    changed.identifier ^= 1U << 8U;
    assert(yg_protocol_canfd_unpack(&changed, 3U, &decoded) == YG_PROTOCOL_INVALID_NODE_ID);
    changed = frame;
    memset(&changed.data[10], 0xFF, 4U);
    assert(yg_protocol_canfd_unpack(&changed, 3U, &decoded) == YG_PROTOCOL_INVALID_LENGTH);
    changed = frame;
    changed.data[15] ^= 1U;
    assert(yg_protocol_canfd_unpack(&changed, 3U, &decoded) == YG_PROTOCOL_INVALID_HEADER_CRC);
    changed = frame;
    changed.length = 48U;
    assert(yg_protocol_canfd_unpack(&changed, 3U, &decoded) == YG_PROTOCOL_INVALID_LENGTH);
    message.destination_node = 0xFFU;
    assert(yg_protocol_canfd_pack(&message, 2U, &frame) == YG_PROTOCOL_OK);
    assert(yg_protocol_canfd_unpack(&frame, 3U, &decoded) == YG_PROTOCOL_OK);
    message.source_node = 0xFFU;
    assert(yg_protocol_canfd_pack(&message, 2U, &frame) == YG_PROTOCOL_INVALID_CAN_FRAME);
    assert(yg_protocol_canfd_pack(NULL, 2U, &frame) == YG_PROTOCOL_INVALID_ARGUMENT);
    assert(yg_protocol_canfd_unpack(NULL, 3U, &decoded) == YG_PROTOCOL_INVALID_ARGUMENT);
}

static void queued_round_trip(void)
{
    /* 冻结已有离线示例字节，避免 pack/unpack 的相同错误互相抵消。 */
    static const uint8_t golden[32] = {0x5A, 0xA5, 0x01, 0x00, 0x02, 0x03, 0x69, 0x00,
                                       0x02, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0xD1,
                                       0x69, 0x02, 0x34, 0x12, 0xE8, 0x03, 0x00, 0x00,
                                       0xD2, 0xE1, 0,    0,    0,    0,    0,    0};
    static const yg_protocol_message_descriptor_t descriptors[] = {{105U, false, true}};
    static const yg_protocol_route_t routes[] = {{105U, handle_test, NULL}};
    yg_protocol_message_registry_t registry;
    yg_protocol_router_t router;
    yg_protocol_service_result_t result;
    yg_protocol_transfer_frame_t tx_storage[2], rx_storage[2], frame, next, received;
    yg_protocol_transfer_queue_t tx, rx;
    yg_protocol_message_t message = {1U, 0U, 2U, 3U, 105U, 2U, 8U, 0U, &golden[16]};
    yg_protocol_message_t decoded;
    assert(yg_protocol_message_registry_init(&registry, descriptors, 1U) == YG_PROTOCOL_OK);
    assert(yg_protocol_router_init(&router, &registry, routes, 1U) == YG_PROTOCOL_OK);
    assert(yg_protocol_transfer_queue_init(&tx, tx_storage, 2U));
    assert(yg_protocol_transfer_queue_init(&rx, rx_storage, 2U));
    assert(yg_protocol_canfd_pack(&message, 2U, &frame) == YG_PROTOCOL_OK);
    assert(memcmp(frame.data, golden, sizeof(golden)) == 0);
    assert(yg_protocol_canfd_send_one(&tx) == YG_PROTOCOL_QUEUE_EMPTY && send_calls == 0U);
    assert(yg_protocol_transfer_queue_push(&tx, &frame));
    message.sequence = 3U;
    assert(yg_protocol_canfd_pack(&message, 2U, &next) == YG_PROTOCOL_OK);
    assert(yg_protocol_transfer_queue_push(&tx, &next));
    assert(yg_protocol_canfd_send_one(&tx) == YG_PROTOCOL_TRANSPORT_BUSY);
    assert(yg_protocol_canfd_send_one(&tx) == YG_PROTOCOL_TRANSPORT_BUSY);
    assert(yg_protocol_transfer_queue_count(&tx) == 2U && send_calls == 2U);
    assert(memcmp(sent.data, golden, 32U) == 0);
    send_ok = true;
    assert(yg_protocol_canfd_send_one(&tx) == YG_PROTOCOL_OK);
    assert(yg_protocol_transfer_queue_count(&tx) == 1U);
    assert(yg_protocol_canfd_receive(&rx, &sent) == YG_PROTOCOL_OK);
    assert(yg_protocol_canfd_receive(&rx, &sent) == YG_PROTOCOL_OK);
    assert(yg_protocol_canfd_receive(&rx, &sent) == YG_PROTOCOL_QUEUE_FULL);
    assert(yg_protocol_transfer_queue_dropped(&rx) == 1U);
    sent.data[0] = 0U;
    assert(yg_protocol_transfer_queue_pop(&rx, &received));
    assert(yg_protocol_canfd_unpack(&received, 3U, &decoded) == YG_PROTOCOL_OK);
    assert(yg_protocol_router_handle(&router, &decoded, &result) == YG_PROTOCOL_OK);
    assert(handler_calls == 1U && result.applied_sequence == 2U);
    assert(yg_protocol_canfd_send_one(&tx) == YG_PROTOCOL_OK);
    assert(sent.data[8] == 3U && yg_protocol_transfer_queue_count(&tx) == 0U);
    assert(yg_protocol_canfd_send_one(&tx) == YG_PROTOCOL_QUEUE_EMPTY && send_calls == 4U);
    sent.extended = false;
    assert(yg_protocol_canfd_receive(&rx, &sent) == YG_PROTOCOL_INVALID_CAN_FRAME);
    assert(yg_protocol_transfer_queue_count(&rx) == 1U);
    frame.length = 1U;
    assert(yg_protocol_transfer_queue_push(&tx, &frame));
    assert(yg_protocol_canfd_send_one(&tx) == YG_PROTOCOL_INVALID_CAN_FRAME);
    assert(send_calls == 4U && yg_protocol_transfer_queue_count(&tx) == 1U);
    assert(yg_protocol_canfd_receive(NULL, &sent) == YG_PROTOCOL_INVALID_ARGUMENT);
    assert(yg_protocol_canfd_send_one(NULL) == YG_PROTOCOL_INVALID_ARGUMENT);
}

void yg_protocol_canfd_test(void)
{
    lengths_and_errors();
    queued_round_trip();
}
