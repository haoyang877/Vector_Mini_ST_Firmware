#include "../../firmware/communication/yg_protocol.c"
#include "../../firmware/communication/yg_protocol_boot.c"
#include "protocol_test_bindings.h"
/* 验证协议核心文件之间的调用契约；测试只运行纯 C 逻辑，不访问硬件。 */
#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "yg_protocol_can_id.h"
#include "yg_protocol_frame_codec.h"
#include "yg_protocol_fragment.h"
#include "yg_protocol.h"
#include "yg_protocol.h"
#include "yg_protocol.h"
#include "vector_mini_st_g431.h"

static void frame_round_trip(void)
{
    static const uint8_t payload[] = {0xE8U, 0x03U};
    uint8_t buffer[YG_PROTOCOL_MAX_FRAME_SIZE];
    yg_protocol_frame_view_t input = {
        YG_PROTOCOL_VERSION,
        0U,
        0x02U,
        0x03U,
        105U,
        2U,
        sizeof(payload),
        0U,
        payload,
    };
    yg_protocol_frame_view_t output;
    size_t written = 0U;

    assert(yg_protocol_frame_encode(&input, buffer, sizeof(buffer), &written) == YG_PROTOCOL_OK);
    assert(written == 20U);
    assert(buffer[0] == 0x5AU && buffer[1] == 0xA5U);
    assert(yg_protocol_frame_decode(buffer, written, &output) == YG_PROTOCOL_OK);
    assert(output.message_type == 105U);
    assert(output.sequence == 2U);
    assert(output.payload_length == sizeof(payload));
    assert(memcmp(output.payload, payload, sizeof(payload)) == 0);

    buffer[15] ^= 0x01U;
    assert(yg_protocol_frame_decode(buffer, written, &output) == YG_PROTOCOL_INVALID_HEADER_CRC);
    assert(yg_protocol_frame_encode(&input, buffer, 19U, NULL) == YG_PROTOCOL_BUFFER_TOO_SMALL);
}

static void can_id_round_trip(void)
{
    yg_protocol_can_id_fields_t input = {3U, 0x02U, 0x03U};
    yg_protocol_can_id_fields_t output;
    uint32_t identifier = 0U;

    assert(yg_protocol_can_id_encode(&input, &identifier) == YG_PROTOCOL_OK);
    assert(identifier == 0x0CEF0302UL);
    assert(yg_protocol_can_id_decode(identifier, &output) == YG_PROTOCOL_OK);
    assert(memcmp(&input, &output, sizeof(input)) == 0);
    assert(yg_protocol_can_id_decode(identifier | (1UL << 25U), &output) ==
           YG_PROTOCOL_INVALID_NODE_ID);
}

static void readonly_request_to_response(void)
{
    DeviceMessages query = {.device_description_available = true};
    query.info.product_id = 0x1234U;
    uint8_t payload[12] = {0};
    ProtocolExchange exchange = {.request = {.version = YG_PROTOCOL_VERSION,
                                             .flags = YG_PROTOCOL_FLAGS_ACK_REQUEST,
                                             .source_node = 2U,
                                             .destination_node = 3U,
                                             .message_type = PROTOCOL_GET_INFO,
                                             .sequence = 9U,
                                             .payload_length = sizeof(payload),
                                             .payload = payload}};
    endpoint.device = query;
    assert(handle_info_query(&endpoint, &exchange) == YG_PROTOCOL_OK);
    assert(exchange.reply.length == 32U);
    assert(exchange.reply.data[12] == 0x34U && exchange.reply.data[13] == 0x12U);
    endpoint.device.device_description_available = false;
    assert(handle_info_query(&endpoint, &exchange) == YG_PROTOCOL_OK);
    assert(exchange.reply.length == 4U && exchange.reply.data[0] == 6U);
}

static void readonly_payload_codecs(void)
{
    static const uint8_t page_request_bytes[] = {
        0x01U,
        0x02U,
        0x03U,
        0x04U,
        0x05U,
        0x06U,
        0x07U,
        0x08U,
        0x09U,
        0x00U,
        0x00U,
        0x00U,
    };
    yg_protocol_message_t page_request_message = {YG_PROTOCOL_VERSION,
                                                  0U,
                                                  2U,
                                                  3U,
                                                  PROTOCOL_GET_INFO,
                                                  1U,
                                                  sizeof(page_request_bytes),
                                                  0U,
                                                  page_request_bytes};
    PageRequest page_request;
    ProtocolInfo info = {
        {0U}, 0x11223344U, 0x5566U, 0x7788U, 0x99AABBCCU, 0xDDEEFF00U, 0x12345678U};
    ProtocolCaps caps = {0x01020304U, 0x0506U, 0x0708U, 0x090AU, 0x0B0CU, 0x0D0EU, 0x0F10U};
    ProtocolResponse encoded_response = {0};
    uint8_t *payload = encoded_response.data;

    assert(decode_page_request(&page_request_message, &page_request) == YG_PROTOCOL_OK);
    assert(page_request.session_id == 0x04030201U && page_request.request_id == 0x08070605U);
    assert(page_request.page == 9U);
    assert(encode_device_info(&info, &encoded_response) == YG_PROTOCOL_OK);
    assert(encoded_response.length == PROTOCOL_INFO_PAGE0_SIZE);
    assert(payload[12] == 0x44U && payload[15] == 0x11U && payload[31] == 0x12U);
    assert(encode_device_caps(&caps, &encoded_response) == YG_PROTOCOL_OK);
    assert(encoded_response.length == PROTOCOL_CAPS_PAGE0_SIZE);
    assert(payload[0] == 0x04U && payload[15] == 0x0FU);

    /* APP 与升级包使用同一目标身份；UID 在黄金字节中置零，不冒充真实设备。 */
    info = (ProtocolInfo){.product_id = TARGET_PRODUCT, .hardware_revision = TARGET_HARDWARE};
    static const uint8_t target_info_golden[PROTOCOL_INFO_PAGE0_SIZE] = {
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0x36U, 0x32U, 0x47U, 0x59U,
        1U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,    0U,    0U,    0U};
    assert(encode_device_info(&info, &encoded_response) == YG_PROTOCOL_OK);
    assert(encoded_response.length == sizeof(target_info_golden) &&
           memcmp(payload, target_info_golden, sizeof(target_info_golden)) == 0);
}

static void motor_status_adapter(void)
{
    s_test_motor_status = (TestMotorStatus){0};
    ProtocolExchange exchange = {.request = {YG_PROTOCOL_VERSION,
                                             YG_PROTOCOL_FLAGS_ACK_REQUEST,
                                             2U,
                                             3U,
                                             PROTOCOL_GET_MOTOR_STATE,
                                             7U,
                                             0U,
                                             0U,
                                             NULL}};

    s_test_motor_status.state = 3U;
    s_test_motor_status.mode = 3U;
    s_test_motor_status.sample_available = true;
    assert(handle_motor_status_query(&endpoint, &exchange) == YG_PROTOCOL_OK);
    assert(exchange.reply.length == PROTOCOL_FEEDBACK_SIZE);
    assert(exchange.reply.data[0] == 0U && exchange.reply.data[2] == 0U &&
           exchange.reply.data[42] == 3U);
    assert(exchange.reply.data[15] == 0x80U && exchange.reply.data[24] == 0U &&
           exchange.reply.data[25] == 0U);
    exchange.request.payload_length = 1U;
    exchange.request.payload = exchange.reply.data;
    assert(handle_motor_status_query(&endpoint, &exchange) == YG_PROTOCOL_OK);
    assert(exchange.reply.data[0] == 3U && exchange.reply.length == PROTOCOL_FEEDBACK_SIZE);
    exchange.request.payload_length = 0U;
    exchange.request.payload = NULL;
    s_test_motor_status.sample_available = false;
    assert(handle_motor_status_query(&endpoint, &exchange) == YG_PROTOCOL_OK);
    assert(exchange.reply.length == PROTOCOL_FEEDBACK_SIZE && exchange.reply.data[0] == 6U &&
           exchange.reply.data[24] == 0U);
}

static void fragments_reassemble(void)
{
    uint8_t storage[128];
    uint8_t first_payload[YG_PROTOCOL_MAX_PAYLOAD_SIZE];
    uint8_t middle_payload[YG_PROTOCOL_MAX_PAYLOAD_SIZE];
    uint8_t last_payload[] = {5U, 6U};
    yg_protocol_fragment_state_t state;
    yg_protocol_message_t first = {YG_PROTOCOL_VERSION,
                                   YG_PROTOCOL_FLAGS_FRAGMENT_FIRST,
                                   1U,
                                   2U,
                                   300U,
                                   0U,
                                   YG_PROTOCOL_MAX_PAYLOAD_SIZE,
                                   0U,
                                   first_payload};
    yg_protocol_message_t middle = {YG_PROTOCOL_VERSION,
                                    YG_PROTOCOL_FLAGS_FRAGMENT_MIDDLE,
                                    1U,
                                    2U,
                                    300U,
                                    1U,
                                    YG_PROTOCOL_MAX_PAYLOAD_SIZE,
                                    0U,
                                    middle_payload};
    yg_protocol_message_t last = {YG_PROTOCOL_VERSION,
                                  YG_PROTOCOL_FLAGS_FRAGMENT_LAST,
                                  1U,
                                  2U,
                                  300U,
                                  2U,
                                  2U,
                                  0U,
                                  last_payload};
    yg_protocol_message_t complete;

    for (size_t index = 0U; index < YG_PROTOCOL_MAX_PAYLOAD_SIZE; ++index)
    {
        first_payload[index] = (uint8_t)(index + 1U);
        middle_payload[index] = (uint8_t)(index + 47U);
    }
    assert(yg_protocol_fragment_init(&state, storage, sizeof(storage)) == YG_PROTOCOL_OK);
    assert(yg_protocol_fragment_accept(&state, &first, 10U, 50U, &complete) ==
           YG_PROTOCOL_FRAGMENT_IN_PROGRESS);
    assert(yg_protocol_fragment_accept(&state, &middle, 20U, 50U, &complete) ==
           YG_PROTOCOL_FRAGMENT_IN_PROGRESS);
    assert(yg_protocol_fragment_accept(&state, &last, 30U, 50U, &complete) == YG_PROTOCOL_OK);
    assert(complete.payload_length == 94U);
    assert(memcmp(complete.payload, first_payload, YG_PROTOCOL_MAX_PAYLOAD_SIZE) == 0);
    assert(memcmp(complete.payload + YG_PROTOCOL_MAX_PAYLOAD_SIZE,
                  middle_payload,
                  YG_PROTOCOL_MAX_PAYLOAD_SIZE) == 0);
    assert(memcmp(complete.payload + 92U, last_payload, sizeof(last_payload)) == 0);

    assert(yg_protocol_fragment_accept(&state, &first, 100U, 50U, &complete) ==
           YG_PROTOCOL_FRAGMENT_IN_PROGRESS);
    assert(yg_protocol_fragment_accept(&state, &last, 160U, 50U, &complete) ==
           YG_PROTOCOL_FRAGMENT_TIMEOUT);
}

static void transfer_queue_is_bounded(void)
{
    BspCanFrame storage[2];
    BspCanFrame frame = {0x123U, 3U, true, false, true, true, {1U, 2U, 3U}};
    BspCanFrame output;
    CanQueue queue;

    queue = (CanQueue){.storage = storage, .capacity = 2U};
    assert(queue_push(&queue, &frame));
    frame.identifier = 0x456U;
    assert(queue_push(&queue, &frame));
    assert(!queue_push(&queue, &frame));
    assert(queue.dropped == 1U);
    assert(queue_pop(&queue, &output) && output.identifier == 0x123U);
    assert(queue_pop(&queue, &output) && output.identifier == 0x456U);
    assert(!queue_pop(&queue, &output));

    /* 任意容量重复绕圈：满与空不能靠 head == tail 区分。 */
    BspCanFrame odd_storage[3];
    queue = (CanQueue){.storage = odd_storage, .capacity = 3U};
    for (uint32_t cycle = 0U; cycle < 20U; ++cycle)
    {
        for (uint32_t index = 0U; index < 3U; ++index)
        {
            frame.identifier = cycle * 3U + index;
            assert(queue_push(&queue, &frame));
        }
        assert(queue_count(&queue) == 3U);
        for (uint32_t index = 0U; index < 3U; ++index)
        {
            assert(queue_peek(&queue, &output));
            assert(output.identifier == cycle * 3U + index);
            assert(queue_pop(&queue, &output));
            assert(output.identifier == cycle * 3U + index);
        }
        assert(queue_count(&queue) == 0U);
        assert(queue.head < queue.capacity && queue.tail < queue.capacity);
    }
}

/* 直接调用业务模块，验证拒绝目标和反馈无需依赖全局链路状态。 */
void yg_protocol_canfd_test(void);
void yg_protocol_contract_test(void);

int main(void)
{
    assert(test_reset_protocol(3U));
    frame_round_trip();
    can_id_round_trip();
    readonly_request_to_response();
    readonly_payload_codecs();
    motor_status_adapter();
    fragments_reassemble();
    transfer_queue_is_bounded();
    yg_protocol_canfd_test();
    yg_protocol_contract_test();
    puts("PASS yg_protocol core interfaces: codec, CAN ID, services, fragments and queue");
    return 0;
}

#include "yg_protocol_canfd_test.c"
#include "yg_protocol_contract_test.c"
