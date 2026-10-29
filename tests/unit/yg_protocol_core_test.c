/* 验证协议核心文件之间的调用契约；测试只运行纯 C 逻辑，不访问硬件。 */
#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "yg_protocol_can_id.h"
#include "yg_protocol_frame_codec.h"
#include "yg_protocol_fragment.h"
#include "yg_protocol_motor.h"
#include "yg_protocol_readonly.h"
#include "yg_protocol_endpoint.h"
#include "yg_protocol_canfd.h"
#include "vector_mini_st_g431.h"

static yg_protocol_result_t handle_position(void *context,
                                            const yg_protocol_message_t *message,
                                            yg_protocol_service_result_t *result)
{
    (void)context;
    assert(message->payload_length == 2U);
    assert(message->payload[0] == 0xE8U);
    assert(message->payload[1] == 0x03U);
    result->response_sequence = message->sequence;
    return YG_PROTOCOL_OK;
}

static yg_protocol_result_t handle_status(void *context,
                                          const yg_protocol_message_t *message,
                                          yg_protocol_service_result_t *result)
{
    (void)context;
    (void)message;
    result->response_sequence = message->sequence;
    return YG_PROTOCOL_OK;
}

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

static void commands_execute_by_type(void)
{
    static const yg_protocol_command_t entries[] = {
        {105U, false, handle_position, NULL},
        {124U, false, handle_status, NULL},
    };
    yg_protocol_commands_t commands;
    yg_protocol_service_result_t result;
    yg_protocol_message_t message = {YG_PROTOCOL_VERSION, 0U, 1U, 2U, 105U, 7U, 2U, 0U, NULL};
    uint8_t payload[] = {0xE8U, 0x03U};

    message.payload = payload;
    assert(yg_protocol_commands_register(&commands, entries, 2U) == YG_PROTOCOL_OK);
    assert(yg_protocol_commands_execute(yg_protocol_commands_find(&commands, message.message_type),
                                        &message,
                                        &result) == YG_PROTOCOL_OK);
    assert(result.response_sequence == 7U);
    assert(result.reply_ready);
    /* 重复 ID 或空表参数，不能破坏已注册的命令表。 */
    yg_protocol_command_t invalid[] = {entries[0], entries[0]};
    assert(yg_protocol_commands_register(&commands, invalid, 2U) ==
           YG_PROTOCOL_INVALID_MESSAGE_TYPE);
    assert(commands.entries == entries && commands.count == 2U);
    assert(yg_protocol_commands_register(&commands, NULL, 1U) == YG_PROTOCOL_INVALID_ARGUMENT);
    assert(yg_protocol_commands_execute(&entries[1], &message, &result) ==
           YG_PROTOCOL_ROUTE_NOT_FOUND);
    message.message_type = 999U;
    assert(yg_protocol_commands_execute(yg_protocol_commands_find(&commands, message.message_type),
                                        &message,
                                        &result) == YG_PROTOCOL_ROUTE_NOT_FOUND);
}

static yg_protocol_result_t provide_protocol_info(void *context,
                                                  const yg_protocol_message_t *request,
                                                  uint8_t *payload,
                                                  uint32_t *payload_length)
{
    (void)context;
    (void)request;
    assert(*payload_length >= 4U);
    payload[0] = YG_PROTOCOL_VERSION;
    payload[1] = YG_PROTOCOL_MAX_FRAME_SIZE;
    payload[2] = YG_PROTOCOL_MAX_PAYLOAD_SIZE;
    payload[3] = 0U;
    *payload_length = 4U;
    return YG_PROTOCOL_OK;
}

static void readonly_request_to_response(void)
{
    yg_protocol_readonly_service_t service = {NULL, provide_protocol_info};
    const yg_protocol_command_t entry = {
        YG_PROTOCOL_READONLY_TYPE_GET_INFO, true, yg_protocol_readonly_handle, &service};
    yg_protocol_commands_t commands;
    yg_protocol_service_result_t result;
    yg_protocol_frame_view_t decoded;
    uint8_t encoded[YG_PROTOCOL_MAX_FRAME_SIZE];
    size_t encoded_length = 0U;
    yg_protocol_message_t request = {YG_PROTOCOL_VERSION,
                                     YG_PROTOCOL_FLAGS_ACK_REQUEST,
                                     2U,
                                     3U,
                                     YG_PROTOCOL_READONLY_TYPE_GET_INFO,
                                     9U,
                                     0U,
                                     0U,
                                     NULL};
    yg_protocol_message_t response;

    assert(yg_protocol_commands_register(&commands, &entry, 1U) == YG_PROTOCOL_OK);
    assert(yg_protocol_commands_execute(yg_protocol_commands_find(&commands, request.message_type),
                                        &request,
                                        &result) == YG_PROTOCOL_OK);
    assert(result.response_payload_length == 4U);
    assert(yg_protocol_commands_build_response(&request, &result, &response) == YG_PROTOCOL_OK);
    assert(response.source_node == 3U && response.destination_node == 2U);
    assert(response.sequence == 9U && response.payload_length == 4U);
    assert(response.payload[0] == YG_PROTOCOL_VERSION);
    assert(yg_protocol_frame_encode(&response, encoded, sizeof(encoded), &encoded_length) ==
           YG_PROTOCOL_OK);
    assert(yg_protocol_frame_decode(encoded, encoded_length, &decoded) == YG_PROTOCOL_OK);
    assert(decoded.source_node == 3U && decoded.destination_node == 2U);
    assert(decoded.message_type == YG_PROTOCOL_READONLY_TYPE_GET_INFO);
    assert(decoded.payload_length == 4U && decoded.payload[2] == YG_PROTOCOL_MAX_PAYLOAD_SIZE);
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
                                                  YG_PROTOCOL_READONLY_TYPE_GET_INFO,
                                                  1U,
                                                  sizeof(page_request_bytes),
                                                  0U,
                                                  page_request_bytes};
    yg_protocol_readonly_page_request_t page_request;
    yg_protocol_readonly_info_page0_t info = {
        {0U}, 0x11223344U, 0x5566U, 0x7788U, 0x99AABBCCU, 0xDDEEFF00U, 0x12345678U};
    yg_protocol_readonly_caps_page0_t caps = {
        0x01020304U, 0x0506U, 0x0708U, 0x090AU, 0x0B0CU, 0x0D0EU, 0x0F10U};
    uint8_t payload[YG_PROTOCOL_READONLY_INFO_PAGE0_SIZE];
    size_t written = 0U;

    assert(yg_protocol_readonly_decode_page_request(&page_request_message, &page_request) ==
           YG_PROTOCOL_OK);
    assert(page_request.session_id == 0x04030201U && page_request.request_id == 0x08070605U);
    assert(page_request.page == 9U);
    assert(yg_protocol_readonly_encode_info_page0(&info, payload, sizeof(payload), &written) ==
           YG_PROTOCOL_OK);
    assert(written == YG_PROTOCOL_READONLY_INFO_PAGE0_SIZE);
    assert(payload[12] == 0x44U && payload[15] == 0x11U && payload[31] == 0x12U);
    assert(yg_protocol_readonly_encode_caps_page0(&caps, payload, sizeof(payload), &written) ==
           YG_PROTOCOL_OK);
    assert(written == YG_PROTOCOL_READONLY_CAPS_PAGE0_SIZE);
    assert(payload[0] == 0x04U && payload[15] == 0x0FU);

    /* APP 与升级包使用同一目标身份；UID 在黄金字节中置零，不冒充真实设备。 */
    info = (yg_protocol_readonly_info_page0_t){.product_id = TARGET_PRODUCT,
                                               .hardware_revision = TARGET_HARDWARE};
    static const uint8_t target_info_golden[YG_PROTOCOL_READONLY_INFO_PAGE0_SIZE] = {
        0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0x36U, 0x32U, 0x47U, 0x59U,
        1U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,    0U,    0U,    0U};
    assert(yg_protocol_readonly_encode_info_page0(&info, payload, sizeof(payload), &written) ==
           YG_PROTOCOL_OK);
    assert(written == sizeof(target_info_golden) &&
           memcmp(payload, target_info_golden, sizeof(target_info_golden)) == 0);
}

static void motor_status_adapter(void)
{
    yg_protocol_motor_status_source_t source = {0};
    uint8_t payload[YG_PROTOCOL_MOTOR_FEEDBACK_PAYLOAD_SIZE];
    yg_protocol_message_t request = {YG_PROTOCOL_VERSION,
                                     YG_PROTOCOL_FLAGS_ACK_REQUEST,
                                     2U,
                                     3U,
                                     YG_PROTOCOL_READONLY_TYPE_GET_MOTOR_STATE,
                                     7U,
                                     0U,
                                     0U,
                                     NULL};
    uint32_t payload_length = sizeof(payload);

    source.sample.position_feedback = 1.2345F;
    source.sample.speed_feedback_rad_s = -40.0F;
    source.sample.current_feedback = 1.2F;
    source.sample.bus_voltage = 24.0F;
    source.sample.bus_current = 1.5F;
    source.sample.temperature = 25.25F;
    source.state = 3U;
    source.mode = 3U;
    source.measurement_valid_bits = YG_PROTOCOL_MOTOR_MEASUREMENT_MASK;
    source.sample_available = true;
    assert(yg_protocol_motor_status_provider(&source, &request, payload, &payload_length) ==
           YG_PROTOCOL_OK);
    assert(payload_length == YG_PROTOCOL_MOTOR_FEEDBACK_PAYLOAD_SIZE);
    assert(payload[0] == 0U && payload[2] == 0U && payload[42] == 3U);
    assert(payload[12] == 0xD3U && payload[13] == 0x04U && payload[24] == 0xFFU);
    request.payload_length = 1U;
    request.payload = payload;
    payload_length = sizeof(payload);
    assert(yg_protocol_motor_status_provider(&source, &request, payload, &payload_length) ==
           YG_PROTOCOL_OK);
    assert(payload[0] == 3U && payload_length == YG_PROTOCOL_MOTOR_FEEDBACK_PAYLOAD_SIZE);
    request.payload_length = 0U;
    request.payload = NULL;
    source.sample_available = false;
    payload_length = sizeof(payload);
    assert(yg_protocol_motor_status_provider(&source, &request, payload, &payload_length) ==
           YG_PROTOCOL_OK);
    assert(payload_length == YG_PROTOCOL_MOTOR_FEEDBACK_PAYLOAD_SIZE && payload[0] == 6U &&
           payload[24] == 0U);
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
    yg_protocol_transfer_frame_t storage[2];
    yg_protocol_transfer_frame_t frame = {0x123U, 3U, true, false, true, true, {1U, 2U, 3U}};
    yg_protocol_transfer_frame_t output;
    yg_protocol_transfer_queue_t queue;

    assert(yg_protocol_transfer_queue_init(&queue, storage, 2U));
    assert(yg_protocol_transfer_queue_push(&queue, &frame));
    frame.identifier = 0x456U;
    assert(yg_protocol_transfer_queue_push(&queue, &frame));
    assert(!yg_protocol_transfer_queue_push(&queue, &frame));
    assert(yg_protocol_transfer_queue_dropped(&queue) == 1U);
    assert(yg_protocol_transfer_queue_pop(&queue, &output) && output.identifier == 0x123U);
    assert(yg_protocol_transfer_queue_pop(&queue, &output) && output.identifier == 0x456U);
    assert(!yg_protocol_transfer_queue_pop(&queue, &output));

    /* 任意容量重复绕圈：满与空不能靠 head == tail 区分。 */
    yg_protocol_transfer_frame_t odd_storage[3];
    assert(yg_protocol_transfer_queue_init(&queue, odd_storage, 3U));
    for (uint32_t cycle = 0U; cycle < 20U; ++cycle)
    {
        for (uint32_t index = 0U; index < 3U; ++index)
        {
            frame.identifier = cycle * 3U + index;
            assert(yg_protocol_transfer_queue_push(&queue, &frame));
        }
        assert(yg_protocol_transfer_queue_count(&queue) == 3U);
        for (uint32_t index = 0U; index < 3U; ++index)
        {
            assert(yg_protocol_transfer_queue_peek(&queue, &output));
            assert(output.identifier == cycle * 3U + index);
            assert(yg_protocol_transfer_queue_pop(&queue, &output));
            assert(output.identifier == cycle * 3U + index);
        }
        assert(yg_protocol_transfer_queue_count(&queue) == 0U);
        assert(queue.head < queue.capacity && queue.tail < queue.capacity);
    }
}

/* 直接调用业务模块，验证历史和反馈无需依赖全局链路状态。 */
static void isolated_control(void *context,
                             const yg_protocol_motor_request_t *request,
                             yg_protocol_service_reply_t *reply)
{
    unsigned *calls = context;
    assert(request->operation == YG_PROTOCOL_MOTOR_SET_CONTROL);
    ++*calls;
    reply->status = YG_PROTOCOL_SERVICE_OK;
}

typedef struct
{
    bool busy;
    unsigned calls;
    uint8_t node;
    uint16_t sequence;
    uint8_t payload[YG_PROTOCOL_MOTOR_FEEDBACK_PAYLOAD_SIZE];
} feedback_sink_t;

static yg_protocol_result_t
isolated_send(void *context, const yg_protocol_message_t *message, uint8_t priority)
{
    feedback_sink_t *sink = context;
    assert(priority == 3U && message->message_type == 0x007CU);
    assert(message->payload_length == sizeof(sink->payload));
    if (sink->busy)
    {
        return YG_PROTOCOL_QUEUE_FULL;
    }
    ++sink->calls;
    sink->node = message->source_node;
    sink->sequence = message->sequence;
    memcpy(sink->payload, message->payload, sizeof(sink->payload));
    return YG_PROTOCOL_OK;
}

static void business_contexts_are_independent(void)
{
    unsigned calls_a = 0U, calls_b = 0U;
    yg_protocol_motor_context_t a = {.node_id = 3U};
    yg_protocol_motor_context_t b = {.node_id = 4U};
    a.motor_status_source = &a.empty_motor_status;
    b.motor_status_source = &b.empty_motor_status;
    a.motor_service = (yg_protocol_motor_service_t){&calls_a, isolated_control};
    b.motor_service = (yg_protocol_motor_service_t){&calls_b, isolated_control};
    uint8_t payload[YG_PROTOCOL_MOTOR_CONTROL_REQUEST_SIZE] = {3U};
    yg_protocol_message_t request = {.version = YG_PROTOCOL_VERSION,
                                     .flags = YG_PROTOCOL_FLAGS_ACK_REQUEST,
                                     .source_node = 2U,
                                     .destination_node = 3U,
                                     .message_type = 0x0074U,
                                     .sequence = 41U,
                                     .payload_length = sizeof(payload),
                                     .payload = payload};
    yg_protocol_service_result_t response = {0};
    feedback_sink_t sink = {.busy = true};
    assert(yg_protocol_motor_handle_control(&a, &request, &response) == YG_PROTOCOL_OK);
    assert(calls_a == 1U && calls_b == 0U && response.response_payload[0] == 0U);
    uint8_t pending[sizeof(sink.payload)];
    memcpy(pending, a.feedback_payload, sizeof(pending));
    assert(yg_protocol_motor_flush_feedback(&a, isolated_send, &sink) == YG_PROTOCOL_QUEUE_FULL);
    assert(a.feedback_pending && a.feedback_sequence == 0U && sink.calls == 0U);

    request.destination_node = 4U;
    assert(yg_protocol_motor_handle_control(&b, &request, &response) == YG_PROTOCOL_OK);
    assert(calls_a == 1U && calls_b == 1U);
    sink.busy = false;
    assert(yg_protocol_motor_flush_feedback(&b, isolated_send, &sink) == YG_PROTOCOL_OK);
    assert(sink.node == 4U && sink.sequence == 0U && !b.feedback_pending);
    assert(a.feedback_pending && a.feedback_sequence == 0U);
    assert(yg_protocol_motor_flush_feedback(&a, isolated_send, &sink) == YG_PROTOCOL_OK);
    assert(sink.node == 3U && sink.sequence == 0U && !a.feedback_pending);
    assert(memcmp(sink.payload, pending, sizeof(pending)) == 0);
    request.destination_node = 3U;
    request.flags |= YG_PROTOCOL_FLAGS_RETRY;
    assert(yg_protocol_motor_handle_control(&a, &request, &response) == YG_PROTOCOL_OK);
    assert(calls_a == 1U && calls_b == 1U);
    assert(yg_protocol_motor_flush_feedback(&a, isolated_send, &sink) == YG_PROTOCOL_OK);
    assert(sink.sequence == 1U && sink.calls == 3U);

    yg_protocol_readonly_context_t unavailable = {0};
    yg_protocol_readonly_context_t available = {.device_description_available = true};
    available.maintenance_pages[0][0] = 0xA7U;
    uint8_t page[YG_PROTOCOL_READONLY_PAGE_REQUEST_SIZE] = {0};
    page[8] = 1U;
    request.message_type = 0x0001U;
    request.payload = page;
    request.payload_length = sizeof(page);
    assert(yg_protocol_readonly_handle_info(&available, &request, &response) == YG_PROTOCOL_OK);
    assert(response.response_payload_length == 32U && response.response_payload[0] == 0xA7U);
    assert(yg_protocol_readonly_handle_info(&unavailable, &request, &response) == YG_PROTOCOL_OK);
    assert(response.response_payload_length == 4U && response.response_payload[0] == 6U);
    assert(yg_protocol_readonly_handle_info(&available, &request, &response) == YG_PROTOCOL_OK);
    assert(response.response_payload_length == 32U && response.response_payload[0] == 0xA7U);
}

void yg_protocol_canfd_test(void);
void yg_protocol_contract_test(void);

int main(void)
{
    frame_round_trip();
    can_id_round_trip();
    commands_execute_by_type();
    readonly_request_to_response();
    readonly_payload_codecs();
    motor_status_adapter();
    fragments_reassemble();
    transfer_queue_is_bounded();
    business_contexts_are_independent();
    yg_protocol_canfd_test();
    yg_protocol_contract_test();
    puts("PASS yg_protocol core interfaces: codec, CAN ID, commands, fragments and queue");
    return 0;
}
