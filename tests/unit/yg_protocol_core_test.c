/* 验证协议核心文件之间的调用契约；测试只运行纯 C 逻辑，不访问硬件。 */
#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "yg_protocol_can_id.h"
#include "yg_protocol_frame_codec.h"
#include "yg_protocol_fragment.h"
#include "yg_protocol_message_registry.h"
#include "yg_protocol_motor_status.h"
#include "yg_protocol_readonly.h"
#include "yg_protocol_readonly_payload.h"
#include "yg_protocol_router.h"
#include "yg_protocol_transfer.h"

static yg_protocol_result_t handle_position(void *context,
                                            const yg_protocol_message_t *message,
                                            yg_protocol_service_result_t *result)
{
    (void)context;
    assert(message->payload_length == 2U);
    assert(message->payload[0] == 0xE8U);
    assert(message->payload[1] == 0x03U);
    result->applied_sequence = message->sequence;
    return YG_PROTOCOL_OK;
}

static yg_protocol_result_t handle_status(void *context,
                                          const yg_protocol_message_t *message,
                                          yg_protocol_service_result_t *result)
{
    (void)context;
    (void)message;
    result->applied_sequence = message->sequence;
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

static void router_routes_by_type(void)
{
    static const yg_protocol_message_descriptor_t descriptors[] = {
        {105U, false, true},
        {124U, false, true},
    };
    static const yg_protocol_route_t routes[] = {
        {105U, handle_position, NULL},
        {124U, handle_status, NULL},
    };
    yg_protocol_message_registry_t registry;
    yg_protocol_router_t router;
    yg_protocol_service_result_t result;
    yg_protocol_message_t message = {YG_PROTOCOL_VERSION, 0U, 1U, 2U, 105U, 7U, 2U, 0U, NULL};
    uint8_t payload[] = {0xE8U, 0x03U};

    message.payload = payload;
    assert(yg_protocol_message_registry_init(&registry, descriptors, 2U) == YG_PROTOCOL_OK);
    assert(yg_protocol_router_init(&router, &registry, routes, 2U) == YG_PROTOCOL_OK);
    assert(yg_protocol_router_handle(&router, &message, &result) == YG_PROTOCOL_OK);
    assert(result.applied_sequence == 7U);
    message.message_type = 999U;
    assert(yg_protocol_router_handle(&router, &message, &result) == YG_PROTOCOL_ROUTE_NOT_FOUND);
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
    static const yg_protocol_readonly_service_t configuration = {
        NULL,
        YG_PROTOCOL_READONLY_TYPE_GET_INFO,
        YG_PROTOCOL_READONLY_TYPE_DISABLED,
        YG_PROTOCOL_READONLY_TYPE_DISABLED,
        YG_PROTOCOL_READONLY_TYPE_DISABLED,
        provide_protocol_info,
        NULL,
        NULL,
        NULL,
    };
    yg_protocol_readonly_service_t service;
    yg_protocol_route_t route = {
        YG_PROTOCOL_READONLY_TYPE_GET_INFO, yg_protocol_readonly_handle, &service};
    yg_protocol_message_registry_t registry;
    yg_protocol_router_t router;
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

    assert(yg_protocol_readonly_init(&service, &configuration) == YG_PROTOCOL_OK);
    assert(yg_protocol_readonly_registry_init(&registry) == YG_PROTOCOL_OK);
    assert(yg_protocol_router_init(&router, &registry, &route, 1U) == YG_PROTOCOL_OK);
    assert(yg_protocol_router_handle(&router, &request, &result) == YG_PROTOCOL_OK);
    assert(result.response_payload_length == 4U);
    assert(yg_protocol_router_build_response(&request, &result, &response) == YG_PROTOCOL_OK);
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
    yg_protocol_readonly_motor_state_t state = {0x10203040U,
                                                0x50607080U,
                                                -1000,
                                                2000,
                                                -3000,
                                                0xAABBCCDDU,
                                                48000U,
                                                -1200,
                                                -2500,
                                                2U,
                                                3U,
                                                4U,
                                                0x0FU};
    uint8_t payload[YG_PROTOCOL_READONLY_MOTOR_STATE_SIZE];
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
    assert(yg_protocol_readonly_encode_motor_state(&state, payload, sizeof(payload), &written) ==
           YG_PROTOCOL_OK);
    assert(written == YG_PROTOCOL_READONLY_MOTOR_STATE_SIZE);
    assert(payload[8] == 0x18U && payload[9] == 0xFCU);
    assert(payload[26] == 0x50U && payload[27] == 0xFBU);
    assert(yg_protocol_readonly_encode_motor_state(&state, payload, 31U, &written) ==
           YG_PROTOCOL_BUFFER_TOO_SMALL);
}

static void motor_status_adapter(void)
{
    yg_protocol_motor_status_source_t source = {0};
    yg_protocol_readonly_motor_state_t state;
    uint8_t payload[YG_PROTOCOL_MOTOR_RESPONSE_SIZE];
    yg_protocol_message_t request = {YG_PROTOCOL_VERSION,
                                     YG_PROTOCOL_FLAGS_ACK_REQUEST,
                                     2U,
                                     3U,
                                     YG_PROTOCOL_READONLY_TYPE_GET_MOTOR_STATE,
                                     7U,
                                     8U,
                                     0U,
                                     payload};
    uint32_t payload_length = sizeof(payload);

    source.sample.position_feedback = 1.2345F;
    source.sample.speed_feedback = -40.0F;
    source.sample.current_feedback = 1.2F;
    source.sample.bus_voltage = 24.0F;
    source.sample.bus_current = 1.5F;
    source.sample.temperature = 25.25F;
    source.boot_id = 0x01020304U;
    source.sample_counter = 99U;
    source.state = 3U;
    source.mode = 3U;
    source.last_applied_sequence = 8U;
    source.measurement_valid_bits = YG_PROTOCOL_MOTOR_MEASUREMENT_MASK;
    source.target_applied = true;
    source.sample_available = true;
    assert(yg_protocol_motor_status_convert(&source, &state) == YG_PROTOCOL_OK);
    assert(state.position_mrad == 1235 && state.speed_mrad_s == -40000 && state.iq_mA == 1200);
    assert(state.bus_mV == 24000U && state.bus_mA == 1500 && state.temperature_centi_c == 2525);
    assert(state.valid_bits == (YG_PROTOCOL_MOTOR_MEASUREMENT_MASK | 0x08U));
    memset(payload, 0, sizeof(payload));
    for (size_t index = 0U; index < 8U; ++index)
    {
        payload[index] = (uint8_t)((index == 4U) ? 1U : 0U);
    }
    assert(yg_protocol_motor_status_provider(&source, &request, payload, &payload_length) ==
           YG_PROTOCOL_OK);
    assert(payload_length == YG_PROTOCOL_MOTOR_RESPONSE_SIZE);
    assert(payload[8] == 0U && payload[9] == 0U && payload[10] == 0U && payload[11] == 0U);
    source.sample_available = false;
    payload_length = sizeof(payload);
    assert(yg_protocol_motor_status_provider(&source, &request, payload, &payload_length) ==
           YG_PROTOCOL_OK);
    assert(payload_length == 12U && payload[8] == 6U);
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

void yg_protocol_canfd_test(void);
void yg_protocol_contract_test(void);

int main(void)
{
    frame_round_trip();
    can_id_round_trip();
    router_routes_by_type();
    readonly_request_to_response();
    readonly_payload_codecs();
    motor_status_adapter();
    fragments_reassemble();
    transfer_queue_is_bounded();
    yg_protocol_canfd_test();
    yg_protocol_contract_test();
    puts("PASS yg_protocol core interfaces: codec, CAN ID, registry, router, fragments and queue");
    return 0;
}
