#include "../../firmware/communication/yg_protocol.c"
#include "../../firmware/communication/yg_protocol_boot.c"
#include "protocol_test_bindings.h"
/* 电机线路编码和设备能力黄金向量；不访问硬件。 */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "yg_protocol.h"
#define CHECK(condition) assert(condition)
bool test_application_capabilities(void);

static unsigned open_loop_calls[2];
static void open_loop_start(void *context, const ProtocolCommand *request, ProtocolReply *reply)
{
    (void)context;
    CHECK(request->mode == 9U || request->mode == 10U);
    ++open_loop_calls[0];
    reply->status = PROTOCOL_OK;
}
static void open_loop_target(void *context, const ProtocolCommand *request, ProtocolReply *reply)
{
    (void)context;
    CHECK(request->mode == 9U || request->mode == 10U);
    ++open_loop_calls[1];
    reply->status = PROTOCOL_OK;
}

static void test_open_loop_vectors(void)
{
    /* v0.3黄金payload：+20Hz、Vd=1V/Vq=2V；-20Hz、Id=1A/Iq=2A。 */
    static const uint8_t voltage[32] = {9, 0, 0, 0, 0, 0,    0,    0,    0, 0,    0,
                                        0, 0, 0, 0, 0, 0xd0, 0x07, 0,    0, 0x20, 0x4e,
                                        0, 0, 0, 0, 0, 0,    0xe8, 0x03, 0, 0};
    static const uint8_t current[32] = {10,   0,    0,    0,    0, 0, 0, 0, 0, 0,    0,
                                        0,    0xd0, 0x07, 0,    0, 0, 0, 0, 0, 0xe0, 0xb1,
                                        0xff, 0xff, 0xe8, 0x03, 0, 0, 0, 0, 0, 0};
    yg_protocol_message_t message = {
        .message_type = PROTOCOL_CONTROL, .payload_length = 32U, .payload = voltage};
    ProtocolCommand decoded;
    CHECK(decode_target_command(&message, &decoded) == YG_PROTOCOL_OK);
    CHECK(decoded.mode == 9 && decoded.electrical_frequency_mHz == 20000 && decoded.vd_mV == 1000 &&
          decoded.v_q_mV == 2000);
    message.payload = current;
    CHECK(decode_target_command(&message, &decoded) == YG_PROTOCOL_OK);
    CHECK(decoded.mode == 10U && decoded.electrical_frequency_mHz == -20000 &&
          decoded.id_mA == 1000 && decoded.iq_mA == 2000);
    ProtocolCommand before = decoded;
    message.payload_length = 20U;
    CHECK(decode_target_command(&message, &decoded) == YG_PROTOCOL_INVALID_LENGTH);
    CHECK(memcmp(&before, &decoded, sizeof(before)) == 0);
    uint8_t invalid[32];
    memcpy(invalid, voltage, sizeof(invalid));
    invalid[24] = 1;
    message.payload = invalid;
    message.payload_length = 32;
    CHECK(decode_target_command(&message, &decoded) == YG_PROTOCOL_INVALID_FIELD);
    invalid[0] = 1;
    invalid[24] = 0;
    CHECK(decode_target_command(&message, &decoded) == YG_PROTOCOL_INVALID_LENGTH);
    CHECK(test_reset_protocol(3U));
    memset(open_loop_calls, 0, sizeof(open_loop_calls));
    endpoint.motor = (ProtocolMotor){.start = open_loop_start, .target = open_loop_target};
    uint8_t start_payload[4] = {9, 0, 0, 0};
    ProtocolExchange exchange = {.request = {.version = YG_PROTOCOL_VERSION,
                                             .flags = YG_PROTOCOL_FLAGS_ACK_REQUEST,
                                             .source_node = 2U,
                                             .destination_node = 3U,
                                             .sequence = 71U,
                                             .message_type = PROTOCOL_START,
                                             .payload_length = 4U,
                                             .payload = start_payload}};
    CHECK(handle_command(&endpoint, &exchange) == YG_PROTOCOL_OK);
    CHECK(open_loop_calls[0] == 1U && exchange.reply.data[0] == 0U);
    start_payload[0] = 10U;
    CHECK(handle_command(&endpoint, &exchange) == YG_PROTOCOL_OK && open_loop_calls[0] == 2U);
    start_payload[0] = 7U;
    CHECK(handle_command(&endpoint, &exchange) == YG_PROTOCOL_OK && open_loop_calls[0] == 2U);
    CHECK(exchange.reply.data[0] == 2U);
    for (size_t i = 1U; i < sizeof(start_payload); ++i)
    {
        start_payload[i] = 1U;
        CHECK(handle_command(&endpoint, &exchange) == YG_PROTOCOL_OK);
        CHECK(exchange.reply.data[0] == PROTOCOL_RESULT_BAD_FIELD && open_loop_calls[0] == 2U);
        start_payload[i] = 0U;
    }
    exchange.request.payload_length = 1U;
    CHECK(handle_command(&endpoint, &exchange) == YG_PROTOCOL_OK);
    CHECK(exchange.reply.data[0] == PROTOCOL_RESULT_BAD_LENGTH && open_loop_calls[0] == 2U);
    exchange.request.message_type = PROTOCOL_CONTROL;
    exchange.request.payload_length = 32U;
    exchange.request.payload = voltage;
    CHECK(handle_command(&endpoint, &exchange) == YG_PROTOCOL_OK);
    CHECK(open_loop_calls[1] == 1U && open_loop_calls[0] == 2U && exchange.reply.data[0] == 0U);
    CHECK(handle_command(&endpoint, &exchange) == YG_PROTOCOL_OK && open_loop_calls[1] == 2U);
    memcpy(invalid, voltage, sizeof(invalid));
    invalid[20] ^= 1U;
    exchange.request.payload = invalid;
    CHECK(handle_command(&endpoint, &exchange) == YG_PROTOCOL_OK && open_loop_calls[1] == 3U);
    CHECK(exchange.reply.data[0] == 0U);
    printf("PASS v0.3 open-loop golden payloads, SI mapping, lengths, start whitelist and "
           "extended-payload replay\n");
}

static void test_minimal_state(void)
{
    ProtocolMotorStatus source = {.state = 5U, .sample_available = true};
    ProtocolFeedback feedback;
    ProtocolResponse encoded_response = {0};
    uint8_t *payload = encoded_response.data;
    CHECK(Protocol_BuildFeedback(&source, 3U, 0U, 0U, &feedback) == YG_PROTOCOL_OK);
    CHECK(feedback.valid_bits == 0U);
    CHECK(encode_motor_feedback(&feedback, &encoded_response) == YG_PROTOCOL_OK);
    printf("VECTOR minimal_state ");
    for (unsigned i = 0; i < PROTOCOL_FEEDBACK_SIZE; ++i)
    {
        printf("%02x", payload[i]);
    }
    printf("\n");
}

int main(void)
{
    test_minimal_state();
    test_open_loop_vectors();
    CHECK(test_application_capabilities());
    puts("PASS motor wire vectors");
    return 0;
}
