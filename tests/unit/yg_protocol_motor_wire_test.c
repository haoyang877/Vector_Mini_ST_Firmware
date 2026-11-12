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
    CHECK(test_reset_protocol(3U));
    s_test_motor_status = (TestMotorStatus){.state = 5U, .sample_available = true};
    ProtocolExchange query = {
        .request = {.message_type = PROTOCOL_GET_MOTOR_STATE, .destination_node = 3U}};
    CHECK(handle_motor_status_query(&endpoint, &query) == YG_PROTOCOL_OK);
    uint8_t *payload = query.reply.data;
    CHECK(YgProtocol_ReadU16Le(payload + 24U) == 0U);
    printf("VECTOR minimal_state ");
    for (unsigned i = 0; i < PROTOCOL_FEEDBACK_SIZE; ++i)
    {
        printf("%02x", payload[i]);
    }
    printf("\n");
}

static unsigned host_start_calls, keepalive_calls;
static void capture_host_start(void *context, const ProtocolCommand *request, ProtocolReply *reply)
{
    (void)context;
    CHECK(request->mode == 3U && request->start_flags <= 1U);
    ++host_start_calls;
    reply->status = PROTOCOL_ACCEPTED;
}
static void capture_keepalive(void *context)
{
    (void)context;
    ++keepalive_calls;
}
static void test_host_position_session_wire(void)
{
    CHECK(test_reset_protocol(3U));
    endpoint.motor.start = capture_host_start;
    endpoint.motor.host_position_keepalive = capture_keepalive;
    uint8_t payload[] = {3U, 1U, 0U, 0U};
    ProtocolExchange exchange = {.request = {.message_type = PROTOCOL_START,
                                             .source_node = PROTOCOL_MASTER_ID,
                                             .destination_node = 3U,
                                             .payload = payload,
                                             .payload_length = sizeof(payload)}};
    CHECK(handle_start_command(&endpoint, &exchange) == YG_PROTOCOL_OK);
    CHECK(YgProtocol_ReadU16Le(exchange.reply.data) == PROTOCOL_RESULT_BAD_FIELD &&
          host_start_calls == 0U);
    endpoint.device.capabilities.features |= PROTOCOL_FEATURE_HOST_POSITION_SESSION;
    CHECK(handle_start_command(&endpoint, &exchange) == YG_PROTOCOL_OK);
    CHECK(YgProtocol_ReadU16Le(exchange.reply.data) == PROTOCOL_RESULT_ACCEPTED &&
          host_start_calls == 1U);
    puts("HOST_POSITION_START 03010000");
    const uint8_t invalid[][4] = {
        {3U, 2U, 0U, 0U}, {2U, 1U, 0U, 0U}, {3U, 1U, 1U, 0U}, {3U, 1U, 0U, 1U}};
    for (unsigned i = 0U; i < sizeof(invalid) / sizeof(invalid[0]); ++i)
    {
        exchange.request.payload = invalid[i];
        CHECK(handle_start_command(&endpoint, &exchange) == YG_PROTOCOL_OK);
        CHECK(YgProtocol_ReadU16Le(exchange.reply.data) == PROTOCOL_RESULT_BAD_FIELD &&
              host_start_calls == 1U);
    }
    exchange.request.payload = payload;
    exchange.request.source_node = 9U;
    CHECK(handle_start_command(&endpoint, &exchange) == YG_PROTOCOL_OK);
    CHECK(YgProtocol_ReadU16Le(exchange.reply.data) == PROTOCOL_RESULT_FORBIDDEN &&
          host_start_calls == 1U);
    exchange.request.source_node = PROTOCOL_MASTER_ID;
    payload[1] = 0U;
    CHECK(handle_start_command(&endpoint, &exchange) == YG_PROTOCOL_OK && host_start_calls == 2U);
    exchange.request.message_type = PROTOCOL_GET_MOTOR_STATE;
    exchange.request.payload_length = 0U;
    CHECK(handle_motor_status_query(&endpoint, &exchange) == YG_PROTOCOL_OK &&
          keepalive_calls == 1U);
    exchange.request.source_node = 9U;
    CHECK(handle_motor_status_query(&endpoint, &exchange) == YG_PROTOCOL_OK &&
          keepalive_calls == 1U);
    exchange.request.source_node = PROTOCOL_MASTER_ID;
    exchange.request.payload_length = 4U;
    CHECK(handle_motor_status_query(&endpoint, &exchange) == YG_PROTOCOL_OK &&
          keepalive_calls == 1U);
}

static ProtocolCode hardware_report(void *context,
                                    uint8_t mode,
                                    uint16_t sequence,
                                    uint32_t expected_run,
                                    ProtocolHardwareReport *report)
{
    CHECK(context == &endpoint && sequence == 0x1234U);
    if (mode != 0U)
    {
        CHECK(mode == 2U && expected_run == 0x01020303U);
    }
    *report = (ProtocolHardwareReport){.run_id = 0x01020304U, .stage = 4U, .directions = 0x99U};
    memset(report->items, 1, sizeof(report->items));
    return mode == 0U ? PROTOCOL_RESULT_OK : PROTOCOL_RESULT_ACCEPTED;
}

static void test_hardware_wire(void)
{
    CHECK(test_reset_protocol(6U));
    uint8_t start[8] = {2, 1, 2, 0, 3, 3, 2, 1};
    ProtocolExchange exchange = {.request = {.version = YG_PROTOCOL_VERSION,
                                             .flags = YG_PROTOCOL_FLAGS_ACK_REQUEST,
                                             .message_type = 127U,
                                             .source_node = 2U,
                                             .destination_node = 6U,
                                             .sequence = 0x1234U,
                                             .payload = start,
                                             .payload_length = 8U}};
    CHECK(handle_command(&endpoint, &exchange) == YG_PROTOCOL_ROUTE_NOT_FOUND);
    endpoint.hardware_check =
        (ProtocolHardwareCheck){.context = &endpoint, .handle = hardware_report};
    CHECK(handle_command(&endpoint, &exchange) == YG_PROTOCOL_OK && exchange.reply.length == 21U &&
          exchange.reply.data[0] == 1);
    exchange.request.payload_length = 0;
    CHECK(handle_command(&endpoint, &exchange) == YG_PROTOCOL_OK && exchange.reply.length == 21U);
    printf("HARDWARE_REPORT ");
    for (unsigned i = 0; i < exchange.reply.length; ++i)
    {
        printf("%02x", exchange.reply.data[i]);
    }
    puts("");
    exchange.request.payload_length = 7;
    CHECK(handle_command(&endpoint, &exchange) == YG_PROTOCOL_OK && exchange.reply.length == 4 &&
          exchange.reply.data[0] == PROTOCOL_RESULT_BAD_LENGTH);
    exchange.request.payload_length = 8;
    start[3] = 1;
    CHECK(handle_command(&endpoint, &exchange) == YG_PROTOCOL_OK &&
          exchange.reply.data[0] == PROTOCOL_RESULT_BAD_FIELD);
    exchange.request.source_node = 8;
    CHECK(handle_command(&endpoint, &exchange) == YG_PROTOCOL_OK &&
          exchange.reply.data[0] == PROTOCOL_RESULT_FORBIDDEN);
}

int main(void)
{
    test_hardware_wire();
    test_host_position_session_wire();
    test_minimal_state();
    test_open_loop_vectors();
    CHECK(test_application_capabilities());
    puts("PASS motor wire vectors");
    return 0;
}
