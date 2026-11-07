/* 核心接口替换性测试：无真实业务处理器、无 HAL、无硬件动作。 */
#include <assert.h>
#include "yg_protocol.h"
#include <limits.h>
#include <math.h>
#include <string.h>
#include "yg_protocol.h"
#include "yg_protocol.h"
#include "yg_protocol.h"
#include "yg_protocol.h"

static unsigned contract_motor_calls;
static void motor_handler(void *context, const ProtocolCommand *request, ProtocolReply *reply)
{
    (void)context;
    ++contract_motor_calls;
    assert(request->position_mrad == -1000);
    reply->status = PROTOCOL_ACCEPTED;
    reply->token = 7U;
}
static void
missing_motor_result(void *context, const ProtocolCommand *request, ProtocolReply *reply)
{
    (void)context;
    (void)request;
    (void)reply;
}

static void job_handler(void *context, const ProtocolJobRequest *request, ProtocolJobReply *reply)
{
    (void)context;
    assert(request->operation == PROTOCOL_JOB_QUERY && request->job_id == 42U);
    reply->status = PROTOCOL_BUSY;
}

static void service_boundaries(void)
{
    assert(test_reset_protocol(3U));
    endpoint.motor = (ProtocolMotor){.start = missing_motor_result, .target = motor_handler};
    uint8_t payload[20] = {3U};
    YgProtocol_WriteU32Le(payload + 4U, (uint32_t)-1000);
    ProtocolExchange exchange = {.request = {.version = YG_PROTOCOL_VERSION,
                                             .flags = YG_PROTOCOL_FLAGS_ACK_REQUEST,
                                             .source_node = 2U,
                                             .destination_node = 3U,
                                             .message_type = PROTOCOL_CONTROL,
                                             .payload = payload,
                                             .payload_length = sizeof(payload)}};
    contract_motor_calls = 0U;
    assert(handle_command(&endpoint, &exchange) == YG_PROTOCOL_OK);
    assert(exchange.reply.data[0] == 1U && contract_motor_calls == 1U);
    exchange.request.payload_length = 19U;
    assert(handle_command(&endpoint, &exchange) == YG_PROTOCOL_OK);
    assert(exchange.reply.data[0] == 3U && contract_motor_calls == 1U);
    exchange.request.payload_length = sizeof(payload);
    endpoint.motor.target = NULL;
    assert(handle_command(&endpoint, &exchange) == YG_PROTOCOL_OK);
    assert(exchange.reply.data[0] == 2U && contract_motor_calls == 1U);
    exchange.request.message_type = PROTOCOL_START;
    exchange.request.payload_length = 4U;
    assert(handle_command(&endpoint, &exchange) == YG_PROTOCOL_OK);
    assert(exchange.reply.data[0] == 8U);
    endpoint.jobs = (ProtocolJobs){NULL, job_handler};
    exchange.request.message_type = PROTOCOL_GET_JOB_STATUS;
    YgProtocol_WriteU32Le(payload, 42U);
    assert(handle_command(&endpoint, &exchange) == YG_PROTOCOL_OK);
    assert(exchange.reply.data[0] == 6U);
    endpoint.jobs.handler = NULL;
    assert(handle_command(&endpoint, &exchange) == YG_PROTOCOL_OK);
    assert(exchange.reply.data[0] == 2U);
}

static ProtocolMotorStatus status_source(void)
{
    return (ProtocolMotorStatus){.state = 1U, .sample_available = true};
}

static void numeric_boundaries(void)
{
    ProtocolMotorStatus source = status_source();
    ProtocolFeedback feedback = {.result = 0x1234U};
    source.state = 10U;
    assert(Protocol_BuildFeedback(&source, 3U, 0U, 0U, &feedback) == YG_PROTOCOL_INVALID_FIELD &&
           feedback.result == 0x1234U);
    source.state = 1U;
    source.mode = 11U;
    assert(Protocol_BuildFeedback(&source, 3U, 0U, 0U, &feedback) == YG_PROTOCOL_INVALID_FIELD);
    source.mode = 0U;
    source.faults = 0x40000U;
    assert(Protocol_BuildFeedback(&source, 3U, 0U, 0U, &feedback) == YG_PROTOCOL_INVALID_FIELD);
}

static void full_feedback_snapshot(void)
{
    ProtocolMotorStatus source = status_source();
    ProtocolFeedback feedback;
    source.mode = 2U;
    source.faults = 4U;
    assert(Protocol_BuildFeedback(&source, 3U, 1U, 41U, &feedback) == YG_PROTOCOL_OK);
    assert(feedback.result == 1U && feedback.correlated_seq == 41U && feedback.fault_code == 4U &&
           feedback.control_mode == 2U && feedback.motor_state == 1U && feedback.valid_bits == 0U);
    assert(feedback.reference_position_mrad == INT32_MIN &&
           feedback.actual_position_mrad == INT32_MIN &&
           feedback.reference_speed_mrad_s == INT32_MIN &&
           feedback.actual_speed_mrad_s == INT32_MIN && feedback.reference_iq_mA == INT16_MIN &&
           feedback.actual_iq_mA == INT16_MIN && feedback.bus_voltage_cV == INT16_MIN &&
           feedback.bus_current_mA == INT16_MIN && feedback.mcu_temperature_centi_c == INT16_MIN &&
           feedback.motor_temperature_centi_c == INT16_MIN && feedback.v_q_mV == INT16_MIN &&
           feedback.v_d_mV == INT16_MIN);
    source.sample_available = false;
    assert(Protocol_BuildFeedback(&source, 3U, 6U, 42U, &feedback) == YG_PROTOCOL_OK);
    assert(feedback.valid_bits == 0U && feedback.correlated_seq == 42U &&
           feedback.fault_code == 4U);
}

static void full_feedback_vector(void)
{
    ProtocolFeedback feedback = {
        .result = 0U,
        .correlated_seq = 0x1234U,
        .fault_code = 0x01020304U,
        .reference_position_mrad = 1000,
        .actual_position_mrad = -2000,
        .reference_speed_mrad_s = 3000,
        .actual_speed_mrad_s = -4000,
        .valid_bits = 0x0DFFU,
        .bus_voltage_cV = 2400,
        .bus_current_mA = 1500,
        .reference_iq_mA = -1200,
        .actual_iq_mA = 1100,
        .mcu_temperature_centi_c = 2534,
        .motor_temperature_centi_c = INT16_MIN,
        .v_q_mV = 5000,
        .v_d_mV = -250,
        .node_id = 3U,
        .motor_state = 6U,
        .control_mode = 3U,
    };
    ProtocolResponse encoded_response = {0};
    uint8_t *encoded = encoded_response.data;
    static const uint8_t golden[PROTOCOL_FEEDBACK_SIZE] = {
        0x00, 0x00, 0x34, 0x12, 0x04, 0x03, 0x02, 0x01, 0xE8, 0x03, 0x00, 0x00, 0x30, 0xF8, 0xFF,
        0xFF, 0xB8, 0x0B, 0x00, 0x00, 0x60, 0xF0, 0xFF, 0xFF, 0xFF, 0x0D, 0x60, 0x09, 0xDC, 0x05,
        0x50, 0xFB, 0x4C, 0x04, 0xE6, 0x09, 0x00, 0x80, 0x88, 0x13, 0x06, 0xFF, 0x03, 0x06, 0x03,
    };
    assert(encode_motor_feedback(&feedback, &encoded_response) == YG_PROTOCOL_OK);
    assert(memcmp(encoded, golden, sizeof(golden)) == 0);
    /* v1.1 八状态编码黄金向量：只变更第 43 字节，其余字段布局保持。 */
    const uint8_t states_v1_1[] = {3U, 5U, 2U, 6U, 5U, 7U, 8U, 9U};
    uint8_t state_golden[PROTOCOL_FEEDBACK_SIZE];
    memcpy(state_golden, golden, sizeof(golden));
    for (unsigned index = 0U; index < sizeof(states_v1_1); ++index)
    {
        feedback.motor_state = states_v1_1[index];
        state_golden[43] = states_v1_1[index];
        assert(encode_motor_feedback(&feedback, &encoded_response) == YG_PROTOCOL_OK);
        assert(memcmp(encoded, state_golden, sizeof(state_golden)) == 0);
    }
    feedback.motor_state = golden[43];
    assert(encode_motor_feedback(&feedback, &encoded_response) == YG_PROTOCOL_OK);
    yg_protocol_message_t message = {
        YG_PROTOCOL_VERSION, 0U, 3U, 2U, 124U, 7U, PROTOCOL_FEEDBACK_SIZE, 0U, encoded};
    BspCanFrame frame;
    yg_protocol_message_t decoded;
    assert(encode_can_frame(&message, 3U, &frame) == YG_PROTOCOL_OK);
    assert(frame.identifier == 0x0CEF0203U && frame.length == 64U && frame.data[63] == 0U);
    assert(decode_can_frame(&frame, 2U, &decoded) == YG_PROTOCOL_OK);
    assert(decoded.message_type == 124U && decoded.sequence == 7U &&
           decoded.payload_length == sizeof(golden) &&
           memcmp(decoded.payload, golden, sizeof(golden)) == 0);
    memset(encoded, 0xA5, PROTOCOL_FEEDBACK_SIZE);
    feedback.valid_bits = 0x1000U;
    assert(encode_motor_feedback(&feedback, &encoded_response) == YG_PROTOCOL_INVALID_FIELD);
    for (size_t index = 0U; index < PROTOCOL_FEEDBACK_SIZE; ++index)
    {
        assert(encoded[index] == 0xA5U);
    }
}

static void motor_command_vectors(void)
{
    static const uint8_t status_query_frame[20] = {0x5A, 0xA5, 0x01, 0x20, 0x02, 0x03, 0x6C,
                                                   0x00, 0x2A, 0x00, 0x00, 0x00, 0x00, 0x00,
                                                   0x00, 0xFA, 0xF5, 0xDA, 0x00, 0x00};
    static const uint8_t enable_frame[24] = {
        0x5A, 0xA5, 0x01, 0x20, 0x02, 0x03, 0x65, 0x00, 0x28, 0x00, 0x04, 0x00,
        0x00, 0x00, 0x00, 0xF3, 0x01, 0x00, 0x00, 0x00, 0x31, 0xBF, 0x00, 0x00,
    };
    static const uint8_t control_frame[48] = {
        0x5A, 0xA5, 0x01, 0x20, 0x02, 0x03, 0x74, 0x00, 0x29, 0x00, 0x14, 0x00,
        0x00, 0x00, 0x00, 0x2D, 0x03, 0x00, 0x00, 0x00, 0xE8, 0x03, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x6B, 0x82, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    };
    uint8_t enable_payload[4] = {1U, 0U, 0U, 0U};
    uint8_t control_payload[20] = {3U, 0U, 0U, 0U, 0xE8U, 0x03U};
    yg_protocol_message_t message = {YG_PROTOCOL_VERSION,
                                     YG_PROTOCOL_FLAGS_ACK_REQUEST,
                                     2U,
                                     3U,
                                     PROTOCOL_ENABLE,
                                     40U,
                                     sizeof(enable_payload),
                                     0U,
                                     enable_payload};
    BspCanFrame frame;
    ProtocolCommand request = {.position_mrad = -7};
    ProtocolExchange enable = {.request = message};

    assert(encode_can_frame(&message, 2U, &frame) == YG_PROTOCOL_OK);
    assert(frame.identifier == 0x08EF0302U && frame.length == sizeof(enable_frame));
    assert(memcmp(frame.data, enable_frame, sizeof(enable_frame)) == 0);
    assert(handle_command(&endpoint, &enable) == YG_PROTOCOL_OK);
    assert(YgProtocol_ReadU16Le(enable.reply.data) == PROTOCOL_RESULT_UNSUPPORTED);
    enable_payload[1] = 1U;
    assert(handle_command(&endpoint, &enable) == YG_PROTOCOL_OK);
    assert(YgProtocol_ReadU16Le(enable.reply.data) == PROTOCOL_RESULT_BAD_FIELD);
    enable_payload[1] = 0U;

    message.message_type = PROTOCOL_CONTROL;
    message.sequence = 41U;
    message.payload = control_payload;
    message.payload_length = sizeof(control_payload);
    assert(encode_can_frame(&message, 2U, &frame) == YG_PROTOCOL_OK);
    assert(frame.identifier == 0x08EF0302U && frame.length == sizeof(control_frame));
    assert(memcmp(frame.data, control_frame, sizeof(control_frame)) == 0);
    assert(decode_target_command(&message, &request) == YG_PROTOCOL_OK);
    assert(request.mode == 3U && request.position_mrad == 1000 && request.sequence == 41U);
    control_payload[12] = 1U;
    assert(decode_target_command(&message, &request) == YG_PROTOCOL_INVALID_FIELD);
    assert(request.position_mrad == 1000);
    control_payload[12] = 0U;
    message.payload_length = 19U;
    assert(decode_target_command(&message, &request) == YG_PROTOCOL_INVALID_LENGTH);

    message.message_type = PROTOCOL_GET_MOTOR_STATE;
    message.sequence = 42U;
    message.payload = NULL;
    message.payload_length = 0U;
    assert(encode_can_frame(&message, 2U, &frame) == YG_PROTOCOL_OK);
    assert(frame.length == sizeof(status_query_frame) &&
           memcmp(frame.data, status_query_frame, sizeof(status_query_frame)) == 0);
}

static void maintenance_kind_extension_vectors(void)
{
    static const uint8_t mechanical_reply[PROTOCOL_JOB_REPLY_SIZE] = {
        1U, 0U, 0U, 0U, 0x78U, 0x56U, 0x34U, 0x12U, 6U, 0U, 1U, 1U};
    static const uint8_t unsupported_reply[PROTOCOL_JOB_REPLY_SIZE] = {
        2U, 0U, 1U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U};
    uint8_t payload[PROTOCOL_JOB_REQUEST_SIZE] = {0};
    ProtocolResponse encoded_response = {0};
    uint8_t *encoded = encoded_response.data;
    const yg_protocol_message_t message = {.message_type = PROTOCOL_START_JOB,
                                           .source_node = 2U,
                                           .sequence = 17U,
                                           .payload_length = sizeof(payload),
                                           .payload = payload};
    ProtocolJobRequest request = {0};
    ProtocolJobReply reply = {.status = PROTOCOL_ACCEPTED,
                              .token = 0x12345678U,
                              .phase = PROTOCOL_JOB_PHASE_IN_PROGRESS,
                              .effects = PROTOCOL_JOB_EFFECT_UNCOMMITTED};
    for (uint16_t kind = PROTOCOL_JOB_KIND_COGGING; kind <= PROTOCOL_JOB_KIND_FRICTION; ++kind)
    {
        payload[0] = (uint8_t)kind;
        reply.kind = kind;
        assert(decode_job_command(&message, &request) == YG_PROTOCOL_OK);
        assert(request.kind == kind && request.source_node == 2U && request.sequence == 17U);
        assert(encode_job_reply(&reply, &encoded_response) == YG_PROTOCOL_INVALID_FIELD);
    }
    for (uint16_t kind = PROTOCOL_JOB_KIND_MECHANICAL_ZERO; kind <= PROTOCOL_JOB_KIND_DEFAULTS;
         ++kind)
    {
        payload[0] = (uint8_t)kind;
        reply.kind = kind;
        assert(decode_job_command(&message, &request) == YG_PROTOCOL_OK);
        assert(request.kind == kind && request.source_node == 2U && request.sequence == 17U);
        assert(encode_job_reply(&reply, &encoded_response) == YG_PROTOCOL_OK);
        assert(encoded_response.length == PROTOCOL_JOB_REPLY_SIZE && encoded[0] == 1U &&
               encoded[4] == 0x78U && encoded[8] == kind &&
               encoded[10] == PROTOCOL_JOB_PHASE_IN_PROGRESS &&
               encoded[11] == PROTOCOL_JOB_EFFECT_UNCOMMITTED);
        if (kind == PROTOCOL_JOB_KIND_MECHANICAL_ZERO)
        {
            assert(memcmp(encoded, mechanical_reply, sizeof(mechanical_reply)) == 0);
        }
    }
    payload[0] = PROTOCOL_JOB_KIND_DEFAULTS + 1U;
    assert(decode_job_command(&message, &request) == YG_PROTOCOL_OK);
    reply.kind = PROTOCOL_JOB_KIND_DEFAULTS + 1U;
    assert(encode_job_reply(&reply, &encoded_response) == YG_PROTOCOL_INVALID_FIELD);
    reply = (ProtocolJobReply){.status = PROTOCOL_UNSUPPORTED,
                               .detail = PROTOCOL_JOB_DETAIL_UNSUPPORTED_KIND};
    assert(encode_job_reply(&reply, &encoded_response) == YG_PROTOCOL_OK);
    assert(memcmp(encoded, unsupported_reply, sizeof(unsupported_reply)) == 0);
}

void yg_protocol_contract_test(void)
{
    service_boundaries();
    numeric_boundaries();
    full_feedback_snapshot();
    full_feedback_vector();
    motor_command_vectors();
    maintenance_kind_extension_vectors();
}
