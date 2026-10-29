/* 核心接口替换性测试：无真实业务处理器、无 HAL、无硬件动作。 */
#include <assert.h>
#include <limits.h>
#include <math.h>
#include <string.h>
#include "yg_protocol_endpoint.h"
#include "yg_protocol_motor.h"
#include "yg_protocol_parameter.h"
#include "yg_protocol_job.h"
#include "yg_protocol_readonly.h"

static void motor_handler(void *context,
                          const yg_protocol_motor_request_t *request,
                          yg_protocol_service_reply_t *reply)
{
    unsigned *calls = context;
    ++*calls;
    assert(request->operation == YG_PROTOCOL_MOTOR_SET_CONTROL);
    assert(request->position_mrad == -1000);
    reply->status = YG_PROTOCOL_SERVICE_ACCEPTED;
    reply->token = 7U;
}

static void parameter_handler(void *context,
                              const yg_protocol_parameter_request_t *request,
                              yg_protocol_service_reply_t *reply)
{
    (void)context;
    assert(request->operation == YG_PROTOCOL_PARAMETER_READ);
    reply->status = YG_PROTOCOL_SERVICE_OK;
    reply->value = 123;
    reply->revision = 2U;
}

static void
job_handler(void *context, const yg_protocol_job_request_t *request, yg_protocol_job_reply_t *reply)
{
    (void)context;
    assert(request->operation == YG_PROTOCOL_JOB_QUERY && request->job_id == 42U);
    reply->status = YG_PROTOCOL_SERVICE_BUSY;
}

static void service_boundaries(void)
{
    unsigned calls = 0U;
    yg_protocol_motor_service_t motor = {&calls, motor_handler};
    yg_protocol_parameter_service_t parameter = {NULL, parameter_handler};
    yg_protocol_job_service_t job = {NULL, job_handler};
    yg_protocol_motor_request_t motor_request = {.operation = YG_PROTOCOL_MOTOR_SET_CONTROL,
                                                 .position_mrad = -1000};
    yg_protocol_parameter_request_t parameter_request = {.operation = YG_PROTOCOL_PARAMETER_READ};
    yg_protocol_job_reply_t job_reply;
    yg_protocol_job_request_t job_request = {.operation = YG_PROTOCOL_JOB_QUERY, .job_id = 42U};
    yg_protocol_service_reply_t reply;
    assert(yg_protocol_motor_call(&motor, &motor_request, &reply) == YG_PROTOCOL_SERVICE_ACCEPTED);
    assert(calls == 1U && reply.token == 7U);
    motor_request.operation = (yg_protocol_motor_operation_t)-1;
    assert(yg_protocol_motor_call(&motor, &motor_request, &reply) ==
           YG_PROTOCOL_SERVICE_INVALID_ARGUMENT);
    motor_request.operation = (yg_protocol_motor_operation_t)(YG_PROTOCOL_MOTOR_CLEAR_FAULT + 1U);
    assert(yg_protocol_motor_call(&motor, &motor_request, &reply) ==
           YG_PROTOCOL_SERVICE_INVALID_ARGUMENT);
    assert(calls == 1U);
    motor_request.operation = YG_PROTOCOL_MOTOR_SET_CONTROL;
    assert(yg_protocol_parameter_call(&parameter, &parameter_request, &reply) ==
           YG_PROTOCOL_SERVICE_OK);
    assert(reply.value == 123 && reply.revision == 2U && reply.token == 0U);
    assert(yg_protocol_job_call(&job, &job_request, &job_reply) == YG_PROTOCOL_SERVICE_BUSY);
    motor.handler = NULL;
    parameter.handler = NULL;
    job.handler = NULL;
    assert(yg_protocol_motor_call(&motor, &motor_request, &reply) ==
           YG_PROTOCOL_SERVICE_UNSUPPORTED);
    assert(yg_protocol_parameter_call(&parameter, &parameter_request, &reply) ==
           YG_PROTOCOL_SERVICE_UNSUPPORTED);
    assert(yg_protocol_job_call(&job, &job_request, &job_reply) == YG_PROTOCOL_SERVICE_UNSUPPORTED);
    assert(yg_protocol_motor_call(NULL, &motor_request, &reply) ==
           YG_PROTOCOL_SERVICE_INVALID_ARGUMENT);
}

static yg_protocol_motor_status_source_t status_source(void)
{
    yg_protocol_motor_status_source_t source = {0};
    source.sample.position_feedback = 0.0625F;
    source.sample.speed_feedback_rad_s = -0.0625F;
    source.sample.current_feedback = 1.25F;
    source.sample.bus_voltage = 24.0F;
    source.sample.bus_current = 1.5F;
    source.sample.temperature = -25.25F;
    source.measurement_valid_bits = YG_PROTOCOL_MOTOR_MEASUREMENT_MASK;
    source.sample_available = true;
    source.state = 1U;
    return source;
}

static void numeric_boundaries(void)
{
    yg_protocol_motor_status_source_t source = status_source();
    yg_protocol_motor_feedback_t feedback;
    source.sample.position_feedback = NAN;
    source.sample.speed_feedback_rad_s = INFINITY;
    source.sample.current_feedback = -INFINITY;
    source.sample.bus_voltage = 400.0F;
    source.sample.bus_current = NAN;
    source.sample.temperature = 400.0F;
    assert(yg_protocol_motor_feedback_from_source(&source, 3U, 0U, 0U, &feedback) ==
           YG_PROTOCOL_OK);
    assert(feedback.actual_position_mrad == INT32_MIN &&
           feedback.actual_speed_mrad_s == INT32_MIN && feedback.actual_iq_mA == INT16_MIN);
    assert(feedback.bus_voltage_cV == INT16_MIN && feedback.bus_current_mA == INT16_MIN &&
           feedback.mcu_temperature_centi_c == INT16_MIN);
    source = status_source();
    source.sample.position_feedback = 2147483.75F;
    source.sample.speed_feedback_rad_s = 32.767F;
    source.sample.current_feedback = -32.768F;
    source.measurement_valid_bits = 0U;
    assert(yg_protocol_motor_feedback_from_source(&source, 3U, 0U, 0U, &feedback) ==
           YG_PROTOCOL_OK);
    assert((feedback.valid_bits & 0x1BAU) == 0U);
    source.mode = 18U;
    feedback.result = 0x1234U;
    assert(yg_protocol_motor_feedback_from_source(&source, 3U, 0U, 0U, &feedback) ==
           YG_PROTOCOL_INVALID_FIELD);
    assert(feedback.result == 0x1234U);
}

static void full_feedback_snapshot(void)
{
    yg_protocol_motor_status_source_t source = status_source();
    yg_protocol_motor_feedback_t feedback;
    source.sample.position_target = 1.0F;
    source.sample.speed_target = 2.0F;
    source.sample.current_reference = 3.0F;
    source.faults = 4U;
    source.mode = 3U;
    assert(yg_protocol_motor_feedback_from_source(&source, 3U, 1U, 41U, &feedback) ==
           YG_PROTOCOL_OK);
    assert(feedback.result == 1U && feedback.correlated_seq == 41U && feedback.fault_code == 4U &&
           feedback.reference_position_mrad == 1000 && feedback.actual_position_mrad == 63 &&
           feedback.reference_speed_mrad_s == 2000 && feedback.bus_voltage_cV == 2400 &&
           feedback.reference_iq_mA == 3000 && feedback.actual_iq_mA == 1250 &&
           feedback.valid_bits == 0x01FFU && feedback.motor_temperature_centi_c == INT16_MIN &&
           feedback.v_q_mV == INT16_MIN);
    source.sample_available = false;
    assert(yg_protocol_motor_feedback_from_source(&source, 3U, 6U, 42U, &feedback) ==
           YG_PROTOCOL_OK);
    assert(feedback.valid_bits == 0U && feedback.actual_position_mrad == INT32_MIN &&
           feedback.fault_code == 4U && feedback.correlated_seq == 42U);
}

static void full_feedback_vector(void)
{
    yg_protocol_motor_feedback_t feedback = {
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
    uint8_t encoded[YG_PROTOCOL_MOTOR_FEEDBACK_PAYLOAD_SIZE] = {0};
    static const uint8_t golden[YG_PROTOCOL_MOTOR_FEEDBACK_PAYLOAD_SIZE] = {
        0x00, 0x00, 0x34, 0x12, 0x04, 0x03, 0x02, 0x01, 0xE8, 0x03, 0x00, 0x00, 0x30, 0xF8, 0xFF,
        0xFF, 0xB8, 0x0B, 0x00, 0x00, 0x60, 0xF0, 0xFF, 0xFF, 0xFF, 0x0D, 0x60, 0x09, 0xDC, 0x05,
        0x50, 0xFB, 0x4C, 0x04, 0xE6, 0x09, 0x00, 0x80, 0x88, 0x13, 0x06, 0xFF, 0x03, 0x06, 0x03,
    };
    assert(yg_protocol_motor_feedback_encode(&feedback, encoded, sizeof(encoded)) ==
           YG_PROTOCOL_OK);
    assert(memcmp(encoded, golden, sizeof(golden)) == 0);
    /* v1.1 八状态编码黄金向量：只变更第 43 字节，其余字段布局保持。 */
    const uint8_t states_v1_1[] = {3U, 5U, 2U, 6U, 5U, 7U, 8U, 9U};
    uint8_t state_golden[YG_PROTOCOL_MOTOR_FEEDBACK_PAYLOAD_SIZE];
    memcpy(state_golden, golden, sizeof(golden));
    for (unsigned index = 0U; index < sizeof(states_v1_1); ++index)
    {
        feedback.motor_state = states_v1_1[index];
        state_golden[43] = states_v1_1[index];
        assert(yg_protocol_motor_feedback_encode(&feedback, encoded, sizeof(encoded)) ==
               YG_PROTOCOL_OK);
        assert(memcmp(encoded, state_golden, sizeof(state_golden)) == 0);
    }
    feedback.motor_state = golden[43];
    assert(yg_protocol_motor_feedback_encode(&feedback, encoded, sizeof(encoded)) ==
           YG_PROTOCOL_OK);
    yg_protocol_message_t message = {
        YG_PROTOCOL_VERSION, 0U, 3U, 2U, 124U, 7U, sizeof(encoded), 0U, encoded};
    yg_protocol_transfer_frame_t frame;
    yg_protocol_message_t decoded;
    assert(yg_protocol_canfd_pack(&message, 3U, &frame) == YG_PROTOCOL_OK);
    assert(frame.identifier == 0x0CEF0203U && frame.length == 64U && frame.data[63] == 0U);
    assert(yg_protocol_canfd_unpack(&frame, 2U, &decoded) == YG_PROTOCOL_OK);
    assert(decoded.message_type == 124U && decoded.sequence == 7U &&
           decoded.payload_length == sizeof(golden) &&
           memcmp(decoded.payload, golden, sizeof(golden)) == 0);
    memset(encoded, 0xA5, sizeof(encoded));
    assert(yg_protocol_motor_feedback_encode(&feedback, encoded, sizeof(encoded) - 1U) ==
           YG_PROTOCOL_BUFFER_TOO_SMALL);
    for (size_t index = 0U; index < sizeof(encoded); ++index)
    {
        assert(encoded[index] == 0xA5U);
    }
    feedback.valid_bits = 0x1000U;
    assert(yg_protocol_motor_feedback_encode(&feedback, encoded, sizeof(encoded)) ==
           YG_PROTOCOL_INVALID_FIELD);
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
                                     YG_PROTOCOL_MOTOR_TYPE_ENABLE,
                                     40U,
                                     sizeof(enable_payload),
                                     0U,
                                     enable_payload};
    yg_protocol_transfer_frame_t frame;
    yg_protocol_motor_request_t request = {.position_mrad = -7};
    bool enabled = false;

    assert(yg_protocol_canfd_pack(&message, 2U, &frame) == YG_PROTOCOL_OK);
    assert(frame.identifier == 0x08EF0302U && frame.length == sizeof(enable_frame));
    assert(memcmp(frame.data, enable_frame, sizeof(enable_frame)) == 0);
    assert(yg_protocol_motor_decode_enable(&message, &enabled) == YG_PROTOCOL_OK && enabled);
    enable_payload[1] = 1U;
    assert(yg_protocol_motor_decode_enable(&message, &enabled) == YG_PROTOCOL_INVALID_FIELD);
    assert(enabled);
    enable_payload[1] = 0U;

    message.message_type = YG_PROTOCOL_MOTOR_TYPE_CONTROL;
    message.sequence = 41U;
    message.payload = control_payload;
    message.payload_length = sizeof(control_payload);
    assert(yg_protocol_canfd_pack(&message, 2U, &frame) == YG_PROTOCOL_OK);
    assert(frame.identifier == 0x08EF0302U && frame.length == sizeof(control_frame));
    assert(memcmp(frame.data, control_frame, sizeof(control_frame)) == 0);
    assert(yg_protocol_motor_decode_control(&message, &request) == YG_PROTOCOL_OK);
    assert(request.operation == YG_PROTOCOL_MOTOR_SET_CONTROL && request.mode == 3U &&
           request.position_mrad == 1000 && request.sequence == 41U);
    control_payload[12] = 1U;
    assert(yg_protocol_motor_decode_control(&message, &request) == YG_PROTOCOL_INVALID_FIELD);
    assert(request.position_mrad == 1000);
    control_payload[12] = 0U;
    message.payload_length = 19U;
    assert(yg_protocol_motor_decode_control(&message, &request) == YG_PROTOCOL_INVALID_LENGTH);

    message.message_type = YG_PROTOCOL_READONLY_TYPE_GET_MOTOR_STATE;
    message.sequence = 42U;
    message.payload = NULL;
    message.payload_length = 0U;
    assert(yg_protocol_canfd_pack(&message, 2U, &frame) == YG_PROTOCOL_OK);
    assert(frame.length == sizeof(status_query_frame) &&
           memcmp(frame.data, status_query_frame, sizeof(status_query_frame)) == 0);
}

static void maintenance_kind_extension_vectors(void)
{
    static const uint8_t mechanical_reply[YG_PROTOCOL_JOB_REPLY_SIZE] = {
        1U, 0U, 0U, 0U, 0x78U, 0x56U, 0x34U, 0x12U, 6U, 0U, 1U, 1U};
    static const uint8_t unsupported_reply[YG_PROTOCOL_JOB_REPLY_SIZE] = {
        2U, 0U, 1U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U};
    uint8_t payload[YG_PROTOCOL_JOB_REQUEST_SIZE] = {0};
    uint8_t encoded[YG_PROTOCOL_JOB_REPLY_SIZE] = {0};
    const yg_protocol_message_t message = {.message_type = YG_PROTOCOL_JOB_TYPE_START,
                                           .source_node = 2U,
                                           .sequence = 17U,
                                           .payload_length = sizeof(payload),
                                           .payload = payload};
    yg_protocol_job_request_t request = {0};
    yg_protocol_job_reply_t reply = {.status = YG_PROTOCOL_SERVICE_ACCEPTED,
                                     .token = 0x12345678U,
                                     .phase = YG_PROTOCOL_JOB_PHASE_IN_PROGRESS,
                                     .effects = YG_PROTOCOL_JOB_EFFECT_UNCOMMITTED};
    size_t written = 0U;
    for (uint16_t kind = YG_PROTOCOL_JOB_KIND_COGGING; kind <= YG_PROTOCOL_JOB_KIND_FRICTION;
         ++kind)
    {
        payload[0] = (uint8_t)kind;
        reply.kind = kind;
        assert(yg_protocol_job_decode(&message, &request) == YG_PROTOCOL_OK);
        assert(request.kind == kind && request.source_node == 2U && request.sequence == 17U);
        assert(yg_protocol_job_encode_reply(&reply, encoded, sizeof(encoded), &written) ==
               YG_PROTOCOL_INVALID_FIELD);
    }
    for (uint16_t kind = YG_PROTOCOL_JOB_KIND_MECHANICAL_ZERO;
         kind <= YG_PROTOCOL_JOB_KIND_DEFAULTS;
         ++kind)
    {
        payload[0] = (uint8_t)kind;
        reply.kind = kind;
        assert(yg_protocol_job_decode(&message, &request) == YG_PROTOCOL_OK);
        assert(request.kind == kind && request.source_node == 2U && request.sequence == 17U);
        assert(yg_protocol_job_encode_reply(&reply, encoded, sizeof(encoded), &written) ==
               YG_PROTOCOL_OK);
        assert(written == sizeof(encoded) && encoded[0] == 1U && encoded[4] == 0x78U &&
               encoded[8] == kind && encoded[10] == YG_PROTOCOL_JOB_PHASE_IN_PROGRESS &&
               encoded[11] == YG_PROTOCOL_JOB_EFFECT_UNCOMMITTED);
        if (kind == YG_PROTOCOL_JOB_KIND_MECHANICAL_ZERO)
        {
            assert(memcmp(encoded, mechanical_reply, sizeof(mechanical_reply)) == 0);
        }
    }
    payload[0] = YG_PROTOCOL_JOB_KIND_DEFAULTS + 1U;
    assert(yg_protocol_job_decode(&message, &request) == YG_PROTOCOL_OK);
    reply.kind = YG_PROTOCOL_JOB_KIND_DEFAULTS + 1U;
    assert(yg_protocol_job_encode_reply(&reply, encoded, sizeof(encoded), &written) ==
           YG_PROTOCOL_INVALID_FIELD);
    reply = (yg_protocol_job_reply_t){.status = YG_PROTOCOL_SERVICE_UNSUPPORTED,
                                      .detail = YG_PROTOCOL_JOB_DETAIL_UNSUPPORTED_KIND};
    assert(yg_protocol_job_encode_reply(&reply, encoded, sizeof(encoded), &written) ==
           YG_PROTOCOL_OK);
    assert(memcmp(encoded, unsupported_reply, sizeof(unsupported_reply)) == 0);
}

static void endpoint_round_trip(void)
{
    yg_protocol_motor_status_source_t source = status_source();
    yg_protocol_readonly_service_t service = {&source, yg_protocol_motor_status_provider};
    const yg_protocol_command_t entries[] = {
        {YG_PROTOCOL_READONLY_TYPE_GET_MOTOR_STATE, false, yg_protocol_readonly_handle, &service},
        {YG_PROTOCOL_READONLY_TYPE_GET_INFO, true, NULL, NULL},
        {YG_PROTOCOL_READONLY_TYPE_MOTION_FEEDBACK, false, NULL, NULL},
    };
    yg_protocol_commands_t commands;
    yg_protocol_transfer_frame_t rx_storage[3], tx_storage[2], frame;
    yg_protocol_transfer_queue_t rx, tx;
    uint8_t fragments[96];
    yg_protocol_endpoint_t endpoint;
    yg_protocol_endpoint_config_t config = {
        3U, 6U, 200U, &rx, &tx, &commands, fragments, sizeof(fragments)};
    yg_protocol_message_t request = {
        1U, YG_PROTOCOL_FLAGS_ACK_REQUEST, 2U, 3U, 108U, 9U, 0U, 0U, NULL};
    yg_protocol_message_t response;
    assert(yg_protocol_commands_register(&commands, entries, 3U) == YG_PROTOCOL_OK);
    assert(yg_protocol_transfer_queue_init(&rx, rx_storage, 3U));
    assert(yg_protocol_transfer_queue_init(&tx, tx_storage, 2U));
    assert(yg_protocol_endpoint_init(&endpoint, &config) == YG_PROTOCOL_OK);
    assert(yg_protocol_endpoint_process_one(&endpoint, 0U) == YG_PROTOCOL_QUEUE_EMPTY);
    assert(yg_protocol_canfd_pack(&request, 6U, &frame) == YG_PROTOCOL_OK);
    assert(yg_protocol_transfer_queue_push(&rx, &frame));
    assert(yg_protocol_transfer_queue_push(&tx, &frame));
    assert(yg_protocol_transfer_queue_push(&tx, &frame));
    assert(yg_protocol_endpoint_process_one(&endpoint, 1U) == YG_PROTOCOL_QUEUE_FULL);
    assert(yg_protocol_endpoint_process_frame(&endpoint, &frame, 1U) == YG_PROTOCOL_QUEUE_FULL);
    assert(endpoint.response_pending);
    source.sample_available = false;
    assert(yg_protocol_transfer_queue_pop(&tx, &frame));
    assert(yg_protocol_endpoint_process_one(&endpoint, 2U) == YG_PROTOCOL_OK);
    assert(yg_protocol_transfer_queue_pop(&tx, &frame));
    assert(yg_protocol_transfer_queue_pop(&tx, &frame));
    assert(frame.length == 64U && frame.identifier == 0x18EF0203U);
    assert(yg_protocol_canfd_unpack(&frame, 2U, &response) == YG_PROTOCOL_OK);
    assert(response.flags == YG_PROTOCOL_FLAGS_RESPONSE && response.sequence == 9U);
    assert(response.payload_length == YG_PROTOCOL_MOTOR_FEEDBACK_PAYLOAD_SIZE);
    assert(response.payload[0] == 0U && response.payload[1] == 0U && response.payload[2] == 0U &&
           response.payload[3] == 0U);
    assert(response.payload[12] == 63U && response.payload[42] == 3U);
    /* TX 满后仍发原先的 OK 快照，没有重新调用 provider 变成 BUSY。 */
    assert(yg_protocol_canfd_pack(&request, 6U, &frame) == YG_PROTOCOL_OK);
    assert(yg_protocol_transfer_queue_push(&rx, &frame));
    assert(yg_protocol_endpoint_process_one(&endpoint, 3U) == YG_PROTOCOL_OK);
    assert(yg_protocol_transfer_queue_pop(&tx, &frame));
    assert(yg_protocol_canfd_unpack(&frame, 2U, &response) == YG_PROTOCOL_OK);
    assert(response.payload_length == YG_PROTOCOL_MOTOR_FEEDBACK_PAYLOAD_SIZE &&
           response.payload[0] == 6U && response.payload[24] == 0U);
    request.flags = YG_PROTOCOL_FLAGS_RESPONSE;
    assert(yg_protocol_canfd_pack(&request, 6U, &frame) == YG_PROTOCOL_OK);
    assert(yg_protocol_transfer_queue_push(&rx, &frame));
    assert(yg_protocol_endpoint_process_one(&endpoint, 4U) == YG_PROTOCOL_INVALID_FIELD);
    assert(yg_protocol_transfer_queue_count(&tx) == 0U);
    request.flags = YG_PROTOCOL_FLAGS_ACK_REQUEST;
    request.destination_node = 0xFFU;
    assert(yg_protocol_canfd_pack(&request, 6U, &frame) == YG_PROTOCOL_OK);
    assert(yg_protocol_transfer_queue_push(&rx, &frame));
    assert(yg_protocol_endpoint_process_one(&endpoint, 5U) == YG_PROTOCOL_INVALID_NODE_ID);
    assert(yg_protocol_transfer_queue_count(&tx) == 0U);
    assert(yg_protocol_endpoint_reset(&endpoint) == YG_PROTOCOL_OK);

    /* GET_INFO 的完整 48B 响应分成 46+2，缺一个空位时没有半包入队。 */
    uint8_t long_payload[48];
    memset(long_payload, 0xA6, sizeof(long_payload));
    yg_protocol_message_t long_message = {
        1U, YG_PROTOCOL_FLAGS_RESPONSE, 3U, 2U, 1U, 9U, 48U, 0U, long_payload};
    assert(yg_protocol_transfer_queue_push(&tx, &frame));
    assert(yg_protocol_endpoint_send(&endpoint, &long_message, 6U) == YG_PROTOCOL_QUEUE_FULL);
    assert(yg_protocol_transfer_queue_count(&tx) == 1U);
    assert(yg_protocol_endpoint_reset(&endpoint) == YG_PROTOCOL_OK);
    assert(yg_protocol_endpoint_send(&endpoint, &long_message, 6U) == YG_PROTOCOL_OK);
    assert(yg_protocol_transfer_queue_count(&tx) == 2U);
    yg_protocol_fragment_state_t assembly;
    uint8_t assembled[48];
    yg_protocol_message_t complete;
    assert(yg_protocol_fragment_init(&assembly, assembled, sizeof(assembled)) == YG_PROTOCOL_OK);
    for (unsigned index = 0U; index < 2U; ++index)
    {
        assert(yg_protocol_transfer_queue_pop(&tx, &frame));
        assert(yg_protocol_canfd_unpack(&frame, 2U, &response) == YG_PROTOCOL_OK);
        assert(response.sequence == index && response.payload_length == (index == 0U ? 46U : 2U));
        assert(yg_protocol_fragment_accept(&assembly, &response, index, 200U, &complete) ==
               (index == 0U ? YG_PROTOCOL_FRAGMENT_IN_PROGRESS : YG_PROTOCOL_OK));
    }
    assert(complete.payload_length == 48U && memcmp(complete.payload, long_payload, 48U) == 0);
    long_message.message_type = 124U;
    assert(yg_protocol_endpoint_send(&endpoint, &long_message, 3U) == YG_PROTOCOL_INVALID_LENGTH);
    assert(yg_protocol_transfer_queue_count(&tx) == 0U);
}

typedef struct
{
    unsigned calls;
    bool defer;
    yg_protocol_result_t status;
    yg_protocol_message_t reply;
    uint8_t payload;
} reply_test_context_t;

static yg_protocol_result_t handle_reply_test(void *context,
                                              const yg_protocol_message_t *request,
                                              yg_protocol_service_result_t *result)
{
    reply_test_context_t *test = context;
    ++test->calls;
    assert(request->payload_length == 1U);
    /* 模拟业务保存异步结果，不能保存 RX 栈内存或 endpoint 的回复缓冲。 */
    test->payload = request->payload[0];
    test->reply = *request;
    test->reply.source_node = request->destination_node;
    test->reply.destination_node = request->source_node;
    test->reply.flags = YG_PROTOCOL_FLAGS_RESPONSE;
    test->reply.payload = &test->payload;
    result->reply_ready = !test->defer;
    result->response_payload[0] = test->payload;
    result->response_payload_length = 1U;
    return test->status;
}

static void command_reply_policy(void)
{
    reply_test_context_t business = {.defer = true, .status = YG_PROTOCOL_OK};
    const yg_protocol_command_t entries[] = {
        {0x0010U, false, handle_reply_test, &business},
        {0x0011U, false, NULL, NULL},
    };
    yg_protocol_commands_t commands;
    yg_protocol_transfer_frame_t rx_storage[2], tx_storage[1], frame;
    yg_protocol_transfer_queue_t rx, tx;
    uint8_t fragments[46], payload = 0x5AU;
    yg_protocol_endpoint_t endpoint;
    const yg_protocol_endpoint_config_t config = {
        3U, 3U, 200U, &rx, &tx, &commands, fragments, sizeof(fragments)};
    yg_protocol_message_t request = {
        1U, YG_PROTOCOL_FLAGS_ACK_REQUEST, 2U, 3U, 0x0010U, 9U, 1U, 0U, &payload};
    yg_protocol_message_t response;
    assert(yg_protocol_commands_register(&commands, entries, 2U) == YG_PROTOCOL_OK);
    assert(yg_protocol_transfer_queue_init(&rx, rx_storage, 2U));
    assert(yg_protocol_transfer_queue_init(&tx, tx_storage, 1U));
    assert(yg_protocol_endpoint_init(&endpoint, &config) == YG_PROTOCOL_OK);
    assert(yg_protocol_canfd_pack(&request, 3U, &frame) == YG_PROTOCOL_OK);
    frame.data[16] ^= 1U;
    assert(yg_protocol_endpoint_process_frame(&endpoint, &frame, 0U) ==
           YG_PROTOCOL_INVALID_FRAME_CRC);
    assert(business.calls == 0U && yg_protocol_transfer_queue_count(&tx) == 0U);
    assert(yg_protocol_canfd_pack(&request, 3U, &frame) == YG_PROTOCOL_OK);
    assert(yg_protocol_endpoint_process_frame(&endpoint, &frame, 1U) == YG_PROTOCOL_OK);
    assert(business.calls == 1U && yg_protocol_transfer_queue_count(&tx) == 0U &&
           !endpoint.response_pending);
    memset(frame.data, 0, sizeof(frame.data));
    /* 完成后走相同发送入口；TX 复制结果，后续可释放业务持有的数据。 */
    assert(yg_protocol_endpoint_send(&endpoint, &business.reply, 3U) == YG_PROTOCOL_OK);
    business.payload = 0U;
    assert(yg_protocol_transfer_queue_pop(&tx, &frame));
    assert(yg_protocol_canfd_unpack(&frame, 2U, &response) == YG_PROTOCOL_OK);
    assert(response.sequence == 9U && response.payload[0] == 0x5AU);

    business.defer = false;
    assert(yg_protocol_canfd_pack(&request, 3U, &frame) == YG_PROTOCOL_OK);
    assert(yg_protocol_transfer_queue_push(&tx, &frame));
    assert(yg_protocol_endpoint_process_frame(&endpoint, &frame, 2U) == YG_PROTOCOL_QUEUE_FULL);
    assert(business.calls == 2U && endpoint.response_pending);
    for (unsigned i = 0U; i < 3U; ++i)
    {
        assert(yg_protocol_endpoint_process_frame(&endpoint, NULL, 3U) == YG_PROTOCOL_QUEUE_FULL);
        assert(business.calls == 2U);
    }
    assert(yg_protocol_transfer_queue_pop(&tx, &frame));
    assert(yg_protocol_endpoint_process_frame(&endpoint, NULL, 4U) == YG_PROTOCOL_OK);
    assert(business.calls == 2U && !endpoint.response_pending);
    assert(yg_protocol_transfer_queue_pop(&tx, &frame));

    business.status = YG_PROTOCOL_INVALID_FIELD;
    assert(yg_protocol_canfd_pack(&request, 3U, &frame) == YG_PROTOCOL_OK);
    assert(yg_protocol_endpoint_process_frame(&endpoint, &frame, 5U) == YG_PROTOCOL_INVALID_FIELD);
    assert(business.calls == 3U && yg_protocol_transfer_queue_count(&tx) == 0U);
    business.status = YG_PROTOCOL_OK;
    request.flags = YG_PROTOCOL_FLAGS_RESPONSE;
    assert(yg_protocol_canfd_pack(&request, 3U, &frame) == YG_PROTOCOL_OK);
    assert(yg_protocol_endpoint_process_frame(&endpoint, &frame, 6U) == YG_PROTOCOL_OK);
    assert(business.calls == 4U &&
           yg_protocol_transfer_queue_count(&tx) == 0U); /* 不对回复再自动回复。 */

    request.flags = YG_PROTOCOL_FLAGS_ACK_REQUEST;
    for (uint16_t type = 0x0011U; type <= 0x0012U; ++type)
    {
        request.message_type = type; /* 仅发送项和未知 ID 均不能执行。 */
        assert(yg_protocol_canfd_pack(&request, 3U, &frame) == YG_PROTOCOL_OK);
        assert(yg_protocol_endpoint_process_frame(&endpoint, &frame, 7U) ==
               YG_PROTOCOL_ROUTE_NOT_FOUND);
        assert(business.calls == 4U && yg_protocol_transfer_queue_count(&tx) == 0U);
    }
    business.reply.message_type = 0x0011U;
    assert(yg_protocol_endpoint_send(&endpoint, &business.reply, 3U) == YG_PROTOCOL_OK);
    assert(yg_protocol_transfer_queue_count(&tx) == 1U);
}

void yg_protocol_contract_test(void)
{
    service_boundaries();
    numeric_boundaries();
    full_feedback_snapshot();
    full_feedback_vector();
    motor_command_vectors();
    maintenance_kind_extension_vectors();
    endpoint_round_trip();
    command_reply_policy();
}
