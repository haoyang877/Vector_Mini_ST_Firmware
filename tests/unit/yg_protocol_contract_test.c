/* 核心接口替换性测试：无真实业务处理器、无 HAL、无硬件动作。 */
#include <assert.h>
#include <limits.h>
#include <math.h>
#include <string.h>
#include "yg_protocol_endpoint.h"
#include "yg_protocol_motor.h"
#include "yg_protocol_parameter.h"
#include "yg_protocol_job.h"
#include "yg_protocol_update.h"
#include "yg_protocol_motor_status.h"
#include "yg_protocol_readonly.h"

static void motor_handler(void *context,
                          const yg_protocol_motor_request_t *request,
                          yg_protocol_service_reply_t *reply)
{
    unsigned *calls = context;
    ++*calls;
    assert(request->operation == YG_PROTOCOL_MOTOR_SET_TARGET);
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

static void job_handler(void *context,
                        const yg_protocol_job_request_t *request,
                        yg_protocol_service_reply_t *reply)
{
    (void)context;
    assert(request->operation == YG_PROTOCOL_JOB_QUERY && request->job_id == 42U);
    reply->status = YG_PROTOCOL_SERVICE_BUSY;
}

static void update_handler(void *context,
                           const yg_protocol_update_request_t *request,
                           yg_protocol_service_reply_t *reply)
{
    (void)context;
    assert(request->operation == YG_PROTOCOL_UPDATE_WRITE);
    assert(request->data_length == 2U && request->data[0] == 5U);
    reply->status = YG_PROTOCOL_SERVICE_ACCEPTED;
    reply->token = request->image_offset;
}

static void service_boundaries(void)
{
    unsigned calls = 0U;
    yg_protocol_motor_service_t motor = {&calls, motor_handler};
    yg_protocol_parameter_service_t parameter = {NULL, parameter_handler};
    yg_protocol_job_service_t job = {NULL, job_handler};
    yg_protocol_update_service_t update = {NULL, update_handler};
    yg_protocol_motor_request_t motor_request = {.operation = YG_PROTOCOL_MOTOR_SET_TARGET,
                                                 .position_mrad = -1000};
    yg_protocol_parameter_request_t parameter_request = {.operation = YG_PROTOCOL_PARAMETER_READ};
    yg_protocol_job_request_t job_request = {.operation = YG_PROTOCOL_JOB_QUERY, .job_id = 42U};
    uint8_t data[] = {5U, 6U};
    yg_protocol_update_request_t update_request = {.operation = YG_PROTOCOL_UPDATE_WRITE,
                                                   .image_offset = 128U,
                                                   .data = data,
                                                   .data_length = sizeof(data)};
    yg_protocol_service_reply_t reply;
    assert(yg_protocol_motor_call(&motor, &motor_request, &reply) == YG_PROTOCOL_SERVICE_ACCEPTED);
    assert(calls == 1U && reply.token == 7U);
    assert(yg_protocol_parameter_call(&parameter, &parameter_request, &reply) ==
           YG_PROTOCOL_SERVICE_OK);
    assert(reply.value == 123 && reply.revision == 2U && reply.token == 0U);
    assert(yg_protocol_job_call(&job, &job_request, &reply) == YG_PROTOCOL_SERVICE_BUSY);
    assert(yg_protocol_update_call(&update, &update_request, &reply) ==
           YG_PROTOCOL_SERVICE_ACCEPTED);
    assert(reply.token == 128U);
    update_request.data = NULL;
    assert(yg_protocol_update_call(&update, &update_request, &reply) ==
           YG_PROTOCOL_SERVICE_INVALID_ARGUMENT);
    motor.handler = NULL;
    parameter.handler = NULL;
    job.handler = NULL;
    update.handler = NULL;
    update_request.data_length = 0U;
    assert(yg_protocol_motor_call(&motor, &motor_request, &reply) ==
           YG_PROTOCOL_SERVICE_UNSUPPORTED);
    assert(yg_protocol_parameter_call(&parameter, &parameter_request, &reply) ==
           YG_PROTOCOL_SERVICE_UNSUPPORTED);
    assert(yg_protocol_job_call(&job, &job_request, &reply) == YG_PROTOCOL_SERVICE_UNSUPPORTED);
    assert(yg_protocol_update_call(&update, &update_request, &reply) ==
           YG_PROTOCOL_SERVICE_UNSUPPORTED);
    assert(yg_protocol_motor_call(NULL, &motor_request, &reply) ==
           YG_PROTOCOL_SERVICE_INVALID_ARGUMENT);
}

static yg_protocol_motor_status_source_t status_source(void)
{
    yg_protocol_motor_status_source_t source = {0};
    source.sample.position_feedback = 0.0625F;
    source.sample.speed_feedback = -0.0625F;
    source.sample.current_feedback = 1.25F;
    source.sample.bus_voltage = 24.0F;
    source.sample.temperature = -25.25F;
    source.measurement_valid_bits = YG_PROTOCOL_MOTOR_MEASUREMENT_MASK;
    source.sample_available = true;
    source.state = 1U;
    return source;
}

static void numeric_boundaries(void)
{
    yg_protocol_motor_status_source_t source = status_source();
    yg_protocol_readonly_motor_state_t state;
    uint8_t feedback[8];
    static const uint8_t golden[] = {0x3F, 0, 0, 0, 0xC1, 0xFF, 0xE2, 0x04};
    assert(yg_protocol_motor_feedback_encode(&source, feedback, sizeof(feedback)) ==
           YG_PROTOCOL_OK);
    assert(memcmp(feedback, golden, sizeof(golden)) == 0);
    source.sample.position_feedback = NAN;
    source.sample.speed_feedback = INFINITY;
    source.sample.current_feedback = -INFINITY;
    source.sample.bus_voltage = 100.0F;
    source.sample.temperature = 400.0F;
    assert(yg_protocol_motor_status_convert(&source, &state) == YG_PROTOCOL_OK);
    assert(state.position_mrad == INT32_MIN && state.speed_mrad_s == INT32_MIN &&
           state.iq_mA == INT32_MIN);
    assert(state.bus_mV == UINT16_MAX && state.temperature_centi_c == INT16_MIN &&
           state.valid_bits == 0U);
    source = status_source();
    source.sample.position_feedback = 2147483.75F;
    source.sample.speed_feedback = 32.767F;
    source.sample.current_feedback = -32.768F;
    assert(yg_protocol_motor_feedback_encode(&source, feedback, sizeof(feedback)) ==
           YG_PROTOCOL_OK);
    assert(feedback[3] == 0x80U && feedback[4] == 0xFFU && feedback[5] == 0x7FU);
    assert(feedback[6] == 0U && feedback[7] == 0x80U);
    source.measurement_valid_bits = 0U;
    assert(yg_protocol_motor_status_convert(&source, &state) == YG_PROTOCOL_OK &&
           state.valid_bits == 0U);
    source.mode = 18U;
    state.boot_id = 0x12345678U;
    assert(yg_protocol_motor_status_convert(&source, &state) == YG_PROTOCOL_INVALID_FIELD);
    assert(state.boot_id == 0x12345678U);
}

static void endpoint_round_trip(void)
{
    yg_protocol_motor_status_source_t source = status_source();
    yg_protocol_readonly_service_t service = {
        .context = &source,
        .protocol_info_type = YG_PROTOCOL_READONLY_TYPE_DISABLED,
        .device_info_type = YG_PROTOCOL_READONLY_TYPE_DISABLED,
        .capabilities_type = YG_PROTOCOL_READONLY_TYPE_DISABLED,
        .motor_status_type = YG_PROTOCOL_READONLY_TYPE_GET_MOTOR_STATE,
        .motor_status = yg_protocol_motor_status_provider};
    yg_protocol_route_t route = {
        YG_PROTOCOL_READONLY_TYPE_GET_MOTOR_STATE, yg_protocol_readonly_handle, &service};
    yg_protocol_message_registry_t registry;
    yg_protocol_router_t router;
    yg_protocol_transfer_frame_t rx_storage[3], tx_storage[2], frame;
    yg_protocol_transfer_queue_t rx, tx;
    uint8_t fragments[96];
    yg_protocol_endpoint_t endpoint;
    yg_protocol_endpoint_config_t config = {
        3U, 6U, 200U, &rx, &tx, &router, fragments, sizeof(fragments)};
    uint8_t query[] = {0, 0, 0, 0, 1, 0, 0, 0};
    yg_protocol_message_t request = {
        1U, YG_PROTOCOL_FLAGS_ACK_REQUEST, 2U, 3U, 108U, 9U, 8U, 0U, query};
    yg_protocol_message_t response;
    assert(yg_protocol_readonly_registry_init(&registry) == YG_PROTOCOL_OK);
    assert(yg_protocol_router_init(&router, &registry, &route, 1U) == YG_PROTOCOL_OK);
    assert(yg_protocol_transfer_queue_init(&rx, rx_storage, 3U));
    assert(yg_protocol_transfer_queue_init(&tx, tx_storage, 2U));
    assert(yg_protocol_endpoint_init(&endpoint, &config) == YG_PROTOCOL_OK);
    assert(yg_protocol_endpoint_process_one(&endpoint, 0U) == YG_PROTOCOL_QUEUE_EMPTY);
    assert(yg_protocol_canfd_pack(&request, 6U, &frame) == YG_PROTOCOL_OK);
    assert(yg_protocol_transfer_queue_push(&rx, &frame));
    assert(yg_protocol_transfer_queue_push(&tx, &frame));
    assert(yg_protocol_transfer_queue_push(&tx, &frame));
    assert(yg_protocol_endpoint_process_one(&endpoint, 1U) == YG_PROTOCOL_QUEUE_FULL);
    source.sample_available = false;
    assert(yg_protocol_transfer_queue_pop(&tx, &frame));
    assert(yg_protocol_endpoint_process_one(&endpoint, 2U) == YG_PROTOCOL_OK);
    assert(yg_protocol_transfer_queue_pop(&tx, &frame));
    assert(yg_protocol_transfer_queue_pop(&tx, &frame));
    assert(frame.length == 64U && frame.identifier == 0x18EF0203U);
    assert(yg_protocol_canfd_unpack(&frame, 2U, &response) == YG_PROTOCOL_OK);
    assert(response.flags == YG_PROTOCOL_FLAGS_RESPONSE && response.sequence == 9U);
    assert(response.payload_length == 44U && response.payload[8] == 0U);
    assert(response.payload[20] == 63U && response.payload[43] == 0x37U);
    /* TX 满后仍发原先的 OK 快照，没有重新调用 provider 变成 BUSY。 */
    assert(yg_protocol_canfd_pack(&request, 6U, &frame) == YG_PROTOCOL_OK);
    assert(yg_protocol_transfer_queue_push(&rx, &frame));
    assert(yg_protocol_endpoint_process_one(&endpoint, 3U) == YG_PROTOCOL_OK);
    assert(yg_protocol_transfer_queue_pop(&tx, &frame));
    assert(yg_protocol_canfd_unpack(&frame, 2U, &response) == YG_PROTOCOL_OK);
    assert(response.payload_length == 12U && response.payload[8] == 6U);
    request.flags = YG_PROTOCOL_FLAGS_RESPONSE;
    assert(yg_protocol_canfd_pack(&request, 6U, &frame) == YG_PROTOCOL_OK);
    assert(yg_protocol_transfer_queue_push(&rx, &frame));
    assert(yg_protocol_endpoint_process_one(&endpoint, 4U) == YG_PROTOCOL_INVALID_FIELD);
    assert(tx.count == 0U);
    request.flags = YG_PROTOCOL_FLAGS_ACK_REQUEST;
    request.destination_node = 0xFFU;
    assert(yg_protocol_canfd_pack(&request, 6U, &frame) == YG_PROTOCOL_OK);
    assert(yg_protocol_transfer_queue_push(&rx, &frame));
    assert(yg_protocol_endpoint_process_one(&endpoint, 5U) == YG_PROTOCOL_INVALID_NODE_ID);
    assert(tx.count == 0U);
    assert(yg_protocol_endpoint_reset(&endpoint) == YG_PROTOCOL_OK);

    /* GET_INFO 的完整 48B 响应分成 46+2，缺一个空位时没有半包入队。 */
    uint8_t long_payload[48];
    memset(long_payload, 0xA6, sizeof(long_payload));
    yg_protocol_message_t long_message = {
        1U, YG_PROTOCOL_FLAGS_RESPONSE, 3U, 2U, 1U, 9U, 48U, 0U, long_payload};
    assert(yg_protocol_transfer_queue_push(&tx, &frame));
    assert(yg_protocol_endpoint_send(&endpoint, &long_message, 6U) == YG_PROTOCOL_QUEUE_FULL);
    assert(tx.count == 1U);
    assert(yg_protocol_endpoint_reset(&endpoint) == YG_PROTOCOL_OK);
    assert(yg_protocol_endpoint_send(&endpoint, &long_message, 6U) == YG_PROTOCOL_OK);
    assert(tx.count == 2U);
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
    assert(tx.count == 0U);
}

void yg_protocol_contract_test(void)
{
    service_boundaries();
    numeric_boundaries();
    endpoint_round_trip();
}
