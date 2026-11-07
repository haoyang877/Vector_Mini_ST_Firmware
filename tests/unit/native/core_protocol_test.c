/* 真实APP适配：控制和三类标定返回受理；保存及通用任务操作不支持。 */
#include <assert.h>
#include <stdio.h>
#include "motor_lifecycle.h"
#include "app_protocol.h"
#include "../../../firmware/communication/yg_protocol.c"
#include "../../../firmware/communication/yg_protocol_boot.c"
#include "motor_axis.h"

static MotorLifecycle life;
static ProtocolEndpoint endpoint;
static AppProtocol adapter;
static unsigned starts, target_calls;
static bool stop_allowed = true, stopped = true;
static uint32_t target_epoch = 1U;
static MotorTarget last_target;
static unsigned calibration_requests;
static MotorControlResult start_result = MOTOR_CONTROL_ACCEPTED;
static MotorControlResult calibration_result = MOTOR_CONTROL_ACCEPTED;
static MotorCalibrationType requested_calibration_type;
static void print_vector(const char *name, const uint8_t *data, size_t length)
{
    printf("VECTOR %s ", name);
    for (size_t i = 0U; i < length; ++i)
    {
        printf("%02x", data[i]);
    }
    puts("");
}

bool MotorLifecycle_RequestRemoteStop(MotorLifecycle *instance)
{
    (void)instance;
    return stop_allowed;
}
bool MotorLifecycle_ArePhasesDisabled(const MotorLifecycle *life)
{
    (void)life;
    return stopped;
}
MotorStatus MotorLifecycle_ReadStatus(const MotorLifecycle *instance)
{
    (void)instance;
    return (MotorStatus){
        .state = MOTOR_STATE_IDLE, .mode = MOTOR_CONTROL_NONE, .fault = MOTOR_FAULT_NONE};
}

MotorControlResult MotorLifecycle_RequestCalibration(MotorLifecycle *instance,
                                                     MotorCalibrationType calibration_type)
{
    (void)instance;
    ++calibration_requests;
    requested_calibration_type = calibration_type;
    return calibration_result;
}

MotorControlResult MotorLifecycle_RequestEnable(MotorLifecycle *instance, MotorControlMode mode)
{
    (void)instance;
    assert(mode == MOTOR_CONTROL_SPEED);
    ++starts;
    return start_result;
}
MotorControlResult MotorLifecycle_RequestClear(MotorLifecycle *instance)
{
    (void)instance;
    return MOTOR_CONTROL_ACCEPTED;
}

static ProtocolResponse send_calibration_command(uint16_t message_type,
                                                 uint32_t request_value,
                                                 uint8_t source_node,
                                                 uint32_t payload_length)
{
    uint8_t request_payload[4];
    YgProtocol_WriteU32Le(request_payload, request_value);
    ProtocolExchange exchange = {.request = {.version = YG_PROTOCOL_VERSION,
                                             .flags = YG_PROTOCOL_FLAGS_ACK_REQUEST,
                                             .source_node = source_node,
                                             .destination_node = 1U,
                                             .message_type = message_type,
                                             .sequence = 1U,
                                             .payload_length = payload_length,
                                             .payload = request_payload}};
    assert(handle_command(&endpoint, &exchange) == YG_PROTOCOL_OK);
    assert(exchange.reply.length == PROTOCOL_JOB_REPLY_SIZE);
    return exchange.reply;
}

static void test_calibration_protocol(void)
{
    const uint16_t kinds[] = {PROTOCOL_JOB_KIND_ENCODER_LUT,
                              PROTOCOL_JOB_KIND_ELECTRICAL_ZERO,
                              PROTOCOL_JOB_KIND_MECHANICAL_ZERO};
    const MotorCalibrationType types[] = {MOTOR_CALIBRATION_ENCODER_LINEARIZATION,
                                          MOTOR_CALIBRATION_ELECTRICAL_ZERO,
                                          MOTOR_CALIBRATION_MECHANICAL_ZERO};
    const char *names[] = {
        "encoder_linearization_accepted", "electrical_zero_accepted", "mechanical_zero_accepted"};
    for (unsigned index = 0U; index < 3U; ++index)
    {
        ProtocolResponse response =
            send_calibration_command(PROTOCOL_START_JOB, kinds[index], PROTOCOL_MASTER_ID, 4U);
        const uint8_t *payload = response.data;
        assert(requested_calibration_type == types[index]);
        assert(calibration_requests == index + 1U);
        assert(YgProtocol_ReadU16Le(payload) == 1U);
        assert(YgProtocol_ReadU32Le(payload + 4U) == index + 1U);
        assert(YgProtocol_ReadU16Le(payload + 8U) == kinds[index]);
        assert(payload[10] == PROTOCOL_JOB_PHASE_IN_PROGRESS);
        assert(payload[11] == PROTOCOL_JOB_EFFECT_UNCOMMITTED);
        print_vector(names[index], payload, response.length);
    }

    const uint16_t unsupported_kinds[] = {3U, 4U, 5U, 7U, 8U, 99U};
    for (unsigned index = 0U; index < sizeof(unsupported_kinds) / sizeof(unsupported_kinds[0]);
         ++index)
    {
        ProtocolResponse response = send_calibration_command(
            PROTOCOL_START_JOB, unsupported_kinds[index], PROTOCOL_MASTER_ID, 4U);
        assert(YgProtocol_ReadU16Le(response.data) == 2U);
        assert(YgProtocol_ReadU16Le(response.data + 2U) == PROTOCOL_JOB_DETAIL_UNSUPPORTED_KIND);
    }
    assert(calibration_requests == 3U);

    const uint16_t unsupported_operations[] = {PROTOCOL_GET_JOB_STATUS, PROTOCOL_CANCEL_JOB};
    for (unsigned index = 0U; index < 2U; ++index)
    {
        ProtocolResponse response =
            send_calibration_command(unsupported_operations[index], 1U, PROTOCOL_MASTER_ID, 4U);
        assert(YgProtocol_ReadU16Le(response.data) == 2U);
        print_vector(index == 0U ? "calibration_query_unsupported"
                                 : "calibration_cancel_unsupported",
                     response.data,
                     response.length);
    }

    ProtocolResponse response = send_calibration_command(PROTOCOL_START_JOB, 1U, 4U, 4U);
    assert(YgProtocol_ReadU16Le(response.data) == 7U);
    response = send_calibration_command(PROTOCOL_START_JOB, 1U, PROTOCOL_MASTER_ID, 3U);
    assert(YgProtocol_ReadU16Le(response.data) == 3U);
    response = send_calibration_command(PROTOCOL_START_JOB, 0x10001U, PROTOCOL_MASTER_ID, 4U);
    assert(YgProtocol_ReadU16Le(response.data) == 4U);
    response = send_calibration_command(PROTOCOL_START_JOB, 0U, PROTOCOL_MASTER_ID, 4U);
    assert(YgProtocol_ReadU16Le(response.data) == 4U);
    assert(calibration_requests == 3U);

    const MotorControlResult rejected[] = {MOTOR_CONTROL_BUSY,
                                           MOTOR_CONTROL_DENIED,
                                           MOTOR_CONTROL_FAULT,
                                           MOTOR_CONTROL_INVALID,
                                           MOTOR_CONTROL_OUT_OF_RANGE};
    const uint16_t result_codes[] = {6U, 5U, 17U, 4U, 11U};
    const char *rejected_names[] = {"calibration_busy",
                                    "calibration_denied",
                                    "calibration_fault",
                                    "calibration_invalid",
                                    "calibration_out_of_range"};
    for (unsigned index = 0U; index < 5U; ++index)
    {
        calibration_result = rejected[index];
        response = send_calibration_command(PROTOCOL_START_JOB, 1U, PROTOCOL_MASTER_ID, 4U);
        assert(YgProtocol_ReadU16Le(response.data) == result_codes[index]);
        assert(YgProtocol_ReadU32Le(response.data + 4U) == 0U);
        print_vector(rejected_names[index], response.data, response.length);
    }
    calibration_result = MOTOR_CONTROL_ACCEPTED;
    response = send_calibration_command(PROTOCOL_START_JOB, 1U, PROTOCOL_MASTER_ID, 4U);
    assert(YgProtocol_ReadU32Le(response.data + 4U) == 4U);
}

int main(void)
{
    AppProtocol_Init(&adapter, &endpoint, &life);
    assert(Protocol_Init(&endpoint, 1U));
    ProtocolReply reply;
    const uint16_t parameter_types[] = {
        PROTOCOL_READ_PARAMETER, PROTOCOL_WRITE_PARAMETER, PROTOCOL_SAVE_PARAMETER};
    const char *parameter_names[] = {
        "parameter_read_unsupported", "parameter_write_unsupported", "save_unsupported"};
    for (unsigned i = 0U; i < 3U; ++i)
    {
        uint8_t bytes[12] = {1U, 1U, 4U, 0U};
        if (i != 1U)
        {
            bytes[1] = bytes[2] = 0U;
        }
        ProtocolExchange exchange = {.request = {.version = YG_PROTOCOL_VERSION,
                                                 .flags = YG_PROTOCOL_FLAGS_ACK_REQUEST,
                                                 .source_node = PROTOCOL_MASTER_ID,
                                                 .destination_node = 1U,
                                                 .message_type = parameter_types[i],
                                                 .payload = bytes,
                                                 .payload_length = i == 1U ? 12U : 4U}};
        assert(reject_parameter_command(&endpoint, &exchange) == YG_PROTOCOL_OK);
        print_vector(parameter_names[i], exchange.reply.data, exchange.reply.length);
    }
    ProtocolResponse job_response =
        send_calibration_command(PROTOCOL_GET_JOB_STATUS, 42U, PROTOCOL_MASTER_ID, 4U);
    print_vector("unbound_job_unsupported", job_response.data, job_response.length);
    test_calibration_protocol();
    ProtocolCommand request = {.mode = 2U, .source_node = 4U, .sequence = 1U};
    endpoint.motor.start(endpoint.motor.context, &request, &reply);
    assert(reply.status == PROTOCOL_ACCEPTED && starts == 1U);
    endpoint.motor.start(endpoint.motor.context, &request, &reply);
    assert(reply.status == PROTOCOL_ACCEPTED && starts == 2U);
    start_result = MOTOR_CONTROL_BUSY;
    endpoint.motor.start(endpoint.motor.context, &request, &reply);
    assert(reply.status == PROTOCOL_BUSY);
    request.mode = 1U;
    request.iq_mA = 250;
    request.sequence = UINT16_MAX;
    endpoint.motor.target(endpoint.motor.context, &request, &reply);
    assert(reply.status == PROTOCOL_OK && target_calls == 1U && last_target.iq_a == 0.25f);
    endpoint.motor.target(endpoint.motor.context, &request, &reply);
    assert(reply.status == PROTOCOL_OK && target_calls == 1U);
    request.iq_mA = 500;
    endpoint.motor.target(endpoint.motor.context, &request, &reply);
    assert(reply.status == PROTOCOL_INVALID_ARGUMENT && target_calls == 1U);
    request.sequence = UINT16_MAX - 1U;
    endpoint.motor.target(endpoint.motor.context, &request, &reply);
    assert(reply.status == PROTOCOL_DENIED && target_calls == 1U);
    request.sequence = 0U;
    endpoint.motor.target(endpoint.motor.context, &request, &reply);
    assert(reply.status == PROTOCOL_OK && target_calls == 2U);
    ++target_epoch;
    endpoint.motor.target(endpoint.motor.context, &request, &reply);
    assert(reply.status == PROTOCOL_DENIED && target_calls == 2U);
    stopped = false;
    endpoint.motor.stop(endpoint.motor.context, &request, &reply);
    assert(reply.status == PROTOCOL_ACCEPTED && reply.token != 0U);
    uint32_t stop_token = reply.token;
    stopped = true;
    endpoint.motor.stop(endpoint.motor.context, &request, &reply);
    assert(reply.status == PROTOCOL_OK && reply.token == stop_token);
    stop_allowed = false;
    endpoint.motor.stop(endpoint.motor.context, &request, &reply);
    assert(reply.status == PROTOCOL_DENIED && reply.token == 0U);
    stop_allowed = true;
    /* 未实现的线路模式仍由 APP 拒绝；无需模拟运行时撤销固定绑定。 */
    request.mode = 4U;
    endpoint.motor.target(endpoint.motor.context, &request, &reply);
    assert(reply.status == PROTOCOL_UNSUPPORTED);
    ProtocolResponse encoded_reply = {0};
    Protocol_WriteResult(&encoded_reply, Protocol_StatusToCode(reply.status), reply.detail);
    print_vector("target_unsupported", encoded_reply.data, encoded_reply.length);
    endpoint.motor.clear_fault(endpoint.motor.context, &request, &reply);
    assert(reply.status == PROTOCOL_OK);
    puts("PASS core protocol: three calibration kinds, wire guards, result mapping, START and "
         "unsupported operations");
    return 0;
}

MotorControlResult MotorLifecycle_SetTarget(MotorLifecycle *instance, const MotorTarget *target)
{
    (void)instance;
    ++target_calls;
    last_target = *target;
    return MOTOR_CONTROL_ACCEPTED;
}
uint32_t MotorLifecycle_WorkEpoch(const MotorLifecycle *instance)
{
    (void)instance;
    return target_epoch;
}
