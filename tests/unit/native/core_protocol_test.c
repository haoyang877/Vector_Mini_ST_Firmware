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
static bool stopped = true;
static unsigned stop_requests;
static uint32_t target_epoch = 1U;
static bool stop_after_target;
static MotorTarget last_target;
static MotorControlMode reported_mode = MOTOR_CONTROL_NONE;
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

void MotorLifecycle_RequestStop(MotorLifecycle *instance)
{
    assert(instance == &life);
    ++stop_requests;
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
        .state = MOTOR_STATE_IDLE, .mode = reported_mode, .fault = MOTOR_FAULT_NONE};
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

static void test_target_mode_mapping(void)
{
    static const struct
    {
        uint8_t wire;
        MotorControlMode mode;
    } modes[] = {{1, MOTOR_CONTROL_CURRENT},
                 {2, MOTOR_CONTROL_SPEED},
                 {3, MOTOR_CONTROL_POSITION},
                 {6, MOTOR_CONTROL_VQ},
                 {9, MOTOR_CONTROL_OPEN_VOLTAGE},
                 {10, MOTOR_CONTROL_OPEN_CURRENT}};
    ProtocolReply reply;
    for (unsigned i = 0; i < sizeof(modes) / sizeof(modes[0]); ++i)
    {
        ProtocolCommand request = {.mode = modes[i].wire, .sequence = (uint16_t)(i + 1)};
        reported_mode = modes[i].mode;
        endpoint.motor.target(endpoint.motor.context, &request, &reply);
        assert(reply.status == PROTOCOL_OK && last_target.mode == modes[i].mode);
        ProtocolFeedback feedback = s_empty_feedback;
        assert(endpoint.motor.read_feedback(endpoint.motor.context, &feedback));
        assert(feedback.control_mode == modes[i].wire);
    }
    unsigned calls_before = target_calls;
    ProtocolCommand request = {.mode = 0U, .sequence = 7U};
    endpoint.motor.target(endpoint.motor.context, &request, &reply);
    assert(reply.status == PROTOCOL_UNSUPPORTED && target_calls == calls_before);
    request.mode = 1U;
    request.sequence = (uint16_t)(adapter.last_target.sequence + 0x8000U);
    endpoint.motor.target(endpoint.motor.context, &request, &reply);
    assert(reply.status == PROTOCOL_DENIED && target_calls == calls_before);
    --request.sequence;
    endpoint.motor.target(endpoint.motor.context, &request, &reply);
    assert(reply.status == PROTOCOL_OK && target_calls == calls_before + 1U);
    /* 目标已写入，返回前被停机抢占；同序号重发不可复用已结束的会话。 */
    ++request.sequence;
    stop_after_target = true;
    endpoint.motor.target(endpoint.motor.context, &request, &reply);
    assert(reply.status == PROTOCOL_OK && target_calls == calls_before + 2U);
    endpoint.motor.target(endpoint.motor.context, &request, &reply);
    assert(reply.status == PROTOCOL_DENIED && target_calls == calls_before + 2U);
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
    assert(reply.status == PROTOCOL_ACCEPTED && reply.detail == 1U && stop_requests == 1U);
    stopped = true;
    endpoint.motor.stop(endpoint.motor.context, &request, &reply);
    assert(reply.status == PROTOCOL_OK && reply.detail == 0U && stop_requests == 2U);
    /* 未实现的线路模式仍由 APP 拒绝；无需模拟运行时撤销固定绑定。 */
    request.mode = 4U;
    endpoint.motor.target(endpoint.motor.context, &request, &reply);
    assert(reply.status == PROTOCOL_UNSUPPORTED);
    ProtocolResponse encoded_reply = {0};
    Protocol_WriteResult(&encoded_reply, Protocol_StatusToCode(reply.status), reply.detail);
    print_vector("target_unsupported", encoded_reply.data, encoded_reply.length);
    endpoint.motor.clear_fault(endpoint.motor.context, &request, &reply);
    assert(reply.status == PROTOCOL_OK);
    test_target_mode_mapping();
    puts("PASS core protocol: three calibration kinds, wire guards, result mapping, START and "
         "unsupported operations");
    return 0;
}

MotorControlResult MotorLifecycle_SetTarget(MotorLifecycle *instance, const MotorTarget *target)
{
    (void)instance;
    ++target_calls;
    last_target = *target;
    if (stop_after_target)
    {
        ++target_epoch;
        stop_after_target = false;
    }
    return MOTOR_CONTROL_ACCEPTED;
}
uint32_t MotorLifecycle_WorkEpoch(const MotorLifecycle *instance)
{
    (void)instance;
    return target_epoch;
}
