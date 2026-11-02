/* 真实APP适配：控制和三类标定返回受理；保存及通用任务操作不支持。 */
#include <assert.h>
#include <stdio.h>
#include "motor_lifecycle.h"
#include "app_protocol.h"
#include "yg_protocol_motor_adapter.h"
#include "yg_protocol_link.h"
#include "motor_axis.h"

static yg_protocol_parameter_service_t parameters;
static yg_protocol_job_service_t calibrations;
static unsigned starts, target_calls;
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
static unsigned binding_step, failing_binding;
static bool record_binding(unsigned step)
{
    assert(++binding_step == step);
    return step != failing_binding;
}
bool YgProtocolLink_BindMotorStatusSource(const yg_protocol_motor_status_source_t *source)
{
    return source != NULL && record_binding(1U);
}
bool YgProtocolLink_BindDeviceStatusSource(const yg_protocol_readonly_device_status_t *source)
{
    return source != NULL && record_binding(2U);
}
static yg_protocol_motor_service_t motor_service;
bool YgProtocolLink_BindMotorService(const yg_protocol_motor_service_t *service)
{
    motor_service = *service;
    return record_binding(3U);
}
static void request_lifecycle(void *context,
                              const yg_protocol_motor_request_t *request,
                              yg_protocol_service_reply_t *reply)
{
    (void)context;
    motor_service.handler(motor_service.context, request, reply);
}
bool MotorLifecycle_RequestRemoteStop(void)
{
    return true;
}
bool BspPwm_ArePhasesDisabled(void)
{
    return true;
}
MotorStatus MotorLifecycle_ReadStatus(void)
{
    return (MotorStatus){
        .state = MOTOR_STATE_IDLE, .mode = MOTOR_CONTROL_NONE, .fault = MOTOR_FAULT_NONE};
}

bool YgProtocolLink_BindParameterService(const yg_protocol_parameter_service_t *service)
{
    parameters = *service;
    return true;
}
bool YgProtocolLink_BindJobService(const yg_protocol_job_service_t *service)
{
    assert(service != NULL && service->handler != NULL);
    calibrations = *service;
    return record_binding(4U);
}

MotorControlResult MotorLifecycle_RequestCalibration(MotorCalibrationType calibration_type)
{
    ++calibration_requests;
    requested_calibration_type = calibration_type;
    return calibration_result;
}

MotorControlResult MotorLifecycle_RequestEnable(MotorControlMode mode)
{
    assert(mode == MOTOR_CONTROL_SPEED);
    ++starts;
    return start_result;
}
MotorControlResult MotorLifecycle_RequestClear(void)
{
    return MOTOR_CONTROL_ACCEPTED;
}

static yg_protocol_service_result_t send_calibration_command(uint16_t message_type,
                                                             uint32_t request_value,
                                                             uint8_t source_node,
                                                             uint32_t payload_length)
{
    uint8_t request_payload[4];
    YgProtocol_WriteU32Le(request_payload, request_value);
    yg_protocol_message_t request = {.version = YG_PROTOCOL_VERSION,
                                     .flags = YG_PROTOCOL_FLAGS_ACK_REQUEST,
                                     .source_node = source_node,
                                     .destination_node = 1U,
                                     .message_type = message_type,
                                     .sequence = 1U,
                                     .payload_length = payload_length,
                                     .payload = request_payload};
    yg_protocol_service_result_t response = {0};
    assert(yg_protocol_job_handle(&calibrations, &request, &response) == YG_PROTOCOL_OK);
    assert(response.response_payload_length == YG_PROTOCOL_JOB_REPLY_SIZE);
    return response;
}

static void test_calibration_protocol(void)
{
    const uint16_t kinds[] = {YG_PROTOCOL_JOB_KIND_ENCODER_LUT,
                              YG_PROTOCOL_JOB_KIND_ELECTRICAL_ZERO,
                              YG_PROTOCOL_JOB_KIND_MECHANICAL_ZERO};
    const MotorCalibrationType types[] = {MOTOR_CALIBRATION_ENCODER_LINEARIZATION,
                                          MOTOR_CALIBRATION_ELECTRICAL_ZERO,
                                          MOTOR_CALIBRATION_MECHANICAL_ZERO};
    const char *names[] = {
        "encoder_linearization_accepted", "electrical_zero_accepted", "mechanical_zero_accepted"};
    for (unsigned index = 0U; index < 3U; ++index)
    {
        yg_protocol_service_result_t response = send_calibration_command(
            YG_PROTOCOL_JOB_TYPE_START, kinds[index], YG_PROTOCOL_MASTER_NODE_ID, 4U);
        const uint8_t *payload = response.response_payload;
        assert(requested_calibration_type == types[index]);
        assert(calibration_requests == index + 1U);
        assert(YgProtocol_ReadU16Le(payload) == 1U);
        assert(YgProtocol_ReadU32Le(payload + 4U) == index + 1U);
        assert(YgProtocol_ReadU16Le(payload + 8U) == kinds[index]);
        assert(payload[10] == YG_PROTOCOL_JOB_PHASE_IN_PROGRESS);
        assert(payload[11] == YG_PROTOCOL_JOB_EFFECT_UNCOMMITTED);
        print_vector(names[index], payload, response.response_payload_length);
    }

    const uint16_t unsupported_kinds[] = {3U, 4U, 5U, 7U, 8U, 99U};
    for (unsigned index = 0U; index < sizeof(unsupported_kinds) / sizeof(unsupported_kinds[0]);
         ++index)
    {
        yg_protocol_service_result_t response = send_calibration_command(
            YG_PROTOCOL_JOB_TYPE_START, unsupported_kinds[index], YG_PROTOCOL_MASTER_NODE_ID, 4U);
        assert(YgProtocol_ReadU16Le(response.response_payload) == 2U);
        assert(YgProtocol_ReadU16Le(response.response_payload + 2U) ==
               YG_PROTOCOL_JOB_DETAIL_UNSUPPORTED_KIND);
    }
    assert(calibration_requests == 3U);

    const uint16_t unsupported_operations[] = {YG_PROTOCOL_JOB_TYPE_STATUS,
                                               YG_PROTOCOL_JOB_TYPE_CANCEL};
    for (unsigned index = 0U; index < 2U; ++index)
    {
        yg_protocol_service_result_t response = send_calibration_command(
            unsupported_operations[index], 1U, YG_PROTOCOL_MASTER_NODE_ID, 4U);
        assert(YgProtocol_ReadU16Le(response.response_payload) == 2U);
        print_vector(index == 0U ? "calibration_query_unsupported"
                                 : "calibration_cancel_unsupported",
                     response.response_payload,
                     response.response_payload_length);
    }

    yg_protocol_service_result_t response =
        send_calibration_command(YG_PROTOCOL_JOB_TYPE_START, 1U, 4U, 4U);
    assert(YgProtocol_ReadU16Le(response.response_payload) == 7U);
    response =
        send_calibration_command(YG_PROTOCOL_JOB_TYPE_START, 1U, YG_PROTOCOL_MASTER_NODE_ID, 3U);
    assert(YgProtocol_ReadU16Le(response.response_payload) == 3U);
    response = send_calibration_command(
        YG_PROTOCOL_JOB_TYPE_START, 0x10001U, YG_PROTOCOL_MASTER_NODE_ID, 4U);
    assert(YgProtocol_ReadU16Le(response.response_payload) == 4U);
    response =
        send_calibration_command(YG_PROTOCOL_JOB_TYPE_START, 0U, YG_PROTOCOL_MASTER_NODE_ID, 4U);
    assert(YgProtocol_ReadU16Le(response.response_payload) == 4U);
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
        response = send_calibration_command(
            YG_PROTOCOL_JOB_TYPE_START, 1U, YG_PROTOCOL_MASTER_NODE_ID, 4U);
        assert(YgProtocol_ReadU16Le(response.response_payload) == result_codes[index]);
        assert(YgProtocol_ReadU32Le(response.response_payload + 4U) == 0U);
        print_vector(
            rejected_names[index], response.response_payload, response.response_payload_length);
    }
    calibration_result = MOTOR_CONTROL_ACCEPTED;
    response =
        send_calibration_command(YG_PROTOCOL_JOB_TYPE_START, 1U, YG_PROTOCOL_MASTER_NODE_ID, 4U);
    assert(YgProtocol_ReadU32Le(response.response_payload + 4U) == 4U);
}

int main(void)
{
    for (failing_binding = 1U; failing_binding <= 4U; ++failing_binding)
    {
        binding_step = 0U;
        assert(!AppProtocol_Init() && binding_step == failing_binding);
    }
    failing_binding = binding_step = 0U;
    assert(AppProtocol_Init() && binding_step == 4U);
    yg_protocol_parameter_request_t save = {.operation = YG_PROTOCOL_PARAMETER_SAVE};
    yg_protocol_service_reply_t reply;
    assert(yg_protocol_parameter_call(&parameters, &save, &reply) ==
           YG_PROTOCOL_SERVICE_UNSUPPORTED);
    assert(reply.status == YG_PROTOCOL_SERVICE_UNSUPPORTED);
    for (unsigned operation = YG_PROTOCOL_PARAMETER_READ; operation <= YG_PROTOCOL_PARAMETER_WRITE;
         ++operation)
    {
        yg_protocol_parameter_request_t request = {.operation = operation, .parameter_id = 1U};
        assert(yg_protocol_parameter_call(&parameters, &request, &reply) ==
               YG_PROTOCOL_SERVICE_UNSUPPORTED);
        uint8_t encoded[4];
        size_t written;
        assert(yg_protocol_parameter_encode_reply(
                   &request, &reply, encoded, sizeof(encoded), &written) == YG_PROTOCOL_OK);
        print_vector(operation == YG_PROTOCOL_PARAMETER_READ ? "parameter_read_unsupported"
                                                             : "parameter_write_unsupported",
                     encoded,
                     written);
    }
    uint8_t payload[16];
    size_t length;
    assert(yg_protocol_parameter_encode_reply(&save, &reply, payload, sizeof(payload), &length) ==
           YG_PROTOCOL_OK);
    print_vector("save_unsupported", payload, length);
    yg_protocol_job_service_t jobs = {0};
    yg_protocol_job_request_t job = {.operation = YG_PROTOCOL_JOB_START,
                                     .kind = YG_PROTOCOL_JOB_KIND_ENCODER_LUT};
    yg_protocol_job_reply_t job_reply;
    assert(yg_protocol_job_call(&jobs, &job, &job_reply) == YG_PROTOCOL_SERVICE_UNSUPPORTED);
    assert(yg_protocol_job_encode_reply(&job_reply, payload, sizeof(payload), &length) ==
           YG_PROTOCOL_OK);
    print_vector("unbound_job_unsupported", payload, length);
    test_calibration_protocol();
    yg_protocol_motor_request_t request = {.operation = YG_PROTOCOL_MOTOR_START_CONTROL,
                                           .mode = 2U,
                                           .source_node = 4U,
                                           .sequence = 1U};
    request_lifecycle(NULL, &request, &reply);
    assert(reply.status == YG_PROTOCOL_SERVICE_ACCEPTED && starts == 1U);
    request_lifecycle(NULL, &request, &reply);
    assert(reply.status == YG_PROTOCOL_SERVICE_ACCEPTED && starts == 2U);
    start_result = MOTOR_CONTROL_BUSY;
    request_lifecycle(NULL, &request, &reply);
    assert(reply.status == YG_PROTOCOL_SERVICE_BUSY);
    request.operation = YG_PROTOCOL_MOTOR_SET_CONTROL;
    request.mode = 1U;
    request.iq_mA = 250;
    request.sequence = UINT16_MAX;
    request_lifecycle(NULL, &request, &reply);
    assert(reply.status == YG_PROTOCOL_SERVICE_OK && target_calls == 1U &&
           last_target.iq_a == 0.25f);
    request_lifecycle(NULL, &request, &reply);
    assert(reply.status == YG_PROTOCOL_SERVICE_OK && target_calls == 1U);
    request.iq_mA = 500;
    request_lifecycle(NULL, &request, &reply);
    assert(reply.status == YG_PROTOCOL_SERVICE_INVALID_ARGUMENT && target_calls == 1U);
    request.sequence = UINT16_MAX - 1U;
    request_lifecycle(NULL, &request, &reply);
    assert(reply.status == YG_PROTOCOL_SERVICE_DENIED && target_calls == 1U);
    request.sequence = 0U;
    request_lifecycle(NULL, &request, &reply);
    assert(reply.status == YG_PROTOCOL_SERVICE_OK && target_calls == 2U);
    ++target_epoch;
    request_lifecycle(NULL, &request, &reply);
    assert(reply.status == YG_PROTOCOL_SERVICE_DENIED && target_calls == 2U);
    yg_protocol_motor_adapter_t adapter = {0};
    request.operation = YG_PROTOCOL_MOTOR_SET_CONTROL;
    yg_protocol_motor_adapter_handle(&adapter, &request, &reply);
    assert(reply.status == YG_PROTOCOL_SERVICE_UNSUPPORTED);
    assert(yg_protocol_motor_encode_reply(&reply, payload, sizeof(payload), &length) ==
           YG_PROTOCOL_OK);
    print_vector("target_unsupported", payload, length);
    request.operation = YG_PROTOCOL_MOTOR_CLEAR_FAULT;
    request_lifecycle(NULL, &request, &reply);
    assert(reply.status == YG_PROTOCOL_SERVICE_OK);
    puts("PASS core protocol: three calibration kinds, wire guards, result mapping, START and "
         "unsupported operations");
    return 0;
}

MotorControlResult MotorLifecycle_SetTarget(const MotorTarget *target)
{
    ++target_calls;
    last_target = *target;
    return MOTOR_CONTROL_ACCEPTED;
}
uint32_t MotorLifecycle_WorkEpoch(void)
{
    return target_epoch;
}
