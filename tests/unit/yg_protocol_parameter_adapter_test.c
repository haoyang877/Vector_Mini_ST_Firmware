/* 参数只读适配器测试：纯 C 逻辑，无 HAL、无 Flash、无硬件动作。
 * 覆盖单位换算、未知 ID、未就绪、非法数值、写操作拒绝和输入不变。 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "parameter_read_service.h"
#include "yg_protocol_parameter.h"
#include "yg_protocol_parameter_adapter.h"

static ParameterReadSource make_source(void)
{
    ParameterReadSource source = {0};

    source.values.speed_limit = 20.0f;
    source.values.current_limit = 1.25f;
    source.config_revision = 7U;
    source.snapshot_ready = true;
    return source;
}

static void read_units_are_explicit(void)
{
    ParameterReadSource source = make_source();
    ParameterReadResult result;

    memset(&result, 0x5A, sizeof(result));
    assert(ParameterRead_Get(&source, PARAMETER_READ_ID_MAX_SPEED_RAD_S, &result) ==
           PARAMETER_READ_OK);
    assert(result.value_type == PARAMETER_READ_VALUE_TYPE_I32);
    assert(result.value == 20000);
    assert(ParameterRead_Get(&source, PARAMETER_READ_ID_MAX_CURRENT_A, &result) ==
           PARAMETER_READ_OK);
    assert(result.value_type == PARAMETER_READ_VALUE_TYPE_I32);
    assert(result.value == 1250);
    assert(ParameterRead_Get(&source, PARAMETER_READ_ID_ACTIVE_REVISION, &result) ==
           PARAMETER_READ_OK);
    assert(result.value_type == PARAMETER_READ_VALUE_TYPE_U32 && result.value == 7);
    puts("PASS READ milli-rad/s, mA and owner-provided revision units");
}

static void unknown_and_not_ready_are_distinct(void)
{
    ParameterReadSource source = make_source();
    ParameterReadResult result;

    memset(&result, 0x5A, sizeof(result));
    assert(ParameterRead_Get(&source, 3U, &result) == PARAMETER_READ_UNSUPPORTED);
    assert(result.value_type == 0U && result.value == 0);
    memset(&result, 0x5A, sizeof(result));
    assert(ParameterRead_Get(&source, 0xFFFFU, &result) == PARAMETER_READ_UNSUPPORTED);
    assert(result.value_type == 0U && result.value == 0);

    source.snapshot_ready = false;
    memset(&result, 0x5A, sizeof(result));
    assert(ParameterRead_Get(&source, PARAMETER_READ_ID_MAX_SPEED_RAD_S, &result) ==
           PARAMETER_READ_NOT_READY);
    assert(result.value_type == 0U && result.value == 0);
    assert(ParameterRead_Get(&source, 3U, &result) == PARAMETER_READ_UNSUPPORTED);
    puts("PASS unknown ID rejects without reporting a stale or zero value");
}

static void invalid_and_out_of_range_reject(void)
{
    ParameterReadSource source = make_source();
    ParameterReadResult result;

    source.values.speed_limit = -1.0f;
    assert(ParameterRead_Get(&source, PARAMETER_READ_ID_MAX_SPEED_RAD_S, &result) ==
           PARAMETER_READ_INVALID_VALUE);
    assert(result.value_type == 0U && result.value == 0);
    source.values.speed_limit = 0.0f;
    assert(ParameterRead_Get(&source, PARAMETER_READ_ID_MAX_SPEED_RAD_S, &result) ==
           PARAMETER_READ_INVALID_VALUE);
    source.values.speed_limit = NAN;
    assert(ParameterRead_Get(&source, PARAMETER_READ_ID_MAX_SPEED_RAD_S, &result) ==
           PARAMETER_READ_INVALID_VALUE);
    source.values.speed_limit = INFINITY;
    assert(ParameterRead_Get(&source, PARAMETER_READ_ID_MAX_SPEED_RAD_S, &result) ==
           PARAMETER_READ_INVALID_VALUE);
    source.values.speed_limit = 3.0e9f;
    assert(ParameterRead_Get(&source, PARAMETER_READ_ID_MAX_SPEED_RAD_S, &result) ==
           PARAMETER_READ_INVALID_VALUE);
    source = make_source();
    source.values.current_limit = -0.25f;
    assert(ParameterRead_Get(&source, PARAMETER_READ_ID_MAX_CURRENT_A, &result) ==
           PARAMETER_READ_INVALID_VALUE);
    assert(ParameterRead_Get(NULL, PARAMETER_READ_ID_MAX_CURRENT_A, &result) ==
           PARAMETER_READ_INVALID_ARGUMENT);
    assert(ParameterRead_Get(&source, PARAMETER_READ_ID_MAX_CURRENT_A, NULL) ==
           PARAMETER_READ_INVALID_ARGUMENT);
    puts("PASS negative, zero, NaN, Inf and overflow values are explicit errors");
}

static void handler_maps_read_and_rejects_writes(void)
{
    ParameterReadSource source = make_source();
    ParameterReadSource before = source;
    yg_protocol_parameter_adapter_t handler = {0};
    yg_protocol_parameter_service_t service;
    yg_protocol_parameter_request_t request = {.operation = YG_PROTOCOL_PARAMETER_READ,
                                               .parameter_id = PARAMETER_READ_ID_MAX_SPEED_RAD_S};
    yg_protocol_service_reply_t reply;

    handler.source = source;
    service.context = &handler;
    service.handler = yg_protocol_parameter_adapter_handle;
    assert(yg_protocol_parameter_call(&service, &request, &reply) == YG_PROTOCOL_SERVICE_OK);
    assert(reply.value == 20000 && reply.revision == 7U);
    assert(reply.token == 0U && reply.detail == 0U);

    request.parameter_id = 3U;
    memset(&reply, 0x5A, sizeof(reply));
    assert(yg_protocol_parameter_call(&service, &request, &reply) ==
           YG_PROTOCOL_SERVICE_UNSUPPORTED);
    assert(reply.value == 0 && reply.token == 0U && reply.revision == 0U && reply.detail == 0U);

    request.operation = YG_PROTOCOL_PARAMETER_WRITE;
    request.parameter_id = PARAMETER_READ_ID_MAX_SPEED_RAD_S;
    request.value = 12345;
    assert(yg_protocol_parameter_call(&service, &request, &reply) ==
           YG_PROTOCOL_SERVICE_UNSUPPORTED);
    request.operation = YG_PROTOCOL_PARAMETER_SAVE;
    assert(yg_protocol_parameter_call(&service, &request, &reply) ==
           YG_PROTOCOL_SERVICE_UNSUPPORTED);
    request.operation = YG_PROTOCOL_PARAMETER_RESTORE_DEFAULTS;
    assert(yg_protocol_parameter_call(&service, &request, &reply) ==
           YG_PROTOCOL_SERVICE_UNSUPPORTED);
    assert(memcmp(&handler.source, &before, sizeof(before)) == 0);

    request.operation = YG_PROTOCOL_PARAMETER_READ;
    request.parameter_id = PARAMETER_READ_ID_MAX_SPEED_RAD_S;
    handler.source.snapshot_ready = false;
    assert(yg_protocol_parameter_call(&service, &request, &reply) == YG_PROTOCOL_SERVICE_BUSY);
    handler.source.snapshot_ready = true;
    handler.source.values.speed_limit = INFINITY;
    assert(yg_protocol_parameter_call(&service, &request, &reply) == YG_PROTOCOL_SERVICE_FAILED);

    handler.source = before;
    service.handler = NULL;
    assert(yg_protocol_parameter_call(&service, &request, &reply) ==
           YG_PROTOCOL_SERVICE_UNSUPPORTED);
    puts("PASS handler maps READ, rejects writes and leaves the read-only copy unchanged");
}

static void handler_guards_null_reply(void)
{
    ParameterReadSource source = make_source();
    yg_protocol_parameter_adapter_t handler = {0};
    yg_protocol_parameter_request_t request = {.operation = YG_PROTOCOL_PARAMETER_READ,
                                               .parameter_id = PARAMETER_READ_ID_ACTIVE_REVISION};

    handler.source = source;
    yg_protocol_parameter_adapter_handle(&handler, &request, NULL);
    yg_protocol_parameter_adapter_handle(NULL, &request, NULL);
    puts("PASS adapter tolerates a null reply without touching the source");
}

int main(void)
{
    read_units_are_explicit();
    unknown_and_not_ready_are_distinct();
    invalid_and_out_of_range_reject();
    handler_maps_read_and_rejects_writes();
    handler_guards_null_reply();
    puts("PASS yg_protocol parameter read handler");
    return 0;
}
