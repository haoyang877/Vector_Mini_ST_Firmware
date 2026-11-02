/* 电机 STOP/DISABLE 通信适配器离线测试：适配器 + 平台无关停机服务。
 * 所有者替身直接使用 五态纯核心，证明处理器复用现有生命周期语义、不会恒返回成功，
 * 且只有所有者确认功率输出禁止才报告 OK。不访问硬件、不接线 CAN 入口。 */

#include <stdbool.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "motor_state_machine.h"
#include "motor_stop_request.h"
#include "yg_protocol_motor.h"
#include "yg_protocol_motor_adapter.h"

static int failures;

/* 测试入口生成的配置体直接来自APP，验证发现页和正式控制入口一致。 */
extern bool test_application_capabilities(void);

#define CHECK(condition)                                                                           \
    do                                                                                             \
    {                                                                                              \
        if (!(condition))                                                                          \
        {                                                                                          \
            printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition);                            \
            ++failures;                                                                            \
        }                                                                                          \
    } while (0)

/* ===== 所有者替身 1：直接复用 五态纯核心的真实语义 ===== */

typedef struct
{
    MotorState state;
    bool power_disabled, denied;
} LifecycleOwner;

static void LifecycleOwner_PowerOn(LifecycleOwner *owner)
{
    owner->state = MotorStateMachine_Step(MOTOR_STATE_IDLE, MOTOR_STATE_EVENT_RUN);
    owner->power_disabled = false;
    owner->denied = false;
}

static void LifecycleOwner_Fatal(LifecycleOwner *owner)
{
    owner->state = MotorStateMachine_Step(owner->state, MOTOR_STATE_EVENT_FAULT);
    owner->denied = true;
}

static bool LifecycleOwner_RequestStop(void *owner_context, uint8_t source_node, uint16_t sequence)
{
    LifecycleOwner *owner = owner_context;
    (void)source_node;
    (void)sequence;
    if (owner->denied)
    {
        return false;
    }
    owner->power_disabled = true;
    owner->state = MotorStateMachine_Step(owner->state, MOTOR_STATE_EVENT_STOP);
    return true;
}

static bool LifecycleOwner_IsPowerDisabled(void *owner_context)
{
    const LifecycleOwner *owner = owner_context;
    return owner->power_disabled;
}

static const motor_stop_owner_port_t LIFECYCLE_PORT = {
    LifecycleOwner_RequestStop,
    LifecycleOwner_IsPowerDisabled,
};

/* ===== 所有者替身 2：可脚本化的异步 / 无法确认所有者 ===== */

typedef struct
{
    bool request_result;
    bool disabled;
    int request_calls;
    uint8_t last_source;
    uint16_t last_sequence;
} ScriptedOwner;

static bool ScriptedOwner_RequestStop(void *owner_context, uint8_t source_node, uint16_t sequence)
{
    ScriptedOwner *owner = owner_context;
    ++owner->request_calls;
    owner->last_source = source_node;
    owner->last_sequence = sequence;
    return owner->request_result;
}

static bool ScriptedOwner_IsPowerDisabled(void *owner_context)
{
    const ScriptedOwner *owner = owner_context;
    return owner->disabled;
}

static const motor_stop_owner_port_t SCRIPTED_PORT = {
    ScriptedOwner_RequestStop,
    ScriptedOwner_IsPowerDisabled,
};

/* ===== 调用辅助 ===== */

static yg_protocol_motor_request_t make_request(yg_protocol_motor_operation_t operation)
{
    yg_protocol_motor_request_t request;

    memset(&request, 0, sizeof(request));
    request.operation = operation;
    request.source_node = 1U;
    request.sequence = 7U;
    return request;
}

static yg_protocol_service_status_t call_handler(yg_protocol_motor_adapter_t *handler,
                                                 yg_protocol_motor_operation_t operation,
                                                 yg_protocol_service_reply_t *reply)
{
    yg_protocol_motor_service_t service;
    yg_protocol_motor_request_t request = make_request(operation);

    service.context = handler;
    service.handler = yg_protocol_motor_adapter_handle;
    return yg_protocol_motor_call(&service, &request, reply);
}

static void test_early_returns(void)
{
    yg_protocol_motor_adapter_t handler = {0};
    yg_protocol_service_reply_t reply;
    yg_protocol_motor_request_t request = make_request(YG_PROTOCOL_MOTOR_STOP);
    yg_protocol_motor_operation_t unknown = (yg_protocol_motor_operation_t)99;

    /* 无绑定处理器：未实现返回 UNSUPPORTED，不是 OK。 */
    CHECK(call_handler(NULL, YG_PROTOCOL_MOTOR_STOP, &reply) ==
          YG_PROTOCOL_SERVICE_INVALID_ARGUMENT);

    /* 绑定存在但没有停机服务：明确 UNSUPPORTED。 */
    CHECK(call_handler(&handler, YG_PROTOCOL_MOTOR_STOP, &reply) ==
          YG_PROTOCOL_SERVICE_UNSUPPORTED);

    /* 稳定初始化：旧值不得泄漏到任何早期返回路径。 */
    memset(&reply, 0xEE, sizeof(reply));
    (void)yg_protocol_motor_adapter_handle(NULL, &request, &reply);
    CHECK(reply.status == YG_PROTOCOL_SERVICE_INVALID_ARGUMENT);
    CHECK(reply.token == 0U);
    CHECK(reply.revision == 0U);
    CHECK(reply.value == 0);
    CHECK(reply.detail == 0U);

    /* 空请求 / 空输出不崩溃且给出确定状态。 */
    memset(&reply, 0xEE, sizeof(reply));
    (void)yg_protocol_motor_adapter_handle(&handler, NULL, &reply);
    CHECK(reply.status == YG_PROTOCOL_SERVICE_INVALID_ARGUMENT);
    (void)yg_protocol_motor_adapter_handle(&handler, &request, NULL);

    /* 本批不提供使能/模式/目标：一律 UNSUPPORTED。 */
    CHECK(call_handler(&handler, YG_PROTOCOL_MOTOR_ENABLE, &reply) ==
          YG_PROTOCOL_SERVICE_UNSUPPORTED);
    CHECK(call_handler(&handler, YG_PROTOCOL_MOTOR_SET_CONTROL, &reply) ==
          YG_PROTOCOL_SERVICE_UNSUPPORTED);

    /* 越界内部操作按参数错误处理，不构造成功响应。 */
    request.operation = unknown;
    (void)yg_protocol_motor_adapter_handle(&handler, &request, &reply);
    CHECK(reply.status == YG_PROTOCOL_SERVICE_INVALID_ARGUMENT);

    printf("PASS early returns and stable initialization\n");
}

static void test_service_api_misuse(void)
{
    motor_stop_service_t service;
    uint32_t token = 0xEEU;
    motor_stop_owner_port_t incomplete = {ScriptedOwner_RequestStop, NULL};

    /* 未初始化服务：UNAVAILABLE，绝不假报停机。 */
    CHECK(MotorStopService_RequestStop(NULL, 1U, 7U, &token) == MOTOR_STOP_RESULT_FAILED);
    CHECK(token == 0U);
    memset(&service, 0, sizeof(service));
    CHECK(MotorStopService_RequestStop(&service, 1U, 7U, &token) == MOTOR_STOP_RESULT_UNAVAILABLE);

    /* 缺少确认通道的端口拒绝初始化。 */
    CHECK(!MotorStopService_Init(&service, &incomplete, NULL));
    CHECK(!MotorStopService_Init(NULL, &SCRIPTED_PORT, NULL));
    CHECK(!MotorStopService_Init(&service, NULL, NULL));

    /* 空 token 输出按 API 误用处理。 */
    CHECK(MotorStopService_Init(&service, &SCRIPTED_PORT, NULL));
    CHECK(MotorStopService_RequestStop(&service, 1U, 7U, NULL) == MOTOR_STOP_RESULT_FAILED);

    printf("PASS service API misuse and missing confirmation channel\n");
}

static void test_lifecycle_owner_stop_confirmed(void)
{
    LifecycleOwner owner;
    motor_stop_service_t stop;
    yg_protocol_motor_adapter_t handler;
    yg_protocol_service_reply_t reply;
    yg_protocol_motor_request_t request = make_request(YG_PROTOCOL_MOTOR_STOP);
    yg_protocol_motor_request_t request_before = request;
    uint32_t first_token;

    LifecycleOwner_PowerOn(&owner);
    CHECK(owner.state == MOTOR_STATE_RUNNING);
    CHECK(!owner.power_disabled);

    CHECK(MotorStopService_Init(&stop, &LIFECYCLE_PORT, &owner));
    handler.stop_service = &stop;

    /* 所有者同步关相：确认通道立即成立，返回 OK 与非零 token。 */
    CHECK(yg_protocol_motor_call(
              &(yg_protocol_motor_service_t){&handler, yg_protocol_motor_adapter_handle},
              &request,
              &reply) == YG_PROTOCOL_SERVICE_OK);
    CHECK(reply.status == YG_PROTOCOL_SERVICE_OK);
    CHECK(reply.token != 0U);
    CHECK(owner.power_disabled);
    CHECK(owner.state == MOTOR_STATE_IDLE);

    /* 请求对象只借用：适配器与所有者都不得改动调用方数据。 */
    CHECK(memcmp(&request_before, &request, sizeof(request)) == 0);

    /* 幂等：重复 STOP 与 DISABLE 都返回 OK 与同一 token，不再次打扰所有者。 */
    first_token = reply.token;
    CHECK(call_handler(&handler, YG_PROTOCOL_MOTOR_STOP, &reply) == YG_PROTOCOL_SERVICE_OK);
    CHECK(reply.token == first_token);
    CHECK(call_handler(&handler, YG_PROTOCOL_MOTOR_DISABLE, &reply) == YG_PROTOCOL_SERVICE_OK);
    CHECK(reply.token == first_token);

    printf("PASS lifecycle owner confirms stop and idempotent repeat\n");
}

static void test_lifecycle_owner_denied(void)
{
    LifecycleOwner owner;
    motor_stop_service_t stop;
    yg_protocol_motor_adapter_t handler;
    yg_protocol_service_reply_t reply;

    LifecycleOwner_PowerOn(&owner);
    LifecycleOwner_Fatal(&owner);
    CHECK(owner.state == MOTOR_STATE_FAULT);

    CHECK(MotorStopService_Init(&stop, &LIFECYCLE_PORT, &owner));
    handler.stop_service = &stop;

    /* 所有者当前状态拒绝停机：DENIED，不是 OK，也不生成 token。 */
    CHECK(call_handler(&handler, YG_PROTOCOL_MOTOR_STOP, &reply) == YG_PROTOCOL_SERVICE_DENIED);
    CHECK(reply.status == YG_PROTOCOL_SERVICE_DENIED);
    CHECK(reply.token == 0U);
    CHECK(reply.detail == (uint16_t)MOTOR_STOP_RESULT_DENIED);

    printf("PASS lifecycle owner denial is propagated, not faked as OK\n");
}

static void test_async_accept_then_confirm(void)
{
    ScriptedOwner owner = {.request_result = true};
    motor_stop_service_t stop;
    yg_protocol_motor_adapter_t handler;
    yg_protocol_service_reply_t reply;
    uint32_t token;

    CHECK(MotorStopService_Init(&stop, &SCRIPTED_PORT, &owner));
    handler.stop_service = &stop;

    /* 所有者受理但未确认：ACCEPTED + token，功率级未关断。 */
    CHECK(call_handler(&handler, YG_PROTOCOL_MOTOR_STOP, &reply) == YG_PROTOCOL_SERVICE_ACCEPTED);
    CHECK(reply.token != 0U);
    CHECK(!owner.disabled);
    token = reply.token;
    CHECK(owner.request_calls == 1);

    /* 每个有效 STOP 都交给所有者更新序号屏障，服务仍复用 token。 */
    CHECK(call_handler(&handler, YG_PROTOCOL_MOTOR_DISABLE, &reply) ==
          YG_PROTOCOL_SERVICE_ACCEPTED);
    CHECK(reply.token == token);
    CHECK(owner.request_calls == 2);

    /* 所有者确认功率输出禁止后，同一 token 升级为 OK。 */
    owner.disabled = true;
    CHECK(call_handler(&handler, YG_PROTOCOL_MOTOR_STOP, &reply) == YG_PROTOCOL_SERVICE_OK);
    CHECK(reply.token == token);
    CHECK(owner.request_calls == 3);

    /* 旧 OK 不能掩盖后来重新使能；新的 STOP 必须重新请求并等待新确认。 */
    owner.disabled = false;
    CHECK(call_handler(&handler, YG_PROTOCOL_MOTOR_STOP, &reply) == YG_PROTOCOL_SERVICE_ACCEPTED);
    CHECK(reply.token != 0U && reply.token != token);
    CHECK(owner.request_calls == 4);
    token = reply.token;
    owner.disabled = true;
    CHECK(call_handler(&handler, YG_PROTOCOL_MOTOR_STOP, &reply) == YG_PROTOCOL_SERVICE_OK);
    CHECK(reply.token == token);

    printf("PASS asynchronous ACCEPTED/token then confirmed OK\n");
}

static void test_first_denied_then_accepted(void)
{
    ScriptedOwner owner = {.request_result = false};
    motor_stop_service_t stop;
    yg_protocol_motor_adapter_t handler;
    yg_protocol_service_reply_t reply;

    CHECK(MotorStopService_Init(&stop, &SCRIPTED_PORT, &owner));
    handler.stop_service = &stop;

    /* 首次拒绝后服务不得停留在 pending：修正后再次请求必须重新下达。 */
    CHECK(call_handler(&handler, YG_PROTOCOL_MOTOR_STOP, &reply) == YG_PROTOCOL_SERVICE_DENIED);
    owner.request_result = true;
    owner.disabled = true;
    CHECK(call_handler(&handler, YG_PROTOCOL_MOTOR_STOP, &reply) == YG_PROTOCOL_SERVICE_OK);
    CHECK(owner.request_calls == 2);

    printf("PASS denied stop can be retried and confirmed\n");
}

static void test_stop_identity_reaches_owner_in_every_phase(void)
{
    ScriptedOwner owner = {.request_result = true};
    motor_stop_service_t stop;
    uint32_t token = 0U;
    uint32_t original;
    CHECK(MotorStopService_Init(&stop, &SCRIPTED_PORT, &owner));
    CHECK(MotorStopService_RequestStop(&stop, 2U, 10U, &token) == MOTOR_STOP_RESULT_ACCEPTED);
    original = token;
    CHECK(owner.last_source == 2U && owner.last_sequence == 10U && owner.request_calls == 1);
    CHECK(MotorStopService_RequestStop(&stop, 2U, 11U, &token) == MOTOR_STOP_RESULT_ACCEPTED);
    CHECK(token == original && owner.last_sequence == 11U && owner.request_calls == 2);
    owner.disabled = true;
    CHECK(MotorStopService_RequestStop(&stop, 2U, 12U, &token) == MOTOR_STOP_RESULT_OK);
    CHECK(token == original && owner.last_sequence == 12U && owner.request_calls == 3);
    CHECK(MotorStopService_RequestStop(&stop, 2U, 9U, &token) == MOTOR_STOP_RESULT_OK);
    CHECK(token == original && owner.last_sequence == 9U && owner.request_calls == 4);
}

static void test_unconfirmable_shutdown_never_ok(void)
{
    ScriptedOwner owner = {.request_result = true};
    motor_stop_service_t stop;
    yg_protocol_motor_adapter_t handler;
    yg_protocol_service_reply_t reply;
    uint32_t token = 0U;
    int i;

    CHECK(MotorStopService_Init(&stop, &SCRIPTED_PORT, &owner));
    handler.stop_service = &stop;

    /* 受理后永远拿不到关断证据：只允许 ACCEPTED，禁止假报 OK。 */
    for (i = 0; i < 1000; ++i)
    {
        yg_protocol_service_status_t status =
            call_handler(&handler, YG_PROTOCOL_MOTOR_DISABLE, &reply);
        CHECK(status == YG_PROTOCOL_SERVICE_ACCEPTED);
        CHECK(reply.status == YG_PROTOCOL_SERVICE_ACCEPTED);
        if (i == 0)
        {
            CHECK(reply.token != 0U);
            token = reply.token;
        }
        else
        {
            CHECK(reply.token == token);
        }
    }
    CHECK(owner.request_calls == 1000);
    CHECK(!owner.disabled);

    printf("PASS unconfirmable shutdown stays ACCEPTED and never reports OK\n");
}

static void test_control_boundary(void)
{
    yg_protocol_motor_adapter_t adapter = {0};
    yg_protocol_motor_request_t request = {.operation = YG_PROTOCOL_MOTOR_SET_CONTROL};
    yg_protocol_service_reply_t reply;
    for (uint8_t mode = 0; mode <= 10; ++mode)
    {
        request.mode = mode;
        yg_protocol_motor_adapter_handle(&adapter, &request, &reply);
        CHECK(reply.status == YG_PROTOCOL_SERVICE_UNSUPPORTED && reply.token == 0U);
    }
}

static void open_loop_service(void *context,
                              const yg_protocol_motor_request_t *request,
                              yg_protocol_service_reply_t *reply)
{
    unsigned *calls = context;
    CHECK(request->mode == 9U || request->mode == 10U);
    CHECK(request->operation == YG_PROTOCOL_MOTOR_START_CONTROL ||
          request->operation == YG_PROTOCOL_MOTOR_SET_CONTROL);
    ++calls[request->operation == YG_PROTOCOL_MOTOR_SET_CONTROL];
    reply->status = YG_PROTOCOL_SERVICE_OK;
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
        .message_type = YG_PROTOCOL_MOTOR_TYPE_CONTROL, .payload_length = 32U, .payload = voltage};
    yg_protocol_motor_request_t decoded;
    CHECK(yg_protocol_motor_decode_control(&message, &decoded) == YG_PROTOCOL_OK);
    CHECK(decoded.mode == 9 && decoded.electrical_frequency_mHz == 20000 && decoded.vd_mV == 1000 &&
          decoded.v_q_mV == 2000);
    message.payload = current;
    CHECK(yg_protocol_motor_decode_control(&message, &decoded) == YG_PROTOCOL_OK);
    CHECK(decoded.mode == 10U && decoded.electrical_frequency_mHz == -20000 &&
          decoded.id_mA == 1000 && decoded.iq_mA == 2000);
    yg_protocol_motor_request_t before = decoded;
    message.payload_length = 20U;
    CHECK(yg_protocol_motor_decode_control(&message, &decoded) == YG_PROTOCOL_INVALID_LENGTH);
    CHECK(memcmp(&before, &decoded, sizeof(before)) == 0);
    uint8_t invalid[32];
    memcpy(invalid, voltage, sizeof(invalid));
    invalid[24] = 1;
    message.payload = invalid;
    message.payload_length = 32;
    CHECK(yg_protocol_motor_decode_control(&message, &decoded) == YG_PROTOCOL_INVALID_FIELD);
    invalid[0] = 1;
    invalid[24] = 0;
    CHECK(yg_protocol_motor_decode_control(&message, &decoded) == YG_PROTOCOL_INVALID_LENGTH);
    unsigned calls[2] = {0};
    yg_protocol_motor_context_t context = {.node_id = 3U,
                                           .motor_service = {calls, open_loop_service}};
    context.motor_status_source = &context.empty_motor_status;
    uint8_t start_payload[4] = {9, 0, 0, 0};
    yg_protocol_message_t start = {.version = YG_PROTOCOL_VERSION,
                                   .flags = YG_PROTOCOL_FLAGS_ACK_REQUEST,
                                   .source_node = 2U,
                                   .destination_node = 3U,
                                   .sequence = 71U,
                                   .message_type = YG_PROTOCOL_MOTOR_TYPE_START,
                                   .payload_length = 4U,
                                   .payload = start_payload};
    yg_protocol_service_result_t result = {0};
    CHECK(yg_protocol_motor_handle_start(&context, &start, &result) == YG_PROTOCOL_OK);
    CHECK(calls[0] == 1U && result.response_payload[0] == 0U);
    start_payload[0] = 10U;
    CHECK(yg_protocol_motor_handle_start(&context, &start, &result) == YG_PROTOCOL_OK &&
          calls[0] == 2U);
    start_payload[0] = 7U;
    CHECK(yg_protocol_motor_handle_start(&context, &start, &result) == YG_PROTOCOL_OK &&
          calls[0] == 2U);
    CHECK(result.response_payload[0] == 2U);
    start.message_type = YG_PROTOCOL_MOTOR_TYPE_CONTROL;
    start.payload_length = 32U;
    start.payload = voltage;
    CHECK(yg_protocol_motor_handle_control(&context, &start, &result) == YG_PROTOCOL_OK);
    CHECK(calls[1] == 1U && calls[0] == 2U && result.response_payload[0] == 0U);
    CHECK(yg_protocol_motor_handle_control(&context, &start, &result) == YG_PROTOCOL_OK &&
          calls[1] == 2U);
    memcpy(invalid, voltage, sizeof(invalid));
    invalid[20] ^= 1U;
    start.payload = invalid;
    CHECK(yg_protocol_motor_handle_control(&context, &start, &result) == YG_PROTOCOL_OK &&
          calls[1] == 3U);
    CHECK(result.response_payload[0] == 0U);
    printf("PASS v0.3 open-loop golden payloads, SI mapping, lengths, start whitelist and "
           "extended-payload replay\n");
}

static void test_minimal_state(void)
{
    yg_protocol_motor_status_source_t source = {.state = 5U, .sample_available = true};
    yg_protocol_motor_feedback_t feedback;
    uint8_t payload[YG_PROTOCOL_MOTOR_FEEDBACK_PAYLOAD_SIZE];
    CHECK(yg_protocol_motor_feedback_from_source(&source, 3U, 0U, 0U, &feedback) == YG_PROTOCOL_OK);
    CHECK(feedback.valid_bits == 0U);
    CHECK(yg_protocol_motor_feedback_encode(&feedback, payload, sizeof(payload)) == YG_PROTOCOL_OK);
    printf("VECTOR minimal_state ");
    for (unsigned i = 0; i < sizeof(payload); ++i)
    {
        printf("%02x", payload[i]);
    }
    printf("\n");
}

int main(void)
{
    test_minimal_state();
    test_early_returns();
    test_service_api_misuse();
    test_lifecycle_owner_stop_confirmed();
    test_lifecycle_owner_denied();
    test_async_accept_then_confirm();
    test_first_denied_then_accepted();
    test_stop_identity_reaches_owner_in_every_phase();
    test_unconfirmable_shutdown_never_ok();
    test_control_boundary();
    ScriptedOwner wrap_owner = {.request_result = true};
    motor_stop_service_t wrap_service;
    uint32_t token;
    CHECK(MotorStopService_Init(&wrap_service, &SCRIPTED_PORT, &wrap_owner));
    wrap_service.token = UINT32_MAX;
    wrap_service.confirmed = true;
    CHECK(MotorStopService_RequestStop(&wrap_service, 2U, 0U, &token) ==
          MOTOR_STOP_RESULT_ACCEPTED);
    CHECK(token == 1U);
    test_open_loop_vectors();
    CHECK(test_application_capabilities());

    if (failures != 0)
    {
        printf("FAILED: %d checks\n", failures);
        return 1;
    }
    printf("PASS yg_protocol_motor_adapter (offline, no hardware)\n");
    return 0;
}
