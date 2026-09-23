/* 电机 STOP/DISABLE 通信适配器离线测试：适配器 + 平台无关停机服务。
 * 所有者替身直接使用 AppLifecycle 纯核心，证明处理器复用现有生命周期语义、不会恒返回成功，
 * 且只有所有者确认功率输出禁止才报告 OK。不访问硬件、不接线 CAN 入口。 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "app_lifecycle.h"
#include "motor_stop_service.h"
#include "yg_protocol_motor.h"
#include "yg_protocol_motor_adapter.h"

static int failures;

#define CHECK(condition)                                                                           \
    do                                                                                             \
    {                                                                                              \
        if (!(condition))                                                                          \
        {                                                                                          \
            printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition);                            \
            ++failures;                                                                            \
        }                                                                                          \
    } while (0)

/* ===== 所有者替身 1：直接复用 AppLifecycle 纯核心的真实语义 ===== */

typedef struct
{
    AppLifecycle lifecycle;
    bool power_disabled;
} LifecycleOwner;

static AppLifecycleGuards LifecycleOwner_Guards(const LifecycleOwner *owner)
{
    AppLifecycleGuards guards;

    guards.active_faults = 0U;
    guards.phases_disabled = owner->power_disabled;
    guards.control_idle = true;
    guards.operation_idle = true;
    guards.maintenance_released = true;
    guards.samples_fresh = true;
    guards.fault_sources_clear = true;
    guards.self_test_passed = true;
    guards.start_permitted = true;
    guards.operation_permitted = true;
    guards.startup_confirmed = true;
    guards.request_fresh = true;
    return guards;
}

/* 与真实适配器一致的动作语义：核心只请求，所有者记录功率级状态。 */
static void LifecycleOwner_Apply(LifecycleOwner *owner, uint32_t actions)
{
    if (actions & APP_ACTION_DISABLE_POWER)
    {
        owner->power_disabled = true;
    }
    if (actions & APP_ACTION_START_CONTROL)
    {
        owner->power_disabled = false;
    }
}

static void LifecycleOwner_Send(LifecycleOwner *owner, uint32_t events, AppControlMode mode)
{
    AppLifecycleEvent event;
    AppLifecycleGuards guards;
    AppLifecycleResult result;

    memset(&event, 0, sizeof(event));
    event.events = events;
    event.control_mode = mode;
    event.request_epoch = owner->lifecycle.snapshot.epoch;
    event.completion_epoch = owner->lifecycle.snapshot.epoch;
    guards = LifecycleOwner_Guards(owner);
    result = AppLifecycle_Step(&owner->lifecycle, &event, &guards);
    LifecycleOwner_Apply(owner, result.actions);
}

static void LifecycleOwner_Init(LifecycleOwner *owner)
{
    AppLifecycle_Init(&owner->lifecycle);
    owner->power_disabled = true;
}

/* 走完 BOOT → READY → STARTING → RUNNING，功率级记录为开。 */
static void LifecycleOwner_PowerOn(LifecycleOwner *owner)
{
    LifecycleOwner_Init(owner);
    LifecycleOwner_Send(owner, APP_EVENT_BOOT_DONE, APP_CONTROL_NONE);
    LifecycleOwner_Send(owner, APP_EVENT_SELF_TEST_DONE, APP_CONTROL_NONE);
    LifecycleOwner_Send(owner, APP_EVENT_ENABLE, APP_CONTROL_CURRENT);
    LifecycleOwner_Send(owner, APP_EVENT_START_DONE, APP_CONTROL_NONE);
}

static void LifecycleOwner_Fatal(LifecycleOwner *owner)
{
    AppLifecycleEvent event;
    AppLifecycleGuards guards;

    memset(&event, 0, sizeof(event));
    event.events = APP_EVENT_FATAL;
    event.faults = APP_FAULT_INTERNAL;
    guards = LifecycleOwner_Guards(owner);
    (void)AppLifecycle_Step(&owner->lifecycle, &event, &guards);
}

/* 所有者窄端口：请求停机即向核心发送 STOP；只有核心请求关相时才回报确认。 */
static motor_stop_owner_result_t LifecycleOwner_RequestStop(void *owner_context)
{
    LifecycleOwner *owner = owner_context;
    AppLifecycleEvent event;
    AppLifecycleGuards guards;
    AppLifecycleResult result;

    memset(&event, 0, sizeof(event));
    event.events = APP_EVENT_STOP;
    guards = LifecycleOwner_Guards(owner);
    result = AppLifecycle_Step(&owner->lifecycle, &event, &guards);
    LifecycleOwner_Apply(owner, result.actions);
    if (result.rejection == APP_ACCEPTED)
    {
        return MOTOR_STOP_OWNER_ACCEPTED;
    }
    return MOTOR_STOP_OWNER_DENIED;
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
    motor_stop_owner_result_t request_result;
    bool disabled;
    int request_calls;
} ScriptedOwner;

static motor_stop_owner_result_t ScriptedOwner_RequestStop(void *owner_context)
{
    ScriptedOwner *owner = owner_context;
    ++owner->request_calls;
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
    CHECK(MotorStopService_RequestStop(NULL, &token) == MOTOR_STOP_RESULT_FAILED);
    CHECK(token == 0U);
    memset(&service, 0, sizeof(service));
    CHECK(MotorStopService_RequestStop(&service, &token) == MOTOR_STOP_RESULT_UNAVAILABLE);

    /* 缺少确认通道的端口拒绝初始化。 */
    CHECK(!MotorStopService_Init(&service, &incomplete, NULL));
    CHECK(!MotorStopService_Init(NULL, &SCRIPTED_PORT, NULL));
    CHECK(!MotorStopService_Init(&service, NULL, NULL));

    /* 空 token 输出按 API 误用处理。 */
    CHECK(MotorStopService_Init(&service, &SCRIPTED_PORT, NULL));
    CHECK(MotorStopService_RequestStop(&service, NULL) == MOTOR_STOP_RESULT_FAILED);

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
    CHECK(owner.lifecycle.snapshot.state == APP_RUNNING);
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
    CHECK(owner.lifecycle.snapshot.state == APP_STOPPING);

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
    CHECK(owner.lifecycle.snapshot.state == APP_FATAL);

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
    ScriptedOwner owner = {MOTOR_STOP_OWNER_ACCEPTED, false, 0};
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

    /* 重复请求只轮询确认，不重复下达停机请求，token 保持稳定。 */
    CHECK(call_handler(&handler, YG_PROTOCOL_MOTOR_DISABLE, &reply) ==
          YG_PROTOCOL_SERVICE_ACCEPTED);
    CHECK(reply.token == token);
    CHECK(owner.request_calls == 1);

    /* 所有者确认功率输出禁止后，同一 token 升级为 OK。 */
    owner.disabled = true;
    CHECK(call_handler(&handler, YG_PROTOCOL_MOTOR_STOP, &reply) == YG_PROTOCOL_SERVICE_OK);
    CHECK(reply.token == token);
    CHECK(owner.request_calls == 1);

    /* 旧 OK 不能掩盖后来重新使能；新的 STOP 必须重新请求并等待新确认。 */
    owner.disabled = false;
    CHECK(call_handler(&handler, YG_PROTOCOL_MOTOR_STOP, &reply) == YG_PROTOCOL_SERVICE_ACCEPTED);
    CHECK(reply.token != 0U && reply.token != token);
    CHECK(owner.request_calls == 2);
    token = reply.token;
    owner.disabled = true;
    CHECK(call_handler(&handler, YG_PROTOCOL_MOTOR_STOP, &reply) == YG_PROTOCOL_SERVICE_OK);
    CHECK(reply.token == token);

    printf("PASS asynchronous ACCEPTED/token then confirmed OK\n");
}

static void test_first_denied_then_accepted(void)
{
    ScriptedOwner owner = {MOTOR_STOP_OWNER_DENIED, false, 0};
    motor_stop_service_t stop;
    yg_protocol_motor_adapter_t handler;
    yg_protocol_service_reply_t reply;

    CHECK(MotorStopService_Init(&stop, &SCRIPTED_PORT, &owner));
    handler.stop_service = &stop;

    /* 首次拒绝后服务不得停留在 pending：修正后再次请求必须重新下达。 */
    CHECK(call_handler(&handler, YG_PROTOCOL_MOTOR_STOP, &reply) == YG_PROTOCOL_SERVICE_DENIED);
    owner.request_result = MOTOR_STOP_OWNER_ACCEPTED;
    owner.disabled = true;
    CHECK(call_handler(&handler, YG_PROTOCOL_MOTOR_STOP, &reply) == YG_PROTOCOL_SERVICE_OK);
    CHECK(owner.request_calls == 2);

    printf("PASS denied stop can be retried and confirmed\n");
}

static void test_unconfirmable_shutdown_never_ok(void)
{
    ScriptedOwner owner = {MOTOR_STOP_OWNER_ACCEPTED, false, 0};
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
    CHECK(owner.request_calls == 1);
    CHECK(!owner.disabled);

    printf("PASS unconfirmable shutdown stays ACCEPTED and never reports OK\n");
}

int main(void)
{
    test_early_returns();
    test_service_api_misuse();
    test_lifecycle_owner_stop_confirmed();
    test_lifecycle_owner_denied();
    test_async_accept_then_confirm();
    test_first_denied_then_accepted();
    test_unconfirmable_shutdown_never_ok();

    if (failures != 0)
    {
        printf("FAILED: %d checks\n", failures);
        return 1;
    }
    printf("PASS yg_protocol_motor_adapter (offline, no hardware)\n");
    return 0;
}
