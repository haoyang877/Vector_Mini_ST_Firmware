#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "comm_hw.h"
#include "critical_hw.h"
#include "yg_protocol_canfd.h"
#include "yg_protocol_link.h"
#include "yg_protocol_runtime.h"
#include "yg_protocol_readonly.h"
#include "yg_protocol_readonly_payload.h"
#include "yg_protocol_boot.h"

static CommHwCanFrame sent_frame;
static bool sent;
static uint32_t irq_mask;
static unsigned motor_calls;
static uint16_t start_stop_barrier;
static bool tx_blocked;
static CommHwCanFrame injected_frame;
static bool inject_on_unlock;
static uint16_t sent_sequences[32];
static size_t sent_count;
static unsigned can_starts;
static unsigned bus_off_services;
static yg_protocol_motor_operation_t power_operation;
static unsigned run_calls;
static unsigned run_progress_calls;
static unsigned plan_calls;
static bool clear_allowed;
static uint8_t last_stop_source;
static yg_protocol_message_t exchange_message(const yg_protocol_message_t *request,
                                              uint32_t now_ms);

/* 本夹具未绑定 boot 服务；普通响应不得进入跟踪发送通道，也不伪造发送完成。 */
bool comm_hw_can_try_send_tracked_frame(const CommHwCanFrame *frame, uint64_t ticket)
{
    (void)frame;
    (void)ticket;
    assert(false);
    return false;
}

bool comm_hw_can_take_tx_completion(CommHwCanTxCompletion *completion)
{
    (void)completion;
    return false;
}

/* 运行时测试用的硬件替身：只计数，不接触真实外设。 */
void comm_hw_can_start_fd(void)
{
    ++can_starts;
}

bool comm_hw_can_service_bus_off(void)
{
    ++bus_off_services;
    return true;
}

uint32_t critical_hw_enter(void)
{
    uint32_t previous = irq_mask;
    irq_mask = 1U;
    return previous;
}

void critical_hw_exit(uint32_t state)
{
    assert(irq_mask == 1U);
    irq_mask = state;
    if (state == 0U && inject_on_unlock)
    {
        inject_on_unlock = false;
        assert(YgProtocolLink_OnRxFrame(&injected_frame));
    }
}

/* 替代平台发送口，验证链路只读请求的编码、路由与响应，不接触真实 CAN 或电机。 */
bool comm_hw_can_try_send_frame(const CommHwCanFrame *frame)
{
    assert(irq_mask == 0U);
    if (frame == NULL || sent || tx_blocked)
    {
        return false;
    }
    sent_frame = *frame;
    sent = true;
    assert(sent_count < sizeof(sent_sequences) / sizeof(sent_sequences[0]));
    sent_sequences[sent_count++] = (uint16_t)frame->data[8] | ((uint16_t)frame->data[9] << 8U);
    return true;
}

static void motor_handler(void *context,
                          const yg_protocol_motor_request_t *request,
                          yg_protocol_service_reply_t *reply)
{
    (void)context;
    assert(irq_mask == 0U);
    assert(request->operation == YG_PROTOCOL_MOTOR_STOP);
    ++motor_calls;
    reply->status = YG_PROTOCOL_SERVICE_OK;
}

static void control_handler(void *context,
                            const yg_protocol_motor_request_t *request,
                            yg_protocol_service_reply_t *reply)
{
    (void)context;
    assert(request->operation == YG_PROTOCOL_MOTOR_SET_CONTROL && request->mode == 3U &&
           request->position_mrad == 1000);
    ++motor_calls;
    reply->status = YG_PROTOCOL_SERVICE_ACCEPTED;
}

/* 在发送持续忙时填满 TX，随后恢复发送：命令不得执行两次或乱序丢失。 */
static void backlog_and_interrupt_handoff(void)
{
    yg_protocol_motor_service_t service = {NULL, motor_handler};
    yg_protocol_message_t request = {.version = YG_PROTOCOL_VERSION,
                                     .flags = YG_PROTOCOL_FLAGS_ACK_REQUEST,
                                     .source_node = 2U,
                                     .destination_node = 7U,
                                     .message_type = YG_PROTOCOL_MOTOR_TYPE_STOP};
    yg_protocol_transfer_frame_t frame;
    assert(YgProtocolLink_Init(7U));
    assert(YgProtocolLink_BindMotorService(&service));
    motor_calls = 0U;
    sent_count = 0U;
    sent = false;
    tx_blocked = true;
    for (unsigned index = 0U; index < 8U; ++index)
    {
        request.sequence = (uint16_t)index;
        assert(yg_protocol_canfd_pack(&request, 3U, &frame) == YG_PROTOCOL_OK);
        injected_frame = (CommHwCanFrame){.identifier = frame.identifier,
                                          .length = frame.length,
                                          .extended = true,
                                          .fd = true,
                                          .bitrate_switch = true};
        memcpy(injected_frame.data, frame.data, frame.length);
        assert(YgProtocolLink_OnRxFrame(&injected_frame));
    }
    /* RX 队列 8 帧已满，最新帧丢弃；后台服务再处理这 8 帧。 */
    assert(!YgProtocolLink_OnRxFrame(&injected_frame));
    assert(motor_calls == 0U);
    YgProtocolLink_Service(10U);
    /* TX 仅容纳 8 帧，预留两个回复槽后第 8 条请求必须留在 RX。 */
    assert(motor_calls == 7U && sent_count == 0U);
    YgProtocolLink_Service(12U);
    assert(motor_calls == 7U);
    tx_blocked = false;
    for (unsigned i = 0U; i < 20U; ++i)
    {
        sent = false;
        YgProtocolLink_Service(13U + i);
    }
    assert(motor_calls == 8U && sent_count == 8U);
    for (unsigned i = 0U; i < 8U; ++i)
    {
        assert(sent_sequences[i] == i);
    }
    /* 在后台解锁出队副本时模拟 RX 中断，验证新帧不会覆盖已出队的请求。 */
    assert(YgProtocolLink_OnRxFrame(&injected_frame));
    inject_on_unlock = true;
    sent = false;
    YgProtocolLink_Service(100U);
    assert(motor_calls == 10U && sent_count == 9U && !inject_on_unlock && irq_mask == 0U);
}

static void put_u16(uint8_t *buffer, uint16_t value)
{
    buffer[0] = (uint8_t)value;
    buffer[1] = (uint8_t)(value >> 8U);
}

static void put_u32(uint8_t *buffer, uint32_t value)
{
    buffer[0] = (uint8_t)value;
    buffer[1] = (uint8_t)(value >> 8U);
    buffer[2] = (uint8_t)(value >> 16U);
    buffer[3] = (uint8_t)(value >> 24U);
}

static void put_u64(uint8_t *buffer, uint64_t value)
{
    put_u32(buffer, (uint32_t)value);
    put_u32(buffer + 4U, (uint32_t)(value >> 32U));
}

static void lifecycle_golden_frames(void)
{
    static const uint8_t expected_run[48] = {
        0x5A, 0xA5, 0x01, 0x20, 0x02, 0x07, 0x76, 0x00, 0x2A, 0x00, 0x14, 0x00,
        0x00, 0x00, 0x00, 0x90, 0x01, 0x01, 0x03, 0x00, 0xE8, 0x03, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x88, 0x77, 0x66, 0x55, 0x44, 0x33, 0x22, 0x11,
        0x59, 0xF6, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    static const uint8_t expected_plan[48] = {
        0x5A, 0xA5, 0x01, 0x20, 0x02, 0x07, 0xA6, 0x00, 0x2B, 0x00, 0x14, 0x00,
        0x00, 0x00, 0x00, 0xC3, 0x01, 0x01, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x88, 0x77, 0x66, 0x55, 0x44, 0x33, 0x22, 0x11,
        0xD5, 0xEE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    static const uint8_t expected_run_reply[24] = {0x5A, 0xA5, 0x01, 0x10, 0x07, 0x02, 0x76, 0x00,
                                                   0x2A, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x08,
                                                   0x01, 0x00, 0x00, 0x00, 0x2D, 0x07, 0x00, 0x00};
    static const uint8_t expected_plan_reply[48] = {
        0x5A, 0xA5, 0x01, 0x10, 0x07, 0x02, 0xA6, 0x00, 0x2B, 0x00, 0x18, 0x00,
        0x00, 0x00, 0x00, 0x03, 0x01, 0x00, 0x00, 0x00, 0x78, 0x56, 0x34, 0x12,
        0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x01, 0x01, 0x00, 0x00, 0xE6, 0xA4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t run_payload[YG_PROTOCOL_RUN_SESSION_REQUEST_SIZE] = {1U, 1U, 3U};
    uint8_t plan_payload[YG_PROTOCOL_PLAN_START_SIZE] = {1U, 1U};
    uint8_t run_reply[YG_PROTOCOL_MOTOR_REPLY_SIZE] = {1U};
    uint8_t plan_reply[YG_PROTOCOL_PLAN_REPLY_SIZE] = {1U};
    yg_protocol_message_t request = {.version = YG_PROTOCOL_VERSION,
                                     .flags = YG_PROTOCOL_FLAGS_ACK_REQUEST,
                                     .source_node = 2U,
                                     .destination_node = 7U,
                                     .message_type = YG_PROTOCOL_MOTOR_TYPE_RUN_SESSION,
                                     .sequence = 0x2AU,
                                     .payload_length = sizeof(run_payload),
                                     .payload = run_payload};
    yg_protocol_transfer_frame_t frame;
    put_u32(run_payload + 4U, 1000U);
    put_u64(run_payload + 12U, UINT64_C(0x1122334455667788));
    assert(yg_protocol_canfd_pack(&request, 3U, &frame) == YG_PROTOCOL_OK);
    assert(frame.identifier == 0x0CEF0702U && frame.length == 48U &&
           memcmp(frame.data, expected_run, sizeof(expected_run)) == 0);
    request.message_type = YG_PROTOCOL_JOB_TYPE_PLAN;
    request.sequence = 0x2BU;
    request.payload = plan_payload;
    put_u32(plan_payload + 4U, 3U);
    put_u64(plan_payload + 12U, UINT64_C(0x1122334455667788));
    assert(yg_protocol_canfd_pack(&request, 3U, &frame) == YG_PROTOCOL_OK);
    assert(frame.identifier == 0x0CEF0702U && frame.length == 48U &&
           memcmp(frame.data, expected_plan, sizeof(expected_plan)) == 0);
    request.flags = YG_PROTOCOL_FLAGS_RESPONSE;
    request.source_node = 7U;
    request.destination_node = 2U;
    request.message_type = YG_PROTOCOL_MOTOR_TYPE_RUN_SESSION;
    request.sequence = 0x2AU;
    request.payload_length = sizeof(run_reply);
    request.payload = run_reply;
    assert(yg_protocol_canfd_pack(&request, 3U, &frame) == YG_PROTOCOL_OK);
    assert(frame.identifier == 0x0CEF0207U && frame.length == 24U &&
           memcmp(frame.data, expected_run_reply, sizeof(expected_run_reply)) == 0);
    request.message_type = YG_PROTOCOL_JOB_TYPE_PLAN;
    request.sequence = 0x2BU;
    request.payload_length = sizeof(plan_reply);
    request.payload = plan_reply;
    put_u32(plan_reply + 4U, 0x12345678U);
    put_u32(plan_reply + 12U, 3U);
    plan_reply[20] = 1U;
    plan_reply[21] = 1U;
    assert(yg_protocol_canfd_pack(&request, 3U, &frame) == YG_PROTOCOL_OK);
    assert(frame.identifier == 0x0CEF0207U && frame.length == 48U &&
           memcmp(frame.data, expected_plan_reply, sizeof(expected_plan_reply)) == 0);
}

static void run_handler(void *context,
                        const yg_protocol_run_request_t *request,
                        yg_protocol_service_reply_t *reply)
{
    (void)context;
    if (request->operation == YG_PROTOCOL_RUN_SESSION_PROGRESS)
    {
        ++run_progress_calls;
        reply->status = YG_PROTOCOL_SERVICE_OK;
        return;
    }
    ++run_calls;
    reply->status = request->operation == YG_PROTOCOL_RUN_SESSION_RENEW
                        ? YG_PROTOCOL_SERVICE_OK
                        : YG_PROTOCOL_SERVICE_ACCEPTED;
}

static void plan_handler(void *context,
                         const yg_protocol_plan_request_t *request,
                         yg_protocol_plan_reply_t *reply)
{
    (void)context;
    ++plan_calls;
    reply->parent_ticket = 0x12345678U;
    reply->requested_steps = 3U;
    reply->completed_steps = request->operation == YG_PROTOCOL_PLAN_STATUS ? 3U : 0U;
    reply->step = request->operation == YG_PROTOCOL_PLAN_STATUS ? 7U : 1U;
    reply->phase = request->operation == YG_PROTOCOL_PLAN_STATUS ? 2U : 1U;
    reply->effects = request->operation == YG_PROTOCOL_PLAN_STATUS ? 2U : 0U;
    reply->status = request->operation == YG_PROTOCOL_PLAN_STATUS ? YG_PROTOCOL_SERVICE_OK
                                                                  : YG_PROTOCOL_SERVICE_ACCEPTED;
}

static void safety_handler(void *context,
                           const yg_protocol_motor_request_t *request,
                           yg_protocol_service_reply_t *reply)
{
    (void)context;
    ++motor_calls;
    if (request->operation == YG_PROTOCOL_MOTOR_STOP)
    {
        last_stop_source = request->source_node;
    }
    reply->status = request->operation == YG_PROTOCOL_MOTOR_CLEAR_FAULT && !clear_allowed
                        ? YG_PROTOCOL_SERVICE_DENIED
                        : YG_PROTOCOL_SERVICE_OK;
}

static void bind_safety_fixture(yg_protocol_readonly_device_status_t *status)
{
    const yg_protocol_run_service_t runs = {NULL, run_handler};
    const yg_protocol_plan_service_t plans = {NULL, plan_handler};
    const yg_protocol_motor_service_t motors = {NULL, safety_handler};
    assert(YgProtocolLink_Init(7U));
    assert(YgProtocolLink_BindDeviceStatusSource(status));
    assert(YgProtocolLink_BindRunService(&runs));
    assert(YgProtocolLink_BindPlanService(&plans));
    assert(YgProtocolLink_BindMotorService(&motors));
    sent_count = 0U;
    motor_calls = 0U;
    plan_calls = 0U;
    run_calls = 0U;
    clear_allowed = true;
    last_stop_source = 0U;
}

static void safety_boundaries_revoke_old_starts(void)
{
    const uint64_t epoch = UINT64_C(0x1122334455667788);
    uint8_t plan_payload[YG_PROTOCOL_PLAN_START_SIZE] = {1U, 1U};
    uint8_t run_payload[YG_PROTOCOL_RUN_SESSION_REQUEST_SIZE] = {1U, 1U, 2U};
    uint8_t disable_payload[YG_PROTOCOL_MOTOR_ENABLE_REQUEST_SIZE] = {0};
    yg_protocol_readonly_device_status_t status = {
        .boot_epoch = epoch, .role = YG_PROTOCOL_DEVICE_ROLE_APP, .epoch_valid = true};
    yg_protocol_message_t request = {.version = YG_PROTOCOL_VERSION,
                                     .flags = YG_PROTOCOL_FLAGS_ACK_REQUEST,
                                     .source_node = 2U,
                                     .destination_node = 7U,
                                     .message_type = YG_PROTOCOL_JOB_TYPE_PLAN,
                                     .sequence = 10U,
                                     .payload_length = sizeof(plan_payload),
                                     .payload = plan_payload};
    yg_protocol_message_t reply;
    put_u32(plan_payload + 4U, 3U);
    put_u64(plan_payload + 12U, epoch);
    put_u32(run_payload + 4U, 100U);
    put_u64(run_payload + 12U, epoch);

    bind_safety_fixture(&status);
    reply = exchange_message(&request, 1U);
    assert(reply.payload[0] == 1U && plan_calls == 1U);
    request.message_type = YG_PROTOCOL_MOTOR_TYPE_STOP;
    request.sequence = 11U;
    request.payload_length = 0U;
    request.payload = NULL;
    reply = exchange_message(&request, 2U);
    assert(reply.payload[0] == 0U && motor_calls == 1U);
    request.message_type = YG_PROTOCOL_JOB_TYPE_PLAN;
    request.sequence = 10U;
    request.payload_length = sizeof(plan_payload);
    request.payload = plan_payload;
    reply = exchange_message(&request, 3U);
    assert(reply.payload[0] == 4U && plan_calls == 1U);

    bind_safety_fixture(&status);
    request.sequence = 20U;
    reply = exchange_message(&request, 4U);
    assert(reply.payload[0] == 1U && plan_calls == 1U);
    request.message_type = YG_PROTOCOL_MOTOR_TYPE_ENABLE;
    request.sequence = 21U;
    request.payload_length = sizeof(disable_payload);
    request.payload = disable_payload;
    reply = exchange_message(&request, 5U);
    assert(reply.payload[0] == 0U && motor_calls == 1U);
    request.message_type = YG_PROTOCOL_JOB_TYPE_PLAN;
    request.sequence = 20U;
    request.payload_length = sizeof(plan_payload);
    request.payload = plan_payload;
    reply = exchange_message(&request, 6U);
    assert(reply.payload[0] == 4U && plan_calls == 1U);

    bind_safety_fixture(&status);
    request.sequence = 30U;
    reply = exchange_message(&request, 7U);
    assert(reply.payload[0] == 1U && plan_calls == 1U);
    clear_allowed = false;
    request.message_type = YG_PROTOCOL_MOTOR_TYPE_CLEAR_FAULT;
    request.sequence = 31U;
    request.payload_length = 0U;
    request.payload = NULL;
    reply = exchange_message(&request, 8U);
    assert(reply.payload[0] == 5U);
    request.message_type = YG_PROTOCOL_JOB_TYPE_PLAN;
    request.sequence = 30U;
    request.payload_length = sizeof(plan_payload);
    request.payload = plan_payload;
    reply = exchange_message(&request, 9U);
    assert(reply.payload[0] == 1U && plan_calls == 1U);
    clear_allowed = true;
    request.message_type = YG_PROTOCOL_MOTOR_TYPE_CLEAR_FAULT;
    request.sequence = 32U;
    request.payload_length = 0U;
    request.payload = NULL;
    reply = exchange_message(&request, 10U);
    assert(reply.payload[0] == 0U);
    request.message_type = YG_PROTOCOL_JOB_TYPE_PLAN;
    request.sequence = 30U;
    request.payload_length = sizeof(plan_payload);
    request.payload = plan_payload;
    reply = exchange_message(&request, 11U);
    assert(reply.payload[0] == 4U && plan_calls == 1U);

    bind_safety_fixture(&status);
    request.sequence = 40U;
    reply = exchange_message(&request, 12U);
    assert(reply.payload[0] == 1U && plan_calls == 1U);
    request.message_type = YG_PROTOCOL_MOTOR_TYPE_RUN_SESSION;
    request.payload_length = sizeof(run_payload);
    request.payload = run_payload;
    reply = exchange_message(&request, 13U);
    assert(reply.payload[0] == 1U && run_calls == 1U);
    request.message_type = YG_PROTOCOL_MOTOR_TYPE_STOP;
    request.source_node = 3U;
    request.sequence = 1000U;
    request.payload_length = 0U;
    request.payload = NULL;
    reply = exchange_message(&request, 14U);
    assert(reply.payload[0] == 0U && last_stop_source == 3U);
    request.source_node = 2U;
    request.sequence = 41U;
    request.message_type = YG_PROTOCOL_JOB_TYPE_PLAN;
    request.payload_length = sizeof(plan_payload);
    request.payload = plan_payload;
    reply = exchange_message(&request, 15U);
    assert(reply.payload[0] == 1U && plan_calls == 2U);
    request.message_type = YG_PROTOCOL_MOTOR_TYPE_RUN_SESSION;
    request.payload_length = sizeof(run_payload);
    request.payload = run_payload;
    reply = exchange_message(&request, 16U);
    assert(reply.payload[0] == 1U && run_calls == 2U);
}

static yg_protocol_message_t exchange_message(const yg_protocol_message_t *request, uint32_t now_ms)
{
    yg_protocol_transfer_frame_t frame;
    static yg_protocol_transfer_frame_t reply_frame;
    CommHwCanFrame received = {0};
    yg_protocol_message_t reply;
    assert(yg_protocol_canfd_pack(request, 3U, &frame) == YG_PROTOCOL_OK);
    received.identifier = frame.identifier;
    received.length = frame.length;
    received.extended = true;
    received.fd = true;
    received.bitrate_switch = true;
    memcpy(received.data, frame.data, frame.length);
    sent = false;
    assert(YgProtocolLink_OnRxFrame(&received));
    YgProtocolLink_Service(now_ms);
    assert(sent);
    reply_frame = (yg_protocol_transfer_frame_t){.identifier = sent_frame.identifier,
                                                 .length = sent_frame.length,
                                                 .extended = true,
                                                 .fd = true,
                                                 .bitrate_switch = true};
    memcpy(reply_frame.data, sent_frame.data, sent_frame.length);
    assert(yg_protocol_canfd_unpack(&reply_frame, request->source_node, &reply) == YG_PROTOCOL_OK);
    return reply;
}

static void lifecycle_epoch_and_retry_contract(void)
{
    const uint64_t epoch = UINT64_C(0x1122334455667788);
    uint8_t run_payload[YG_PROTOCOL_RUN_SESSION_REQUEST_SIZE] = {1U, 1U, 2U};
    uint8_t plan_payload[YG_PROTOCOL_PLAN_START_SIZE] = {1U, 1U};
    yg_protocol_readonly_device_status_t status = {
        .boot_epoch = epoch, .role = YG_PROTOCOL_DEVICE_ROLE_APP, .epoch_valid = false};
    yg_protocol_run_service_t runs = {NULL, run_handler};
    yg_protocol_plan_service_t plans = {NULL, plan_handler};
    yg_protocol_message_t request = {.version = YG_PROTOCOL_VERSION,
                                     .flags = YG_PROTOCOL_FLAGS_ACK_REQUEST,
                                     .source_node = 2U,
                                     .destination_node = 7U,
                                     .message_type = YG_PROTOCOL_MOTOR_TYPE_RUN_SESSION,
                                     .sequence = 10U,
                                     .payload_length = sizeof(run_payload),
                                     .payload = run_payload};
    yg_protocol_message_t reply;
    assert(YgProtocolLink_Init(7U));
    assert(YgProtocolLink_BindRunService(&runs));
    assert(YgProtocolLink_BindPlanService(&plans));
    assert(YgProtocolLink_BindDeviceStatusSource(&status));
    sent_count = 0U;
    run_calls = 0U;
    run_progress_calls = 0U;
    plan_calls = 0U;
    put_u32(run_payload + 4U, 100U);
    put_u64(run_payload + 12U, epoch);
    reply = exchange_message(&request, 1U);
    assert(reply.payload[0] == 2U && run_calls == 0U);
    status.epoch_valid = true;
    reply = exchange_message(&request, 2U);
    assert(reply.payload[0] == 1U && run_calls == 1U);
    reply = exchange_message(&request, 3U);
    assert(reply.payload[0] == 0U && run_calls == 1U && run_progress_calls == 1U);
    put_u32(run_payload + 4U, 101U);
    reply = exchange_message(&request, 4U);
    assert(reply.payload[0] == 4U && run_calls == 1U);
    put_u32(run_payload + 4U, 100U);
    run_payload[1] = YG_PROTOCOL_RUN_SESSION_RENEW;
    run_payload[2] = 0U;
    put_u16(run_payload + 8U, 10U);
    request.sequence = 11U;
    reply = exchange_message(&request, 5U);
    assert(reply.payload[0] == 0U && run_calls == 2U);
    reply = exchange_message(&request, 6U);
    assert(reply.payload[0] == 0U && run_calls == 2U);
    request.message_type = YG_PROTOCOL_MOTOR_TYPE_STOP;
    request.sequence = 12U;
    request.payload_length = 0U;
    request.payload = NULL;
    reply = exchange_message(&request, 7U);
    assert(reply.payload[0] == 2U);
    request.message_type = YG_PROTOCOL_MOTOR_TYPE_RUN_SESSION;
    request.sequence = 10U;
    request.payload_length = sizeof(run_payload);
    request.payload = run_payload;
    run_payload[1] = YG_PROTOCOL_RUN_SESSION_START;
    run_payload[2] = 2U;
    put_u16(run_payload + 8U, 0U);
    reply = exchange_message(&request, 8U);
    assert(reply.payload[0] == 4U && run_calls == 2U);
    put_u32(plan_payload + 4U, 3U);
    put_u64(plan_payload + 12U, epoch);
    request.message_type = YG_PROTOCOL_JOB_TYPE_PLAN;
    request.sequence = 20U;
    request.payload_length = sizeof(plan_payload);
    request.payload = plan_payload;
    reply = exchange_message(&request, 9U);
    assert(reply.payload_length == YG_PROTOCOL_PLAN_REPLY_SIZE && reply.payload[0] == 1U &&
           plan_calls == 1U);
    reply = exchange_message(&request, 10U);
    assert(reply.payload[0] == 1U && plan_calls == 1U);
    put_u32(plan_payload + 4U, 1U);
    reply = exchange_message(&request, 11U);
    assert(reply.payload[0] == 4U && plan_calls == 1U);
    request.sequence = 23U;
    put_u32(plan_payload + 4U, 1U << 6U);
    reply = exchange_message(&request, 11U);
    assert(reply.payload[0] == 2U && plan_calls == 1U);
    status.epoch_valid = false;
    plan_payload[1] = YG_PROTOCOL_PLAN_STATUS;
    put_u32(plan_payload + 4U, 0x12345678U);
    request.sequence = 21U;
    request.payload_length = YG_PROTOCOL_PLAN_QUERY_SIZE;
    reply = exchange_message(&request, 12U);
    assert(reply.payload[0] == 0U && reply.payload[16] == 3U && reply.payload[20] == 7U &&
           reply.payload[21] == 2U && reply.payload[22] == 2U && plan_calls == 2U);
    plan_payload[1] = YG_PROTOCOL_PLAN_CANCEL;
    request.sequence = 22U;
    reply = exchange_message(&request, 13U);
    assert(reply.payload[0] == 1U && plan_calls == 3U);
}

static void control_reply_and_feedback(void)
{
    uint8_t payload[YG_PROTOCOL_MOTOR_CONTROL_REQUEST_SIZE] = {3U, 0U, 0U, 0U, 0xE8U, 0x03U};
    yg_protocol_message_t request = {.version = YG_PROTOCOL_VERSION,
                                     .flags = YG_PROTOCOL_FLAGS_ACK_REQUEST,
                                     .source_node = 2U,
                                     .destination_node = 7U,
                                     .message_type = YG_PROTOCOL_MOTOR_TYPE_CONTROL,
                                     .sequence = 41U,
                                     .payload_length = sizeof(payload),
                                     .payload = payload};
    yg_protocol_transfer_frame_t packed, response_frame;
    yg_protocol_message_t response;
    CommHwCanFrame received = {0};
    assert(YgProtocolLink_Init(7U));
    assert(yg_protocol_canfd_pack(&request, 2U, &packed) == YG_PROTOCOL_OK);
    received.identifier = packed.identifier;
    received.length = packed.length;
    received.extended = true;
    received.fd = true;
    received.bitrate_switch = true;
    memcpy(received.data, packed.data, packed.length);
    sent = false;
    assert(YgProtocolLink_OnRxFrame(&received));
    YgProtocolLink_Service(1U);
    assert(sent);
    response_frame = (yg_protocol_transfer_frame_t){0};
    response_frame.identifier = sent_frame.identifier;
    response_frame.length = sent_frame.length;
    response_frame.extended = true;
    response_frame.fd = true;
    response_frame.bitrate_switch = true;
    memcpy(response_frame.data, sent_frame.data, sent_frame.length);
    assert(yg_protocol_canfd_unpack(&response_frame, 2U, &response) == YG_PROTOCOL_OK);
    assert(response.message_type == YG_PROTOCOL_MOTOR_TYPE_CONTROL && response.sequence == 41U);
    assert(response.payload_length == YG_PROTOCOL_MOTOR_REPLY_SIZE && response.payload[0] == 2U);

    sent = false;
    YgProtocolLink_Service(2U);
    assert(sent);
    response_frame.identifier = sent_frame.identifier;
    response_frame.length = sent_frame.length;
    memcpy(response_frame.data, sent_frame.data, sent_frame.length);
    assert(yg_protocol_canfd_unpack(&response_frame, 2U, &response) == YG_PROTOCOL_OK);
    assert(response.message_type == YG_PROTOCOL_READONLY_TYPE_MOTION_FEEDBACK &&
           response.payload_length == YG_PROTOCOL_MOTOR_FEEDBACK_PAYLOAD_SIZE &&
           response.payload[0] == 2U && response.payload[2] == 41U && response.payload[42] == 7U);
    assert(response.payload[24] == 0U && response.payload[25] == 0U);

    /* 同序号重传重发回复与反馈，但不得再次提交目标。 */
    {
        yg_protocol_motor_service_t service = {NULL, control_handler};
        yg_protocol_motor_status_source_t source = {
            .control_progress = {.state = MOTOR_CONTROL_PROGRESS_PENDING,
                                 .result = MOTOR_CONTROL_ACCEPTED,
                                 .source_node = 2U,
                                 .sequence = 41U}};
        assert(YgProtocolLink_Init(7U));
        assert(YgProtocolLink_BindMotorService(&service));
        assert(YgProtocolLink_BindMotorStatusSource(&source));
        motor_calls = 0U;
        sent = false;
        assert(YgProtocolLink_OnRxFrame(&received));
        YgProtocolLink_Service(3U);
        assert(motor_calls == 1U);
        assert(sent && sent_frame.data[6] == YG_PROTOCOL_MOTOR_TYPE_CONTROL &&
               sent_frame.data[16] == 1U);
        sent = false;
        YgProtocolLink_Service(4U);
        assert(!sent); /* ACK 已受理，所有者尚未应用，不提前发送成功反馈。 */
        source.control_progress.state = MOTOR_CONTROL_PROGRESS_APPLIED;
        YgProtocolLink_Service(5U);
        assert(sent && sent_frame.data[6] == YG_PROTOCOL_READONLY_TYPE_MOTION_FEEDBACK &&
               sent_frame.data[16] == 0U && sent_frame.data[18] == 41U);
        sent = false;
        request.flags |= YG_PROTOCOL_FLAGS_RETRY;
        assert(yg_protocol_canfd_pack(&request, 2U, &packed) == YG_PROTOCOL_OK);
        received.length = packed.length;
        memcpy(received.data, packed.data, packed.length);
        assert(YgProtocolLink_OnRxFrame(&received));
        YgProtocolLink_Service(6U);
        assert(motor_calls == 1U);
        assert(sent && sent_frame.data[6] == YG_PROTOCOL_MOTOR_TYPE_CONTROL &&
               sent_frame.data[16] == 1U && sent_frame.data[18] == 0U);
        sent = false;
        YgProtocolLink_Service(7U);
        assert(sent && sent_frame.data[6] == YG_PROTOCOL_READONLY_TYPE_MOTION_FEEDBACK &&
               sent_frame.data[16] == 0U && sent_frame.data[18] == 41U);
    }
}

static void enable_route(void)
{
    uint8_t payload[YG_PROTOCOL_MOTOR_ENABLE_REQUEST_SIZE] = {1U};
    yg_protocol_message_t request = {.version = YG_PROTOCOL_VERSION,
                                     .flags = YG_PROTOCOL_FLAGS_ACK_REQUEST,
                                     .source_node = 2U,
                                     .destination_node = 7U,
                                     .message_type = YG_PROTOCOL_MOTOR_TYPE_ENABLE,
                                     .sequence = 40U,
                                     .payload_length = sizeof(payload),
                                     .payload = payload};
    yg_protocol_transfer_frame_t frame;
    CommHwCanFrame received = {0};
    assert(YgProtocolLink_Init(7U));
    assert(yg_protocol_canfd_pack(&request, 2U, &frame) == YG_PROTOCOL_OK);
    received.identifier = frame.identifier;
    received.length = frame.length;
    received.extended = true;
    received.fd = true;
    received.bitrate_switch = true;
    memcpy(received.data, frame.data, frame.length);
    sent = false;
    assert(YgProtocolLink_OnRxFrame(&received));
    YgProtocolLink_Service(1U);
    assert(sent && sent_frame.data[6] == YG_PROTOCOL_MOTOR_TYPE_ENABLE &&
           sent_frame.data[8] == 40U && sent_frame.data[16] == 2U);
}

static void power_handler(void *context,
                          const yg_protocol_motor_request_t *request,
                          yg_protocol_service_reply_t *reply)
{
    (void)context;
    ++motor_calls;
    power_operation = request->operation;
    reply->status = request->operation == YG_PROTOCOL_MOTOR_ENABLE ? YG_PROTOCOL_SERVICE_UNSUPPORTED
                                                                   : YG_PROTOCOL_SERVICE_OK;
    reply->detail = 0x1234U;
}

static void enqueue_request(const yg_protocol_message_t *request)
{
    yg_protocol_transfer_frame_t packed;
    CommHwCanFrame received = {0};
    assert(yg_protocol_canfd_pack(request, 2U, &packed) == YG_PROTOCOL_OK);
    received.identifier = packed.identifier;
    received.length = packed.length;
    received.extended = true;
    received.fd = true;
    received.bitrate_switch = true;
    memcpy(received.data, packed.data, packed.length);
    assert(YgProtocolLink_OnRxFrame(&received));
}

static void completed_control_handler(void *context,
                                      const yg_protocol_motor_request_t *request,
                                      yg_protocol_service_reply_t *reply)
{
    (void)context;
    assert(request->operation == YG_PROTOCOL_MOTOR_SET_CONTROL);
    ++motor_calls;
    reply->status = YG_PROTOCOL_SERVICE_OK;
}

/* 历史槽覆盖后旧序号只能拒绝；仍在窗口内的重发须重放原结果。 */
static void control_history_window(void)
{
    uint8_t payload[YG_PROTOCOL_MOTOR_CONTROL_REQUEST_SIZE] = {3U, 0U, 0U, 0U, 0xE8U, 0x03U};
    yg_protocol_motor_service_t service = {NULL, completed_control_handler};
    yg_protocol_message_t request = {.version = YG_PROTOCOL_VERSION,
                                     .flags = YG_PROTOCOL_FLAGS_ACK_REQUEST,
                                     .source_node = 2U,
                                     .destination_node = 7U,
                                     .message_type = YG_PROTOCOL_MOTOR_TYPE_CONTROL,
                                     .payload_length = sizeof(payload),
                                     .payload = payload};
    assert(YgProtocolLink_Init(7U));
    assert(YgProtocolLink_BindMotorService(&service));
    motor_calls = 0U;
    tx_blocked = false;
    for (uint16_t index = 0U; index <= 32U; ++index)
    {
        request.sequence = (uint16_t)(100U + index);
        sent = false;
        sent_count = 0U;
        enqueue_request(&request);
        YgProtocolLink_Service((uint32_t)index * 2U);
        assert(sent && sent_frame.data[16] == YG_PROTOCOL_SERVICE_OK);
        sent = false;
        YgProtocolLink_Service((uint32_t)index * 2U + 1U);
        assert(sent);
    }
    assert(motor_calls == 33U);

    request.sequence = 100U;
    sent = false;
    sent_count = 0U;
    enqueue_request(&request);
    YgProtocolLink_Service(100U);
    assert(sent && sent_frame.data[16] == 4U);
    assert(motor_calls == 33U);
    sent = false;
    YgProtocolLink_Service(101U);

    request.sequence = 132U;
    sent = false;
    sent_count = 0U;
    enqueue_request(&request);
    YgProtocolLink_Service(102U);
    assert(sent && sent_frame.data[16] == YG_PROTOCOL_SERVICE_OK);
    assert(motor_calls == 33U);
}

static void start_handler(void *context,
                          const yg_protocol_motor_request_t *request,
                          yg_protocol_service_reply_t *reply)
{
    (void)context;
    if (request->operation == YG_PROTOCOL_MOTOR_STOP)
    {
        start_stop_barrier = request->sequence;
        reply->status = YG_PROTOCOL_SERVICE_OK;
        return;
    }
    assert(request->operation == YG_PROTOCOL_MOTOR_START_CONTROL);
    assert(request->source_node == 2U && request->sequence == 0x1001U && request->mode == 2U);
    ++motor_calls;
    reply->status = start_stop_barrier == 0U ? YG_PROTOCOL_SERVICE_ACCEPTED
                                             : YG_PROTOCOL_SERVICE_INVALID_ARGUMENT;
    reply->detail = start_stop_barrier == 0U ? 0U : 0x0101U;
}

static void start_route_contract(void)
{
    static const uint8_t request_bytes[24] = {0x5A, 0xA5, 0x01, 0x20, 0x02, 0x01, 0x75, 0x00,
                                              0x01, 0x10, 0x04, 0x00, 0x00, 0x00, 0x00, 0x82,
                                              0x02, 0x00, 0x00, 0x00, 0x99, 0xF6, 0x00, 0x00};
    static const uint8_t accepted_bytes[24] = {0x5A, 0xA5, 0x01, 0x10, 0x01, 0x02, 0x75, 0x00,
                                               0x01, 0x10, 0x04, 0x00, 0x00, 0x00, 0x00, 0x8E,
                                               0x01, 0x00, 0x00, 0x00, 0x20, 0x12, 0x00, 0x00};
    static const uint8_t retry_bytes[24] = {0x5A, 0xA5, 0x01, 0x24, 0x02, 0x01, 0x75, 0x00,
                                            0x01, 0x10, 0x04, 0x00, 0x00, 0x00, 0x00, 0x70,
                                            0x02, 0x00, 0x00, 0x00, 0xE7, 0x3A, 0x00, 0x00};
    static const uint8_t stop_bytes[20] = {0x5A, 0xA5, 0x01, 0x20, 0x02, 0x01, 0x6E,
                                           0x00, 0x04, 0x10, 0x00, 0x00, 0x00, 0x00,
                                           0x00, 0xA7, 0xE3, 0x14, 0x00, 0x00};
    static const uint8_t stale_bytes[24] = {0x5A, 0xA5, 0x01, 0x10, 0x01, 0x02, 0x75, 0x00,
                                            0x01, 0x10, 0x04, 0x00, 0x00, 0x00, 0x00, 0x8E,
                                            0x04, 0x00, 0x01, 0x01, 0x86, 0x62, 0x00, 0x00};
    uint8_t payload[4] = {2U};
    yg_protocol_message_t request = {.version = YG_PROTOCOL_VERSION,
                                     .flags = YG_PROTOCOL_FLAGS_ACK_REQUEST,
                                     .source_node = 2U,
                                     .destination_node = 1U,
                                     .message_type = YG_PROTOCOL_MOTOR_TYPE_START,
                                     .sequence = 0x1001U,
                                     .payload_length = sizeof(payload),
                                     .payload = payload};
    const yg_protocol_motor_service_t service = {NULL, start_handler};
    yg_protocol_transfer_frame_t packed;
    assert(YgProtocolLink_Init(1U));
    assert(YgProtocolLink_BindMotorService(&service));
    start_stop_barrier = 0U;
    sent_count = 0U;
    assert(yg_protocol_canfd_pack(&request, 2U, &packed) == YG_PROTOCOL_OK);
    assert(packed.identifier == 0x08EF0102U && packed.length == 24U);
    assert(memcmp(packed.data, request_bytes, sizeof(request_bytes)) == 0);
    motor_calls = 0U;
    sent = false;
    enqueue_request(&request);
    YgProtocolLink_Service(1U);
    assert(motor_calls == 1U && sent && sent_frame.identifier == 0x0CEF0201U);
    assert(sent_frame.length == 24U);
    assert(memcmp(sent_frame.data, accepted_bytes, sizeof(accepted_bytes)) == 0);

    sent = false;
    request.source_node = 3U;
    enqueue_request(&request);
    YgProtocolLink_Service(2U);
    assert(sent && sent_frame.data[16] == 7U && motor_calls == 1U);

    sent = false;
    request.source_node = 2U;
    request.payload_length = 3U;
    enqueue_request(&request);
    YgProtocolLink_Service(3U);
    assert(sent && sent_frame.data[16] == 3U && motor_calls == 1U);

    sent = false;
    request.payload_length = 4U;
    payload[1] = 1U;
    enqueue_request(&request);
    YgProtocolLink_Service(4U);
    assert(sent && sent_frame.data[16] == 4U && motor_calls == 1U);

    sent = false;
    payload[1] = 0U;
    payload[0] = 3U;
    enqueue_request(&request);
    YgProtocolLink_Service(5U);
    assert(sent && sent_frame.data[16] == 2U && motor_calls == 1U);

    /* 新会话中 STOP 先抵达，之后重发的旧 START 只得到陈旧身份结果。 */
    assert(YgProtocolLink_Init(1U));
    assert(YgProtocolLink_BindMotorService(&service));
    request.message_type = YG_PROTOCOL_MOTOR_TYPE_STOP;
    request.sequence = 0x1004U;
    request.payload_length = 0U;
    request.payload = NULL;
    assert(yg_protocol_canfd_pack(&request, 2U, &packed) == YG_PROTOCOL_OK);
    assert(packed.length == 20U && memcmp(packed.data, stop_bytes, sizeof(stop_bytes)) == 0);
    sent = false;
    enqueue_request(&request);
    YgProtocolLink_Service(6U);
    assert(sent && start_stop_barrier == 0x1004U);
    request.message_type = YG_PROTOCOL_MOTOR_TYPE_START;
    request.sequence = 0x1001U;
    request.flags = YG_PROTOCOL_FLAGS_ACK_REQUEST | YG_PROTOCOL_FLAGS_RETRY;
    request.payload_length = sizeof(payload);
    request.payload = payload;
    payload[0] = 2U;
    assert(yg_protocol_canfd_pack(&request, 2U, &packed) == YG_PROTOCOL_OK);
    assert(packed.length == 24U && memcmp(packed.data, retry_bytes, sizeof(retry_bytes)) == 0);
    sent = false;
    enqueue_request(&request);
    YgProtocolLink_Service(7U);
    assert(sent && sent_frame.length == 24U);
    assert(memcmp(sent_frame.data, stale_bytes, sizeof(stale_bytes)) == 0);
}

/* 固定合并前的权限/长度检查顺序及结果；使能只返回模拟服务值，不产生功率动作。 */
static void power_route_contract(void)
{
    static const uint8_t enabled[] = {1U, 0U, 0U, 0U};
    static const uint8_t disabled[] = {0U, 0U, 0U, 0U};
    static const uint8_t invalid[] = {2U, 0U, 0U, 0U};
    static const struct
    {
        uint16_t type;
        uint8_t source;
        const uint8_t *payload;
        uint32_t length;
        bool response;
        uint8_t result;
        bool dispatched;
        yg_protocol_motor_operation_t operation;
    } cases[] = {
        {101U, 2U, disabled, 4U, true, 0U, true, YG_PROTOCOL_MOTOR_DISABLE},
        {101U, 2U, enabled, 4U, true, 2U, true, YG_PROTOCOL_MOTOR_ENABLE},
        {110U, 2U, NULL, 0U, true, 0U, true, YG_PROTOCOL_MOTOR_STOP},
        {101U, 1U, enabled, 1U, true, 7U, false, YG_PROTOCOL_MOTOR_STOP},
        {101U, 2U, enabled, 1U, true, 3U, false, YG_PROTOCOL_MOTOR_STOP},
        {101U, 2U, invalid, 4U, true, 4U, false, YG_PROTOCOL_MOTOR_STOP},
        {110U, 1U, NULL, 0U, true, 0U, true, YG_PROTOCOL_MOTOR_STOP},
        {110U, 1U, enabled, 1U, false, 0U, false, YG_PROTOCOL_MOTOR_STOP},
        {110U, 2U, enabled, 1U, false, 0U, false, YG_PROTOCOL_MOTOR_STOP},
    };
    const yg_protocol_motor_service_t service = {NULL, power_handler};
    for (size_t index = 0U; index < sizeof(cases) / sizeof(cases[0]); ++index)
    {
        yg_protocol_message_t request = {.version = YG_PROTOCOL_VERSION,
                                         .flags = YG_PROTOCOL_FLAGS_ACK_REQUEST,
                                         .source_node = cases[index].source,
                                         .destination_node = 7U,
                                         .message_type = cases[index].type,
                                         .sequence = (uint16_t)(100U + index),
                                         .payload_length = cases[index].length,
                                         .payload = cases[index].payload};
        assert(YgProtocolLink_Init(7U));
        assert(YgProtocolLink_BindMotorService(&service));
        motor_calls = 0U;
        sent_count = 0U;
        sent = false;
        enqueue_request(&request);
        YgProtocolLink_Service(0U);
        assert(sent == cases[index].response);
        assert(motor_calls == (cases[index].dispatched ? 1U : 0U));
        if (cases[index].dispatched)
        {
            assert(power_operation == cases[index].operation);
        }
        if (sent)
        {
            assert(sent_frame.length == 24U && sent_frame.data[3] == YG_PROTOCOL_FLAGS_RESPONSE);
            assert(sent_frame.data[4] == 7U && sent_frame.data[5] == cases[index].source);
            assert(sent_frame.data[6] == cases[index].type &&
                   sent_frame.data[8] == request.sequence);
            assert(sent_frame.data[10] == 4U && sent_frame.data[16] == cases[index].result);
            assert(sent_frame.data[18] == (cases[index].dispatched ? 0x34U : 0U));
            assert(sent_frame.data[19] == (cases[index].dispatched ? 0x12U : 0U));
        }
    }
}

/* 解绑或重新初始化后不得保留旧快照，查询必须回退为无新鲜数据。 */
static void status_binding_reset(void)
{
    const yg_protocol_motor_status_source_t source = {
        .sample_available = true, .state = 6U, .mode = 3U};
    const yg_protocol_message_t request = {.version = YG_PROTOCOL_VERSION,
                                           .flags = YG_PROTOCOL_FLAGS_ACK_REQUEST,
                                           .source_node = 2U,
                                           .destination_node = 7U,
                                           .message_type =
                                               YG_PROTOCOL_READONLY_TYPE_GET_MOTOR_STATE,
                                           .sequence = 199U};
    assert(YgProtocolLink_Init(7U));
    assert(YgProtocolLink_BindMotorStatusSource(&source));
    sent_count = 0U;
    sent = false;
    enqueue_request(&request);
    YgProtocolLink_Service(0U);
    assert(sent && sent_frame.data[16] == 0U && sent_frame.data[59] == 6U);
    for (unsigned reset = 0U; reset < 2U; ++reset)
    {
        assert(YgProtocolLink_BindMotorStatusSource(&source));
        if (reset == 0U)
        {
            assert(YgProtocolLink_BindMotorStatusSource(NULL));
        }
        else
        {
            assert(YgProtocolLink_Init(7U));
        }
        sent = false;
        enqueue_request(&request);
        YgProtocolLink_Service(0U);
        assert(sent && sent_frame.data[16] == 6U && sent_frame.data[40] == 0U &&
               sent_frame.data[59] == 0U && sent_frame.data[60] == 0U);
    }
}

/* 运行时只启动一次；参数桥更新不改变冻结地址，重复初始化不丢弃在途请求。 */
static void runtime_start_boundary(void)
{
    yg_protocol_message_t stop = {.version = YG_PROTOCOL_VERSION,
                                  .flags = YG_PROTOCOL_FLAGS_ACK_REQUEST,
                                  .source_node = 2U,
                                  .destination_node = 7U,
                                  .message_type = YG_PROTOCOL_MOTOR_TYPE_STOP,
                                  .sequence = 200U};
    YgProtocolRuntime_Service(1000U);
    assert(can_starts == 0U && bus_off_services == 0U);
    assert(!YgProtocolRuntime_Init(255U));
    assert(can_starts == 0U && !YgProtocolLink_IsReady());
    assert(YgProtocolRuntime_Init(7U));
    enqueue_request(&stop);
    assert(!YgProtocolRuntime_Init(9U));
    sent = false;
    sent_count = 0U;
    YgProtocolRuntime_Service(999U);
    assert(can_starts == 0U && bus_off_services == 0U && !sent);
    assert(YgProtocolRuntime_Start());
    assert(YgProtocolRuntime_Start());
    YgProtocolRuntime_Service(999U);
    assert(can_starts == 1U && bus_off_services == 0U && sent);
    /* 重新初始化撤销旧服务；仍从冻结的节点 7 返回未绑定结果。 */
    assert(sent_frame.data[4] == 7U && sent_frame.data[16] == 2U);
    sent = false;
    YgProtocolRuntime_Service(1000U);
    YgProtocolRuntime_Service(1999U);
    assert(bus_off_services == 1U);
    YgProtocolRuntime_Service(2000U);
    assert(bus_off_services == 2U);
}

static unsigned confirm_calls;
static bool confirm_allowed;
static void boot_request(void *context, const BootEntryRequest *request, BootEntryReply *reply)
{
    (void)context;
    (void)request;
    (void)reply;
    assert(false); /* 本夹具只允许57，不得转为停机交接。 */
}
static void boot_tx(void *context, uint64_t ticket, bool success)
{
    (void)context;
    (void)ticket;
    (void)success;
    assert(false);
}
static bool boot_confirm(void *context, uint32_t version, uint32_t crc)
{
    (void)context;
    ++confirm_calls;
    assert(version == 0x10000U && crc == 7U);
    return confirm_allowed;
}
/* 57只接收主机节点的8B身份，维护页面保持只读与初始化清空。 */
static void boot_confirm_and_info_contract(void)
{
    const BootEntryService service = {NULL, boot_request, boot_tx, boot_confirm};
    uint8_t identity[8] = {0U}, page[32] = {0U}, page_request[12] = {0U};
    yg_protocol_message_t request = {.version = YG_PROTOCOL_VERSION,
                                     .flags = YG_PROTOCOL_FLAGS_ACK_REQUEST,
                                     .source_node = 2U,
                                     .destination_node = 7U,
                                     .message_type = YG_PROTOCOL_BOOT_TYPE_CONFIRM,
                                     .sequence = 211U,
                                     .payload = identity,
                                     .payload_length = 8U};
    put_u32(identity, 0x10000U);
    put_u32(identity + 4U, 7U);
    for (unsigned scenario = 0U; scenario < 5U; ++scenario)
    {
        BootEntryService bound = service;
        assert(YgProtocolLink_Init(7U));
        bound.confirm = scenario == 4U ? NULL : boot_confirm;
        assert(YgProtocolLink_BindBootService(&bound));
        confirm_calls = sent_count = 0U;
        sent = false;
        confirm_allowed = scenario == 0U;
        request.source_node = scenario == 2U ? 1U : 2U;
        request.payload_length = scenario == 3U ? 7U : 8U;
        enqueue_request(&request);
        YgProtocolLink_Service(1U);
        const uint8_t results[5] = {0U, 5U, 7U, 3U, 2U};
        assert(sent && sent_frame.data[16] == results[scenario]);
        assert(confirm_calls == (scenario < 2U ? 1U : 0U));
    }
    yg_protocol_readonly_info_page0_t info = {0};
    yg_protocol_readonly_caps_page0_t caps = {0};
    assert(YgProtocolLink_Init(7U));
    assert(YgProtocolLink_SetDeviceDescription(&info, &caps));
    request.source_node = 2U;
    request.message_type = 1U;
    request.payload = page_request;
    request.payload_length = 12U;
    assert(!YgProtocolLink_SetMaintenanceInfoPage(0U, page));
    assert(!YgProtocolLink_SetMaintenanceInfoPage(3U, page));
    for (unsigned index = 1U; index <= 2U; ++index)
    {
        memset(page, (int)index, sizeof(page));
        assert(YgProtocolLink_SetMaintenanceInfoPage((uint16_t)index, page));
        put_u16(page_request + 8U, (uint16_t)index);
        sent = false;
        sent_count = 0U;
        enqueue_request(&request);
        YgProtocolLink_Service(index);
        assert(sent && sent_frame.data[10] == 32U && memcmp(sent_frame.data + 16U, page, 32U) == 0);
    }
    assert(YgProtocolLink_Init(7U));
    assert(YgProtocolLink_SetDeviceDescription(&info, &caps));
    memset(page, 0, sizeof(page));
    sent = false;
    enqueue_request(&request);
    YgProtocolLink_Service(3U);
    assert(sent && memcmp(sent_frame.data + 16U, page, 32U) == 0);
}

int main(void)
{
    uint8_t payload[YG_PROTOCOL_READONLY_PAGE_REQUEST_SIZE] = {0};
    yg_protocol_message_t request = {0};
    yg_protocol_transfer_frame_t request_frame = {0};
    yg_protocol_transfer_frame_t response_frame = {0};
    yg_protocol_message_t response = {0};
    CommHwCanFrame received = {0};
    uint32_t session_id = 0x11223344U;
    uint32_t request_id = 0x55667788U;

    assert(YgProtocolLink_Init(7U));
    assert(YgProtocolLink_IsReady());

    /* 身份由组合层显式绑定；初始化不能再假定存在默认设备能力。 */
    yg_protocol_readonly_info_page0_t info = {0};
    yg_protocol_readonly_caps_page0_t capabilities = {.features = 1U};
    assert(YgProtocolLink_SetDeviceDescription(&info, &capabilities));

    put_u32(payload, session_id);
    put_u32(payload + 4U, request_id);
    put_u16(payload + 8U, 0U);
    request.version = YG_PROTOCOL_VERSION;
    request.flags = YG_PROTOCOL_FLAGS_ACK_REQUEST;
    request.source_node = 1U;
    request.destination_node = 7U;
    request.message_type = YG_PROTOCOL_READONLY_TYPE_GET_CAPS;
    request.sequence = 12U;
    request.payload_length = sizeof(payload);
    request.payload = payload;
    assert(yg_protocol_canfd_pack(&request, 2U, &request_frame) == YG_PROTOCOL_OK);

    {
        received.identifier = request_frame.identifier;
        received.length = request_frame.length;
        received.extended = request_frame.extended;
        received.remote = request_frame.remote;
        received.fd = request_frame.fd;
        received.bitrate_switch = request_frame.bitrate_switch;
        memcpy(received.data, request_frame.data, received.length);
        assert(YgProtocolLink_OnRxFrame(&received));
    }

    YgProtocolLink_Service(100U);
    assert(sent);
    assert(sent_frame.extended && sent_frame.fd && sent_frame.bitrate_switch);
    response_frame.identifier = sent_frame.identifier;
    response_frame.length = sent_frame.length;
    response_frame.extended = sent_frame.extended;
    response_frame.remote = sent_frame.remote;
    response_frame.fd = sent_frame.fd;
    response_frame.bitrate_switch = sent_frame.bitrate_switch;
    memcpy(response_frame.data, sent_frame.data, sent_frame.length);
    assert(yg_protocol_canfd_unpack(&response_frame, 1U, &response) == YG_PROTOCOL_OK);
    assert(response.source_node == 7U);
    assert(response.destination_node == 1U);
    assert(response.message_type == YG_PROTOCOL_READONLY_TYPE_GET_CAPS);
    assert(response.sequence == 12U);
    assert((response.flags & YG_PROTOCOL_FLAGS_RESPONSE) != 0U);
    assert(response.payload_length == YG_PROTOCOL_READONLY_CAPS_PAGE0_SIZE);
    assert(response.payload[0] == 1U);
    assert(response.payload[1] == 0U);

    sent = false;
    assert(YgProtocolLink_Init(7U));
    request.flags = YG_PROTOCOL_FLAGS_ACK_REQUEST;
    request.source_node = 2U;
    request.message_type = YG_PROTOCOL_MOTOR_TYPE_STOP;
    request.payload_length = 0U;
    request.payload = NULL;
    assert(yg_protocol_canfd_pack(&request, 2U, &request_frame) == YG_PROTOCOL_OK);
    received.identifier = request_frame.identifier;
    received.length = request_frame.length;
    received.extended = request_frame.extended;
    received.remote = request_frame.remote;
    received.fd = request_frame.fd;
    received.bitrate_switch = request_frame.bitrate_switch;
    memcpy(received.data, request_frame.data, received.length);
    assert(YgProtocolLink_OnRxFrame(&received));
    YgProtocolLink_Service(200U);
    assert(sent);
    response_frame.identifier = sent_frame.identifier;
    response_frame.length = sent_frame.length;
    response_frame.extended = sent_frame.extended;
    response_frame.remote = sent_frame.remote;
    response_frame.fd = sent_frame.fd;
    response_frame.bitrate_switch = sent_frame.bitrate_switch;
    memcpy(response_frame.data, sent_frame.data, sent_frame.length);
    assert(yg_protocol_canfd_unpack(&response_frame, 2U, &response) == YG_PROTOCOL_OK);
    assert(response.message_type == YG_PROTOCOL_MOTOR_TYPE_STOP);
    assert(response.payload_length == YG_PROTOCOL_MOTOR_REPLY_SIZE);
    assert(response.payload[0] == YG_PROTOCOL_SERVICE_UNSUPPORTED);
    assert(response.payload[1] == 0U && response.payload[2] == 0U && response.payload[3] == 0U);

    /* 旧 111 不再注册；即使帧校验正确也不能触发控制或产生成功回复。 */
    sent = false;
    request.message_type = 111U;
    assert(yg_protocol_canfd_pack(&request, 2U, &request_frame) == YG_PROTOCOL_OK);
    received.identifier = request_frame.identifier;
    received.length = request_frame.length;
    memcpy(received.data, request_frame.data, received.length);
    assert(YgProtocolLink_OnRxFrame(&received));
    YgProtocolLink_Service(201U);
    assert(!sent);

    sent = false;
    assert(YgProtocolLink_Init(7U));
    request.flags = YG_PROTOCOL_FLAGS_ACK_REQUEST;
    request.source_node = 1U;
    request.message_type = YG_PROTOCOL_READONLY_TYPE_GET_MOTOR_STATE;
    request.sequence = 13U;
    request.payload_length = 0U;
    request.payload = NULL;
    assert(yg_protocol_canfd_pack(&request, 2U, &request_frame) == YG_PROTOCOL_OK);
    received.identifier = request_frame.identifier;
    received.length = request_frame.length;
    received.extended = request_frame.extended;
    received.remote = request_frame.remote;
    received.fd = request_frame.fd;
    received.bitrate_switch = request_frame.bitrate_switch;
    memcpy(received.data, request_frame.data, received.length);
    assert(YgProtocolLink_OnRxFrame(&received));
    YgProtocolLink_Service(300U);
    assert(sent);
    response_frame.identifier = sent_frame.identifier;
    response_frame.length = sent_frame.length;
    response_frame.extended = sent_frame.extended;
    response_frame.remote = sent_frame.remote;
    response_frame.fd = sent_frame.fd;
    response_frame.bitrate_switch = sent_frame.bitrate_switch;
    memcpy(response_frame.data, sent_frame.data, sent_frame.length);
    assert(yg_protocol_canfd_unpack(&response_frame, 1U, &response) == YG_PROTOCOL_OK);
    assert(response.message_type == YG_PROTOCOL_READONLY_TYPE_GET_MOTOR_STATE);
    assert(response.payload_length == YG_PROTOCOL_MOTOR_FEEDBACK_PAYLOAD_SIZE);
    assert(response.payload[0] == 6U && response.payload[1] == 0U && response.payload[24] == 0U &&
           response.payload[42] == 7U);
    enable_route();
    start_route_contract();
    control_reply_and_feedback();
    backlog_and_interrupt_handoff();
    control_history_window();
    power_route_contract();
    status_binding_reset();
    boot_confirm_and_info_contract();
    runtime_start_boundary();
    lifecycle_epoch_and_retry_contract();
    safety_boundaries_revoke_old_starts();
    lifecycle_golden_frames();
    return 0;
}
