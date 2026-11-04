#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "bsp_can.h"
#include "bsp_system.h"
#include "yg_protocol.h"
#include "yg_protocol.h"
#include "yg_protocol.h"
#include "yg_protocol.h"

/* 同一进程复用独立测试场景；生产入口禁止运行中重置。 */
#include "../../firmware/communication/yg_protocol.c"
#include "../../firmware/communication/yg_protocol_boot.c"
#include "protocol_test_bindings.h"
static bool fresh_link(uint8_t node)
{
    return test_reset_protocol(node);
}

static BspCanFrame sent_frame;
static bool sent;
static uint32_t irq_mask;
static unsigned critical_entries;
static unsigned motor_calls;
static uint16_t start_stop_barrier;
static bool tx_blocked;
static BspCanFrame injected_frame;
static bool inject_on_unlock;
static bool inject_on_consume;
static uint16_t sent_sequences[128];
static size_t sent_count;
static unsigned can_starts;

static bool tracked_allowed;
static bool hardware_busy, hardware_complete;
static unsigned boot_completion_calls;
static bool boot_completion_success;
static unsigned rx_reads;
static unsigned rx_remaining;

/* 直接验证迁入 link 的中断入口，FIFO 替身不接触硬件。 */
bool BspCan_Receive(BspCanFrame *frame)
{
    ++rx_reads;
    if (rx_remaining == 0U)
    {
        return false;
    }
    --rx_remaining;
    *frame = injected_frame;
    return true;
}

/* 单帧硬件完成标志由夹具注入；事务关联由真实link拥有。 */
bool BspCan_IsTransmitBusy(void)
{
    return hardware_busy;
}
bool BspCan_IsTransmitComplete(void)
{
    return !hardware_busy && hardware_complete;
}
void BspCan_Start(void)
{
    ++can_starts;
}

uint32_t BspSystem_EnterCritical(void)
{
    ++critical_entries;
    uint32_t previous = irq_mask;
    if (previous == 0U && inject_on_consume)
    {
        inject_on_consume = false;
        assert(Protocol_QueueReceivedFrame(&injected_frame));
    }
    irq_mask = 1U;
    return previous;
}

void BspSystem_ExitCritical(uint32_t state)
{
    assert(irq_mask == 1U);
    irq_mask = state;
    if (state == 0U && inject_on_unlock)
    {
        inject_on_unlock = false;
        assert(Protocol_QueueReceivedFrame(&injected_frame));
    }
}

void BspSystem_MemoryBarrier(void)
{
    assert(irq_mask == 0U);
    if (inject_on_consume || inject_on_unlock)
    {
        inject_on_consume = inject_on_unlock = false;
        assert(Protocol_QueueReceivedFrame(&injected_frame));
    }
}

/* 替代平台发送口，验证链路只读请求的编码、路由与响应，不接触真实 CAN 或电机。 */
bool BspCan_TrySend(const BspCanFrame *frame)
{
    assert(irq_mask == 0U);
    if (frame == NULL || sent || tx_blocked || hardware_busy)
    {
        return false;
    }
    sent_frame = *frame;
    if (tracked_allowed && frame->data[6] == PROTOCOL_BOOT_ENTER)
    {
        hardware_busy = true;
        hardware_complete = false;
    }
    sent = true;
    assert(sent_count < sizeof(sent_sequences) / sizeof(sent_sequences[0]));
    sent_sequences[sent_count++] = (uint16_t)frame->data[8] | ((uint16_t)frame->data[9] << 8U);
    return true;
}

static void motor_handler(const ProtocolCommand *request, ProtocolReply *reply)
{
    assert(irq_mask == 0U);
    (void)request;
    ++motor_calls;
    reply->status = PROTOCOL_OK;
}

/* 在发送持续忙时填满 TX，随后恢复发送：命令不得执行两次或乱序丢失。 */
static void irq_receive_budget(void)
{
    ProtocolMotor service = {.stop = motor_handler};
    yg_protocol_message_t request = {.version = YG_PROTOCOL_VERSION,
                                     .flags = YG_PROTOCOL_FLAGS_ACK_REQUEST,
                                     .source_node = 2U,
                                     .destination_node = 7U,
                                     .message_type = PROTOCOL_STOP};
    BspCanFrame frame;
    assert(fresh_link(7U));
    s_test_motor = service;
    assert(encode_can_frame(&request, 3U, &frame) == YG_PROTOCOL_OK);
    injected_frame = (BspCanFrame){.identifier = frame.identifier,
                                   .length = frame.length,
                                   .extended = true,
                                   .fd = true,
                                   .bitrate_switch = true};
    memcpy(injected_frame.data, frame.data, frame.length);
    motor_calls = 0U;
    sent_count = 0U;
    sent = false;
    rx_reads = 0U;
    rx_remaining = 5U;
    unsigned entries_before = critical_entries;
    Protocol_RxInterrupt();
    assert(critical_entries == entries_before);
    assert(rx_reads == 3U && rx_remaining == 2U && motor_calls == 0U && !sent);
    Protocol_Process(1U);
    assert(motor_calls == 3U);
    entries_before = critical_entries;
    Protocol_RxInterrupt();
    assert(critical_entries == entries_before);
    assert(rx_reads == 6U && rx_remaining == 0U && motor_calls == 3U);
    Protocol_Process(2U);
    assert(motor_calls == 5U);
}

static void backlog_and_interrupt_handoff(void)
{
    ProtocolMotor service = {.stop = motor_handler};
    yg_protocol_message_t request = {.version = YG_PROTOCOL_VERSION,
                                     .flags = YG_PROTOCOL_FLAGS_ACK_REQUEST,
                                     .source_node = 2U,
                                     .destination_node = 7U,
                                     .message_type = PROTOCOL_STOP};
    BspCanFrame frame;
    assert(fresh_link(7U));
    s_test_motor = service;
    motor_calls = 0U;
    sent_count = 0U;
    sent = false;
    tx_blocked = true;
    for (unsigned index = 0U; index < 8U; ++index)
    {
        request.sequence = (uint16_t)index;
        assert(encode_can_frame(&request, 3U, &frame) == YG_PROTOCOL_OK);
        injected_frame = (BspCanFrame){.identifier = frame.identifier,
                                       .length = frame.length,
                                       .extended = true,
                                       .fd = true,
                                       .bitrate_switch = true};
        memcpy(injected_frame.data, frame.data, frame.length);
        assert(Protocol_QueueReceivedFrame(&injected_frame));
    }
    /* RX 队列 8 帧已满，最新帧丢弃；后台服务再处理这 8 帧。 */
    assert(!Protocol_QueueReceivedFrame(&injected_frame));
    assert(motor_calls == 0U);
    Protocol_Process(10U);
    /* TX 仅容纳 8 帧，预留两个回复槽后第 8 条请求必须留在 RX。 */
    assert(motor_calls == 7U && sent_count == 0U);
    Protocol_Process(12U);
    assert(motor_calls == 7U);
    tx_blocked = false;
    for (unsigned i = 0U; i < 20U; ++i)
    {
        sent = false;
        Protocol_Process(13U + i);
    }
    assert(motor_calls == 8U && sent_count == 8U);
    for (unsigned i = 0U; i < 8U; ++i)
    {
        assert(sent_sequences[i] == i);
    }
    /* 在后台解锁出队副本时模拟 RX 中断，验证新帧不会覆盖已出队的请求。 */
    assert(Protocol_QueueReceivedFrame(&injected_frame));
    inject_on_unlock = true;
    sent = false;
    Protocol_Process(100U);
    assert(motor_calls == 10U && sent_count == 9U && !inject_on_unlock && irq_mask == 0U);
    /* 队首先复制，再由ISR追加，最后提交消费；原帧和新帧都只执行一次。 */
    assert(Protocol_QueueReceivedFrame(&injected_frame));
    inject_on_consume = true;
    sent = false;
    Protocol_Process(101U);
    assert(motor_calls == 12U && !inject_on_consume && irq_mask == 0U);
    assert(critical_entries == 0U);
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

static void control_reply_and_feedback(void)
{
    uint8_t payload[PROTOCOL_CONTROL_SIZE] = {3U, 0U, 0U, 0U, 0xE8U, 0x03U};
    yg_protocol_message_t request = {.version = YG_PROTOCOL_VERSION,
                                     .flags = YG_PROTOCOL_FLAGS_ACK_REQUEST,
                                     .source_node = 2U,
                                     .destination_node = 7U,
                                     .message_type = PROTOCOL_CONTROL,
                                     .sequence = 41U,
                                     .payload_length = sizeof(payload),
                                     .payload = payload};
    BspCanFrame packed, response_frame;
    yg_protocol_message_t response;
    BspCanFrame received = {0};
    assert(fresh_link(7U));
    assert(encode_can_frame(&request, 2U, &packed) == YG_PROTOCOL_OK);
    received.identifier = packed.identifier;
    received.length = packed.length;
    received.extended = true;
    received.fd = true;
    received.bitrate_switch = true;
    memcpy(received.data, packed.data, packed.length);
    sent = false;
    assert(Protocol_QueueReceivedFrame(&received));
    Protocol_Process(1U);
    assert(sent);
    response_frame = (BspCanFrame){0};
    response_frame.identifier = sent_frame.identifier;
    response_frame.length = sent_frame.length;
    response_frame.extended = true;
    response_frame.fd = true;
    response_frame.bitrate_switch = true;
    memcpy(response_frame.data, sent_frame.data, sent_frame.length);
    assert(decode_can_frame(&response_frame, 2U, &response) == YG_PROTOCOL_OK);
    assert(response.message_type == PROTOCOL_CONTROL && response.sequence == 41U);
    assert(response.payload_length == PROTOCOL_REPLY_SIZE && response.payload[0] == 2U);

    sent = false;
    Protocol_Process(2U);
    assert(sent);
    response_frame.identifier = sent_frame.identifier;
    response_frame.length = sent_frame.length;
    memcpy(response_frame.data, sent_frame.data, sent_frame.length);
    assert(decode_can_frame(&response_frame, 2U, &response) == YG_PROTOCOL_OK);
    assert(response.message_type == PROTOCOL_MOTION_FEEDBACK &&
           response.payload_length == PROTOCOL_FEEDBACK_SIZE && response.payload[0] == 2U &&
           response.payload[2] == 41U && response.payload[42] == 7U);
    assert(response.payload[24] == 0U && response.payload[25] == 0U);
}

static void enable_route(void)
{
    uint8_t payload[PROTOCOL_ENABLE_SIZE] = {1U};
    yg_protocol_message_t request = {.version = YG_PROTOCOL_VERSION,
                                     .flags = YG_PROTOCOL_FLAGS_ACK_REQUEST,
                                     .source_node = 2U,
                                     .destination_node = 7U,
                                     .message_type = PROTOCOL_ENABLE,
                                     .sequence = 40U,
                                     .payload_length = sizeof(payload),
                                     .payload = payload};
    BspCanFrame frame;
    BspCanFrame received = {0};
    assert(fresh_link(7U));
    assert(encode_can_frame(&request, 2U, &frame) == YG_PROTOCOL_OK);
    received.identifier = frame.identifier;
    received.length = frame.length;
    received.extended = true;
    received.fd = true;
    received.bitrate_switch = true;
    memcpy(received.data, frame.data, frame.length);
    sent = false;
    assert(Protocol_QueueReceivedFrame(&received));
    Protocol_Process(1U);
    assert(sent && sent_frame.data[6] == PROTOCOL_ENABLE && sent_frame.data[8] == 40U &&
           sent_frame.data[16] == 2U);
}

static void power_handler(const ProtocolCommand *request, ProtocolReply *reply)
{
    ++motor_calls;
    (void)request;
    reply->status = PROTOCOL_OK;
    reply->detail = 0x1234U;
}

static void enqueue_request(const yg_protocol_message_t *request)
{
    BspCanFrame packed;
    BspCanFrame received = {0};
    assert(encode_can_frame(request, 2U, &packed) == YG_PROTOCOL_OK);
    received.identifier = packed.identifier;
    received.length = packed.length;
    received.extended = true;
    received.fd = true;
    received.bitrate_switch = true;
    memcpy(received.data, packed.data, packed.length);
    assert(Protocol_QueueReceivedFrame(&received));
}

/* 历史槽覆盖后旧序号只能拒绝；仍在窗口内的重发须重放原结果。 */
static void unsupported_control_requests(void)
{
    uint8_t payload[PROTOCOL_CONTROL_SIZE] = {3U, 0U, 0U, 0U, 0xE8U, 0x03U};
    yg_protocol_message_t request = {.version = YG_PROTOCOL_VERSION,
                                     .flags = YG_PROTOCOL_FLAGS_ACK_REQUEST,
                                     .source_node = 2U,
                                     .destination_node = 7U,
                                     .message_type = PROTOCOL_CONTROL,
                                     .payload_length = sizeof(payload),
                                     .payload = payload};
    assert(fresh_link(7U));
    sent_count = 0U;
    for (uint16_t index = 0U; index < 35U; ++index)
    {
        request.sequence = (uint16_t)(100U + index % 33U);
        sent = false;
        enqueue_request(&request);
        Protocol_Process((uint32_t)index * 2U);
        assert(sent && sent_frame.data[16] == 2U);
        sent = false;
        Protocol_Process((uint32_t)index * 2U + 1U);
        assert(sent && sent_frame.data[6] == PROTOCOL_MOTION_FEEDBACK && sent_frame.data[16] == 2U);
    }
}

static void start_handler(const ProtocolCommand *request, ProtocolReply *reply)
{
    assert(request->source_node == 2U && request->sequence == 0x1001U && request->mode == 2U);
    ++motor_calls;
    reply->status = start_stop_barrier == 0U ? PROTOCOL_ACCEPTED : PROTOCOL_INVALID_ARGUMENT;
    reply->detail = start_stop_barrier == 0U ? 0U : 0x0101U;
}

static void start_stop_handler(const ProtocolCommand *request, ProtocolReply *reply)
{
    start_stop_barrier = request->sequence;
    reply->status = PROTOCOL_OK;
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
                                     .message_type = PROTOCOL_START,
                                     .sequence = 0x1001U,
                                     .payload_length = sizeof(payload),
                                     .payload = payload};
    const ProtocolMotor service = {.start = start_handler, .stop = start_stop_handler};
    BspCanFrame packed;
    assert(fresh_link(1U));
    s_test_motor = service;
    start_stop_barrier = 0U;
    sent_count = 0U;
    assert(encode_can_frame(&request, 2U, &packed) == YG_PROTOCOL_OK);
    assert(packed.identifier == 0x08EF0102U && packed.length == 24U);
    assert(memcmp(packed.data, request_bytes, sizeof(request_bytes)) == 0);
    motor_calls = 0U;
    sent = false;
    enqueue_request(&request);
    Protocol_Process(1U);
    assert(motor_calls == 1U && sent && sent_frame.identifier == 0x0CEF0201U);
    assert(sent_frame.length == 24U);
    assert(memcmp(sent_frame.data, accepted_bytes, sizeof(accepted_bytes)) == 0);

    sent = false;
    request.source_node = 3U;
    enqueue_request(&request);
    Protocol_Process(2U);
    assert(sent && sent_frame.data[16] == 7U && motor_calls == 1U);

    sent = false;
    request.source_node = 2U;
    request.payload_length = 3U;
    enqueue_request(&request);
    Protocol_Process(3U);
    assert(sent && sent_frame.data[16] == 3U && motor_calls == 1U);

    sent = false;
    request.payload_length = 4U;
    payload[1] = 1U;
    enqueue_request(&request);
    Protocol_Process(4U);
    assert(sent && sent_frame.data[16] == 4U && motor_calls == 1U);

    sent = false;
    payload[1] = 0U;
    payload[0] = 4U;
    enqueue_request(&request);
    Protocol_Process(5U);
    assert(sent && sent_frame.data[16] == 2U && motor_calls == 1U);

    /* 新会话中 STOP 先抵达，之后重发的旧 START 只得到陈旧身份结果。 */
    assert(fresh_link(1U));
    s_test_motor = service;
    request.message_type = PROTOCOL_STOP;
    request.sequence = 0x1004U;
    request.payload_length = 0U;
    request.payload = NULL;
    assert(encode_can_frame(&request, 2U, &packed) == YG_PROTOCOL_OK);
    assert(packed.length == 20U && memcmp(packed.data, stop_bytes, sizeof(stop_bytes)) == 0);
    sent = false;
    enqueue_request(&request);
    Protocol_Process(6U);
    assert(sent && start_stop_barrier == 0x1004U);
    request.message_type = PROTOCOL_START;
    request.sequence = 0x1001U;
    request.flags = YG_PROTOCOL_FLAGS_ACK_REQUEST | YG_PROTOCOL_FLAGS_RETRY;
    request.payload_length = sizeof(payload);
    request.payload = payload;
    payload[0] = 2U;
    assert(encode_can_frame(&request, 2U, &packed) == YG_PROTOCOL_OK);
    assert(packed.length == 24U && memcmp(packed.data, retry_bytes, sizeof(retry_bytes)) == 0);
    sent = false;
    enqueue_request(&request);
    Protocol_Process(7U);
    assert(sent && sent_frame.length == 24U);
    assert(memcmp(sent_frame.data, stale_bytes, sizeof(stale_bytes)) == 0);
}

static uint8_t accepted_start_mode;

static void multimode_start_handler(const ProtocolCommand *request, ProtocolReply *reply)
{
    accepted_start_mode = request->mode;
    ++motor_calls;
    reply->status = PROTOCOL_ACCEPTED;
}

static void multimode_start_vectors(void)
{
    static const uint8_t modes[] = {1U, 3U, 6U};
    static const uint8_t frames[][24] = {
        {0x5A, 0xA5, 0x01, 0x20, 0x02, 0x01, 0x75, 0x00, 0x01, 0x10, 0x04, 0x00,
         0x00, 0x00, 0x00, 0x82, 0x01, 0x00, 0x00, 0x00, 0xA2, 0x46, 0x00, 0x00},
        {0x5A, 0xA5, 0x01, 0x20, 0x02, 0x01, 0x75, 0x00, 0x01, 0x10, 0x04, 0x00,
         0x00, 0x00, 0x00, 0x82, 0x03, 0x00, 0x00, 0x00, 0x70, 0x99, 0x00, 0x00},
        {0x5A, 0xA5, 0x01, 0x20, 0x02, 0x01, 0x75, 0x00, 0x01, 0x10, 0x04, 0x00,
         0x00, 0x00, 0x00, 0x82, 0x06, 0x00, 0x00, 0x00, 0x90, 0xF3, 0x00, 0x00}};
    uint8_t payload[PROTOCOL_START_SIZE] = {0};
    yg_protocol_message_t request = {.version = YG_PROTOCOL_VERSION,
                                     .flags = YG_PROTOCOL_FLAGS_ACK_REQUEST,
                                     .source_node = 2U,
                                     .destination_node = 1U,
                                     .message_type = PROTOCOL_START,
                                     .sequence = 0x1001U,
                                     .payload_length = sizeof(payload),
                                     .payload = payload};
    const ProtocolMotor service = {.start = multimode_start_handler};
    BspCanFrame packed;
    for (size_t index = 0U; index < sizeof(modes); ++index)
    {
        assert(fresh_link(1U));
        s_test_motor = service;
        motor_calls = 0U;
        sent_count = 0U;
        sent = false;
        payload[0] = modes[index];
        assert(encode_can_frame(&request, 2U, &packed) == YG_PROTOCOL_OK);
        assert(packed.identifier == 0x08EF0102U && packed.length == 24U);
        assert(memcmp(packed.data, frames[index], sizeof(frames[index])) == 0);
        enqueue_request(&request);
        Protocol_Process((uint32_t)index);
        assert(sent && sent_frame.data[16] == 1U && motor_calls == 1U &&
               accepted_start_mode == modes[index]);
    }
    assert(fresh_link(1U));
    s_test_motor = service;
    motor_calls = 0U;
    sent_count = 0U;
    sent = false;
    payload[0] = 4U;
    enqueue_request(&request);
    Protocol_Process(10U);
    assert(sent && sent_frame.data[16] == 2U && motor_calls == 0U);
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
    } cases[] = {
        {101U, 2U, disabled, 4U, true, 0U, true},
        {101U, 2U, enabled, 4U, true, 2U, false},
        {110U, 2U, NULL, 0U, true, 0U, true},
        {101U, 1U, enabled, 1U, true, 7U, false},
        {101U, 2U, enabled, 1U, true, 3U, false},
        {101U, 2U, invalid, 4U, true, 4U, false},
        {110U, 1U, NULL, 0U, true, 7U, false},
        {110U, 1U, enabled, 1U, false, 0U, false},
        {110U, 2U, enabled, 1U, false, 0U, false},
    };
    const ProtocolMotor service = {.stop = power_handler};
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
        assert(fresh_link(7U));
        s_test_motor = service;
        motor_calls = 0U;
        sent_count = 0U;
        sent = false;
        enqueue_request(&request);
        Protocol_Process(0U);
        assert(sent == cases[index].response);
        assert(motor_calls == (cases[index].dispatched ? 1U : 0U));
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
    const ProtocolMotorStatus source = {.sample_available = true, .state = 6U, .mode = 3U};
    const yg_protocol_message_t request = {.version = YG_PROTOCOL_VERSION,
                                           .flags = YG_PROTOCOL_FLAGS_ACK_REQUEST,
                                           .source_node = 2U,
                                           .destination_node = 7U,
                                           .message_type = PROTOCOL_GET_MOTOR_STATE,
                                           .sequence = 199U};
    assert(fresh_link(7U));
    s_test_motor_status = source;
    sent_count = 0U;
    sent = false;
    enqueue_request(&request);
    Protocol_Process(0U);
    assert(sent && sent_frame.data[16] == 0U && sent_frame.data[59] == 6U);
    for (unsigned reset = 0U; reset < 2U; ++reset)
    {
        s_test_motor_status = source;
        if (reset == 0U)
        {
            s_test_motor_status = (ProtocolMotorStatus){0};
        }
        else
        {
            assert(fresh_link(7U));
        }
        sent = false;
        enqueue_request(&request);
        Protocol_Process(0U);
        assert(sent && sent_frame.data[16] == 6U && sent_frame.data[40] == 0U &&
               sent_frame.data[59] == 0U && sent_frame.data[60] == 0U);
    }
}

/* 运行时只启动一次；参数桥更新不改变冻结地址，重复初始化不丢弃在途请求。 */
static void rejected_frames_do_not_execute(void)
{
    assert(fresh_link(7U));
    ProtocolMotor service = {.stop = motor_handler};
    s_test_motor = service;
    yg_protocol_message_t request = {.version = YG_PROTOCOL_VERSION,
                                     .flags = YG_PROTOCOL_FLAGS_ACK_REQUEST,
                                     .source_node = 2U,
                                     .destination_node = 7U,
                                     .message_type = PROTOCOL_STOP,
                                     .sequence = 700U};
    BspCanFrame frame;
    uint8_t payload[46] = {0};
    motor_calls = 0U;
    sent_count = 0U;
    tx_blocked = false;
    for (unsigned variant = 0; variant < 14U; ++variant)
    {
        request.flags = YG_PROTOCOL_FLAGS_ACK_REQUEST;
        request.destination_node = 7U;
        request.message_type = PROTOCOL_STOP;
        request.payload = NULL;
        request.payload_length = 0U;
        if (variant < 3U)
        {
            request.flags |= (uint8_t)((variant + 1U) << 6U);
            request.payload = payload;
            request.payload_length = sizeof(payload);
        }
        else if (variant == 3U)
        {
            request.destination_node = YG_PROTOCOL_BROADCAST_NODE_ID;
        }
        else if (variant == 4U)
        {
            request.flags = YG_PROTOCOL_FLAGS_RESPONSE;
        }
        else if (variant == 5U)
        {
            request.message_type = 0x7F00U;
        }
        assert(encode_can_frame(&request, 3U, &frame) == YG_PROTOCOL_OK);
        if (variant == 6U)
        {
            frame.data[15] ^= 1U;
        }
        if (variant == 7U)
        {
            frame.data[17] ^= 1U;
        }
        if (variant == 8U)
        {
            frame.extended = false;
        }
        if (variant == 9U)
        {
            frame.remote = true;
        }
        if (variant == 10U)
        {
            frame.fd = false;
        }
        if (variant == 11U)
        {
            frame.bitrate_switch = false;
        }
        if (variant == 12U)
        {
            frame.length = 19U;
        }
        if (variant == 13U)
        {
            frame.identifier |= 1UL << 25U;
        }
        assert(Protocol_QueueReceivedFrame(&frame));
        Protocol_Process(variant);
        assert(motor_calls == 0U && sent_count == 0U);
        assert(queue_count(&s_rx_queue) == 0U);
    }
    /* GET_INFO 也不接受分片；合法查询已有完整单帧回归。 */
    request.message_type = PROTOCOL_GET_INFO;
    request.flags = YG_PROTOCOL_FLAGS_ACK_REQUEST | YG_PROTOCOL_FLAGS_FRAGMENT_FIRST;
    request.payload = payload;
    request.payload_length = sizeof(payload);
    assert(encode_can_frame(&request, 3U, &frame) == YG_PROTOCOL_OK);
    assert(Protocol_QueueReceivedFrame(&frame));
    Protocol_Process(10U);
    assert(motor_calls == 0U && sent_count == 0U);
}

static void parameter_wire_contract(void)
{
    assert(fresh_link(7U));
    for (unsigned variant = 0U; variant < 8U; ++variant)
    {
        uint8_t payload[12] = {0x66U, 0U, 0U, 0U};
        yg_protocol_message_t request = {.version = YG_PROTOCOL_VERSION,
                                         .flags = YG_PROTOCOL_FLAGS_ACK_REQUEST,
                                         .source_node = 2U,
                                         .destination_node = 7U,
                                         .sequence = (uint16_t)variant,
                                         .message_type = PROTOCOL_READ_PARAMETER,
                                         .payload = payload,
                                         .payload_length = 4U};
        uint16_t expected = 2U, detail = 0U;
        if (variant == 1U)
        {
            payload[3] = 1U;
            expected = 4U;
            detail = 6U;
        }
        if (variant >= 2U && variant <= 4U)
        {
            request.message_type = PROTOCOL_WRITE_PARAMETER;
            request.payload_length = 12U;
            payload[1] = variant == 3U ? 2U : 1U;
            payload[2] = 4U;
            payload[11] = 0xFFU;
            if (variant == 4U)
            {
                payload[2] = 3U;
                expected = 4U;
                detail = 6U;
            }
        }
        if (variant == 5U)
        {
            request.message_type = PROTOCOL_SAVE_PARAMETER;
        }
        if (variant == 6U)
        {
            request.payload_length = 3U;
            expected = 3U;
        }
        if (variant == 7U)
        {
            request.source_node = 4U;
            payload[3] = 1U;
            expected = 7U;
        }
        sent = false;
        enqueue_request(&request);
        Protocol_Process(variant);
        assert(sent && sent_frame.data[16] == expected && sent_frame.data[17] == 0U);
        assert(sent_frame.data[18] == detail && sent_frame.data[19] == 0U);
    }
}

static void runtime_start_boundary(void)
{
    yg_protocol_message_t stop = {.version = YG_PROTOCOL_VERSION,
                                  .flags = YG_PROTOCOL_FLAGS_ACK_REQUEST,
                                  .source_node = 2U,
                                  .destination_node = 1U,
                                  .message_type = PROTOCOL_STOP,
                                  .sequence = 200U};
    s_started = false;
    s_ready = false;
    can_starts = 0U;
    Protocol_Process(1000U);
    assert(can_starts == 0U);
    assert(!Protocol_Init(255U));
    assert(can_starts == 0U && !Protocol_IsReady());
    assert(Protocol_Init(1U));
    enqueue_request(&stop);
    assert(!Protocol_Init(9U));
    sent = false;
    sent_count = 0U;
    Protocol_Process(999U);
    assert(can_starts == 0U && !sent);
    assert(Protocol_Start());
    assert(Protocol_Start());
    Protocol_Process(999U);
    assert(can_starts == 1U && sent);
    /* 重新初始化撤销旧服务；仍从冻结的节点 1 返回未绑定结果。 */
    assert(sent_frame.data[4] == 1U && sent_frame.data[16] == 2U);
    sent = false;
    Protocol_Process(1000U);
    Protocol_Process(1999U);
    assert(can_starts == 1U);
    Protocol_Process(2000U);
    assert(can_starts == 1U);

    /* attempt-10实际Type1/seq12原始帧；只替换硬件端口，不以pack重建请求。 */
    const BspCanFrame captured = {.identifier = 0x0CEF0102U,
                                  .length = 32U,
                                  .extended = true,
                                  .fd = true,
                                  .bitrate_switch = true,
                                  .data = {0x5A, 0xA5, 0x01, 0x20, 0x02, 0x01, 0x01, 0x00,
                                           0x0C, 0x00, 0x0C, 0x00, 0x00, 0x00, 0x00, 0x4A,
                                           0x00, 0x00, 0x00, 0x00, 0x0C, 0x00, 0x00, 0x00,
                                           0x00, 0x00, 0x00, 0x00, 0x95, 0x6A, 0x00, 0x00}};
    const ProtocolInfo info = {
        .uid = {0x57, 0x00, 0x46, 0x00, 0x10, 0x50, 0x4E, 0x48, 0x31, 0x30, 0x33, 0x20}};
    const ProtocolCaps capabilities = {0};
    assert(Protocol_SetInfo(&info, &capabilities));
    sent = false;
    sent_count = 0U;
    assert(Protocol_QueueReceivedFrame(&captured));
    Protocol_Process(2001U);
    assert(sent && sent_count == 1U && can_starts == 1U);
    BspCanFrame response_frame = {.identifier = sent_frame.identifier,
                                  .length = sent_frame.length,
                                  .extended = sent_frame.extended,
                                  .remote = sent_frame.remote,
                                  .fd = sent_frame.fd,
                                  .bitrate_switch = sent_frame.bitrate_switch};
    yg_protocol_message_t response;
    memcpy(response_frame.data, sent_frame.data, sent_frame.length);
    /* unpack同时核对CAN元数据、头/帧CRC、填充与本机地址。 */
    assert(sent_frame.identifier == 0x0CEF0201U);
    assert(decode_can_frame(&response_frame, 2U, &response) == YG_PROTOCOL_OK);
    assert(response.version == YG_PROTOCOL_VERSION &&
           response.flags == YG_PROTOCOL_FLAGS_RESPONSE && response.source_node == 1U &&
           response.destination_node == 2U && response.message_type == PROTOCOL_GET_INFO &&
           response.sequence == 12U && response.payload_length == 32U &&
           memcmp(response.payload, info.uid, sizeof(info.uid)) == 0);
}

static unsigned confirm_calls;
static bool confirm_allowed;
static void
ready_boot_request(void *context, const BootEntryRequest *request, BootEntryReply *reply)
{
    (void)context;
    assert(request->boot_epoch == 9U);
    *reply = (BootEntryReply){.result = BOOT_ENTRY_READY, .ack_ticket = request->boot_epoch};
}

static void ready_boot_complete(void *context, uint64_t ticket, bool success)
{
    (void)context;
    assert(ticket == 9U);
    ++boot_completion_calls;
    boot_completion_success = success;
}

/* READY在回绕队列中排队；真实完成才成功回调，超时只移除该回复并保持普通帧顺序。 */
static void boot_ready_queue_contract(void)
{
    const BootEntryService boot_service = {NULL, ready_boot_request, ready_boot_complete, NULL};
    const ProtocolMotor motor_service = {.stop = motor_handler};
    uint8_t nonce[8] = {9U};
    yg_protocol_message_t request = {.version = YG_PROTOCOL_VERSION,
                                     .flags = YG_PROTOCOL_FLAGS_ACK_REQUEST,
                                     .source_node = 2U,
                                     .destination_node = 7U,
                                     .message_type = PROTOCOL_STOP};
    for (unsigned scenario = 0U; scenario < 3U; ++scenario)
    {
        assert(fresh_link(7U));
        s_test_motor = motor_service;
        s_test_boot = boot_service;
        tracked_allowed = true;
        hardware_busy = hardware_complete = false;
        boot_completion_calls = sent_count = 0U;
        tx_blocked = false;
        /* 先移动读写位置到末尾，让后续四条回复跨过环形边界。 */
        request.message_type = PROTOCOL_STOP;
        request.payload = NULL;
        request.payload_length = 0U;
        for (unsigned i = 0U; i < 7U; ++i)
        {
            sent = false;
            request.sequence = (uint16_t)i;
            enqueue_request(&request);
            Protocol_Process(i);
            assert(sent);
        }
        sent = false;
        tx_blocked = true;
        for (unsigned i = 0U; i < 4U; ++i)
        {
            request.sequence = (uint16_t)(50U + i);
            request.message_type = i == 2U ? PROTOCOL_BOOT_ENTER : PROTOCOL_STOP;
            request.payload = i == 2U ? nonce : NULL;
            request.payload_length = i == 2U ? sizeof(nonce) : 0U;
            enqueue_request(&request);
        }
        Protocol_Process(10U);
        assert(boot_completion_calls == 0U && !hardware_busy);
        if (scenario == 1U)
        {
            Protocol_Process(110U);
            assert(boot_completion_calls == 1U && !boot_completion_success);
        }
        tx_blocked = false;
        for (unsigned i = 0U; i < 4U; ++i)
        {
            sent = false;
            Protocol_Process((scenario == 1U ? 111U : 11U) + i);
            if (scenario != 1U && i == 2U)
            {
                assert(hardware_busy && boot_completion_calls == 0U);
                /* 查询未完成硬件不得消费票据，也不得发送后续普通帧。 */
                size_t before = sent_count;
                Protocol_Process(15U);
                assert(boot_completion_calls == 0U && sent_count == before);
                hardware_busy = false;
                hardware_complete = scenario == 0U;
                Protocol_Process(16U);
                assert(boot_completion_calls == 1U && boot_completion_success == (scenario == 0U));
            }
        }
        assert(boot_completion_calls == 1U);
        assert(sent_count == (scenario == 1U ? 10U : 11U));
        assert(sent_sequences[7] == 50U && sent_sequences[8] == 51U);
        assert(sent_sequences[9] == (scenario == 1U ? 53U : 52U));
        if (scenario != 1U)
        {
            assert(sent_sequences[10] == 53U);
        }
    }
    tracked_allowed = false;
}
/* 在途硬件一直忙时按应用期限失败；晚到完成不能重复消费事务。 */
static void boot_inflight_deadline(void)
{
    const BootEntryService service = {NULL, ready_boot_request, ready_boot_complete, NULL};
    uint8_t nonce[8] = {9U};
    yg_protocol_message_t request = {.version = YG_PROTOCOL_VERSION,
                                     .flags = YG_PROTOCOL_FLAGS_ACK_REQUEST,
                                     .source_node = 2U,
                                     .destination_node = 7U,
                                     .message_type = PROTOCOL_BOOT_ENTER,
                                     .payload = nonce,
                                     .payload_length = sizeof(nonce)};
    assert(fresh_link(7U));
    s_test_boot = service;
    tracked_allowed = true;
    sent = tx_blocked = hardware_busy = hardware_complete = false;
    boot_completion_calls = sent_count = 0U;
    enqueue_request(&request);
    Protocol_Process(10U);
    assert(sent_count == 1U && hardware_busy && boot_completion_calls == 0U);
    Protocol_Process(109U);
    assert(boot_completion_calls == 0U);
    Protocol_Process(110U);
    assert(boot_completion_calls == 1U && !boot_completion_success && hardware_busy);
    hardware_busy = false;
    hardware_complete = true;
    Protocol_Process(111U);
    assert(boot_completion_calls == 1U && sent_count == 1U);
    tracked_allowed = false;
}

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
                                     .message_type = PROTOCOL_BOOT_CONFIRM,
                                     .sequence = 211U,
                                     .payload = identity,
                                     .payload_length = 8U};
    put_u32(identity, 0x10000U);
    put_u32(identity + 4U, 7U);
    for (unsigned scenario = 0U; scenario < 4U; ++scenario)
    {
        BootEntryService bound = service;
        assert(fresh_link(7U));
        bound.confirm = boot_confirm;
        s_test_boot = bound;
        confirm_calls = sent_count = 0U;
        sent = false;
        confirm_allowed = scenario == 0U;
        request.source_node = scenario == 2U ? 1U : 2U;
        request.payload_length = scenario == 3U ? 7U : 8U;
        enqueue_request(&request);
        Protocol_Process(1U);
        const uint8_t results[4] = {0U, 5U, 7U, 3U};
        assert(sent && sent_frame.data[16] == results[scenario]);
        assert(confirm_calls == (scenario < 2U ? 1U : 0U));
    }
    ProtocolInfo info = {0};
    ProtocolCaps caps = {0};
    assert(fresh_link(7U));
    assert(Protocol_SetInfo(&info, &caps));
    request.source_node = 2U;
    request.message_type = 1U;
    request.payload = page_request;
    request.payload_length = 12U;
    assert(!Protocol_SetInfoPage(0U, page));
    assert(!Protocol_SetInfoPage(3U, page));
    for (unsigned index = 1U; index <= 2U; ++index)
    {
        memset(page, (int)index, sizeof(page));
        assert(Protocol_SetInfoPage((uint16_t)index, page));
        put_u16(page_request + 8U, (uint16_t)index);
        sent = false;
        sent_count = 0U;
        enqueue_request(&request);
        Protocol_Process(index);
        assert(sent && sent_frame.data[10] == 32U && memcmp(sent_frame.data + 16U, page, 32U) == 0);
    }
    assert(fresh_link(7U));
    assert(Protocol_SetInfo(&info, &caps));
    memset(page, 0, sizeof(page));
    sent = false;
    enqueue_request(&request);
    Protocol_Process(3U);
    assert(sent && memcmp(sent_frame.data + 16U, page, 32U) == 0);
}

static void receive_watch_request(yg_protocol_message_t *request)
{
    ++request->sequence;
    sent = false;
    enqueue_request(request);
    Protocol_Process(request->sequence);
}

/* 通过实际 RX 队列分发，验证最近命令跨业务覆盖，非法报文保留先前值。 */
static void last_command_watch(void)
{
    assert(fresh_link(3U));
    sent_count = 0U;
    tx_blocked = hardware_busy = tracked_allowed = false;
    uint8_t data[20] = {2U};
    yg_protocol_message_t request = {.version = YG_PROTOCOL_VERSION,
                                     .flags = YG_PROTOCOL_FLAGS_ACK_REQUEST,
                                     .source_node = 2U,
                                     .destination_node = 3U,
                                     .sequence = 41U,
                                     .message_type = PROTOCOL_START,
                                     .payload = data,
                                     .payload_length = 4U};
    receive_watch_request(&request);
    memset(data, 0, sizeof(data));
    assert(g_protocol_command.type == PROTOCOL_START && g_protocol_command.sequence == 42U);
    assert(g_protocol_command.source_node == 2U && g_protocol_command.destination_node == 3U &&
           g_protocol_command.flags == YG_PROTOCOL_FLAGS_ACK_REQUEST);
    assert(g_protocol_command.args.motor.mode == 2U);

    data[1] = 1U;
    receive_watch_request(&request);
    assert(g_protocol_command.sequence == 42U && g_protocol_command.args.motor.mode == 2U);
    data[0] = 2U;
    data[1] = 0U;
    request.source_node = 1U;
    receive_watch_request(&request);
    assert(g_protocol_command.sequence == 42U);
    request.source_node = 2U;
    BspCanFrame bad;
    assert(encode_can_frame(&request, 2U, &bad) == YG_PROTOCOL_OK);
    bad.data[15] ^= 1U;
    assert(Protocol_QueueReceivedFrame(&bad));
    Protocol_Process(45U);
    assert(g_protocol_command.sequence == 42U);

    request.message_type = PROTOCOL_CONTROL;
    request.payload_length = PROTOCOL_CONTROL_SIZE;
    YgProtocol_WriteU32Le(data + 8U, (uint32_t)-2500);
    receive_watch_request(&request);
    assert(g_protocol_command.args.motor.speed_mrad_s == -2500);
    request.message_type = PROTOCOL_GET_INFO;
    request.payload_length = PROTOCOL_PAGE_REQUEST_SIZE;
    YgProtocol_WriteU32Le(data + 8U, 1U);
    receive_watch_request(&request);
    assert(g_protocol_command.type == PROTOCOL_GET_INFO && g_protocol_command.args.page.page == 1U);
    request.message_type = PROTOCOL_GET_DEVICE_STATUS;
    request.payload_length = 0U;
    receive_watch_request(&request);
    assert(g_protocol_command.type == PROTOCOL_GET_DEVICE_STATUS);

    request.message_type = PROTOCOL_START_JOB;
    request.payload_length = PROTOCOL_JOB_REQUEST_SIZE;
    YgProtocol_WriteU32Le(data, PROTOCOL_JOB_KIND_MECHANICAL_ZERO);
    receive_watch_request(&request);
    assert(g_protocol_command.args.job.kind == PROTOCOL_JOB_KIND_MECHANICAL_ZERO);
    request.message_type = PROTOCOL_WRITE_PARAMETER;
    request.payload_length = 12U;
    data[0] = 9U;
    data[1] = 1U;
    data[2] = 4U;
    data[3] = 0U;
    YgProtocol_WriteU32Le(data + 4U, 7U);
    YgProtocol_WriteU32Le(data + 8U, 1234U);
    receive_watch_request(&request);
    assert(g_protocol_command.type == PROTOCOL_WRITE_PARAMETER);
    assert(g_protocol_command.args.parameter.id == 9U &&
           g_protocol_command.args.parameter.expected_revision == 7U &&
           g_protocol_command.args.parameter.value == 1234U);

    request.message_type = PROTOCOL_BOOT_ENTER;
    request.payload_length = 8U;
    YgProtocol_WriteU32Le(data, 0x12345678U);
    receive_watch_request(&request);
    assert(g_protocol_command.type == PROTOCOL_BOOT_ENTER &&
           g_protocol_command.args.boot_epoch == UINT64_C(0x0000000712345678));
    request.message_type = PROTOCOL_BOOT_CONFIRM;
    receive_watch_request(&request);
    assert(g_protocol_command.type == PROTOCOL_BOOT_CONFIRM &&
           g_protocol_command.args.confirm.version == 0x12345678U &&
           g_protocol_command.args.confirm.image_crc32 == 7U);
}

int main(void)
{
    uint8_t payload[PROTOCOL_PAGE_REQUEST_SIZE] = {0};
    yg_protocol_message_t request = {0};
    BspCanFrame request_frame = {0};
    BspCanFrame response_frame = {0};
    yg_protocol_message_t response = {0};
    BspCanFrame received = {0};
    uint32_t session_id = 0x11223344U;
    uint32_t request_id = 0x55667788U;

    assert(g_protocol_command.type == 0U);
    assert(fresh_link(7U));
    assert(Protocol_IsReady());

    /* 身份由组合层显式绑定；初始化不能再假定存在默认设备能力。 */
    ProtocolInfo info = {0};
    ProtocolCaps capabilities = {.features = 1U};
    assert(Protocol_SetInfo(&info, &capabilities));

    put_u32(payload, session_id);
    put_u32(payload + 4U, request_id);
    put_u16(payload + 8U, 0U);
    request.version = YG_PROTOCOL_VERSION;
    request.flags = YG_PROTOCOL_FLAGS_ACK_REQUEST;
    request.source_node = 1U;
    request.destination_node = 7U;
    request.message_type = PROTOCOL_GET_CAPS;
    request.sequence = 12U;
    request.payload_length = sizeof(payload);
    request.payload = payload;
    assert(encode_can_frame(&request, 2U, &request_frame) == YG_PROTOCOL_OK);

    {
        received.identifier = request_frame.identifier;
        received.length = request_frame.length;
        received.extended = request_frame.extended;
        received.remote = request_frame.remote;
        received.fd = request_frame.fd;
        received.bitrate_switch = request_frame.bitrate_switch;
        memcpy(received.data, request_frame.data, received.length);
        assert(Protocol_QueueReceivedFrame(&received));
    }

    Protocol_Process(100U);
    assert(sent);
    assert(sent_frame.extended && sent_frame.fd && sent_frame.bitrate_switch);
    response_frame.identifier = sent_frame.identifier;
    response_frame.length = sent_frame.length;
    response_frame.extended = sent_frame.extended;
    response_frame.remote = sent_frame.remote;
    response_frame.fd = sent_frame.fd;
    response_frame.bitrate_switch = sent_frame.bitrate_switch;
    memcpy(response_frame.data, sent_frame.data, sent_frame.length);
    assert(decode_can_frame(&response_frame, 1U, &response) == YG_PROTOCOL_OK);
    assert(response.source_node == 7U);
    assert(response.destination_node == 1U);
    assert(response.message_type == PROTOCOL_GET_CAPS);
    assert(response.sequence == 12U);
    assert((response.flags & YG_PROTOCOL_FLAGS_RESPONSE) != 0U);
    assert(response.payload_length == PROTOCOL_CAPS_PAGE0_SIZE);
    assert(response.payload[0] == 1U);
    assert(response.payload[1] == 0U);

    sent = false;
    assert(fresh_link(7U));
    request.flags = YG_PROTOCOL_FLAGS_ACK_REQUEST;
    request.source_node = 2U;
    request.message_type = PROTOCOL_STOP;
    request.payload_length = 0U;
    request.payload = NULL;
    assert(encode_can_frame(&request, 2U, &request_frame) == YG_PROTOCOL_OK);
    received.identifier = request_frame.identifier;
    received.length = request_frame.length;
    received.extended = request_frame.extended;
    received.remote = request_frame.remote;
    received.fd = request_frame.fd;
    received.bitrate_switch = request_frame.bitrate_switch;
    memcpy(received.data, request_frame.data, received.length);
    assert(Protocol_QueueReceivedFrame(&received));
    Protocol_Process(200U);
    assert(sent);
    response_frame.identifier = sent_frame.identifier;
    response_frame.length = sent_frame.length;
    response_frame.extended = sent_frame.extended;
    response_frame.remote = sent_frame.remote;
    response_frame.fd = sent_frame.fd;
    response_frame.bitrate_switch = sent_frame.bitrate_switch;
    memcpy(response_frame.data, sent_frame.data, sent_frame.length);
    assert(decode_can_frame(&response_frame, 2U, &response) == YG_PROTOCOL_OK);
    assert(response.message_type == PROTOCOL_STOP);
    assert(response.payload_length == PROTOCOL_REPLY_SIZE);
    assert(response.payload[0] == PROTOCOL_UNSUPPORTED);
    assert(response.payload[1] == 0U && response.payload[2] == 0U && response.payload[3] == 0U);

    /* 旧 111 不再注册；即使帧校验正确也不能触发控制或产生成功回复。 */
    sent = false;
    request.message_type = 111U;
    assert(encode_can_frame(&request, 2U, &request_frame) == YG_PROTOCOL_OK);
    received.identifier = request_frame.identifier;
    received.length = request_frame.length;
    memcpy(received.data, request_frame.data, received.length);
    assert(Protocol_QueueReceivedFrame(&received));
    Protocol_Process(201U);
    assert(!sent);

    /* 已删除的台架 INFO 请求经过完整收帧路径，也不得获得回复或调用电机。 */
    {
        uint8_t retired_payload[4] = {0U, 0U, 1U, 0U};
        unsigned calls_before = motor_calls;
        request.message_type = 0x7F00U;
        request.payload = retired_payload;
        request.payload_length = sizeof(retired_payload);
        enqueue_request(&request);
        Protocol_Process(202U);
        assert(!sent);
        assert(motor_calls == calls_before);
    }

    sent = false;
    assert(fresh_link(7U));
    request.flags = YG_PROTOCOL_FLAGS_ACK_REQUEST;
    request.source_node = 1U;
    request.message_type = PROTOCOL_GET_MOTOR_STATE;
    request.sequence = 13U;
    request.payload_length = 0U;
    request.payload = NULL;
    assert(encode_can_frame(&request, 2U, &request_frame) == YG_PROTOCOL_OK);
    received.identifier = request_frame.identifier;
    received.length = request_frame.length;
    received.extended = request_frame.extended;
    received.remote = request_frame.remote;
    received.fd = request_frame.fd;
    received.bitrate_switch = request_frame.bitrate_switch;
    memcpy(received.data, request_frame.data, received.length);
    assert(Protocol_QueueReceivedFrame(&received));
    Protocol_Process(300U);
    assert(sent);
    response_frame.identifier = sent_frame.identifier;
    response_frame.length = sent_frame.length;
    response_frame.extended = sent_frame.extended;
    response_frame.remote = sent_frame.remote;
    response_frame.fd = sent_frame.fd;
    response_frame.bitrate_switch = sent_frame.bitrate_switch;
    memcpy(response_frame.data, sent_frame.data, sent_frame.length);
    assert(decode_can_frame(&response_frame, 1U, &response) == YG_PROTOCOL_OK);
    assert(response.message_type == PROTOCOL_GET_MOTOR_STATE);
    assert(response.payload_length == PROTOCOL_FEEDBACK_SIZE);
    assert(response.payload[0] == 6U && response.payload[1] == 0U && response.payload[24] == 0U &&
           response.payload[42] == 7U);
    enable_route();
    start_route_contract();
    multimode_start_vectors();
    control_reply_and_feedback();
    irq_receive_budget();
    backlog_and_interrupt_handoff();
    unsupported_control_requests();
    power_route_contract();
    status_binding_reset();
    boot_confirm_and_info_contract();
    boot_ready_queue_contract();
    boot_inflight_deadline();
    rejected_frames_do_not_execute();
    parameter_wire_contract();
    runtime_start_boundary();
    last_command_watch();
    return 0;
}
