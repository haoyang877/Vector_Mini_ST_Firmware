/* CAN FD 组合层：后台路由，电机业务经显式绑定注入；不直接访问电机控制。 */
#include "yg_protocol_link.h"
#include "yg_protocol_request.h"
#include "yg_protocol_byte_order.h"

#include <stddef.h>
#include <string.h>

#include "critical_hw.h"

#include "yg_protocol_endpoint.h"
#include "yg_protocol_crc.h"
#include "yg_protocol_readonly.h"
#include "yg_protocol_readonly_payload.h"
#include "yg_protocol_transfer.h"
#include "yg_protocol_motor.h"
#include "yg_protocol_parameter.h"
#include "yg_protocol_boot.h"

/* APP 当前只发送单帧回复/反馈；TX 保留突发余量和控制回复+反馈的两个位置。
 * 通用端点仍可处理 1767B 消息，但 APP 不承诺一次排入 39 个发送分片。 */
#define YG_LINK_RX_QUEUE_CAPACITY 8U
#define YG_LINK_TX_QUEUE_CAPACITY 8U
#define YG_LINK_FRAGMENT_CAPACITY YG_PROTOCOL_MAX_MESSAGE_PAYLOAD
#define YG_LINK_MASTER_NODE_ID 2U
/* 只重放最近 32 个控制序号的原结果；更早的序号由水位拒绝，绝不重新执行。 */
#define YG_LINK_CONTROL_HISTORY_SIZE 32U
#define YG_LINK_BOOT_QUEUE_TIMEOUT_MS 100U

/* 仅56需要精确发送身份，旁记录不扩大每一条普通CAN帧。 */
typedef enum
{
    YG_BOOT_TX_NONE,
    YG_BOOT_TX_PREPARED,
    YG_BOOT_TX_QUEUED,
    YG_BOOT_TX_IN_FLIGHT
} yg_link_boot_tx_state_t;

typedef struct
{
    uint16_t sequence;
    uint16_t request_crc;
    uint16_t result;
    /* result=1 时复用为该请求的最终反馈结果；ACK 的 detail 仍为 0。 */
    uint16_t detail;
    bool valid;
} yg_link_control_history_t;

typedef struct
{
    uint8_t node_id;
    yg_protocol_readonly_info_page0_t info;
    uint8_t maintenance_pages[2][32];
    yg_protocol_readonly_caps_page0_t capabilities;
    const yg_protocol_readonly_device_status_t *device_status_source;
    yg_protocol_motor_service_t motor_service;
    yg_protocol_parameter_service_t parameter_service;
    yg_protocol_job_service_t job_service;
    BootEntryService boot_service;
    uint64_t boot_ticket;
    uint32_t service_now_ms;
    uint32_t boot_ready_ms;
    size_t boot_tx_slot;
    size_t base_route_count;
    yg_link_boot_tx_state_t boot_tx_state;
    uint16_t boot_reply_sequence;
    uint8_t boot_reply_destination;
    yg_protocol_motor_status_source_t empty_motor_status;
    const yg_protocol_motor_status_source_t *motor_status_source;
    yg_link_control_history_t control_history[YG_LINK_CONTROL_HISTORY_SIZE];
    uint16_t last_control_sequence;
    uint16_t feedback_sequence;
    bool device_description_available;
    bool control_sequence_seen;
    bool feedback_pending;
    bool control_completion_pending;
    uint8_t control_completion_source;
    uint16_t control_completion_sequence;
    uint8_t feedback_destination;
    uint8_t feedback_payload[YG_PROTOCOL_MOTOR_FEEDBACK_PAYLOAD_SIZE];
} yg_protocol_link_context_t;

static yg_protocol_link_context_t s_link_context;
static const yg_protocol_readonly_device_status_t s_empty_device_status = {
    .role = YG_PROTOCOL_DEVICE_ROLE_APP};
static yg_protocol_transfer_frame_t s_rx_storage[YG_LINK_RX_QUEUE_CAPACITY];
static yg_protocol_transfer_frame_t s_tx_storage[YG_LINK_TX_QUEUE_CAPACITY];
static uint8_t s_fragment_storage[YG_LINK_FRAGMENT_CAPACITY];
static yg_protocol_transfer_queue_t s_rx_queue;
static yg_protocol_transfer_queue_t s_tx_queue;
static yg_protocol_message_registry_t s_registry;
static yg_protocol_router_t s_router;
static yg_protocol_readonly_service_t s_readonly_service;
static yg_protocol_endpoint_t s_endpoint;
static bool s_ready;

static void write_result(yg_protocol_service_result_t *response, uint16_t code, uint16_t detail);

/* 只编码已经关联的值快照；与原始请求身份一起保留到 TX 队列接收。 */
static yg_protocol_result_t prepare_control_feedback(yg_protocol_link_context_t *link,
                                                     uint8_t destination,
                                                     uint16_t sequence,
                                                     uint16_t code)
{
    yg_protocol_motor_feedback_t feedback;
    if (yg_protocol_motor_feedback_from_source(
            link->motor_status_source, link->node_id, code, sequence, &feedback) !=
            YG_PROTOCOL_OK ||
        yg_protocol_motor_feedback_encode(
            &feedback, link->feedback_payload, sizeof(link->feedback_payload)) != YG_PROTOCOL_OK)
    {
        return YG_PROTOCOL_INVALID_FIELD;
    }
    link->feedback_destination = destination;
    link->feedback_pending = true;
    return YG_PROTOCOL_OK;
}

/* 终态来自同一前台快照中的所有者进展，不能以 ACK 或接收时刻猜测。 */
static void prepare_completed_control(yg_protocol_link_context_t *link)
{
    if (!link->control_completion_pending || link->feedback_pending)
    {
        return;
    }
    const MotorControlProgress *progress = &link->motor_status_source->control_progress;
    if (progress->source_node != link->control_completion_source ||
        progress->sequence != link->control_completion_sequence ||
        (progress->state != MOTOR_CONTROL_PROGRESS_APPLIED &&
         progress->state != MOTOR_CONTROL_PROGRESS_REJECTED))
    {
        return;
    }
    uint16_t code = 8U;
    if (progress->state == MOTOR_CONTROL_PROGRESS_APPLIED)
    {
        code = 0U;
    }
    else
    {
        switch (progress->result)
        {
        case MOTOR_CONTROL_INVALID:
            code = 4U;
            break;
        case MOTOR_CONTROL_DENIED:
            code = 5U;
            break;
        case MOTOR_CONTROL_BUSY:
            code = 6U;
            break;
        case MOTOR_CONTROL_OUT_OF_RANGE:
            code = 11U;
            break;
        case MOTOR_CONTROL_FAULT:
            code = 17U;
            break;
        default:
            break;
        }
    }
    if (prepare_control_feedback(link, progress->source_node, progress->sequence, code) !=
        YG_PROTOCOL_OK)
    {
        return;
    }
    yg_link_control_history_t *entry =
        &link->control_history[progress->sequence % YG_LINK_CONTROL_HISTORY_SIZE];
    if (entry->valid && entry->sequence == progress->sequence && entry->result == 1U)
    {
        entry->detail = code;
    }
    link->control_completion_pending = false;
}

/* 业务入口公共守卫：上下文/请求/结果指针非空且线格式有效，否则按无效字段拒绝。 */
static bool request_context_valid(const yg_protocol_link_context_t *link,
                                  const yg_protocol_message_t *request,
                                  const yg_protocol_service_result_t *result)
{
    return link != NULL && request != NULL && result != NULL &&
           YgProtocol_IsAckUnicastRequest(request);
}

static yg_protocol_result_t provide_unavailable(uint8_t *payload, uint32_t *payload_length)
{
    /* 缺少真实身份/能力也须给业务错误回复，不能让端点将内部错误当作丢帧。 */
    payload[0] = 6U;
    payload[1] = 0U;
    payload[2] = 0U;
    payload[3] = 0U;
    *payload_length = 4U;
    return YG_PROTOCOL_OK;
}

static yg_protocol_result_t provide_info(void *context,
                                         const yg_protocol_message_t *request,
                                         uint8_t *payload,
                                         uint32_t *payload_length)
{
    yg_protocol_readonly_page_request_t page_request;
    yg_protocol_link_context_t *link = context;
    size_t written = 0U;

    if (link == NULL ||
        yg_protocol_readonly_decode_page_request(request, &page_request) != YG_PROTOCOL_OK ||
        page_request.page > 2U)
    {
        return YG_PROTOCOL_INVALID_FIELD;
    }
    if (!link->device_description_available)
    {
        return provide_unavailable(payload, payload_length);
    }
    if (page_request.page != 0U)
    {
        memcpy(payload, link->maintenance_pages[page_request.page - 1U], 32U);
        *payload_length = 32U;
        return YG_PROTOCOL_OK;
    }
    if (yg_protocol_readonly_encode_info_page0(
            &link->info, payload, YG_PROTOCOL_MAX_PAYLOAD_SIZE, &written) != YG_PROTOCOL_OK)
    {
        return YG_PROTOCOL_BUFFER_TOO_SMALL;
    }
    *payload_length = (uint32_t)written;
    return YG_PROTOCOL_OK;
}

static yg_protocol_result_t provide_capabilities(void *context,
                                                 const yg_protocol_message_t *request,
                                                 uint8_t *payload,
                                                 uint32_t *payload_length)
{
    yg_protocol_readonly_page_request_t page_request;
    yg_protocol_link_context_t *link = context;
    size_t written = 0U;

    if (link == NULL ||
        yg_protocol_readonly_decode_page_request(request, &page_request) != YG_PROTOCOL_OK ||
        page_request.page != 0U)
    {
        return YG_PROTOCOL_INVALID_FIELD;
    }
    if (!link->device_description_available)
    {
        return provide_unavailable(payload, payload_length);
    }
    if (yg_protocol_readonly_encode_caps_page0(
            &link->capabilities, payload, YG_PROTOCOL_MAX_PAYLOAD_SIZE, &written) != YG_PROTOCOL_OK)
    {
        return YG_PROTOCOL_BUFFER_TOO_SMALL;
    }
    *payload_length = (uint32_t)written;
    return YG_PROTOCOL_OK;
}

static yg_protocol_result_t handle_device_status(void *context,
                                                 const yg_protocol_message_t *request,
                                                 yg_protocol_service_result_t *result)
{
    yg_protocol_link_context_t *link = context;
    if (!request_context_valid(link, request, result))
    {
        return YG_PROTOCOL_INVALID_FIELD;
    }
    if (request->payload_length != 0U)
    {
        write_result(result, 3U, 0U);
        return YG_PROTOCOL_OK;
    }
    if (yg_protocol_readonly_encode_device_status(link->device_status_source,
                                                  result->response_payload,
                                                  sizeof(result->response_payload),
                                                  NULL) != YG_PROTOCOL_OK)
    {
        write_result(result, 8U, 0U);
        return YG_PROTOCOL_OK;
    }
    result->response_payload_length = YG_PROTOCOL_READONLY_DEVICE_STATUS_SIZE;
    return YG_PROTOCOL_OK;
}

static yg_protocol_result_t handle_motor_action(void *context,
                                                const yg_protocol_message_t *request,
                                                yg_protocol_service_result_t *result)
{
    yg_protocol_motor_request_t command = {.operation = YG_PROTOCOL_MOTOR_STOP};
    yg_protocol_service_reply_t reply;
    yg_protocol_link_context_t *link = context;
    yg_protocol_result_t decoded;
    bool enabled;

    /* STOP 的非空请求在来源检查前拒绝，保留既有线路处理顺序。 */
    if (!request_context_valid(link, request, result) ||
        (request->message_type != YG_PROTOCOL_MOTOR_TYPE_ENABLE && request->payload_length != 0U))
    {
        return YG_PROTOCOL_INVALID_FIELD;
    }
    if (request->source_node != YG_LINK_MASTER_NODE_ID)
    {
        write_result(result, 7U, 0U);
        return YG_PROTOCOL_OK;
    }
    if (request->message_type == YG_PROTOCOL_MOTOR_TYPE_ENABLE)
    {
        decoded = yg_protocol_motor_decode_enable(request, &enabled);
        if (decoded != YG_PROTOCOL_OK)
        {
            write_result(result, decoded == YG_PROTOCOL_INVALID_LENGTH ? 3U : 4U, 0U);
            return YG_PROTOCOL_OK;
        }
        command.operation = enabled ? YG_PROTOCOL_MOTOR_ENABLE : YG_PROTOCOL_MOTOR_DISABLE;
    }
    else if (request->message_type == YG_PROTOCOL_MOTOR_TYPE_CLEAR_FAULT)
    {
        command.operation = YG_PROTOCOL_MOTOR_CLEAR_FAULT;
    }
    command.source_node = request->source_node;
    command.sequence = request->sequence;
    (void)yg_protocol_motor_call(&link->motor_service, &command, &reply);
    if (yg_protocol_motor_encode_reply(
            &reply, result->response_payload, sizeof(result->response_payload), NULL) !=
        YG_PROTOCOL_OK)
    {
        return YG_PROTOCOL_BUFFER_TOO_SMALL;
    }
    result->response_payload_length = YG_PROTOCOL_MOTOR_REPLY_SIZE;
    result->response_flags = YG_PROTOCOL_FLAGS_RESPONSE;
    return YG_PROTOCOL_OK;
}

static yg_protocol_result_t handle_motor_start(void *context,
                                               const yg_protocol_message_t *request,
                                               yg_protocol_service_result_t *result)
{
    yg_protocol_link_context_t *link = context;
    yg_protocol_motor_request_t command;
    yg_protocol_service_reply_t reply;
    yg_protocol_result_t decoded;
    if (!request_context_valid(link, request, result))
    {
        return YG_PROTOCOL_INVALID_FIELD;
    }
    if (request->source_node != YG_LINK_MASTER_NODE_ID)
    {
        write_result(result, 7U, 0U);
        return YG_PROTOCOL_OK;
    }
    decoded = yg_protocol_motor_decode_start(request, &command);
    if (decoded != YG_PROTOCOL_OK)
    {
        write_result(result, decoded == YG_PROTOCOL_INVALID_LENGTH ? 3U : 4U, 0U);
        return YG_PROTOCOL_OK;
    }
    if (command.mode != 1U && command.mode != 2U && command.mode != 3U && command.mode != 6U)
    {
        write_result(result, 2U, 0U);
        return YG_PROTOCOL_OK;
    }
    (void)yg_protocol_motor_call(&link->motor_service, &command, &reply);
    if (yg_protocol_motor_encode_reply(
            &reply, result->response_payload, sizeof(result->response_payload), NULL) !=
        YG_PROTOCOL_OK)
    {
        return YG_PROTOCOL_BUFFER_TOO_SMALL;
    }
    result->response_payload_length = YG_PROTOCOL_MOTOR_REPLY_SIZE;
    result->response_flags = YG_PROTOCOL_FLAGS_RESPONSE;
    return YG_PROTOCOL_OK;
}

static void write_result(yg_protocol_service_result_t *response, uint16_t code, uint16_t detail)
{
    YgProtocol_WriteU16Le(response->response_payload, code);
    YgProtocol_WriteU16Le(response->response_payload + 2U, detail);
    response->response_payload_length = YG_PROTOCOL_MOTOR_REPLY_SIZE;
}

static yg_protocol_result_t handle_motor_control(void *context,
                                                 const yg_protocol_message_t *request,
                                                 yg_protocol_service_result_t *result)
{
    yg_protocol_link_context_t *link = context;
    yg_protocol_motor_request_t command;
    yg_protocol_service_reply_t reply;
    yg_link_control_history_t *entry;
    yg_protocol_result_t decoded;
    uint16_t code, feedback_code, detail = 0U, digest = 0U;
    bool defer_feedback = false;
    if (!request_context_valid(link, request, result))
    {
        return YG_PROTOCOL_INVALID_FIELD;
    }
    code = 7U;
    feedback_code = code;
    if (request->source_node == YG_LINK_MASTER_NODE_ID)
    {
        decoded = yg_protocol_motor_decode_control(request, &command);
        code = decoded == YG_PROTOCOL_INVALID_LENGTH ? 3U : 4U;
        feedback_code = code;
        if (decoded == YG_PROTOCOL_OK)
        {
            digest = yg_protocol_crc16(request->payload, request->payload_length);
            entry = &link->control_history[request->sequence % YG_LINK_CONTROL_HISTORY_SIZE];
            if (entry->valid && entry->sequence == request->sequence)
            {
                code = entry->request_crc == digest ? entry->result : 4U;
                detail = entry->request_crc == digest && code != 1U ? entry->detail : 0U;
                feedback_code = entry->request_crc == digest && code == 1U ? entry->detail : code;
            }
            else if (link->control_sequence_seen &&
                     (request->sequence == link->last_control_sequence ||
                      (uint16_t)(request->sequence - link->last_control_sequence) >= 0x8000U))
            {
                code = 4U; /* 未缓存的相同或旧序号不得在终态发布后重新执行。 */
                feedback_code = code;
            }
            else
            {
                if (link->control_completion_pending)
                {
                    code = 6U;
                }
                else
                {
                    (void)yg_protocol_motor_call(&link->motor_service, &command, &reply);
                    code = yg_protocol_motor_result_code(reply.status);
                    detail = reply.detail;
                }
                feedback_code = code;
                /* 不能让拥塞拒绝覆盖仍待发布终态的历史槽。 */
                if (!link->control_completion_pending ||
                    request->sequence % YG_LINK_CONTROL_HISTORY_SIZE !=
                        link->control_completion_sequence % YG_LINK_CONTROL_HISTORY_SIZE)
                {
                    *entry = (yg_link_control_history_t){
                        request->sequence, digest, code, code == 1U ? 1U : detail, true};
                }
                if (code == 1U)
                {
                    link->control_completion_pending = true;
                    link->control_completion_source = request->source_node;
                    link->control_completion_sequence = request->sequence;
                    defer_feedback = true;
                    detail = 0U;
                }
                link->last_control_sequence = request->sequence;
                link->control_sequence_seen = true;
            }
        }
    }
    if (!defer_feedback &&
        prepare_control_feedback(link, request->source_node, request->sequence, feedback_code) !=
            YG_PROTOCOL_OK)
    {
        return YG_PROTOCOL_INVALID_FIELD;
    }
    write_result(result, code, detail);
    return YG_PROTOCOL_OK;
}

static yg_protocol_result_t provide_motor_status(void *context,
                                                 const yg_protocol_message_t *request,
                                                 uint8_t *payload,
                                                 uint32_t *payload_length)
{
    yg_protocol_link_context_t *link = context;

    if (link == NULL)
    {
        return YG_PROTOCOL_INVALID_ARGUMENT;
    }
    return yg_protocol_motor_status_provider(
        (void *)link->motor_status_source, request, payload, payload_length);
}

static yg_protocol_result_t handle_parameter(void *context,
                                             const yg_protocol_message_t *request,
                                             yg_protocol_service_result_t *result)
{
    yg_protocol_link_context_t *link = context;
    yg_protocol_parameter_request_t command;
    yg_protocol_service_reply_t reply;
    yg_protocol_result_t decoded;
    size_t written;
    if (!request_context_valid(link, request, result))
    {
        return YG_PROTOCOL_INVALID_FIELD;
    }
    if (request->source_node != YG_LINK_MASTER_NODE_ID)
    {
        write_result(result, 7U, 0U);
        return YG_PROTOCOL_OK;
    }
    decoded = yg_protocol_parameter_decode(request, &command);
    if (decoded != YG_PROTOCOL_OK)
    {
        write_result(
            result,
            decoded == YG_PROTOCOL_INVALID_LENGTH ? 3U : 4U,
            decoded == YG_PROTOCOL_INVALID_LENGTH ? 0U : YG_PROTOCOL_PARAMETER_DETAIL_BAD_FIELD);
        return YG_PROTOCOL_OK;
    }
    /* 幂等与持久作业由参数所有者掌握，通信不缓存一个尚未完成的“成功”。 */
    (void)yg_protocol_parameter_call(&link->parameter_service, &command, &reply);
    if (yg_protocol_parameter_encode_reply(&command,
                                           &reply,
                                           result->response_payload,
                                           sizeof(result->response_payload),
                                           &written) != YG_PROTOCOL_OK)
    {
        write_result(result, 8U, 0U);
        return YG_PROTOCOL_OK;
    }
    result->response_payload_length = (uint32_t)written;
    return YG_PROTOCOL_OK;
}

static void write_job_error(yg_protocol_service_result_t *response, uint16_t code, uint16_t detail)
{
    write_result(response, code, detail);
    memset(response->response_payload + 4U, 0, YG_PROTOCOL_JOB_REPLY_SIZE - 4U);
    response->response_payload_length = YG_PROTOCOL_JOB_REPLY_SIZE;
}

static yg_protocol_result_t handle_job(void *context,
                                       const yg_protocol_message_t *request,
                                       yg_protocol_service_result_t *result)
{
    yg_protocol_link_context_t *link = context;
    yg_protocol_job_request_t command;
    yg_protocol_job_reply_t reply;
    yg_protocol_result_t decoded;
    if (!request_context_valid(link, request, result))
    {
        return YG_PROTOCOL_INVALID_FIELD;
    }
    if (request->source_node != YG_LINK_MASTER_NODE_ID)
    {
        write_job_error(result, 7U, 0U);
        return YG_PROTOCOL_OK;
    }
    decoded = yg_protocol_job_decode(request, &command);
    if (decoded != YG_PROTOCOL_OK)
    {
        write_job_error(result,
                        decoded == YG_PROTOCOL_INVALID_LENGTH ? 3U : 4U,
                        YG_PROTOCOL_JOB_DETAIL_BAD_FIELD);
        return YG_PROTOCOL_OK;
    }
    (void)yg_protocol_job_call(&link->job_service, &command, &reply);
    if (yg_protocol_job_encode_reply(
            &reply, result->response_payload, sizeof(result->response_payload), NULL) !=
        YG_PROTOCOL_OK)
    {
        write_job_error(result, 8U, 0U);
        return YG_PROTOCOL_OK;
    }
    result->response_payload_length = YG_PROTOCOL_JOB_REPLY_SIZE;
    return YG_PROTOCOL_OK;
}

static yg_protocol_result_t handle_boot(void *context,
                                        const yg_protocol_message_t *request,
                                        yg_protocol_service_result_t *result)
{
    yg_protocol_link_context_t *link = context;
    BootEntryRequest command;
    BootEntryReply reply = {.result = BOOT_ENTRY_FAILED};
    yg_protocol_result_t decoded;
    if (!request_context_valid(link, request, result))
    {
        return YG_PROTOCOL_INVALID_FIELD;
    }
    if (request->source_node != YG_LINK_MASTER_NODE_ID)
    {
        write_result(result, 7U, 0U);
        return YG_PROTOCOL_OK;
    }
    if (request->message_type == YG_PROTOCOL_BOOT_TYPE_CONFIRM)
    {
        if (request->payload_length != 8U)
        {
            write_result(result, 3U, 0U);
        }
        else if (link->boot_service.confirm == NULL)
        {
            write_result(result, 2U, 0U);
        }
        else
        {
            bool confirmed =
                link->boot_service.confirm(link->boot_service.context,
                                           YgProtocol_ReadU32Le(request->payload),
                                           YgProtocol_ReadU32Le(request->payload + 4U));
            write_result(result, confirmed ? 0U : 5U, 0U);
        }
        return YG_PROTOCOL_OK;
    }
    decoded = yg_protocol_boot_decode(request, &command);
    if (decoded != YG_PROTOCOL_OK)
    {
        write_result(result, decoded == YG_PROTOCOL_INVALID_LENGTH ? 3U : 4U, 0U);
        return YG_PROTOCOL_OK;
    }
    if (link->boot_tx_state != YG_BOOT_TX_NONE)
    {
        /* 一次只发送一个带票据的READY；重发不能替换原票据或续长其截止。 */
        write_result(result, 6U, 0U);
        return YG_PROTOCOL_OK;
    }
    if (link->boot_service.request == NULL || link->boot_service.tx_complete == NULL)
    {
        write_result(result, 2U, 0U);
        return YG_PROTOCOL_OK;
    }
    link->boot_service.request(link->boot_service.context, &command, &reply);
    if (yg_protocol_boot_encode_reply(
            &reply, result->response_payload, sizeof(result->response_payload), NULL) !=
        YG_PROTOCOL_OK)
    {
        if (reply.ack_ticket != 0U)
        {
            link->boot_service.tx_complete(link->boot_service.context, reply.ack_ticket, false);
        }
        write_result(result, 8U, 0U);
        return YG_PROTOCOL_OK;
    }
    if (reply.result == BOOT_ENTRY_READY)
    {
        link->boot_ticket = reply.ack_ticket;
        link->boot_ready_ms = link->service_now_ms;
        link->boot_reply_sequence = request->sequence;
        link->boot_reply_destination = request->source_node;
        link->boot_tx_state = YG_BOOT_TX_PREPARED;
    }
    result->response_payload_length = YG_PROTOCOL_BOOT_REPLY_SIZE;
    return YG_PROTOCOL_OK;
}

bool YgProtocolLink_Init(uint8_t node_id)
{
    static const yg_protocol_route_t routes[] = {
        {YG_PROTOCOL_READONLY_TYPE_GET_INFO, yg_protocol_readonly_handle, &s_readonly_service},
        {YG_PROTOCOL_READONLY_TYPE_GET_CAPS, yg_protocol_readonly_handle, &s_readonly_service},
        {YG_PROTOCOL_READONLY_TYPE_GET_DEVICE_STATUS, handle_device_status, &s_link_context},
        {YG_PROTOCOL_READONLY_TYPE_GET_MOTOR_STATE,
         yg_protocol_readonly_handle,
         &s_readonly_service},
        {YG_PROTOCOL_MOTOR_TYPE_ENABLE, handle_motor_action, &s_link_context},
        {YG_PROTOCOL_MOTOR_TYPE_CLEAR_FAULT, handle_motor_action, &s_link_context},
        {YG_PROTOCOL_MOTOR_TYPE_STOP, handle_motor_action, &s_link_context},
        {YG_PROTOCOL_MOTOR_TYPE_CONTROL, handle_motor_control, &s_link_context},
        {YG_PROTOCOL_MOTOR_TYPE_START, handle_motor_start, &s_link_context},
        {YG_PROTOCOL_PARAMETER_TYPE_READ, handle_parameter, &s_link_context},
        {YG_PROTOCOL_PARAMETER_TYPE_WRITE, handle_parameter, &s_link_context},
        {YG_PROTOCOL_PARAMETER_TYPE_SAVE, handle_parameter, &s_link_context},
        {YG_PROTOCOL_JOB_TYPE_START, handle_job, &s_link_context},
        {YG_PROTOCOL_JOB_TYPE_STATUS, handle_job, &s_link_context},
        {YG_PROTOCOL_JOB_TYPE_CANCEL, handle_job, &s_link_context},
        {YG_PROTOCOL_BOOT_TYPE_ENTER, handle_boot, &s_link_context},
        {YG_PROTOCOL_BOOT_TYPE_CONFIRM, handle_boot, &s_link_context},
    };
    yg_protocol_readonly_service_t configuration = {0};
    yg_protocol_endpoint_config_t endpoint_config;

    if (s_ready && s_link_context.boot_tx_state != YG_BOOT_TX_NONE)
    {
        /* 初始化不能丢弃尚未完成的物理票据；由所有者/平台先确认失败或完成。 */
        return false;
    }
    s_ready = false;
    if (node_id == YG_PROTOCOL_BROADCAST_NODE_ID ||
        !yg_protocol_transfer_queue_init(&s_rx_queue, s_rx_storage, YG_LINK_RX_QUEUE_CAPACITY) ||
        !yg_protocol_transfer_queue_init(&s_tx_queue, s_tx_storage, YG_LINK_TX_QUEUE_CAPACITY) ||
        yg_protocol_product_registry_init(&s_registry) != YG_PROTOCOL_OK)
    {
        return false;
    }

    /* 初始化一次撤销全部旧绑定、历史和待发状态；此时 RX 尚未启动。 */
    memset(&s_link_context, 0, sizeof(s_link_context));
    s_link_context.node_id = node_id;
    s_link_context.motor_status_source = &s_link_context.empty_motor_status;
    s_link_context.device_status_source = &s_empty_device_status;
    s_link_context.base_route_count = sizeof(routes) / sizeof(routes[0]) - 2U;

    configuration.context = &s_link_context;
    configuration.protocol_info_type = YG_PROTOCOL_READONLY_TYPE_GET_INFO;
    configuration.capabilities_type = YG_PROTOCOL_READONLY_TYPE_GET_CAPS;
    configuration.device_info_type = YG_PROTOCOL_READONLY_TYPE_DISABLED;
    configuration.motor_status_type = YG_PROTOCOL_READONLY_TYPE_GET_MOTOR_STATE;
    configuration.protocol_info = provide_info;
    configuration.capabilities = provide_capabilities;
    configuration.motor_status = provide_motor_status;
    if (yg_protocol_readonly_init(&s_readonly_service, &configuration) != YG_PROTOCOL_OK ||
        yg_protocol_router_init(&s_router, &s_registry, routes, s_link_context.base_route_count) !=
            YG_PROTOCOL_OK)
    {
        return false;
    }

    endpoint_config.local_node = node_id;
    endpoint_config.reply_priority = 3U;
    endpoint_config.fragment_timeout_ms = 200U;
    endpoint_config.rx = &s_rx_queue;
    endpoint_config.tx = &s_tx_queue;
    endpoint_config.router = &s_router;
    endpoint_config.fragment_storage = s_fragment_storage;
    endpoint_config.fragment_capacity = sizeof(s_fragment_storage);
    if (yg_protocol_endpoint_init(&s_endpoint, &endpoint_config) != YG_PROTOCOL_OK)
    {
        return false;
    }
    s_ready = true;
    return true;
}

bool YgProtocolLink_SetMaintenanceInfoPage(uint16_t page, const uint8_t payload[32])
{
    if (!s_ready || payload == NULL || page < 1U || page > 2U)
    {
        return false;
    }
    memcpy(s_link_context.maintenance_pages[page - 1U], payload, 32U);
    return true;
}

bool YgProtocolLink_SetDeviceDescription(const yg_protocol_readonly_info_page0_t *info,
                                         const yg_protocol_readonly_caps_page0_t *capabilities)
{
    if (!s_ready || (info == NULL) != (capabilities == NULL))
    {
        return false;
    }
    s_link_context.info = info != NULL ? *info : (yg_protocol_readonly_info_page0_t){0};
    s_link_context.capabilities =
        capabilities != NULL ? *capabilities : (yg_protocol_readonly_caps_page0_t){0};
    s_link_context.device_description_available = info != NULL;
    return true;
}

bool YgProtocolLink_BindDeviceStatusSource(const yg_protocol_readonly_device_status_t *source)
{
    if (!s_ready)
    {
        return false;
    }
    s_link_context.device_status_source = source != NULL ? source : &s_empty_device_status;
    return true;
}

bool YgProtocolLink_BindMotorService(const yg_protocol_motor_service_t *service)
{
    if (!s_ready)
    {
        return false;
    }
    s_link_context.motor_service = service != NULL ? *service : (yg_protocol_motor_service_t){0};
    return true;
}

bool YgProtocolLink_BindParameterService(const yg_protocol_parameter_service_t *service)
{
    if (!s_ready)
    {
        return false;
    }
    s_link_context.parameter_service =
        service != NULL ? *service : (yg_protocol_parameter_service_t){0};
    return true;
}

bool YgProtocolLink_BindJobService(const yg_protocol_job_service_t *service)
{
    if (!s_ready)
    {
        return false;
    }
    s_link_context.job_service = service != NULL ? *service : (yg_protocol_job_service_t){0};
    return true;
}

bool YgProtocolLink_BindBootService(const BootEntryService *service)
{
    if (!s_ready || s_link_context.boot_tx_state != YG_BOOT_TX_NONE ||
        s_endpoint.response_pending || yg_protocol_transfer_queue_count(&s_rx_queue) != 0U ||
        yg_protocol_transfer_queue_count(&s_tx_queue) != 0U ||
        (service != NULL && (service->request == NULL || service->tx_complete == NULL)))
    {
        return false;
    }
    const size_t count = s_link_context.base_route_count + (service != NULL ? 2U : 0U);
    if (yg_protocol_router_init(&s_router, &s_registry, s_router.routes, count) != YG_PROTOCOL_OK)
    {
        return false;
    }
    s_link_context.boot_service = service != NULL ? *service : (BootEntryService){0};
    return true;
}

bool YgProtocolLink_BindMotorStatusSource(const yg_protocol_motor_status_source_t *source)
{
    if (!s_ready)
    {
        return false;
    }
    s_link_context.motor_status_source =
        source != NULL ? source : &s_link_context.empty_motor_status;
    return true;
}

bool YgProtocolLink_OnRxFrame(const CommHwCanFrame *frame)
{
    uint32_t state;
    bool accepted;
    if (!s_ready || frame == NULL || !frame->extended || !frame->fd || !frame->bitrate_switch ||
        frame->remote)
    {
        return false;
    }
    state = critical_hw_enter();
    accepted = yg_protocol_canfd_receive(&s_rx_queue, frame) == YG_PROTOCOL_OK;
    critical_hw_exit(state);
    return accepted;
}

static yg_protocol_result_t send_pending_feedback(void)
{
    yg_protocol_message_t feedback = {0};
    yg_protocol_result_t result;
    prepare_completed_control(&s_link_context);
    if (!s_link_context.feedback_pending)
    {
        return YG_PROTOCOL_OK;
    }
    feedback.version = YG_PROTOCOL_VERSION;
    feedback.source_node = s_link_context.node_id;
    feedback.destination_node = s_link_context.feedback_destination;
    feedback.message_type = YG_PROTOCOL_READONLY_TYPE_MOTION_FEEDBACK;
    feedback.sequence = s_link_context.feedback_sequence;
    feedback.payload_length = YG_PROTOCOL_MOTOR_FEEDBACK_PAYLOAD_SIZE;
    feedback.payload = s_link_context.feedback_payload;
    result = yg_protocol_endpoint_send(&s_endpoint, &feedback, 3U);
    if (result == YG_PROTOCOL_OK)
    {
        ++s_link_context.feedback_sequence;
        s_link_context.feedback_pending = false;
    }
    return result;
}

static void finish_boot_ticket(bool success)
{
    const uint64_t ticket = s_link_context.boot_ticket;
    s_link_context.boot_ticket = 0U;
    s_link_context.boot_tx_state = YG_BOOT_TX_NONE;
    if (ticket != 0U && s_link_context.boot_service.tx_complete != NULL)
    {
        s_link_context.boot_service.tx_complete(
            s_link_context.boot_service.context, ticket, success);
    }
}

static void service_boot_completion(void)
{
    CommHwCanTxCompletion completion;
    for (uint8_t received = 0U; received < 8U; ++received)
    {
        if (!comm_hw_can_take_tx_completion(&completion))
        {
            break;
        }
        if (s_link_context.boot_tx_state == YG_BOOT_TX_IN_FLIGHT &&
            completion.ticket == s_link_context.boot_ticket && completion.ticket != 0U)
        {
            finish_boot_ticket(completion.success);
        }
        /* 旧票据或其他帧事件不能使当前交接完成。 */
    }
}

static void capture_boot_reply_slot(size_t previous_head, yg_protocol_result_t result)
{
    if (s_link_context.boot_tx_state != YG_BOOT_TX_PREPARED || s_endpoint.response_pending)
    {
        return;
    }
    if (result != YG_PROTOCOL_OK || s_tx_queue.head == previous_head)
    {
        finish_boot_ticket(false);
        return;
    }
    /* head为入队位置、tail为出队位置；此位置即当前READY，不搜索相似ID/seq的旧帧。 */
    s_link_context.boot_tx_slot = previous_head;
    s_link_context.boot_tx_state = YG_BOOT_TX_QUEUED;
}

static bool is_boot_ready_frame(const yg_protocol_transfer_frame_t *frame)
{
    yg_protocol_message_t response;
    return yg_protocol_canfd_unpack(frame, s_link_context.boot_reply_destination, &response) ==
               YG_PROTOCOL_OK &&
           response.message_type == YG_PROTOCOL_BOOT_TYPE_ENTER &&
           response.flags == YG_PROTOCOL_FLAGS_RESPONSE &&
           response.source_node == s_link_context.node_id &&
           response.sequence == s_link_context.boot_reply_sequence &&
           response.payload_length == YG_PROTOCOL_BOOT_REPLY_SIZE && response.payload[0] == 1U &&
           response.payload[1] == 0U && response.payload[2] == 0U && response.payload[3] == 0U;
}

static void discard_queued_boot_reply(void)
{
    size_t slot = s_tx_queue.tail;
    for (size_t i = 0U; i < s_tx_queue.count; ++i)
    {
        if (slot == s_link_context.boot_tx_slot)
        {
            if (!is_boot_ready_frame(&s_tx_queue.storage[slot]))
            {
                return;
            }
            const size_t last = (s_tx_queue.head + s_tx_queue.capacity - 1U) % s_tx_queue.capacity;
            /* 只有一条带票据回复；按当前队列容量有界移动，保持其余帧顺序。 */
            while (slot != last)
            {
                size_t next = (slot + 1U) % s_tx_queue.capacity;
                s_tx_queue.storage[slot] = s_tx_queue.storage[next];
                slot = next;
            }
            s_tx_queue.head = last;
            --s_tx_queue.count;
            return;
        }
        slot = (slot + 1U) % s_tx_queue.capacity;
    }
}

static void expire_boot_reply(uint32_t now_ms)
{
    if ((s_link_context.boot_tx_state != YG_BOOT_TX_PREPARED &&
         s_link_context.boot_tx_state != YG_BOOT_TX_QUEUED) ||
        (uint32_t)(now_ms - s_link_context.boot_ready_ms) < YG_LINK_BOOT_QUEUE_TIMEOUT_MS)
    {
        return;
    }
    if (s_link_context.boot_tx_state == YG_BOOT_TX_PREPARED)
    {
        if (s_endpoint.response_pending &&
            s_endpoint.pending_response.message_type == YG_PROTOCOL_BOOT_TYPE_ENTER &&
            s_endpoint.pending_response.sequence == s_link_context.boot_reply_sequence &&
            s_endpoint.pending_response.destination_node == s_link_context.boot_reply_destination)
        {
            s_endpoint.response_pending = false;
        }
    }
    else
    {
        discard_queued_boot_reply();
    }
    finish_boot_ticket(false);
}

static yg_protocol_result_t send_one(void)
{
    yg_protocol_transfer_frame_t frame;
    CommHwCanFrame hardware = {0};
    if (s_link_context.boot_tx_state != YG_BOOT_TX_QUEUED ||
        s_tx_queue.tail != s_link_context.boot_tx_slot)
    {
        return yg_protocol_canfd_send_one(&s_tx_queue);
    }
    if (!yg_protocol_transfer_queue_peek(&s_tx_queue, &frame))
    {
        finish_boot_ticket(false);
        return YG_PROTOCOL_QUEUE_EMPTY;
    }
    /* 复用既有承载校验，避免位置关联被后续队列改动错误指向普通回复。 */
    if (!is_boot_ready_frame(&frame))
    {
        (void)yg_protocol_transfer_queue_pop(&s_tx_queue, &frame);
        finish_boot_ticket(false);
        return YG_PROTOCOL_INVALID_CAN_FRAME;
    }
    hardware.identifier = frame.identifier;
    hardware.length = frame.length;
    hardware.extended = frame.extended;
    hardware.remote = frame.remote;
    hardware.fd = frame.fd;
    hardware.bitrate_switch = frame.bitrate_switch;
    memcpy(hardware.data, frame.data, frame.length);
    if (!comm_hw_can_try_send_tracked_frame(&hardware, s_link_context.boot_ticket))
    {
        return YG_PROTOCOL_TRANSPORT_BUSY;
    }
    (void)yg_protocol_transfer_queue_pop(&s_tx_queue, &frame);
    s_link_context.boot_tx_state = YG_BOOT_TX_IN_FLIGHT;
    return YG_PROTOCOL_OK;
}

void YgProtocolLink_Service(uint32_t now_ms)
{
    uint8_t processed = 0U;
    uint8_t sent = 0U;

    if (!s_ready)
    {
        return;
    }
    service_boot_completion();
    s_link_context.service_now_ms = now_ms;
    expire_boot_reply(now_ms);
    while (processed < 8U)
    {
        yg_protocol_transfer_frame_t frame;
        yg_protocol_result_t result;
        size_t previous_head;
        if (!s_endpoint.response_pending && send_pending_feedback() != YG_PROTOCOL_OK)
        {
            break;
        }
        if (s_endpoint.response_pending)
        {
            previous_head = s_tx_queue.head;
            result = yg_protocol_endpoint_process_frame(&s_endpoint, NULL, now_ms);
        }
        else
        {
            /* 116 执行业务前，为同 Type 回复和 124 反馈各留一帧。 */
            if (YG_LINK_TX_QUEUE_CAPACITY - yg_protocol_transfer_queue_count(&s_tx_queue) < 2U)
            {
                break;
            }
            uint32_t state = critical_hw_enter();
            bool received = yg_protocol_transfer_queue_pop(&s_rx_queue, &frame);
            critical_hw_exit(state);
            if (!received)
            {
                break;
            }
            /* 临界区只复制帧；CRC、路由和业务处理始终允许电机中断抢占。 */
            previous_head = s_tx_queue.head;
            result = yg_protocol_endpoint_process_frame(&s_endpoint, &frame, now_ms);
        }
        capture_boot_reply_slot(previous_head, result);
        if (result == YG_PROTOCOL_QUEUE_FULL)
        {
            break;
        }
        if (!s_endpoint.response_pending && send_pending_feedback() != YG_PROTOCOL_OK)
        {
            break;
        }
        ++processed;
    }
    while (sent < 8U && send_one() == YG_PROTOCOL_OK)
    {
        ++sent;
    }
}

bool YgProtocolLink_IsReady(void)
{
    return s_ready;
}
