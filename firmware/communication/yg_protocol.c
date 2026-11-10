#include "yg_protocol.h"
#include "yg_protocol_can_id.h"
#include "yg_protocol_frame_codec.h"
#include <limits.h>
#include <string.h>

/* 普通通信：CAN 入队、前台解码、命令执行和回复编码。
 * RX 是中断到前台的单生产者队列；TX 只在前台访问。Loader 状态归 yg_protocol_boot.c。 */
#define PROCESS_BUDGET 8U
#define RX_INTERRUPT_BUDGET 3U
#define REPLY_PRIORITY 3U

static uint8_t align_can_frame_length(size_t length);
static yg_protocol_result_t
encode_can_frame(const yg_protocol_message_t *message, uint8_t priority, BspCanFrame *frame);
static yg_protocol_result_t
decode_can_frame(const BspCanFrame *frame, uint8_t local_node, yg_protocol_message_t *message);
static bool queue_push(CanQueue *queue, const BspCanFrame *frame);
static yg_protocol_result_t handle_can_request(ProtocolEndpoint *endpoint,
                                               const BspCanFrame *frame);
static yg_protocol_result_t handle_command(ProtocolEndpoint *endpoint, ProtocolExchange *exchange);
static yg_protocol_result_t decode_page_request(const yg_protocol_message_t *message,
                                                PageRequest *request);
static yg_protocol_result_t encode_device_info(const ProtocolInfo *page,
                                               ProtocolResponse *response);
static yg_protocol_result_t handle_info_query(ProtocolEndpoint *endpoint,
                                              ProtocolExchange *exchange);
static yg_protocol_result_t encode_device_caps(const ProtocolCaps *page,
                                               ProtocolResponse *response);
static yg_protocol_result_t handle_caps_query(ProtocolEndpoint *endpoint,
                                              ProtocolExchange *exchange);
static yg_protocol_result_t encode_device_status(const ProtocolDeviceStatus *status,
                                                 ProtocolResponse *response);
static yg_protocol_result_t handle_device_status_query(ProtocolEndpoint *endpoint,
                                                       ProtocolExchange *exchange);
static yg_protocol_result_t decode_target_command(const yg_protocol_message_t *message,
                                                  ProtocolCommand *request);
static yg_protocol_result_t handle_state_command(ProtocolEndpoint *endpoint,
                                                 ProtocolExchange *exchange);
static yg_protocol_result_t handle_start_command(ProtocolEndpoint *endpoint,
                                                 ProtocolExchange *exchange);
static yg_protocol_result_t handle_target_command(ProtocolEndpoint *endpoint,
                                                  ProtocolExchange *exchange);
static yg_protocol_result_t encode_motor_feedback(const ProtocolFeedback *feedback,
                                                  ProtocolResponse *response);
static yg_protocol_result_t handle_motor_status_query(ProtocolEndpoint *endpoint,
                                                      ProtocolExchange *exchange);
static yg_protocol_result_t queue_pending_feedback(ProtocolEndpoint *endpoint);
static yg_protocol_result_t decode_job_command(const yg_protocol_message_t *message,
                                               ProtocolJobRequest *request);
static yg_protocol_result_t encode_job_reply(const ProtocolJobReply *reply,
                                             ProtocolResponse *response);
static void write_job_error_reply(ProtocolResponse *response, ProtocolCode code, uint16_t detail);
static yg_protocol_result_t handle_job_command(ProtocolEndpoint *endpoint,
                                               ProtocolExchange *exchange);
static yg_protocol_result_t reject_parameter_command(ProtocolEndpoint *endpoint,
                                                     ProtocolExchange *exchange);

/* CAN 接收与前台入口 */

bool Protocol_Init(ProtocolEndpoint *endpoint, uint8_t node_id)
{
    if (endpoint == NULL || endpoint->ready || endpoint->started)
    {
        return false;
    }
    if (node_id == YG_PROTOCOL_BROADCAST_NODE_ID)
    {
        return false;
    }
    endpoint->rx_queue =
        (CanQueue){.storage = endpoint->rx_storage, .capacity = PROTOCOL_QUEUE_CAPACITY};
    endpoint->tx_queue =
        (CanQueue){.storage = endpoint->tx_storage, .capacity = PROTOCOL_QUEUE_CAPACITY};
    endpoint->node_id = node_id;
    endpoint->ready = true;
    return true;
}

bool Protocol_IsReady(ProtocolEndpoint *endpoint)
{
    return endpoint->ready;
}

bool Protocol_Start(ProtocolEndpoint *endpoint)
{
    if (!endpoint->ready)
    {
        return false;
    }
    if (!endpoint->started)
    {
        BspCan_Start();
        endpoint->started = true;
    }
    return true;
}

bool Protocol_QueueReceivedFrame(ProtocolEndpoint *endpoint, const BspCanFrame *frame)
{
    /* ISR 仅复制整帧；前台在解码时统一检查格式和 CRC。 */
    return endpoint->ready && frame != NULL && queue_push(&endpoint->rx_queue, frame);
}

void Protocol_RxInterrupt(ProtocolEndpoint *endpoint)
{
    BspCanFrame frame;

    /* 每次最多读取 3 帧，持续输入不能把一次 ISR 拉成无界循环。 */
    for (uint8_t count = 0U; count < RX_INTERRUPT_BUDGET; ++count)
    {
        if (!BspCan_Receive(&frame))
        {
            break;
        }
        /* 中断只复制入队，帧格式、CRC 与命令处理留在前台。 */
        (void)Protocol_QueueReceivedFrame(endpoint, &frame);
    }
}

void Protocol_Process(ProtocolEndpoint *endpoint, uint32_t now_ms)
{
    uint8_t processed = 0U;
    uint8_t sent = 0U;

    if (!endpoint->started)
    {
        return;
    }
    ProtocolBoot_Process(endpoint, &endpoint->tx_queue, now_ms);
    /* 先确认或作废上一条升级回复，再处理本轮请求。 */
    while (processed < PROCESS_BUDGET)
    {
        BspCanFrame frame;
        yg_protocol_result_t result;
        size_t previous_head;
        if (queue_pending_feedback(endpoint) != YG_PROTOCOL_OK)
        {
            break;
        }
        /* 先留出回复及反馈空间；拥塞时保留 RX，不重新执行业务。 */
        if (PROTOCOL_QUEUE_CAPACITY - queue_count(&endpoint->tx_queue) < 2U ||
            !queue_pop(&endpoint->rx_queue, &frame))
        {
            break;
        }
        previous_head = endpoint->tx_queue.head;
        result = handle_can_request(endpoint, &frame);
        ProtocolBoot_TrackReply(endpoint, &endpoint->tx_queue, previous_head, result, now_ms);
        if (result == YG_PROTOCOL_QUEUE_FULL)
        {
            break;
        }
        if (queue_pending_feedback(endpoint) != YG_PROTOCOL_OK)
        {
            break;
        }
        ++processed;
    }
    /* 最后提交排好的回复；硬件忙时留到下一轮。 */
    while (sent < PROCESS_BUDGET &&
           ProtocolBoot_SendNext(endpoint, &endpoint->tx_queue) == YG_PROTOCOL_OK)
    {
        ++sent;
    }
}

yg_protocol_result_t Protocol_QueueMessage(ProtocolEndpoint *endpoint,
                                           const yg_protocol_message_t *message,
                                           uint8_t priority)
{
    BspCanFrame frame;
    if (message == NULL || message->source_node != endpoint->node_id)
    {
        return YG_PROTOCOL_INVALID_NODE_ID;
    }
    yg_protocol_result_t result = encode_can_frame(message, priority, &frame);
    if (result != YG_PROTOCOL_OK)
    {
        return result;
    }
    return queue_push(&endpoint->tx_queue, &frame) ? YG_PROTOCOL_OK : YG_PROTOCOL_QUEUE_FULL;
}

/* 帧校验、编码与唯一命令分发 */

static uint8_t align_can_frame_length(size_t length)
{
    static const uint8_t lengths[] = {20U, 24U, 32U, 48U, 64U};
    for (size_t i = 0U; i < sizeof(lengths); ++i)
    {
        if (length <= lengths[i])
        {
            return lengths[i];
        }
    }
    return 0U;
}

static yg_protocol_result_t
encode_can_frame(const yg_protocol_message_t *message, uint8_t priority, BspCanFrame *frame)
{
    BspCanFrame candidate = {0};
    yg_protocol_can_id_fields_t fields;
    size_t length;
    yg_protocol_result_t result;
    if (message->source_node == YG_PROTOCOL_BROADCAST_NODE_ID || priority > 7U)
    {
        return YG_PROTOCOL_INVALID_CAN_FRAME;
    }
    fields.priority = priority;
    fields.source_node = message->source_node;
    fields.destination_node = message->destination_node;
    result = yg_protocol_can_id_encode(&fields, &candidate.identifier);
    if (result != YG_PROTOCOL_OK)
    {
        return result;
    }
    result = yg_protocol_frame_encode(message, candidate.data, sizeof(candidate.data), &length);
    if (result != YG_PROTOCOL_OK)
    {
        return result;
    }
    candidate.length = align_can_frame_length(length);
    candidate.extended = true;
    candidate.fd = true;
    candidate.bitrate_switch = true;
    *frame = candidate;
    return YG_PROTOCOL_OK;
}

static yg_protocol_result_t
decode_can_frame(const BspCanFrame *frame, uint8_t local_node, yg_protocol_message_t *message)
{
    yg_protocol_message_t candidate;
    yg_protocol_result_t result;
    uint32_t payload_length;
    size_t length;
    if (!frame->extended || frame->remote || !frame->fd || !frame->bitrate_switch ||
        frame->length < 20U || frame->length > 64U ||
        align_can_frame_length(frame->length) != frame->length)
    {
        return YG_PROTOCOL_INVALID_CAN_FRAME;
    }
    yg_protocol_can_id_fields_t fields;
    if (yg_protocol_can_id_decode(frame->identifier, &fields) != YG_PROTOCOL_OK ||
        fields.source_node == YG_PROTOCOL_BROADCAST_NODE_ID)
    {
        return YG_PROTOCOL_INVALID_CAN_FRAME;
    }
    /* 先限制不可信长度再求和；codec 随后检查完整头部和双 CRC。 */
    payload_length = YgProtocol_ReadU32Le(frame->data + 10U);
    if (payload_length > YG_PROTOCOL_MAX_PAYLOAD_SIZE)
    {
        return YG_PROTOCOL_INVALID_LENGTH;
    }
    length = YG_PROTOCOL_HEADER_SIZE + payload_length + YG_PROTOCOL_TRAILER_SIZE;
    if (align_can_frame_length(length) != frame->length)
    {
        return YG_PROTOCOL_INVALID_LENGTH;
    }
    result = yg_protocol_frame_decode(frame->data, length, &candidate);
    if (result != YG_PROTOCOL_OK)
    {
        return result;
    }
    if (candidate.source_node != fields.source_node ||
        candidate.destination_node != fields.destination_node ||
        (candidate.destination_node != local_node &&
         candidate.destination_node != YG_PROTOCOL_BROADCAST_NODE_ID))
    {
        return YG_PROTOCOL_INVALID_NODE_ID;
    }
    for (size_t i = length; i < frame->length; ++i)
    {
        if (frame->data[i] != 0U)
        {
            return YG_PROTOCOL_INVALID_PADDING;
        }
    }
    *message = candidate;
    return YG_PROTOCOL_OK;
}

static bool queue_push(CanQueue *queue, const BspCanFrame *frame)
{
    if (queue_count(queue) >= queue->capacity)
    {
        ++queue->dropped;
        return false;
    }
    BspSystem_MemoryBarrier();
    queue->storage[queue->head] = *frame;
    queue->head = (queue->head + 1U) % queue->capacity;
    BspSystem_MemoryBarrier();
    ++queue->produced;
    return true;
}

static yg_protocol_result_t handle_can_request(ProtocolEndpoint *endpoint, const BspCanFrame *frame)
{
    ProtocolExchange exchange = {0};
    yg_protocol_result_t result = decode_can_frame(frame, endpoint->node_id, &exchange.request);
    if (result != YG_PROTOCOL_OK)
    {
        return result;
    }
    /* APP 只处理单帧，分片属于独立 Loader。 */
    if ((exchange.request.flags & YG_PROTOCOL_FLAGS_FRAGMENT_MASK) != 0U)
    {
        return YG_PROTOCOL_INVALID_LENGTH;
    }
    result = handle_command(endpoint, &exchange);
    if (result != YG_PROTOCOL_OK)
    {
        return result;
    }
    const yg_protocol_message_t response = {
        .version = exchange.request.version,
        .flags = YG_PROTOCOL_FLAGS_RESPONSE,
        .source_node = endpoint->node_id,
        .destination_node = exchange.request.source_node,
        .message_type = exchange.request.message_type,
        .sequence = exchange.request.sequence,
        .payload_length = exchange.reply.length,
        .payload = exchange.reply.data,
    };
    return Protocol_QueueMessage(endpoint, &response, REPLY_PRIORITY);
}

void Protocol_RecordCommand(ProtocolEndpoint *endpoint,
                            const yg_protocol_message_t *request,
                            ProtocolCommandArgs args)
{
    endpoint->command = (ProtocolReceivedCommand){
        .type = request->message_type,
        .sequence = request->sequence,
        .source_node = request->source_node,
        .destination_node = request->destination_node,
        .flags = request->flags,
        .args = args,
    };
}

static yg_protocol_result_t handle_command(ProtocolEndpoint *endpoint, ProtocolExchange *exchange)
{
    const yg_protocol_message_t *request = &exchange->request;
    if (!YgProtocol_IsAckUnicastRequest(request))
    {
        return YG_PROTOCOL_INVALID_FIELD;
    }
    switch (request->message_type)
    {
    case PROTOCOL_GET_INFO:
        return handle_info_query(endpoint, exchange);
    case PROTOCOL_GET_CAPS:
        return handle_caps_query(endpoint, exchange);
    case PROTOCOL_GET_DEVICE_STATUS:
        return handle_device_status_query(endpoint, exchange);
    case PROTOCOL_BOOT_ENTER:
    case PROTOCOL_BOOT_CONFIRM:
        return ProtocolBoot_HandleCommand(endpoint, exchange);
    case PROTOCOL_GET_MOTOR_STATE:
        return handle_motor_status_query(endpoint, exchange);
    case PROTOCOL_ENABLE:
    case PROTOCOL_CLEAR_FAULT:
    case PROTOCOL_STOP:
        return handle_state_command(endpoint, exchange);
    case PROTOCOL_CONTROL:
        return handle_target_command(endpoint, exchange);
    case PROTOCOL_RENEW_MOTION:
    {
        const yg_protocol_message_t *request = &exchange->request;
        ProtocolReply reply = {.status = PROTOCOL_UNSUPPORTED};
        if (request->source_node != PROTOCOL_MASTER_ID)
        {
            Protocol_WriteResult(&exchange->reply, PROTOCOL_RESULT_FORBIDDEN, 0U);
            return YG_PROTOCOL_OK;
        }
        if (request->payload == NULL || request->payload_length != 4U)
        {
            Protocol_WriteResult(&exchange->reply, PROTOCOL_RESULT_BAD_LENGTH, 0U);
            return YG_PROTOCOL_OK;
        }
        ProtocolCommand command = {.source_node = request->source_node,
                                   .sequence = request->sequence,
                                   .renewal_counter = YgProtocol_ReadU32Le(request->payload)};
        Protocol_RecordCommand(endpoint, request, (ProtocolCommandArgs){.motor = command});
        if (endpoint->motor.renew != NULL)
        {
            endpoint->motor.renew(endpoint->motor.context, &command, &reply);
        }
        Protocol_WriteResult(&exchange->reply, Protocol_StatusToCode(reply.status), reply.detail);
        return YG_PROTOCOL_OK;
    }
    case PROTOCOL_START:
        return handle_start_command(endpoint, exchange);
    case PROTOCOL_READ_PARAMETER:
    case PROTOCOL_WRITE_PARAMETER:
    case PROTOCOL_SAVE_PARAMETER:
        return reject_parameter_command(endpoint, exchange);
    case PROTOCOL_START_JOB:
    case PROTOCOL_GET_JOB_STATUS:
    case PROTOCOL_CANCEL_JOB:
        return handle_job_command(endpoint, exchange);
    default:
        return YG_PROTOCOL_ROUTE_NOT_FOUND;
    }
}

/* 设备查询 */

bool Protocol_SetInfo(ProtocolEndpoint *endpoint,
                      const ProtocolInfo *info,
                      const ProtocolCaps *capabilities)
{
    if ((info == NULL) != (capabilities == NULL))
    {
        return false;
    }
    endpoint->device.info = info != NULL ? *info : (ProtocolInfo){0};
    endpoint->device.capabilities = capabilities != NULL ? *capabilities : (ProtocolCaps){0};
    endpoint->device.device_description_available = info != NULL;
    return true;
}

bool Protocol_SetInfoPage(ProtocolEndpoint *endpoint, uint16_t page, const uint8_t payload[32])
{
    if (payload == NULL || page < 1U || page > 2U)
    {
        return false;
    }
    memcpy(endpoint->device.maintenance_pages[page - 1U], payload, 32U);
    return true;
}

static yg_protocol_result_t decode_page_request(const yg_protocol_message_t *message,
                                                PageRequest *request)
{
    const uint8_t *payload;

    if (message->payload_length != PROTOCOL_PAGE_REQUEST_SIZE)
    {
        return YG_PROTOCOL_INVALID_LENGTH;
    }
    payload = message->payload;
    if (YgProtocol_ReadU16Le(payload + 10U) != 0U)
    {
        return YG_PROTOCOL_INVALID_FIELD;
    }
    request->session_id = YgProtocol_ReadU32Le(payload);
    request->request_id = YgProtocol_ReadU32Le(payload + 4U);
    request->page = YgProtocol_ReadU16Le(payload + 8U);
    return YG_PROTOCOL_OK;
}

static yg_protocol_result_t encode_device_info(const ProtocolInfo *page, ProtocolResponse *response)
{
    if (page == NULL || response == NULL)
    {
        return YG_PROTOCOL_INVALID_ARGUMENT;
    }
    uint8_t *payload = response->data;
    memcpy(payload, page->uid, sizeof(page->uid));
    YgProtocol_WriteU32Le(payload + 12U, page->product_id);
    YgProtocol_WriteU16Le(payload + 16U, page->hardware_revision);
    YgProtocol_WriteU16Le(payload + 18U, page->boot_api);
    YgProtocol_WriteU32Le(payload + 20U, page->application_version);
    YgProtocol_WriteU32Le(payload + 24U, page->boot_version);
    YgProtocol_WriteU32Le(payload + 28U, page->boot_id);
    response->length = PROTOCOL_INFO_PAGE0_SIZE;
    return YG_PROTOCOL_OK;
}

static yg_protocol_result_t handle_info_query(ProtocolEndpoint *endpoint,
                                              ProtocolExchange *exchange)
{
    const yg_protocol_message_t *request = &exchange->request;
    ProtocolResponse *response = &exchange->reply;
    PageRequest page;
    if (decode_page_request(request, &page) != YG_PROTOCOL_OK || page.page > 2U)
    {
        return YG_PROTOCOL_INVALID_FIELD;
    }
    Protocol_RecordCommand(endpoint, request, (ProtocolCommandArgs){.page = page});
    if (!endpoint->device.device_description_available)
    {
        Protocol_WriteResult(response, PROTOCOL_RESULT_BUSY, 0U);
        return YG_PROTOCOL_OK;
    }
    if (page.page != 0U)
    {
        memcpy(response->data, endpoint->device.maintenance_pages[page.page - 1U], 32U);
        response->length = 32U;
        return YG_PROTOCOL_OK;
    }
    return encode_device_info(&endpoint->device.info, response);
}

static yg_protocol_result_t encode_device_caps(const ProtocolCaps *page, ProtocolResponse *response)
{
    if (page == NULL || response == NULL)
    {
        return YG_PROTOCOL_INVALID_ARGUMENT;
    }
    uint8_t *payload = response->data;
    YgProtocol_WriteU32Le(payload, page->features);
    YgProtocol_WriteU16Le(payload + 4U, page->supported_modes);
    YgProtocol_WriteU16Le(payload + 6U, page->max_full_frame);
    YgProtocol_WriteU16Le(payload + 8U, page->max_block_data);
    YgProtocol_WriteU16Le(payload + 10U, page->write_alignment);
    YgProtocol_WriteU16Le(payload + 12U, page->default_watchdog_ms);
    YgProtocol_WriteU16Le(payload + 14U, page->max_watchdog_ms);
    response->length = PROTOCOL_CAPS_PAGE0_SIZE;
    return YG_PROTOCOL_OK;
}

static yg_protocol_result_t handle_caps_query(ProtocolEndpoint *endpoint,
                                              ProtocolExchange *exchange)
{
    const yg_protocol_message_t *request = &exchange->request;
    ProtocolResponse *response = &exchange->reply;
    PageRequest page;
    if (decode_page_request(request, &page) != YG_PROTOCOL_OK || page.page != 0U)
    {
        return YG_PROTOCOL_INVALID_FIELD;
    }
    Protocol_RecordCommand(endpoint, request, (ProtocolCommandArgs){.page = page});
    if (!endpoint->device.device_description_available)
    {
        Protocol_WriteResult(response, PROTOCOL_RESULT_BUSY, 0U);
        return YG_PROTOCOL_OK;
    }
    return encode_device_caps(&endpoint->device.capabilities, response);
}

static yg_protocol_result_t encode_device_status(const ProtocolDeviceStatus *status,
                                                 ProtocolResponse *response)
{
    if (status == NULL || response == NULL)
    {
        return YG_PROTOCOL_INVALID_ARGUMENT;
    }
    uint8_t *payload = response->data;
    if ((status->role != PROTOCOL_DEVICE_ROLE_APP && status->role != PROTOCOL_DEVICE_ROLE_LOADER) ||
        status->boot_state > PROTOCOL_BOOT_STATE_LOADER ||
        (status->role == PROTOCOL_DEVICE_ROLE_APP &&
         status->boot_state == PROTOCOL_BOOT_STATE_LOADER) ||
        (status->role == PROTOCOL_DEVICE_ROLE_LOADER &&
         status->boot_state != PROTOCOL_BOOT_STATE_LOADER) ||
        (status->epoch_valid &&
         (status->boot_epoch == 0U || status->boot_state == PROTOCOL_BOOT_STATE_UNAVAILABLE)) ||
        (!status->epoch_valid &&
         (status->boot_epoch != 0U || (status->role == PROTOCOL_DEVICE_ROLE_APP &&
                                       status->boot_state != PROTOCOL_BOOT_STATE_UNAVAILABLE))))
    {
        return YG_PROTOCOL_INVALID_FIELD;
    }
    YgProtocol_WriteU16Le(payload, status->epoch_valid ? PROTOCOL_RESULT_OK : PROTOCOL_RESULT_BUSY);
    YgProtocol_WriteU16Le(payload + 2U, 0U);
    payload[4] = status->role;
    payload[5] = status->boot_state;
    YgProtocol_WriteU16Le(payload + 6U, status->epoch_valid ? 1U : 0U);
    YgProtocol_WriteU32Le(payload + 8U, (uint32_t)status->boot_epoch);
    YgProtocol_WriteU32Le(payload + 12U, (uint32_t)(status->boot_epoch >> 32U));
    response->length = PROTOCOL_DEVICE_STATUS_SIZE;
    return YG_PROTOCOL_OK;
}

static yg_protocol_result_t handle_device_status_query(ProtocolEndpoint *endpoint,
                                                       ProtocolExchange *exchange)
{
    const yg_protocol_message_t *request = &exchange->request;
    ProtocolResponse *response = &exchange->reply;
    if (request->payload_length != 0U)
    {
        Protocol_WriteResult(response, PROTOCOL_RESULT_BAD_LENGTH, 0U);
        return YG_PROTOCOL_OK;
    }
    Protocol_RecordCommand(endpoint, request, (ProtocolCommandArgs){0});
    if (encode_device_status(&endpoint->device_status, response) != YG_PROTOCOL_OK)
    {
        Protocol_WriteResult(response, PROTOCOL_RESULT_FAILED, 0U);
        return YG_PROTOCOL_OK;
    }
    return YG_PROTOCOL_OK;
}

/* 电机启停与目标 */

static yg_protocol_result_t decode_target_command(const yg_protocol_message_t *message,
                                                  ProtocolCommand *request)
{
    ProtocolCommand decoded = {0};
    const uint8_t *payload;
    if ((message->payload_length != PROTOCOL_CONTROL_SIZE &&
         message->payload_length != PROTOCOL_OPEN_LOOP_SIZE) ||
        message->payload == NULL)
    {
        return YG_PROTOCOL_INVALID_LENGTH;
    }
    payload = message->payload;
    decoded.source_node = message->source_node;
    decoded.sequence = message->sequence;
    decoded.mode = payload[0];
    decoded.position_mrad = (int32_t)YgProtocol_ReadU32Le(payload + 4U);
    decoded.speed_mrad_s = (int32_t)YgProtocol_ReadU32Le(payload + 8U);
    decoded.iq_mA = (int32_t)YgProtocol_ReadU32Le(payload + 12U);
    decoded.v_q_mV = (int32_t)YgProtocol_ReadU32Le(payload + 16U);
    bool open_loop = decoded.mode == 9U || decoded.mode == 10U;
    uint32_t expected_bytes = open_loop ? PROTOCOL_OPEN_LOOP_SIZE : PROTOCOL_CONTROL_SIZE;
    if (message->payload_length != expected_bytes)
    {
        return YG_PROTOCOL_INVALID_LENGTH;
    }
    if (open_loop)
    {
        decoded.electrical_frequency_mHz = (int32_t)YgProtocol_ReadU32Le(payload + 20U);
        decoded.id_mA = (int32_t)YgProtocol_ReadU32Le(payload + 24U);
        decoded.vd_mV = (int32_t)YgProtocol_ReadU32Le(payload + 28U);
    }
    if (decoded.mode < 1U || decoded.mode > 10U || decoded.mode == 7U || decoded.mode == 8U ||
        payload[1] != 0U || payload[2] != 0U || payload[3] != 0U ||
        (decoded.mode != 3U && decoded.mode != 4U && decoded.position_mrad != 0) ||
        (decoded.mode != 2U && decoded.mode != 4U && decoded.speed_mrad_s != 0) ||
        (decoded.mode != 1U && decoded.mode != 4U && decoded.mode != 10U && decoded.iq_mA != 0) ||
        (decoded.mode != 6U && decoded.mode != 9U && decoded.v_q_mV != 0) ||
        (decoded.mode != 10U && decoded.id_mA != 0) || (decoded.mode != 9U && decoded.vd_mV != 0))
    {
        return YG_PROTOCOL_INVALID_FIELD;
    }
    *request = decoded;
    return YG_PROTOCOL_OK;
}

static yg_protocol_result_t handle_state_command(ProtocolEndpoint *endpoint,
                                                 ProtocolExchange *exchange)
{
    const yg_protocol_message_t *request = &exchange->request;
    ProtocolResponse *response = &exchange->reply;
    ProtocolCommand command = {.source_node = request->source_node, .sequence = request->sequence};
    ProtocolReply reply = {.status = PROTOCOL_FAILED};
    ProtocolMotorHandler handler = endpoint->motor.stop;

    /* STOP 的非空请求在来源检查前拒绝，保留原来的错误优先级。 */
    if (request->message_type != PROTOCOL_ENABLE && request->payload_length != 0U)
    {
        return YG_PROTOCOL_INVALID_FIELD;
    }
    if (request->source_node != PROTOCOL_MASTER_ID)
    {
        Protocol_WriteResult(response, PROTOCOL_RESULT_FORBIDDEN, 0U);
        return YG_PROTOCOL_OK;
    }
    if (request->message_type == PROTOCOL_ENABLE)
    {
        const uint8_t *payload = request->payload;
        if (request->payload_length != PROTOCOL_ENABLE_SIZE || payload == NULL)
        {
            Protocol_WriteResult(response, PROTOCOL_RESULT_BAD_LENGTH, 0U);
            return YG_PROTOCOL_OK;
        }
        if (payload[0] > 1U || payload[1] != 0U || payload[2] != 0U || payload[3] != 0U)
        {
            Protocol_WriteResult(response, PROTOCOL_RESULT_BAD_FIELD, 0U);
            return YG_PROTOCOL_OK;
        }
        /* enable=0 等同停机；enable=1 保持不支持，不能映射为启动。 */
        Protocol_RecordCommand(endpoint, request, (ProtocolCommandArgs){.enable = payload[0]});
        if (payload[0] == 1U)
        {
            Protocol_WriteResult(response, PROTOCOL_RESULT_UNSUPPORTED, 0U);
            return YG_PROTOCOL_OK;
        }
    }
    else
    {
        Protocol_RecordCommand(endpoint, request, (ProtocolCommandArgs){0});
        if (request->message_type == PROTOCOL_CLEAR_FAULT)
        {
            handler = endpoint->motor.clear_fault;
        }
    }
    if (handler != NULL)
    {
        handler(endpoint->motor.context, &command, &reply);
    }
    else
    {
        reply.status = PROTOCOL_UNSUPPORTED;
    }
    Protocol_WriteResult(response, Protocol_StatusToCode(reply.status), reply.detail);
    return YG_PROTOCOL_OK;
}

static yg_protocol_result_t handle_start_command(ProtocolEndpoint *endpoint,
                                                 ProtocolExchange *exchange)
{
    const yg_protocol_message_t *request = &exchange->request;
    ProtocolResponse *response = &exchange->reply;
    if (request->source_node != PROTOCOL_MASTER_ID)
    {
        Protocol_WriteResult(response, PROTOCOL_RESULT_FORBIDDEN, 0U);
        return YG_PROTOCOL_OK;
    }
    const uint8_t *payload = request->payload;
    if (request->payload_length != PROTOCOL_START_SIZE || payload == NULL)
    {
        Protocol_WriteResult(response, PROTOCOL_RESULT_BAD_LENGTH, 0U);
        return YG_PROTOCOL_OK;
    }
    if (payload[1] != 0U || payload[2] != 0U || payload[3] != 0U)
    {
        Protocol_WriteResult(response, PROTOCOL_RESULT_BAD_FIELD, 0U);
        return YG_PROTOCOL_OK;
    }
    const uint8_t mode = payload[0];
    if (mode != 1U && mode != 2U && mode != 3U && mode != 6U && mode != 9U && mode != 10U)
    {
        Protocol_WriteResult(response, PROTOCOL_RESULT_UNSUPPORTED, 0U);
        return YG_PROTOCOL_OK;
    }
    const ProtocolCommand command = {
        .source_node = request->source_node, .sequence = request->sequence, .mode = mode};
    Protocol_RecordCommand(endpoint, request, (ProtocolCommandArgs){.motor = command});
    ProtocolReply reply = {.status = PROTOCOL_FAILED};
    if (endpoint->motor.start != NULL)
    {
        endpoint->motor.start(endpoint->motor.context, &command, &reply);
    }
    else
    {
        reply.status = PROTOCOL_UNSUPPORTED;
    }
    Protocol_WriteResult(response, Protocol_StatusToCode(reply.status), reply.detail);
    return YG_PROTOCOL_OK;
}

/* 每帧从同一组无效值出发，APP 只填写本次确实读到的状态和测量。 */
static const ProtocolFeedback s_empty_feedback = {
    .reference_position_mrad = INT32_MIN,
    .actual_position_mrad = INT32_MIN,
    .reference_speed_mrad_s = INT32_MIN,
    .actual_speed_mrad_s = INT32_MIN,
    .bus_voltage_cV = INT16_MIN,
    .bus_current_mA = INT16_MIN,
    .reference_iq_mA = INT16_MIN,
    .actual_iq_mA = INT16_MIN,
    .mcu_temperature_centi_c = INT16_MIN,
    .motor_temperature_centi_c = INT16_MIN,
    .v_q_mV = INT16_MIN,
    .v_d_mV = INT16_MIN,
};

static yg_protocol_result_t encode_feedback_reply(const ProtocolEndpoint *endpoint,
                                                  const yg_protocol_message_t *request,
                                                  ProtocolCode code,
                                                  ProtocolResponse *response)
{
    bool query = request->message_type == PROTOCOL_GET_MOTOR_STATE;
    uint8_t node_id = query ? request->destination_node : endpoint->node_id;
    if (node_id == 0U || node_id == YG_PROTOCOL_BROADCAST_NODE_ID ||
        (query && endpoint->motor.read_feedback == NULL))
    {
        return YG_PROTOCOL_INVALID_ARGUMENT;
    }
    ProtocolFeedback feedback = s_empty_feedback;
    if (endpoint->motor.read_feedback != NULL)
    {
        bool available = endpoint->motor.read_feedback(endpoint->motor.context, &feedback);
        if (query && !available && code == PROTOCOL_RESULT_OK)
        {
            code = PROTOCOL_RESULT_BUSY;
        }
    }
    if (feedback.motor_state > 9U || feedback.control_mode > 10U || feedback.fault_code > 0x3FFFFU)
    {
        return YG_PROTOCOL_INVALID_FIELD;
    }
    feedback.node_id = node_id;
    feedback.result = code;
    feedback.correlated_seq = query ? 0U : request->sequence;
    return encode_motor_feedback(&feedback, response);
}

static yg_protocol_result_t handle_target_command(ProtocolEndpoint *endpoint,
                                                  ProtocolExchange *exchange)
{
    const yg_protocol_message_t *request = &exchange->request;
    ProtocolResponse *response = &exchange->reply;
    ProtocolCommand command;
    ProtocolCode code = PROTOCOL_RESULT_FORBIDDEN;
    if (request->source_node == PROTOCOL_MASTER_ID)
    {
        yg_protocol_result_t decoded = decode_target_command(request, &command);
        if (decoded == YG_PROTOCOL_OK)
        {
            ProtocolReply reply = {.status = PROTOCOL_FAILED};
            Protocol_RecordCommand(endpoint, request, (ProtocolCommandArgs){.motor = command});
            if (endpoint->motor.target != NULL)
            {
                endpoint->motor.target(endpoint->motor.context, &command, &reply);
            }
            else
            {
                reply.status = PROTOCOL_UNSUPPORTED;
            }
            code = Protocol_StatusToCode(reply.status);
        }
        else
        {
            code = decoded == YG_PROTOCOL_INVALID_LENGTH ? PROTOCOL_RESULT_BAD_LENGTH
                                                         : PROTOCOL_RESULT_BAD_FIELD;
        }
    }
    if (encode_feedback_reply(endpoint, request, code, &endpoint->messages.feedback) !=
        YG_PROTOCOL_OK)
    {
        return YG_PROTOCOL_INVALID_FIELD;
    }
    endpoint->messages.feedback_destination = request->source_node;
    endpoint->messages.feedback_pending = true;
    Protocol_WriteResult(response, code, 0U);
    return YG_PROTOCOL_OK;
}

/* 状态反馈 */

static yg_protocol_result_t encode_motor_feedback(const ProtocolFeedback *feedback,
                                                  ProtocolResponse *response)
{
    if (feedback == NULL || response == NULL)
    {
        return YG_PROTOCOL_INVALID_ARGUMENT;
    }
    uint8_t *payload = response->data;
    if ((feedback->valid_bits & ~PROTOCOL_FEEDBACK_VALID_MASK) != 0U || feedback->node_id == 0U ||
        feedback->node_id == YG_PROTOCOL_BROADCAST_NODE_ID)
    {
        return YG_PROTOCOL_INVALID_FIELD;
    }
    YgProtocol_WriteU16Le(payload, feedback->result);
    YgProtocol_WriteU16Le(payload + 2U, feedback->correlated_seq);
    YgProtocol_WriteU32Le(payload + 4U, feedback->fault_code);
    YgProtocol_WriteU32Le(payload + 8U, (uint32_t)feedback->reference_position_mrad);
    YgProtocol_WriteU32Le(payload + 12U, (uint32_t)feedback->actual_position_mrad);
    YgProtocol_WriteU32Le(payload + 16U, (uint32_t)feedback->reference_speed_mrad_s);
    YgProtocol_WriteU32Le(payload + 20U, (uint32_t)feedback->actual_speed_mrad_s);
    YgProtocol_WriteU16Le(payload + 24U, feedback->valid_bits);
    YgProtocol_WriteU16Le(payload + 26U, (uint16_t)feedback->bus_voltage_cV);
    YgProtocol_WriteU16Le(payload + 28U, (uint16_t)feedback->bus_current_mA);
    YgProtocol_WriteU16Le(payload + 30U, (uint16_t)feedback->reference_iq_mA);
    YgProtocol_WriteU16Le(payload + 32U, (uint16_t)feedback->actual_iq_mA);
    YgProtocol_WriteU16Le(payload + 34U, (uint16_t)feedback->mcu_temperature_centi_c);
    YgProtocol_WriteU16Le(payload + 36U, (uint16_t)feedback->motor_temperature_centi_c);
    YgProtocol_WriteU16Le(payload + 38U, (uint16_t)feedback->v_q_mV);
    YgProtocol_WriteU16Le(payload + 40U, (uint16_t)feedback->v_d_mV);
    payload[42] = feedback->node_id;
    payload[43] = feedback->motor_state;
    payload[44] = feedback->control_mode;
    response->length = PROTOCOL_FEEDBACK_SIZE;
    return YG_PROTOCOL_OK;
}

static yg_protocol_result_t handle_motor_status_query(ProtocolEndpoint *endpoint,
                                                      ProtocolExchange *exchange)
{
    const yg_protocol_message_t *request = &exchange->request;
    ProtocolCode code = PROTOCOL_RESULT_BAD_LENGTH;
    if (request->payload_length == 0U)
    {
        Protocol_RecordCommand(endpoint, request, (ProtocolCommandArgs){0});
        code = PROTOCOL_RESULT_OK;
    }
    return encode_feedback_reply(endpoint, request, code, &exchange->reply);
}

static yg_protocol_result_t queue_pending_feedback(ProtocolEndpoint *endpoint)
{
    MotorMessages *motor = &endpoint->messages;
    yg_protocol_message_t feedback = {0};
    yg_protocol_result_t result;
    if (!motor->feedback_pending)
    {
        return YG_PROTOCOL_OK;
    }
    feedback.version = YG_PROTOCOL_VERSION;
    feedback.source_node = endpoint->node_id;
    feedback.destination_node = motor->feedback_destination;
    feedback.message_type = PROTOCOL_MOTION_FEEDBACK;
    feedback.sequence = motor->feedback_sequence;
    feedback.payload_length = motor->feedback.length;
    feedback.payload = motor->feedback.data;
    result = Protocol_QueueMessage(endpoint, &feedback, 3U);
    if (result == YG_PROTOCOL_OK)
    {
        ++motor->feedback_sequence;
        motor->feedback_pending = false;
    }
    return result;
}

/* 标定与参数命令 */

static yg_protocol_result_t decode_job_command(const yg_protocol_message_t *message,
                                               ProtocolJobRequest *request)
{
    ProtocolJobRequest candidate = {.operation = PROTOCOL_JOB_START};
    if (message == NULL || request == NULL)
    {
        return YG_PROTOCOL_INVALID_ARGUMENT;
    }
    if (message->payload_length != PROTOCOL_JOB_REQUEST_SIZE)
    {
        return YG_PROTOCOL_INVALID_LENGTH;
    }
    if (message->payload == NULL)
    {
        return YG_PROTOCOL_INVALID_ARGUMENT;
    }
    switch (message->message_type)
    {
    case PROTOCOL_START_JOB:
        candidate.operation = PROTOCOL_JOB_START;
        candidate.kind = YgProtocol_ReadU16Le(message->payload);
        if (candidate.kind == 0U || YgProtocol_ReadU16Le(message->payload + 2U) != 0U)
        {
            return YG_PROTOCOL_INVALID_FIELD;
        }
        break;
    case PROTOCOL_GET_JOB_STATUS:
    case PROTOCOL_CANCEL_JOB:
        candidate.operation = message->message_type == PROTOCOL_GET_JOB_STATUS
                                  ? PROTOCOL_JOB_QUERY
                                  : PROTOCOL_JOB_CANCEL;
        candidate.job_id = YgProtocol_ReadU32Le(message->payload);
        if (candidate.job_id == 0U)
        {
            return YG_PROTOCOL_INVALID_FIELD;
        }
        break;
    default:
        return YG_PROTOCOL_INVALID_MESSAGE_TYPE;
    }
    candidate.source_node = message->source_node;
    candidate.sequence = message->sequence;
    *request = candidate;
    return YG_PROTOCOL_OK;
}

static yg_protocol_result_t encode_job_reply(const ProtocolJobReply *reply,
                                             ProtocolResponse *response)
{
    if (reply == NULL || response == NULL)
    {
        return YG_PROTOCOL_INVALID_ARGUMENT;
    }
    uint8_t *payload = response->data;
    if (reply->phase > PROTOCOL_JOB_PHASE_FAILED || reply->effects > PROTOCOL_JOB_EFFECT_COMMITTED)
    {
        return YG_PROTOCOL_INVALID_FIELD;
    }
    if (reply->status == PROTOCOL_OK || reply->status == PROTOCOL_ACCEPTED)
    {
        const uint16_t kind = reply->kind;
        const bool supported = kind == PROTOCOL_JOB_KIND_ENCODER_LUT ||
                               kind == PROTOCOL_JOB_KIND_ELECTRICAL_ZERO ||
                               kind == PROTOCOL_JOB_KIND_MECHANICAL_ZERO ||
                               kind == PROTOCOL_JOB_KIND_SAVE || kind == PROTOCOL_JOB_KIND_DEFAULTS;
        if (!supported || reply->token == 0U || reply->phase == PROTOCOL_JOB_PHASE_NONE)
        {
            return YG_PROTOCOL_INVALID_FIELD;
        }
    }
    YgProtocol_WriteU16Le(payload, Protocol_StatusToCode(reply->status));
    YgProtocol_WriteU16Le(payload + 2U, reply->detail);
    YgProtocol_WriteU32Le(payload + 4U, reply->token);
    YgProtocol_WriteU16Le(payload + 8U, reply->kind);
    payload[10] = reply->phase;
    payload[11] = reply->effects;
    response->length = PROTOCOL_JOB_REPLY_SIZE;
    return YG_PROTOCOL_OK;
}

static void write_job_error_reply(ProtocolResponse *response, ProtocolCode code, uint16_t detail)
{
    Protocol_WriteResult(response, code, detail);
    memset(response->data + 4U, 0, PROTOCOL_JOB_REPLY_SIZE - 4U);
    response->length = PROTOCOL_JOB_REPLY_SIZE;
}

static yg_protocol_result_t handle_job_command(ProtocolEndpoint *endpoint,
                                               ProtocolExchange *exchange)
{
    const yg_protocol_message_t *request = &exchange->request;
    ProtocolResponse *response = &exchange->reply;
    ProtocolJobRequest command;
    ProtocolJobReply reply = {.status = PROTOCOL_FAILED};
    yg_protocol_result_t decoded;
    if (request->source_node != PROTOCOL_MASTER_ID)
    {
        write_job_error_reply(response, PROTOCOL_RESULT_FORBIDDEN, 0U);
        return YG_PROTOCOL_OK;
    }
    decoded = decode_job_command(request, &command);
    if (decoded != YG_PROTOCOL_OK)
    {
        write_job_error_reply(response,
                              decoded == YG_PROTOCOL_INVALID_LENGTH ? PROTOCOL_RESULT_BAD_LENGTH
                                                                    : PROTOCOL_RESULT_BAD_FIELD,
                              PROTOCOL_JOB_DETAIL_BAD_FIELD);
        return YG_PROTOCOL_OK;
    }
    Protocol_RecordCommand(endpoint, request, (ProtocolCommandArgs){.job = command});
    if (endpoint->jobs.handler != NULL)
    {
        endpoint->jobs.handler(endpoint->jobs.context, &command, &reply);
    }
    else
    {
        reply.status = PROTOCOL_UNSUPPORTED;
    }
    if (encode_job_reply(&reply, response) != YG_PROTOCOL_OK)
    {
        write_job_error_reply(response, PROTOCOL_RESULT_FAILED, 0U);
        return YG_PROTOCOL_OK;
    }
    return YG_PROTOCOL_OK;
}

static yg_protocol_result_t reject_parameter_command(ProtocolEndpoint *endpoint,
                                                     ProtocolExchange *exchange)
{
    const yg_protocol_message_t *request = &exchange->request;
    ProtocolResponse *reply = &exchange->reply;
    if (request->source_node != PROTOCOL_MASTER_ID)
    {
        Protocol_WriteResult(reply, PROTOCOL_RESULT_FORBIDDEN, 0U);
        return YG_PROTOCOL_OK;
    }
    bool write = request->message_type == PROTOCOL_WRITE_PARAMETER;
    if (request->payload == NULL || request->payload_length != (write ? 12U : 4U))
    {
        Protocol_WriteResult(reply, PROTOCOL_RESULT_BAD_LENGTH, 0U);
        return YG_PROTOCOL_OK;
    }
    const uint8_t *data = request->payload;
    if ((write && ((data[1] != 1U && data[1] != 2U) || data[2] != 4U || data[3] != 0U)) ||
        (request->message_type == PROTOCOL_READ_PARAMETER &&
         (data[1] != 0U || data[2] != 0U || data[3] != 0U)))
    {
        Protocol_WriteResult(reply, PROTOCOL_RESULT_BAD_FIELD, 6U);
        return YG_PROTOCOL_OK;
    }
    ProtocolCommandArgs args = {0};
    if (request->message_type == PROTOCOL_SAVE_PARAMETER)
    {
        args.parameter.expected_revision = YgProtocol_ReadU32Le(data);
    }
    else
    {
        args.parameter.id = data[0];
        if (write)
        {
            args.parameter.value_type = data[1];
            args.parameter.expected_revision = YgProtocol_ReadU32Le(data + 4U);
            args.parameter.value = YgProtocol_ReadU32Le(data + 8U);
        }
    }
    Protocol_RecordCommand(endpoint, request, args);
    Protocol_WriteResult(reply, PROTOCOL_RESULT_UNSUPPORTED, 0U);
    return YG_PROTOCOL_OK;
}
