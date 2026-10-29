#include "yg_protocol_link.h"
#include <stddef.h>
#include <string.h>
#include "bsp_system.h"
#include "yg_protocol_endpoint.h"
#include "yg_protocol_readonly.h"
#include "yg_protocol_canfd.h"
#include "yg_protocol_motor.h"
#include "yg_protocol_parameter.h"
#include "yg_protocol_boot.h"
#include "bsp_can.h"

/* CAN FD 组合层：后台路由，电机业务经显式绑定注入；不直接访问电机控制。 */

/* APP 当前只发送单帧回复/反馈；TX 保留突发余量和控制回复+反馈的两个位置。
 * 通用端点仍可处理 1767B 消息，但 APP 不承诺一次排入 39 个发送分片。 */
#define YG_LINK_RX_QUEUE_CAPACITY 8U
#define YG_LINK_TX_QUEUE_CAPACITY 8U
#define YG_LINK_FRAGMENT_CAPACITY YG_PROTOCOL_MAX_MESSAGE_PAYLOAD
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
    uint8_t node_id;
    size_t base_command_count;
    size_t boot_tx_slot;
    yg_link_boot_tx_state_t boot_tx_state;
} yg_protocol_link_context_t;

static yg_protocol_motor_context_t s_motor_context;
static yg_protocol_readonly_context_t s_readonly_context;
static yg_protocol_parameter_service_t s_parameter_service;
static yg_protocol_job_service_t s_job_service;
static yg_protocol_boot_context_t s_boot_context;

static yg_protocol_link_context_t s_link_context;
static const yg_protocol_readonly_device_status_t s_empty_device_status = {
    .role = YG_PROTOCOL_DEVICE_ROLE_APP};
static yg_protocol_transfer_frame_t s_rx_storage[YG_LINK_RX_QUEUE_CAPACITY];
static yg_protocol_transfer_frame_t s_tx_storage[YG_LINK_TX_QUEUE_CAPACITY];
static uint8_t s_fragment_storage[YG_LINK_FRAGMENT_CAPACITY];
static yg_protocol_transfer_queue_t s_rx_queue;
static yg_protocol_transfer_queue_t s_tx_queue;
static yg_protocol_commands_t s_commands;
static yg_protocol_endpoint_t s_endpoint;
static bool s_ready;

bool YgProtocolLink_Init(uint8_t node_id)
{
    /* 唯一命令表：ID、分片许可、处理函数、上下文。末两项按升级服务绑定开放。 */
    static const yg_protocol_command_t commands[] = {
        {YG_PROTOCOL_READONLY_TYPE_GET_INFO,
         true,
         yg_protocol_readonly_handle_info,
         &s_readonly_context},
        {YG_PROTOCOL_READONLY_TYPE_GET_CAPS,
         false,
         yg_protocol_readonly_handle_caps,
         &s_readonly_context},
        {YG_PROTOCOL_READONLY_TYPE_GET_DEVICE_STATUS,
         false,
         yg_protocol_readonly_handle_device_status,
         &s_readonly_context},
        {YG_PROTOCOL_READONLY_TYPE_GET_MOTOR_STATE,
         false,
         yg_protocol_motor_handle_status,
         &s_motor_context},
        {YG_PROTOCOL_READONLY_TYPE_MOTION_FEEDBACK, false, NULL, NULL},
        {YG_PROTOCOL_MOTOR_TYPE_ENABLE, false, yg_protocol_motor_handle_action, &s_motor_context},
        {YG_PROTOCOL_MOTOR_TYPE_CLEAR_FAULT,
         false,
         yg_protocol_motor_handle_action,
         &s_motor_context},
        {YG_PROTOCOL_MOTOR_TYPE_STOP, false, yg_protocol_motor_handle_action, &s_motor_context},
        {YG_PROTOCOL_MOTOR_TYPE_CONTROL, false, yg_protocol_motor_handle_control, &s_motor_context},
        {YG_PROTOCOL_MOTOR_TYPE_START, false, yg_protocol_motor_handle_start, &s_motor_context},
        {YG_PROTOCOL_PARAMETER_TYPE_READ,
         false,
         yg_protocol_parameter_handle,
         &s_parameter_service},
        {YG_PROTOCOL_PARAMETER_TYPE_WRITE,
         false,
         yg_protocol_parameter_handle,
         &s_parameter_service},
        {YG_PROTOCOL_PARAMETER_TYPE_SAVE,
         false,
         yg_protocol_parameter_handle,
         &s_parameter_service},
        {YG_PROTOCOL_JOB_TYPE_START, false, yg_protocol_job_handle, &s_job_service},
        {YG_PROTOCOL_JOB_TYPE_STATUS, false, yg_protocol_job_handle, &s_job_service},
        {YG_PROTOCOL_JOB_TYPE_CANCEL, false, yg_protocol_job_handle, &s_job_service},
        {YG_PROTOCOL_BOOT_TYPE_ENTER, false, yg_protocol_boot_handle, &s_boot_context},
        {YG_PROTOCOL_BOOT_TYPE_CONFIRM, false, yg_protocol_boot_handle, &s_boot_context},
    };
    yg_protocol_endpoint_config_t endpoint_config;

    if (s_ready && s_link_context.boot_tx_state != YG_BOOT_TX_NONE)
    {
        /* 初始化不能丢弃尚未完成的物理票据；由所有者/平台先确认失败或完成。 */
        return false;
    }
    s_ready = false;
    if (node_id == YG_PROTOCOL_BROADCAST_NODE_ID ||
        !yg_protocol_transfer_queue_init(&s_rx_queue, s_rx_storage, YG_LINK_RX_QUEUE_CAPACITY) ||
        !yg_protocol_transfer_queue_init(&s_tx_queue, s_tx_storage, YG_LINK_TX_QUEUE_CAPACITY))
    {
        return false;
    }

    /* 初始化一次撤销全部旧绑定、历史和待发状态；此时 RX 尚未启动。 */
    memset(&s_link_context, 0, sizeof(s_link_context));
    memset(&s_motor_context, 0, sizeof(s_motor_context));
    memset(&s_readonly_context, 0, sizeof(s_readonly_context));
    memset(&s_parameter_service, 0, sizeof(s_parameter_service));
    memset(&s_job_service, 0, sizeof(s_job_service));
    memset(&s_boot_context, 0, sizeof(s_boot_context));
    s_link_context.node_id = node_id;
    s_motor_context.node_id = node_id;
    s_motor_context.motor_status_source = &s_motor_context.empty_motor_status;
    s_readonly_context.device_status_source = &s_empty_device_status;
    s_link_context.base_command_count = sizeof(commands) / sizeof(commands[0]) - 2U;
    if (yg_protocol_commands_register(&s_commands, commands, s_link_context.base_command_count) !=
        YG_PROTOCOL_OK)
    {
        return false;
    }

    endpoint_config.local_node = node_id;
    endpoint_config.reply_priority = 3U;
    endpoint_config.fragment_timeout_ms = 200U;
    endpoint_config.rx = &s_rx_queue;
    endpoint_config.tx = &s_tx_queue;
    endpoint_config.commands = &s_commands;
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
    memcpy(s_readonly_context.maintenance_pages[page - 1U], payload, 32U);
    return true;
}

bool YgProtocolLink_SetDeviceDescription(const yg_protocol_readonly_info_page0_t *info,
                                         const yg_protocol_readonly_caps_page0_t *capabilities)
{
    if (!s_ready || (info == NULL) != (capabilities == NULL))
    {
        return false;
    }
    s_readonly_context.info = info != NULL ? *info : (yg_protocol_readonly_info_page0_t){0};
    s_readonly_context.capabilities =
        capabilities != NULL ? *capabilities : (yg_protocol_readonly_caps_page0_t){0};
    s_readonly_context.device_description_available = info != NULL;
    return true;
}

bool YgProtocolLink_BindDeviceStatusSource(const yg_protocol_readonly_device_status_t *source)
{
    if (!s_ready)
    {
        return false;
    }
    s_readonly_context.device_status_source = source != NULL ? source : &s_empty_device_status;
    return true;
}

bool YgProtocolLink_BindMotorService(const yg_protocol_motor_service_t *service)
{
    if (!s_ready)
    {
        return false;
    }
    s_motor_context.motor_service = service != NULL ? *service : (yg_protocol_motor_service_t){0};
    return true;
}

bool YgProtocolLink_BindParameterService(const yg_protocol_parameter_service_t *service)
{
    if (!s_ready)
    {
        return false;
    }
    s_parameter_service = service != NULL ? *service : (yg_protocol_parameter_service_t){0};
    return true;
}

bool YgProtocolLink_BindJobService(const yg_protocol_job_service_t *service)
{
    if (!s_ready)
    {
        return false;
    }
    s_job_service = service != NULL ? *service : (yg_protocol_job_service_t){0};
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
    const size_t count = s_link_context.base_command_count + (service != NULL ? 2U : 0U);
    if (yg_protocol_commands_register(&s_commands, s_commands.entries, count) != YG_PROTOCOL_OK)
    {
        return false;
    }
    s_boot_context.boot_service = service != NULL ? *service : (BootEntryService){0};
    return true;
}

bool YgProtocolLink_BindMotorStatusSource(const yg_protocol_motor_status_source_t *source)
{
    if (!s_ready)
    {
        return false;
    }
    s_motor_context.motor_status_source =
        source != NULL ? source : &s_motor_context.empty_motor_status;
    return true;
}

bool YgProtocolLink_OnRxFrame(const BspCanFrame *frame)
{
    if (!s_ready || frame == NULL || !frame->extended || !frame->fd || !frame->bitrate_switch ||
        frame->remote)
    {
        return false;
    }
    /* 唯一CAN RX IRQ生产，前台不能抢占本函数，消费端已在临界区取走。
     * ADC和监督ISR不访问RX队列；帧校验及复制不应全局屏蔽电机中断。 */
    return yg_protocol_canfd_receive(&s_rx_queue, frame) == YG_PROTOCOL_OK;
}

static yg_protocol_result_t
send_message(void *context, const yg_protocol_message_t *message, uint8_t priority)
{
    return yg_protocol_endpoint_send(context, message, priority);
}

static void finish_boot_ticket(bool success)
{
    const uint64_t ticket = s_boot_context.boot_ticket;
    s_link_context.boot_tx_state = YG_BOOT_TX_NONE;
    yg_protocol_boot_complete(&s_boot_context, ticket, success);
}

static void service_boot_completion(void)
{
    BspCanCompletion completion;
    for (uint8_t received = 0U; received < 8U; ++received)
    {
        if (BspCan_TakeCompletion(&completion) != BSP_OK)
        {
            break;
        }
        if (s_link_context.boot_tx_state == YG_BOOT_TX_IN_FLIGHT &&
            completion.ticket == s_boot_context.boot_ticket && completion.ticket != 0U)
        {
            finish_boot_ticket(completion.result == BSP_CAN_TX_OK);
        }
        /* 旧票据或其他帧事件不能使当前交接完成。 */
    }
}

static void capture_boot_reply_slot(size_t previous_head, yg_protocol_result_t result)
{
    if (s_link_context.boot_tx_state == YG_BOOT_TX_NONE && s_boot_context.boot_ticket != 0U)
    {
        s_link_context.boot_tx_state = YG_BOOT_TX_PREPARED;
    }
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
    return yg_protocol_canfd_unpack(frame, s_boot_context.boot_reply_destination, &response) ==
               YG_PROTOCOL_OK &&
           response.message_type == YG_PROTOCOL_BOOT_TYPE_ENTER &&
           response.flags == YG_PROTOCOL_FLAGS_RESPONSE &&
           response.source_node == s_link_context.node_id &&
           response.sequence == s_boot_context.boot_reply_sequence &&
           response.payload_length == YG_PROTOCOL_BOOT_REPLY_SIZE && response.payload[0] == 1U &&
           response.payload[1] == 0U && response.payload[2] == 0U && response.payload[3] == 0U;
}

static void discard_queued_boot_reply(void)
{
    size_t slot = s_tx_queue.tail;
    for (size_t i = 0U; i < yg_protocol_transfer_queue_count(&s_tx_queue); ++i)
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
            --s_tx_queue.produced;
            return;
        }
        slot = (slot + 1U) % s_tx_queue.capacity;
    }
}

static void expire_boot_reply(uint32_t now_ms)
{
    if ((s_link_context.boot_tx_state != YG_BOOT_TX_PREPARED &&
         s_link_context.boot_tx_state != YG_BOOT_TX_QUEUED) ||
        (uint32_t)(now_ms - s_boot_context.boot_ready_ms) < YG_LINK_BOOT_QUEUE_TIMEOUT_MS)
    {
        return;
    }
    if (s_link_context.boot_tx_state == YG_BOOT_TX_PREPARED)
    {
        if (s_endpoint.response_pending &&
            s_endpoint.pending_response.message_type == YG_PROTOCOL_BOOT_TYPE_ENTER &&
            s_endpoint.pending_response.sequence == s_boot_context.boot_reply_sequence &&
            s_endpoint.pending_response.destination_node == s_boot_context.boot_reply_destination)
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
    BspCanFrame hardware = {0};
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
    if (BspCan_TrySendTracked(&hardware, s_boot_context.boot_ticket) != BSP_OK)
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
    s_boot_context.service_now_ms = now_ms;
    expire_boot_reply(now_ms);
    while (processed < 8U)
    {
        yg_protocol_transfer_frame_t frame;
        yg_protocol_result_t result;
        size_t previous_head;
        if (!s_endpoint.response_pending &&
            yg_protocol_motor_flush_feedback(&s_motor_context, send_message, &s_endpoint) !=
                YG_PROTOCOL_OK)
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
            /* 唯一前台消费者先复制但不释放队首；ISR只追加，队满丢新帧，
             * 因此复制期间不会覆盖此槽位。仅消费计数需要和ISR互斥。 */
            if (!yg_protocol_transfer_queue_peek(&s_rx_queue, &frame))
            {
                break;
            }
            bool received = yg_protocol_transfer_queue_discard(&s_rx_queue);
            if (!received)
            {
                break;
            }
            /* 复制、CRC、路由和业务处理始终允许电机中断抢占。 */
            previous_head = s_tx_queue.head;
            result = yg_protocol_endpoint_process_frame(&s_endpoint, &frame, now_ms);
        }
        capture_boot_reply_slot(previous_head, result);
        if (result == YG_PROTOCOL_QUEUE_FULL)
        {
            break;
        }
        if (!s_endpoint.response_pending &&
            yg_protocol_motor_flush_feedback(&s_motor_context, send_message, &s_endpoint) !=
                YG_PROTOCOL_OK)
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

/* runtime 只拥有硬件调度；link 接收组合层的地址值并独占端点状态。 */
static uint32_t s_last_recovery_ms;
static bool s_initialized;
static bool s_started;

bool YgProtocolRuntime_Init(uint8_t node_id)
{
    /* 不允许前台重复初始化与已经运行的 RX ISR 竞争同一队列。 */
    if (s_initialized || !YgProtocolLink_Init(node_id))
    {
        return false;
    }
    s_last_recovery_ms = 0U;
    s_initialized = true;
    return true;
}

bool YgProtocolRuntime_Start(void)
{
    if (!s_initialized || !YgProtocolLink_IsReady())
    {
        return false;
    }
    if (s_started)
    {
        return true;
    }
    if (BspCan_Start() != BSP_OK)
    {
        return false;
    }
    s_started = true;
    return true;
}

void YgProtocolRuntime_Service(uint32_t now_ms)
{
    if (!s_started)
    {
        return;
    }
    if ((uint32_t)(now_ms - s_last_recovery_ms) >= 1000U)
    {
        s_last_recovery_ms = now_ms;
        (void)BspCan_Recover();
    }
    YgProtocolLink_Service(now_ms);
}

void YgProtocolCanfd_RxIrqHandler(void)
{
    BspCanFrame frame;

    /* 每次最多读取 3 帧，持续输入不能把一次 ISR 拉成无界循环。 */
    for (uint8_t count = 0U; count < 3U; ++count)
    {
        if (BspCan_Receive(&frame) != BSP_OK)
        {
            /* 空 FIFO 或被端口层丢弃的非 CAN FD 帧都计入本次上限；继续清理剩余槽位。 */
            continue;
        }
        /* 格式验证由 link 唯一负责；ISR 只提供帧值，不重复维护准入条件。 */
        (void)YgProtocolLink_OnRxFrame(&frame);
    }
}
