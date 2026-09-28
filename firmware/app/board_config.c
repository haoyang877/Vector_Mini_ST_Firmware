#include "board_config.h"

#include "board_hw.h"
#include "boot_hw.h"
#include "comm_hw.h"
#include "critical_hw.h"
#include "foc_run_state.h"
#include "yg_protocol_runtime.h"
#include "yg_protocol_link.h"
#include "yg_protocol_motor_adapter.h"
#include "motor_stop_service.h"
#include "motor_hw.h"
#include "motor_state.h"
#include "param_comm_bridge.h"
#include "power_stage_hw.h"
#include "yg_protocol_status_adapter.h"
#include "yg_protocol_owner_adapter.h"
#include "time_hw.h"

/* 启动编排：参数装载 → 应用状态初始化 → 板级采样时钟 → CAN FD 端点。
 * 硬件细节全部位于 platform 契约实现内；本文件不再包含任何硬件头。 */

static motor_stop_service_t stop_service;
static yg_protocol_motor_adapter_t motor_adapter;
static bool communication_ready;

static motor_stop_owner_result_t request_stop(void *context)
{
    (void)context;
    return FocRunState_RequestRemoteStop() ? MOTOR_STOP_OWNER_ACCEPTED : MOTOR_STOP_OWNER_DENIED;
}

static bool phases_disabled(void *context)
{
    (void)context;
    return power_stage_hw_phases_disabled();
}

static MotorControlResult submit_control(void *context, const MotorControlCommand *command)
{
    (void)context;
    return FocRunState_SubmitControl(command);
}

static bool bind_motor_stop(void)
{
    static const motor_stop_owner_port_t owner = {request_stop, phases_disabled};
    yg_protocol_motor_service_t service;
    if (!MotorStopService_Init(&stop_service, &owner, NULL))
    {
        return false;
    }
    motor_adapter.stop_service = &stop_service;
    motor_adapter.submit_control = submit_control;
    motor_adapter.control_context = NULL;
    motor_adapter.lifecycle_handler = YgProtocolOwner_HandleLifecycle;
    motor_adapter.lifecycle_context = NULL;
    service.context = &motor_adapter;
    service.handler = yg_protocol_motor_adapter_handle;
    return YgProtocolLink_BindMotorService(&service);
}

static bool bind_device_description(void)
{
    /* 0表示尚无权威产品/板卡/发布版本来源；不把协议版本冒充应用版本。 */
    yg_protocol_readonly_info_page0_t info = {0};
    yg_protocol_readonly_caps_page0_t capabilities = {
        .features = (1UL << 10U) | (1UL << 11U) | (1UL << 12U),
        .supported_modes = 0U,
        .max_full_frame = YG_PROTOCOL_MAX_FRAME_SIZE,
        .max_block_data = YG_PROTOCOL_MAX_PAYLOAD_SIZE};
    return board_hw_read_uid(info.uid) && YgProtocolLink_SetDeviceDescription(&info, &capabilities);
}

#if defined(YG_APP_LOADER_IMAGE) && YG_APP_LOADER_IMAGE
/* 56 进 Loader 的最小所有者：停机→关相→精确TX完成后写ENTER邮箱复位；无持久确认。 */
#define BOOT_ENTRY_TIMEOUT_MS 1000U
#define BOOT_HEALTH_INTERVAL_MS 100U
static BootEntryRequest entry_request;
static BootEntryResult entry_result;
static uint64_t entry_ticket;
static uint32_t entry_started_ms;
static uint32_t health_checked_ms, observed_supervisor_epoch;
static bool entry_requested, boot_supported;

static bool same_request(const BootEntryRequest *left, const BootEntryRequest *right)
{
    return left->boot_epoch == right->boot_epoch && left->source_node == right->source_node &&
           left->sequence == right->sequence;
}

static void poll_entry(uint32_t now_ms)
{
    if (!entry_requested ||
        (entry_result != BOOT_ENTRY_PENDING && entry_result != BOOT_ENTRY_READY))
    {
        return;
    }
    if ((uint32_t)(now_ms - entry_started_ms) >= BOOT_ENTRY_TIMEOUT_MS)
    {
        entry_result = BOOT_ENTRY_FAILED;
    }
    else if (FocRunState_BootHandoffReady(entry_ticket) && MotorControl_SaveIdle())
    {
        entry_result = BOOT_ENTRY_READY;
    }
    else
    {
        entry_result = BOOT_ENTRY_PENDING;
    }
}

static void handle_entry(void *context, const BootEntryRequest *request, BootEntryReply *reply)
{
    (void)context;
    uint32_t now_ms;
    if (reply == NULL)
    {
        return;
    }
    *reply = (BootEntryReply){.result = BOOT_ENTRY_UNAVAILABLE};
    if (!boot_supported || request == NULL || request->boot_epoch == 0U ||
        request->source_node == UINT8_MAX)
    {
        return;
    }
    if (entry_requested && !same_request(request, &entry_request))
    {
        reply->result = BOOT_ENTRY_DENIED;
        return;
    }
    now_ms = time_hw_now_ms();
    if (!entry_requested)
    {
        entry_request = *request;
        entry_ticket = request->boot_epoch;
        entry_started_ms = now_ms;
        entry_requested = true;
        entry_result = FocRunState_RequestBootStop(entry_ticket) == MOTOR_CONTROL_ACCEPTED
                           ? BOOT_ENTRY_PENDING
                           : BOOT_ENTRY_DENIED;
    }
    poll_entry(now_ms);
    reply->result = entry_result;
    reply->ack_ticket = entry_result == BOOT_ENTRY_READY ? entry_ticket : 0U;
}

static void handle_tx_complete(void *context, uint64_t ticket, bool success)
{
    (void)context;
    uint32_t mask;
    bool permitted;
    if (!entry_requested || ticket == 0U || ticket != entry_ticket ||
        entry_result != BOOT_ENTRY_READY)
    {
        return;
    }
    mask = critical_hw_enter();
    /* 最终二次复核后撤销票据使用权限；复位失败也不能再次消费旧TX结果。 */
    permitted = success && entry_result == BOOT_ENTRY_READY &&
                (uint32_t)(time_hw_now_ms() - entry_started_ms) < BOOT_ENTRY_TIMEOUT_MS &&
                FocRunState_BootHandoffReady(ticket) && MotorControl_SaveIdle();
    entry_result = BOOT_ENTRY_FAILED;
    if (permitted)
    {
        (void)BootHw_EnterLoader(Protocol_NodeId_Get());
    }
    critical_hw_exit(mask);
}

static bool bind_boot_entry(void)
{
    const BootEntryService service = {NULL, handle_entry, handle_tx_complete};
    boot_supported = BootHw_Init();
    return YgProtocolLink_BindBootService(&service);
}
#endif

void Board_Init(void)
{
    /* 电机所有者装载参数与标定并初始化；无效记录回退默认值。 */
    MotorControl_Init();

    /* 新协议复位态为失能；CH4 采样时钟保持运行，三相输出等待控制所有者授权。 */
    board_hw_start(false);

    /* CAN FD 是唯一通信入口；硬件启动由运行时端点委托给平台层。 */
    communication_ready = YgProtocolRuntime_Init(Protocol_NodeId_Get()) &&
                          YgProtocolStatusAdapter_Init() && bind_motor_stop() &&
                          YgProtocolOwner_BindServices() && bind_device_description();
#if defined(YG_APP_LOADER_IMAGE) && YG_APP_LOADER_IMAGE
    communication_ready = communication_ready && bind_boot_entry();
#endif
    communication_ready = communication_ready && YgProtocolRuntime_Start();
    if (!communication_ready)
    {
        /* 初始化失败不能留下已启动但无通信入口的功率输出。 */
        power_stage_hw_stop();
    }
}

void Board_ServiceBoot(uint32_t now_ms)
{
#if defined(YG_APP_LOADER_IMAGE) && YG_APP_LOADER_IMAGE
    poll_entry(now_ms);
    if ((uint32_t)(now_ms - health_checked_ms) >= BOOT_HEALTH_INTERVAL_MS)
    {
        uint32_t progress = 0U;
        health_checked_ms = now_ms;
        bool healthy = FocRunState_ReadBootHealth(&progress);
        healthy = healthy && progress != observed_supervisor_epoch && communication_ready &&
                  YgProtocolLink_IsReady() && comm_hw_can_is_healthy();
        observed_supervisor_epoch = progress;
        BootHw_Watchdog(healthy);
    }
#else
    (void)now_ms;
#endif
}
