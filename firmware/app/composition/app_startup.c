#include "app.h"

#include "app_tasks.h"
#include "boot_hw.h"
#include "bsp_can.h"
#include "bsp_system.h"
#include "motor_state_machine.h"
#include "yg_protocol_link.h"
#include "yg_protocol_motor_adapter.h"
#include "motor_stop_request.h"
#include "bsp_pwm.h"
#include "motor_axis.h"
#include "yg_protocol_status_adapter.h"
#include "yg_protocol_owner_adapter.h"
#include "app_target.h"
#include "yg_protocol_common.h"
#include "device_identity.h"

/* 启动编排：参数装载 → 应用状态初始化 → 板级采样时钟 → CAN FD 端点。
 * 硬件细节全部位于 platform 契约实现内；本文件不再包含任何硬件头。 */

static motor_stop_service_t s_stop_service;
static yg_protocol_motor_adapter_t s_motor_adapter;
static bool s_communication_ready;
static DeviceIdentity s_identity;

/* 公开入口在前，绑定及Loader交接细节在后。 */
static bool bind_motor_stop(void);
static bool bind_device_description(void);
static void service_boot(uint32_t now_ms);
#if defined(YG_APP_LOADER_IMAGE) && YG_APP_LOADER_IMAGE
static bool bind_boot_entry(void);
#endif

void App_Init(void)
{
    /* 电机所有者装载参数与标定并初始化；无效记录回退默认值。 */
    AppTasks_Init();
    MotorAxis_Init();

    /* 新协议复位态为失能；CH4 采样时钟保持运行，三相输出等待控制所有者授权。 */
    /* 采样由main在应用对象就绪后调用Bsp_Start开放。 */

    /* CAN FD 是唯一通信入口；硬件启动由运行时端点委托给平台层。 */
    s_communication_ready = YgProtocolRuntime_Init(MotorAxis_NodeId()) &&
                            YgProtocolStatusAdapter_Init() && bind_motor_stop() &&
                            YgProtocolOwner_BindServices() && bind_device_description();
#if defined(YG_APP_LOADER_IMAGE) && YG_APP_LOADER_IMAGE
    s_communication_ready = s_communication_ready && bind_boot_entry();
#endif

    if (!s_communication_ready)
    {
        /* 初始化失败不能留下已启动但无通信入口的功率输出。 */
        BspPwm_Disable();
    }
}

void App_Start(void)
{
    s_communication_ready = s_communication_ready && YgProtocolRuntime_Start();
    if (!s_communication_ready)
    {
        BspPwm_Disable();
    }
    AppTasks_Start();
}

void App_Process(void)
{
    MotorAxis_ServiceBackground();
    YgProtocolStatusAdapter_Refresh();
    YgProtocolRuntime_Service(BspSystem_TimeMs());
    service_boot(BspSystem_TimeMs());
    AppTasks_Process();
}

static void publish_identity(void)
{
    uint8_t payload[32];
    uint8_t state = 0U;
#if defined(YG_APP_LOADER_IMAGE) && YG_APP_LOADER_IMAGE
    state = BootHw_IsConfirmed() ? 1U : 2U;
#endif
    DeviceIdentity_EncodeInfo(
        &s_identity, MotorAxis_NodeId(), 1U, state, BootHw_ImageCrc32(), payload);
    (void)YgProtocolLink_SetMaintenanceInfoPage(1U, payload);
}

static motor_stop_owner_result_t request_stop(void *context, uint8_t source_node, uint16_t sequence)
{
    (void)context;
    return MotorStateMachine_RequestIdentifiedStop(source_node, sequence)
               ? MOTOR_STOP_OWNER_ACCEPTED
               : MOTOR_STOP_OWNER_DENIED;
}

static bool phases_disabled(void *context)
{
    (void)context;
    return BspPwm_ArePhasesDisabled();
}

static MotorControlResult submit_control(void *context, const MotorControlCommand *command)
{
    (void)context;
    return MotorStateMachine_SubmitControl(command);
}

static bool bind_motor_stop(void)
{
    static const motor_stop_owner_port_t owner = {request_stop, phases_disabled};
    yg_protocol_motor_service_t service;
    if (!MotorStopService_Init(&s_stop_service, &owner, NULL))
    {
        return false;
    }
    s_motor_adapter.stop_service = &s_stop_service;
    s_motor_adapter.submit_control = submit_control;
    s_motor_adapter.control_context = NULL;
    s_motor_adapter.lifecycle_handler = YgProtocolOwner_HandleLifecycle;
    s_motor_adapter.lifecycle_context = NULL;
    service.context = &s_motor_adapter;
    service.handler = yg_protocol_motor_adapter_handle;
    return YgProtocolLink_BindMotorService(&service);
}

static bool bind_device_description(void)
{
    /* 产品和硬件修订取共同目标清单；未知发布版本仍为 0，不以协议版本替代。 */
    yg_protocol_readonly_info_page0_t info = {.product_id = TARGET_PRODUCT,
                                              .hardware_revision = TARGET_HARDWARE,
                                              .application_version = TARGET_APP_VERSION,
                                              .boot_version = TARGET_LOADER_VERSION,
                                              .boot_api = TARGET_BOOT_API};
    yg_protocol_readonly_caps_page0_t capabilities = {
        .features = (1UL << 8U) | (1UL << 10U) | (1UL << 11U) | (1UL << 12U),
        /* 公布已接通的速度和两种开环路径；能力位不代表实机验收。 */
        .supported_modes = (1U << 2U) | (1U << 9U) | (1U << 10U),
        .max_full_frame = YG_PROTOCOL_MAX_FRAME_SIZE,
        .max_block_data = YG_PROTOCOL_MAX_PAYLOAD_SIZE};
    uint8_t payload[32], record[64];
    const uint32_t target[8] = {TARGET_MCU,
                                TARGET_LAYOUT,
                                TARGET_PARAM_ABI,
                                TARGET_SCHEMA,
                                TARGET_APP_BASE,
                                TARGET_APP_BYTES,
                                TARGET_LOADER_BYTES,
                                TARGET_IDENTITY_BASE};
#if !defined(YG_APP_LOADER_IMAGE) || !YG_APP_LOADER_IMAGE
    info.boot_api = 0U;
    info.boot_version = 0U;
#endif
    if (BspSystem_ReadIdentity(record) == BSP_OK)
    {
        (void)DeviceIdentity_Decode(record, &s_identity);
    }
    for (uint32_t i = 0; i < 8U; ++i)
    {
        YgProtocol_WriteU32Le(payload + i * 4U, target[i]);
    }
    publish_identity();
    return BspSystem_ReadUid(info.uid) == BSP_OK &&
           YgProtocolLink_SetDeviceDescription(&info, &capabilities) &&
           YgProtocolLink_SetMaintenanceInfoPage(2U, payload);
}

#if defined(YG_APP_LOADER_IMAGE) && YG_APP_LOADER_IMAGE
/* 56负责停机交接；57仅在持续健康且关相时持久确认当前试运行镜像。 */
#define BOOT_ENTRY_TIMEOUT_MS 1000U
#define BOOT_HEALTH_INTERVAL_MS 100U
static BootEntryRequest s_entry_request;
static BootEntryResult s_entry_result;
static uint64_t s_entry_ticket;
static uint32_t s_entry_started_ms;
static uint32_t s_health_checked_ms, s_observed_supervisor_epoch;
static bool s_entry_requested, s_boot_supported;
static uint8_t s_healthy_intervals;

static bool confirm_image(void *context, uint32_t version, uint32_t crc32)
{
    (void)context;
    uint32_t progress;
    bool confirmed = s_boot_supported && !s_entry_requested && s_healthy_intervals >= 11U &&
                     MotorStateMachine_ReadBootHealth(&progress) && s_communication_ready &&
                     YgProtocolLink_IsReady() && (BspCan_GetState() == BSP_CAN_READY) &&
                     MotorAxis_SaveIdle() && BspPwm_ArePhasesDisabled() &&
                     BootHw_Confirm(version, crc32);
    publish_identity();
    return confirmed;
}

static bool same_request(const BootEntryRequest *left, const BootEntryRequest *right)
{
    return left->boot_epoch == right->boot_epoch && left->source_node == right->source_node &&
           left->sequence == right->sequence;
}

static void poll_entry(uint32_t now_ms)
{
    if (!s_entry_requested ||
        (s_entry_result != BOOT_ENTRY_PENDING && s_entry_result != BOOT_ENTRY_READY))
    {
        return;
    }
    if ((uint32_t)(now_ms - s_entry_started_ms) >= BOOT_ENTRY_TIMEOUT_MS)
    {
        s_entry_result = BOOT_ENTRY_FAILED;
    }
    else if (MotorStateMachine_BootHandoffReady(s_entry_ticket) && MotorAxis_SaveIdle())
    {
        s_entry_result = BOOT_ENTRY_READY;
    }
    else
    {
        s_entry_result = BOOT_ENTRY_PENDING;
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
    if (!s_boot_supported || request == NULL || request->boot_epoch == 0U ||
        request->source_node == UINT8_MAX)
    {
        return;
    }
    if (s_entry_requested && !same_request(request, &s_entry_request))
    {
        reply->result = BOOT_ENTRY_DENIED;
        return;
    }
    now_ms = BspSystem_TimeMs();
    if (!s_entry_requested)
    {
        s_entry_request = *request;
        s_entry_ticket = request->boot_epoch;
        s_entry_started_ms = now_ms;
        s_entry_requested = true;
        s_entry_result = MotorStateMachine_RequestBootStop(s_entry_ticket) == MOTOR_CONTROL_ACCEPTED
                             ? BOOT_ENTRY_PENDING
                             : BOOT_ENTRY_DENIED;
    }
    poll_entry(now_ms);
    reply->result = s_entry_result;
    reply->ack_ticket = s_entry_result == BOOT_ENTRY_READY ? s_entry_ticket : 0U;
}

static void handle_tx_complete(void *context, uint64_t ticket, bool success)
{
    (void)context;
    uint32_t mask;
    bool permitted;
    if (!s_entry_requested || ticket == 0U || ticket != s_entry_ticket ||
        s_entry_result != BOOT_ENTRY_READY)
    {
        return;
    }
    mask = BspSystem_EnterCritical();
    /* 最终二次复核后撤销票据使用权限；复位失败也不能再次消费旧TX结果。 */
    permitted = success && s_entry_result == BOOT_ENTRY_READY &&
                (uint32_t)(BspSystem_TimeMs() - s_entry_started_ms) < BOOT_ENTRY_TIMEOUT_MS &&
                MotorStateMachine_BootHandoffReady(ticket) && MotorAxis_SaveIdle();
    s_entry_result = BOOT_ENTRY_FAILED;
    if (permitted)
    {
        (void)BootHw_EnterLoader(MotorAxis_NodeId());
    }
    BspSystem_ExitCritical(mask);
}

static bool bind_boot_entry(void)
{
    const BootEntryService service = {NULL, handle_entry, handle_tx_complete, confirm_image};
    s_boot_supported = BootHw_IsAvailable();
    publish_identity();
    return YgProtocolLink_BindBootService(&service);
}
#endif

static void service_boot(uint32_t now_ms)
{
#if defined(YG_APP_LOADER_IMAGE) && YG_APP_LOADER_IMAGE
    poll_entry(now_ms);
    if ((uint32_t)(now_ms - s_health_checked_ms) >= BOOT_HEALTH_INTERVAL_MS)
    {
        uint32_t progress = 0U;
        s_health_checked_ms = now_ms;
        bool healthy = MotorStateMachine_ReadBootHealth(&progress);
        healthy = healthy && progress != s_observed_supervisor_epoch && s_communication_ready &&
                  YgProtocolLink_IsReady() && (BspCan_GetState() == BSP_CAN_READY);
        s_observed_supervisor_epoch = progress;
        s_healthy_intervals =
            healthy ? (s_healthy_intervals < 11U ? s_healthy_intervals + 1U : 11U) : 0U;
        BootHw_Watchdog(healthy);
    }
#else
    (void)now_ms;
#endif
}
