#include "app.h"

#include "boot_hw.h"
#include "bsp_system.h"
#include "motor_lifecycle.h"
#include "yg_protocol_link.h"
#include "bsp_pwm.h"
#include "motor_axis.h"
#include "app_protocol.h"
#include "application_config.h"
#if defined(APP_TARGET_AT32_F456) && APP_TARGET_AT32_F456
#include "r40_f456_bringup.h"
#else
#include "vector_mini_st_g431.h"
#endif
#include "yg_protocol_common.h"
#include "device_identity.h"

#if APP_ENABLE_MOTOR_BENCH
#include "motor_bench.h"
#endif

/* 应用生命周期、调度和健康监督；硬件操作经BSP，协议装配归AppProtocol。 */

static bool s_communication_ready;
static DeviceIdentity s_identity;

static volatile uint32_t s_fast_samples, s_supervisor_ticks;
static uint32_t s_watchdog_last_ms, s_watchdog_last_fast, s_watchdog_last_slow;
static bool s_watchdog_enabled;

static void halt_disabled(void)
{
    (void)BspSystem_EnterCritical();
    BspPwm_Disable();
    for (;;)
    {
        /* 应用失败停止健康喂狗；硬件故障IRQ由BSP独立关断。 */
    }
}

/* 公开入口在前，绑定及Loader交接细节在后。 */
static bool bind_device_description(void);
static void service_boot(uint32_t now_ms);
#if defined(YG_APP_LOADER_IMAGE) && YG_APP_LOADER_IMAGE
static bool bind_boot_entry(void);
#endif

void App_Init(void)
{
    /* 电机所有者装载参数与标定并初始化；无效记录回退默认值。 */
    s_fast_samples = s_supervisor_ticks = 0U;
    s_watchdog_enabled = false;
    MotorAxis_Init();

    /* CAN FD 是唯一通信入口；硬件启动由运行时端点委托给平台层。 */
    s_communication_ready = YgProtocolRuntime_Init(MotorAxis_NodeId()) && AppProtocol_Init() &&
                            bind_device_description();
#if APP_ENABLE_MOTOR_BENCH
    s_communication_ready =
        s_communication_ready && YgProtocolLink_BindBenchService(MotorBench_Handle, NULL);
#endif
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
    BspResult result = BspWatchdog_Start();
    if (result != BSP_OK && result != BSP_NOT_SUPPORTED)
    {
        halt_disabled();
    }
    s_watchdog_enabled = result == BSP_OK;
    s_watchdog_last_ms = BspSystem_TimeMs();
    s_watchdog_last_fast = s_fast_samples;
    s_watchdog_last_slow = s_supervisor_ticks;
}

void App_Process(void)
{
    MotorAxis_ProcessCalibrationPublication();
    AppProtocol_Refresh();
    YgProtocolRuntime_Service(BspSystem_TimeMs());
    service_boot(BspSystem_TimeMs());
    if (!s_watchdog_enabled)
    {
        return;
    }
    uint32_t now = BspSystem_TimeMs();
    if ((uint32_t)(now - s_watchdog_last_ms) >= 10U)
    {
        if (s_fast_samples == s_watchdog_last_fast || s_supervisor_ticks == s_watchdog_last_slow)
        {
            halt_disabled();
        }
        s_watchdog_last_fast = s_fast_samples;
        s_watchdog_last_slow = s_supervisor_ticks;
        s_watchdog_last_ms = now;
        BspWatchdog_Feed();
    }
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
        .supported_modes = (1U << 1) | (1U << 2) | (1U << 3) | (1U << 6) | (1U << 9) | (1U << 10),
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
static uint32_t s_entry_started_ms;
static uint32_t s_health_checked_ms, s_observed_supervisor_epoch;
static bool s_entry_requested, s_boot_supported;
static uint8_t s_healthy_intervals;

static bool confirm_image(void *context, uint32_t version, uint32_t crc32)
{
    (void)context;
    uint32_t progress;
    bool confirmed = s_boot_supported && !s_entry_requested && s_healthy_intervals >= 11U &&
                     MotorLifecycle_ReadBootHealth(&progress) && s_communication_ready &&
                     YgProtocolLink_IsReady() && BspPwm_ArePhasesDisabled() &&
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
    else if (MotorLifecycle_IsStopped())
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
        s_entry_started_ms = now_ms;
        s_entry_requested = true;
        s_entry_result = MotorLifecycle_LockStop() ? BOOT_ENTRY_PENDING : BOOT_ENTRY_DENIED;
    }
    poll_entry(now_ms);
    reply->result = s_entry_result;
    reply->ack_ticket = s_entry_result == BOOT_ENTRY_READY ? s_entry_request.boot_epoch : 0U;
}

static void handle_tx_complete(void *context, uint64_t ticket, bool success)
{
    (void)context;
    uint32_t mask;
    bool permitted;
    if (!s_entry_requested || ticket == 0U || ticket != s_entry_request.boot_epoch ||
        s_entry_result != BOOT_ENTRY_READY)
    {
        return;
    }
    mask = BspSystem_EnterCritical();
    /* 最终二次复核后撤销票据使用权限；复位失败也不能再次消费旧TX结果。 */
    permitted = success && s_entry_result == BOOT_ENTRY_READY &&
                (uint32_t)(BspSystem_TimeMs() - s_entry_started_ms) < BOOT_ENTRY_TIMEOUT_MS &&
                MotorLifecycle_IsStopped();
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
        bool healthy = MotorLifecycle_ReadBootHealth(&progress);
        healthy = healthy && progress != s_observed_supervisor_epoch && s_communication_ready &&
                  YgProtocolLink_IsReady();
        s_observed_supervisor_epoch = progress;
        s_healthy_intervals =
            healthy ? (s_healthy_intervals < 11U ? s_healthy_intervals + 1U : 11U) : 0U;
        BootHw_Watchdog(healthy);
    }
#else
    (void)now_ms;
#endif
}

void App_FastTick(void)
{
    MotorAxis_FastTick();
    ++s_fast_samples;
}

void BspApp_OnControlTick(void)
{
    App_SupervisorTick();
    ++s_supervisor_ticks;
}

void BspApp_OnCanReceive(void)
{
    YgProtocolCanfd_RxIrqHandler();
}

void App_SupervisorTick(void)
{
    MotorAxis_SupervisorTick();
}
