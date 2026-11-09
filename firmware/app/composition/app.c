#include "app.h"

#include "app_protocol.h"
#include "motor_axis.h"
#include "motor_lifecycle.h"
#include "yg_protocol.h"

#include "boot_hw.h"
#include "bsp_pwm.h"
#include "bsp_system.h"
#include "device_identity.h"
#include "yg_protocol_common.h"

#include "application_config.h"
#if defined(YG_APP_LOADER_IMAGE) && YG_APP_LOADER_IMAGE
#include "loader_target.h"
#elif defined(APP_TARGET_AT32_F456) && APP_TARGET_AT32_F456
#include "at32f456_bringup.h"
#else
#include "vector_mini_st_g431.h"
#endif

/* 应用组合层：前台组织通信与Loader事务，中断仅推进电机及进展计数。
 * 硬件操作经BSP，运行准入和停机判定归MotorLifecycle，协议业务与状态换算归AppProtocol。 */

/* 应用看门狗观察双环进展；Loader健康确认另用独立的100 ms观察窗口。 */
#define APP_WATCHDOG_INTERVAL_MS 10U
#define BOOT_ENTRY_TIMEOUT_MS 1000U
#define BOOT_HEALTH_INTERVAL_MS 100U
#define BOOT_REQUIRED_HEALTHY_INTERVALS 11U

static bool s_communication_ready;
static DeviceIdentity s_identity;

/* ISR各自写入进展计数，前台独占看门狗观察窗口。 */
volatile uint32_t g_app_fast_samples;
static volatile uint32_t s_supervisor_ticks;
static uint32_t s_watchdog_last_ms;
static uint32_t s_watchdog_last_fast;
static uint32_t s_watchdog_last_slow;
static bool s_watchdog_enabled;

static void start_watchdog(void);
static void service_watchdog(void);
static bool bind_device_description(void);
static void service_boot(uint32_t now_ms);
#if defined(YG_APP_LOADER_IMAGE) && YG_APP_LOADER_IMAGE
static void init_boot_status(void);
#endif

/**
 * @brief 初始化电机、CAN 节点及运行时设备信息；协议回调和快照地址已静态配置。
 * @note main在Bsp_Init后调用一次；依次绑定，任一步失败便停止后续绑定并关相。
 *       此处不开放采样，采样由后续Bsp_Start启动。
 */
void App_Init(void)
{
    /* 电机所有者装载参数与标定并初始化；无效记录回退默认值。 */
    g_app_fast_samples = s_supervisor_ticks = 0U;
    s_watchdog_enabled = false;
    MotorAxis_Init();

    /* 创建CAN协议运行时并固定本机节点地址，此时不启动CAN硬件。 */
    s_communication_ready = Protocol_Init(MotorAxis_NodeId());
    if (s_communication_ready)
    {
        /* 设置主机可读取的设备身份、版本和能力信息。 */
        s_communication_ready = bind_device_description();
    }
#if defined(YG_APP_LOADER_IMAGE) && YG_APP_LOADER_IMAGE
    if (s_communication_ready)
    {
        /* Loader 回调已静态配置，此处只读取本机运行时的支持状态。 */
        init_boot_status();
    }
#endif

    if (!s_communication_ready)
    {
        /* 初始化失败不能留下已启动但无通信入口的功率输出。 */
        BspPwm_Disable();
    }
}

/**
 * @brief 启动已绑定的通信入口，并建立应用看门狗的双环观察窗口。
 * @note main在Bsp_Start后调用一次；通信未就绪时保持关相。
 */
void App_Start(void)
{
    /* 初始化成功后才启动CAN硬件，并保存启动结果。 */
    if (s_communication_ready)
    {
        s_communication_ready = Protocol_Start();
    }
    if (!s_communication_ready)
    {
        BspPwm_Disable();
    }
    start_watchdog();
}

/**
 * @brief 按顺序推进标定发布、状态刷新、通信、Loader事务和看门狗检查。
 * @note 仅由main前台循环调用。通信与Loader分别读取执行当时的时钟，
 *       避免用通信处理之前的时间判定其后的交接超时。
 */
void App_Process(void)
{
    MotorAxis_ProcessPositionPlan();
    MotorAxis_ProcessCalibrationPublication(service_watchdog);
    AppProtocol_Refresh();
    Protocol_Process(BspSystem_TimeMs());
    service_boot(BspSystem_TimeMs());
    service_watchdog();
}

/**
 * @brief 执行一次20 kHz电机快环，完成后递增快环进展计数。
 * @note 由ADC采样完成路径调用；计数供前台看门狗检查，不代表运行模式或控制成功。
 */
void BspApp_OnFastSample(void)
{
    MotorAxis_SampleFast();
    MotorAxis_ControlFast();
    ++g_app_fast_samples;
}

/**
 * @brief 执行一次电机监督任务，完成后递增监督进展计数。
 * @note 由监督定时器中断调用；与快环计数一起供前台判断任务是否持续推进。
 */
void App_SupervisorTick(void)
{
    MotorAxis_SupervisorTick();
    ++s_supervisor_ticks;
}

/**
 * @brief 将BSP的CAN接收事件交给协议接收队列。
 * @note CAN中断回调，只执行有界帧搬运；业务解析由App_Process中的通信服务完成。
 */
void BspApp_OnCanReceive(void)
{
    Protocol_RxInterrupt();
}

/**
 * @brief 在应用健康监督失败时关相并永久停止正常执行。
 * @note 屏蔽可屏蔽中断后关闭PWM，不再喂狗；本函数不返回。
 */
static void halt_disabled(void)
{
    (void)BspSystem_EnterCritical();
    BspPwm_Disable();
    for (;;)
    {
        /* 应用失败停止健康喂狗；硬件故障IRQ由BSP独立关断。 */
    }
}

/**
 * @brief 启动BSP应用看门狗，并记录当前时间及双环进展作为首次检查基线。
 * @note 未配置看门狗时允许继续运行；其他启动错误进入关相停留路径。
 *       仅在App_Start的前台启动阶段调用。
 */
static void start_watchdog(void)
{
#if defined(YG_APP_LOADER_IMAGE) && YG_APP_LOADER_IMAGE
    /* 继承Loader长周期WDT；service_boot独占健康检查、确认时限与喂狗。 */
    s_watchdog_enabled = false;
#else
    BspResult result = BspWatchdog_Start();
    if (result != BSP_OK && result != BSP_NOT_SUPPORTED)
    {
        halt_disabled();
    }
    s_watchdog_enabled = result == BSP_OK;
#endif
    s_watchdog_last_ms = BspSystem_TimeMs();
    s_watchdog_last_fast = g_app_fast_samples;
    s_watchdog_last_slow = s_supervisor_ticks;
}

/**
 * @brief 每满10 ms检查快环和监督任务均有进展，满足条件后喂应用看门狗。
 * @note 前台调用；任一任务停滞即关相停留。无符号时间差保留毫秒计数回绕语义。
 */
static void service_watchdog(void)
{
    if (!s_watchdog_enabled)
    {
        return;
    }
    uint32_t now_ms = BspSystem_TimeMs();
    if ((uint32_t)(now_ms - s_watchdog_last_ms) < APP_WATCHDOG_INTERVAL_MS)
    {
        return;
    }
    if (g_app_fast_samples == s_watchdog_last_fast || s_supervisor_ticks == s_watchdog_last_slow)
    {
        halt_disabled();
    }
    s_watchdog_last_fast = g_app_fast_samples;
    s_watchdog_last_slow = s_supervisor_ticks;
    s_watchdog_last_ms = now_ms;
    BspWatchdog_Feed();
}

/**
 * @brief 将生产身份、当前节点、启动确认状态和镜像CRC更新到GET_INFO第1页。
 * @note 前台调用，仅更新协议缓存，不直接发帧或写Flash；独立APP的启动状态为0。
 */
static void publish_identity(void)
{
    uint8_t payload[DEVICE_IDENTITY_INFO_BYTES];
    uint8_t boot_state = 0U;
#if defined(YG_APP_LOADER_IMAGE) && YG_APP_LOADER_IMAGE
    boot_state = BootHw_IsConfirmed() ? 1U : 2U;
#endif
    DeviceIdentity_EncodeInfo(
        &s_identity, MotorAxis_NodeId(), 1U, boot_state, BootHw_ImageCrc32(), payload);
    (void)Protocol_SetInfoPage(1U, payload);
}

/**
 * @brief 从目标清单和生产记录装配设备版本、能力及布局页面。
 * @return 芯片UID读取、设备描述绑定和布局页绑定均成功时返回true。
 * @note 仅启动前台调用；生产身份读取失败不阻止其他信息绑定。
 *       独立APP公布的Loader版本及API为0，页面字段顺序属于既有协议契约。
 */
static bool bind_device_description(void)
{
    /* 产品和硬件修订取共同目标清单；未知发布版本仍为 0，不以协议版本替代。 */
    ProtocolInfo info = {.product_id = TARGET_PRODUCT,
                         .hardware_revision = TARGET_HARDWARE,
                         .application_version = TARGET_APP_VERSION,
                         .boot_version = TARGET_LOADER_VERSION,
                         .boot_api = TARGET_BOOT_API};
    /* supported_modes使用线路模式编号的位图，不是内部MotorControlMode枚举。 */
    ProtocolCaps capabilities = {
        .features = (1UL << 8U) | (1UL << 10U) | (1UL << 11U) | (1UL << 12U),
        .supported_modes = (1U << 1) | (1U << 2) | (1U << 3) | (1U << 6) | (1U << 9) | (1U << 10),
        .max_full_frame = YG_PROTOCOL_MAX_FRAME_SIZE,
        .max_block_data = YG_PROTOCOL_MAX_PAYLOAD_SIZE};
    uint8_t target_payload[DEVICE_IDENTITY_INFO_BYTES];
    uint8_t identity_record[DEVICE_IDENTITY_RECORD_BYTES];
    const uint32_t target_description[] = {
        TARGET_MCU,
        TARGET_LAYOUT,
        TARGET_PARAM_ABI,
        TARGET_SCHEMA,
        TARGET_APP_BASE,
        TARGET_APP_BYTES,
        TARGET_LOADER_BYTES,
        TARGET_IDENTITY_BASE,
    };
#if !defined(YG_APP_LOADER_IMAGE) || !YG_APP_LOADER_IMAGE
    info.boot_api = 0U;
    info.boot_version = 0U;
#endif
    if (BspSystem_ReadIdentity(identity_record) == BSP_OK)
    {
        (void)DeviceIdentity_Decode(identity_record, &s_identity);
    }
    for (uint32_t i = 0; i < sizeof(target_description) / sizeof(target_description[0]); ++i)
    {
        YgProtocol_WriteU32Le(target_payload + i * 4U, target_description[i]);
    }
    publish_identity();
    return BspSystem_ReadUid(info.uid) == BSP_OK && Protocol_SetInfo(&info, &capabilities) &&
           Protocol_SetInfoPage(2U, target_payload);
}

#if defined(YG_APP_LOADER_IMAGE) && YG_APP_LOADER_IMAGE
/* 56负责停机交接；57仅在持续健康且关相时持久确认当前试运行镜像。 */
static BootEntryRequest s_entry_request;
static BootEntryResult s_entry_result;
static uint32_t s_entry_started_ms;
static uint32_t s_health_checked_ms;
static uint32_t s_observed_supervisor_ticks;
/* requested表示事务已登记，包括拒绝/失败终态；终态不能因同一请求重试而恢复。 */
static bool s_entry_requested;
static bool s_boot_supported;
static uint8_t s_healthy_intervals;

/**
 * @brief 处理镜像确认请求，复核连续健康记录、当前健康状态、通信及数字关相。
 * @param context 未使用的协议服务上下文，可为NULL。
 * @param version 主机核对的当前镜像版本。
 * @param crc32 主机核对的当前镜像CRC32。
 * @return 全部条件满足且底层确认成功时返回true，否则返回false。
 * @note 前台回调；禁止与进入Loader事务并行。无论成功与否都刷新身份页。
 */
static bool confirm_image(void *context, uint32_t version, uint32_t crc32)
{
    (void)context;
    uint32_t supervisor_ticks;
    bool confirmed = s_boot_supported && !s_entry_requested &&
                     s_healthy_intervals >= BOOT_REQUIRED_HEALTHY_INTERVALS &&
                     MotorLifecycle_ReadBootHealth(&supervisor_ticks) && s_communication_ready &&
                     Protocol_IsReady() && BspPwm_ArePhasesDisabled() &&
                     BootHw_Confirm(version, crc32);
    publish_identity();
    return confirmed;
}

/**
 * @brief 比较两次Loader请求是否属于同一事务，供重发查询复用已登记的结果。
 * @param left 第一个有效请求，非NULL。
 * @param right 第二个有效请求，非NULL。
 * @return 随机事务标识、来源节点和序号全部一致时返回true。
 * @note 纯比较；boot_epoch是主机生成的事务标识，不是设备持久启动次数。
 */
static bool same_request(const BootEntryRequest *left, const BootEntryRequest *right)
{
    return left->boot_epoch == right->boot_epoch && left->source_node == right->source_node &&
           left->sequence == right->sequence;
}

/**
 * @brief 根据交接截止时间和实际停机状态推进已登记的Loader请求。
 * @param now_ms 当前单调毫秒计数，允许32位自然回绕。
 * @note 前台调用；仅推进PENDING/READY，超时优先判定，拒绝或失败终态不再恢复。
 *       READY仅表示可回复交接票据，不执行复位。
 */
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

/**
 * @brief 登记或查询进入Loader事务，并返回当前结果及可用的交接票据。
 * @param context 未使用的协议服务上下文，可为NULL。
 * @param request 借用的请求；NULL、零事务标识或无效来源返回UNAVAILABLE。
 * @param reply 调用方持有的结果；NULL时不处理请求。
 * @note 前台回调；首次有效请求复制身份并请求锁定停机，相同请求可查询，其他事务被拒绝。
 *       停机请求受理不代表已关相，票据必须等poll_entry确认READY后才给出。
 */
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

/**
 * @brief 在交接回复物理发送完成后，复核票据、时限和停机状态并尝试复位进Loader。
 * @param context 未使用的协议服务上下文，可为NULL。
 * @param ticket 通信层返回的64位交接票据，必须匹配当前READY事务。
 * @param success 对应回复帧是否物理发送成功；仅入队不能视为成功。
 * @note 前台回调；最终复核和票据失效化在同一临界区内完成。
 *       发送或复位失败均不恢复票据，防止重复消费旧发送结果。
 */
static void handle_tx_complete(void *context, uint64_t ticket, bool success)
{
    (void)context;
    uint32_t saved_interrupt_mask;
    bool entry_permitted;
    if (!s_entry_requested || ticket == 0U || ticket != s_entry_request.boot_epoch ||
        s_entry_result != BOOT_ENTRY_READY)
    {
        return;
    }
    saved_interrupt_mask = BspSystem_EnterCritical();
    /* 最终二次复核后撤销票据使用权限；复位失败也不能再次消费旧TX结果。 */
    entry_permitted = success && s_entry_result == BOOT_ENTRY_READY &&
                      (uint32_t)(BspSystem_TimeMs() - s_entry_started_ms) < BOOT_ENTRY_TIMEOUT_MS &&
                      MotorLifecycle_IsStopped();
    s_entry_result = BOOT_ENTRY_FAILED;
    if (entry_permitted)
    {
        (void)BootHw_EnterLoader(MotorAxis_NodeId());
    }
    BspSystem_ExitCritical(saved_interrupt_mask);
}

const BootEntryService Protocol_Boot = {NULL, handle_entry, handle_tx_complete, confirm_image};

/** @brief 记录本次运行时读取的 Loader 可用状态，并刷新身份信息。 */
static void init_boot_status(void)
{
    s_boot_supported = BootHw_IsAvailable();
    publish_identity();
}
#else
const BootEntryService Protocol_Boot = {0};
#endif

/**
 * @brief 推进Loader停机事务，并按100 ms间隔检查应用健康以维护继承看门狗。
 * @param now_ms 通信服务执行之后读取的当前毫秒计数，允许32位自然回绕。
 * @note 前台调用；健康要求监督拍数有进展、生命周期输入有效且通信就绪。
 *       连续11次健康才满足镜像确认门槛，任一次失败清零；独立APP编译为空操作。
 */
static void service_boot(uint32_t now_ms)
{
#if defined(YG_APP_LOADER_IMAGE) && YG_APP_LOADER_IMAGE
    poll_entry(now_ms);
    if ((uint32_t)(now_ms - s_health_checked_ms) < BOOT_HEALTH_INTERVAL_MS)
    {
        return;
    }
    uint32_t supervisor_ticks = 0U;
    s_health_checked_ms = now_ms;
    bool healthy = MotorLifecycle_ReadBootHealth(&supervisor_ticks);
    healthy = healthy && supervisor_ticks != s_observed_supervisor_ticks && s_communication_ready &&
              Protocol_IsReady();
    s_observed_supervisor_ticks = supervisor_ticks;
    if (!healthy)
    {
        s_healthy_intervals = 0U;
    }
    else if (s_healthy_intervals < BOOT_REQUIRED_HEALTHY_INTERVALS)
    {
        ++s_healthy_intervals;
    }
    else
    {
        s_healthy_intervals = BOOT_REQUIRED_HEALTHY_INTERVALS;
    }
    BootHw_Watchdog(healthy);
#else
    (void)now_ms;
#endif
}
