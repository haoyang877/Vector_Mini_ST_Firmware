#include "app.h"
#include "app_context.h"

#include "app_protocol.h"
#include "motor_axis.h"
#include "motor_lifecycle.h"
#include "yg_protocol.h"

#include "boot_hw.h"
#include "bsp_system.h"
#include "device_identity.h"
#include "yg_protocol_common.h"

#include "application_config.h"
#if defined(APP_TARGET_AT32_F456) && APP_TARGET_AT32_F456
#include "at32f456_bringup.h"
#else
#include "vector_mini_st_g431.h"
#endif

/* 应用组合层：前台组织通信与Loader事务，中断仅推进电机及进展计数。
 * 硬件操作经BSP，运行准入和停机判定归MotorLifecycle，协议业务与状态换算归AppProtocol。 */

/* 应用看门狗观察双环进展；Loader健康确认另用独立的100 ms观察窗口。 */
#define APP_WATCHDOG_INTERVAL_MS 10U

static void start_watchdog(AppContext *app);
static void service_watchdog(AppContext *app);
static bool bind_device_description(AppContext *app);

static void publish_identity(void *context);

/* Loader 提交前读取活动轴的节点号，沿用板级交接时的身份。 */
static uint8_t read_node_id(void *context)
{
    AppContext *app = context;
    return MotorAxis_NodeId(&app->axis);
}

static void watchdog_progress(void *context)
{
    service_watchdog(context);
}

/**
 * @brief 初始化电机、CAN 节点及运行时设备信息；协议回调和快照地址已静态配置。
 * @note main在Bsp_Init后调用一次；依次绑定，任一步失败便停止后续绑定并关相。
 *       此处不开放采样，采样由后续Bsp_Start启动。
 */
void App_Init(AppContext *app)
{
    /* 电机所有者装载参数与标定并初始化；无效记录回退默认值。 */
    if (app->initialized)
    {
        return;
    }
    app->initialized = true;
    app->fast_samples = app->supervisor_ticks = 0U;
    app->watchdog_enabled = false;
    MotorAxis_Init(&app->axis);
    AppProtocol_Init(&app->protocol, &app->endpoint, &app->axis.lifecycle);

    /* 创建CAN协议运行时并固定本机节点地址，此时不启动CAN硬件。 */
    app->communication_ready = Protocol_Init(&app->endpoint, MotorAxis_NodeId(&app->axis));
    if (app->communication_ready)
    {
        /* 设置主机可读取的设备身份、版本和能力信息。 */
        app->communication_ready = bind_device_description(app);
    }
    app->loader.life = &app->axis.lifecycle;
    app->loader.endpoint = &app->endpoint;
    app->loader.communication_ready = &app->communication_ready;
    app->loader.read_node_id = read_node_id;
    app->loader.publish_identity = publish_identity;
    app->loader.identity_context = app;
    if (app->communication_ready)
    {
        AppLoader_Init(&app->loader);
    }

    if (!app->communication_ready)
    {
        /* 初始化失败不能留下已启动但无通信入口的功率输出。 */
        app->axis.hardware->power.disable();
    }
}

/**
 * @brief 启动已绑定的通信入口，并建立应用看门狗的双环观察窗口。
 * @note main在Bsp_Start后调用一次；通信未就绪时保持关相。
 */
void App_Start(AppContext *app)
{
    /* 初始化成功后才启动CAN硬件，并保存启动结果。 */
    if (app->communication_ready)
    {
        app->communication_ready = Protocol_Start(&app->endpoint);
    }
    if (!app->communication_ready)
    {
        app->axis.hardware->power.disable();
    }
    start_watchdog(app);
}

/**
 * @brief 按顺序推进标定发布、状态刷新、通信、Loader事务和看门狗检查。
 * @note 仅由main前台循环调用。通信与Loader分别读取执行当时的时钟，
 *       避免用通信处理之前的时间判定其后的交接超时。
 */
void App_Process(AppContext *app)
{
    MotorAxis_ProcessPositionPlan(&app->axis);
    MotorCalibration_ProcessPublication(&app->axis.calibration, watchdog_progress, app);
    AppProtocol_Refresh(&app->protocol);
    Protocol_Process(&app->endpoint, BspSystem_TimeMs());
    AppLoader_Process(&app->loader, BspSystem_TimeMs());
    service_watchdog(app);
}

/**
 * @brief 执行一次电机监督任务，完成后递增监督进展计数。
 * @note 由监督定时器中断调用；与快环计数一起供前台判断任务是否持续推进。
 */
void App_SupervisorTick(MotorAxis *axis, volatile uint32_t *progress)
{
    MotorAxis_SupervisorTick(axis);
    ++*progress;
}

/**
 * @brief 在应用健康监督失败时关相并永久停止正常执行。
 * @note 屏蔽可屏蔽中断后关闭PWM，不再喂狗；本函数不返回。
 */
static void halt_disabled(AppContext *app)
{
    (void)BspSystem_EnterCritical();
    app->axis.hardware->power.disable();
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
static void start_watchdog(AppContext *app)
{
    BspResult result = BspWatchdog_Start();
    if (result != BSP_OK && result != BSP_NOT_SUPPORTED)
    {
        halt_disabled(app);
    }
    app->watchdog_enabled = result == BSP_OK;
    app->watchdog_last_ms = BspSystem_TimeMs();
    app->watchdog_last_fast = app->fast_samples;
    app->watchdog_last_slow = app->supervisor_ticks;
}

/**
 * @brief 每满10 ms检查快环和监督任务均有进展，满足条件后喂应用看门狗。
 * @note 前台调用；任一任务停滞即关相停留。无符号时间差保留毫秒计数回绕语义。
 */
static void service_watchdog(AppContext *app)
{
    if (!app->watchdog_enabled)
    {
        return;
    }
    uint32_t now_ms = BspSystem_TimeMs();
    if ((uint32_t)(now_ms - app->watchdog_last_ms) < APP_WATCHDOG_INTERVAL_MS)
    {
        return;
    }
    if (app->fast_samples == app->watchdog_last_fast ||
        app->supervisor_ticks == app->watchdog_last_slow)
    {
        halt_disabled(app);
    }
    app->watchdog_last_fast = app->fast_samples;
    app->watchdog_last_slow = app->supervisor_ticks;
    app->watchdog_last_ms = now_ms;
    BspWatchdog_Feed();
}

/**
 * @brief 将生产身份、当前节点、启动确认状态和镜像CRC更新到GET_INFO第1页。
 * @note 前台调用，仅更新协议缓存，不直接发帧或写Flash；独立APP的启动状态为0。
 */
static void publish_identity(void *context)
{
    AppContext *app = context;
    uint8_t payload[DEVICE_IDENTITY_INFO_BYTES];
    uint8_t boot_state = 0U;
#if defined(YG_APP_LOADER_IMAGE) && YG_APP_LOADER_IMAGE
    boot_state = BootHw_IsConfirmed() ? 1U : 2U;
#endif
    DeviceIdentity_EncodeInfo(
        &app->identity, MotorAxis_NodeId(&app->axis), 1U, boot_state, BootHw_ImageCrc32(), payload);
    (void)Protocol_SetInfoPage(&app->endpoint, 1U, payload);
}

/**
 * @brief 从目标清单和生产记录装配设备版本、能力及布局页面。
 * @return 芯片UID读取、设备描述绑定和布局页绑定均成功时返回true。
 * @note 仅启动前台调用；生产身份读取失败不阻止其他信息绑定。
 *       独立APP公布的Loader版本及API为0，页面字段顺序属于既有协议契约。
 */
static bool bind_device_description(AppContext *app)
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
        (void)DeviceIdentity_Decode(identity_record, &app->identity);
    }
    for (uint32_t i = 0; i < sizeof(target_description) / sizeof(target_description[0]); ++i)
    {
        YgProtocol_WriteU32Le(target_payload + i * 4U, target_description[i]);
    }
    publish_identity(app);
    return BspSystem_ReadUid(info.uid) == BSP_OK &&
           Protocol_SetInfo(&app->endpoint, &info, &capabilities) &&
           Protocol_SetInfoPage(&app->endpoint, 2U, target_payload);
}
