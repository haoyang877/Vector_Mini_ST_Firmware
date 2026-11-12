#include "app.h"

#include "app_protocol.h"
#include "motor_axis.h"
#include "motor_state_machine.h"
#include "yg_protocol.h"

#include "boot_hw.h"
#include "bsp_system.h"
#include "device_identity.h"
#include "yg_protocol_common.h"

#include "application_config.h"
#include "board_config.h"
#include "can_bitrate.h"
#if defined(YG_APP_LOADER_IMAGE) && YG_APP_LOADER_IMAGE
#include "loader_target.h"
#elif defined(APP_TARGET_AT32_F456) && APP_TARGET_AT32_F456
#include "at32f456_bringup.h"
#else
#include "vector_mini_st_g431.h"
#endif

/* 应用组合层：前台组织通信与Loader事务，中断仅推进电机及进展计数。
 * 硬件操作经BSP，运行准入和停机判定归MotorStateMachine，协议业务与状态换算归AppProtocol。 */

static bool configure_protocol_device_info(AppContext *app);

static void handle_parameter_request(void *context,
                                     const ProtocolParameterRequest *request,
                                     ProtocolParameterReply *reply)
{
    MotorAxis *axis = context;
    ParameterStatus result;
    ParameterValue value = {0};
    switch (request->type)
    {
    case PROTOCOL_READ_PARAMETER:
    case PROTOCOL_READ_SAVED_PARAMETER:
        result = MotorAxis_ReadParameter(
            axis, request->id, request->type == PROTOCOL_READ_SAVED_PARAMETER, &value);
        break;
    case PROTOCOL_WRITE_PARAMETER:
    {
        ParameterWriteRequest write = {.parameter_id = request->id,
                                       .value_type = request->value_type,
                                       .expected_revision = request->expected_revision,
                                       .raw_value = request->value};
        result = MotorAxis_WriteParameter(axis, &write);
        if (result == PARAMETER_OK)
        {
            result = MotorAxis_ReadParameter(axis, request->id, false, &value);
        }
        break;
    }
    case PROTOCOL_SAVE_PARAMETER:
        result = MotorAxis_SaveParameters(axis, request->expected_revision);
        break;
    case PROTOCOL_RESTORE_CONTROL_DEFAULTS:
        result = MotorAxis_RestoreControlDefaults(axis, request->expected_revision);
        break;
    default:
        result = PARAMETER_UNSUPPORTED;
        break;
    }
    ProtocolStatus status = PROTOCOL_FAILED;
    uint16_t detail = 7U;
    switch (result)
    {
    case PARAMETER_OK:
        status = PROTOCOL_OK;
        detail = 0U;
        break;
    case PARAMETER_UNSUPPORTED:
        status = PROTOCOL_UNSUPPORTED;
        detail = 1U;
        break;
    case PARAMETER_WRONG_STATE:
        status = PROTOCOL_DENIED;
        detail = 5U;
        break;
    case PARAMETER_READ_ONLY:
        status = PROTOCOL_DENIED;
        detail = 2U;
        break;
    case PARAMETER_REVISION_CONFLICT:
        status = PROTOCOL_DENIED;
        detail = 6U;
        break;
    case PARAMETER_BAD_TYPE:
    case PARAMETER_INVALID_ARGUMENT:
        status = PROTOCOL_INVALID_ARGUMENT;
        detail = 3U;
        break;
    case PARAMETER_OUT_OF_RANGE:
        status = PROTOCOL_OUT_OF_RANGE;
        detail = 4U;
        break;
    default:
        break;
    }
    *reply = (ProtocolParameterReply){
        .status = status,
        .detail = detail,
        .id = value.parameter_id,
        .value_type = value.value_type,
        .access = value.access,
        .value = value.raw_value,
        .revision = axis->parameters.revision,
        .saved_revision = axis->parameters.saved_revision,
        .flags = (axis->parameters.saved_revision != 0U ? 1U : 0U) |
                 (axis->parameters.revision != axis->parameters.saved_revision ? 2U : 0U) |
                 (axis->parameters.save_status == PARAMETER_PERSIST_FAILED ? 4U : 0U)};
}

static void update_identity_info_page(void *context);

/**
 * @brief 初始化电机、CAN 节点及运行时设备信息，绑定本机协议回调。
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
    MotorAxis_Init(&app->axis);
    AppProtocol_Init(&app->protocol, &app->endpoint, &app->axis.state_machine);
    AppMotorConfiguration_Init(&app->motor_configuration, &app->axis, &app->endpoint);

    app->endpoint.parameters = (ProtocolParameters){&app->axis, handle_parameter_request};
    app->communication_ready =
        Protocol_Init(&app->endpoint, app->axis.configuration.device.node_id);
#if BSP_BOARD == BSP_BOARD_AT32F456
    if (app->communication_ready &&
        app->axis.configuration.device.can_bitrate_profile != CAN_BITRATE_DEFAULT_PROFILE)
    {
        app->communication_ready =
            BspCan_ConfigureBitrate(app->axis.configuration.device.can_bitrate_profile);
    }
#endif
    if (app->communication_ready)
    {
        app->communication_ready = configure_protocol_device_info(app);
    }
#if defined(YG_APP_LOADER_IMAGE) && YG_APP_LOADER_IMAGE
    app->loader = (AppLoader){.state_machine = &app->axis.state_machine,
                              .endpoint = &app->endpoint,
                              .communication_ready = &app->communication_ready,
                              .node_id = &app->axis.configuration.device.node_id,
                              .publish_identity = update_identity_info_page,
                              .identity_context = app};
    if (app->communication_ready)
    {
        AppLoader_Init(&app->loader);
    }
#endif
    if (!app->communication_ready)
    {
        app->axis.hardware->power.disable();
    }
}

/**
 * @brief 启动已绑定的通信入口。
 * @note main在Bsp_Start后调用一次；通信未就绪时保持关相。
 */
void App_Start(AppContext *app)
{
    if (app->communication_ready)
    {
        app->communication_ready = Protocol_Start(&app->endpoint);
    }
    if (!app->communication_ready)
    {
        app->axis.hardware->power.disable();
    }
}

/**
 * @brief 按顺序推进位置建表、标定发布、通信及可选的 Loader 事务。
 * @note 仅由main前台循环调用。通信与Loader分别读取执行当时的时钟，
 *       避免用通信处理之前的时间判定其后的交接超时。
 */
void App_Process(AppContext *app)
{
    MotorAxis_ProcessPositionPlan(&app->axis);
    MotorCalibration_ProcessPublication(&app->axis.calibration);
    Protocol_Process(&app->endpoint, BspSystem_TimeMs());
    if (app->motor_configuration.restart_requested)
    {
        AppMotorConfiguration_Process(&app->motor_configuration, BspSystem_TimeMs());
    }
#if defined(YG_APP_LOADER_IMAGE) && YG_APP_LOADER_IMAGE
    AppLoader_Process(&app->loader, BspSystem_TimeMs());
#endif
}

/**
 * @brief 将生产身份、当前节点、启动确认状态和镜像CRC更新到GET_INFO第1页。
 * @note 前台调用，仅更新协议缓存，不直接发帧或写Flash；独立APP的启动状态为0。
 */
static void update_identity_info_page(void *context)
{
    AppContext *app = context;
    uint8_t payload[DEVICE_IDENTITY_INFO_BYTES];
    uint8_t boot_state = 0U;
#if defined(YG_APP_LOADER_IMAGE) && YG_APP_LOADER_IMAGE
    boot_state = BootHw_IsConfirmed() ? 1U : 2U;
#endif
    DeviceIdentity_EncodeInfo(&app->identity,
                              app->axis.configuration.device.node_id,
                              1U,
                              boot_state,
                              BootHw_ImageCrc32(),
                              payload);
    (void)Protocol_SetInfoPage(&app->endpoint, 1U, payload);
}

/**
 * @brief 从目标清单和生产记录装配设备版本、能力及布局页面。
 * @return 芯片UID读取、设备描述绑定和布局页绑定均成功时返回true。
 * @note 仅启动前台调用；生产身份读取失败不阻止其他信息绑定。
 *       独立APP公布的Loader版本及API为0，页面字段顺序属于既有协议契约。
 */
static bool configure_protocol_device_info(AppContext *app)
{
    /* 产品和硬件修订取共同目标清单；未知发布版本仍为 0，不以协议版本替代。 */
    ProtocolInfo info = {.product_id = TARGET_PRODUCT,
                         .hardware_revision = TARGET_HARDWARE,
                         .application_version = TARGET_APP_VERSION,
                         .boot_version = TARGET_LOADER_VERSION,
                         .boot_api = TARGET_BOOT_API};
    /* supported_modes使用线路模式编号的位图，不是内部MotorControlMode枚举。 */
    ProtocolCaps capabilities = {
        .default_watchdog_ms = PROTOCOL_MOTION_LEASE_MS,
        .max_watchdog_ms = PROTOCOL_MOTION_LEASE_MS,
        .features = PROTOCOL_FEATURE_MOTOR_CONFIGURATION | PROTOCOL_FEATURE_HOST_POSITION_SESSION |
                    PROTOCOL_FEATURE_MOTION_LEASE | PROTOCOL_FEATURE_PARAMETER_SERVICE |
                    PROTOCOL_FEATURE_JOB_STATUS | PROTOCOL_FEATURE_OUTPUT_FEEDBACK | (1UL << 8U) |
                    (1UL << 10U) | (1UL << 11U) | (1UL << 12U),
        .supported_modes = (1U << 1) | (1U << 2) | (1U << 3) | (1U << 6) | (1U << 9) | (1U << 10),
        .max_full_frame = YG_PROTOCOL_MAX_FRAME_SIZE,
        .max_block_data = YG_PROTOCOL_MAX_PAYLOAD_SIZE};
    if (app->endpoint.hardware_check.handle != NULL)
    {
        capabilities.features |= PROTOCOL_FEATURE_HARDWARE_CHECK;
    }
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
    update_identity_info_page(app);
    if (BspSystem_ReadUid(info.uid) != BSP_OK)
    {
        return false;
    }
    if (!Protocol_SetInfo(&app->endpoint, &info, &capabilities))
    {
        return false;
    }
    return Protocol_SetInfoPage(&app->endpoint, 2U, target_payload);
}
