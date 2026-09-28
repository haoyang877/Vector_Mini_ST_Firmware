#include "board_config.h"

#include "board_hw.h"
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

/* 启动编排：参数装载 → 应用状态初始化 → 板级采样时钟 → CAN FD 端点。
 * 硬件细节全部位于 platform 契约实现内；本文件不再包含任何硬件头。 */

static motor_stop_service_t stop_service;
static yg_protocol_motor_adapter_t motor_adapter;

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

void Board_Init(void)
{
    /* 电机所有者装载参数与标定并初始化；无效记录回退默认值。 */
    MotorControl_Init();

    /* 新协议复位态为失能；CH4 采样时钟保持运行，三相输出等待控制所有者授权。 */
    board_hw_start(false);

    /* CAN FD 是唯一通信入口；硬件启动由运行时端点委托给平台层。 */
    if (!YgProtocolRuntime_Init(Protocol_NodeId_Get()) || !YgProtocolStatusAdapter_Init() ||
        !bind_motor_stop() || !YgProtocolOwner_BindServices() || !bind_device_description())
    {
        /* 初始化失败不能留下已启动但无通信入口的功率输出。 */
        power_stage_hw_stop();
    }
}
