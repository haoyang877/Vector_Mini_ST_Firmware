#include "yg_protocol_status_adapter.h"
#include "bsp_config.h"
#if BSP_SUPPORTED_BUS_CURRENT
#include "bus_current_feedback.h"
#endif

#include <math.h>

#include "angle_feedback.h"
#include "bsp_system.h"
#include "motor_types.h"
#include "foc_algorithm.h"
#include "foc_sensing.h"
#include "motor_state_machine.h"
#include "motor_axis.h"
#include "yg_protocol_link.h"
#include "yg_protocol_motor.h"

/* 状态适配器只组装 SI 快照；线路缩放、哨兵和小端编码由 protocol 层统一完成。 */
static yg_protocol_motor_status_source_t s_source;
static yg_protocol_readonly_device_status_t s_boot_source;
static bool s_initialized;

static void refresh_boot_source(void)
{
    /* 启动记录已移除；APP 恒报无持久启动身份，不伪造 epoch 或试启动阶段。 */
    s_boot_source = (yg_protocol_readonly_device_status_t){.role = YG_PROTOCOL_DEVICE_ROLE_APP};
}

static uint32_t fault_bits(MotorFaultCode error)
{
    switch (error)
    {
    case MOTOR_FAULT_OVERCURRENT:
        return 1U << 0U;
    case MOTOR_FAULT_OVERVOLTAGE:
        return 1U << 1U;
    case MOTOR_FAULT_UNDERVOLTAGE:
        return 1U << 2U;
    case MOTOR_FAULT_OVERTEMPERATURE:
        /* 现有温度来自 MCU，而非电机绕组。 */
        return 1U << 4U;
    case MOTOR_FAULT_TEMPERATURE_SENSOR:
        return 1U << 12U;
    case MOTOR_FAULT_ENCODER:
        return 1U << 5U;
    case MOTOR_FAULT_CAN_DISCONNECTED:
        return 1U << 6U;
    case MOTOR_FAULT_INVALID_PARAMETERS:
    case MOTOR_FAULT_POLE_PAIRS:
    case MOTOR_FAULT_ENCODER_NOT_CALIBRATED:
        return 1U << 9U;
    case MOTOR_FAULT_CURRENT_OFFSET:
    case MOTOR_FAULT_COGGING_CALIBRATION:
        return 1U << 14U;
    case MOTOR_FAULT_PHASE_RESISTANCE:
    case MOTOR_FAULT_PHASE_INDUCTANCE:
    case MOTOR_FAULT_FRICTION_IDENTIFICATION:
        return 1U << 15U;
    case MOTOR_FAULT_NONE:
        return 0U;
    default:
        return 1U << 12U;
    }
}

/* 业务控制与维护类型只在协议边界映射为既有线上模式号。 */
static uint8_t protocol_mode(MotorControlMode mode, MotorMaintenanceKind kind)
{
    if (kind == MOTOR_MAINTENANCE_ENCODER_LUT || kind == MOTOR_MAINTENANCE_ELECTRICAL_ZERO ||
        kind == MOTOR_MAINTENANCE_COGGING)
    {
        return 7U;
    }
    if (kind == MOTOR_MAINTENANCE_PHASE_RESISTANCE || kind == MOTOR_MAINTENANCE_FRICTION)
    {
        return 8U;
    }
    switch (mode)
    {
    case MOTOR_CONTROL_CURRENT:
        return 1U;
    case MOTOR_CONTROL_SPEED:
        return 2U;
    case MOTOR_CONTROL_POSITION:
        return 3U;
    case MOTOR_CONTROL_VQ:
        return 6U;
    case MOTOR_CONTROL_OPEN_VOLTAGE:
        return 9U;
    case MOTOR_CONTROL_OPEN_CURRENT:
        return 10U;
    default:
        return 0U;
    }
}

static uint8_t protocol_state(MotorState state, uint8_t mode)
{
    switch (state)
    {
    case MOTOR_STATE_INIT:
        return 3U;
    case MOTOR_STATE_IDLE:
        return 5U;
    case MOTOR_STATE_STARTING:
        return 2U;
    case MOTOR_STATE_RUNNING:
        return 6U;
    case MOTOR_STATE_STOPPING:
        return 7U;
    case MOTOR_STATE_MAINTENANCE:
        return mode == 7U || mode == 8U ? 4U : 5U;
    case MOTOR_STATE_FAULT:
        return 8U;
    case MOTOR_STATE_FATAL:
    default:
        return 9U;
    }
}

bool YgProtocolStatusAdapter_Init(void)
{
    s_source = (yg_protocol_motor_status_source_t){0};
    refresh_boot_source();
    s_initialized = YgProtocolLink_BindMotorStatusSource(&s_source) &&
                    YgProtocolLink_BindDeviceStatusSource(&s_boot_source);
    return s_initialized;
}

void YgProtocolStatusAdapter_Refresh(void)
{
    if (!s_initialized)
    {
        return;
    }
    /* 前台独占 s_source；版本核对取得快慢环一致快照，CRC与发送不持有采集临界区。 */
    MotorMaintenanceKind kind;
    MotorState state = MotorAxis_ReadStatus(
        &s_source.sample, &s_source.measurement_valid_bits, &kind, &s_source.control_progress);
#if BSP_SUPPORTED_BUS_CURRENT
    /* 有硬件时使用实测；失效保留NAN及无效位，无硬件时保留原估算。 */
    bool bus_current_valid = BusCurrentFeedback_Read(&s_source.sample.bus_current);
    s_source.measurement_valid_bits =
        (s_source.measurement_valid_bits & (uint8_t)~0x40U) | (bus_current_valid ? 0x40U : 0U);
#endif
    s_source.faults = fault_bits((MotorFaultCode)s_source.sample.fault);
    s_source.mode = protocol_mode((MotorControlMode)s_source.sample.mode, kind);
    s_source.state = state == MOTOR_STATE_FATAL ? 9U
                     : s_source.sample.fault != MOTOR_FAULT_NONE
                         ? 8U
                         : protocol_state(state, s_source.mode);
    s_source.sample_available = true;
    /* 与协议前台串行，完整64位来源只取本次已验证交接，不从主机或旧记录重建。 */
    refresh_boot_source();
}
