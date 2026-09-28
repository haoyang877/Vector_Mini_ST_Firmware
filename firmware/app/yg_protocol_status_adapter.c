#include "yg_protocol_status_adapter.h"

#include <math.h>

#include "angle_feedback.h"
#include "critical_hw.h"
#include "data_type.h"
#include "foc_algorithm.h"
#include "foc_sensing.h"
#include "foc_run_state.h"
#include "motor_state.h"
#include "yg_protocol_link.h"
#include "yg_protocol_motor_status.h"

/* 状态适配器只组装 SI 快照；线路缩放、哨兵和小端编码由 protocol 层统一完成。 */
static yg_protocol_motor_status_source_t source;
static bool initialized;

static uint32_t fault_bits(ErrorNow_TypeDef error)
{
    switch (error)
    {
    case Over_Current:
        return 1U << 0U;
    case Over_Voltage:
        return 1U << 1U;
    case Under_Voltage:
        return 1U << 2U;
    case High_Temprature:
        /* 现有温度来自 MCU，而非电机绕组。 */
        return 1U << 4U;
    case TemperatureSensor_Error:
        return 1U << 12U;
    case Encoder_Error:
        return 1U << 5U;
    case CAN_DisConnect:
        return 1U << 6U;
    case MotorParam_Error:
    case PolePairs_Error:
    case Encoder_NotCalibrated:
        return 1U << 9U;
    case CurrentOffset_Error:
    case CoggingCalibration_Error:
        return 1U << 14U;
    case Large_Phase_Resistance:
    case Large_Phase_Inductance:
    case FrictionIdentification_Error:
        return 1U << 15U;
    case No_Error:
        return 0U;
    default:
        return 1U << 12U;
    }
}

static uint8_t protocol_mode(ModeNow_TypeDef mode)
{
    switch (mode)
    {
    case Current_Mode:
        return 1U;
    case Speed_Mode:
    case Sensorless_Speed_Mode:
        return 2U;
    case Position_Mode:
        return 3U;
    case Position_Impedance_Mode:
        return 4U;
    case Vq_Mode:
        return 6U;
    case Calib_EncoderOffset:
    case Calib_CurrentOffset:
    case Calib_Anticogging:
    case Calib_EncoderObserver:
    case Calib_EleAngelOffset:
        return 7U;
    case Calib_Motor_R_L_Flux:
    case Calib_PhaseResistance:
    case Calib_Friction:
        return 8U;
    default:
        return 0U;
    }
}

static uint8_t protocol_state(AppLifecycleState state, uint8_t mode)
{
    switch (state)
    {
    case APP_BOOT:
        return 0U;
    case APP_SELF_TEST:
        return 3U;
    case APP_READY:
        return 5U;
    case APP_STARTING:
        return 2U;
    case APP_RUNNING:
        return 6U;
    case APP_STOPPING:
        return 7U;
    case APP_MAINTENANCE:
        return mode == 7U || mode == 8U ? 4U : 5U;
    case APP_FAULT:
        return 8U;
    case APP_FATAL:
    default:
        return 9U;
    }
}

bool YgProtocolStatusAdapter_Init(void)
{
    source = (yg_protocol_motor_status_source_t){0};
    initialized = YgProtocolLink_BindMotorStatusSource(&source);
    return initialized;
}

void YgProtocolStatusAdapter_Refresh(void)
{
    if (!initialized)
    {
        return;
    }
    /* 前台独占 source；采集短临界区避免快/慢环打断，CRC 与发送在恢复中断后执行。 */
    AppLifecycleState state =
        MotorControl_ReadStatus(&source.sample, &source.measurement_valid_bits);
    source.faults = fault_bits((ErrorNow_TypeDef)source.sample.fault);
    source.mode = protocol_mode((ModeNow_TypeDef)source.sample.mode);
    source.state = state == APP_FATAL                ? 9U
                   : source.sample.fault != No_Error ? 8U
                                                     : protocol_state(state, source.mode);
    source.sample_available = true;
}
