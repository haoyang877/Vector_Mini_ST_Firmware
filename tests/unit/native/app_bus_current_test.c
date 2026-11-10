/* 验证真实APP状态适配器与固定协议布局；无测量源或硬件访问。 */
#include <assert.h>
#include <limits.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "motor_state_machine.h"
#include "motor_axis.h"
#include "../../../firmware/communication/yg_protocol.c"
#include "../../../firmware/communication/yg_protocol_boot.c"
#include "app_protocol.h"

static ProtocolEndpoint endpoint;
static AppProtocol adapter;
static MotorStateMachine machine = {.epoch = 1U};
static MotorStatus s_status;
static unsigned s_status_reads;

bool MotorStateMachine_ReadStatus(const MotorStateMachine *instance, MotorStatus *status)
{
    assert(instance == &machine);
    ++s_status_reads;
    *status = s_status;
    return true;
}

bool MotorStateMachine_ArePhasesDisabled(const MotorStateMachine *machine)
{
    (void)machine;
    return true;
}
void MotorStateMachine_RequestStop(MotorStateMachine *instance)
{
    assert(instance == &machine);
}
MotorControlResult MotorStateMachine_RequestStart(MotorStateMachine *instance,
                                                  MotorControlMode mode)
{
    (void)instance;
    (void)mode;
    return MOTOR_CONTROL_DENIED;
}
MotorControlResult MotorStateMachine_RequestCalibration(MotorStateMachine *instance,
                                                        MotorCalibrationType type)
{
    (void)instance;
    (void)type;
    return MOTOR_CONTROL_DENIED;
}
MotorControlResult MotorStateMachine_RequestHostPosition(MotorStateMachine *instance)
{
    (void)instance;
    return MOTOR_CONTROL_DENIED;
}

void MotorStateMachine_KeepHostPositionAlive(MotorStateMachine *instance)
{
    (void)instance;
}

MotorControlResult MotorStateMachine_RequestFaultClear(MotorStateMachine *instance)
{
    (void)instance;
    return MOTOR_CONTROL_DENIED;
}

static ProtocolFeedback read_feedback(void)
{
    ProtocolFeedback feedback = s_empty_feedback;
    assert(endpoint.motor.read_feedback(endpoint.motor.context, &feedback));
    return feedback;
}

/* 用原协议舍入定义核对两个位宽和有效位，不复用生产整数编码器。 */
static void check_scaled_value(float value)
{
    s_status =
        (MotorStatus){.measurements_valid = true, .position_rad = value, .bus_voltage_v = value};
    ProtocolFeedback feedback = read_feedback();
    double position = round((double)value * 1000);
    double voltage = round((double)value * 100);
    bool position_valid = isfinite(position) && position > INT32_MIN && position <= INT32_MAX;
    bool voltage_valid = isfinite(voltage) && voltage > INT16_MIN && voltage <= INT16_MAX;
    assert(feedback.actual_position_mrad == (position_valid ? (int32_t)position : INT32_MIN));
    assert(feedback.bus_voltage_cV == (voltage_valid ? (int16_t)voltage : INT16_MIN));
    assert(((feedback.valid_bits & (1U << 1)) != 0U) == position_valid);
    assert(((feedback.valid_bits & (1U << 4)) != 0U) == voltage_valid);
}

static void check_scaling_boundaries(void)
{
    const float samples[] = {0.0f,
                             -0.0f,
                             0.0045f,
                             -0.0045f,
                             0.0625f,
                             -0.0625f,
                             327.675f,
                             -327.675f,
                             2147483.5f,
                             -2147483.5f,
                             INFINITY,
                             -INFINITY,
                             NAN};
    for (unsigned i = 0; i < sizeof(samples) / sizeof(samples[0]); ++i)
    {
        check_scaled_value(samples[i]);
        check_scaled_value(nextafterf(samples[i], INFINITY));
        check_scaled_value(nextafterf(samples[i], -INFINITY));
    }
    uint32_t bits = 20261107U;
    for (unsigned i = 0; i < 100000U; ++i)
    {
        bits = bits * 1664525U + 1013904223U;
        float value;
        memcpy(&value, &bits, sizeof(value));
        check_scaled_value(value);
    }
}

/* 同一轮查询、写目标、再查询；发送忙时读取已排队的真实反馈，不重复采样。 */
static void check_feedback_dispatch(void)
{
    uint8_t payload[PROTOCOL_CONTROL_SIZE] = {1U};
    YgProtocol_WriteU32Le(payload + 12U, 250U);
    yg_protocol_message_t query = {.version = YG_PROTOCOL_VERSION,
                                   .flags = YG_PROTOCOL_FLAGS_ACK_REQUEST,
                                   .source_node = PROTOCOL_MASTER_ID,
                                   .destination_node = 1U,
                                   .message_type = PROTOCOL_GET_MOTOR_STATE};
    yg_protocol_message_t target = query;
    target.message_type = PROTOCOL_CONTROL;
    target.payload = payload;
    target.payload_length = sizeof(payload);
    yg_protocol_message_t requests[] = {query, target, query, target};
    requests[3].payload_length = 0U;

    s_status = (MotorStatus){.state = MOTOR_STATE_RUNNING,
                             .mode = MOTOR_CONTROL_CURRENT,
                             .measurements_valid = true,
                             .iq_reference_a = -0.5f};
    unsigned before = s_status_reads;
    Protocol_Process(&endpoint, 1U);
    assert(s_status_reads == before);
    BspCanFrame frame;
    for (unsigned i = 0; i < 4U; ++i)
    {
        requests[i].sequence = (uint16_t)(40U + i);
        assert(encode_can_frame(&requests[i], 3U, &frame) == YG_PROTOCOL_OK);
        assert(Protocol_QueueReceivedFrame(&endpoint, &frame));
    }
    Protocol_Process(&endpoint, 2U);
    Protocol_Process(&endpoint, 3U);
    assert(s_status_reads == before + 4U);
    const uint16_t types[] = {PROTOCOL_GET_MOTOR_STATE,
                              PROTOCOL_CONTROL,
                              PROTOCOL_MOTION_FEEDBACK,
                              PROTOCOL_GET_MOTOR_STATE,
                              PROTOCOL_CONTROL,
                              PROTOCOL_MOTION_FEEDBACK};
    for (unsigned i = 0; i < 6U; ++i)
    {
        assert(queue_pop(&endpoint.tx_queue, &frame));
        yg_protocol_message_t response;
        assert(decode_can_frame(&frame, PROTOCOL_MASTER_ID, &response) == YG_PROTOCOL_OK);
        assert(response.message_type == types[i]);
        assert(YgProtocol_ReadU16Le(response.payload) ==
               (i >= 4U ? PROTOCOL_RESULT_BAD_LENGTH : PROTOCOL_RESULT_OK));
        if (response.message_type != PROTOCOL_CONTROL)
        {
            assert(response.payload_length == PROTOCOL_FEEDBACK_SIZE);
            assert((int16_t)YgProtocol_ReadU16Le(response.payload + 30U) == (i == 0U ? -500 : 250));
            assert(YgProtocol_ReadU16Le(response.payload + 2U) == (i == 2U   ? 41U
                                                                   : i == 5U ? 43U
                                                                             : 0U));
        }
    }
    assert(!queue_pop(&endpoint.tx_queue, &frame));
}

uint32_t BspSystem_TimeMs(void)
{
    return 0U;
}

int main(void)
{
    AppProtocol_Init(&adapter, &endpoint, &machine);
    assert(Protocol_Init(&endpoint, 1U));
    assert(Protocol_Start(&endpoint));
    check_feedback_dispatch();
    check_scaling_boundaries();
    const uint8_t states[] = {5U, 3U, 4U, 6U, 8U};
    for (unsigned i = 0; i <= MOTOR_STATE_FAULT; ++i)
    {
        s_status = (MotorStatus){
            .state = (MotorState)i, .mode = MOTOR_CONTROL_SPEED, .fault = MOTOR_FAULT_NONE};
        ProtocolFeedback feedback = read_feedback();
        assert(feedback.motor_state == states[i] && feedback.control_mode == 2U);
        assert(feedback.valid_bits == 0U && feedback.bus_current_mA == INT16_MIN &&
               feedback.reference_position_mrad == INT32_MIN);
    }
    s_status = (MotorStatus){.state = MOTOR_STATE_RUNNING,
                             .mode = MOTOR_CONTROL_CURRENT,
                             .measurements_valid = true,
                             .position_reference_rad = NAN,
                             .position_rad = -1.25f,
                             .speed_reference_rad_s = NAN,
                             .speed_rad_s = 2.5f,
                             .bus_voltage_v = 30.5f,
                             .bus_current_a = 0.125f,
                             .iq_reference_a = -0.25f,
                             .iq_a = -0.249f,
                             .temperature_c = 25.5f,
                             .vq_v = -1.5f,
                             .vd_v = 0.0625f};
    ProtocolFeedback live = read_feedback();
    assert(live.valid_bits == 0x0DFAU && live.actual_position_mrad == -1250 &&
           live.actual_speed_mrad_s == 2500 && live.bus_voltage_cV == 3050 &&
           live.bus_current_mA == 125 && live.reference_iq_mA == -250 &&
           live.actual_iq_mA == -249 && live.mcu_temperature_centi_c == 2550 &&
           live.motor_temperature_centi_c == INT16_MIN && live.v_q_mV == -1500 &&
           live.v_d_mV == 63);
    ProtocolResponse response = {0};
    yg_protocol_message_t request = {.message_type = PROTOCOL_CONTROL, .sequence = 123U};
    assert(encode_feedback_reply(&endpoint, &request, PROTOCOL_RESULT_OK, &response) ==
           YG_PROTOCOL_OK);
    assert(YgProtocol_ReadU16Le(response.data + 2U) == 123U);
    assert(response.length == 45U);
    printf("VECTOR live_current_feedback ");
    for (unsigned i = 0; i < response.length; ++i)
    {
        printf("%02x", response.data[i]);
    }
    puts("");
    s_status.bus_current_a = INFINITY;
    s_status.position_rad = 3000000.0f;
    s_status.temperature_c = -327.68f;
    live = read_feedback();
    assert((live.valid_bits & ((1U << 1) | (1U << 5) | (1U << 8))) == 0U &&
           live.actual_position_mrad == INT32_MIN && live.bus_current_mA == INT16_MIN &&
           live.mcu_temperature_centi_c == INT16_MIN);
    s_status = (MotorStatus){.state = MOTOR_STATE_RUNNING,
                             .mode = MOTOR_CONTROL_CURRENT,
                             .fault = MOTOR_FAULT_OVERCURRENT};
    live = read_feedback();
    assert(live.motor_state == 8U && live.fault_code == 1U && live.control_mode == 1U);
    puts("PASS APP state/fault mapping and invalid measurement feedback");
    return 0;
}

MotorControlResult MotorStateMachine_SetTarget(MotorStateMachine *instance,
                                               const MotorTarget *target)
{
    assert(instance == &machine && target->mode == MOTOR_CONTROL_CURRENT);
    s_status.iq_reference_a = target->iq_a;
    return MOTOR_CONTROL_ACCEPTED;
}
