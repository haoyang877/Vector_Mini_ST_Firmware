#include <math.h>
#include <stdio.h>
#include "virtual_platform.h"
#include "motor_axis.h"
#include "motor_state_machine.h"
#include "control_config.h"

/* 跟踪发送必须消费后才成功；超时帧不得迟到发送。 */
static bool check_tx_contract(void)
{
    CommHwCanFrame frame = {
        .identifier = 1U, .length = 12U, .extended = true, .fd = true, .bitrate_switch = true};
    CommHwCanFrame output;
    CommHwCanTxCompletion completion;
    const uint64_t ticket = UINT64_C(0x100000001);
    /* 先消费真实APP启动产生的报文，不把它们误认为本次跟踪帧。 */
    for (unsigned count = 0U; count < 32U && VirtualPlatform_TakeTx(&output); ++count)
    {
    }
    if (!comm_hw_can_try_send_tracked_frame(&frame, ticket) ||
        comm_hw_can_take_tx_completion(&completion) || comm_hw_can_try_send_frame(&frame) ||
        !VirtualPlatform_TakeTx(&output) || !comm_hw_can_take_tx_completion(&completion) ||
        completion.ticket != ticket || !completion.success ||
        comm_hw_can_take_tx_completion(&completion))
    {
        return false;
    }
    if (!comm_hw_can_try_send_tracked_frame(&frame, ticket + 1U) ||
        !VirtualPlatform_Advance(FOC_FREQ / 10U) || VirtualPlatform_TakeTx(&output) ||
        comm_hw_can_take_tx_completion(&completion) || !comm_hw_can_try_send_frame(&frame))
    {
        return false;
    }
    /* 超时终态已由真实协议前台消费，且发送槽已释放。 */
    if (!VirtualPlatform_TakeTx(&output))
    {
        return false;
    }
    frame.length = 9U;
    return !comm_hw_can_try_send_frame(&frame);
}

/* 仅验证真实APP在无硬件、无动力模型下完成启动校正并保持待机。 */
int main(void)
{
    const VirtualConfig config = {.bus_voltage_v = 33.0f,
                                  .temperature_c = 25.0f,
                                  .phase_offset_counts = {2040U, 2048U, 2056U},
                                  .encoder_angle_q15 = 1000U};
    if (!VirtualPlatform_Init(&config) || !VirtualPlatform_Advance(FOC_FREQ / 2U) ||
        MotorStateMachine_GetState() != MOTOR_STATE_INIT || !VirtualPlatform_Advance(FOC_FREQ))
    {
        fputs("virtual platform initialization/scheduling failed\n", stderr);
        return 1;
    }
    if (VirtualPlatform_Init(&config) || !check_tx_contract())
    {
        fputs("virtual platform contract failed\n", stderr);
        return 1;
    }
    MotorStatus status;
    uint8_t valid_bits;
    MotorMaintenanceKind maintenance;
    MotorControlProgress progress;
    MotorState state = MotorAxis_ReadStatus(&status, &valid_bits, &maintenance, &progress);
    VirtualSnapshot snapshot = VirtualPlatform_Read();
    MotorReadiness readiness;
    MotorStateMachine_ReadReadiness(&readiness);
    bool success = state == MOTOR_STATE_IDLE && status.fault == 0U &&
                   status.mode == MOTOR_CONTROL_NONE && snapshot.phases_disabled &&
                   !snapshot.power_was_enabled && snapshot.fast_ticks == FOC_FREQ * 8U / 5U &&
                   snapshot.supervisor_ticks == SUPERVISOR_FREQ * 8U / 5U &&
                   readiness.self_test_checks == MOTOR_SELF_TEST_ALL &&
                   (readiness.start_blockers & MOTOR_START_CONFIGURATION_REQUIRED) != 0U &&
                   MotorStateMachine_RequestEnable(MOTOR_CONTROL_SPEED) == MOTOR_CONTROL_DENIED &&
                   fabsf(status.bus_voltage - config.bus_voltage_v) < 0.1f &&
                   status.current_reference == 0.0f;
    printf("{\"success\":%s,\"state\":%u,\"fault\":%u,\"mode\":%u,"
           "\"fast_ticks\":%u,\"supervisor_ticks\":%u,\"time_us\":%llu,"
           "\"bus_voltage_v\":%.6f,\"phases_disabled\":%s,\"power_was_enabled\":%s}\n",
           success ? "true" : "false",
           (unsigned)state,
           (unsigned)status.fault,
           (unsigned)status.mode,
           snapshot.fast_ticks,
           snapshot.supervisor_ticks,
           (unsigned long long)snapshot.time_us,
           (double)status.bus_voltage,
           snapshot.phases_disabled ? "true" : "false",
           snapshot.power_was_enabled ? "true" : "false");
    return success ? 0 : 1;
}
