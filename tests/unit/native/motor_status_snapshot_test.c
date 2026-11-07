/* 只检查现有状态元数据接口，不再构造测量快照。 */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "motor_lifecycle.h"
#include "bsp_system.h"
static MotorControlState motor;
static FocState foc;
static EncoderState encoder;
bool McuTemperature_IsFresh(const volatile McuTemperatureTelemetry *temperature)
{
    (void)temperature;
    return false;
}
static MotorLifecycle life = {.drive = {.motor = &motor, .foc = &foc, .rotor_encoder = &encoder},
                              .state = MOTOR_STATE_IDLE,
                              .fast_samples_fresh = true};
static unsigned depth;
uint32_t BspSystem_EnterCritical(void)
{
    return depth++;
}
void BspSystem_ExitCritical(uint32_t saved)
{
    assert(depth == saved + 1U);
    depth = saved;
}
/* REAL_FUNCTIONS */
int main(void)
{
    for (unsigned i = 0U; i <= MOTOR_STATE_FAULT; ++i)
    {
        life.state = (MotorState)i;
        motor.control_mode = MOTOR_CONTROL_SPEED;
        motor.fault_code = MOTOR_FAULT_OVERCURRENT;
        MotorStatus status = MotorLifecycle_ReadStatus(&life);
        assert(status.state == life.state && status.mode == MOTOR_CONTROL_SPEED &&
               status.fault == MOTOR_FAULT_OVERCURRENT && depth == 0U);
    }
    life.state = MOTOR_STATE_RUNNING;
    motor.control_mode = MOTOR_CONTROL_POSITION;
    motor.position_target_rad = 1.5f;
    motor.position_reference_rad = 0.3f;
    assert(MotorLifecycle_ReadStatus(&life).position_reference_rad == 0.3f);
    assert(protocol_state(MOTOR_STATE_CALIBRATION) == 4U);
    assert(protocol_state(MOTOR_STATE_IDLE) == 5U);
    assert(protocol_state(MOTOR_STATE_RUNNING) == 6U);
    assert(protocol_state(MOTOR_STATE_FAULT) == 8U);
    puts("PASS minimal status metadata: state, mode, fault and invalid measurement channels");
    return 0;
}
