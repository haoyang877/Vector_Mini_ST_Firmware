/* 由run-state夹具包含，复用其真实owner和硬件替身；不执行旧温度必需用例。 */
int main(void)
{
    reset();
    temperature_fresh = false;
    foc.temperature_c = NAN;
    MotorStateMachine_Init(&cfg, &motor, &foc, &pi, &encoder, &startup);
    for (unsigned i = 0U; i < 4U; ++i)
    {
        tick();
    }
    assert(MotorStateMachine_GetState() == MOTOR_STATE_IDLE);
    assert(motor.fault_code == MOTOR_FAULT_NONE && output_off && start_count == 0U);
    MotorReadiness readiness;
    MotorStateMachine_ReadReadiness(&readiness);
    assert(readiness.self_test_checks ==
           (MOTOR_SELF_TEST_ALL & ~MOTOR_SELF_TEST_TEMPERATURE_VALID));
    assert(readiness.start_blockers == 0U);
    assert_boot_health(true);

    foc.bus_voltage_v = NAN;
    assert_boot_health(false);
    foc.bus_voltage_v = 29.0f;
    foc.phase_a_current_a = NAN;
    assert_boot_health(false);
    foc.phase_a_current_a = 0.0f;
    assert_boot_health(true);
    cfg.configuration_confirmed = false;
    MotorStateMachine_ReadReadiness(&readiness);
    assert(readiness.start_blockers & MOTOR_START_CONFIGURATION_REQUIRED);
    assert(readiness.startable_modes == 0U);
    cfg.configuration_confirmed = true;

    MotorFault_Report(&motor, MOTOR_FAULT_TEMPERATURE_SENSOR);
    tick();
    assert(MotorStateMachine_GetState() == MOTOR_STATE_FAULT && output_off);
    assert(MotorStateMachine_RequestClear() == MOTOR_CONTROL_ACCEPTED);
    assert(MotorStateMachine_GetState() == MOTOR_STATE_IDLE &&
           motor.fault_code == MOTOR_FAULT_NONE);
    MotorFault_Report(&motor, MOTOR_FAULT_OVERCURRENT);
    tick();
    assert(MotorStateMachine_GetState() == MOTOR_STATE_FAULT && output_off);
    assert(MotorStateMachine_RequestClear() == MOTOR_CONTROL_DENIED);
    assert(motor.fault_code == MOTOR_FAULT_OVERCURRENT && start_count == 0U);
    assert(critical_depth == 0U);
    puts("PASS R40 optional temperature: real INIT/IDLE, truthful checks, explicit CLEAR, other "
         "guards");
    return 0;
}
