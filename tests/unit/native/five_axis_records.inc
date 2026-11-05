
/* 模拟电机轴范围和热参数，仅供离线测试。 */
int main(void)
{
    const MotorJointType roles[] = {MOTOR_JOINT_WHEEL_LEFT, MOTOR_JOINT_WHEEL_RIGHT,
                                    MOTOR_JOINT_ROLL, MOTOR_JOINT_PITCH, MOTOR_JOINT_YAW};
    MotorWindingThermalConfig heat = {3, 7, 66, .8f, .7f, 1, .5f};
    InterfaceParam record;
    Param_Return_Default(&configuration);
    assert(configuration.device.node_id == UINT8_MAX && !configuration.configuration_confirmed);
    assert(!Param_CreateAxis(6, -1, 1, .5f, NULL, &configuration));
    for (unsigned node = 3; node <= 7; ++node)
    {
        bool wheel = node < 5, ht = node <= 5;
        assert(Param_CreateAxis(node, wheel ? 0 : -1, wheel ? 0 : 1, .5f, &heat, &configuration));
        Param_Upload(&record, &configuration);
        assert(record.record_crc32 == five_axis_crc[node - 3]);
        configuration.configuration_confirmed = true;
        configuration.device.encoder.calib_flag = ENC_CALIB_ALL;
        configuration.device.encoder.linearization_lut_q15[71] = -42;
        Param_Upload(&record, &configuration);
        record.current_limit_a = 10;
        record.calibration_current_a = 10;
        record.record_crc32 = Param_RecordCrc(&record);
        Param_Download(&record, sizeof(record), &configuration);
        const MotorAxisPreset *p = configuration.preset;
        assert(configuration.configuration_confirmed && p && p->node_id == node);
        assert(p->joint == roles[node-3] && configuration.model.motor_pole_pairs == (ht ? 21 : 14));
        assert(configuration.model.motor_phase_resistance_ohm == (ht ? 1.905f : .55f));
        assert(configuration.model.calibration_current_a == (ht ? 3.f : 1.6f));
        assert(configuration.application.current_limit_a == 3);
        assert(configuration.application.speed_kp == (wheel ? .05f : ht ? .5f : .048f));
        assert(configuration.application.speed_ki == (node == 5 ? 1.f : .5f));
        assert(fabsf(configuration.model.iq_kp - (ht ? 12.32761f : .60995855f)) < .0001f);
        assert(fabsf(configuration.model.iq_ki - (ht ? 14363.36f : 3226.22067f)) < .01f);
        assert(p->startup->target_electrical_velocity_rad_s == (ht ? 315 : 420));
        assert(p->electrical->speed_pll_hz == (ht ? 0 : 600));
        assert(p->electrical->gear_ratio == (ht ? 1 : 8));
        assert(p->electrical->observer_gamma == 800000);
        assert(configuration.device.encoder.linearization_lut_q15[71] == -42);
        assert(MotorAxisProfile_AllowsPosition(&record.axis_profile, true, 0, 0) == !wheel);
        printf("node %u role %s iq PI %.4f/%.2f speed PI %.4f/%.2f run/cal %.1f/%.1f A\n",
               node, MotorJointType_Name(p->joint), configuration.model.iq_kp, configuration.model.iq_ki,
               configuration.application.speed_kp, configuration.application.speed_ki,
               configuration.application.current_limit_a, configuration.model.calibration_current_a);
        record.node_id = node == 7 ? 3 : node+1;
        record.record_crc32 = Param_RecordCrc(&record);
        Param_Download(&record, sizeof(record), &configuration);
        assert(!configuration.configuration_confirmed && !configuration.preset);
        assert(configuration.device.node_id == UINT8_MAX && !configuration.device.encoder.calib_flag);
        record.node_id = node;
        record.motor_pole_pairs += 1;
        record.record_crc32 = Param_RecordCrc(&record);
        Param_Download(&record, sizeof(record), &configuration);
        assert(!configuration.preset);
    }
    memset(&record.axis_profile, 0, sizeof(record.axis_profile));
    record.record_crc32 = Param_RecordCrc(&record);
    Param_Download(&record, sizeof(record), &configuration);
    assert(!configuration.preset);
    record.record_crc32 ^= 1;
    Param_Download(&record, sizeof(record), &configuration);
    assert(!configuration.preset && !configuration.configuration_confirmed);
    puts("PASS five-axis record matrix, limits, mismatches and reboot");
    return 0;
}
