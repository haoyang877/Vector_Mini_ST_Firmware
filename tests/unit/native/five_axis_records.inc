
/* 检查有效记录经过 APP 写入、重新装载和 Loader 的分块 CRC 后得到相同的档位。 */
static void check_bitrate(const InterfaceParam *original)
{
    const unsigned data_kbps[] = {500, 1000, 2000, 3000, 4000, 5000};
    InterfaceParam record;
    MotorConfiguration cfg;
    ParameterValue read;
    ParameterWriteRequest write = {.parameter_id = 0x71, .value_type = PARAMETER_VALUE_U32};
    Param_Download(original, sizeof(*original), &cfg);
    assert(cfg.device.can_bitrate_profile == 11U);
    assert(ParameterValue_Read(&cfg, 1, 0x71, &read) == PARAMETER_OK);
    assert(read.raw_value == ((1000UL << 16U) | 5000U));
    for (unsigned nominal = 500; nominal <= 1000; nominal += 500)
    {
        for (unsigned i = 0; i < 6; ++i)
        {
            write.raw_value = (nominal << 16) | data_kbps[i];
            assert(ParameterValue_Apply(&cfg, &write) == PARAMETER_OK);
            assert(ParameterValue_MatchesRequest(&cfg, &write));
            assert(ParameterValue_Read(&cfg, 2, 0x71, &read) == PARAMETER_OK);
            assert(read.raw_value == write.raw_value && read.access == 3 && read.value_type == 2);
            uint8_t profile = cfg.device.can_bitrate_profile;
            Param_Upload(&record, &cfg);
            assert(record.can_bitrate_record == (profile == 11U ? 0U : 0x43420100U | profile));
            memcpy(parameter_flash, &record, sizeof(record));
            assert(LoaderParameters_ReadBitrate(YI_PARAMS_RAW11) == profile);
            assert(LoaderParameters_ReadBitrate(YI_PARAMS_BLANK) == 11U);
            assert(LoaderParameters_ReadBitrate(YI_PARAMS_UNSAFE) == 11U);
            Param_Download(&record, sizeof(record), &cfg);
            assert(cfg.device.can_bitrate_profile == profile);
            assert(cfg.device.node_id == original->node_id);
        }
    }
    write.raw_value = (250U << 16) | 2000U;
    assert(ParameterValue_Apply(&cfg, &write) == PARAMETER_OUT_OF_RANGE);
    write.raw_value = (500U << 16) | 2500U;
    assert(ParameterValue_Apply(&cfg, &write) == PARAMETER_OUT_OF_RANGE);
    write.value_type = PARAMETER_VALUE_I32;
    assert(ParameterValue_Apply(&cfg, &write) == PARAMETER_BAD_TYPE);
    assert(cfg.device.can_bitrate_profile == 11);
    /* v0.7.0 曾显式保存所有非零档位，包括现在的默认档 11。 */
    for (unsigned id = 0; id < 12; ++id)
    {
        record.can_bitrate_record = 0x43420100U | id;
        record.record_crc32 = Param_RecordCrc(&record);
        memcpy(parameter_flash, &record, sizeof(record));
        Param_Download(&record, sizeof(record), &cfg);
        assert(cfg.device.can_bitrate_profile == id);
        assert(LoaderParameters_ReadBitrate(YI_PARAMS_RAW11) == id);
    }
    record.can_bitrate_record = 0x43420100U; /* 显式 500K/2M 也必须通过基础 CRC。 */
    record.record_crc32 = Param_RecordCrc(&record) ^ 1U;
    memcpy(parameter_flash, &record, sizeof(record));
    assert(LoaderParameters_ReadBitrate(YI_PARAMS_RAW11) == 11U);
    Param_Download(&record, sizeof(record), &cfg);
    assert(cfg.device.can_bitrate_profile == 11U);
    const uint32_t legacy[] = {0U, 0xFFFFFFFFU, 0x4342010CU, 0x43420201U};
    for (unsigned i = 0; i < sizeof(legacy)/sizeof(legacy[0]); ++i)
    {
        record.can_bitrate_record = legacy[i];
        record.record_crc32 = Param_RecordCrc(&record);
        memcpy(parameter_flash, &record, sizeof(record));
        assert(LoaderParameters_ReadBitrate(YI_PARAMS_RAW11) == 11U);
        Param_Download(&record, sizeof(record), &cfg);
        assert(cfg.device.can_bitrate_profile == 11U && cfg.device.node_id == original->node_id);
    }
}

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
        bool unrestricted = wheel;
#if MOTOR_DEBUG_JOINT_NO_TRAVEL_LIMITS
        unrestricted = true;
#elif MOTOR_DEBUG_PITCH_NO_TRAVEL_LIMITS
        unrestricted = wheel || node == 6;
#endif
        assert(Param_CreateAxis(node, wheel ? 0 : -1, wheel ? 0 : 1, .5f, &heat, &configuration));
        Param_Upload(&record, &configuration);
        assert(record.record_crc32 == five_axis_crc[node - 3]);
        check_bitrate(&record);
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
        assert(strcmp(p->model_name, ht ? "HT8115-4" : node == 6 ? "YQ-5740" : "YQ-5708") == 0);
        assert(p->joint == roles[node-3] && configuration.model.motor_pole_pairs == (ht ? 21 : 14));
        assert(configuration.model.motor_phase_resistance_ohm == (ht ? 1.905f : .55f));
        assert(configuration.model.calibration_current_a == (ht ? 3.f : 1.6f));
        assert(configuration.application.current_limit_a == 7);
        assert(configuration.model.thermal.continuous_vector_a == 3);
        if (node == 5)
        {
            assert(fabsf(p->speed_ceiling_rad_s - 3.14159265f) < 1e-6f);
            assert(p->control.acceleration_rad_s2 == p->speed_ceiling_rad_s);
            assert(p->control.deceleration_rad_s2 == p->speed_ceiling_rad_s);
        }
        assert(configuration.application.speed_acceleration_rad_s2 == (wheel ? 20.0f : p->control.acceleration_rad_s2));
        assert(configuration.application.speed_deceleration_rad_s2 == (wheel ? 20.0f : p->control.deceleration_rad_s2));
        assert(configuration.application.position_acceleration_rad_s2 == configuration.application.speed_acceleration_rad_s2);
        assert(configuration.application.position_deceleration_rad_s2 == configuration.application.speed_deceleration_rad_s2);
        if (node >= 6)
        {
            const float ratio = p->gear_ratio;
            const float output_limit = 2.094395102f;
            assert(fabsf(p->speed_ceiling_rad_s / ratio - output_limit) < 1e-6f);
            assert(fabsf(p->control.acceleration_rad_s2 / ratio - output_limit) < 1e-6f);
            assert(fabsf(p->control.deceleration_rad_s2 / ratio - output_limit) < 1e-6f);
            MotorConfiguration full;
            assert(Param_CreateAxis(node, -1, 1, output_limit * ratio, &heat, &full));
            assert(!Param_CreateAxis(node, -1, 1, (output_limit + 0.001f) * ratio, &heat, &full));
        }
        assert(configuration.application.speed_kp == (wheel ? .05f : ht ? .5f : .048f));
        assert(configuration.application.speed_ki == (node == 5 ? 1.f : .5f));
        assert(fabsf(configuration.model.iq_kp - (ht ? 12.32761f : .60995855f)) < .0001f);
        assert(fabsf(configuration.model.iq_ki - (ht ? 14363.36f : 3226.22067f)) < .01f);
        assert(p->startup->target_electrical_velocity_rad_s == (ht ? 315 : node == 6 ? 336 : 420));
        assert(p->electrical->speed_pll_hz == 600);
        assert(p->gear_ratio == (ht ? 1 : 40));
        assert(p->electrical->observer_gamma == 800000);
        assert(configuration.device.encoder.linearization_lut_q15[71] == -42);
        assert(p->damper_torque_nm == (wheel ? 0.0f : 2.0f));
        assert(MotorAxisPreset_HasLinearHall(p) == (node == 6 || node == 7));
        assert(MotorAxisProfile_AllowsPosition(&record.axis_profile, true, 0, 0));
        assert(MotorAxisProfile_AllowsPosition(&record.axis_profile, true, -8.0f, 14.0f) == unrestricted);
        assert(MotorAxisProfile_TargetWithinBounds(&record.axis_profile, -16.0f) == unrestricted);
        assert(!MotorAxisProfile_AllowsPosition(&record.axis_profile, false, 0, 0));
        assert(!MotorAxisProfile_AllowsPosition(&record.axis_profile, true, NAN, 0));
        assert(!MotorAxisProfile_TargetWithinBounds(&record.axis_profile, INFINITY));
        if (wheel)
        {
            InterfaceParam old = record;
            old.cascade_pos_kp = old.cascade_pos_kd = 0;
            old.current_limit_a = 3;
            old.speed_acceleration_rad_s2 = old.speed_deceleration_rad_s2 = 1.0f;
            old.record_crc32 = Param_RecordCrc(&old);
            MotorConfiguration replay;
            Param_Download(&old, sizeof(old), &replay);
            assert(replay.application.cascade_position_kp == 2.0f);
            assert(replay.application.current_limit_a == 3);
            assert(replay.application.speed_acceleration_rad_s2 == 1.0f);
        }
        if (!wheel)
        {
            const float degrees = node == 5 ? 150.0f : node == 6 ? 30.0f : 180.0f;
            assert(fabsf(p->output_travel_limit_rad - degrees * 0.01745329252f) < 1e-6f);
            const float limit = p->output_travel_limit_rad * p->gear_ratio;
            MotorConfiguration full;
            assert(Param_CreateAxis(node, -limit, limit, .5f, &heat, &full));
            const MotorAxisProfile *axis = &full.application.axis_profile;
            assert(MotorAxisProfile_AllowsPosition(axis, true, 0, limit));
            assert(MotorAxisProfile_AllowsPosition(axis, true, 0, -limit));
            assert(MotorAxisProfile_AllowsPosition(axis, true, limit, 0) == unrestricted);
            assert(MotorAxisProfile_AllowsPosition(axis, true, -limit - 1.0f, 0) == unrestricted);
            assert(MotorAxisProfile_TargetWithinBounds(axis, limit + 0.001f) == unrestricted);
            assert(MotorAxisProfile_AllowsPosition(axis, true, 0, -limit - 1.0f) == unrestricted);
            InterfaceParam old = record;
            old.axis_profile.minimum_position_rad = -21.949001f;
            old.axis_profile.maximum_position_rad = node == 6 ? 19.939001f : 1.0f;
            old.axis_profile.crc32 = Crc32_Compute(&old.axis_profile, offsetof(MotorAxisProfile, crc32), 0);
            old.record_crc32 = Param_RecordCrc(&old);
            Param_Download(&old, sizeof(old), &full);
            assert(full.configuration_confirmed && full.application.axis_profile_valid);
            assert(axis->minimum_position_rad == fmaxf(old.axis_profile.minimum_position_rad, -limit));
            assert(axis->maximum_position_rad == fminf(old.axis_profile.maximum_position_rad, limit));
            assert(axis->crc32 == Crc32_Compute(axis, offsetof(MotorAxisProfile, crc32), 0));
            assert(full.device.encoder.calib_flag == ENC_CALIB_ALL && full.device.encoder.linearization_lut_q15[71] == -42);
            old.axis_profile.crc32 ^= 1;
            old.record_crc32 = Param_RecordCrc(&old);
            Param_Download(&old, sizeof(old), &full);
            assert(full.preset == NULL);
            old.axis_profile.minimum_position_rad = limit;
            old.axis_profile.maximum_position_rad = limit + 1.0f;
            old.axis_profile.crc32 = Crc32_Compute(&old.axis_profile, offsetof(MotorAxisProfile, crc32), 0);
            old.record_crc32 = Param_RecordCrc(&old);
            Param_Download(&old, sizeof(old), &full);
            assert(full.preset == NULL);
            assert(!Param_CreateAxis(node, -limit, limit + 0.001f, .5f, &heat, &full));
        }
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
