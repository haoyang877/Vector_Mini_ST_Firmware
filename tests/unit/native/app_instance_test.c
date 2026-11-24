/* 生产实例不共享电机、协议或标定状态。设备回调在 A 尚未返回时进入 B 的同一入口。 */
#include "app.h"
#include "boot_hw.h"
#include "bsp_adc.h"
#include "bsp_encoder.h"
#include "bsp_pwm.h"
#include "bsp_system.h"
#include "motor_axis_profile.h"
#include "yg_protocol_frame_codec.h"
#include "crc32.h"
#include <assert.h>
#include <limits.h>
#include <stdio.h>
#include <string.h>

void BspSystem_Reset(void)
{
    assert(false && "instance test must not reset without an explicit configuration request");
}
static AppContext apps[2];
static uint32_t now_ms;
static bool load_saved, write_fails;
static unsigned depth;
static bool nested;
static bool calibrating;

typedef struct
{
    unsigned reads, duty_writes, record_writes;
    bool disabled;
    float duty[3];
    MotorAxisRecord saved;
} Device;
static Device devices[2];

static void publish(unsigned id)
{
    MotorCalibration_ProcessPublication(&apps[id].axis.calibration);
}
static BspResult initial_record(void *data)
{
#if BSP_BOARD == BSP_BOARD_AT32F456
    MotorConfiguration config = {0};
    assert(Param_CreateAxis(3, 0.0f, 0.0f, 2.0f, NULL, &config));
    config.device.phase_a_offset_counts = config.device.phase_b_offset_counts =
        config.device.phase_c_offset_counts = 2048.0f;
    Param_Upload(&((MotorAxisRecord *)data)->motor, &config);
    return BSP_OK;
#else
    (void)data;
    return BSP_NOT_READY;
#endif
}
static BspResult read_a(void *data, size_t length)
{
    assert(data == &apps[0].axis.calibration.storage.record && length == sizeof(MotorAxisRecord));
    ++devices[0].reads;
    if (nested)
    {
        App_Init(&apps[1]);
    }
    if (load_saved)
    {
        memcpy(data, &devices[0].saved, sizeof(MotorAxisRecord));
        return BSP_OK;
    }
    return initial_record(data);
}
static BspResult read_b(void *data, size_t length)
{
    assert(data == &apps[1].axis.calibration.storage.record && length == sizeof(MotorAxisRecord));
    ++devices[1].reads;
    return initial_record(data);
}
static void electrical_zero(unsigned id)
{
    MotorCalibration *cal = &apps[id].axis.calibration;
    MotorTaskResult result = EncoderCalibration_StepElecZero(&cal->session, &cal->drive);
    assert(result.result == MOTOR_TASK_RUNNING && result.error == MOTOR_FAULT_NONE);
}
static void record_duty(unsigned id, float a, float b, float c)
{
    assert(a >= 0.0f && a <= 1.0f && b >= 0.0f && b <= 1.0f && c >= 0.0f && c <= 1.0f);
    if (!calibrating)
    {
        assert(a == 0.5f && b == a && c == a);
    }
    devices[id].duty[0] = a;
    devices[id].duty[1] = b;
    devices[id].duty[2] = c;
    ++devices[id].duty_writes;
}
static void duty_a(float a, float b, float c)
{
    record_duty(0, a, b, c);
    if (nested)
    {
        if (calibrating)
        {
            electrical_zero(1);
        }
        else
        {
            MotorAxis_ControlFast(&apps[1].axis);
        }
    }
}
static void duty_b(float a, float b, float c)
{
    record_duty(1, a, b, c);
}
static BspResult store(unsigned id, const void *data, size_t length)
{
    assert(data == &apps[id].axis.calibration.storage.record && length == sizeof(MotorAxisRecord));
    assert(depth && devices[id].disabled);
    ++devices[id].record_writes;
    if (write_fails)
    {
        return BSP_IO_ERROR;
    }
    memcpy(&devices[id].saved, data, length);
    return BSP_OK;
}
static BspResult write_a(const void *data, size_t length)
{
    if (nested)
    {
        publish(1);
    }
    return store(0, data, length);
}
static BspResult write_b(const void *data, size_t length)
{
    return store(1, data, length);
}
static void off_a(void)
{
    devices[0].disabled = true;
}
static void off_b(void)
{
    devices[1].disabled = true;
}
static bool disabled_a(void)
{
    return devices[0].disabled;
}
static bool disabled_b(void)
{
    return devices[1].disabled;
}
static void unexpected_enable(void)
{
    assert(false);
}
static const MotorAxisHardware hardware[] = {
    {{unexpected_enable, off_a, disabled_a, duty_a, NULL}, read_a, write_a},
    {{unexpected_enable, off_b, disabled_b, duty_b, NULL}, read_b, write_b}};

/* 板级默认写入必须由本测试显式绑定的硬件接收。 */
void BspPwm_SetDuty(float a, float b, float c)
{
    (void)a;
    (void)b;
    (void)c;
    assert(false);
}
void BspPwm_Enable(void)
{
    assert(false);
}
void BspPwm_Disable(void)
{
    assert(false);
}
bool BspPwm_ArePhasesDisabled(void)
{
    assert(false);
    return false;
}
BspResult BspFlash_Read(void *data, size_t length)
{
    (void)data;
    (void)length;
    assert(false);
    return BSP_IO_ERROR;
}
BspResult BspFlash_Write(const void *data, size_t length)
{
    (void)data;
    (void)length;
    assert(false);
    return BSP_IO_ERROR;
}
uint32_t BspSystem_EnterCritical(void)
{
    return depth++;
}
void BspSystem_ExitCritical(uint32_t key)
{
    assert(depth == key + 1U);
    depth = key;
}
uint32_t BspSystem_EnterControlCritical(void)
{
    return BspSystem_EnterCritical();
}
void BspSystem_ExitControlCritical(uint32_t key)
{
    BspSystem_ExitCritical(key);
}
void BspSystem_MemoryBarrier(void)
{
}
bool BspAdc_ReadMotorTemperature(float *celsius)
{
    (void)celsius;
    return false;
}
uint32_t BspSystem_TimeMs(void)
{
    return now_ms;
}
BspResult BspSystem_ReadUid(uint8_t uid[BSP_SYSTEM_UID_BYTES])
{
    memset(uid, 0, BSP_SYSTEM_UID_BYTES);
    return BSP_OK;
}
BspResult BspSystem_ReadIdentity(uint8_t data[BSP_SYSTEM_IDENTITY_BYTES])
{
    (void)data;
    return BSP_NOT_READY;
}
uint32_t BootHw_ImageCrc32(void)
{
    return 0U;
}
void BspEncoder_Begin(void)
{
}
uint32_t BspEncoder_ReadAngle(BspEncoderResolution resolution)
{
    (void)resolution;
    return 0;
}
float BspAdc_ReadChannel(BspAdcChannel channel)
{
    (void)channel;
    return 0;
}
float BspAdc_ReadRawChannel(BspAdcChannel channel)
{
    (void)channel;
    return 2048;
}
void BspAdc_SetBusCurrentOffset(float value)
{
    (void)value;
}
void BspAdc_SetPhaseOffsets(const float offsets[3])
{
    (void)offsets;
}

static void run(bool interleave)
{
    nested = interleave;
    memset(apps, 0, sizeof(apps));
    memset(devices, 0, sizeof(devices));
    for (unsigned id = 0; id < 2; id++)
    {
        devices[id].disabled = true;
        apps[id].axis.hardware = &hardware[id];
    }
    App_Init(&apps[0]);
    if (!nested)
    {
        App_Init(&apps[1]);
    }
    for (unsigned id = 0; id < 2; id++)
    {
        AppContext *app = &apps[id];
        App_Init(app);
        assert(devices[id].reads == 1 && app->communication_ready && app->endpoint.ready);
        assert(app->axis.state_machine.drive.motor == &app->axis.motor &&
               app->protocol.state_machine == &app->axis.state_machine);
        assert(app->endpoint.rx_queue.storage == app->endpoint.rx_storage &&
               app->endpoint.motor.context == &app->protocol);
        app->axis.configuration.device.node_id = (uint8_t)(id + 4);
        app->axis.configuration.application.axis_profile_valid = true;
        app->axis.current_offsets.complete = true;
        app->axis.motor.fault_code = MOTOR_FAULT_NONE;
        app->axis.speed_pi.integral_output = (float)(id + 1);
    }
    MotorAxis_ControlFast(&apps[0].axis);
    if (!nested)
    {
        MotorAxis_ControlFast(&apps[1].axis);
    }
    for (unsigned id = 0; id < 2; id++)
    {
        MotorAxis *axis = &apps[id].axis;
        assert(axis->state_machine.fast_sample_ticks == 1 && devices[id].duty_writes == 1);
        assert(axis->speed_pi.integral_output == (float)(id + 1));
        axis->state_machine.state = MOTOR_STATE_CALIBRATION;
        axis->state_machine.calibration_type = MOTOR_CALIBRATION_MECHANICAL_ZERO;
        axis->state_machine.calibration_handoff = MOTOR_CALIBRATION_AWAITING_PUBLICATION;
    }
    /* 真正的零位算法与 FOC 在提交 PWM 的途中进入第二根轴的同一算法。 */
    calibrating = true;
    for (unsigned id = 0; id < 2; id++)
    {
        MotorAxis *axis = &apps[id].axis;
        axis->configuration.application.current_limit_a = 4.0f;
        axis->configuration.model.calibration_current_a = 2.0f;
        axis->foc.bus_voltage_v = axis->foc.bus_voltage_filtered_v = 24.0f;
        axis->foc.phase_current_a = (ABCVect){0};
        axis->calibration.session.zero.ticks = 100U * id;
    }
    electrical_zero(0);
    if (!nested)
    {
        electrical_zero(1);
    }
    calibrating = false;
    for (unsigned id = 0; id < 2; id++)
    {
        assert(apps[id].axis.calibration.session.zero.ticks == 100U * id + 1);
        assert(devices[id].duty_writes == 2);
    }
    publish(0);
    if (!nested)
    {
        publish(1);
    }
    for (unsigned id = 0; id < 2; id++)
    {
        MotorAxis *axis = &apps[id].axis;
        assert(devices[id].record_writes == 1 &&
               axis->state_machine.calibration_handoff == MOTOR_CALIBRATION_AWAITING_COMPLETION);
        assert(devices[id].saved.motor.record_crc32 == Param_RecordCrc(&devices[id].saved.motor));
    }
    assert(memcmp(&devices[0].saved, &devices[1].saved, sizeof(MotorAxisRecord)) != 0);
    assert(depth == 0);
}

static void stop_between_snapshot_and_commit(void)
{
    MotorAxis *axis = &apps[0].axis;
    MotorAxis *other = &apps[1].axis;
    axis->state_machine.state = MOTOR_STATE_RUNNING;
    axis->state_machine.calibration_type = MOTOR_CALIBRATION_NONE;
    axis->motor.control_mode = MOTOR_CONTROL_SPEED;
    MotorOuterWork work;
    MotorStateMachine_ReadOuterInputs(&axis->state_machine, &work);
    assert(work.allowed);
    work.motor.iq_target_a = 31.0f;
    work.speed_pi.integral_output = 19.0f;
    MotorStateMachine_CommitOuterResult(&other->state_machine, &work);
    assert(other->motor.iq_target_a != 31.0f && other->speed_pi.integral_output == 2.0f);
    ProtocolCommand command = {0};
    ProtocolReply reply = {0};
    ProtocolMotor *motor = &apps[0].endpoint.motor;
    devices[0].disabled = false;
    motor->stop(motor->context, &command, &reply);
    assert(reply.status == PROTOCOL_ACCEPTED && reply.detail == 1U &&
           axis->state_machine.stop_pending);
    assert(!other->state_machine.stop_pending);
    MotorStateMachine_CommitOuterResult(&axis->state_machine, &work);
    assert(devices[0].disabled && axis->motor.iq_target_a == 0.0f &&
           axis->speed_pi.integral_output == 0.0f);
    assert(axis->state_machine.phase_enable_epoch == 0 && axis->state_machine.epoch != work.epoch);
    motor->stop(motor->context, &command, &reply);
    assert(reply.status == PROTOCOL_OK && reply.detail == 0U);
    BspCanFrame frame = {.identifier = 17};
    assert(Protocol_QueueReceivedFrame(&apps[0].endpoint, &frame));
    assert(queue_count(&apps[0].endpoint.rx_queue) == 1 &&
           queue_count(&apps[1].endpoint.rx_queue) == 0);
}
static void motion_permission_expires(void)
{
    MotorStateMachine *machine = &apps[0].axis.state_machine;
    ProtocolMotor *motor = &apps[0].endpoint.motor;
    const MotorControlMode modes[] = {MOTOR_CONTROL_CURRENT,
                                      MOTOR_CONTROL_SPEED,
                                      MOTOR_CONTROL_POSITION,
                                      MOTOR_CONTROL_VQ,
                                      MOTOR_CONTROL_OPEN_VOLTAGE,
                                      MOTOR_CONTROL_OPEN_CURRENT};
    for (unsigned i = 0; i < sizeof(modes) / sizeof(modes[0]); ++i)
    {
        machine->state = MOTOR_STATE_RUNNING;
        machine->drive.motor->control_mode = modes[i];
        machine->drive.motor->fault_code = MOTOR_FAULT_NONE;
        machine->motion_lease_active = true;
        machine->motion_renewed_ms = now_ms = UINT32_MAX - 200U;
        machine->stop_pending = false;
        devices[0].disabled = false;
        apps[0].protocol.renewal_counter = 0U;
        ProtocolCommand request = {.renewal_counter = 1U};
        ProtocolReply reply;
        now_ms += 10U;
        motor->renew(motor->context, &request, &reply);
        assert(reply.status == PROTOCOL_OK && machine->motion_renewed_ms == now_ms);
        now_ms += 100U;
        const uint8_t wire_modes[] = {1U, 2U, 3U, 6U, 9U, 10U};
        ProtocolCommand old_target = {.mode = wire_modes[i], .sequence = 17U};
        apps[0].protocol.last_target = old_target;
        apps[0].protocol.last_target_epoch = machine->epoch;
        uint32_t last_renewed = machine->motion_renewed_ms;
        motor->target(motor->context, &old_target, &reply);
        assert(reply.status == PROTOCOL_OK && machine->motion_renewed_ms == last_renewed);
        ProtocolFeedback feedback = {0};
        (void)motor->read_feedback(motor->context, &feedback);
        assert(machine->motion_renewed_ms == last_renewed);
        motor->renew(motor->context, &request, &reply);
        assert(reply.status == PROTOCOL_DENIED && machine->motion_renewed_ms != now_ms);
        now_ms += 400U;
        request.renewal_counter = 2U;
        motor->renew(motor->context, &request, &reply);
        assert(reply.status == PROTOCOL_DENIED);
        MotorStateMachine_SupervisorTick(machine);
        assert(!machine->motion_lease_active && devices[0].disabled);
        assert(machine->drive.motor->control_mode == MOTOR_CONTROL_NONE);
        motor->renew(motor->context, &request, &reply);
        assert(reply.status == PROTOCOL_DENIED && devices[0].disabled);
        assert(!apps[1].axis.state_machine.motion_lease_active);
    }
}
static void parameter_and_output_services(void)
{
    MotorAxis *axis = &apps[0].axis;
    nested = false;
    devices[0].disabled = true;
    axis->state_machine.state = MOTOR_STATE_IDLE;
    axis->state_machine.stop_pending = false;
    axis->state_machine.start_pending = false;
    axis->state_machine.starts_locked = false;
    axis->state_machine.calibration_type = MOTOR_CALIBRATION_NONE;
    axis->state_machine.calibration_handoff = MOTOR_CALIBRATION_NO_HANDOFF;
    axis->motor.fault_code = MOTOR_FAULT_NONE;
    axis->motor.control_mode = MOTOR_CONTROL_NONE;
#if BSP_BOARD == BSP_BOARD_STM32G4
    assert(MotorAxisProfile_Create(
        &axis->configuration.application.axis_profile, "roll", -1.0f, 1.0f, 2.0f));
    axis->configuration.application.axis_profile_valid = true;
    axis->configuration.application.speed_ceiling_rad_s = 2.0f;
    axis->configuration.model.thermal = (MotorWindingThermalConfig){3, 7, 66, .8f, .7f, 1, .5f};
#endif
    axis->configuration.device.phase_a_offset_counts = 2048.0f;
    axis->configuration.device.phase_b_offset_counts = 2048.0f;
    axis->configuration.device.phase_c_offset_counts = 2048.0f;
#if BSP_BOARD == BSP_BOARD_AT32F456
    axis->configuration.device.node_id = 3U;
#endif
    axis->configuration.configuration_confirmed = true;
    ParameterWriteRequest write = {.parameter_id = 1U,
                                   .value_type = PARAMETER_VALUE_I32,
                                   .expected_revision = axis->parameters.revision,
                                   .raw_value = 1000U};
    uint32_t before = axis->parameters.revision;
    assert(MotorAxis_WriteParameter(axis, &write) == PARAMETER_OK);
    assert(axis->parameters.revision == before + 1U);
    assert(MotorAxis_WriteParameter(axis, &write) == PARAMETER_OK);
    assert(axis->parameters.revision == before + 1U);
    write.raw_value = 1100U;
    assert(MotorAxis_WriteParameter(axis, &write) == PARAMETER_REVISION_CONFLICT);
    write.expected_revision = axis->parameters.revision;
    write.raw_value = 0U;
    assert(MotorAxis_WriteParameter(axis, &write) == PARAMETER_OUT_OF_RANGE);
    write.parameter_id = 16U;
    write.raw_value = 500000U;
    assert(MotorAxis_WriteParameter(axis, &write) == PARAMETER_OK);
    assert(axis->speed_pi.proportional_gain == 0.5f);
    uint32_t revision = axis->parameters.revision;
    unsigned writes = devices[0].record_writes;
    assert(MotorAxis_SaveParameters(axis, revision) == PARAMETER_OK);
    assert(axis->parameters.saved_revision == revision && devices[0].record_writes == writes + 1U);
    assert(MotorAxis_SaveParameters(axis, revision) == PARAMETER_OK);
    assert(devices[0].record_writes == writes + 1U);
    load_saved = true;
    ParameterValue value;
    assert(MotorAxis_ReadParameter(axis, 16U, true, &value) == PARAMETER_OK);
    assert(value.raw_value == 500000U);
    assert(MotorAxis_ReadParameter(axis, 0x70U, true, &value) == PARAMETER_OK);
    uint32_t coordinate_crc = value.raw_value;
    assert(MotorAxis_ReadParameter(axis, 0x70U, false, &value) == PARAMETER_OK);
    assert(value.raw_value == coordinate_crc);
    write.parameter_id = 16U;
    write.expected_revision = axis->parameters.revision;
    write.raw_value = 600000U;
    assert(MotorAxis_WriteParameter(axis, &write) == PARAMETER_OK);
    assert(MotorAxis_ReadParameter(axis, 0x70U, false, &value) == PARAMETER_OK);
    assert(value.raw_value == coordinate_crc);
    ++axis->configuration.device.encoder.mechanical_zero_q15;
    assert(MotorAxis_ReadParameter(axis, 0x70U, false, &value) == PARAMETER_OK);
    assert(value.raw_value != coordinate_crc);
    write_fails = true;
    assert(MotorAxis_SaveParameters(axis, axis->parameters.revision) == PARAMETER_PERSIST_FAILED);
    assert(axis->parameters.saved_revision == 0U);
    assert(devices[0].disabled && !axis->state_machine.parameters_updating);
    write_fails = false;
    axis->state_machine.state = MOTOR_STATE_RUNNING;
    assert(MotorAxis_SaveParameters(axis, axis->parameters.revision) == PARAMETER_WRONG_STATE);
    axis->state_machine.state = MOTOR_STATE_IDLE;
    MotorOutputFeedback output;
    axis->output_feedback = (MotorOutputTelemetry){.hall_position_rad = 1.25f,
                                                   .tracked_position_rad = -2.5f,
                                                   .speed_rad_s = -1.0f,
                                                   .zero_rad = 0.125f,
                                                   .gear_ratio = 8.0f,
                                                   .sample_ms = now_ms,
                                                   .sample_counter = 123U,
                                                   .flags = 31U};
    axis->output_feedback.direction = -1;
    assert(apps[0].endpoint.motor.read_output(&apps[0].protocol, &output));
    assert(output.flags == 31U && output.hall_mrad == 1250 && output.tracked_mrad == -2500);
    axis->state_machine.fast_samples_fresh = true;
    axis->rotor_encoder.has_valid_sample = true;
    /* ADC 已有更新时，CAN 仍发送与输出位置同一外环样本的速度。 */
    axis->rotor_encoder.mechanical_velocity_continuous_rad_s = 16.0f;
    ProtocolFeedback output_motion = {0};
    assert(apps[0].endpoint.motor.read_feedback(&apps[0].protocol, &output_motion));
    assert(output_motion.actual_position_mrad == -2500 &&
           output_motion.actual_speed_mrad_s == -1000);
    now_ms += 11U;
    assert(apps[0].endpoint.motor.read_output(&apps[0].protocol, &output));
    assert((output.flags & 3U) == 0U && output.hall_mrad == INT32_MIN);
    output_motion = (ProtocolFeedback){0};
    assert(apps[0].endpoint.motor.read_feedback(&apps[0].protocol, &output_motion));
    assert((output_motion.valid_bits & 15U) == 0U &&
           output_motion.actual_speed_mrad_s == INT32_MIN);
    axis->motor_temperature = (MotorTemperatureTelemetry){25.0f, now_ms, true};
    ProtocolFeedback feedback = {0};
    axis->state_machine.fast_samples_fresh = true;
    assert(apps[0].endpoint.motor.read_feedback(&apps[0].protocol, &feedback));
    assert(feedback.motor_temperature_centi_c == 2500 && (feedback.valid_bits & (1U << 9)));
    now_ms += 31U;
    feedback = (ProtocolFeedback){0};
    assert(apps[0].endpoint.motor.read_feedback(&apps[0].protocol, &feedback));
    assert(!(feedback.valid_bits & (1U << 9)) && feedback.motor_temperature_centi_c == INT16_MIN);
    assert(apps[1].axis.parameters.revision != axis->parameters.revision);
    uint32_t restore_revision = axis->parameters.revision;
    assert(MotorAxis_ReadParameter(axis, 0x70U, false, &value) == PARAMETER_OK);
    uint32_t before_restore_crc = value.raw_value;
    MotorAxisProfile before_restore_profile = axis->configuration.application.axis_profile;
    assert(MotorAxis_RestoreControlDefaults(axis, restore_revision) == PARAMETER_OK);
    assert(axis->parameters.revision == restore_revision + 1U);
    assert(ParameterValue_ConfigurationValid(&axis->configuration));
    assert(memcmp(&before_restore_profile,
                  &axis->configuration.application.axis_profile,
                  sizeof(before_restore_profile)) == 0);
    assert(MotorAxis_ReadParameter(axis, 0x70U, false, &value) == PARAMETER_OK);
    assert(value.raw_value == before_restore_crc);
    assert(MotorAxis_RestoreControlDefaults(axis, restore_revision) == PARAMETER_REVISION_CONFLICT);
    uint32_t quantized_revision = axis->parameters.revision;
    write = (ParameterWriteRequest){.parameter_id = 16U,
                                    .value_type = PARAMETER_VALUE_I32,
                                    .expected_revision = quantized_revision,
                                    .raw_value = 900000001U};
    assert(MotorAxis_WriteParameter(axis, &write) == PARAMETER_OK);
    assert(axis->parameters.revision == quantized_revision + 1U);
    assert(MotorAxis_WriteParameter(axis, &write) == PARAMETER_OK);
    write.expected_revision = axis->parameters.revision;
    write.raw_value = 900000002U;
    assert(MotorAxis_WriteParameter(axis, &write) == PARAMETER_OK);
    assert(axis->parameters.revision == quantized_revision + 1U);
    write.parameter_id = 3U;
    write.raw_value = axis->configuration.device.node_id;
    assert(MotorAxis_WriteParameter(axis, &write) == PARAMETER_READ_ONLY);
    ParameterWriteRequest rate = {.parameter_id = 0x71U,
                                  .value_type = PARAMETER_VALUE_U32,
                                  .expected_revision = axis->parameters.revision,
                                  .raw_value = (1000UL << 16U) | 3000U};
#if BSP_BOARD == BSP_BOARD_AT32F456
    axis->state_machine.state = MOTOR_STATE_RUNNING;
    assert(MotorAxis_WriteParameter(axis, &rate) == PARAMETER_WRONG_STATE);
    axis->state_machine.state = MOTOR_STATE_IDLE;
    assert(MotorAxis_WriteParameter(axis, &rate) == PARAMETER_OK);
    assert(MotorAxis_WriteParameter(axis, &rate) == PARAMETER_OK);
    assert(axis->parameters.revision == rate.expected_revision + 1U);
    assert(MotorAxis_SaveParameters(axis, rate.expected_revision) == PARAMETER_REVISION_CONFLICT);
    assert(MotorAxis_ReadParameter(axis, 0x71U, true, &value) == PARAMETER_OK);
    assert(value.raw_value == ((1000UL << 16U) | 5000U));
    assert(MotorAxis_SaveParameters(axis, axis->parameters.revision) == PARAMETER_OK);
    assert(MotorAxis_ReadParameter(axis, 0x71U, true, &value) == PARAMETER_OK);
    assert(value.raw_value == rate.raw_value);
#else
    assert(MotorAxis_WriteParameter(axis, &rate) == PARAMETER_UNSUPPORTED);
#endif
    write.parameter_id = 16U;
    axis->parameters.revision = UINT32_MAX;
    write.expected_revision = UINT32_MAX;
    assert(MotorAxis_WriteParameter(axis, &write) == PARAMETER_OUT_OF_RANGE);
    MotorAxisRecord canonical = {0};
    canonical.motor.motor_pole_pairs = 7.0f;
    canonical.motor.encoder_electrical_zero_q15 = 0x1234U;
    canonical.motor.encoder_mechanical_zero_q15 = 0x5678U;
    canonical.motor.encoder_calib_flag = 7U;
    canonical.motor.encoder_reverse = 1U;
    canonical.motor.hall_sin_off_counts = 2000.0f;
    canonical.motor.hall_cos_off_counts = 2100.0f;
    canonical.motor.hall_sin_amp = 1000.0f;
    canonical.motor.hall_cos_amp = 1100.0f;
    canonical.motor.hall_tag_status = 0x48410180U;
    canonical.output.magic = PARAM_OUTPUT_ENCODER_V3;
    canonical.output.params = (OutputEncoderParams){8.0f, .125f, .25f, -1, 0x180U};
    canonical.output.hall_lut_status = 0x80U;
    printf("COORDINATE_CRC %08x\n", (unsigned)Param_CoordinateCrc(&canonical));

    puts("PASS parameter revision/retry/limits, real save/failure/reload, coordinate CRC, "
         "output/NTC freshness");
}

static void test_maintenance_storage_exclusion(void)
{
    MotorAxis *axis = &apps[0].axis;
    axis->state_machine.maintenance_capture = true;
    memset(&axis->calibration.storage, 0x5a, sizeof(axis->calibration.storage));
    ParameterValue value;
    assert(MotorAxis_ReadParameter(axis, 0x70U, false, &value) == PARAMETER_WRONG_STATE);
    assert(MotorAxis_ReadParameter(axis, 0x70U, true, &value) == PARAMETER_WRONG_STATE);
    assert(MotorAxis_ReadParameter(axis, 16U, true, &value) == PARAMETER_WRONG_STATE);
    const uint8_t *storage = (const uint8_t *)&axis->calibration.storage;
    for (unsigned i = 0; i < sizeof(axis->calibration.storage); ++i)
    {
        assert(storage[i] == 0x5a);
    }
    axis->state_machine.maintenance_capture = false;
    assert(MotorAxis_ReadParameter(axis, 0x70U, false, &value) == PARAMETER_OK);
}

static void calibration_readback_services(void)
{
    MotorAxis *axis = &apps[0].axis;
    ProtocolCalibrationReadback *service = &apps[0].endpoint.calibration_readback;
    ProtocolCalibrationRequest request = {0};
    ProtocolCalibrationBlock block = {0};
    uint16_t detail = 0U;
    axis->parameters.saved_revision = 0U;
    assert(service->read_record(service->context, &request, &block, &detail) ==
           PROTOCOL_RESULT_FAILED);
    axis->parameters.saved_revision = 7U;
    assert(service->read_record(service->context, &request, &block, &detail) == PROTOCOL_RESULT_OK);
    assert(block.total == sizeof(MotorAxisRecord));
    assert(block.crc32 == Crc32_Compute(&devices[0].saved, sizeof(devices[0].saved), 0U));
    request = (ProtocolCalibrationRequest){block.revision, block.crc32, 7U, 28U};
    assert(service->read_record(service->context, &request, &block, &detail) == PROTOCOL_RESULT_OK);
    assert(memcmp(block.data, (const uint8_t *)&devices[0].saved + 7U, 28U) == 0);
    request.offset = block.total - 1U;
    assert(service->read_record(service->context, &request, &block, &detail) ==
           PROTOCOL_RESULT_OUT_OF_RANGE);
    request.offset = 7U;
    ++request.revision;
    assert(service->read_record(service->context, &request, &block, &detail) ==
           PROTOCOL_RESULT_DENIED);
    assert(detail == 6U);
    axis->state_machine.maintenance_capture = true;
    request = (ProtocolCalibrationRequest){0};
    assert(service->read_record(service->context, &request, &block, &detail) ==
           PROTOCOL_RESULT_DENIED);
    assert(detail == 5U);
    axis->state_machine.maintenance_capture = false;
#if BSP_ENABLED_LINEAR_HALL
    const MotorAxisPreset *previous = axis->configuration.preset;
    const MotorAxisPreset preset = {.has_linear_hall = true};
    axis->configuration.preset = &preset;
    axis->hall_published_slot = 1U;
    axis->hall_samples[1].sin_counts = 123U;
    axis->hall_samples[1].cos_counts = 3456U;
    axis->state_machine.fast_sample_ticks = 7U;
    ProtocolHallRaw sample = {0};
    assert(service->read_hall(service->context, &sample) == PROTOCOL_RESULT_OK);
    assert(sample.flags == 1U && sample.sin_counts == 123U && sample.cos_counts == 3456U);
    assert(sample.sample_counter == 7U);
    axis->configuration.preset = previous;
#endif
    puts("PASS saved-record blocks, revision/CRC/limit guards and raw Hall snapshot");
}

/* 真实快拍、发布、Flash记录和重载均不得改写电机原点。 */
static void output_mechanical_zero(void)
{
    nested = false;
    write_fails = false;
    MotorAxis *axis = &apps[0].axis;
    MotorStateMachine *machine = &axis->state_machine;
    machine->state = MOTOR_STATE_CALIBRATION;
    machine->calibration_type = MOTOR_CALIBRATION_MECHANICAL_ZERO;
    machine->calibration_handoff = MOTOR_CALIBRATION_NO_HANDOFF;
    machine->stop_pending = false;
    machine->start_pending = false;
    axis->motor.fault_code = MOTOR_FAULT_NONE;
    devices[0].disabled = true;
    axis->rotor_encoder.has_valid_sample = true;
    axis->rotor_encoder.linearized_q15 = 8192U;
    axis->rotor_encoder.mechanical_velocity_continuous_rad_s = 0.0f;
    axis->rotor_encoder.mechanical_velocity_rad_s = 0.0f;
    axis->rotor_encoder.zero_requested = false;
    axis->configuration.device.encoder.mechanical_zero_q15 = 1234U;
    uint16_t flags = axis->configuration.device.encoder.calib_flag;
    OutputEncoder_Init(&axis->output_encoder, 8.0f);
    axis->output_encoder.params.status = OUTPUT_ENCODER_CAL_OK;
    axis->output_encoder.params.direction = -1;
    OutputEncoder_Locate(&axis->output_encoder, 8192U, 1.0f);
    float expected_zero = axis->output_encoder.position_rad;
    assert(MotorCalibration_StepFast(&axis->calibration).result == MOTOR_TASK_COMPLETED);
    assert(axis->configuration.device.encoder.mechanical_zero_q15 == 1234U &&
           axis->configuration.device.encoder.calib_flag == flags &&
           !axis->rotor_encoder.zero_requested);
    machine->calibration_handoff = MOTOR_CALIBRATION_AWAITING_PUBLICATION;
    publish(0);
    assert(axis->motor.fault_code == MOTOR_FAULT_NONE && axis->output_encoder.position_rad == 0.0f);
    assert(axis->output_encoder.params.zero_rad == expected_zero);
    assert(devices[0].saved.motor.encoder_mechanical_zero_q15 == 1234U);
    OutputEncoder restored;
    LinearHall hall;
    OutputEncoder_Init(&restored, 8.0f);
    LinearHall_Init(&hall);
    assert(Param_RestoreOutputEncoder(&devices[0].saved, &restored, &hall));
    OutputEncoder_Locate(&restored, 8192U, 1.0f);
    assert(restored.position_rad == 0.0f && restored.params.zero_rad == expected_zero);
    axis->output_encoder.ready = false;
    assert(MotorCalibration_StepFast(&axis->calibration).result == MOTOR_TASK_FAILED);
}

static void test_recovered_plan_parameters(void)
{
    MotorAxis *axis = &apps[0].axis;
    MotorPositionPlan *work = &axis->outer.plan.work;
    const PositionOtgReference start = {-0x1.57d594p-5f, 0x1.851eb8p-2f, -0x1.b224cep-11f};
    ParameterValue value;
    assert(MotorAxis_ReadParameter(axis, 0x72U, false, &value) == PARAMETER_OK &&
           value.raw_value == 0U);
    assert(PositionOtg_Reset(&work->trajectory, start));
    work->target_rad = -1e-9f;
    work->limits = (PositionOtgLimits){0.38f, 3.14159265f, 3.14159265f / 0.1f};
    axis->outer.plan.state = PLAN_QUEUED;
    MotorAxis_ProcessPositionPlan(axis);
    assert(axis->outer.plan.state == PLAN_READY && axis->recovered_plan_count == 1U);
    printf("OTG_RECOVERY");
    for (uint8_t id = 0x72U; id <= 0x79U; ++id)
    {
        assert(MotorAxis_ReadParameter(axis, id, false, &value) == PARAMETER_OK);
        assert(value.parameter_id == id && value.access == PARAMETER_ACCESS_READ &&
               value.value_type == PARAMETER_VALUE_U32);
        printf(" %08x", (unsigned)value.raw_value);
        assert(MotorAxis_ReadParameter(axis, id, true, &value) == PARAMETER_UNSUPPORTED);
        ParameterWriteRequest attempt = {.parameter_id = id};
        assert(MotorAxis_WriteParameter(axis, &attempt) == PARAMETER_READ_ONLY);
    }
    puts("");
    work->lead_ticks = 1U;
    axis->outer.plan.state = PLAN_QUEUED;
    MotorAxis_ProcessPositionPlan(axis);
    assert(axis->outer.plan.state == PLAN_READY && axis->recovered_plan_count == 1U);
    assert(apps[1].axis.recovered_plan_count == 0U);
}

/* 经实际 CAN 编解码、参数分发和轴维护写入口，使用当前读回的修订号。 */
static uint32_t exchange_ramp_parameter(ProtocolEndpoint *host,
                                        ProtocolEndpoint *drive,
                                        uint16_t type,
                                        uint8_t id,
                                        uint32_t value,
                                        uint32_t *revision,
                                        ProtocolCode expected)
{
    uint8_t payload[12] = {id};
    bool write = type == PROTOCOL_WRITE_PARAMETER;
    if (write)
    {
        payload[1] = PARAMETER_VALUE_I32;
        payload[2] = 4U;
        YgProtocol_WriteU32Le(payload + 4, *revision);
        YgProtocol_WriteU32Le(payload + 8, value);
    }
    yg_protocol_message_t request = {.version = YG_PROTOCOL_VERSION,
                                     .flags = YG_PROTOCOL_FLAGS_ACK_REQUEST,
                                     .source_node = host->node_id,
                                     .destination_node = drive->node_id,
                                     .message_type = type,
                                     .sequence = (uint16_t)(++now_ms),
                                     .payload_length = write ? 12U : 4U,
                                     .payload = payload};
    assert(Protocol_QueueMessage(host, &request, 3U) == YG_PROTOCOL_OK);
    BspCanFrame frame;
    assert(queue_pop(&host->tx_queue, &frame) && Protocol_QueueReceivedFrame(drive, &frame));
    Protocol_Process(drive, now_ms);
    assert(queue_pop(&drive->tx_queue, &frame));
    yg_protocol_message_t reply;
    size_t length =
        YG_PROTOCOL_HEADER_SIZE + YgProtocol_ReadU32Le(frame.data + 10U) + YG_PROTOCOL_TRAILER_SIZE;
    assert(length <= frame.length &&
           yg_protocol_frame_decode(frame.data, length, &reply) == YG_PROTOCOL_OK);
    assert(reply.source_node == drive->node_id && reply.destination_node == host->node_id &&
           reply.message_type == type && reply.sequence == request.sequence);
    if (YgProtocol_ReadU16Le(reply.payload) != expected)
    {
        fprintf(stderr,
                "node %u type %u parameter %u value %u: result %u, expected %u\n",
                drive->node_id,
                type,
                id,
                value,
                YgProtocol_ReadU16Le(reply.payload),
                expected);
    }
    assert(YgProtocol_ReadU16Le(reply.payload) == expected);
    if (expected != PROTOCOL_RESULT_OK)
    {
        assert(reply.payload_length == 4U && YgProtocol_ReadU16Le(reply.payload + 2) == 4U);
        return 0U;
    }
    assert(reply.payload_length == 20U && reply.payload[4] == id &&
           reply.payload[5] == PARAMETER_VALUE_I32);
    *revision = YgProtocol_ReadU32Le(reply.payload + 12);
    return YgProtocol_ReadU32Le(reply.payload + 8);
}

static void test_ramp_parameter_roundtrip(void)
{
    ProtocolEndpoint host = {0};
    assert(Protocol_Init(&host, 2U));
    MotorAxis *axis = &apps[0].axis;
    nested = false;
    load_saved = false;
    memset(&apps[0], 0, sizeof(apps[0]));
    axis->hardware = &hardware[0];
    devices[0].disabled = true;
    App_Init(&apps[0]);
    axis->state_machine.state = MOTOR_STATE_IDLE;
    axis->motor.fault_code = MOTOR_FAULT_NONE;
    const unsigned writes = devices[0].record_writes;
    const MotorAxisRecord saved = devices[0].saved;
    const uint32_t saved_revision = axis->parameters.saved_revision;
    /* 左轮、右轮、Roll、Pitch、Yaw、Lift 的产品上限，以 mrad/s² 表示。 */
    const uint32_t wire_limits[] = {20000U, 20000U, 3142U, 83776U, 83776U, 83776U};
    const MotorWindingThermalConfig thermal = {3, 7, 66, .8f, .7f, 1, .5f};
    for (unsigned node = 3U; node <= 8U; ++node)
    {
        bool lift = node == 8U;
        assert(Param_CreateAxis(
            node, lift ? -1.0f : 0.0f, lift ? 1.0f : 0.0f, 1.0f, &thermal, &axis->configuration));
        ProtocolEndpoint drive = {0};
        assert(Protocol_Init(&drive, node) && Protocol_Start(&drive));
        drive.parameters = apps[0].endpoint.parameters;
        const uint32_t limit = wire_limits[node - 3U];
        const MotorApplicationConfig *app = &axis->configuration.application;
        const uint8_t ids[] = {19U, 20U, 24U, 25U};
        const float *values[] = {&app->speed_acceleration_rad_s2,
                                 &app->speed_deceleration_rad_s2,
                                 &app->position_acceleration_rad_s2,
                                 &app->position_deceleration_rad_s2};
        for (unsigned i = 0; i < sizeof(ids); ++i)
        {
            uint8_t id = ids[i];
            const float ceiling = *values[i];
            uint32_t revision = 0U;
            uint32_t original = exchange_ramp_parameter(
                &host, &drive, PROTOCOL_READ_PARAMETER, id, 0, &revision, PROTOCOL_RESULT_OK);
            assert(original == limit);
            assert(exchange_ramp_parameter(&host,
                                           &drive,
                                           PROTOCOL_WRITE_PARAMETER,
                                           id,
                                           limit - 1U,
                                           &revision,
                                           PROTOCOL_RESULT_OK) == limit - 1U);
            assert(*values[i] < ceiling);
            assert(
                exchange_ramp_parameter(
                    &host, &drive, PROTOCOL_READ_PARAMETER, id, 0, &revision, PROTOCOL_RESULT_OK) ==
                limit - 1U);
            assert(exchange_ramp_parameter(&host,
                                           &drive,
                                           PROTOCOL_WRITE_PARAMETER,
                                           id,
                                           original,
                                           &revision,
                                           PROTOCOL_RESULT_OK) == original);
            assert(*values[i] == ceiling);
            assert(
                exchange_ramp_parameter(
                    &host, &drive, PROTOCOL_READ_PARAMETER, id, 0, &revision, PROTOCOL_RESULT_OK) ==
                original);
            uint32_t restored_revision = revision;
            (void)exchange_ramp_parameter(&host,
                                          &drive,
                                          PROTOCOL_WRITE_PARAMETER,
                                          id,
                                          limit + 1U,
                                          &revision,
                                          PROTOCOL_RESULT_OUT_OF_RANGE);
            assert(
                exchange_ramp_parameter(
                    &host, &drive, PROTOCOL_READ_PARAMETER, id, 0, &revision, PROTOCOL_RESULT_OK) ==
                original);
            assert(revision == restored_revision && *values[i] == ceiling);
        }
    }
    assert(devices[0].record_writes == writes && !memcmp(&saved, &devices[0].saved, sizeof(saved)));
    assert(axis->parameters.saved_revision == saved_revision);
    puts("PASS Type120/121 ramp limit round trips for nodes 3-8 without Flash writes");
}

int main(void)
{
    run(false);
    Device reference[2];
    memcpy(reference, devices, sizeof(reference));
    run(true);
    assert(memcmp(reference, devices, sizeof(devices)) == 0);
    stop_between_snapshot_and_commit();
    motion_permission_expires();
    test_maintenance_storage_exclusion();
    parameter_and_output_services();
    calibration_readback_services();
    output_mechanical_zero();
    test_recovered_plan_parameters();
    test_ramp_parameter_roundtrip();
    puts("PASS independent and nested startup, fast PWM, calibration records, STOP epoch/PI and "
         "CAN queues");
    return 0;
}

void BspPwm_SelectPhases(uint8_t mask)
{
    (void)mask;
}
