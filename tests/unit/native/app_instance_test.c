/* 生产实例不共享电机、协议或标定状态。设备回调在 A 尚未返回时进入 B 的同一入口。 */
#include "app.h"
#include "boot_hw.h"
#include "bsp_adc.h"
#include "bsp_encoder.h"
#include "bsp_pwm.h"
#include "bsp_system.h"
#include "motor_axis_profile.h"
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
    assert(data == &apps[0].axis.record && length == sizeof(MotorAxisRecord));
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
    assert(data == &apps[1].axis.record && length == sizeof(MotorAxisRecord));
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
    assert(data == &apps[id].axis.record && length == sizeof(MotorAxisRecord));
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
    {{unexpected_enable, off_a, disabled_a, duty_a}, read_a, write_a},
    {{unexpected_enable, off_b, disabled_b, duty_b}, read_b, write_b}};

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
                                                   .zero_rad = 0.125f,
                                                   .gear_ratio = 8.0f,
                                                   .sample_ms = now_ms,
                                                   .sample_counter = 123U,
                                                   .flags = 31U};
    assert(apps[0].endpoint.motor.read_output(&apps[0].protocol, &output));
    assert(output.flags == 31U && output.hall_mrad == 1250 && output.tracked_mrad == -2500);
    now_ms += 11U;
    assert(apps[0].endpoint.motor.read_output(&apps[0].protocol, &output));
    assert((output.flags & 3U) == 0U && output.hall_mrad == INT32_MIN);
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
    assert(value.raw_value == ((500UL << 16U) | 2000U));
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
int main(void)
{
    run(false);
    Device reference[2];
    memcpy(reference, devices, sizeof(reference));
    run(true);
    assert(memcmp(reference, devices, sizeof(devices)) == 0);
    stop_between_snapshot_and_commit();
    motion_permission_expires();
    parameter_and_output_services();
    puts("PASS independent and nested startup, fast PWM, calibration records, STOP epoch/PI and "
         "CAN queues");
    return 0;
}
