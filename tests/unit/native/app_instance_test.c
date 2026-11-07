/* 生产实例不共享电机、协议或标定状态。设备回调在 A 尚未返回时进入 B 的同一入口。 */
#include "app.h"
#include "boot_hw.h"
#include "bsp_adc.h"
#include "bsp_encoder.h"
#include "bsp_pwm.h"
#include "bsp_system.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static AppContext apps[2];
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
    MotorCalibration_ProcessPublication(&apps[id].axis.calibration, NULL, NULL);
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
uint32_t BspSystem_TimeMs(void)
{
    return 0U;
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
BspResult BspWatchdog_Start(void)
{
    return BSP_NOT_SUPPORTED;
}
void BspWatchdog_Feed(void)
{
    assert(false);
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
        assert(app->axis.lifecycle.drive.motor == &app->axis.motor &&
               app->protocol.lifecycle == &app->axis.lifecycle);
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
        assert(axis->lifecycle.fast_sample_ticks == 1 && devices[id].duty_writes == 1);
        assert(axis->speed_pi.integral_output == (float)(id + 1));
        axis->lifecycle.state = MOTOR_STATE_CALIBRATION;
        axis->lifecycle.calibration_type = MOTOR_CALIBRATION_MECHANICAL_ZERO;
        axis->lifecycle.calibration_publication_pending = true;
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
        assert(devices[id].record_writes == 1 && axis->lifecycle.calibration_completion_pending);
        assert(devices[id].saved.motor.record_crc32 == Param_RecordCrc(&devices[id].saved.motor));
    }
    assert(memcmp(&devices[0].saved, &devices[1].saved, sizeof(MotorAxisRecord)) != 0);
    assert(depth == 0);
}

static void stop_between_snapshot_and_commit(void)
{
    MotorAxis *axis = &apps[0].axis;
    MotorAxis *other = &apps[1].axis;
    axis->lifecycle.state = MOTOR_STATE_RUNNING;
    axis->lifecycle.calibration_type = MOTOR_CALIBRATION_NONE;
    axis->motor.control_mode = MOTOR_CONTROL_SPEED;
    MotorOuterWork work;
    MotorLifecycle_BeginOuter(&axis->lifecycle, &work);
    assert(work.allowed);
    work.motor.iq_target_a = 31.0f;
    work.speed_pi.integral_output = 19.0f;
    MotorLifecycle_CommitOuter(&other->lifecycle, &work);
    assert(other->motor.iq_target_a != 31.0f && other->speed_pi.integral_output == 2.0f);
    ProtocolCommand command = {0};
    ProtocolReply reply = {0};
    ProtocolMotor *motor = &apps[0].endpoint.motor;
    devices[0].disabled = false;
    motor->stop(motor->context, &command, &reply);
    assert(reply.status == PROTOCOL_ACCEPTED && reply.token != 0);
    assert(apps[1].protocol.stop_token == 0 && !other->lifecycle.stop_pending);
    MotorLifecycle_CommitOuter(&axis->lifecycle, &work);
    assert(devices[0].disabled && axis->motor.iq_target_a == 0.0f &&
           axis->speed_pi.integral_output == 0.0f);
    assert(axis->lifecycle.power_enable_epoch == 0 && axis->lifecycle.epoch != work.epoch);
    uint32_t token = reply.token;
    motor->stop(motor->context, &command, &reply);
    assert(reply.status == PROTOCOL_OK && reply.token == token);
    BspCanFrame frame = {.identifier = 17};
    assert(Protocol_QueueReceivedFrame(&apps[0].endpoint, &frame));
    assert(queue_count(&apps[0].endpoint.rx_queue) == 1 &&
           queue_count(&apps[1].endpoint.rx_queue) == 0);
}
int main(void)
{
    run(false);
    Device reference[2];
    memcpy(reference, devices, sizeof(reference));
    run(true);
    assert(memcmp(reference, devices, sizeof(devices)) == 0);
    stop_between_snapshot_and_commit();
    puts("PASS independent and nested startup, fast PWM, calibration records, STOP epoch/PI and "
         "CAN queues");
    return 0;
}
