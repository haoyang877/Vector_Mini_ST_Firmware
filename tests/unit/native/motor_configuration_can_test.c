/* 型号配置事务：真实参数构造/CRC，替代 Flash 和复位，不访问硬件。 */
#include "app_motor_configuration.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static MotorAxis axis;
static AppMotorConfiguration service;
static ProtocolEndpoint endpoint;
static MotorAxisRecord flash_record;
static bool off, write_failed, corrupt_read, busy, complete;
static unsigned writes, resets;
static uint32_t now_ms;

static bool phases_disabled(void)
{
    return off;
}
static BspResult write_record(const void *record, size_t bytes)
{
    assert(off && axis.state_machine.starts_locked && bytes == sizeof(flash_record));
    ++writes;
    if (write_failed)
    {
        return BSP_IO_ERROR;
    }
    memcpy(&flash_record, record, bytes);
    return BSP_OK;
}
static BspResult read_record(void *record, size_t bytes)
{
    memcpy(record, &flash_record, bytes);
    if (corrupt_read)
    {
        ((uint8_t *)record)[0] ^= 1U;
    }
    return BSP_OK;
}
static const MotorAxisHardware hardware = {.power = {.phases_disabled = phases_disabled},
                                           .write_record = write_record,
                                           .read_record = read_record};
uint32_t BspSystem_EnterCritical(void)
{
    return 0U;
}
void BspSystem_ExitCritical(uint32_t saved)
{
    (void)saved;
}
BspResult BspSystem_ReadUid(uint8_t uid[BSP_SYSTEM_UID_BYTES])
{
    memset(uid, 0x11, BSP_SYSTEM_UID_BYTES);
    return BSP_OK;
}
uint32_t BspSystem_TimeMs(void)
{
    return now_ms;
}
void BspSystem_Reset(void)
{
    ++resets;
}
bool BspCan_IsTransmitBusy(void)
{
    return busy;
}
bool BspCan_IsTransmitComplete(void)
{
    return complete;
}
bool MotorStateMachine_ArePhasesDisabled(const MotorStateMachine *machine)
{
    (void)machine;
    return off;
}
bool MotorStateMachine_LockStartsAndRequestStop(MotorStateMachine *machine)
{
    machine->starts_locked = true;
    return true;
}
bool MotorStateMachine_ReadStatus(const MotorStateMachine *machine, MotorStatus *status)
{
    (void)machine;
    *status = (MotorStatus){.measurements_valid = true, .speed_rad_s = 0.0f};
    return true;
}

static void setup(void)
{
    memset(&axis, 0, sizeof(axis));
    memset(&endpoint, 0, sizeof(endpoint));
    memset(&service, 0, sizeof(service));
    axis.hardware = &hardware;
    assert(Param_CreateAxis(5U, -2.0f, 2.0f, 1.0f, NULL, &axis.configuration));
    axis.configuration.configuration_confirmed = true;
    axis.configuration.device.encoder.calib_flag = ENC_CALIB_ALL;
    Param_Upload(&axis.record.motor, &axis.configuration);
    memset(&axis.record.output, 0x5a, sizeof(axis.record.output));
    flash_record = axis.record;
    axis.parameters.stored_record_crc = flash_record.motor.record_crc32;
    axis.state_machine.state = MOTOR_STATE_IDLE;
    off = complete = true;
    write_failed = corrupt_read = busy = false;
    writes = resets = 0;
    now_ms = 0;
    AppMotorConfiguration_Init(&service, &axis, &endpoint);
}
static ProtocolMotorConfigurationReply call(ProtocolMotorConfigurationRequest request)
{
    ProtocolMotorConfigurationReply reply = {0};
    endpoint.configuration.handler(endpoint.configuration.context, &request, &reply);
    return reply;
}
static ProtocolMotorConfigurationRequest uid_request(uint8_t operation, uint32_t revision)
{
    ProtocolMotorConfigurationRequest request = {.operation = operation, .revision = revision};
    memset(request.uid, 0x11, sizeof(request.uid));
    return request;
}
static ProtocolMotorConfigurationRequest yq_request(void)
{
    return (ProtocolMotorConfigurationRequest){
        .operation = 1U,
        .version = 1U,
        .model = 3U,
        .node = 6U,
        .flags = 1U,
        .revision = axis.parameters.stored_record_crc,
        .speed_mrad_s = 1000,
        .thermal = {3000, 7000, 65000, 800000, 700000, 1000000, 500000}};
}
int main(void)
{
    setup();
    ProtocolMotorConfigurationRequest request = yq_request();
    /* 模拟活动坐标 CRC 读取：工作区可重新导出，Flash 修订应保持稳定。 */
    axis.record.motor.encoder_mechanical_zero_q15 ^= 1U;
    ProtocolMotorConfigurationReply active = call((ProtocolMotorConfigurationRequest){0});
    assert(active.revision == flash_record.motor.record_crc32);
    assert(active.status == PROTOCOL_OK && active.model == 1U && active.node == 5U &&
           (active.flags & 6U) == 6U);
    request.revision ^= 1U;
    assert(call(request).status == PROTOCOL_DENIED && writes == 0 &&
           !axis.state_machine.starts_locked);
    request = yq_request();
    request.model = 1U;
    assert(call(request).status == PROTOCOL_INVALID_ARGUMENT && writes == 0);
    request = yq_request();
    memset(request.thermal, 0, sizeof(request.thermal));
    assert(call(request).status == PROTOCOL_OUT_OF_RANGE && writes == 0);
    request = yq_request();
    axis.state_machine.state = MOTOR_STATE_RUNNING;
    assert(call(request).status == PROTOCOL_DENIED && writes == 0);
    axis.state_machine.state = MOTOR_STATE_IDLE;
    off = false;
    assert(call(request).status == PROTOCOL_DENIED && writes == 0);
    off = true;
    axis.state_machine.calibration_handoff = MOTOR_CALIBRATION_AWAITING_PUBLICATION;
    assert(call(request).status == PROTOCOL_DENIED && writes == 0);
    axis.state_machine.calibration_handoff = MOTOR_CALIBRATION_NO_HANDOFF;
    axis.motor.fault_code = MOTOR_FAULT_OVERCURRENT;
    assert(call(request).status == PROTOCOL_DENIED && writes == 0);
    axis.motor.fault_code = MOTOR_FAULT_NONE;
    ProtocolMotorConfigurationReply first = call(request);
    assert(first.status == PROTOCOL_OK && writes == 0U);
    ProtocolMotorConfigurationRequest invalid = request;
    invalid.thermal[0] = -1;
    assert(call(invalid).status == PROTOCOL_OUT_OF_RANGE);
    assert(call(uid_request(2U, first.revision)).status == PROTOCOL_DENIED && writes == 0U);
    ProtocolMotorConfigurationReply second = call(request);
    assert(second.status == PROTOCOL_OK && second.revision != first.revision);
    assert(call(uid_request(2U, first.revision)).status == PROTOCOL_DENIED);
    flash_record.motor.speed_limit_rad_s += 0.1f;
    flash_record.motor.record_crc32 = Param_RecordCrc(&flash_record.motor);
    axis.parameters.stored_record_crc = flash_record.motor.record_crc32;
    assert(call(uid_request(2U, second.revision)).status == PROTOCOL_DENIED && writes == 0U);
    setup();
    request = yq_request();
    ProtocolMotorConfigurationReply prepared = call(request);
    assert(prepared.status == PROTOCOL_OK && (prepared.flags & 32U) != 0U && writes == 0U &&
           !axis.state_machine.starts_locked);
    assert(call(request).revision == prepared.revision && writes == 0U);
    ProtocolMotorConfigurationRequest commit = uid_request(2U, prepared.revision);
    commit.uid[0] ^= 1U;
    assert(call(commit).status == PROTOCOL_DENIED && writes == 0U);
    commit.uid[0] ^= 1U;
    ProtocolMotorConfigurationReply saved = call(commit);
    assert(call(commit).status == PROTOCOL_OK && writes == 1U);
    assert(saved.status == PROTOCOL_OK && saved.node == 6U && saved.model == 3U &&
           saved.flags == 9U && writes == 1U);
    assert(axis.configuration.device.node_id == 5U &&
           axis.configuration.device.encoder.calib_flag == ENC_CALIB_ALL);
    assert(flash_record.motor.node_id == 6.0f && flash_record.motor.encoder_calib_flag == 0U &&
           flash_record.motor.record_crc32 == Param_RecordCrc(&flash_record.motor));
    assert(axis.configuration.model.motor_pole_pairs == HT8115_POLE_PAIRS);
    assert(flash_record.motor.motor_pole_pairs == YQ5708_POLE_PAIRS &&
           flash_record.motor.motor_phase_resistance_ohm == YQ5708_PHASE_RESISTANCE_OHM &&
           flash_record.motor.iq_kp == AT32_YQ5708_CURRENT_KP);
    const uint8_t *output = (const uint8_t *)&flash_record.output;
    for (unsigned i = 0; i < sizeof(flash_record.output); ++i)
    {
        assert(output[i] == 0U);
    }
    assert(call(request).status == PROTOCOL_OK && writes == 1U);
    request.speed_mrad_s = 1500;
    assert(call(request).status == PROTOCOL_BUSY && writes == 1U);
    active = call((ProtocolMotorConfigurationRequest){0});
    assert(active.node == 5U && (active.flags & 8U) != 0U);
    saved = call((ProtocolMotorConfigurationRequest){.page = 1U});
    assert(saved.node == 6U && (saved.flags & 6U) == 0U);
    assert(call(uid_request(3U, saved.revision ^ 1U)).status == PROTOCOL_DENIED);
    assert(call(uid_request(3U, saved.revision)).status == PROTOCOL_ACCEPTED);
    AppMotorConfiguration_Process(&service, 99U);
    assert(resets == 0U);
    busy = true;
    AppMotorConfiguration_Process(&service, 100U);
    assert(resets == 0U);
    busy = false;
    complete = false;
    AppMotorConfiguration_Process(&service, 100U);
    assert(resets == 0U);
    complete = true;
    endpoint.tx_queue.produced = 1U;
    AppMotorConfiguration_Process(&service, 100U);
    assert(resets == 0U);
    endpoint.tx_queue.consumed = 1U;
    off = false;
    AppMotorConfiguration_Process(&service, 100U);
    assert(resets == 0U);
    off = true;
    AppMotorConfiguration_Process(&service, 100U);
    assert(resets == 1U);
    AppMotorConfiguration_Process(&service, 101U);
    assert(resets == 1U);
    for (unsigned error = 0; error < 2U; ++error)
    {
        setup();
        request = yq_request();
        write_failed = error == 0U;
        corrupt_read = error == 1U;
        prepared = call(request);
        assert(prepared.status == PROTOCOL_OK && writes == 0U);
        assert(call(uid_request(2U, prepared.revision)).status == PROTOCOL_FAILED &&
               axis.state_machine.starts_locked && !service.saved);
        active = call((ProtocolMotorConfigurationRequest){0});
        assert((active.flags & 16U) != 0U);
        assert(call(request).status == PROTOCOL_DENIED);
        assert(call(uid_request(3U, 0U)).status == PROTOCOL_DENIED && resets == 0U);
    }
    puts("PASS motor configuration: revision, pairing, stop, faults, persistence, idempotency, "
         "calibration invalidation, deferred reset, flash failures");
    return 0;
}
