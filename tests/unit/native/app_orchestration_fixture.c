/* 记录生产前台入口的事件顺序：普通 APP 仅读取协议时钟，LoaderApp 交接前再读取一次。 */
#include "app.h"
#include "boot_hw.h"
#include "bsp_system.h"
#include "bsp_pwm.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static uint32_t trace = 2166136261U, now_ms, failure_stage;
static unsigned time_reads, disables, enters, confirms, depth;
static bool stopped, lock_allowed = true, healthy = true, link_ready = true;
static uint32_t progress = 1U;
static AppContext app;
static const MotorAxisHardware hardware = {.power.disable = BspPwm_Disable};
#define boot_service app.endpoint.loader
static void event(uint32_t kind, uint32_t value)
{
    trace = (trace ^ kind) * 16777619U;
    trace = (trace ^ value) * 16777619U;
}
static void bytes(uint32_t kind, const uint8_t *data, unsigned count)
{
    for (unsigned i = 0; i < count; ++i)
        event(kind, data[i]);
}
ParameterStatus
MotorAxis_ReadParameter(MotorAxis *axis, uint8_t id, bool saved, ParameterValue *value)
{
    (void)axis;
    (void)id;
    (void)saved;
    (void)value;
    return PARAMETER_UNSUPPORTED;
}
ParameterStatus MotorAxis_WriteParameter(MotorAxis *axis, const ParameterWriteRequest *request)
{
    (void)axis;
    (void)request;
    return PARAMETER_UNSUPPORTED;
}
ParameterStatus MotorAxis_SaveParameters(MotorAxis *axis, uint32_t revision)
{
    (void)axis;
    (void)revision;
    return PARAMETER_UNSUPPORTED;
}
ParameterStatus MotorAxis_RestoreControlDefaults(MotorAxis *axis, uint32_t revision)
{
    (void)axis;
    (void)revision;
    return PARAMETER_UNSUPPORTED;
}
void MotorAxis_Init(MotorAxis *instance)
{
    instance->hardware = &hardware;
    instance->configuration.device.node_id = 7U;
    event(1, 0);
}
bool Protocol_Init(ProtocolEndpoint *instance, uint8_t node)
{
    (void)instance;
    assert(node == 7U);
    instance->node_id = node;
    event(3, node);
    return failure_stage != 1;
}
void AppProtocol_Init(AppProtocol *protocol, ProtocolEndpoint *endpoint, MotorStateMachine *machine)
{
    assert(protocol == &app.protocol && endpoint == &app.endpoint &&
           machine == &app.axis.state_machine);
}
bool Protocol_Start(ProtocolEndpoint *instance)
{
    (void)instance;
    event(5, 0);
    return failure_stage != 5;
}
void BspPwm_Disable(void)
{
    event(6, ++disables);
}
uint32_t BspSystem_TimeMs(void)
{
    ++time_reads;
    event(9, now_ms);
    return now_ms;
}
uint32_t BspSystem_EnterCritical(void)
{
    event(10, depth);
    return depth++;
}
void BspSystem_ExitCritical(uint32_t saved)
{
    assert(depth == saved + 1);
    depth = saved;
    event(11, saved);
}
void BspSystem_MemoryBarrier(void)
{
    event(12, 0U);
}
BspResult BspSystem_ReadIdentity(uint8_t *record)
{
    event(13, 0);
    memset(record, 0, 64);
    return BSP_OK;
}
BspResult BspSystem_ReadUid(uint8_t *uid)
{
    event(14, 0);
    memset(uid, 0x35, 12);
    return failure_stage == 3 ? BSP_IO_ERROR : BSP_OK;
}
bool Protocol_SetInfo(ProtocolEndpoint *instance,
                      const ProtocolInfo *info,
                      const ProtocolCaps *caps)
{
    (void)instance;
    event(15, info->product_id);
    event(16, info->hardware_revision);
    event(17, info->application_version);
    event(18, info->boot_version);
    event(19, info->boot_api);
    bytes(20, info->uid, 12);
    event(21, caps->features);
    event(22, caps->supported_modes);
    event(23, caps->max_full_frame);
    event(24, caps->max_block_data);
    assert(caps->features == 0x1fd00U && caps->supported_modes == 0x64eU);
    assert(caps->default_watchdog_ms == 500U && caps->max_watchdog_ms == 500U);
    return failure_stage != 4;
}
bool Protocol_SetInfoPage(ProtocolEndpoint *instance, uint16_t page, const uint8_t *payload)
{
    (void)instance;
    bytes(25 + page, payload, 32);
    return true;
}
uint32_t BootHw_ImageCrc32(void)
{
    event(28, 0);
    return 0x12345678U;
}
bool BootHw_IsConfirmed(void)
{
    event(29, 0);
    return false;
}
bool BootHw_IsAvailable(void)
{
    event(30, 0);
    return true;
}
bool MotorStateMachine_ReadBootHealth(MotorStateMachine *instance, uint32_t *value)
{
    (void)instance;
    *value = progress;
    event(32, progress);
    return healthy;
}
bool Protocol_IsReady(ProtocolEndpoint *instance)
{
    (void)instance;
    event(33, link_ready);
    return link_ready;
}
bool MotorStateMachine_ArePhasesDisabled(const MotorStateMachine *machine)
{
    (void)machine;
    event(34, stopped);
    return stopped;
}
bool MotorStateMachine_IsStopped(const MotorStateMachine *instance)
{
    (void)instance;
    event(35, stopped);
    return stopped;
}
bool MotorStateMachine_LockStartsAndRequestStop(MotorStateMachine *instance)
{
    (void)instance;
    event(36, lock_allowed);
    return lock_allowed;
}
bool BootHw_EnterLoader(uint8_t node, uint32_t started_ms, uint32_t timeout_ms)
{
    assert(depth == 0 && stopped && node == app.axis.configuration.device.node_id);
    assert(now_ms - started_ms < timeout_ms);
    event(37, node);
    ++enters;
    return false;
}
bool BootHw_Confirm(uint32_t version, uint32_t crc)
{
    event(38, version);
    event(39, crc);
    ++confirms;
    return true;
}
void BootHw_Watchdog(bool valid)
{
    event(40, valid);
}
void MotorAxis_ProcessPositionPlan(MotorAxis *instance)
{
    (void)instance;
}
void MotorCalibration_ProcessPublication(MotorCalibration *cal)
{
    assert(cal == &app.axis.calibration);
    event(41, 0);
}
void Protocol_Process(ProtocolEndpoint *instance, uint32_t time)
{
    (void)instance;
    event(43, time);
    ++now_ms;
}
void MotorAxis_SampleFast(MotorAxis *instance)
{
    (void)instance;
}
void MotorAxis_ControlFast(MotorAxis *instance)
{
    ++instance->state_machine.fast_sample_ticks;
    event(44, 0);
}
void MotorAxis_SupervisorTick(MotorAxis *instance)
{
    ++instance->state_machine.supervisor_ticks;
    event(45, 0);
}
void Protocol_RxInterrupt(ProtocolEndpoint *instance)
{
    (void)instance;
    event(46, 0);
}

static void fast_sample(void)
{
    MotorAxis_SampleFast(&app.axis);
    MotorAxis_ControlFast(&app.axis);
}
static void check_startup(void)
{
    for (failure_stage = 0; failure_stage <= 5; ++failure_stage)
    {
        unsigned before = disables;
        app = (AppContext){0};
        App_Init(&app);
        bool init_ok = failure_stage == 0 || failure_stage == 2 || failure_stage == 5 ||
                       (!YG_APP_LOADER_IMAGE && failure_stage == 6);
        assert(app.communication_ready == init_ok);
        assert(disables == before + !init_ok);
        App_Start(&app);
        assert(app.communication_ready == (init_ok && failure_stage != 5));
    }
    failure_stage = 0;
    app = (AppContext){0};
    App_Init(&app);
    App_Start(&app);
}
static void process_at(uint32_t time)
{
    now_ms = time;
    App_Process(&app);
}
static void check_foreground(void)
{
    fast_sample();
    MotorAxis_SupervisorTick(&app.axis);
    unsigned before = time_reads;
    process_at(10U);
    assert(time_reads - before == (YG_APP_LOADER_IMAGE ? 2U : 1U));
    Protocol_RxInterrupt(&app.endpoint);
}
#if YG_APP_LOADER_IMAGE
static void request(const BootEntryRequest *value, BootEntryResult expected, uint64_t ticket)
{
    BootEntryReply reply = {0};
    boot_service.request(boot_service.context, value, &reply);
    assert(reply.result == expected && reply.ack_ticket == ticket);
    event(47, reply.result);
    event(48, (uint32_t)reply.ack_ticket);
}
static void reset_entry(void)
{
    app.loader.request = (BootEntryRequest){0};
    app.loader.result = BOOT_ENTRY_UNAVAILABLE;
    stopped = false;
    lock_allowed = true;
    depth = 0;
    now_ms = 1000;
}
static void check_loader(void)
{
    const BootEntryRequest valid = {0x123456789abcdefULL, 7, 8};
    BootEntryRequest invalid = valid;
    reset_entry();
    boot_service.request(boot_service.context, &valid, NULL);
    request(NULL, BOOT_ENTRY_UNAVAILABLE, 0);
    invalid.boot_epoch = 0;
    request(&invalid, BOOT_ENTRY_UNAVAILABLE, 0);
    invalid = valid;
    invalid.source_node = UINT8_MAX;
    request(&invalid, BOOT_ENTRY_UNAVAILABLE, 0);
    app.loader.supported = false;
    request(&valid, BOOT_ENTRY_UNAVAILABLE, 0);
    app.loader.supported = true;
    request(&valid, BOOT_ENTRY_PENDING, 0);
    for (unsigned field = 0; field < 3; ++field)
    {
        invalid = valid;
        if (field == 0)
            ++invalid.boot_epoch;
        if (field == 1)
            ++invalid.sequence;
        if (field == 2)
            ++invalid.source_node;
        request(&invalid, BOOT_ENTRY_DENIED, 0);
    }
    stopped = true;
    request(&valid, BOOT_ENTRY_READY, valid.boot_epoch);
    boot_service.tx_complete(boot_service.context, valid.boot_epoch + 1, true);
    assert(enters == 0);
    boot_service.tx_complete(boot_service.context, valid.boot_epoch, false);
    assert(enters == 0);
    boot_service.tx_complete(boot_service.context, valid.boot_epoch, true);
    assert(enters == 0);
    request(&valid, BOOT_ENTRY_FAILED, 0);
    reset_entry();
    request(&valid, BOOT_ENTRY_PENDING, 0);
    stopped = true;
    request(&valid, BOOT_ENTRY_READY, valid.boot_epoch);
    app.axis.configuration.device.node_id = 9U;
    boot_service.tx_complete(boot_service.context, valid.boot_epoch, true);
    assert(enters == 1 && depth == 0);
    boot_service.tx_complete(boot_service.context, valid.boot_epoch, true);
    assert(enters == 1);
    reset_entry();
    lock_allowed = false;
    request(&valid, BOOT_ENTRY_DENIED, 0);
    stopped = true;
    request(&valid, BOOT_ENTRY_DENIED, 0);
    reset_entry();
    now_ms = UINT32_MAX - 499U;
    request(&valid, BOOT_ENTRY_PENDING, 0);
    stopped = true;
    now_ms = 499;
    request(&valid, BOOT_ENTRY_READY, valid.boot_epoch);
    now_ms = 500;
    boot_service.tx_complete(boot_service.context, valid.boot_epoch, true);
    assert(enters == 1);
    request(&valid, BOOT_ENTRY_FAILED, 0);
    reset_entry();
    request(&valid, BOOT_ENTRY_PENDING, 0);
    now_ms = 2000;
    request(&valid, BOOT_ENTRY_FAILED, 0);
    reset_entry();
    request(&valid, BOOT_ENTRY_PENDING, 0);
    stopped = true;
    request(&valid, BOOT_ENTRY_READY, valid.boot_epoch);
    stopped = false;
    boot_service.tx_complete(boot_service.context, valid.boot_epoch, true);
    assert(enters == 1);
    app.loader.request = (BootEntryRequest){0};
    app.loader.result = BOOT_ENTRY_UNAVAILABLE;
    stopped = true;
    app.loader.healthy_intervals = 0;
    app.loader.health_checked_ms = 0;
    for (unsigned i = 1; i <= 12; ++i)
    {
        ++progress;
        process_at(i * 100);
        assert(boot_service.confirm(boot_service.context, 1, 2) == (i >= 11));
    }
    assert(app.loader.healthy_intervals == 11 && confirms == 2);
    healthy = false;
    process_at(1300);
    assert(app.loader.healthy_intervals == 0 && !boot_service.confirm(boot_service.context, 1, 2));
}
#endif
int main(void)
{
    check_startup();
    check_foreground();
#if YG_APP_LOADER_IMAGE
    check_loader();
#endif
    printf("PASS APP trace=%08x disables=%u enters=%u confirms=%u\n",
           trace,
           disables,
           enters,
           confirms);
    return 0;
}
