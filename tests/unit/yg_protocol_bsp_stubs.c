/* 协议单元测试的默认 CAN 替身；链路夹具可用强符号覆盖，不访问硬件。 */
#include "bsp_can.h"
#include "bsp_system.h"

__attribute__((weak)) bool BspCan_Receive(BspCanFrame *frame)
{
    (void)frame;
    return false;
}

__attribute__((weak)) bool BspCan_TrySend(const BspCanFrame *frame)
{
    (void)frame;
    return false;
}

__attribute__((weak)) void BspCan_Start(void)
{
}

__attribute__((weak)) bool BspCan_IsTransmitBusy(void)
{
    return false;
}

__attribute__((weak)) bool BspCan_IsTransmitComplete(void)
{
    return false;
}

#include "yg_protocol.h"
static void unsupported_motor(const ProtocolCommand *request, ProtocolReply *reply)
{
    (void)request;
    *reply = (ProtocolReply){.status = PROTOCOL_UNSUPPORTED};
}
static const ProtocolMotorStatus s_default_motor_status;
__attribute__((weak)) const ProtocolMotor Protocol_Motor = {unsupported_motor,
                                                            unsupported_motor,
                                                            unsupported_motor,
                                                            unsupported_motor,
                                                            &s_default_motor_status};
static void
unsupported_job(void *context, const ProtocolJobRequest *request, ProtocolJobReply *reply)
{
    (void)context;
    (void)request;
    *reply = (ProtocolJobReply){.status = PROTOCOL_UNSUPPORTED};
}
__attribute__((weak)) const ProtocolJobs Protocol_Jobs = {NULL, unsupported_job};
__attribute__((weak)) const BootEntryService Protocol_Boot = {0};

static const ProtocolDeviceStatus s_default_device_status = {.role = PROTOCOL_DEVICE_ROLE_APP};
__attribute__((weak)) const ProtocolDeviceStatus *const Protocol_DeviceStatusSource =
    &s_default_device_status;
