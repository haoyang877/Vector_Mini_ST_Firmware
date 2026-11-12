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

__attribute__((weak)) bool BspCan_ConfigureBitrate(uint8_t profile)
{
    return profile == 0U;
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
