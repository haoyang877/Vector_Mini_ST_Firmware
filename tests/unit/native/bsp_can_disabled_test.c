/* 禁用CAN仍有同一公开入口，拒绝发送且保持调用方快照，绝不伪造已发送。 */
#include <assert.h>
#include <string.h>
#include "bsp_config.h"
#include "bsp_can.h"
int main(void)
{
    BspCanFrame frame, before;
    memset(&frame, 0xA5, sizeof(frame));
    before = frame;
    BspCanCompletion completion = {UINT64_C(0x123456789ABCDEF0), BSP_CAN_TX_OK};
    assert(BSP_SUPPORTED_CAN && !BSP_ENABLED_CAN);
    assert(BspCan_Start() == BSP_NOT_SUPPORTED);
    assert(BspCan_TrySend(&frame) == BSP_NOT_SUPPORTED);
    assert(BspCan_TrySendTracked(&frame, completion.ticket) == BSP_NOT_SUPPORTED);
    assert(BspCan_Receive(&frame) == BSP_NOT_SUPPORTED &&
           memcmp(&frame, &before, sizeof(frame)) == 0);
    assert(BspCan_TakeCompletion(&completion) == BSP_NOT_SUPPORTED);
    assert(completion.ticket == UINT64_C(0x123456789ABCDEF0) && completion.result == BSP_CAN_TX_OK);
    assert(BspCan_GetState() == BSP_CAN_DISABLED && BspCan_Recover() == BSP_NOT_SUPPORTED);
    return 0;
}
