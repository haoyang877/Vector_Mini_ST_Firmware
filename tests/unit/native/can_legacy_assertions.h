/* 旧反例保持bool断言，实际调用全部指向新接口；新增结果语义另有显式断言。 */
typedef struct
{
    uint64_t ticket;
    bool success;
} BspCanTxCompletion;
static bool BspCan_Init(void)
{
    return Can_Init();
}
static bool BspCan_TrySendFrame(const BspCanFrame *f)
{
    return BspCan_TrySend(f) == BSP_OK;
}
static bool BspCan_TrySendTrackedFrame(const BspCanFrame *f, uint64_t t)
{
    return BspCan_TrySendTracked(f, t) == BSP_OK;
}
static bool BspCan_TakeTxCompletion(BspCanTxCompletion *out)
{
    BspCanCompletion value;
    if (out == NULL || BspCan_TakeCompletion(&value) != BSP_OK)
    {
        return false;
    }
    out->ticket = value.ticket;
    out->success = value.result == BSP_CAN_TX_OK;
    return true;
}
static bool BspCan_IsHealthy(void)
{
    return BspCan_GetState() == BSP_CAN_READY;
}
static bool BspCan_ServiceBusOff(void)
{
    return BspCan_Recover() == BSP_OK;
}
#define BspCan_Receive(...) (BspCan_Receive(__VA_ARGS__) == BSP_OK)
#define BspCan_OnReceiveInterrupt(handler) Can_OnReceiveInterrupt(handler, 0U)
