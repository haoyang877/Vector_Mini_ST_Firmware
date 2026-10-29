/* 使用真实通信端口和窄 HAL 状态桩复现 bus-off；不访问寄存器或硬件。 */
#include "bsp_can.h"
#include "cmsis_preemption_fake.h"
#include "bsp_system.h"
#include <stdio.h>
#include <string.h>
#include <setjmp.h>
#include <assert.h>

#define HAL_OK 0
#define HAL_ERROR 1
#define HAL_FDCAN_STATE_READY 1U
#define HAL_FDCAN_STATE_BUSY 2U
#define HAL_FDCAN_STATE_ERROR 3U
#define FDCAN_CCCR_INIT 1U
#define FDCAN_PSR_BO (1U << 7U)
#define FDCAN_PSR_EP (1U << 5U)
#define FDCAN_PSR_EW (1U << 6U)
#define FDCAN_STANDARD_ID 0U
#define FDCAN_EXTENDED_ID (1U << 30U)
#define FDCAN_DATA_FRAME 0U
#define FDCAN_FD_CAN (1U << 21U)
#define FDCAN_BRS_ON (1U << 20U)
#define FDCAN_BRS_OFF 0U
#define FDCAN_NO_TX_EVENTS 0U
#define FDCAN_RX_FIFO0 0U
#define FDCAN_REJECT 0U
#define FDCAN_ACCEPT_IN_RX_FIFO0 1U
#define FDCAN_FILTER_REMOTE 1U
#define FDCAN_IT_RX_FIFO0_NEW_MESSAGE 1U
#define FDCAN_IT_RX_FIFO0_MESSAGE_LOST 2U
#define FDCAN_IR_RF0L 2U
#define FDCAN_FLAG_RX_FIFO0_MESSAGE_LOST 2U
#define __HAL_FDCAN_CLEAR_FLAG(h, f) ((h)->Instance->IR &= ~(f))
#define FDCAN_CLASSIC_CAN 0U
#define FDCAN_REMOTE_FRAME 1U

typedef struct
{
    uint32_t PSR, CCCR, TXBRP, TXBTO, TXBCF, IR;
} FakeRegisters;
typedef struct
{
    FakeRegisters *Instance;
    uint32_t State, LatestTxFifoQRequest;
} FDCAN_HandleTypeDef;
typedef struct
{
    uint32_t DataLength, Identifier, IdType, RxFrameType, FDFormat, BitRateSwitch;
} FDCAN_RxHeaderTypeDef;
typedef struct
{
    uint32_t DataLength, Identifier, IdType, TxFrameType, FDFormat, BitRateSwitch,
        TxEventFifoControl;
} FDCAN_TxHeaderTypeDef;

static FakeRegisters registers;
FDCAN_HandleTypeDef hfdcan1;
static unsigned stops, starts, errors, sends, aborts, failures;
static bool stop_fails, start_fails;
static char lifecycle[32];
static unsigned lifecycle_size;
static uint32_t now_ms = 10U;
static bool filter_fails, notification_fails, send_fails, abort_delayed, rx_ready, expect_fatal;
static unsigned notifications, rx_reads, callback_count;
static FDCAN_RxHeaderTypeDef rx_header;
static FDCAN_TxHeaderTypeDef sent_header;
static uint8_t rx_data[64], sent_data[64];
static jmp_buf fatal_exit;

uint32_t BspSystem_EnterCritical(void)
{
    return 0U;
}
void BspSystem_ExitCritical(uint32_t key)
{
    (void)key;
}
uint32_t HAL_GetTick(void)
{
    return now_ms;
}
void Error_Handler(void)
{
    ++errors;
    if (expect_fatal)
    {
        longjmp(fatal_exit, 1);
    }
}

/* 与本仓库 HAL 一致：Start 只接受 READY；Stop 接受 BUSY 并发布 READY。 */
int HAL_FDCAN_Start(FDCAN_HandleTypeDef *handle)
{
    ++starts;
    lifecycle[lifecycle_size++] = 'S';
    if (start_fails || handle->State != HAL_FDCAN_STATE_READY)
    {
        return HAL_ERROR;
    }
    handle->State = HAL_FDCAN_STATE_BUSY;
    handle->Instance->CCCR &= ~FDCAN_CCCR_INIT;
    return HAL_OK;
}
int HAL_FDCAN_Stop(FDCAN_HandleTypeDef *handle)
{
    ++stops;
    lifecycle[lifecycle_size++] = 'T';
    if (stop_fails)
    {
        handle->State = HAL_FDCAN_STATE_ERROR;
        return HAL_ERROR;
    }
    if (handle->State != HAL_FDCAN_STATE_BUSY)
    {
        return HAL_ERROR;
    }
    handle->Instance->CCCR |= FDCAN_CCCR_INIT;
    handle->LatestTxFifoQRequest = 0U;
    handle->State = HAL_FDCAN_STATE_READY;
    return HAL_OK;
}
int HAL_FDCAN_AbortTxRequest(FDCAN_HandleTypeDef *handle, uint32_t mask)
{
    ++aborts;
    if (!abort_delayed)
    {
        handle->Instance->TXBRP &= ~mask;
    }
    return HAL_OK;
}
uint32_t HAL_FDCAN_GetLatestTxFifoQRequestBuffer(FDCAN_HandleTypeDef *handle)
{
    return handle->LatestTxFifoQRequest;
}
int HAL_FDCAN_AddMessageToTxFifoQ(FDCAN_HandleTypeDef *handle,
                                  const FDCAN_TxHeaderTypeDef *header,
                                  const uint8_t *data)
{
    ++sends;
    if (send_fails || handle->State != HAL_FDCAN_STATE_BUSY)
    {
        return HAL_ERROR;
    }
    handle->LatestTxFifoQRequest = 1U;
    handle->Instance->TXBRP = 1U;
    handle->Instance->TXBTO = 0U;
    handle->Instance->TXBCF = 0U;
    sent_header = *header;
    const uint8_t lengths[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};
    memcpy(sent_data, data, lengths[header->DataLength]);
    return HAL_OK;
}
int HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef *handle,
                           uint32_t fifo,
                           FDCAN_RxHeaderTypeDef *header,
                           uint8_t *data)
{
    (void)handle;
    (void)fifo;
    ++rx_reads;
    if (!rx_ready)
    {
        return HAL_ERROR;
    }
    rx_ready = false;
    *header = rx_header;
    memcpy(data, rx_data, sizeof(rx_data));
    return HAL_OK;
}

void MX_FDCAN1_Init(void)
{
    lifecycle[lifecycle_size++] = 'I';
    hfdcan1.Instance = &registers;
    hfdcan1.State = HAL_FDCAN_STATE_READY;
}
int HAL_FDCAN_ConfigGlobalFilter(FDCAN_HandleTypeDef *handle,
                                 unsigned standard,
                                 unsigned extended,
                                 unsigned remote_standard,
                                 unsigned remote_extended)
{
    (void)handle;
    lifecycle[lifecycle_size++] = 'F';
    return !filter_fails && standard == FDCAN_REJECT && extended == FDCAN_ACCEPT_IN_RX_FIFO0 &&
                   remote_standard == FDCAN_FILTER_REMOTE && remote_extended == FDCAN_FILTER_REMOTE
               ? HAL_OK
               : HAL_ERROR;
}
int HAL_FDCAN_ActivateNotification(FDCAN_HandleTypeDef *handle,
                                   unsigned notification,
                                   unsigned buffers)
{
    (void)handle;
    ++notifications;
    lifecycle[lifecycle_size++] = 'N';
    return !notification_fails &&
                   notification ==
                       (FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO0_MESSAGE_LOST) &&
                   buffers == 0U
               ? HAL_OK
               : HAL_ERROR;
}
uint32_t HAL_FDCAN_GetRxFifoFillLevel(FDCAN_HandleTypeDef *h, uint32_t fifo)
{
    (void)h;
    (void)fifo;
    return rx_ready ? 1U : 0U;
}
#include "can.c"
#include "../../../firmware/platform/bsp/bsp_can.c"
#include "../../../firmware/platform/bsp/can_frame_codec.c"
#include "can_legacy_assertions.h"

static void check(bool value, const char *message)
{
    if (!value)
    {
        ++failures;
        printf("FAIL: %s\n", message);
    }
}
static void setup(uint32_t state, bool unavailable)
{
    memset(&registers, 0, sizeof(registers));
    hfdcan1 = (FDCAN_HandleTypeDef){&registers, state, 0U};
    if (unavailable)
    {
        registers.CCCR = FDCAN_CCCR_INIT;
    }
    stops = starts = errors = sends = aborts = lifecycle_size = 0U;
    stop_fails = start_fails = false;
    filter_fails = notification_fails = send_fails = abort_delayed = rx_ready = expect_fatal =
        false;
    notifications = rx_reads = callback_count = 0U;
    now_ms = 10U;
    memset(&s_tx, 0, sizeof(s_tx));
    s_initialized = true;
    s_started = true;
    s_recovery_seen = false;
    s_rx_overflow = false;
    memset(lifecycle, 0, sizeof(lifecycle));
}
/** @brief 验证真实启动的顺序和各失败位置不会继续开放后续功能。 */
static void test_startup(void)
{
    setup(HAL_FDCAN_STATE_READY, true);
    s_initialized = false;
    assert(BspCan_Init());
    s_started = false;
    BspCan_Start();
    assert(strcmp(lifecycle, "IFSN") == 0 && notifications == 1U);
    for (unsigned stage = 0U; stage < 3U; ++stage)
    {
        setup(HAL_FDCAN_STATE_READY, true);
        filter_fails = stage == 0U;
        start_fails = stage == 1U;
        notification_fails = stage == 2U;
        s_started = false;
        expect_fatal = true;
        if (setjmp(fatal_exit) == 0)
        {
            BspCan_Start();
            assert(!"startup failure must enter the original fatal path");
        }
        assert(errors == 1U);
        assert(strcmp(lifecycle, stage == 0U ? "F" : stage == 1U ? "FS" : "FSN") == 0);
    }
}
/** @brief 仅用于观测BSP收到一次有界接收事件。 */
static void on_receive(void)
{
    ++callback_count;
}
/** @brief 穷举物理长度和元数据，检查不合法输入不提交、收包失败不修改输出。 */
static void test_frames(void)
{
    const uint8_t lengths[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};
    for (unsigned length = 0U; length < 256U; ++length)
    {
        for (unsigned flags = 0U; flags < 16U; ++flags)
        {
            setup(HAL_FDCAN_STATE_BUSY, false);
            BspCanFrame frame = {.identifier = 0x1FFFFFFFU,
                                 .length = (uint8_t)length,
                                 .extended = (flags & 1U) != 0U,
                                 .remote = (flags & 2U) != 0U,
                                 .fd = (flags & 4U) != 0U,
                                 .bitrate_switch = (flags & 8U) != 0U};
            for (unsigned i = 0; i < 64U; ++i)
            {
                frame.data[i] = (uint8_t)(i ^ length);
            }
            bool legal = false;
            for (unsigned i = 0; i < 16U; ++i)
            {
                legal |= length == lengths[i];
            }
            legal = legal && (flags & 1U) &&
                    ((flags & 4U) ? !(flags & 2U) : !(flags & 8U) && length <= 8U);
            assert(BspCan_TrySendFrame(&frame) == legal);
            assert(sends == (unsigned)legal);
            if (legal)
            {
                assert(lengths[sent_header.DataLength] == length);
                assert(sent_header.Identifier == frame.identifier);
                assert(memcmp(frame.data, sent_data, length) == 0);
            }
        }
    }
    for (unsigned dlc = 0U; dlc < 32U; ++dlc)
    {
        for (unsigned flags = 0U; flags < 16U; ++flags)
        {
            setup(HAL_FDCAN_STATE_BUSY, false);
            rx_header = (FDCAN_RxHeaderTypeDef){
                .DataLength = dlc,
                .Identifier = 0x1FFFFFFFU,
                .IdType = flags & 1U ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID,
                .RxFrameType = flags & 2U ? 1U : FDCAN_DATA_FRAME,
                .FDFormat = flags & 4U ? FDCAN_FD_CAN : 0U,
                .BitRateSwitch = flags & 8U ? FDCAN_BRS_ON : FDCAN_BRS_OFF};
            memset(rx_data, (int)(dlc ^ flags), sizeof(rx_data));
            rx_ready = true;
            BspCanFrame frame, before;
            memset(&frame, 0xA5, sizeof(frame));
            before = frame;
            bool valid = dlc < 16U && (flags & 1U) &&
                         ((flags & 4U) ? !(flags & 2U) : !(flags & 8U) && dlc <= 8U);
            assert(BspCan_Receive(&frame) == valid);
            if (valid)
            {
                assert(frame.length == lengths[dlc] && frame.identifier == rx_header.Identifier);
                assert(memcmp(frame.data, rx_data, frame.length) == 0);
            }
            else
            {
                assert(memcmp(&before, &frame, sizeof(frame)) == 0);
            }
        }
    }
    setup(HAL_FDCAN_STATE_BUSY, false);
    BspCan_OnReceiveInterrupt(on_receive);
    BspCan_OnReceiveInterrupt(NULL);
    assert(callback_count == 1U);
    assert(!BspCan_Receive(NULL) && rx_reads == 0U);
}
/** @brief 验证超时/取消后槽释放前不可重用，旧ACK不能复活64位票据。 */
static void test_completion_matrix(void)
{
    const uint64_t ticket = UINT64_C(0xFEDCBA9876543210);
    for (unsigned flags = 0U; flags < 32U; ++flags)
    {
        setup(HAL_FDCAN_STATE_BUSY, false);
        now_ms = UINT32_MAX - 40U;
        BspCanFrame frame = {.identifier = 0x1234567U,
                             .length = 8U,
                             .extended = true,
                             .fd = true,
                             .bitrate_switch = true};
        assert(!BspCan_TrySendTrackedFrame(&frame, 0U) && sends == 0U);
        assert(BspCan_TrySendTrackedFrame(&frame, ticket));
        registers.TXBTO = flags & 1U ? 1U : 0U;
        registers.TXBCF = flags & 2U ? 1U : 0U;
        registers.PSR = flags & 4U ? FDCAN_PSR_BO : 0U;
        now_ms += flags & 8U ? 100U : 99U;
        registers.CCCR = flags & 16U ? FDCAN_CCCR_INIT : 0U;
        abort_delayed = true;
        BspCanTxCompletion result = {123U, true}, before = result;
        bool terminal = flags != 0U;
        assert(BspCan_TakeTxCompletion(&result) == terminal);
        if (terminal)
        {
            bool success = (flags & 1U) && !(flags & (4U | 8U | 16U));
            assert(result.ticket == ticket && result.success == success);
            assert(!BspCan_TakeTxCompletion(&result));
            assert(!BspCan_TrySendFrame(&frame));
            registers.TXBRP = 0U;
            registers.TXBTO = 1U;
            assert(!BspCan_TakeTxCompletion(&result));
        }
        else
        {
            assert(memcmp(&before, &result, sizeof(before)) == 0);
        }
    }
}
int main(void)
{
    test_startup();
    test_frames();
    test_completion_matrix();
    setup(HAL_FDCAN_STATE_BUSY, false);
    check(!BspCan_ServiceBusOff() && stops == 0 && starts == 0 && errors == 0,
          "healthy controller was restarted");

    setup(HAL_FDCAN_STATE_BUSY, true);
    registers.PSR = FDCAN_PSR_BO;
    check(BspCan_ServiceBusOff() && stops == 1 && starts == 1 && errors == 0 &&
              strcmp(lifecycle, "TS") == 0 && hfdcan1.State == HAL_FDCAN_STATE_BUSY,
          "BUSY bus-off must Stop before Start without Error_Handler");
    check(!BspCan_IsHealthy(), "bus-off recovery request was treated as physical recovery");
    registers.PSR = 0U; /* 仅模拟之后硬件完成总线恢复；HAL Start 本身不证明这一点。 */
    check(BspCan_IsHealthy(), "completed controller recovery stayed unhealthy");

    setup(HAL_FDCAN_STATE_READY, true);
    check(BspCan_ServiceBusOff() && stops == 0 && starts == 1 && errors == 0,
          "READY controller did not Start directly");

    setup(HAL_FDCAN_STATE_BUSY, true);
    stop_fails = true;
    check(!BspCan_ServiceBusOff() && stops == 1 && starts == 0 && errors == 0 &&
              !BspCan_IsHealthy(),
          "Stop failure must return false and preserve unavailable controller");

    setup(HAL_FDCAN_STATE_READY, true);
    start_fails = true;
    check(!BspCan_ServiceBusOff() && starts == 1 && errors == 0 && !BspCan_IsHealthy(),
          "Start failure must return false without Error_Handler");

    setup(HAL_FDCAN_STATE_BUSY, false);
    const uint64_t ticket = UINT64_C(0x1122334455667788);
    BspCanFrame frame = {.identifier = 0x0CEF0201U,
                         .length = 20,
                         .extended = true,
                         .fd = true,
                         .bitrate_switch = true};
    check(BspCan_TrySendTrackedFrame(&frame, ticket), "tracked frame setup failed");
    registers.PSR = FDCAN_PSR_BO;
    registers.CCCR = FDCAN_CCCR_INIT;
    registers.TXBTO = 1U; /* 恢复附近的旧完成位不能复活已失败票据。 */
    (void)BspCan_ServiceBusOff();
    registers.PSR = 0U;
    BspCanTxCompletion completion = {0};
    check(BspCan_TakeTxCompletion(&completion) && completion.ticket == ticket &&
              !completion.success && aborts >= 1 && errors == 0,
          "recovered old ACK became a successful tracked completion");
    check(!BspCan_TakeTxCompletion(&completion) && sends == 1,
          "recovery replayed a frame or duplicated completion");
    printf("comm bus-off: %u failure(s); no hardware access\n", failures);
    return failures ? 1 : 0;
}

uint32_t BspSystem_TimeMs(void)
{
    return HAL_GetTick();
}
