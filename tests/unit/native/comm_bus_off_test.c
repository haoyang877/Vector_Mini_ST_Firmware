/* 使用真实通信端口和窄 HAL 状态桩复现 bus-off；不访问寄存器或硬件。 */
#include "comm_hw.h"
#include "critical_hw.h"
#include <stdio.h>
#include <string.h>

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

typedef struct
{
    uint32_t PSR, CCCR, TXBRP, TXBTO, TXBCF;
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
static char lifecycle[8];
static unsigned lifecycle_size;

uint32_t critical_hw_enter(void)
{
    return 0U;
}
void critical_hw_exit(uint32_t key)
{
    (void)key;
}
uint32_t HAL_GetTick(void)
{
    return 10U;
}
void Error_Handler(void)
{
    ++errors;
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
    handle->Instance->TXBRP &= ~mask;
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
    (void)header;
    (void)data;
    ++sends;
    if (handle->State != HAL_FDCAN_STATE_BUSY)
    {
        return HAL_ERROR;
    }
    handle->LatestTxFifoQRequest = 1U;
    handle->Instance->TXBRP = 1U;
    handle->Instance->TXBTO = 0U;
    return HAL_OK;
}
int HAL_FDCAN_GetRxMessage(FDCAN_HandleTypeDef *handle,
                           uint32_t fifo,
                           FDCAN_RxHeaderTypeDef *header,
                           uint8_t *data)
{
    (void)handle;
    (void)fifo;
    (void)header;
    (void)data;
    return HAL_ERROR;
}

#include "comm_status_stm32g4.c"

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
    memset(lifecycle, 0, sizeof(lifecycle));
}
int main(void)
{
    setup(HAL_FDCAN_STATE_BUSY, false);
    check(!comm_hw_can_service_bus_off() && stops == 0 && starts == 0 && errors == 0,
          "healthy controller was restarted");

    setup(HAL_FDCAN_STATE_BUSY, true);
    registers.PSR = FDCAN_PSR_BO;
    check(comm_hw_can_service_bus_off() && stops == 1 && starts == 1 && errors == 0 &&
              strcmp(lifecycle, "TS") == 0 && hfdcan1.State == HAL_FDCAN_STATE_BUSY,
          "BUSY bus-off must Stop before Start without Error_Handler");
    check(!comm_hw_can_is_healthy(), "bus-off recovery request was treated as physical recovery");
    registers.PSR = 0U; /* 仅模拟之后硬件完成总线恢复；HAL Start 本身不证明这一点。 */
    check(comm_hw_can_is_healthy(), "completed controller recovery stayed unhealthy");

    setup(HAL_FDCAN_STATE_READY, true);
    check(comm_hw_can_service_bus_off() && stops == 0 && starts == 1 && errors == 0,
          "READY controller did not Start directly");

    setup(HAL_FDCAN_STATE_BUSY, true);
    stop_fails = true;
    check(!comm_hw_can_service_bus_off() && stops == 1 && starts == 0 && errors == 0 &&
              !comm_hw_can_is_healthy(),
          "Stop failure must return false and preserve unavailable controller");

    setup(HAL_FDCAN_STATE_READY, true);
    start_fails = true;
    check(!comm_hw_can_service_bus_off() && starts == 1 && errors == 0 && !comm_hw_can_is_healthy(),
          "Start failure must return false without Error_Handler");

    setup(HAL_FDCAN_STATE_BUSY, false);
    const uint64_t ticket = UINT64_C(0x1122334455667788);
    CommHwCanFrame frame = {.identifier = 0x0CEF0201U,
                            .length = 20,
                            .extended = true,
                            .fd = true,
                            .bitrate_switch = true};
    check(comm_hw_can_try_send_tracked_frame(&frame, ticket), "tracked frame setup failed");
    registers.PSR = FDCAN_PSR_BO;
    registers.CCCR = FDCAN_CCCR_INIT;
    registers.TXBTO = 1U; /* 恢复附近的旧完成位不能复活已失败票据。 */
    (void)comm_hw_can_service_bus_off();
    registers.PSR = 0U;
    CommHwCanTxCompletion completion = {0};
    check(comm_hw_can_take_tx_completion(&completion) && completion.ticket == ticket &&
              !completion.success && aborts >= 1 && errors == 0,
          "recovered old ACK became a successful tracked completion");
    check(!comm_hw_can_take_tx_completion(&completion) && sends == 1,
          "recovery replayed a frame or duplicated completion");
    printf("comm bus-off: %u failure(s); no hardware access\n", failures);
    return failures ? 1 : 0;
}
