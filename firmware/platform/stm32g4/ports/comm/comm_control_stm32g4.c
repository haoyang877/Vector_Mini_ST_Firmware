#include "comm_hw.h"
#include "fdcan.h"
#include "main.h"

/* STM32G4 控制路径移植：把 FDCAN 滤波/启停/波特率与应答发送的 HAL 细节
 * 全部收在本文件，通信层只使用 comm_hw 契约。失败语义与迁移前逐字一致：
 * 初始化类失败即进入平台错误处理，应答发送只做单次非阻塞尝试。 */

void comm_hw_can_start(uint8_t node)
{
    FDCAN_FilterTypeDef FDCAN_Filter;

    FDCAN_Filter.IdType = FDCAN_STANDARD_ID;
    FDCAN_Filter.FilterIndex = 0;
    FDCAN_Filter.FilterType = FDCAN_FILTER_RANGE;
    FDCAN_Filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    FDCAN_Filter.FilterID1 = (((uint32_t)node) << 8);
    FDCAN_Filter.FilterID2 = (((uint32_t)node) << 8) + 0xFF;

    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &FDCAN_Filter) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_FDCAN_ConfigGlobalFilter(
            &hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) !=
        HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
        Error_Handler();
    }
}

void comm_hw_can_set_baudrate(uint32_t kbps)
{
    if (HAL_FDCAN_Stop(&hfdcan1) != HAL_OK)
    {
        Error_Handler();
    }

    if (kbps <= 1000U)
    {
        hfdcan1.Init.DataPrescaler = 10000U / kbps;
        hfdcan1.Init.NominalPrescaler = 10000U / kbps;
    }
    else
    {
        hfdcan1.Init.DataPrescaler = 10000U / kbps;
        hfdcan1.Init.NominalPrescaler = 10;
    }

    if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
    {
        Error_Handler();
    }
}

bool comm_hw_can_try_send_reply(uint16_t identifier, const uint8_t *data, uint8_t length)
{
    /* 先整体清零：HAL 会把 ESI 与 MessageMarker 等未显式赋值字段一并写入消息 RAM。 */
    FDCAN_TxHeaderTypeDef FDCAN_TxHeader = {0};

    FDCAN_TxHeader.IdType = FDCAN_STANDARD_ID;
    FDCAN_TxHeader.Identifier = identifier;
    FDCAN_TxHeader.FDFormat = FDCAN_FD_CAN;
    FDCAN_TxHeader.DataLength = length;
    FDCAN_TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    FDCAN_TxHeader.BitRateSwitch = FDCAN_BRS_ON;
    FDCAN_TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;

    return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &FDCAN_TxHeader, data) == HAL_OK;
}
