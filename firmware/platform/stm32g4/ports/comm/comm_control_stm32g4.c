#include "comm_hw.h"
#include "fdcan.h"
#include "main.h"

/* STM32G4 CAN FD 启动端口：只负责启动、全局帧型策略和 RX 中断通知。
 * 应用 ID、CRC、应答和业务语义全部由协议层处理。 */

void comm_hw_can_start_fd(void)
{

    if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
                                     FDCAN_REJECT,
                                     FDCAN_ACCEPT_IN_RX_FIFO0,
                                     FDCAN_FILTER_REMOTE,
                                     FDCAN_FILTER_REMOTE) != HAL_OK)
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

/* 保留原有 bus-off 恢复能力，但调度归前台 CAN FD 运行时，不依赖旧波特率命令。 */
bool comm_hw_can_service_bus_off(void)
{
    if ((hfdcan1.Instance->PSR & FDCAN_PSR_BO) == 0U &&
        (hfdcan1.Instance->CCCR & FDCAN_CCCR_INIT) == 0U)
    {
        return false;
    }
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
    {
        Error_Handler();
    }
    return true;
}
