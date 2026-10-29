/** @file bsp_uart.c
 * @brief 当前板卡未分配UART；禁用功能统一返回明确结果，不创建MCU空驱动。
 */
#include "bsp_uart.h"

BspResult BspUart_TryWrite(const uint8_t *data, size_t length)
{
    (void)data;
    (void)length;
    return BSP_NOT_SUPPORTED;
}
BspResult BspUart_TakeTxResult(BspUartTxResult *result)
{
    (void)result;
    return BSP_NOT_SUPPORTED;
}
BspResult BspUart_Read(uint8_t *data, size_t capacity, size_t *received)
{
    (void)data;
    (void)capacity;
    if (received != NULL)
    {
        *received = 0U;
    }
    return BSP_NOT_SUPPORTED;
}
BspUartState BspUart_GetState(void)
{
    return BSP_UART_DISABLED;
}
