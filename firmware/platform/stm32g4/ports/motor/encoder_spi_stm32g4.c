#include "encoder_spi.h"

#include "main.h"
#include "spi.h"

/* 编码器 SPI 端口：唯一访问 SPI2、片选与 MOSI 方向的实现。
 * 时序、超时常数与表达式与去耦前的 bsp/encoder.c 完全一致。 */

#define ENCODER_SPI_HANDLE hspi2
#define ENCODER_SPI_CS_ENABLE BRD_ENC_CS_GPIO_Port->BSRR = (uint32_t)BRD_ENC_CS_Pin << 16U
#define ENCODER_SPI_CS_DISABLE BRD_ENC_CS_GPIO_Port->BSRR = BRD_ENC_CS_Pin

#ifndef ENC_SPI_XFER_SPIN_MAX
#define ENC_SPI_XFER_SPIN_MAX 340U
#endif

static bool SPI_WaitFlag(SPI_TypeDef *SPIx, uint32_t flag, uint32_t timeout_spin)
{
    uint32_t spin = 0U;

    while ((SPIx->SR & flag) == 0U)
    {
        if (++spin > timeout_spin)
        {
            return false;
        }
    }
    return true;
}

static bool SPI_WaitBSYClear(SPI_TypeDef *SPIx, uint32_t timeout_spin)
{
    uint32_t spin = 0U;

    while ((SPIx->SR & SPI_FLAG_BSY) != 0U)
    {
        if (++spin > timeout_spin)
        {
            return false;
        }
    }
    return true;
}

static uint16_t SPI_Reg_ReadRx16(SPI_TypeDef *SPIx, bool *ok)
{
    uint16_t rx_data;

    if (!SPI_WaitFlag(SPIx, SPI_FLAG_RXNE, ENC_SPI_XFER_SPIN_MAX))
    {
        *ok = false;
        return 0U;
    }
    rx_data = *(__IO uint16_t *)&SPIx->DR;
    if (!SPI_WaitBSYClear(SPIx, ENC_SPI_XFER_SPIN_MAX))
    {
        *ok = false;
        return 0U;
    }
    *ok = true;
    return rx_data;
}

static uint16_t SPI_Reg_TxRx16(SPI_TypeDef *SPIx, uint16_t tx_data, bool *ok)
{
    if ((SPIx->CR1 & SPI_CR1_SPE) == 0U)
    {
        SPIx->CR1 |= SPI_CR1_SPE;
    }

    if ((SPIx->SR & SPI_FLAG_OVR) != 0U)
    {
        (void)*(__IO uint16_t *)&SPIx->DR;
        (void)SPIx->SR;
    }

    if (!SPI_WaitFlag(SPIx, SPI_FLAG_TXE, ENC_SPI_XFER_SPIN_MAX))
    {
        *ok = false;
        return 0U;
    }
    *(__IO uint16_t *)&SPIx->DR = tx_data;
    return SPI_Reg_ReadRx16(SPIx, ok);
}

/* TLE/SSC 类传感器在读相位由传感器驱动数据线，主机须先释放 MOSI。 */
static void SPI2_MOSI_HiZ(void)
{
    GPIOB->MODER &= ~(0x3UL << 30U);
}

static void SPI2_MOSI_RestoreAF(void)
{
    GPIOB->MODER = (GPIOB->MODER & ~(0x3UL << 30U)) | (0x2UL << 30U);
    GPIOB->AFR[1] = (GPIOB->AFR[1] & ~(0xFUL << 28U)) | (0x5UL << 28U);
}

void encoder_spi_init(void)
{
    SPI_HandleTypeDef *encoder_spi = &ENCODER_SPI_HANDLE;

    encoder_spi->Init.DataSize = SPI_DATASIZE_16BIT;
    encoder_spi->Init.CLKPolarity = SPI_POLARITY_LOW;
    encoder_spi->Init.CLKPhase = SPI_PHASE_2EDGE;
    encoder_spi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
    if (HAL_SPI_Init(encoder_spi) != HAL_OK)
    {
        Error_Handler();
    }
}

bool encoder_spi_read_begin(uint16_t request_frame)
{
    SPI_TypeDef *SPIx = ENCODER_SPI_HANDLE.Instance;

    /* 不抢在保护之前等待：总线状态异常时保留同步回退路径。 */
    if ((SPIx->CR1 & SPI_CR1_SPE) == 0U ||
        (SPIx->SR & (SPI_FLAG_TXE | SPI_FLAG_OVR | SPI_FLAG_BSY | SPI_FLAG_RXNE)) != SPI_FLAG_TXE)
    {
        return false;
    }
    ENCODER_SPI_CS_ENABLE;
    *(__IO uint16_t *)&SPIx->DR = request_frame;
    return true;
}

bool encoder_spi_read_complete(bool begin_ok,
                               uint16_t request_frame,
                               uint16_t read_frame,
                               uint16_t *word)
{
    SPI_TypeDef *SPIx = ENCODER_SPI_HANDLE.Instance;
    uint16_t received = 0U;
    bool transfer_ok = true;

    if (begin_ok)
    {
        (void)SPI_Reg_ReadRx16(SPIx, &transfer_ok);
    }
    else
    {
        ENCODER_SPI_CS_ENABLE;
        (void)SPI_Reg_TxRx16(SPIx, request_frame, &transfer_ok);
    }
    if (transfer_ok)
    {
        SPI2_MOSI_HiZ();
        __NOP();
        __NOP();
        received = SPI_Reg_TxRx16(SPIx, read_frame, &transfer_ok);
        SPI2_MOSI_RestoreAF();
    }
    (void)SPI_WaitBSYClear(SPIx, ENC_SPI_XFER_SPIN_MAX);
    ENCODER_SPI_CS_DISABLE;

    if (!transfer_ok)
    {
        return false;
    }
    *word = received;
    return true;
}
