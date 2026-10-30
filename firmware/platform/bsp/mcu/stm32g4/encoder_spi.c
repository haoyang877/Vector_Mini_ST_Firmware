/**
 * @file encoder_spi.c
 * @brief STM32编码器初始化与SPI直接采样。
 */
#include "../../bsp_internal.h"
#include "bsp_encoder.h"
#include "board_config.h"
#include "stm32g4xx_hal.h"

void McuEncoder_Init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_SPI2_CLK_ENABLE();
    __HAL_RCC_SPI2_FORCE_RESET();
    __HAL_RCC_SPI2_RELEASE_RESET();
    GPIOB->BSRR = 1U << BSP_ENCODER_CS_PIN;

    GPIO_InitTypeDef pins = {0};
    pins.Pin = 1U << BSP_ENCODER_CS_PIN;
    pins.Mode = GPIO_MODE_OUTPUT_PP;
    pins.Pull = GPIO_PULLUP;
    pins.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &pins);
    pins.Pin =
        (1U << BSP_ENCODER_SCK_PIN) | (1U << BSP_ENCODER_MISO_PIN) | (1U << BSP_ENCODER_MOSI_PIN);
    pins.Mode = GPIO_MODE_AF_PP;
    pins.Pull = GPIO_NOPULL;
    pins.Alternate = BSP_ENCODER_GPIO_AF;
    HAL_GPIO_Init(GPIOB, &pins);

    SPI2->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_CPHA | SPI_CR1_BR_2;
    SPI2->CR2 = SPI_CR2_DS;
    SPI2->CR1 |= SPI_CR1_SPE;
}

void BspEncoder_Begin(void)
{
    GPIOB->BSRR = (1U << BSP_ENCODER_CS_PIN) << 16U;
    while ((SPI2->SR & SPI_SR_TXE) == 0U)
    {
    }
    *(__IO uint16_t *)&SPI2->DR = 0x8021U;
    while ((SPI2->SR & SPI_SR_RXNE) == 0U)
    {
    }
    (void)*(__IO uint16_t *)&SPI2->DR;
    while ((SPI2->SR & SPI_SR_BSY) != 0U)
    {
    }
    /* TLE5012B读回由传感器驱动共用数据线，主机释放MOSI。 */
    GPIOB->MODER &= ~(3U << (2U * BSP_ENCODER_MOSI_PIN));
    __NOP();
    __NOP();
    *(__IO uint16_t *)&SPI2->DR = 0U;
}

uint16_t BspEncoder_ReadAngle(void)
{
    while ((SPI2->SR & SPI_SR_RXNE) == 0U)
    {
    }
    uint16_t word = *(__IO uint16_t *)&SPI2->DR;
    while ((SPI2->SR & SPI_SR_BSY) != 0U)
    {
    }
    GPIOB->MODER =
        (GPIOB->MODER & ~(3U << (2U * BSP_ENCODER_MOSI_PIN))) | (2U << (2U * BSP_ENCODER_MOSI_PIN));
    GPIOB->BSRR = 1U << BSP_ENCODER_CS_PIN;
    return (uint16_t)((word & 0x7FFFU) << 1U);
}
