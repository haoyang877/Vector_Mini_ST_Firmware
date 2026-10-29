#ifndef STM32_ENCODER_FAKE_MAIN_H
#define STM32_ENCODER_FAKE_MAIN_H
#include <stdint.h>
/* 仅替代外设存储和初始化入口；执行真实SPI驱动及有界轮询函数。 */
#define __IO volatile
#define __NOP() ((void)0)
#define SPI_CR1_SPE 64U
#define SPI_FLAG_TXE 2U
#define SPI_FLAG_RXNE 1U
#define SPI_FLAG_BSY 128U
#define SPI_FLAG_OVR 64U
#define SPI_DATASIZE_16BIT 16U
#define SPI_POLARITY_LOW 0U
#define SPI_PHASE_2EDGE 2U
#define SPI_BAUDRATEPRESCALER_32 32U
#define HAL_OK 0U
typedef struct
{
    volatile uint32_t CR1, SR, DR;
} SPI_TypeDef;
typedef struct
{
    uint32_t DataSize, CLKPolarity, CLKPhase, BaudRatePrescaler;
} SpiInit;
typedef struct
{
    SPI_TypeDef *Instance;
    SpiInit Init;
} SPI_HandleTypeDef;
typedef struct
{
    volatile uint32_t MODER, AFR[2], BSRR;
} GPIO_TypeDef;
extern GPIO_TypeDef g_test_gpio;
#define GPIOB (&g_test_gpio)
#define BRD_ENC_CS_GPIO_Port GPIOB
#define BRD_ENC_CS_Pin 4096U
/** @brief 将真实驱动的初始化失败转回测试夹具。 */
void Error_Handler(void);
/**
 * @brief 捕获真实驱动提交的初始化设置。
 * @param spi 本次初始化的模拟SPI句柄。
 * @return 由测试场景指定的HAL成功或失败状态。
 */
unsigned HAL_SPI_Init(SPI_HandleTypeDef *spi);
#endif
