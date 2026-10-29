#include "bsp_internal.h"
#include "vector_mini_st_g431.h"
#include <stddef.h>
#include <string.h>
#include "bsp_system.h"
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "fdcan.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"
#include "bsp.h"
#include "boot_hw.h"
#include "bsp_pwm.h"
#include "bsp_can.h"
#include "bsp_config.h"

static bool s_microseconds_ready;
static uint32_t s_cycle_anchor, s_cycle_remainder, s_microseconds, s_cycles_per_us;

static void system_time_advance(void)
{
    if (!s_microseconds_ready)
    {
        return;
    }
    uint32_t cycles = DWT->CYCCNT;
    uint32_t elapsed = cycles - s_cycle_anchor;
    s_cycle_anchor = cycles;
    s_microseconds += elapsed / s_cycles_per_us;
    s_cycle_remainder += elapsed % s_cycles_per_us;
    if (s_cycle_remainder >= s_cycles_per_us)
    {
        s_cycle_remainder -= s_cycles_per_us;
        ++s_microseconds;
    }
}

static void system_time_init(void)
{
    uint32_t key = BspSystem_EnterCritical();
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    s_cycles_per_us = BspSystem_CoreClockHz() / 1000000U;
    s_microseconds = BspSystem_TimeMs() * 1000U;
    s_cycle_anchor = DWT->CYCCNT;
    s_cycle_remainder = 0U;
    s_microseconds_ready = s_cycles_per_us != 0U;
    BspSystem_ExitCritical(key);
}

uint32_t BspSystem_TimeUs(void)
{
    uint32_t key = BspSystem_EnterCritical();
    system_time_advance();
    uint32_t now_us = s_microseconds;
    BspSystem_ExitCritical(key);
    return now_us;
}

uint32_t BspSystem_CoreClockHz(void)
{
    return SystemCoreClock;
}

void BspSystem_Reset(void)
{
    NVIC_SystemReset();
}

uint32_t BspSystem_TimeMs(void)
{
    return HAL_GetTick();
}

/* 时基与PRIMASK恢复集中在本板同一系统原语文件。 */
uint32_t BspSystem_EnterCritical(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    return primask;
}

void BspSystem_ExitCritical(uint32_t state)
{
    __set_PRIMASK(state);
}

BspResult BspSystem_ReadUid(uint8_t uid[BSP_SYSTEM_UID_BYTES])
{
    if (uid == NULL)
    {
        return BSP_INVALID_ARGUMENT;
    }
    const uint32_t words[3] = {HAL_GetUIDw0(), HAL_GetUIDw1(), HAL_GetUIDw2()};
    for (uint32_t word = 0U; word < 3U; ++word)
    {
        for (uint32_t byte = 0U; byte < 4U; ++byte)
        {
            uid[word * 4U + byte] = (uint8_t)(words[word] >> (byte * 8U));
        }
    }
    return BSP_OK;
}

BspResult BspSystem_ReadIdentity(uint8_t record[64])
{
    if (record == NULL)
    {
        return BSP_INVALID_ARGUMENT;
    }
    memcpy(record, (const void *)TARGET_IDENTITY_BASE, 64U);
    return BSP_OK;
}

/* HAL弱钩子由本系统实现接管，只增加扩展时基维护，原毫秒步进不变。 */
void HAL_IncTick(void)
{
    uint32_t key = BspSystem_EnterCritical();
    uwTick += (uint32_t)uwTickFreq;
    system_time_advance();
    BspSystem_ExitCritical(key);
}

/* 启动状态只由本MCU实现持有；失败不允许同次启动重试。 */
static unsigned s_startup_state;

static void system_clock_init(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
  */
    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

    /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
    RCC_OscInitStruct.PLL.PLLN = 85;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
  */
    RCC_ClkInitStruct.ClockType =
        RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
    {
        Error_Handler();
    }
}

BspResult Bsp_Init(void)
{
    if (s_startup_state != 0U)
    {
        return BSP_NOT_READY;
    }
    s_startup_state = 1U;

    (void)BootHw_Init();
    HAL_Init();
    system_clock_init();
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_ADC2_Init();
    if (!Can_Init())
    {
        goto failed;
    }
    Pwm_Init();
    MX_TIM2_Init();
    MX_TIM7_Init();
    Led_Init();
    if (BSP_ENABLED_INTERNAL_ANALOG_SAMPLING)
    {
        MX_ADC1_Init();
    }
    MX_SPI1_Init();
    MX_SPI2_Init();
    /* 原有预取必须先于App_Init启动采样，时钟和外设顺序保持不变。 */
    __HAL_FLASH_PREFETCH_BUFFER_ENABLE();

    if (!Adc_Init())
    {
        goto failed;
    }

    system_time_init();
    if (!Flash_Init() || !Encoder_Init())
    {
        goto failed;
    }
    s_startup_state = 2U;
    return BSP_OK;
failed:
    BspPwm_Disable();
    s_startup_state = 4U;
    return BSP_IO_ERROR;
}

BspResult Bsp_Start(void)
{
    if (s_startup_state != 2U)
    {
        return BSP_NOT_READY;
    }
    s_startup_state = 3U;
    if (!Adc_Start() || HAL_TIM_Base_Start_IT(&htim7) != HAL_OK)
    {
        goto failed;
    }

    return BSP_OK;
failed:
    BspPwm_Disable();
    s_startup_state = 4U;
    return BSP_IO_ERROR;
}

BspResult BspWatchdog_Start(void)
{
    return BSP_NOT_SUPPORTED;
}

void BspWatchdog_Feed(void)
{
    /* 独立APP没有应用看门狗；Loader继承看门狗仍由BootHw管理。 */
}

void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    BspPwm_Disable();
    while (1)
    {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file;
    (void)line;
}
#endif

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)

{

    if (hadc == &hadc2 && Adc_CompleteFast())

    {

        BspApp_OnFastSample();
    }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)

{

    if (hfdcan == &hfdcan1)

    {

        Can_OnReceiveInterrupt(BspApp_OnCanReceive, RxFifo0ITs);
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)

{

    if (htim == (&htim7))

    {

        BspApp_OnControlTick();
        Led_SupervisorTick();
    }
}

/* ADC为0，载波/采样DMA为1；前台只与低频监督互斥。 */
uint32_t BspSystem_EnterControlCritical(void)
{
    uint32_t state = __get_BASEPRI();
    __set_BASEPRI_MAX(2U << (8U - __NVIC_PRIO_BITS));
    __DSB();
    __ISB();
    return state;
}

void BspSystem_ExitControlCritical(uint32_t state)
{
    __DMB();
    __set_BASEPRI(state);
    __ISB();
}

void BspSystem_MemoryBarrier(void)
{
    __DMB();
}

bool BspSystem_CompareExchange32(volatile uint32_t *address, uint32_t expected, uint32_t desired)
{
    do
    {
        if (__LDREXW(address) != expected)
        {
            __CLREX();
            return false;
        }
        __DMB();
    } while (__STREXW(desired, address) != 0U);
    __DMB();
    return true;
}
