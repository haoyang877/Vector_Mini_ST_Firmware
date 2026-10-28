/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32g4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32g4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "common_inc.h"
#include "fast_loop_profile.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
/* 同一CAN IRQ不可重入；标记HAL回调是否已消耗本次三帧预算。 */
static bool can_rx_serviced;
#if defined(SERVO_HIL_ENABLE) && SERVO_HIL_ENABLE
static volatile struct {
  uint32_t count, last_cycles, max_cycles, preempted_jobs;
  uint64_t total_cycles;
} outer_irq_profile;
#endif
#if defined(SERVO_HIL_ENABLE) && SERVO_HIL_ENABLE
static volatile uint32_t hil_irq_last_cycles;
static volatile uint32_t hil_irq_max_cycles;
static volatile uint32_t hil_irq_histogram[4];
/* Additional profiling is independent of host resets of the legacy counters.
 * Read while halted/disabled or use two snapshots; a 64-bit read is not atomic. */
static volatile uint32_t hil_profile_count;
static volatile uint64_t hil_profile_total_cycles;
static volatile uint32_t hil_profile_min_cycles = UINT32_MAX;
static volatile uint32_t hil_profile_interval_min_cycles = UINT32_MAX;
static volatile uint32_t hil_profile_interval_max_cycles;
static uint32_t hil_profile_previous_start;
#endif

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
static void CAN_DrainRxFifo0(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#if defined(FAST_LOOP_STAGE_PROFILE) && FAST_LOOP_STAGE_PROFILE
/* Diagnostic storage is private to the board clock adapter. No per-stage
 * arrays: one selected stage keeps the temporary profiling image within RAM. */
static volatile struct {
  uint32_t selected, count;
  uint64_t total;
  uint32_t minimum, maximum, started;
} fast_loop_stage_profile;

void FastLoopProfile_Begin(unsigned stage)
{
  if (fast_loop_stage_profile.selected == stage)
    fast_loop_stage_profile.started = DWT->CYCCNT;
}

void FastLoopProfile_End(unsigned stage)
{
  if (fast_loop_stage_profile.selected == stage) {
    uint32_t elapsed = DWT->CYCCNT - fast_loop_stage_profile.started;
    if (elapsed < fast_loop_stage_profile.minimum)
      fast_loop_stage_profile.minimum = elapsed;
    if (elapsed > fast_loop_stage_profile.maximum)
      fast_loop_stage_profile.maximum = elapsed;
    fast_loop_stage_profile.total += elapsed;
    fast_loop_stage_profile.count++;
  }
}
#endif

/* Board IRQ adapter: specialize only the independently triggered, single JEOS
 * event used by this board. All other configurations/events retain HAL handling.
 * Keep the callback-before-clear ordering and HAL injected state semantics. */
static void Board_ADC2DispatchInterrupt(void)
{
  uint32_t pending = hadc2.Instance->ISR & hadc2.Instance->IER;
  if (pending == 0U)
    return;
#if (USE_HAL_ADC_REGISTER_CALLBACKS == 0)
  if (pending == ADC_FLAG_JEOS &&
      (hadc2.Instance->JSQR & ADC_JSQR_JEXTEN) != 0U &&
      (hadc2.Instance->CFGR & (ADC_CFGR_JAUTO | ADC_CFGR_JQM)) == 0U &&
      (ADC12_COMMON->CCR & ADC_CCR_DUAL) == 0U &&
      (hadc2.State & HAL_ADC_STATE_ERROR_INTERNAL) == 0U)
  {
    hadc2.State |= HAL_ADC_STATE_INJ_EOC;
    HAL_ADCEx_InjectedConvCpltCallback(&hadc2);
    __HAL_ADC_CLEAR_FLAG(&hadc2, ADC_FLAG_JEOC | ADC_FLAG_JEOS);
    return;
  }
#endif
  HAL_ADC_IRQHandler(&hadc2);
}

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern FDCAN_HandleTypeDef hfdcan1;
extern DMA_HandleTypeDef hdma_tim2_ch3;
extern TIM_HandleTypeDef htim7;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */
#if defined(SERVO_HIL_ENABLE) && SERVO_HIL_ENABLE
  uint32_t started = DWT->CYCCNT;
  uint32_t fast_count = hil_profile_count;
#endif
  MotorOuterLoop_Service();
  FOC_TelemetryService();
#if defined(SERVO_HIL_ENABLE) && SERVO_HIL_ENABLE
  outer_irq_profile.last_cycles = DWT->CYCCNT - started;
  if (outer_irq_profile.last_cycles > outer_irq_profile.max_cycles)
    outer_irq_profile.max_cycles = outer_irq_profile.last_cycles;
  outer_irq_profile.total_cycles += outer_irq_profile.last_cycles;
  if (fast_count != hil_profile_count) outer_irq_profile.preempted_jobs++;
  outer_irq_profile.count++;
#endif
  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32G4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel1 global interrupt.
  */
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */

  /* USER CODE END DMA1_Channel1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_tim2_ch3);
  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
  * @brief This function handles ADC1 and ADC2 global interrupt.
  */
void ADC1_2_IRQHandler(void)
{
  /* USER CODE BEGIN ADC1_2_IRQn 0 */
#if defined(SERVO_HIL_ENABLE) && SERVO_HIL_ENABLE
  uint32_t hil_start, hil_elapsed;
  if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0U) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  }
  hil_start = DWT->CYCCNT;
  if (hil_profile_count != 0U) {
    uint32_t interval = hil_start - hil_profile_previous_start;
    if (interval < hil_profile_interval_min_cycles) hil_profile_interval_min_cycles = interval;
    if (interval > hil_profile_interval_max_cycles) hil_profile_interval_max_cycles = interval;
  }
  hil_profile_previous_start = hil_start;
#endif

  /* USER CODE END ADC1_2_IRQn 0 */
  /* Board IRQ dispatch: ISR/IER bit positions match on STM32G4.
   * Skip idle peripherals, but preserve every enabled ADC event/error path.
   * Retain these guards when regenerating the shared ADC vector. */

  if ((hadc1.Instance->ISR & hadc1.Instance->IER) != 0U)
    HAL_ADC_IRQHandler(&hadc1);
  Board_ADC2DispatchInterrupt();
  /* USER CODE BEGIN ADC1_2_IRQn 1 */
#if defined(SERVO_HIL_ENABLE) && SERVO_HIL_ENABLE
  hil_elapsed = DWT->CYCCNT - hil_start;
  hil_irq_last_cycles = hil_elapsed;
  if (hil_elapsed > hil_irq_max_cycles) hil_irq_max_cycles = hil_elapsed;
  /* 170 MHz board clock: bins below 50, 100, 150 us, and >=150 us. */
  hil_irq_histogram[hil_elapsed < 8500U ? 0 : hil_elapsed < 17000U ? 1 :
                    hil_elapsed < 25500U ? 2 : 3]++;
  if (hil_elapsed < hil_profile_min_cycles) hil_profile_min_cycles = hil_elapsed;
  hil_profile_total_cycles += hil_elapsed;
  hil_profile_count++;
#endif

  /* USER CODE END ADC1_2_IRQn 1 */
}





/**
  * @brief This function handles FDCAN1 interrupt 0.
  */
void FDCAN1_IT0_IRQHandler(void)
{
  /* USER CODE BEGIN FDCAN1_IT0_IRQn 0 */
  can_rx_serviced = false;
  /* USER CODE END FDCAN1_IT0_IRQn 0 */
  HAL_FDCAN_IRQHandler(&hfdcan1);
  /* USER CODE BEGIN FDCAN1_IT0_IRQn 1 */
  /* 软件挂起的IRQ不一定有RF0N标志，HAL不会回调；也必须继续消费。 */
  if (!can_rx_serviced) { CAN_DrainRxFifo0(); }

  /* USER CODE END FDCAN1_IT0_IRQn 1 */
}

/**
  * @brief This function handles TIM7 global interrupt.
  */
void TIM7_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_IRQn 0 */

  /* USER CODE END TIM7_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM7_IRQn 1 */

  /* USER CODE END TIM7_IRQn 1 */
}

/* USER CODE BEGIN 1 */
/**
 * @brief 每次IRQ最多消费硬件FIFO深度的三帧，ADC仍可抢占。
 * @note 预算后仍有数据则挂起下一次IRQ，不能等待新的总线报文唤醒。
 * HAL已清除RF0N时，软件续调度仍通过IRQ尾部入口获得下一份预算。
 */
static void CAN_DrainRxFifo0(void)
{
    uint32_t count;
    uint32_t depth = HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0);
    can_rx_serviced = true;
    if (depth > CAN_Diagnostics.rx_peak_depth) { CAN_Diagnostics.rx_peak_depth = depth; }
    for (count = 0U; count < 3U && depth != 0U; ++count) {
        CANRxIRQHandler();
        depth = HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0);
        if (depth > CAN_Diagnostics.rx_peak_depth) { CAN_Diagnostics.rx_peak_depth = depth; }
    }
    if (depth != 0U) {
        if (CAN_Diagnostics.rx_budget_exhausted != UINT32_MAX) {
            ++CAN_Diagnostics.rx_budget_exhausted;
        }
        HAL_NVIC_SetPendingIRQ(FDCAN1_IT0_IRQn);
    }
}


/**
   * @brief  ADCIRQHandler, deal with FOC related tasks
			 Interrupt frequency: 20kHz
   * @param  
   * @retval 
   */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	if(hadc == &hadc2 && __HAL_ADC_GET_FLAG(hadc, ADC_FLAG_JEOS))
	{		
		FOC20kHzIRQHandler();
	}
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	if(hfdcan==&hfdcan1)
	{
        if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_MESSAGE_LOST) != 0U &&
            CAN_Diagnostics.rx_fifo_lost != UINT32_MAX) {
            ++CAN_Diagnostics.rx_fifo_lost;
        }
        CAN_DrainRxFifo0();
	}
}

/**
   * @brief  TIM7IRQHandler, deal with low priority tasks
			 Interrupt frequency: 1kHz
   * @param  
   * @retval 
   */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == (&htim7))
    {
		BSP1kHzIRQHandler();
    }
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    HAL_TIM_PWM_Stop_DMA(&htim2,TIM_CHANNEL_3);
}
/* USER CODE END 1 */
