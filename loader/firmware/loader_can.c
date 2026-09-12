/**
  ******************************************************************************
  * @file    loader_can.c
  * @brief   CAN-FD implementation of the Loader link seam (1M/1M FD + BRS).
  *
  * Implements loader_link.h for FDCAN1 (PB8/PB9, AF9).  This is the only file
  * that knows about the vendor CAN API; an RS-485 or other-MCU port replaces
  * it without touching the Loader core.
  ******************************************************************************
  */
#include "loader_link.h"

#include "loader_cfg.h"
#include "stm32g4xx_hal.h"

#define LDR_RX_RING_SIZE 8U

static FDCAN_HandleTypeDef hldr_can;

static uint8_t s_node;

static ldr_rx_frame_t s_rx_ring[LDR_RX_RING_SIZE];
static volatile uint8_t s_rx_head;
static volatile uint8_t s_rx_tail;

static const uint8_t k_dlc_to_len[16] = {0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U,
										 8U, 12U, 16U, 20U, 24U, 32U, 48U, 64U};

static uint32_t len_to_dlc(uint8_t len)
{
	if (len <= 8U)
		return FDCAN_DLC_BYTES_8;
	if (len <= 12U)
		return FDCAN_DLC_BYTES_12;
	if (len <= 16U)
		return FDCAN_DLC_BYTES_16;
	if (len <= 20U)
		return FDCAN_DLC_BYTES_20;
	if (len <= 24U)
		return FDCAN_DLC_BYTES_24;
	if (len <= 32U)
		return FDCAN_DLC_BYTES_32;
	if (len <= 48U)
		return FDCAN_DLC_BYTES_48;
	return FDCAN_DLC_BYTES_64;
}

bool loader_link_init(uint8_t node)
{
	RCC_PeriphCLKInitTypeDef periph_clk = {0};
	GPIO_InitTypeDef gpio = {0};
	FDCAN_FilterTypeDef filter = {0};

	s_node = node;

	/* FDCAN kernel clock = PCLK1 (170 MHz), same as the application. */
	periph_clk.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
	periph_clk.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
	if (HAL_RCCEx_PeriphCLKConfig(&periph_clk) != HAL_OK)
	{
		return false;
	}
	__HAL_RCC_FDCAN_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/* PB8-BOOT0 = FDCAN1_RX, PB9 = FDCAN1_TX */
	gpio.Pin = GPIO_PIN_8 | GPIO_PIN_9;
	gpio.Mode = GPIO_MODE_AF_PP;
	gpio.Pull = GPIO_NOPULL;
	gpio.Speed = GPIO_SPEED_FREQ_LOW;
	gpio.Alternate = GPIO_AF9_FDCAN1;
	HAL_GPIO_Init(GPIOB, &gpio);

	hldr_can.Instance = FDCAN1;
	hldr_can.Init.ClockDivider = FDCAN_CLOCK_DIV1;
	hldr_can.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
	hldr_can.Init.Mode = FDCAN_MODE_NORMAL;
	hldr_can.Init.AutoRetransmission = ENABLE;
	hldr_can.Init.TransmitPause = DISABLE;
	hldr_can.Init.ProtocolException = DISABLE;
	hldr_can.Init.NominalPrescaler = 10U;
	hldr_can.Init.NominalSyncJumpWidth = 1U;
	hldr_can.Init.NominalTimeSeg1 = 12U;
	hldr_can.Init.NominalTimeSeg2 = 4U;
	hldr_can.Init.DataPrescaler = 10U;
	hldr_can.Init.DataSyncJumpWidth = 1U;
	hldr_can.Init.DataTimeSeg1 = 12U;
	hldr_can.Init.DataTimeSeg2 = 4U;
	hldr_can.Init.StdFiltersNbr = 8U;
	hldr_can.Init.ExtFiltersNbr = 0U;
	hldr_can.Init.TxFifoQueueMode = FDCAN_TX_QUEUE_OPERATION;
	if (HAL_FDCAN_Init(&hldr_can) != HAL_OK)
	{
		return false;
	}

	filter.IdType = FDCAN_STANDARD_ID;
	filter.FilterIndex = 0U;
	filter.FilterType = FDCAN_FILTER_RANGE;
	filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	filter.FilterID1 = (uint32_t)(LDR_REQ_BASE + node);
	filter.FilterID2 = (uint32_t)(LDR_REQ_BASE + node);
	if (HAL_FDCAN_ConfigFilter(&hldr_can, &filter) != HAL_OK)
	{
		return false;
	}
	if (HAL_FDCAN_ConfigGlobalFilter(&hldr_can, FDCAN_REJECT, FDCAN_REJECT,
									 FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
	{
		return false;
	}
	if (HAL_FDCAN_Start(&hldr_can) != HAL_OK)
	{
		return false;
	}
	if (HAL_FDCAN_ActivateNotification(&hldr_can,
									   FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_BUS_OFF,
									   0U) != HAL_OK)
	{
		return false;
	}

	HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 1U, 0U);
	HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
	return true;
}

void FDCAN1_IT0_IRQHandler(void)
{
	HAL_FDCAN_IRQHandler(&hldr_can);
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	FDCAN_RxHeaderTypeDef header;
	ldr_rx_frame_t frame;
	uint8_t next;

	(void)RxFifo0ITs;
	if (hfdcan != &hldr_can)
	{
		return;
	}
	if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &header, frame.data) != HAL_OK)
	{
		return;
	}
	if (header.IdType != FDCAN_STANDARD_ID || header.RxFrameType != FDCAN_DATA_FRAME)
	{
		return;
	}
	if (header.DataLength > 15U)
	{
		return;
	}
	if (header.Identifier != (uint32_t)(LDR_REQ_BASE + s_node))
	{
		return; /* Defense in depth; the hardware filter is already exact. */
	}

	frame.id = (uint16_t)header.Identifier;
	frame.len = k_dlc_to_len[header.DataLength];

	next = (uint8_t)((s_rx_head + 1U) % LDR_RX_RING_SIZE);
	if (next != s_rx_tail)
	{
		s_rx_ring[s_rx_head] = frame;
		__DMB();
		s_rx_head = next;
	}
	/* else: ring full, drop frame (host retries). */
}

/**
  * @brief Bus-off recovery.
  *
  * The FDCAN sets CCCR.INIT on Bus_Off and never recovers by itself; software
  * must clear INIT, after which the controller waits out 129x11 recessive bits
  * and resumes.  Runs in the FDCAN interrupt context.
  */
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
	FDCAN_ProtocolStatusTypeDef protocol_status;

	if (hfdcan != &hldr_can)
	{
		return;
	}
	if ((ErrorStatusITs & FDCAN_IT_BUS_OFF) == 0U)
	{
		return;
	}

	HAL_FDCAN_GetProtocolStatus(hfdcan, &protocol_status);
	if (protocol_status.BusOff != 0U)
	{
		CLEAR_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_INIT);
	}
}

bool loader_link_rx_pop(ldr_rx_frame_t *out)
{
	if (s_rx_tail == s_rx_head)
	{
		return false;
	}
	*out = s_rx_ring[s_rx_tail];
	s_rx_tail = (uint8_t)((s_rx_tail + 1U) % LDR_RX_RING_SIZE);
	return true;
}

void loader_link_send(const uint8_t *data, uint8_t len)
{
	FDCAN_TxHeaderTypeDef tx = {0};
	uint8_t tries = 0U;

	tx.Identifier = (uint32_t)(LDR_RSP_BASE + s_node);
	tx.IdType = FDCAN_STANDARD_ID;
	tx.TxFrameType = FDCAN_DATA_FRAME;
	tx.FDFormat = FDCAN_FD_CAN;
	tx.BitRateSwitch = FDCAN_BRS_ON;
	tx.DataLength = len_to_dlc(len);
	tx.TxEventFifoControl = FDCAN_NO_TX_EVENTS;

	while (HAL_FDCAN_AddMessageToTxFifoQ(&hldr_can, &tx, (uint8_t *)data) != HAL_OK)
	{
		if (++tries >= 10U)
		{
			return;
		}
	}
}

void loader_link_shutdown(void)
{
	(void)HAL_FDCAN_Stop(&hldr_can);
	HAL_NVIC_DisableIRQ(FDCAN1_IT0_IRQn);
}
