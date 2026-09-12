/**
  ******************************************************************************
  * @file    loader_main.c
  * @brief   Resident Loader entry point, session state machine, command handlers.
  *
  * Boot flow:
  *   RESET -> init -> mailbox check -> 50 ms boot window -> validate APP ->
  *   jump to APP; otherwise stay in the loader and serve CAN update requests.
  ******************************************************************************
  */
#include "stm32g4xx_hal.h"

#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#include "boot_mailbox.h"
#include "loader_link.h"
#include "loader_cfg.h"
#include "loader_crc.h"
#include "loader_flash.h"
#include "loader_jump.h"
#include "loader_proto.h"

enum
{
	LDR_ST_IDLE = 0,
	LDR_ST_MANIFEST = 1,
	LDR_ST_RECEIVING = 2,
	LDR_ST_VERIFIED = 3,
	LDR_ST_ERROR = 4
};

typedef struct
{
	uint8_t state;
	uint8_t last_error;
	uint32_t session;
	uint32_t image_size;
	uint32_t image_crc;
	uint32_t version;
	uint32_t image_type;
	uint32_t next_offset;
	uint32_t session_counter;
} ldr_ctx_t;

static ldr_ctx_t s_ctx;

/* Single-entry request/reply cache so a stop-and-wait retransmission of the
   identical request is answered from the cache without re-executing it. */
static uint8_t s_cache_valid;
static uint16_t s_cache_seq;
static uint32_t s_cache_session;
static uint8_t s_cache_op;
static uint8_t s_cache_req[64];
static uint8_t s_cache_req_len;
static uint8_t s_cache_rsp[64];
static uint8_t s_cache_rsp_len;

void SystemClock_Config(void);

void SysTick_Handler(void)
{
	HAL_IncTick();
}

void Error_Handler(void)
{
	__disable_irq();
	for (;;)
	{
	}
}

/**
  * @brief System clock: HSE 8 MHz -> PLL -> 170 MHz (same as the application).
  */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

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

	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
								  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
  * @brief Drive the six gate-driver inputs to the power-stage safe state.
  *
  * The board gate driver (FD6288Q) has no enable pin and its inputs are in
  * phase; holding every PWM input low therefore keeps all MOSFETs off while
  * the Loader runs.  The APP re-configures these pins for TIM1 when it boots.
  */
static void power_stage_safe_init(void)
{
	GPIO_InitTypeDef gpio = {0};

	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	gpio.Mode = GPIO_MODE_OUTPUT_PP;
	gpio.Pull = GPIO_NOPULL;
	gpio.Speed = GPIO_SPEED_FREQ_LOW;

	/* PWM_AL/PWM_AH/PWM_BH/PWM_CH */
	gpio.Pin = GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;
	HAL_GPIO_WritePin(GPIOA, gpio.Pin, GPIO_PIN_RESET);
	HAL_GPIO_Init(GPIOA, &gpio);

	/* PWM_BL/PWM_CL */
	gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1;
	HAL_GPIO_WritePin(GPIOB, gpio.Pin, GPIO_PIN_RESET);
	HAL_GPIO_Init(GPIOB, &gpio);
}

static void send_payload(const ldr_request_t *req, uint16_t result,
						 const uint8_t *data, uint16_t data_len)
{
	uint8_t payload[48];
	uint8_t out[64];
	uint8_t n;

	payload[0] = (uint8_t)(result >> 8);
	payload[1] = (uint8_t)result;
	if (data_len != 0U)
	{
		memcpy(&payload[2], data, (size_t)data_len);
	}

	n = ldr_build_reply(req->seq, req->session, req->opcode,
						payload, (uint16_t)(2U + data_len), out);
	if (n == 0U)
	{
		return;
	}

	/* Remember request and reply for stop-and-wait retransmissions. */
	s_cache_valid = 0U;
	__DMB();
	memcpy(s_cache_req, req->raw, (size_t)req->raw_len);
	s_cache_req_len = req->raw_len;
	memcpy(s_cache_rsp, out, (size_t)n);
	s_cache_rsp_len = n;
	s_cache_seq = req->seq;
	s_cache_session = req->session;
	s_cache_op = req->opcode;
	__DMB();
	s_cache_valid = 1U;

	loader_link_send(out, n);
}

static void handle_info(const ldr_request_t *req)
{
	uint8_t d[16];

	ldr_put_u32(&d[0], LDR_LOADER_VERSION);
	ldr_put_u32(&d[4], (uint32_t)LDR_APP_BASE);
	ldr_put_u32(&d[8], (uint32_t)LDR_APP_SIZE);
	d[12] = (uint8_t)LDR_CAN_NODE;
	d[13] = 0U;
	d[14] = 0U;
	d[15] = 0U;
	send_payload(req, LDR_RES_OK, d, 16U);
}

static void handle_status(const ldr_request_t *req)
{
	uint8_t d[20];

	d[0] = s_ctx.state;
	d[1] = s_ctx.last_error;
	d[2] = 0U;
	d[3] = 0U;
	ldr_put_u32(&d[4], s_ctx.next_offset);
	ldr_put_u32(&d[8], s_ctx.image_size);
	ldr_put_u32(&d[12], s_ctx.image_crc);
	ldr_put_u32(&d[16], s_ctx.session);
	send_payload(req, LDR_RES_OK, d, 20U);
}

static void handle_begin(const ldr_request_t *req)
{
	uint32_t size;
	uint32_t crc;
	uint32_t version;
	uint16_t image_type;
	uint8_t d[8];

	if (req->plen < 16U)
	{
		send_payload(req, LDR_RES_BAD_FRAME, NULL, 0U);
		return;
	}

	size = ldr_get_u32(&req->payload[0]);
	crc = ldr_get_u32(&req->payload[4]);
	version = ldr_get_u32(&req->payload[8]);
	image_type = ldr_get_u16(&req->payload[12]);

	if (size == 0U || size > (uint32_t)LDR_APP_SIZE || (size & 7U) != 0U)
	{
		send_payload(req, LDR_RES_BAD_SIZE, NULL, 0U);
		return;
	}

	s_ctx.session_counter++;
	s_ctx.session = ((s_ctx.session_counter << 16) ^ HAL_GetTick()) | 1U;
	s_ctx.state = LDR_ST_MANIFEST;
	s_ctx.last_error = 0U;
	s_ctx.image_size = size;
	s_ctx.image_crc = crc;
	s_ctx.version = version;
	s_ctx.image_type = (uint32_t)image_type;
	s_ctx.next_offset = 0U;

	ldr_put_u32(&d[0], s_ctx.session);
	ldr_put_u32(&d[4], (uint32_t)LDR_APP_BASE);
	send_payload(req, LDR_RES_OK, d, 8U);
}

static void handle_erase(const ldr_request_t *req)
{
	if (s_ctx.state != LDR_ST_MANIFEST &&
		s_ctx.state != LDR_ST_RECEIVING &&
		s_ctx.state != LDR_ST_VERIFIED &&
		s_ctx.state != LDR_ST_ERROR)
	{
		send_payload(req, LDR_RES_BAD_STATE, NULL, 0U);
		return;
	}

	/* Invalidate the stored record first so a power loss during erasure can
	   never boot an incomplete application. */
	if (!loader_flash_erase_record() || !loader_flash_erase_app())
	{
		s_ctx.state = LDR_ST_ERROR;
		s_ctx.last_error = (uint8_t)LDR_RES_FLASH;
		send_payload(req, LDR_RES_FLASH, NULL, 0U);
		return;
	}

	s_ctx.state = LDR_ST_RECEIVING;
	s_ctx.next_offset = 0U;
	send_payload(req, LDR_RES_OK, NULL, 0U);
}

static void handle_program(const ldr_request_t *req)
{
	uint32_t offset;
	uint16_t data_len;
	uint8_t d[4];

	if (s_ctx.state != LDR_ST_RECEIVING)
	{
		send_payload(req, LDR_RES_BAD_STATE, NULL, 0U);
		return;
	}
	if (req->plen < 8U || req->plen > (4U + LDR_MAX_CHUNK))
	{
		send_payload(req, LDR_RES_BAD_FRAME, NULL, 0U);
		return;
	}

	offset = ldr_get_u32(&req->payload[0]);
	data_len = (uint16_t)(req->plen - 4U);
	if (((data_len & 7U) != 0U) || data_len > LDR_MAX_CHUNK)
	{
		send_payload(req, LDR_RES_BAD_SIZE, NULL, 0U);
		return;
	}
	if (((offset & 7U) != 0U) || offset != s_ctx.next_offset)
	{
		send_payload(req, LDR_RES_BAD_OFFSET, NULL, 0U);
		return;
	}
	if (((offset % LDR_PAGE_SIZE) + (uint32_t)data_len) > LDR_PAGE_SIZE)
	{
		send_payload(req, LDR_RES_BAD_OFFSET, NULL, 0U);
		return;
	}
	if (!loader_flash_program(offset, &req->payload[4], (uint32_t)data_len))
	{
		s_ctx.state = LDR_ST_ERROR;
		s_ctx.last_error = (uint8_t)LDR_RES_FLASH;
		send_payload(req, LDR_RES_FLASH, NULL, 0U);
		return;
	}

	s_ctx.next_offset = offset + (uint32_t)data_len;
	ldr_put_u32(d, s_ctx.next_offset);
	send_payload(req, LDR_RES_OK, d, 4U);
}

static void handle_verify(const ldr_request_t *req)
{
	uint32_t crc;
	uint8_t d[4];

	if (s_ctx.state != LDR_ST_RECEIVING)
	{
		send_payload(req, LDR_RES_BAD_STATE, NULL, 0U);
		return;
	}
	if (s_ctx.next_offset != s_ctx.image_size)
	{
		send_payload(req, LDR_RES_BAD_SIZE, NULL, 0U);
		return;
	}

	crc = loader_crc32((const uint8_t *)LDR_APP_BASE, s_ctx.image_size);
	ldr_put_u32(d, crc);
	if (crc != s_ctx.image_crc)
	{
		s_ctx.last_error = (uint8_t)LDR_RES_VERIFY;
		send_payload(req, LDR_RES_VERIFY, d, 4U);
		return;
	}

	s_ctx.state = LDR_ST_VERIFIED;
	send_payload(req, LDR_RES_OK, d, 4U);
}

static void handle_activate(const ldr_request_t *req)
{
	ldr_app_record_t record;

	if (s_ctx.state != LDR_ST_VERIFIED)
	{
		send_payload(req, LDR_RES_BAD_STATE, NULL, 0U);
		return;
	}

	record.magic = LDR_RECORD_MAGIC;
	record.size = s_ctx.image_size;
	record.crc32 = s_ctx.image_crc;
	record.version = s_ctx.version;
	record.image_type = s_ctx.image_type;
	record.record_crc = 0U;

	if (!loader_record_write(&record))
	{
		s_ctx.state = LDR_ST_ERROR;
		s_ctx.last_error = (uint8_t)LDR_RES_FLASH;
		send_payload(req, LDR_RES_FLASH, NULL, 0U);
		return;
	}

	send_payload(req, LDR_RES_OK, NULL, 0U);
	HAL_Delay(30U);
	boot_mailbox_clear();
	loader_jump_to_app();
}

static void handle_abort(const ldr_request_t *req)
{
	s_ctx.state = LDR_ST_IDLE;
	s_ctx.session = 0U;
	s_ctx.next_offset = 0U;
	send_payload(req, LDR_RES_OK, NULL, 0U);
}

static void handle_request(const ldr_request_t *req)
{
	switch (req->opcode)
	{
	case LDR_OP_INFO:
		handle_info(req);
		break;
	case LDR_OP_STATUS:
		handle_status(req);
		break;
	case LDR_OP_BEGIN:
		handle_begin(req);
		break;
	case LDR_OP_ERASE:
		handle_erase(req);
		break;
	case LDR_OP_PROGRAM:
		handle_program(req);
		break;
	case LDR_OP_VERIFY:
		handle_verify(req);
		break;
	case LDR_OP_ACTIVATE:
		handle_activate(req);
		break;
	case LDR_OP_ABORT:
		handle_abort(req);
		break;
	default:
		send_payload(req, LDR_RES_BAD_OP, NULL, 0U);
		break;
	}
}

static void handle_frame(const ldr_rx_frame_t *frame)
{
	ldr_request_t req;

	/* The link delivers frames addressed to this node only. */
	if (!ldr_parse_request(frame->data, frame->len, &req))
	{
		return;
	}

	/* Identical retransmission of the previous request: re-send cached reply. */
	if (s_cache_valid != 0U &&
		req.seq == s_cache_seq &&
		req.session == s_cache_session &&
		req.opcode == s_cache_op &&
		req.raw_len == s_cache_req_len &&
		memcmp(req.raw, s_cache_req, (size_t)req.raw_len) == 0)
	{
		loader_link_send(s_cache_rsp, s_cache_rsp_len);
		return;
	}

	handle_request(&req);
}

int main(void)
{
	ldr_rx_frame_t frame;
	uint32_t window_start;
	uint8_t request;
	bool stay = false;

	HAL_Init();
	SystemClock_Config();

	power_stage_safe_init();

	if (!loader_link_init((uint8_t)LDR_CAN_NODE))
	{
		Error_Handler();
	}
	s_ctx.state = LDR_ST_IDLE;

	request = (uint8_t)boot_mailbox_consume();
	if (request == (uint8_t)BOOT_MAILBOX_CMD_ENTER_LOADER)
	{
		stay = true;
	}
	else
	{
		boot_mailbox_clear();
	}

	/* Short window to catch a host that resets the device into update mode. */
	if (!stay)
	{
		window_start = HAL_GetTick();
		while ((HAL_GetTick() - window_start) < LDR_BOOT_WINDOW_MS)
		{
			if (loader_link_rx_pop(&frame))
			{
				stay = true;
				handle_frame(&frame);
				break;
			}
		}
	}

	/* No request and no early frame: boot the APP if it is fully valid. */
	if (!stay)
	{
		ldr_app_record_t record;
		if (loader_record_read(&record) && loader_image_valid(&record))
		{
			boot_mailbox_clear();
			loader_jump_to_app();
		}
		/* Otherwise stay in the loader (recovery). */
	}

	for (;;)
	{
		if (loader_link_rx_pop(&frame))
		{
			handle_frame(&frame);
		}
	}
}
