/**
  ******************************************************************************
  * @file    loader_jump.c
  * @brief   APP image validation and boot jump for the resident Loader.
  ******************************************************************************
  */
#include "loader_jump.h"

#include "loader_link.h"
#include "loader_cfg.h"
#include "loader_crc.h"
#include "stm32g4xx_hal.h"

bool loader_image_valid(const ldr_app_record_t *record)
{
	uint32_t sp;
	uint32_t reset_vector;
	uint32_t reset_addr;
	uint32_t crc;

	if (record->size == 0U || record->size > LDR_APP_SIZE)
	{
		return false;
	}
	if ((record->size & 7U) != 0U)
	{
		return false;
	}

	crc = loader_crc32((const uint8_t *)LDR_APP_BASE, record->size);
	if (crc != record->crc32)
	{
		return false;
	}

	/* Cortex-M vector table checks. */
	sp = *(volatile uint32_t *)LDR_APP_BASE;
	reset_vector = *(volatile uint32_t *)(LDR_APP_BASE + 4U);

	if ((sp & 7U) != 0U)
	{
		return false;
	}
	if (sp < 0x20000020UL || sp > 0x20008000UL)
	{
		return false;
	}
	if ((reset_vector & 1U) == 0U)
	{
		return false;
	}
	reset_addr = reset_vector & 0xFFFFFFFEUL;
	/* Reset handler must be within the APP region (no range wrapping). */
	if (reset_addr < LDR_APP_BASE)
	{
		return false;
	}
	if (reset_addr >= (LDR_APP_BASE + record->size))
	{
		return false;
	}
	return true;
}

void loader_jump_to_app(void)
{
	uint32_t sp;
	uint32_t reset_vector;
	uint32_t i;

	__disable_irq();

	/* Stop time base and transport, clear all NVIC state. */
	SysTick->CTRL = 0U;
	SysTick->LOAD = 0U;
	SysTick->VAL = 0U;
	loader_link_shutdown();
	for (i = 0U; i < 8U; ++i)
	{
		NVIC->ICER[i] = 0xFFFFFFFFU;
		NVIC->ICPR[i] = 0xFFFFFFFFU;
	}

	SCB->VTOR = LDR_APP_BASE;
	__DSB();
	__ISB();

	sp = *(volatile uint32_t *)LDR_APP_BASE;
	reset_vector = *(volatile uint32_t *)(LDR_APP_BASE + 4U);
	__set_MSP(sp);
	__DSB();
	__ISB();

	/* A direct jump preserves PRIMASK; clear it so the APP's interrupts run. */
	__enable_irq();

	((void (*)(void))reset_vector)();

	/* A returned jump is a failure: restart into the loader. */
	NVIC_SystemReset();
}
