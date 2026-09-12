/**
  ******************************************************************************
  * @file    boot_mailbox.h
  * @brief   Shared boot handoff mailbox between the resident Loader and the APP.
  *
  * Located in the first 32 bytes of SRAM (0x20000000..0x2000001F). Both firmwares
  * link their RW/ZI regions starting at 0x20000020, so the mailbox is never
  * zero-initialized by startup code and survives a system reset (not power loss).
  ******************************************************************************
  */
#ifndef BOOT_MAILBOX_H
#define BOOT_MAILBOX_H

#include <stdint.h>

#define BOOT_MAILBOX_ADDR 0x20000000UL
#define BOOT_MAILBOX_MAGIC 0xB00710ADUL

#define BOOT_MAILBOX_CMD_ENTER_LOADER 1UL

typedef struct
{
	volatile uint32_t magic;
	volatile uint32_t command;
	volatile uint32_t arg;
	volatile uint32_t reserved[5];
} boot_mailbox_t;

#define BOOT_MAILBOX ((volatile boot_mailbox_t *)BOOT_MAILBOX_ADDR)

static __inline void boot_mailbox_request(uint32_t command)
{
	BOOT_MAILBOX->arg = 0U;
	BOOT_MAILBOX->command = command;
	__DSB();
	BOOT_MAILBOX->magic = BOOT_MAILBOX_MAGIC;
	__DSB();
}

static __inline uint32_t boot_mailbox_consume(void)
{
	uint32_t command = 0U;
	if (BOOT_MAILBOX->magic == BOOT_MAILBOX_MAGIC)
	{
		command = BOOT_MAILBOX->command;
		BOOT_MAILBOX->magic = 0U;
		BOOT_MAILBOX->command = 0U;
		BOOT_MAILBOX->arg = 0U;
	}
	return command;
}

static __inline void boot_mailbox_clear(void)
{
	BOOT_MAILBOX->magic = 0U;
	BOOT_MAILBOX->command = 0U;
	BOOT_MAILBOX->arg = 0U;
}

#endif /* BOOT_MAILBOX_H */
