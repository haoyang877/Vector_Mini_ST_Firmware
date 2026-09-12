/**
  ******************************************************************************
  * @file    flash_layout.h
  * @brief   Single source of truth for the STM32G431CB flash/RAM partition.
  *
  * Shared by the resident Loader, the APP (scatter guards, parameter region)
  * and the host tooling under loader/tools/.  Do not copy these numbers into
  * other files: `tests/test_protocol_single_source.py` parses this header and
  * fails when the Keil scatter/project files or the Python constants drift.
  *
  * Flash (128 KiB, page = 2 KiB):
  *   0x08000000 - 0x08002FFF  Loader            (12 KiB, pages 0-5)
  *   0x08003000 - 0x080037FF  APP record page   (2 KiB,  page 6)
  *   0x08003800 - 0x08003FFF  spare             (2 KiB,  page 7)
  *   0x08004000 - 0x0801BFFF  APP               (96 KiB, pages 8-55)
  *   0x0801C000 - 0x0801FFFF  parameters        (16 KiB, pages 56-63)
  *
  * RAM: the first 32 bytes are the Loader/APP boot mailbox (boot_mailbox.h);
  * both firmwares link their RW/ZI after it.
  ******************************************************************************
  */
#ifndef FLASH_LAYOUT_H
#define FLASH_LAYOUT_H

/* Flash geometry */
#define FLASH_LAYOUT_BASE              0x08000000UL
#define FLASH_LAYOUT_PAGE_SIZE         0x00000800UL /* 2 KiB */

/* Partitions */
#define FLASH_LAYOUT_LOADER_BASE       0x08000000UL
#define FLASH_LAYOUT_LOADER_SIZE       0x00003000UL /* 12 KiB: pages 0-5 */
#define FLASH_LAYOUT_RECORD_BASE       0x08003000UL /* APP record page, page 6 */
#define FLASH_LAYOUT_SPARE_BASE        0x08003800UL /* spare page, page 7 */
#define FLASH_LAYOUT_APP_BASE          0x08004000UL
#define FLASH_LAYOUT_APP_SIZE          0x00018000UL /* 96 KiB: pages 8-55 */
#define FLASH_LAYOUT_PARAM_BASE        0x0801C000UL /* never written by the Loader */
#define FLASH_LAYOUT_PARAM_SIZE        0x00004000UL /* 16 KiB: pages 56-63 */

/* APP record page magic ("APP1") */
#define FLASH_LAYOUT_RECORD_MAGIC      0x41505031UL

/* Derived page indices */
#define FLASH_LAYOUT_APP_FIRST_PAGE    ((FLASH_LAYOUT_APP_BASE - FLASH_LAYOUT_BASE) / FLASH_LAYOUT_PAGE_SIZE)
#define FLASH_LAYOUT_APP_PAGES         (FLASH_LAYOUT_APP_SIZE / FLASH_LAYOUT_PAGE_SIZE)
#define FLASH_LAYOUT_RECORD_PAGE       ((FLASH_LAYOUT_RECORD_BASE - FLASH_LAYOUT_BASE) / FLASH_LAYOUT_PAGE_SIZE)

/* RAM window: mailbox + firmware RW/ZI (stack top = RAM_BASE + RAM_SIZE) */
#define FLASH_LAYOUT_RAM_BASE          0x20000000UL
#define FLASH_LAYOUT_RAM_SIZE          0x00008000UL /* 32 KiB */
#define FLASH_LAYOUT_RAM_MAILBOX_SIZE  0x00000020UL /* 32 B: see boot_mailbox.h */
#define FLASH_LAYOUT_RAM_FW_BASE       (FLASH_LAYOUT_RAM_BASE + FLASH_LAYOUT_RAM_MAILBOX_SIZE)
#define FLASH_LAYOUT_RAM_FW_SIZE       (FLASH_LAYOUT_RAM_SIZE - FLASH_LAYOUT_RAM_MAILBOX_SIZE)

#endif /* FLASH_LAYOUT_H */
