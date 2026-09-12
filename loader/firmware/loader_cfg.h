/**
  ******************************************************************************
  * @file    loader_cfg.h
  * @brief   Loader-local configuration.
  *
  * Every cross-firmware constant now lives in shared/:
  *   shared/flash_layout/flash_layout.h   partitions, record magic, RAM window
  *   shared/protocol/loader_can_v1.h      upgrade protocol v1 constants
  *
  * The LDR_* names below are aliases kept for the loader sources.  Never put a
  * literal copy here: tests/test_protocol_single_source.py parses this header
  * and fails when an alias turns back into a number or drifts from shared/.
  ******************************************************************************
  */
#ifndef LOADER_CFG_H
#define LOADER_CFG_H

#include <stdint.h>

#include "flash_layout/flash_layout.h"
#include "protocol/loader_can_v1.h"

/* Loader-local identity */
#define LDR_CAN_NODE 0U
#define LDR_LOADER_VERSION 0x00010000UL

/* Flash layout aliases */
#define LDR_FLASH_BASE FLASH_LAYOUT_BASE
#define LDR_PAGE_SIZE FLASH_LAYOUT_PAGE_SIZE
#define LDR_APP_BASE FLASH_LAYOUT_APP_BASE
#define LDR_APP_SIZE FLASH_LAYOUT_APP_SIZE
#define LDR_APP_FIRST_PAGE FLASH_LAYOUT_APP_FIRST_PAGE
#define LDR_APP_PAGES FLASH_LAYOUT_APP_PAGES
#define LDR_RECORD_ADDR FLASH_LAYOUT_RECORD_BASE
#define LDR_RECORD_PAGE FLASH_LAYOUT_RECORD_PAGE
#define LDR_RECORD_MAGIC FLASH_LAYOUT_RECORD_MAGIC

/* Protocol aliases */
#define LDR_REQ_BASE LOADER_CAN_REQ_ID_BASE
#define LDR_RSP_BASE LOADER_CAN_RSP_ID_BASE
#define LDR_MAGIC LOADER_CAN_MAGIC
#define LDR_VERSION LOADER_CAN_PROTOCOL_VERSION
#define LDR_HEADER_SIZE LOADER_CAN_HEADER_SIZE
#define LDR_CRC_SIZE LOADER_CAN_CRC_SIZE
#define LDR_MAX_PAYLOAD LOADER_CAN_MAX_PAYLOAD
#define LDR_OP_INFO LOADER_CAN_OP_GET_INFO
#define LDR_OP_STATUS LOADER_CAN_OP_GET_STATUS
#define LDR_OP_BEGIN LOADER_CAN_OP_BEGIN
#define LDR_OP_ERASE LOADER_CAN_OP_ERASE
#define LDR_OP_PROGRAM LOADER_CAN_OP_PROGRAM
#define LDR_OP_VERIFY LOADER_CAN_OP_VERIFY
#define LDR_OP_ACTIVATE LOADER_CAN_OP_ACTIVATE
#define LDR_OP_ABORT LOADER_CAN_OP_ABORT
#define LDR_RES_OK LOADER_CAN_RES_OK
#define LDR_RES_BAD_FRAME LOADER_CAN_RES_BAD_FRAME
#define LDR_RES_BAD_CRC LOADER_CAN_RES_BAD_CRC
#define LDR_RES_BAD_STATE LOADER_CAN_RES_BAD_STATE
#define LDR_RES_BAD_OFFSET LOADER_CAN_RES_BAD_OFFSET
#define LDR_RES_BAD_SIZE LOADER_CAN_RES_BAD_SIZE
#define LDR_RES_FLASH LOADER_CAN_RES_FLASH
#define LDR_RES_VERIFY LOADER_CAN_RES_VERIFY
#define LDR_RES_BAD_OP LOADER_CAN_RES_BAD_OP
#define LDR_MAX_CHUNK LOADER_CAN_MAX_CHUNK
#define LDR_BOOT_WINDOW_MS LOADER_CAN_BOOT_WINDOW_MS

#endif /* LOADER_CFG_H */
