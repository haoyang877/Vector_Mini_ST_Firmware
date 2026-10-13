#ifndef LOADER_CFG_H
#define LOADER_CFG_H
#include "boot_contract.h"
#define LDR_REQ_BASE 0x7D0U
#define LDR_RSP_BASE 0x7E0U
#define LDR_MAGIC 0xB1U
#define LDR_VERSION 2U
#define LDR_HEADER_SIZE 12U
#define LDR_CRC_SIZE 4U
#define LDR_MAX_PAYLOAD 48U
#define LDR_MAX_CHUNK 40U
#define LDR_BOOT_WINDOW_MS 50U
#define LDR_OP_INFO 1U
#define LDR_OP_STATUS 2U
#define LDR_OP_UID 3U
#define LDR_OP_BEGIN 0x20U
#define LDR_OP_ERASE 0x21U
#define LDR_OP_PROGRAM 0x22U
#define LDR_OP_VERIFY 0x23U
#define LDR_OP_ACTIVATE 0x24U
#define LDR_OP_ABORT 0x25U
#endif
