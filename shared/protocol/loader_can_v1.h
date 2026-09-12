/**
  ******************************************************************************
  * @file    loader_can_v1.h
  * @brief   Single source of truth for the resident-Loader CAN-FD upgrade
  *          protocol (v1): transport IDs, frame constants, commands, result
  *          codes, PROGRAM chunking rules and the boot window.
  *
  * Consumed by the Loader firmware (through loader_cfg.h) and the host tooling
  * (loader/tools/loader_proto.py); tests/test_protocol_single_source.py keeps
  * both ends in sync.  A wire-incompatible change requires bumping
  * LOADER_CAN_PROTOCOL_VERSION: both ends reject frames whose version byte
  * differs, so old and new peers fail loudly instead of misparsing.
  ******************************************************************************
  */
#ifndef LOADER_CAN_V1_H
#define LOADER_CAN_V1_H

/* Transport: request 0x7D0+node, response 0x7E0+node (standard 11-bit IDs) */
#define LOADER_CAN_REQ_ID_BASE        0x7D0U
#define LOADER_CAN_RSP_ID_BASE        0x7E0U

/* Frame: magic | version | op | flags | seq BE16 | plen BE16 | session BE32 |
          payload | CRC32 BE (ISO-HDLC), 0xFF-padded to the next legal DLC */
#define LOADER_CAN_MAGIC              0xB1U
#define LOADER_CAN_PROTOCOL_VERSION   0x01U
#define LOADER_CAN_HEADER_SIZE        12U
#define LOADER_CAN_CRC_SIZE           4U
#define LOADER_CAN_MAX_PAYLOAD        48U

/* Commands */
#define LOADER_CAN_OP_GET_INFO        0x01U
#define LOADER_CAN_OP_GET_STATUS      0x02U
#define LOADER_CAN_OP_BEGIN           0x20U
#define LOADER_CAN_OP_ERASE           0x21U
#define LOADER_CAN_OP_PROGRAM         0x22U
#define LOADER_CAN_OP_VERIFY          0x23U
#define LOADER_CAN_OP_ACTIVATE        0x24U
#define LOADER_CAN_OP_ABORT           0x25U

/* Result codes: first two reply payload bytes, big endian */
#define LOADER_CAN_RES_OK             0x0000U
#define LOADER_CAN_RES_BAD_FRAME      0x0001U
#define LOADER_CAN_RES_BAD_CRC        0x0002U
#define LOADER_CAN_RES_BAD_STATE      0x0003U
#define LOADER_CAN_RES_BAD_OFFSET     0x0004U
#define LOADER_CAN_RES_BAD_SIZE       0x0005U
#define LOADER_CAN_RES_FLASH          0x0006U
#define LOADER_CAN_RES_VERIFY         0x0007U
#define LOADER_CAN_RES_BAD_OP         0x0008U

/* PROGRAM chunking: 8-byte aligned, <= 40 B, must not cross a 2 KiB page */
#define LOADER_CAN_CHUNK_ALIGN        8U
#define LOADER_CAN_MAX_CHUNK          40U

/* Reset-to-APP boot window: any Loader frame inside it keeps the Loader
   resident instead of booting the APP (host rescue path). */
#define LOADER_CAN_BOOT_WINDOW_MS     50U

#endif /* LOADER_CAN_V1_H */
