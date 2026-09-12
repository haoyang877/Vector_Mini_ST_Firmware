/**
  ******************************************************************************
  * @file    loader_crc.h
  * @brief   CRC-32/ISO-HDLC (zlib compatible) used for frames and images.
  ******************************************************************************
  */
#ifndef LOADER_CRC_H
#define LOADER_CRC_H

#include <stdint.h>

/** CRC-32/ISO-HDLC: poly 0xEDB88320 (reflected), init 0xFFFFFFFF, xorout 0xFFFFFFFF. */
uint32_t loader_crc32(const uint8_t *data, uint32_t length);

#endif /* LOADER_CRC_H */
