/**
  ******************************************************************************
  * @file    loader_crc.c
  * @brief   Bitwise CRC-32/ISO-HDLC (no lookup table) - matches Python zlib.crc32.
  *          Check vector: crc32("123456789") = 0xCBF43926.
  ******************************************************************************
  */
#include "loader_crc.h"

uint32_t loader_crc32(const uint8_t *data, uint32_t length)
{
	uint32_t crc = 0xFFFFFFFFU;
	uint32_t i;
	uint8_t bit;

	for (i = 0U; i < length; ++i)
	{
		crc ^= (uint32_t)data[i];
		for (bit = 0U; bit < 8U; ++bit)
		{
			if (crc & 1U)
			{
				crc = (crc >> 1) ^ 0xEDB88320U;
			}
			else
			{
				crc = (crc >> 1);
			}
		}
	}
	return ~crc;
}
