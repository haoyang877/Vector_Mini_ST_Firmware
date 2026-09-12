/**
  ******************************************************************************
  * @file    loader_proto.c
  * @brief   Loader v1 frame parsing/building.
  *
  * Frame: [0]=0xB1 [1]=0x01 [2]=op [3]=flags [4..5]=seq BE16 [6..7]=plen BE16
  *        [8..11]=session BE32 [12..12+plen)=payload [next 4]=CRC32 BE
  *        remaining bytes up to DLC: 0xFF padding.
  ******************************************************************************
  */
#include "loader_proto.h"

#include <string.h>

#include "loader_cfg.h"
#include "loader_crc.h"

uint16_t ldr_get_u16(const uint8_t *p)
{
	return (uint16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

uint32_t ldr_get_u32(const uint8_t *p)
{
	return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
		   ((uint32_t)p[2] << 8) | (uint32_t)p[3];
}

void ldr_put_u16(uint8_t *p, uint16_t v)
{
	p[0] = (uint8_t)(v >> 8);
	p[1] = (uint8_t)v;
}

void ldr_put_u32(uint8_t *p, uint32_t v)
{
	p[0] = (uint8_t)(v >> 24);
	p[1] = (uint8_t)(v >> 16);
	p[2] = (uint8_t)(v >> 8);
	p[3] = (uint8_t)v;
}

bool ldr_parse_request(const uint8_t *raw, uint8_t len, ldr_request_t *out)
{
	uint16_t plen;
	uint32_t need;
	uint32_t crc_rx;
	uint32_t crc_calc;

	if ((raw == NULL) || (out == NULL) || (len < 16U))
	{
		return false;
	}
	if (raw[0] != LDR_MAGIC || raw[1] != LDR_VERSION)
	{
		return false;
	}

	plen = ldr_get_u16(&raw[6]);
	if (plen > LDR_MAX_PAYLOAD)
	{
		return false;
	}
	need = LDR_HEADER_SIZE + (uint32_t)plen + LDR_CRC_SIZE;
	if ((uint32_t)len < need)
	{
		return false;
	}

	crc_rx = ldr_get_u32(&raw[LDR_HEADER_SIZE + plen]);
	crc_calc = loader_crc32(raw, LDR_HEADER_SIZE + (uint32_t)plen);
	if (crc_rx != crc_calc)
	{
		return false;
	}

	out->opcode = raw[2];
	out->flags = raw[3];
	out->seq = ldr_get_u16(&raw[4]);
	out->plen = plen;
	out->session = ldr_get_u32(&raw[8]);
	out->payload = &raw[LDR_HEADER_SIZE];
	out->raw = raw;
	out->raw_len = len;
	return true;
}

static uint8_t padded_dlc(uint32_t need)
{
	if (need <= 16U)
		return 16U;
	if (need <= 20U)
		return 20U;
	if (need <= 24U)
		return 24U;
	if (need <= 32U)
		return 32U;
	if (need <= 48U)
		return 48U;
	return 64U;
}

uint8_t ldr_build_reply(uint16_t seq, uint32_t session, uint8_t opcode,
						const uint8_t *payload, uint16_t plen, uint8_t *out)
{
	uint32_t total;
	uint32_t crc;
	uint8_t dlc;

	if (plen > LDR_MAX_PAYLOAD)
	{
		return 0U;
	}

	out[0] = LDR_MAGIC;
	out[1] = LDR_VERSION;
	out[2] = opcode;
	out[3] = 0U;
	ldr_put_u16(&out[4], seq);
	ldr_put_u16(&out[6], plen);
	ldr_put_u32(&out[8], session);
	if (plen != 0U)
	{
		memcpy(&out[LDR_HEADER_SIZE], payload, plen);
	}

	crc = loader_crc32(out, LDR_HEADER_SIZE + (uint32_t)plen);
	ldr_put_u32(&out[LDR_HEADER_SIZE + plen], crc);

	total = LDR_HEADER_SIZE + (uint32_t)plen + LDR_CRC_SIZE;
	dlc = padded_dlc(total);
	if (total < (uint32_t)dlc)
	{
		memset(&out[total], 0xFF, (uint32_t)dlc - total);
	}
	return dlc;
}
