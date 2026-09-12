/**
  ******************************************************************************
  * @file    loader_proto.h
  * @brief   Loader v1 frame parsing/building (pure functions, no I/O).
  ******************************************************************************
  */
#ifndef LOADER_PROTO_H
#define LOADER_PROTO_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
	uint8_t opcode;
	uint8_t flags;
	uint16_t seq;
	uint16_t plen;
	uint32_t session;
	const uint8_t *payload;
	const uint8_t *raw;
	uint8_t raw_len;
} ldr_request_t;

/** Validate magic/version/length/CRC. Returns false for any malformed frame. */
bool ldr_parse_request(const uint8_t *raw, uint8_t len, ldr_request_t *out);

/** Build a padded reply frame; returns its DLC byte length (16..64). */
uint8_t ldr_build_reply(uint16_t seq, uint32_t session, uint8_t opcode,
						const uint8_t *payload, uint16_t plen, uint8_t *out);

/** Big-endian field helpers. */
uint16_t ldr_get_u16(const uint8_t *p);
uint32_t ldr_get_u32(const uint8_t *p);
void ldr_put_u16(uint8_t *p, uint16_t v);
void ldr_put_u32(uint8_t *p, uint32_t v);

#endif /* LOADER_PROTO_H */
