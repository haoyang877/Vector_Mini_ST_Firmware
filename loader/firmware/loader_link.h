/**
  ******************************************************************************
  * @file    loader_link.h
  * @brief   Transport seam of the resident Loader.
  *
  * The Loader core (loader_main/loader_jump) talks to the host only through
  * this interface.  loader_can.c implements it for CAN-FD; an RS-485/UART or
  * a different MCU port provides another implementation without touching the
  * core.  This header must stay free of vendor-specific types.
  ******************************************************************************
  */
#ifndef LOADER_LINK_H
#define LOADER_LINK_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
	uint16_t id; /* transport-level address of the frame (CAN ID today) */
	uint8_t len;
	uint8_t data[64];
} ldr_rx_frame_t;

/**
  * @brief Configure the transport for `node`.
  * @retval false on a fatal init error (caller halts; see loader_main.c).
  */
bool loader_link_init(uint8_t node);

/** Pop one received frame (ISR fills a small ring). Returns false when empty. */
bool loader_link_rx_pop(ldr_rx_frame_t *out);

/** Best-effort non-blocking send of one reply frame to the host. */
void loader_link_send(const uint8_t *data, uint8_t len);

/** Quiesce the transport (no IRQs, no RX/TX) before jumping to the APP. */
void loader_link_shutdown(void);

#endif /* LOADER_LINK_H */
