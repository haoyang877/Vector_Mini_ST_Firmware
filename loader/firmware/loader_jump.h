/**
  ******************************************************************************
  * @file    loader_jump.h
  * @brief   APP image validation and boot jump for the resident Loader.
  ******************************************************************************
  */
#ifndef LOADER_JUMP_H
#define LOADER_JUMP_H

#include <stdbool.h>

#include "loader_flash.h"

/** Full validation: record fields, whole-image CRC, MSP and reset vector. */
bool loader_image_valid(const ldr_app_record_t *record);

/** Jump to the APP. Never returns on success; resets on a returned jump. */
void loader_jump_to_app(void);

#endif /* LOADER_JUMP_H */
