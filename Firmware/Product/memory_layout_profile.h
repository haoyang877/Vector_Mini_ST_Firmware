#ifndef PRODUCT_MEMORY_LAYOUT_PROFILE_H
#define PRODUCT_MEMORY_LAYOUT_PROFILE_H

#include <stdint.h>

#define MEMORY_LAYOUT_PROFILE_VECTOR_MINI_ST 1U

typedef struct
{
	uint16_t profile_id;
	uint32_t flash_base_address;
	uint32_t application_start_address;
	uint32_t application_size_bytes;
	uint32_t flash_page_size_bytes;
	uint32_t parameter_slot_size_bytes;
	uint32_t parameter_slot_0_address;
	uint32_t parameter_slot_1_address;
} MemoryLayoutProfile;

const MemoryLayoutProfile *MemoryLayoutProfile_GetActive(void);

#endif
