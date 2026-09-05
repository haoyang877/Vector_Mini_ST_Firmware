#include "memory_layout_profile.h"

static const MemoryLayoutProfile VectorMiniStMemoryLayout =
{
	.profile_id = MEMORY_LAYOUT_PROFILE_VECTOR_MINI_ST,
	.flash_base_address = 0x08000000UL,
	.application_start_address = 0x08000000UL,
	.application_size_bytes = 0x0001C000UL,
	.flash_page_size_bytes = 2048U,
	.parameter_slot_size_bytes = 8192U,
	.parameter_slot_0_address = 0x0801C000UL,
	.parameter_slot_1_address = 0x0801E000UL
};

const MemoryLayoutProfile *MemoryLayoutProfile_GetActive(void)
{
	return &VectorMiniStMemoryLayout;
}
