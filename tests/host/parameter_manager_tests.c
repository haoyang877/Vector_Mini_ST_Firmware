#include "parameter_manager.h"

#include <string.h>

#define TEST_CHECK(condition) do { if (!(condition)) return __LINE__; } while (0)
#define TEST_SLOT_SIZE 1024U

typedef struct
{
	unsigned char slot[PARAMETER_STORE_SLOT_COUNT][TEST_SLOT_SIZE];
} TestStore;

static bool TestStore_Read(void *raw_context, uint8_t slot, uint32_t offset,
	void *destination, uint32_t size_bytes)
{
	TestStore *store = (TestStore *)raw_context;
	if (store == 0 || slot >= PARAMETER_STORE_SLOT_COUNT ||
		offset + size_bytes > TEST_SLOT_SIZE)
		return false;
	memcpy(destination, &store->slot[slot][offset], size_bytes);
	return true;
}

static bool TestStore_Erase(void *raw_context, uint8_t slot)
{
	TestStore *store = (TestStore *)raw_context;
	if (store == 0 || slot >= PARAMETER_STORE_SLOT_COUNT)
		return false;
	memset(store->slot[slot], 0xFF, TEST_SLOT_SIZE);
	return true;
}

static bool TestStore_Program(void *raw_context, uint8_t slot, uint32_t offset,
	const void *source, uint32_t size_bytes)
{
	TestStore *store = (TestStore *)raw_context;
	const unsigned char *source_bytes = (const unsigned char *)source;
	uint32_t index;
	if (store == 0 || source == 0 || slot >= PARAMETER_STORE_SLOT_COUNT ||
		offset + size_bytes > TEST_SLOT_SIZE)
		return false;
	for (index = 0U; index < size_bytes; ++index)
	{
		unsigned char *destination = &store->slot[slot][offset + index];
		if ((*destination & source_bytes[index]) != source_bytes[index])
			return false;
		*destination &= source_bytes[index];
	}
	return true;
}

int ParameterManager_RunHostTests(void)
{
	TestStore memory;
	ParameterStorePort store = {0};
	ParameterCompatibility compatibility = {0};
	ParameterManagerContext writer;
	ParameterManagerContext reader;
	uint32_t written = 0x12345678UL;
	uint32_t loaded = 0U;

	memset(&memory, 0xFF, sizeof(memory));
	store.context = &memory;
	store.read = TestStore_Read;
	store.erase = TestStore_Erase;
	store.program = TestStore_Program;
	compatibility.product_id = 1U;
	compatibility.hardware_profile_id = 2U;
	compatibility.motor_profile_id = 3U;
	compatibility.parameter_schema_version = 4U;
	compatibility.configuration_fingerprint = 0xA5A55A5AUL;
	ParameterManager_Initialize(&writer, &store, &compatibility,
		sizeof(written));
	TEST_CHECK(ParameterManager_Save(&writer, &written));
	ParameterManager_Initialize(&reader, &store, &compatibility,
		sizeof(loaded));
	TEST_CHECK(ParameterManager_Load(&reader, &loaded));
	TEST_CHECK(loaded == written);

	compatibility.configuration_fingerprint++;
	ParameterManager_Initialize(&reader, &store, &compatibility,
		sizeof(loaded));
	TEST_CHECK(!ParameterManager_Load(&reader, &loaded));
	return 0;
}
