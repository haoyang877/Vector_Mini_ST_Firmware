#include "Core/Infrastructure/Parameters/parameter_manager.h"

#include <string.h>

#define TEST_CHECK(condition) do { if (!(condition)) return __LINE__; } while (0)
#define TEST_SLOT_SIZE 1024U
#define TEST_PROGRAM_ALIGNMENT 8U

typedef struct
{
	unsigned char bytes[PARAMETER_MANAGER_SLOT_COUNT * TEST_SLOT_SIZE];
	uint32_t program_call_count;
	uint32_t fail_program_call;
	uint32_t corrupt_after_program_call;
} TestStore;

static BspResult TestStore_Read(void *raw_context, uint32_t offset,
	void *destination, size_t size_bytes)
{
	TestStore *store = (TestStore *)raw_context;
	if (store == 0 || destination == 0 ||
		offset > sizeof(store->bytes) ||
		size_bytes > sizeof(store->bytes) - offset)
		return BSP_RESULT_INVALID_ARGUMENT;
	memcpy(destination, &store->bytes[offset], size_bytes);
	return BSP_RESULT_OK;
}

static BspResult TestStore_Erase(void *raw_context, uint32_t offset,
	size_t size_bytes)
{
	TestStore *store = (TestStore *)raw_context;
	if (store == 0 || offset > sizeof(store->bytes) ||
		size_bytes > sizeof(store->bytes) - offset)
		return BSP_RESULT_INVALID_ARGUMENT;
	memset(&store->bytes[offset], 0xFF, size_bytes);
	return BSP_RESULT_OK;
}

static BspResult TestStore_Program(void *raw_context, uint32_t offset,
	const void *source, size_t size_bytes)
{
	TestStore *store = (TestStore *)raw_context;
	const unsigned char *source_bytes = (const unsigned char *)source;
	size_t index;
	if (store == 0 || source == 0 || offset > sizeof(store->bytes) ||
		size_bytes > sizeof(store->bytes) - offset ||
		offset % TEST_PROGRAM_ALIGNMENT != 0U)
		return BSP_RESULT_INVALID_ARGUMENT;
	store->program_call_count++;
	if (store->fail_program_call != 0U &&
		store->program_call_count == store->fail_program_call)
		return BSP_RESULT_IO_ERROR;
	for (index = 0U; index < size_bytes; ++index)
	{
		unsigned char *destination = &store->bytes[offset + index];
		if ((*destination & source_bytes[index]) != source_bytes[index])
			return BSP_RESULT_IO_ERROR;
		*destination &= source_bytes[index];
	}
	if (store->corrupt_after_program_call != 0U &&
		store->program_call_count == store->corrupt_after_program_call)
	{
		uint32_t slot_offset = (offset / TEST_SLOT_SIZE) * TEST_SLOT_SIZE;
		store->bytes[slot_offset + 48U] ^= 0x01U;
	}
	return BSP_RESULT_OK;
}

static int ParameterManager_CheckGoldenRecord(void)
{
	static const unsigned char GoldenRecord[] =
	{
		0x52U, 0x4DU, 0x50U, 0x56U, 0x01U, 0x00U, 0x30U, 0x00U,
		0x04U, 0x00U, 0x00U, 0x00U, 0x07U, 0x00U, 0x00U, 0x00U,
		0xD2U, 0x87U, 0x6DU, 0xAFU, 0x5AU, 0x5AU, 0xA5U, 0xA5U,
		0x01U, 0x00U, 0x00U, 0x00U, 0x02U, 0x00U, 0x00U, 0x00U,
		0x03U, 0x00U, 0x00U, 0x00U, 0x04U, 0x00U, 0x00U, 0x00U,
		0x54U, 0x4DU, 0x4FU, 0x43U, 0xABU, 0xB2U, 0xB0U, 0xBCU,
		0x78U, 0x56U, 0x34U, 0x12U
	};
	TestStore memory;
	BspNonvolatileStoragePort store = {0};
	ParameterCompatibility compatibility = {0};
	ParameterManagerContext manager;
	uint32_t payload = 0U;

	/* This literal freezes the deployed little-endian v1 ABI: header size 48,
	 * commit offset 40, payload offset 48, and CRC32 0xAF6D87D2. */
	memset(&memory, 0, sizeof(memory));
	memset(memory.bytes, 0xFF, sizeof(memory.bytes));
	memcpy(memory.bytes, GoldenRecord, sizeof(GoldenRecord));
	store.context = &memory;
	store.read = TestStore_Read;
	store.erase = TestStore_Erase;
	store.program = TestStore_Program;
	store.geometry.capacity_bytes = sizeof(memory.bytes);
	store.geometry.erase_size_bytes = TEST_SLOT_SIZE;
	store.geometry.program_alignment_bytes = TEST_PROGRAM_ALIGNMENT;
	compatibility.product_id = 1U;
	compatibility.hardware_profile_id = 2U;
	compatibility.motor_profile_id = 3U;
	compatibility.parameter_schema_version = 4U;
	compatibility.configuration_fingerprint = 0xA5A55A5AUL;
	ParameterManager_Initialize(&manager, &store, &compatibility,
		sizeof(payload));
	TEST_CHECK(manager.is_initialized);
	TEST_CHECK(ParameterManager_Load(&manager, &payload));
	TEST_CHECK(payload == 0x12345678UL);
	TEST_CHECK(manager.active_slot == 0U);
	TEST_CHECK(manager.active_sequence == 7U);
	return 0;
}

int ParameterManager_RunHostTests(void)
{
	TestStore memory;
	BspNonvolatileStoragePort store = {0};
	ParameterCompatibility compatibility = {0};
	ParameterManagerContext writer;
	ParameterManagerContext reader;
	uint32_t written = 0x12345678UL;
	uint32_t loaded = 0U;
	BspNonvolatileStoragePort invalid_store;

	memset(&memory, 0, sizeof(memory));
	memset(memory.bytes, 0xFF, sizeof(memory.bytes));
	store.context = &memory;
	store.read = TestStore_Read;
	store.erase = TestStore_Erase;
	store.program = TestStore_Program;
	store.geometry.capacity_bytes = sizeof(memory.bytes);
	store.geometry.erase_size_bytes = TEST_SLOT_SIZE;
	store.geometry.program_alignment_bytes = TEST_PROGRAM_ALIGNMENT;
	compatibility.product_id = 1U;
	compatibility.hardware_profile_id = 2U;
	compatibility.motor_profile_id = 3U;
	compatibility.parameter_schema_version = 4U;
	compatibility.configuration_fingerprint = 0xA5A55A5AUL;
	ParameterManager_Initialize(&writer, &store, &compatibility,
		sizeof(written));
	TEST_CHECK(writer.is_initialized);
	TEST_CHECK(ParameterManager_Save(&writer, &written));
	TEST_CHECK(memory.bytes[0] == 0xFFU);
	TEST_CHECK(memory.bytes[TEST_SLOT_SIZE] != 0xFFU);
	ParameterManager_Initialize(&reader, &store, &compatibility,
		sizeof(loaded));
	TEST_CHECK(ParameterManager_Load(&reader, &loaded));
	TEST_CHECK(loaded == written);

	written = 0x87654321UL;
	TEST_CHECK(ParameterManager_Save(&writer, &written));
	ParameterManager_Initialize(&reader, &store, &compatibility,
		sizeof(loaded));
	TEST_CHECK(ParameterManager_Load(&reader, &loaded));
	TEST_CHECK(loaded == written);

	/* A failed commit to the inactive slot must leave the previous slot valid. */
	memory.fail_program_call = memory.program_call_count + 3U;
	written = 0xABCDEF01UL;
	TEST_CHECK(!ParameterManager_Save(&writer, &written));
	ParameterManager_Initialize(&reader, &store, &compatibility,
		sizeof(loaded));
	TEST_CHECK(ParameterManager_Load(&reader, &loaded));
	TEST_CHECK(loaded == 0x87654321UL);
	memory.fail_program_call = 0U;

	compatibility.configuration_fingerprint++;
	ParameterManager_Initialize(&reader, &store, &compatibility,
		sizeof(loaded));
	TEST_CHECK(!ParameterManager_Load(&reader, &loaded));

	invalid_store = store;
	invalid_store.geometry.program_alignment_bytes = 16U;
	ParameterManager_Initialize(&reader, &invalid_store, &compatibility,
		sizeof(loaded));
	TEST_CHECK(!reader.is_initialized);
	invalid_store = store;
	invalid_store.geometry.capacity_bytes--;
	ParameterManager_Initialize(&reader, &invalid_store, &compatibility,
		sizeof(loaded));
	TEST_CHECK(!reader.is_initialized);
	ParameterManager_Initialize(&reader, &store, &compatibility, UINT32_MAX);
	TEST_CHECK(!reader.is_initialized);

	/* A store that reports success but corrupts the committed payload must not
	 * make Save report success or replace the active record. */
	memset(&memory, 0, sizeof(memory));
	memset(memory.bytes, 0xFF, sizeof(memory.bytes));
	store.context = &memory;
	store.geometry.capacity_bytes = sizeof(memory.bytes);
	store.geometry.erase_size_bytes = TEST_SLOT_SIZE;
	store.geometry.program_alignment_bytes = TEST_PROGRAM_ALIGNMENT;
	compatibility.configuration_fingerprint = 0xA5A55A5AUL;
	ParameterManager_Initialize(&writer, &store, &compatibility,
		sizeof(written));
	memory.corrupt_after_program_call = 3U;
	written = 0x12345679UL;
	TEST_CHECK(!ParameterManager_Save(&writer, &written));
	TEST_CHECK(!writer.has_active_record);
	return ParameterManager_CheckGoldenRecord();
}
