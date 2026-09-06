#include "parameter_persistence_adapter.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "product_catalog.h"

#define TEST_CHECK(condition_) \
	do { if (!(condition_)) return __LINE__; } while (0)

#define TEST_PARAMETER_RECORD_MAGIC             UINT32_C(0x56504D52)
#define TEST_PARAMETER_RECORD_FORMAT_VERSION    1U
#define TEST_PARAMETER_RECORD_COMMIT_MARKER     UINT32_C(0x434F4D54)
#define TEST_PARAMETER_CRC32_POLYNOMIAL         UINT32_C(0xEDB88320)
#define TEST_PARAMETER_SLOT_SIZE_BYTES          4096U
#define TEST_PARAMETER_STORE_CAPACITY_BYTES     \
	(2U * TEST_PARAMETER_SLOT_SIZE_BYTES)
#define TEST_PARAMETER_PROGRAM_ALIGNMENT_BYTES 8U

typedef struct
{
	uint32_t magic;
	uint16_t format_version;
	uint16_t header_size;
	uint32_t payload_size;
	uint32_t sequence;
	uint32_t payload_crc32;
	uint32_t configuration_fingerprint;
	uint32_t product_id;
	uint32_t hardware_profile_id;
	uint32_t motor_profile_id;
	uint32_t parameter_schema_version;
	uint32_t commit_marker;
	uint32_t commit_marker_inverse;
} PersistenceTestRecordHeader;

typedef char PersistenceTestRecordHeaderMustRemain48Bytes[
	(sizeof(PersistenceTestRecordHeader) == 48U) ? 1 : -1];
typedef char PersistenceTestRecordFingerprintOffsetMustRemain20[
	(offsetof(PersistenceTestRecordHeader, configuration_fingerprint) == 20U) ?
		1 : -1];

typedef struct
{
	uint8_t bytes[TEST_PARAMETER_STORE_CAPACITY_BYTES];
} PersistenceTestStore;

static uint32_t PersistenceTestApplyCount;
static uint32_t PersistenceTestDefaultCount;
static uint32_t PersistenceTestCaptureCount;
static ParameterSnapshot PersistenceTestAppliedSnapshot;
static ParameterSnapshot PersistenceTestCaptureSource;

void ParameterSnapshot_LoadDefaults(ParameterSnapshotContext *context)
{
	(void)context;
	PersistenceTestDefaultCount++;
}

void ParameterSnapshot_Capture(const ParameterSnapshotContext *context,
	ParameterSnapshot *snapshot)
{
	(void)context;
	PersistenceTestCaptureCount++;
	if (snapshot != NULL)
		*snapshot = PersistenceTestCaptureSource;
}

void ParameterSnapshot_Apply(ParameterSnapshotContext *context,
	const ParameterSnapshot *snapshot)
{
	ParameterSnapshot_LoadDefaults(context);
	PersistenceTestApplyCount++;
	if (snapshot != NULL)
		PersistenceTestAppliedSnapshot = *snapshot;
}

static void PersistenceTest_ResetHooks(void)
{
	PersistenceTestApplyCount = 0U;
	PersistenceTestDefaultCount = 0U;
	PersistenceTestCaptureCount = 0U;
	memset(&PersistenceTestAppliedSnapshot, 0,
		sizeof(PersistenceTestAppliedSnapshot));
	memset(&PersistenceTestCaptureSource, 0,
		sizeof(PersistenceTestCaptureSource));
}

static void PersistenceTest_ResetStore(PersistenceTestStore *store)
{
	memset(store->bytes, 0xFF, sizeof(store->bytes));
}

static BspResult PersistenceTest_Read(void *context, uint32_t offset,
	void *destination, size_t length)
{
	PersistenceTestStore *store = (PersistenceTestStore *)context;

	if (store == NULL || destination == NULL ||
		offset > sizeof(store->bytes) || length > sizeof(store->bytes) - offset)
		return BSP_RESULT_INVALID_ARGUMENT;
	memcpy(destination, &store->bytes[offset], length);
	return BSP_RESULT_OK;
}

static BspResult PersistenceTest_Erase(void *context, uint32_t offset,
	size_t length)
{
	PersistenceTestStore *store = (PersistenceTestStore *)context;

	if (store == NULL || offset > sizeof(store->bytes) ||
		length > sizeof(store->bytes) - offset ||
		offset % TEST_PARAMETER_SLOT_SIZE_BYTES != 0U ||
		length % TEST_PARAMETER_SLOT_SIZE_BYTES != 0U)
		return BSP_RESULT_INVALID_ARGUMENT;
	memset(&store->bytes[offset], 0xFF, length);
	return BSP_RESULT_OK;
}

static BspResult PersistenceTest_Program(void *context, uint32_t offset,
	const void *source, size_t length)
{
	PersistenceTestStore *store = (PersistenceTestStore *)context;
	const uint8_t *source_bytes = (const uint8_t *)source;
	size_t index;

	if (store == NULL || source == NULL || offset > sizeof(store->bytes) ||
		length > sizeof(store->bytes) - offset ||
		offset % TEST_PARAMETER_PROGRAM_ALIGNMENT_BYTES != 0U ||
		length % TEST_PARAMETER_PROGRAM_ALIGNMENT_BYTES != 0U)
		return BSP_RESULT_INVALID_ARGUMENT;
	for (index = 0U; index < length; index++)
	{
		if ((store->bytes[offset + index] & source_bytes[index]) !=
			source_bytes[index])
			return BSP_RESULT_IO_ERROR;
	}
	for (index = 0U; index < length; index++)
		store->bytes[offset + index] &= source_bytes[index];
	return BSP_RESULT_OK;
}

static BspNonvolatileStoragePort PersistenceTest_CreateStorePort(
	PersistenceTestStore *store)
{
	BspNonvolatileStoragePort port = {0};

	port.context = store;
	port.geometry.capacity_bytes = sizeof(store->bytes);
	port.geometry.erase_size_bytes = TEST_PARAMETER_SLOT_SIZE_BYTES;
	port.geometry.program_alignment_bytes =
		TEST_PARAMETER_PROGRAM_ALIGNMENT_BYTES;
	port.read = PersistenceTest_Read;
	port.erase = PersistenceTest_Erase;
	port.program = PersistenceTest_Program;
	return port;
}

static uint32_t PersistenceTest_CalculateCrc32(const void *data,
	uint32_t size_bytes)
{
	const uint8_t *bytes = (const uint8_t *)data;
	uint32_t crc = UINT32_MAX;
	uint32_t byte_index;

	for (byte_index = 0U; byte_index < size_bytes; byte_index++)
	{
		uint8_t bit_index;

		crc ^= bytes[byte_index];
		for (bit_index = 0U; bit_index < 8U; bit_index++)
			crc = (crc >> 1U) ^ ((crc & 1U) != 0U ?
				TEST_PARAMETER_CRC32_POLYNOMIAL : 0U);
	}
	return ~crc;
}

static ParameterPersistenceRuntimeConfig PersistenceTest_RuntimeConfig(
	const ProductCatalogEntry *entry)
{
	ParameterPersistenceRuntimeConfig config = {0};

	config.product_id = entry->manifest.product_id;
	config.hardware_compatibility_id =
		entry->persistence.hardware_compatibility_id;
	config.motor_compatibility_id =
		entry->persistence.motor_compatibility_id;
	config.parameter_schema_version =
		entry->manifest.parameter_schema_version;
	config.configuration_fingerprint =
		entry->config->identity.configuration_fingerprint;
	config.allow_erased_fingerprint_migration =
		entry->persistence.allow_erased_fingerprint_migration;
	return config;
}

static ParameterSnapshot PersistenceTest_CreateSnapshot(float marker,
	uint32_t schema_version)
{
	ParameterSnapshot snapshot;

	memset(&snapshot, 0, sizeof(snapshot));
	snapshot.node_id = marker;
	snapshot.schema_version = schema_version;
	snapshot.magic_word = PARAMETER_SNAPSHOT_MAGIC;
	return snapshot;
}

static bool PersistenceTest_WriteRecord(PersistenceTestStore *store,
	uint8_t slot, const ParameterPersistenceRuntimeConfig *runtime_config,
	uint32_t configuration_fingerprint, uint32_t schema_version,
	const ParameterSnapshot *snapshot, uint32_t payload_size)
{
	PersistenceTestRecordHeader header;
	uint32_t base;

	if (store == NULL || slot >= 2U || runtime_config == NULL ||
		snapshot == NULL || payload_size > sizeof(*snapshot) ||
		payload_size > TEST_PARAMETER_SLOT_SIZE_BYTES - sizeof(header))
		return false;
	base = (uint32_t)slot * TEST_PARAMETER_SLOT_SIZE_BYTES;
	memset(&header, 0, sizeof(header));
	header.magic = TEST_PARAMETER_RECORD_MAGIC;
	header.format_version = TEST_PARAMETER_RECORD_FORMAT_VERSION;
	header.header_size = sizeof(header);
	header.payload_size = payload_size;
	header.sequence = 1U;
	header.payload_crc32 = PersistenceTest_CalculateCrc32(snapshot,
		payload_size);
	header.configuration_fingerprint = configuration_fingerprint;
	header.product_id = runtime_config->product_id;
	header.hardware_profile_id = runtime_config->hardware_compatibility_id;
	header.motor_profile_id = runtime_config->motor_compatibility_id;
	header.parameter_schema_version = schema_version;
	header.commit_marker = TEST_PARAMETER_RECORD_COMMIT_MARKER;
	header.commit_marker_inverse = ~TEST_PARAMETER_RECORD_COMMIT_MARKER;
	memcpy(&store->bytes[base], &header, sizeof(header));
	memcpy(&store->bytes[base + sizeof(header)], snapshot, payload_size);
	return true;
}

static bool PersistenceTest_InitializeAdapter(
	ParameterPersistenceAdapterContext *adapter,
	ParameterSnapshotContext *snapshot_context, PersistenceTestStore *store,
	const ParameterPersistenceRuntimeConfig *runtime_config)
{
	BspNonvolatileStoragePort port = PersistenceTest_CreateStorePort(store);

	memset(adapter, 0, sizeof(*adapter));
	memset(snapshot_context, 0, sizeof(*snapshot_context));
	return ParameterPersistenceAdapter_Initialize(adapter, &port,
		snapshot_context, runtime_config);
}

int ParameterPersistenceAdapter_RunHostTests(void)
{
	const ProductCatalogEntry *damped = ProductCatalog_GetByVariant(
		PRODUCT_CATALOG_VARIANT_DAMPED);
	const ProductCatalogEntry *no_damper = ProductCatalog_GetByVariant(
		PRODUCT_CATALOG_VARIANT_NO_DAMPER);
	const ProductCatalogEntry *current = ProductCatalog_GetCurrent();
	ParameterPersistenceRuntimeConfig damped_runtime;
	ParameterPersistenceRuntimeConfig no_damper_runtime;
	ParameterPersistenceRuntimeConfig current_runtime;
	ParameterPersistenceAdapterContext adapter;
	ParameterSnapshotContext snapshot_context;
	PersistenceTestStore store;
	ParameterSnapshot snapshot;
	PersistenceTestRecordHeader saved_header;
	uint32_t saved_base = TEST_PARAMETER_SLOT_SIZE_BYTES;

	TEST_CHECK(damped != NULL && damped->config != NULL);
	TEST_CHECK(no_damper != NULL && no_damper->config != NULL);
	TEST_CHECK(current != NULL && current->config != NULL);
	damped_runtime = PersistenceTest_RuntimeConfig(damped);
	no_damper_runtime = PersistenceTest_RuntimeConfig(no_damper);
	current_runtime = PersistenceTest_RuntimeConfig(current);

	/* The deployed damped fingerprint restores a complete schema-v10 record. */
	PersistenceTest_ResetStore(&store);
	snapshot = PersistenceTest_CreateSnapshot(10.0f, PARAM_SCHEMA_VERSION);
	TEST_CHECK(PersistenceTest_WriteRecord(&store, 0U, &damped_runtime,
		PRODUCT_CATALOG_FINGERPRINT_DAMPED, PARAM_SCHEMA_VERSION, &snapshot,
		sizeof(snapshot)));
	PersistenceTest_ResetHooks();
	TEST_CHECK(PersistenceTest_InitializeAdapter(&adapter, &snapshot_context,
		&store, &damped_runtime));
	ParameterPersistenceAdapter_Load(&adapter);
	TEST_CHECK(PersistenceTestApplyCount == 1U);
	TEST_CHECK(PersistenceTestAppliedSnapshot.node_id == 10.0f);

	/* Only the deployed damped entry accepts an erased header fingerprint. */
	PersistenceTest_ResetStore(&store);
	snapshot = PersistenceTest_CreateSnapshot(11.0f, PARAM_SCHEMA_VERSION);
	TEST_CHECK(PersistenceTest_WriteRecord(&store, 0U, &damped_runtime,
		UINT32_MAX, PARAM_SCHEMA_VERSION, &snapshot, sizeof(snapshot)));
	PersistenceTest_ResetHooks();
	TEST_CHECK(PersistenceTest_InitializeAdapter(&adapter, &snapshot_context,
		&store, &damped_runtime));
	ParameterPersistenceAdapter_Load(&adapter);
	TEST_CHECK(PersistenceTestApplyCount == 1U);
	TEST_CHECK(PersistenceTestAppliedSnapshot.node_id == 11.0f);

	PersistenceTest_ResetHooks();
	TEST_CHECK(PersistenceTest_InitializeAdapter(&adapter, &snapshot_context,
		&store, &no_damper_runtime));
	ParameterPersistenceAdapter_Load(&adapter);
	TEST_CHECK(PersistenceTestApplyCount == 0U);
	TEST_CHECK(PersistenceTestDefaultCount == 1U);

	/* The no-damper entry also rejects a valid damped fingerprint. */
	PersistenceTest_ResetStore(&store);
	snapshot = PersistenceTest_CreateSnapshot(12.0f, PARAM_SCHEMA_VERSION);
	TEST_CHECK(PersistenceTest_WriteRecord(&store, 0U, &damped_runtime,
		PRODUCT_CATALOG_FINGERPRINT_DAMPED, PARAM_SCHEMA_VERSION, &snapshot,
		sizeof(snapshot)));
	PersistenceTest_ResetHooks();
	TEST_CHECK(PersistenceTest_InitializeAdapter(&adapter, &snapshot_context,
		&store, &no_damper_runtime));
	ParameterPersistenceAdapter_Load(&adapter);
	TEST_CHECK(PersistenceTestApplyCount == 0U);
	TEST_CHECK(PersistenceTestDefaultCount == 1U);

	/* Pre-record-manager raw snapshots remain a damped-only migration path. */
	PersistenceTest_ResetStore(&store);
	snapshot = PersistenceTest_CreateSnapshot(13.0f, PARAM_SCHEMA_VERSION);
	memcpy(store.bytes, &snapshot, sizeof(snapshot));
	PersistenceTest_ResetHooks();
	TEST_CHECK(PersistenceTest_InitializeAdapter(&adapter, &snapshot_context,
		&store, &damped_runtime));
	ParameterPersistenceAdapter_Load(&adapter);
	TEST_CHECK(PersistenceTestApplyCount == 1U);
	TEST_CHECK(PersistenceTestAppliedSnapshot.node_id == 13.0f);

	PersistenceTest_ResetHooks();
	TEST_CHECK(PersistenceTest_InitializeAdapter(&adapter, &snapshot_context,
		&store, &no_damper_runtime));
	ParameterPersistenceAdapter_Load(&adapter);
	TEST_CHECK(PersistenceTestApplyCount == 0U);
	TEST_CHECK(PersistenceTestDefaultCount == 1U);

	/* Schema 9 restores its prefix and zero-fills the schema-10 cogging tail. */
	PersistenceTest_ResetStore(&store);
	snapshot = PersistenceTest_CreateSnapshot(9.0f,
		PARAM_SCHEMA_VERSION_PREVIOUS_COGGING);
	snapshot.friction_coulomb_pos_a = 0.75f;
	snapshot.cogging_compensation_map_ma[0] = 123;
	TEST_CHECK(PersistenceTest_WriteRecord(&store, 0U, &damped_runtime,
		PRODUCT_CATALOG_FINGERPRINT_DAMPED,
		PARAM_SCHEMA_VERSION_PREVIOUS_COGGING, &snapshot,
		offsetof(ParameterSnapshot, cogging_compensation_map_ma)));
	PersistenceTest_ResetHooks();
	TEST_CHECK(PersistenceTest_InitializeAdapter(&adapter, &snapshot_context,
		&store, &damped_runtime));
	ParameterPersistenceAdapter_Load(&adapter);
	TEST_CHECK(PersistenceTestApplyCount == 1U);
	TEST_CHECK(PersistenceTestAppliedSnapshot.node_id == 9.0f);
	TEST_CHECK(PersistenceTestAppliedSnapshot.friction_coulomb_pos_a == 0.75f);
	TEST_CHECK(PersistenceTestAppliedSnapshot.cogging_compensation_map_ma[0] == 0);

	/* Schema 8 restores its prefix and zero-fills friction and cogging fields. */
	PersistenceTest_ResetStore(&store);
	snapshot = PersistenceTest_CreateSnapshot(8.0f,
		PARAM_SCHEMA_VERSION_PREVIOUS_FRICTION);
	snapshot.cascade_position_kd = 0.33f;
	snapshot.friction_coulomb_pos_a = 0.75f;
	TEST_CHECK(PersistenceTest_WriteRecord(&store, 0U, &damped_runtime,
		PRODUCT_CATALOG_FINGERPRINT_DAMPED,
		PARAM_SCHEMA_VERSION_PREVIOUS_FRICTION, &snapshot,
		offsetof(ParameterSnapshot, friction_coulomb_pos_a)));
	PersistenceTest_ResetHooks();
	TEST_CHECK(PersistenceTest_InitializeAdapter(&adapter, &snapshot_context,
		&store, &damped_runtime));
	ParameterPersistenceAdapter_Load(&adapter);
	TEST_CHECK(PersistenceTestApplyCount == 1U);
	TEST_CHECK(PersistenceTestAppliedSnapshot.node_id == 8.0f);
	TEST_CHECK(PersistenceTestAppliedSnapshot.cascade_position_kd == 0.33f);
	TEST_CHECK(PersistenceTestAppliedSnapshot.friction_coulomb_pos_a == 0.0f);
	TEST_CHECK(PersistenceTestAppliedSnapshot.cogging_compensation_map_ma[0] == 0);

	/* Saving writes the selected active entry's fingerprint into slot 1. */
	PersistenceTest_ResetStore(&store);
	PersistenceTest_ResetHooks();
	PersistenceTestCaptureSource = PersistenceTest_CreateSnapshot(20.0f,
		PARAM_SCHEMA_VERSION);
	TEST_CHECK(PersistenceTest_InitializeAdapter(&adapter, &snapshot_context,
		&store, &current_runtime));
	TEST_CHECK(ParameterPersistenceAdapter_Save(&adapter));
	TEST_CHECK(PersistenceTestCaptureCount == 1U);
	memcpy(&saved_header, &store.bytes[saved_base], sizeof(saved_header));
	TEST_CHECK(saved_header.magic == TEST_PARAMETER_RECORD_MAGIC);
	TEST_CHECK(saved_header.payload_size == sizeof(ParameterSnapshot));
	TEST_CHECK(saved_header.configuration_fingerprint ==
		current->config->identity.configuration_fingerprint);
	TEST_CHECK(saved_header.product_id == current->manifest.product_id);
	TEST_CHECK(saved_header.hardware_profile_id ==
		current->persistence.hardware_compatibility_id);
	TEST_CHECK(saved_header.motor_profile_id ==
		current->persistence.motor_compatibility_id);
	TEST_CHECK(saved_header.parameter_schema_version == PARAM_SCHEMA_VERSION);
	TEST_CHECK(saved_header.commit_marker ==
		TEST_PARAMETER_RECORD_COMMIT_MARKER);
	TEST_CHECK(saved_header.commit_marker_inverse ==
		~TEST_PARAMETER_RECORD_COMMIT_MARKER);

	return 0;
}
