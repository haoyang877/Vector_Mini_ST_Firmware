#include "Core/Infrastructure/Parameters/parameter_manager.h"

#include <stddef.h>

#define PARAMETER_RECORD_MAGIC 0x56504D52UL
#define PARAMETER_RECORD_FORMAT_VERSION 1U
#define PARAMETER_RECORD_COMMIT_MARKER 0x434F4D54UL
#define PARAMETER_CRC32_POLYNOMIAL 0xEDB88320UL
#define PARAMETER_VERIFY_CHUNK_SIZE 64U

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
} ParameterRecordHeader;

typedef struct
{
	uint32_t marker;
	uint32_t marker_inverse;
} ParameterCommitBlock;

static bool ParameterManager_ReadStore(ParameterManagerContext *context,
	uint8_t slot, uint32_t offset, void *destination, uint32_t size_bytes)
{
	if (slot >= PARAMETER_MANAGER_SLOT_COUNT ||
		offset > context->slot_size_bytes ||
		size_bytes > context->slot_size_bytes - offset)
	{
		return false;
	}
	return context->store.read(context->store.context,
		(uint32_t)slot * context->slot_size_bytes + offset, destination,
		size_bytes) == BSP_RESULT_OK;
}

static bool ParameterManager_EraseSlot(ParameterManagerContext *context,
	uint8_t slot)
{
	if (slot >= PARAMETER_MANAGER_SLOT_COUNT)
		return false;
	return context->store.erase(context->store.context,
		(uint32_t)slot * context->slot_size_bytes,
		context->slot_size_bytes) == BSP_RESULT_OK;
}

static bool ParameterManager_ProgramStore(ParameterManagerContext *context,
	uint8_t slot, uint32_t offset, const void *source, uint32_t size_bytes)
{
	if (slot >= PARAMETER_MANAGER_SLOT_COUNT ||
		offset > context->slot_size_bytes ||
		size_bytes > context->slot_size_bytes - offset)
	{
		return false;
	}
	return context->store.program(context->store.context,
		(uint32_t)slot * context->slot_size_bytes + offset, source,
		size_bytes) == BSP_RESULT_OK;
}

static uint32_t ParameterManager_UpdateCrc32(uint32_t crc,
	const uint8_t *data, uint32_t size_bytes)
{
	uint32_t byte_index;

	for (byte_index = 0U; byte_index < size_bytes; ++byte_index)
	{
		uint8_t bit_index;

		crc ^= data[byte_index];
		for (bit_index = 0U; bit_index < 8U; ++bit_index)
			crc = (crc >> 1U) ^ ((crc & 1U) != 0U ?
				PARAMETER_CRC32_POLYNOMIAL : 0U);
	}
	return crc;
}

static uint32_t ParameterManager_CalculateCrc32(const void *data,
	uint32_t size_bytes)
{
	return ~ParameterManager_UpdateCrc32(UINT32_MAX,
		(const uint8_t *)data, size_bytes);
}

static bool ParameterManager_IsHeaderCompatible(
	const ParameterManagerContext *context, const ParameterRecordHeader *header,
	uint32_t schema_version, uint32_t payload_size)
{
	return header->magic == PARAMETER_RECORD_MAGIC &&
		header->format_version == PARAMETER_RECORD_FORMAT_VERSION &&
		header->header_size == sizeof(ParameterRecordHeader) &&
		header->payload_size == payload_size &&
		header->product_id == context->compatibility.product_id &&
		header->hardware_profile_id == context->compatibility.hardware_profile_id &&
		header->motor_profile_id == context->compatibility.motor_profile_id &&
		(header->configuration_fingerprint ==
			context->compatibility.configuration_fingerprint ||
		 (context->compatibility.allow_legacy_configuration_fingerprint &&
		  header->configuration_fingerprint == UINT32_MAX)) &&
		header->parameter_schema_version == schema_version &&
		header->commit_marker == PARAMETER_RECORD_COMMIT_MARKER &&
		header->commit_marker_inverse == ~PARAMETER_RECORD_COMMIT_MARKER;
}

static bool ParameterManager_IsHeaderValid(const ParameterManagerContext *context,
	const ParameterRecordHeader *header)
{
	return ParameterManager_IsHeaderCompatible(context, header,
		context->compatibility.parameter_schema_version, context->payload_size);
}

static bool ParameterManager_IsSequenceNewer(uint32_t candidate, uint32_t reference)
{
	return (int32_t)(candidate - reference) > 0;
}

static bool ParameterManager_ReadCompatiblePayload(ParameterManagerContext *context,
	uint8_t slot, const ParameterRecordHeader *header, void *payload,
	uint32_t schema_version, uint32_t payload_size)
{
	if (!ParameterManager_IsHeaderCompatible(context, header, schema_version,
		payload_size))
		return false;
	if (!ParameterManager_ReadStore(context, slot,
		(uint32_t)sizeof(*header), payload, payload_size))
		return false;

	return ParameterManager_CalculateCrc32(payload, payload_size) ==
		header->payload_crc32;
}

static bool ParameterManager_ReadValidPayload(ParameterManagerContext *context,
	uint8_t slot, const ParameterRecordHeader *header, void *payload)
{
	return ParameterManager_ReadCompatiblePayload(context, slot, header, payload,
		context->compatibility.parameter_schema_version, context->payload_size);
}

static bool ParameterManager_VerifyStoredPayload(ParameterManagerContext *context,
	uint8_t slot, uint32_t expected_crc)
{
	uint8_t verification_buffer[PARAMETER_VERIFY_CHUNK_SIZE];
	uint32_t crc = UINT32_MAX;
	uint32_t offset = 0U;

	while (offset < context->payload_size)
	{
		uint32_t chunk_size = context->payload_size - offset;

		if (chunk_size > sizeof(verification_buffer))
			chunk_size = sizeof(verification_buffer);
		if (!ParameterManager_ReadStore(context, slot,
			(uint32_t)sizeof(ParameterRecordHeader) + offset,
			verification_buffer, chunk_size))
			return false;
		crc = ParameterManager_UpdateCrc32(crc, verification_buffer, chunk_size);
		offset += chunk_size;
	}

	return ~crc == expected_crc;
}

void ParameterManager_Initialize(ParameterManagerContext *context,
	const BspNonvolatileStoragePort *store,
	const ParameterCompatibility *compatibility,
	uint32_t payload_size)
{
	if (context == NULL)
		return;

	context->payload_size = 0U;
	context->slot_size_bytes = 0U;
	context->active_sequence = 0U;
	context->active_slot = 0U;
	context->has_active_record = false;
	context->is_initialized = false;
	if (store == NULL || compatibility == NULL || store->read == NULL ||
		store->erase == NULL || store->program == NULL || payload_size == 0U ||
		store->geometry.capacity_bytes < PARAMETER_MANAGER_SLOT_COUNT ||
		store->geometry.erase_size_bytes == 0U ||
		store->geometry.program_alignment_bytes == 0U ||
		store->geometry.capacity_bytes % PARAMETER_MANAGER_SLOT_COUNT != 0U)
		return;
	context->slot_size_bytes = store->geometry.capacity_bytes /
		PARAMETER_MANAGER_SLOT_COUNT;
	if (context->slot_size_bytes % store->geometry.erase_size_bytes != 0U ||
		context->slot_size_bytes % store->geometry.program_alignment_bytes != 0U ||
		(uint32_t)sizeof(ParameterRecordHeader) %
			store->geometry.program_alignment_bytes != 0U ||
		offsetof(ParameterRecordHeader, commit_marker) %
			store->geometry.program_alignment_bytes != 0U ||
		context->slot_size_bytes < (uint32_t)sizeof(ParameterRecordHeader) ||
		payload_size > context->slot_size_bytes -
			(uint32_t)sizeof(ParameterRecordHeader))
	{
		context->slot_size_bytes = 0U;
		return;
	}

	context->store = *store;
	context->compatibility = *compatibility;
	context->payload_size = payload_size;
	context->is_initialized = true;
}

bool ParameterManager_Load(ParameterManagerContext *context, void *payload)
{
	ParameterRecordHeader headers[PARAMETER_MANAGER_SLOT_COUNT];
	bool header_valid[PARAMETER_MANAGER_SLOT_COUNT];
	uint8_t first_slot = 0U;
	uint8_t second_slot = 1U;
	uint8_t slot;

	if (context == NULL || !context->is_initialized || payload == NULL)
		return false;

	for (slot = 0U; slot < PARAMETER_MANAGER_SLOT_COUNT; ++slot)
	{
		header_valid[slot] = ParameterManager_ReadStore(context,
			slot, 0U, &headers[slot], sizeof(headers[slot])) &&
			ParameterManager_IsHeaderValid(context, &headers[slot]);
	}

	if (header_valid[1] && (!header_valid[0] ||
		ParameterManager_IsSequenceNewer(headers[1].sequence, headers[0].sequence)))
	{
		first_slot = 1U;
		second_slot = 0U;
	}

	for (slot = 0U; slot < PARAMETER_MANAGER_SLOT_COUNT; ++slot)
	{
		uint8_t candidate_slot = slot == 0U ? first_slot : second_slot;

		if (header_valid[candidate_slot] &&
			ParameterManager_ReadValidPayload(context, candidate_slot,
				&headers[candidate_slot], payload))
		{
			context->active_slot = candidate_slot;
			context->active_sequence = headers[candidate_slot].sequence;
			context->has_active_record = true;
			return true;
		}
	}

	context->has_active_record = false;
	return false;
}

bool ParameterManager_LoadCompatible(ParameterManagerContext *context, void *payload,
	uint32_t schema_version, uint32_t payload_size)
{
	ParameterRecordHeader headers[PARAMETER_MANAGER_SLOT_COUNT];
	bool header_valid[PARAMETER_MANAGER_SLOT_COUNT];
	uint8_t first_slot = 0U;
	uint8_t second_slot = 1U;
	uint8_t slot;

	if (context == NULL || !context->is_initialized || payload == NULL ||
		payload_size == 0U || payload_size > context->payload_size)
		return false;

	for (slot = 0U; slot < PARAMETER_MANAGER_SLOT_COUNT; ++slot)
	{
		header_valid[slot] = ParameterManager_ReadStore(context,
			slot, 0U, &headers[slot], sizeof(headers[slot])) &&
			ParameterManager_IsHeaderCompatible(context, &headers[slot],
				schema_version, payload_size);
	}
	if (header_valid[1] && (!header_valid[0] ||
		ParameterManager_IsSequenceNewer(headers[1].sequence, headers[0].sequence)))
	{
		first_slot = 1U;
		second_slot = 0U;
	}
	for (slot = 0U; slot < PARAMETER_MANAGER_SLOT_COUNT; ++slot)
	{
		uint8_t candidate_slot = slot == 0U ? first_slot : second_slot;
		if (header_valid[candidate_slot] &&
			ParameterManager_ReadCompatiblePayload(context, candidate_slot,
				&headers[candidate_slot], payload, schema_version, payload_size))
		{
			context->active_slot = candidate_slot;
			context->active_sequence = headers[candidate_slot].sequence;
			context->has_active_record = true;
			return true;
		}
	}
	return false;
}

bool ParameterManager_Save(ParameterManagerContext *context, const void *payload)
{
	ParameterRecordHeader header;
	ParameterCommitBlock commit;
	ParameterRecordHeader stored_header;
	uint8_t target_slot;

	if (context == NULL || !context->is_initialized || payload == NULL)
		return false;

	/* Slot 1 is written first so a previous-format record in slot 0 survives migration. */
	target_slot = context->has_active_record ?
		(uint8_t)(context->active_slot ^ 1U) : 1U;
	header.magic = PARAMETER_RECORD_MAGIC;
	header.format_version = PARAMETER_RECORD_FORMAT_VERSION;
	header.header_size = sizeof(header);
	header.payload_size = context->payload_size;
	header.sequence = context->has_active_record ?
		context->active_sequence + 1U : 1U;
	header.payload_crc32 = ParameterManager_CalculateCrc32(payload,
		context->payload_size);
	header.configuration_fingerprint =
		context->compatibility.configuration_fingerprint;
	header.product_id = context->compatibility.product_id;
	header.hardware_profile_id = context->compatibility.hardware_profile_id;
	header.motor_profile_id = context->compatibility.motor_profile_id;
	header.parameter_schema_version =
		context->compatibility.parameter_schema_version;
	header.commit_marker = UINT32_MAX;
	header.commit_marker_inverse = UINT32_MAX;
	commit.marker = PARAMETER_RECORD_COMMIT_MARKER;
	commit.marker_inverse = ~PARAMETER_RECORD_COMMIT_MARKER;

	if (!ParameterManager_EraseSlot(context, target_slot))
		return false;
	if (!ParameterManager_ProgramStore(context, target_slot, 0U,
		&header, offsetof(ParameterRecordHeader, commit_marker)))
		return false;
	if (!ParameterManager_ProgramStore(context, target_slot,
		(uint32_t)sizeof(header), payload, context->payload_size))
		return false;
	if (!ParameterManager_VerifyStoredPayload(context, target_slot,
		header.payload_crc32))
		return false;
	if (!ParameterManager_ProgramStore(context, target_slot,
		offsetof(ParameterRecordHeader, commit_marker), &commit, sizeof(commit)))
		return false;
	if (!ParameterManager_ReadStore(context, target_slot, 0U,
		&stored_header, sizeof(stored_header)) ||
		!ParameterManager_IsHeaderValid(context, &stored_header))
		return false;

	context->active_slot = target_slot;
	context->active_sequence = header.sequence;
	context->has_active_record = true;
	return true;
}
