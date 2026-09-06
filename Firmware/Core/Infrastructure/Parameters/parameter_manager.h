#ifndef CORE_INFRASTRUCTURE_PARAMETERS_PARAMETER_MANAGER_H
#define CORE_INFRASTRUCTURE_PARAMETERS_PARAMETER_MANAGER_H

#include <stdbool.h>
#include <stdint.h>
#include "bsp_system.h"

#define PARAMETER_MANAGER_SLOT_COUNT 2U

typedef struct
{
	uint32_t product_id;
	uint32_t hardware_profile_id;
	uint32_t motor_profile_id;
	uint32_t parameter_schema_version;
	uint32_t configuration_fingerprint;
	bool allow_legacy_configuration_fingerprint;
} ParameterCompatibility;

typedef struct
{
	BspNonvolatileStoragePort store;
	ParameterCompatibility compatibility;
	uint32_t payload_size;
	uint32_t slot_size_bytes;
	uint32_t active_sequence;
	uint8_t active_slot;
	bool has_active_record;
	bool is_initialized;
} ParameterManagerContext;

void ParameterManager_Initialize(ParameterManagerContext *context,
	const BspNonvolatileStoragePort *store,
	const ParameterCompatibility *compatibility,
	uint32_t payload_size);
bool ParameterManager_Load(ParameterManagerContext *context, void *payload);
bool ParameterManager_LoadCompatible(ParameterManagerContext *context, void *payload,
	uint32_t schema_version, uint32_t payload_size);
bool ParameterManager_Save(ParameterManagerContext *context, const void *payload);

#endif
