#ifndef APPLICATION_PARAMETER_MANAGER_H
#define APPLICATION_PARAMETER_MANAGER_H

#include <stdbool.h>
#include <stdint.h>
#include "parameter_store_port.h"

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
	ParameterStorePort store;
	ParameterCompatibility compatibility;
	uint32_t payload_size;
	uint32_t active_sequence;
	uint8_t active_slot;
	bool has_active_record;
	bool is_initialized;
} ParameterManagerContext;

void ParameterManager_Initialize(ParameterManagerContext *context,
	const ParameterStorePort *store, const ParameterCompatibility *compatibility,
	uint32_t payload_size);
bool ParameterManager_Load(ParameterManagerContext *context, void *payload);
bool ParameterManager_LoadCompatible(ParameterManagerContext *context, void *payload,
	uint32_t schema_version, uint32_t payload_size);
bool ParameterManager_Save(ParameterManagerContext *context, const void *payload);

#endif
