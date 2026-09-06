#include "Core/Application/MotorControl/parameter_persistence_adapter.h"

#include <stddef.h>
#include <string.h>
#include "parameter_snapshot.h"
#include "Core/Infrastructure/Parameters/parameter_manager.h"

#if defined(__CC_ARM)
#pragma O3
#pragma Ospace
#endif

#define ParameterTransferBuffer (context->transfer_buffer)
#define ParameterManager (context->manager)
#define ParameterStore (context->store)
#define ParameterSnapshotRuntime (context->snapshot)
#define ParameterManagerInitialized (context->manager_is_initialized)

bool ParameterPersistenceAdapter_Initialize(
	ParameterPersistenceAdapterContext *context,
	const BspNonvolatileStoragePort *store,
	ParameterSnapshotContext *snapshot,
	const ParameterPersistenceRuntimeConfig *runtime_config)
{
	if (context == 0 || store == 0 || store->read == 0 ||
		store->erase == 0 || store->program == 0 || snapshot == 0 ||
		runtime_config == 0 || runtime_config->product_id == 0U ||
		runtime_config->hardware_compatibility_id == 0U ||
		runtime_config->motor_compatibility_id == 0U ||
		runtime_config->parameter_schema_version != PARAM_SCHEMA_VERSION ||
		runtime_config->configuration_fingerprint == 0U)
		return false;
	context->store = *store;
	context->snapshot = snapshot;
	context->runtime_config = *runtime_config;
	context->manager_is_initialized = false;
	return true;
}

static bool ParameterPersistenceAdapter_InitializeManager(
	ParameterPersistenceAdapterContext *context)
{
	ParameterCompatibility compatibility;
	const ParameterPersistenceRuntimeConfig *runtime_config;

	if (context == 0)
		return false;
	if (ParameterManagerInitialized)
		return true;
	runtime_config = &context->runtime_config;
	compatibility.product_id = runtime_config->product_id;
	compatibility.hardware_profile_id =
		runtime_config->hardware_compatibility_id;
	compatibility.motor_profile_id = runtime_config->motor_compatibility_id;
	compatibility.parameter_schema_version =
		runtime_config->parameter_schema_version;
	compatibility.configuration_fingerprint =
		runtime_config->configuration_fingerprint;
	compatibility.allow_legacy_configuration_fingerprint =
		runtime_config->allow_erased_fingerprint_migration;
	ParameterManager_Initialize(&ParameterManager, &ParameterStore, &compatibility,
		sizeof(ParameterTransferBuffer));
	ParameterManagerInitialized = ParameterManager.is_initialized;
	return ParameterManagerInitialized;
}

bool ParameterPersistenceAdapter_Save(ParameterPersistenceAdapterContext *context)
{
	if (!ParameterPersistenceAdapter_InitializeManager(context))
		return false;
	ParameterSnapshot_Capture(ParameterSnapshotRuntime, &ParameterTransferBuffer);
	ParameterTransferBuffer.magic_word = PARAMETER_SNAPSHOT_MAGIC;
	return ParameterManager_Save(&ParameterManager, &ParameterTransferBuffer);
}

void ParameterPersistenceAdapter_Load(ParameterPersistenceAdapterContext *context)
{
	if (!ParameterPersistenceAdapter_InitializeManager(context))
	{
		ParameterSnapshot_LoadDefaults(ParameterSnapshotRuntime);
		return;
	}
	if (ParameterManager_Load(&ParameterManager, &ParameterTransferBuffer))
	{
		ParameterSnapshot_Apply(ParameterSnapshotRuntime, &ParameterTransferBuffer);
		return;
	}
	memset(&ParameterTransferBuffer, 0, sizeof(ParameterTransferBuffer));
	if (ParameterManager_LoadCompatible(&ParameterManager,
		&ParameterTransferBuffer, PARAM_SCHEMA_VERSION_PREVIOUS_COGGING,
		offsetof(ParameterSnapshot, cogging_compensation_map_ma)))
	{
		ParameterSnapshot_Apply(ParameterSnapshotRuntime, &ParameterTransferBuffer);
		return;
	}
	memset(&ParameterTransferBuffer, 0, sizeof(ParameterTransferBuffer));
	if (ParameterManager_LoadCompatible(&ParameterManager,
		&ParameterTransferBuffer, PARAM_SCHEMA_VERSION_PREVIOUS_FRICTION,
		offsetof(ParameterSnapshot, friction_coulomb_pos_a)))
	{
		ParameterSnapshot_Apply(ParameterSnapshotRuntime, &ParameterTransferBuffer);
		return;
	}
	if (!ParameterManager.compatibility.allow_legacy_configuration_fingerprint ||
		ParameterStore.read(ParameterStore.context, 0U,
			&ParameterTransferBuffer, sizeof(ParameterTransferBuffer)) !=
			BSP_RESULT_OK)
		ParameterSnapshot_LoadDefaults(ParameterSnapshotRuntime);
	else
		ParameterSnapshot_Apply(ParameterSnapshotRuntime, &ParameterTransferBuffer);
}
