#include "parameter_persistence_adapter.h"

#include <stddef.h>
#include <string.h>
#include "parameter_snapshot.h"
#include "parameter_manager.h"
#include "product_manifest.h"
#include "product_variant.h"

#define ParameterTransferBuffer (context->transfer_buffer)
#define ParameterManager (context->manager)
#define ParameterStore (context->store)
#define ParameterSnapshotRuntime (context->snapshot)
#define ParameterManagerInitialized (context->manager_is_initialized)

bool ParameterPersistenceAdapter_Initialize(
	ParameterPersistenceAdapterContext *context,
	const ParameterStorePort *store, ParameterSnapshotContext *snapshot)
{
	if (context == 0 || store == 0 || store->read == 0 ||
		store->erase == 0 || store->program == 0 || snapshot == 0)
		return false;
	context->store = *store;
	context->snapshot = snapshot;
	context->manager_is_initialized = false;
	return true;
}

static bool ParameterPersistenceAdapter_InitializeManager(
	ParameterPersistenceAdapterContext *context)
{
	ParameterCompatibility compatibility;
	const ProductManifest *manifest;
	ProductVariant variant;

	if (context == 0)
		return false;
	if (ParameterManagerInitialized)
		return true;
	manifest = ProductManifest_Get();
	if (manifest == 0 || !ProductVariant_GetActive(&variant))
		return false;
	compatibility.product_id = manifest->product_id;
	compatibility.hardware_profile_id = manifest->hardware_profile_id;
	compatibility.motor_profile_id = manifest->motor_profile_id;
	compatibility.parameter_schema_version = manifest->parameter_schema_version;
	compatibility.configuration_fingerprint =
		manifest->configuration_fingerprint;
	compatibility.allow_legacy_configuration_fingerprint =
		variant.allow_legacy_parameter_migration;
	ParameterManager_Initialize(&ParameterManager, &ParameterStore, &compatibility,
		sizeof(ParameterTransferBuffer));
	ParameterManagerInitialized = true;
	return true;
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
		&ParameterTransferBuffer, PARAM_SCHEMA_VERSION_PREVIOUS_FRICTION,
		offsetof(ParameterSnapshot, friction_coulomb_pos_a)))
	{
		ParameterSnapshot_Apply(ParameterSnapshotRuntime, &ParameterTransferBuffer);
		return;
	}
	if (!ParameterManager.compatibility.allow_legacy_configuration_fingerprint ||
		ParameterStore.read_previous_format == 0 ||
		!ParameterStore.read_previous_format(ParameterStore.context,
			&ParameterTransferBuffer, sizeof(ParameterTransferBuffer)))
		ParameterSnapshot_LoadDefaults(ParameterSnapshotRuntime);
	else
		ParameterSnapshot_Apply(ParameterSnapshotRuntime, &ParameterTransferBuffer);
}
