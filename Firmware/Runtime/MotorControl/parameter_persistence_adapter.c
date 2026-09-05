#include "parameter_persistence_adapter.h"

#include <stddef.h>
#include <string.h>
#include "parameter_snapshot.h"
#include "parameter_manager.h"
#include "product_manifest.h"

static ParameterPersistenceAdapterContext *ActiveContext;
#define ParameterTransferBuffer (ActiveContext->transfer_buffer)
#define ParameterManager (ActiveContext->manager)
#define ParameterStore (ActiveContext->store)
#define ParameterManagerInitialized (ActiveContext->manager_is_initialized)

bool ParameterPersistenceAdapter_Initialize(
	ParameterPersistenceAdapterContext *context,
	const ParameterStorePort *store)
{
	if (context == 0 || store == 0 || store->read == 0 ||
		store->erase == 0 || store->program == 0)
		return false;
	context->store = *store;
	context->manager_is_initialized = false;
	ActiveContext = context;
	return true;
}

static bool ParameterPersistenceAdapter_InitializeManager(void)
{
	ParameterCompatibility compatibility;
	const ProductManifest *manifest;

	if (ActiveContext == 0)
		return false;
	if (ParameterManagerInitialized)
		return true;
	manifest = ProductManifest_Get();
	if (manifest == 0)
		return false;
	compatibility.product_id = manifest->product_id;
	compatibility.hardware_profile_id = manifest->hardware_profile_id;
	compatibility.motor_profile_id = manifest->motor_profile_id;
	compatibility.parameter_schema_version = manifest->parameter_schema_version;
	ParameterManager_Initialize(&ParameterManager, &ParameterStore, &compatibility,
		sizeof(ParameterTransferBuffer));
	ParameterManagerInitialized = true;
	return true;
}

bool ParameterPersistenceAdapter_Save(void)
{
	if (!ParameterPersistenceAdapter_InitializeManager())
		return false;
	ParameterSnapshot_Capture(&ParameterTransferBuffer);
	ParameterTransferBuffer.magic_word = PARAMETER_SNAPSHOT_MAGIC;
	return ParameterManager_Save(&ParameterManager, &ParameterTransferBuffer);
}

void ParameterPersistenceAdapter_Load(void)
{
	if (!ParameterPersistenceAdapter_InitializeManager())
	{
		ParameterSnapshot_LoadDefaults();
		return;
	}
	if (ParameterManager_Load(&ParameterManager, &ParameterTransferBuffer))
	{
		ParameterSnapshot_Apply(&ParameterTransferBuffer);
		return;
	}
	memset(&ParameterTransferBuffer, 0, sizeof(ParameterTransferBuffer));
	if (ParameterManager_LoadCompatible(&ParameterManager,
		&ParameterTransferBuffer, PARAM_SCHEMA_VERSION_PREVIOUS_FRICTION,
		offsetof(ParameterSnapshot, friction_coulomb_pos_a)))
	{
		ParameterSnapshot_Apply(&ParameterTransferBuffer);
		return;
	}
	if (ParameterStore.read_previous_format == 0 ||
		!ParameterStore.read_previous_format(ParameterStore.context,
			&ParameterTransferBuffer, sizeof(ParameterTransferBuffer)))
		ParameterSnapshot_LoadDefaults();
	else
		ParameterSnapshot_Apply(&ParameterTransferBuffer);
}
