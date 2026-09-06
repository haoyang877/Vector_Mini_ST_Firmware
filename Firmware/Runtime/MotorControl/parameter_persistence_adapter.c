#include "parameter_persistence_adapter.h"

#include <stddef.h>
#include <string.h>
#include "parameter_snapshot.h"
#include "Core/Infrastructure/Parameters/parameter_manager.h"
#include "product_manifest.h"
#include "product_catalog.h"

/* IDs of the already-deployed tuple whose erased legacy fingerprint may be
 * migrated once. These are storage compatibility keys, not runtime selectors. */
#define DEPLOYED_LEGACY_HARDWARE_PROFILE_ID 1U
#define DEPLOYED_LEGACY_MOTOR_PROFILE_ID 1U
#define DEPLOYED_LEGACY_ENCODER_PROFILE_ID 1U
#define DEPLOYED_LEGACY_LOAD_PROFILE_ID 1U

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
	ParameterSnapshotContext *snapshot)
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
	const ProductConfig *product_config;

	if (context == 0)
		return false;
	if (ParameterManagerInitialized)
		return true;
	manifest = ProductManifest_Get();
	product_config = &ProductCatalog_CurrentConfig;
	if (manifest == 0 || product_config == 0 ||
		manifest->product_id != product_config->identity.product_id ||
		manifest->configuration_fingerprint !=
			product_config->identity.configuration_fingerprint)
		return false;
	compatibility.product_id = manifest->product_id;
	compatibility.hardware_profile_id = manifest->hardware_profile_id;
	compatibility.motor_profile_id = manifest->motor_profile_id;
	compatibility.parameter_schema_version = manifest->parameter_schema_version;
	compatibility.configuration_fingerprint =
		product_config->identity.configuration_fingerprint;
	compatibility.allow_legacy_configuration_fingerprint =
		product_config->identity.configuration_fingerprint ==
			PRODUCT_CATALOG_CONFIGURATION_FINGERPRINT &&
		manifest->hardware_profile_id == DEPLOYED_LEGACY_HARDWARE_PROFILE_ID &&
		manifest->motor_profile_id == DEPLOYED_LEGACY_MOTOR_PROFILE_ID &&
		manifest->encoder_profile_id == DEPLOYED_LEGACY_ENCODER_PROFILE_ID &&
		manifest->mechanical_load_profile_id ==
			DEPLOYED_LEGACY_LOAD_PROFILE_ID;
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
