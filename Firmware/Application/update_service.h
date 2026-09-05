#ifndef APPLICATION_UPDATE_SERVICE_H
#define APPLICATION_UPDATE_SERVICE_H

#include <stdbool.h>
#include <stdint.h>

#include "device_lifecycle.h"
#include "power_stage.h"
#include "parameter_transaction_service.h"
#include "update_control_port.h"

typedef enum
{
	UPDATE_SERVICE_ACCEPTED = 0,
	UPDATE_SERVICE_INVALID_STATE,
	UPDATE_SERVICE_INVALID_CANDIDATE,
	UPDATE_SERVICE_NOT_AVAILABLE,
	UPDATE_SERVICE_INTERNAL_ERROR
} UpdateServiceResult;

typedef struct
{
	DeviceLifecycleContext *lifecycle;
	PowerStageContext *power_stage;
	ParameterTransactionServiceContext *parameter_transactions;
	UpdateControlPort port;
	bool is_initialized;
	bool request_is_prepared;
} UpdateServiceContext;

bool UpdateService_Initialize(UpdateServiceContext *context,
	DeviceLifecycleContext *lifecycle, PowerStageContext *power_stage,
	ParameterTransactionServiceContext *parameter_transactions,
	const UpdateControlPort *port);
UpdateServiceResult UpdateService_PrepareInstall(UpdateServiceContext *context,
	uint32_t candidate_address, uint32_t candidate_size_bytes);
UpdateServiceResult UpdateService_CommitReset(UpdateServiceContext *context);
bool UpdateService_Cancel(UpdateServiceContext *context);

#endif
