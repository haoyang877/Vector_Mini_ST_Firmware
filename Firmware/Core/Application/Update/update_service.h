#ifndef CORE_APPLICATION_UPDATE_UPDATE_SERVICE_H
#define CORE_APPLICATION_UPDATE_UPDATE_SERVICE_H

#include <stdbool.h>
#include <stdint.h>

#include "Core/Application/device_lifecycle.h"
#include "Core/Application/parameter_transaction_service.h"
#include "Core/Application/Contracts/motor_output_safety_port.h"
#include "Core/Application/Contracts/update_control_port.h"

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
	MotorOutputSafetyPort motor_output_safety;
	ParameterTransactionServiceContext *parameter_transactions;
	UpdateControlPort port;
	bool is_initialized;
	bool request_is_prepared;
} UpdateServiceContext;

bool UpdateService_Initialize(UpdateServiceContext *context,
	DeviceLifecycleContext *lifecycle,
	const MotorOutputSafetyPort *motor_output_safety,
	ParameterTransactionServiceContext *parameter_transactions,
	const UpdateControlPort *port);
UpdateServiceResult UpdateService_PrepareInstall(UpdateServiceContext *context,
	uint32_t candidate_address, uint32_t candidate_size_bytes);
UpdateServiceResult UpdateService_CommitReset(UpdateServiceContext *context);
bool UpdateService_Cancel(UpdateServiceContext *context);

#endif
