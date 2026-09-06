#ifndef APPLICATION_PARAMETER_TRANSACTION_SERVICE_H
#define APPLICATION_PARAMETER_TRANSACTION_SERVICE_H

#include <stdbool.h>

#include "Core/Application/Contracts/parameter_transaction_port.h"

typedef struct
{
	ParameterTransactionPort port;
	bool is_initialized;
	bool operation_has_run;
} ParameterTransactionServiceContext;

bool ParameterTransactionService_Initialize(
	ParameterTransactionServiceContext *context,
	const ParameterTransactionPort *port);
void ParameterTransactionService_RunBackground(
	ParameterTransactionServiceContext *context);
bool ParameterTransactionService_IsIdle(
	const ParameterTransactionServiceContext *context);

#endif
