#ifndef PORTS_PARAMETER_TRANSACTION_PORT_H
#define PORTS_PARAMETER_TRANSACTION_PORT_H

#include <stdbool.h>

typedef enum
{
	PARAMETER_TRANSACTION_NONE = 0,
	PARAMETER_TRANSACTION_SAVE,
	PARAMETER_TRANSACTION_RESTORE_DEFAULTS
} ParameterTransactionOperation;

typedef struct
{
	void *context;
	ParameterTransactionOperation (*get_operation)(void *context);
	bool (*begin)(void *context);
	bool (*restore_defaults)(void *context);
	bool (*save)(void *context);
	void (*end)(void *context);
	void (*complete)(void *context);
	void (*fail)(void *context);
} ParameterTransactionPort;

#endif
