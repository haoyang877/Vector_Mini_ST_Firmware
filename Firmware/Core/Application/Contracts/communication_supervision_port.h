#ifndef CORE_APPLICATION_CONTRACTS_COMMUNICATION_SUPERVISION_PORT_H
#define CORE_APPLICATION_CONTRACTS_COMMUNICATION_SUPERVISION_PORT_H

#include <stdbool.h>

typedef struct
{
	bool can_motion_mode_active;
} CommunicationSupervisionInput;

/*
 * The supervisor owns scheduling only. Concrete CAN/USB interfaces and their
 * application services stay behind this bounded 1 kHz port.
 */
typedef struct
{
	void *context;
	void (*execute_1khz)(void *context,
		const CommunicationSupervisionInput *input);
} CommunicationSupervisionPort;

#endif
