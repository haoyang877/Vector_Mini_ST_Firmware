#ifndef APPLICATION_FRICTION_IDENTIFICATION_SERVICE_H
#define APPLICATION_FRICTION_IDENTIFICATION_SERVICE_H

#include <stdbool.h>
#include <stdint.h>

#include "friction_identification_port.h"

typedef struct
{
	FrictionIdentificationPort port;
	bool is_initialized;
} FrictionIdentificationServiceContext;

bool FrictionIdentificationService_Initialize(
	FrictionIdentificationServiceContext *context,
	const FrictionIdentificationPort *port);
bool FrictionIdentificationService_ReadStatus(
	const FrictionIdentificationServiceContext *context,
	FrictionIdentificationPortStatus *status);
bool FrictionIdentificationService_ReadSample(
	const FrictionIdentificationServiceContext *context, uint8_t index,
	FrictionIdentificationPortSample *sample);
bool FrictionIdentificationService_ApplyCandidate(
	FrictionIdentificationServiceContext *context);

#endif
