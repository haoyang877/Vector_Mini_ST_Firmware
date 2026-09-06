#ifndef CORE_APPLICATION_CONTROL_AUTHORITY_SERVICE_H
#define CORE_APPLICATION_CONTROL_AUTHORITY_SERVICE_H

#include <stdbool.h>

typedef enum
{
	CONTROL_AUTHORITY_NONE = 0U,
	CONTROL_AUTHORITY_CAN,
	CONTROL_AUTHORITY_USB
} ControlAuthority;

typedef struct
{
	volatile ControlAuthority owner;
} ControlAuthorityServiceContext;

bool ControlAuthorityService_Initialize(ControlAuthorityServiceContext *context);
void ControlAuthorityService_Claim(ControlAuthorityServiceContext *context,
	ControlAuthority authority);
void ControlAuthorityService_Release(ControlAuthorityServiceContext *context,
	ControlAuthority authority);
bool ControlAuthorityService_IsOwner(
	const ControlAuthorityServiceContext *context, ControlAuthority authority);

#endif
