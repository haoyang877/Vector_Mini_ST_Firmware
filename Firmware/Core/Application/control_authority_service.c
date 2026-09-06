#include "Core/Application/control_authority_service.h"

bool ControlAuthorityService_Initialize(ControlAuthorityServiceContext *context)
{
	if (context == 0)
		return false;
	context->owner = CONTROL_AUTHORITY_NONE;
	return true;
}

void ControlAuthorityService_Claim(ControlAuthorityServiceContext *context,
	ControlAuthority authority)
{
	if (context != 0 && authority != CONTROL_AUTHORITY_NONE)
		context->owner = authority;
}

void ControlAuthorityService_Release(ControlAuthorityServiceContext *context,
	ControlAuthority authority)
{
	if (context != 0 && context->owner == authority)
		context->owner = CONTROL_AUTHORITY_NONE;
}

bool ControlAuthorityService_IsOwner(
	const ControlAuthorityServiceContext *context, ControlAuthority authority)
{
	return context != 0 && context->owner == authority;
}
