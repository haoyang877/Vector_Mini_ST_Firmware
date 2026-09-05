#include "control_authority_service.h"

#define TEST_CHECK(condition) do { if (!(condition)) return __LINE__; } while (0)

int ControlAuthorityService_RunHostTests(void)
{
	ControlAuthorityServiceContext context;

	TEST_CHECK(ControlAuthorityService_Initialize(&context));
	TEST_CHECK(ControlAuthorityService_IsOwner(&context,
		CONTROL_AUTHORITY_NONE));
	ControlAuthorityService_Claim(&context, CONTROL_AUTHORITY_CAN);
	TEST_CHECK(ControlAuthorityService_IsOwner(&context,
		CONTROL_AUTHORITY_CAN));
	ControlAuthorityService_Claim(&context, CONTROL_AUTHORITY_USB);
	TEST_CHECK(ControlAuthorityService_IsOwner(&context,
		CONTROL_AUTHORITY_USB));
	ControlAuthorityService_Release(&context, CONTROL_AUTHORITY_CAN);
	TEST_CHECK(ControlAuthorityService_IsOwner(&context,
		CONTROL_AUTHORITY_USB));
	ControlAuthorityService_Release(&context, CONTROL_AUTHORITY_USB);
	TEST_CHECK(ControlAuthorityService_IsOwner(&context,
		CONTROL_AUTHORITY_NONE));
	return 0;
}
