#include "vector_mini_st_bsp.h"

BspBoardValidationResult BspVectorMiniSt_ValidateBindingRequest(
	const BspBoardBindingRequest *request)
{
	return BspBoard_ValidateBindingRequest(&BspVectorMiniSt_Capabilities,
		request);
}
