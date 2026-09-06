#include "Core/Services/Safety/fault_manager.h"

#include <stddef.h>
#include <string.h>

static FaultSet FaultManager_CodeToMask(uint8_t fault_code)
{
	if (fault_code == 0U || fault_code > FAULT_MANAGER_MAX_CODE)
		return 0U;

	return (FaultSet)1U << fault_code;
}

static uint8_t FaultManager_SelectPrimary(FaultSet faults)
{
	uint8_t fault_code;

	for (fault_code = 1U; fault_code <= FAULT_MANAGER_MAX_CODE; ++fault_code)
	{
		if ((faults & ((FaultSet)1U << fault_code)) != 0U)
			return fault_code;
	}

	return 0U;
}

void FaultManager_Initialize(FaultManagerContext *context)
{
	if (context == NULL)
		return;

	context->active_faults = 0U;
	context->latched_faults = 0U;
	context->primary_fault = 0U;
	context->event_sequence = 0U;
	memset(context->records, 0, sizeof(context->records));
}

bool FaultManager_Raise(FaultManagerContext *context, uint8_t fault_code,
	const FaultObservation *observation)
{
	FaultSet fault_mask;

	if (context == NULL)
		return false;

	fault_mask = FaultManager_CodeToMask(fault_code);
	if (fault_mask == 0U)
		return false;

	if ((context->active_faults & fault_mask) == 0U)
	{
		FaultRecord *record = &context->records[fault_code];

		context->event_sequence++;
		if (context->event_sequence == 0U)
			context->event_sequence = 1U;
		if (record->occurrence_count != UINT32_MAX)
			record->occurrence_count++;
		if (record->first_event_sequence == 0U)
		{
			record->first_event_sequence = context->event_sequence;
			record->first_time_ms = observation != NULL ? observation->time_ms : 0U;
		}
		record->latest_event_sequence = context->event_sequence;
		if (observation != NULL)
		{
			record->latest_time_ms = observation->time_ms;
			record->latest_observation = *observation;
		}
	}
	context->active_faults |= fault_mask;
	context->latched_faults |= fault_mask;
	context->primary_fault = FaultManager_SelectPrimary(context->active_faults);
	return true;
}

bool FaultManager_ClearActive(FaultManagerContext *context, uint8_t fault_code)
{
	FaultSet fault_mask;

	if (context == NULL)
		return false;

	fault_mask = FaultManager_CodeToMask(fault_code);
	if (fault_mask == 0U)
		return false;

	context->active_faults &= ~fault_mask;
	context->primary_fault = FaultManager_SelectPrimary(context->active_faults);
	return true;
}

void FaultManager_ClearAll(FaultManagerContext *context)
{
	if (context == NULL)
		return;
	context->active_faults = 0U;
	context->latched_faults = 0U;
	context->primary_fault = 0U;
}

bool FaultManager_HasFaults(const FaultManagerContext *context)
{
	return context != NULL && context->active_faults != 0U;
}

FaultSet FaultManager_GetActiveFaults(const FaultManagerContext *context)
{
	return context != NULL ? context->active_faults : 0U;
}

FaultSet FaultManager_GetLatchedFaults(const FaultManagerContext *context)
{
	return context != NULL ? context->latched_faults : 0U;
}

uint8_t FaultManager_GetPrimaryFault(const FaultManagerContext *context)
{
	return context != NULL ? context->primary_fault : 0U;
}

uint32_t FaultManager_GetEventSequence(const FaultManagerContext *context)
{
	return context != NULL ? context->event_sequence : 0U;
}

bool FaultManager_GetRecord(const FaultManagerContext *context,
	uint8_t fault_code, FaultRecord *record)
{
	if (context == NULL || record == NULL || fault_code == 0U ||
		fault_code > FAULT_MANAGER_MAX_CODE)
		return false;
	*record = context->records[fault_code];
	return true;
}
