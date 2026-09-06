#ifndef CORE_SERVICES_SAFETY_FAULT_MANAGER_H
#define CORE_SERVICES_SAFETY_FAULT_MANAGER_H

#include <stdbool.h>
#include <stdint.h>

typedef uint32_t FaultSet;

#define FAULT_MANAGER_MAX_CODE 31U

typedef struct
{
	uint32_t time_ms;
	float bus_voltage_v;
	float phase_a_current_a;
	float phase_b_current_a;
	float phase_c_current_a;
	float temperature_c;
	float mechanical_position_rad;
	float mechanical_speed_rad_s;
} FaultObservation;

typedef struct
{
	uint32_t occurrence_count;
	uint32_t first_event_sequence;
	uint32_t latest_event_sequence;
	uint32_t first_time_ms;
	uint32_t latest_time_ms;
	FaultObservation latest_observation;
} FaultRecord;

typedef struct
{
	FaultSet active_faults;
	FaultSet latched_faults;
	uint8_t primary_fault;
	uint32_t event_sequence;
	FaultRecord records[FAULT_MANAGER_MAX_CODE + 1U];
} FaultManagerContext;

void FaultManager_Initialize(FaultManagerContext *context);
bool FaultManager_Raise(FaultManagerContext *context, uint8_t fault_code,
	const FaultObservation *observation);
bool FaultManager_ClearActive(FaultManagerContext *context, uint8_t fault_code);
void FaultManager_ClearAll(FaultManagerContext *context);
bool FaultManager_HasFaults(const FaultManagerContext *context);
FaultSet FaultManager_GetActiveFaults(const FaultManagerContext *context);
FaultSet FaultManager_GetLatchedFaults(const FaultManagerContext *context);
uint8_t FaultManager_GetPrimaryFault(const FaultManagerContext *context);
uint32_t FaultManager_GetEventSequence(const FaultManagerContext *context);
bool FaultManager_GetRecord(const FaultManagerContext *context,
	uint8_t fault_code, FaultRecord *record);

#endif
