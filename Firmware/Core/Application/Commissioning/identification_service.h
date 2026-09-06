#ifndef CORE_APPLICATION_COMMISSIONING_IDENTIFICATION_SERVICE_H
#define CORE_APPLICATION_COMMISSIONING_IDENTIFICATION_SERVICE_H

#include <stdbool.h>
#include <stdint.h>

#include "Core/Application/device_lifecycle.h"

typedef struct
{
	float phase_resistance_design_ohm;
	float phase_resistance_min_ohm;
	float phase_resistance_max_ohm;
	float phase_resistance_balance_fault_pct;
	float phase_resistance_design_tolerance_pct;
} IdentificationServiceConfig;

typedef struct
{
	DeviceLifecycleContext *lifecycle;
	IdentificationServiceConfig config;
	uint32_t elapsed_ticks;
	uint32_t timeout_ticks;
} IdentificationServiceContext;

bool IdentificationService_Initialize(IdentificationServiceContext *context,
	DeviceLifecycleContext *lifecycle,
	const IdentificationServiceConfig *config,
	uint32_t timeout_ticks);
bool IdentificationService_OwnsProcedure(ServiceProcedure procedure);
bool IdentificationService_Supervise1kHz(IdentificationServiceContext *context);
bool IdentificationService_AcceptPhaseResistanceResult(
	const IdentificationServiceContext *context,
	float phase_a_resistance_ohm, float phase_b_resistance_ohm,
	float phase_c_resistance_ohm, float spread_pct, bool is_balanced,
	float *mean_resistance_ohm);

#endif
