#ifndef APPLICATION_IDENTIFICATION_SERVICE_H
#define APPLICATION_IDENTIFICATION_SERVICE_H

#include <stdbool.h>
#include <stdint.h>

#include "Core/Application/device_lifecycle.h"
#include "motor_profiles.h"

typedef struct
{
	DeviceLifecycleContext *lifecycle;
	const MotorProfile *motor_profile;
	uint32_t elapsed_ticks;
	uint32_t timeout_ticks;
} IdentificationServiceContext;

bool IdentificationService_Initialize(IdentificationServiceContext *context,
	DeviceLifecycleContext *lifecycle, const MotorProfile *motor_profile,
	uint32_t timeout_ticks);
bool IdentificationService_OwnsProcedure(ServiceProcedure procedure);
bool IdentificationService_Supervise1kHz(IdentificationServiceContext *context);
bool IdentificationService_AcceptPhaseResistanceResult(
	const IdentificationServiceContext *context,
	float phase_a_resistance_ohm, float phase_b_resistance_ohm,
	float phase_c_resistance_ohm, float spread_pct, bool is_balanced,
	float *mean_resistance_ohm);

#endif
