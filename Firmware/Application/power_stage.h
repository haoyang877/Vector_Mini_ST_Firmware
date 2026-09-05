#ifndef APPLICATION_POWER_STAGE_H
#define APPLICATION_POWER_STAGE_H

#include <stdbool.h>

typedef struct
{
	void *context;
	bool (*enable_outputs)(void *context);
	void (*disable_outputs)(void *context);
	void (*write_duty_cycles)(void *context, float phase_a, float phase_b, float phase_c);
} PowerStagePort;

typedef struct
{
	PowerStagePort port;
	bool is_initialized;
	bool outputs_enabled;
	bool has_latched_fault;
} PowerStageContext;

void PowerStage_Initialize(PowerStageContext *context, const PowerStagePort *port);
bool PowerStage_RequestEnable(PowerStageContext *context, bool safety_interlock_clear);
void PowerStage_ForceDisable(PowerStageContext *context);
bool PowerStage_ApplyDutyCycles(PowerStageContext *context,
	float phase_a, float phase_b, float phase_c);
bool PowerStage_AreOutputsEnabled(const PowerStageContext *context);
bool PowerStage_HasLatchedFault(const PowerStageContext *context);
void PowerStage_RejectOutputCommand(PowerStageContext *context);
void PowerStage_ClearLatchedFault(PowerStageContext *context);

#endif
