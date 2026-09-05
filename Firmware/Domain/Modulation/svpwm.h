#ifndef DOMAIN_MODULATION_SVPWM_H
#define DOMAIN_MODULATION_SVPWM_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
	float phase_a_duty;
	float phase_b_duty;
	float phase_c_duty;
	uint8_t sector;
} SvpwmOutput;

bool Svpwm_Calculate(float alpha, float beta, SvpwmOutput *output);

#endif
