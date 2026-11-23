#include "status_lights.h"
#include "config.h"

/* 100 ms 为所有闪烁的基本脉宽，按实际监督频率换算成拍。 */
#define PULSE_TICKS (SUPERVISOR_FREQ / 10U)
#if SUPERVISOR_FREQ % 10U != 0U || SUPERVISOR_FREQ < 10U
#error "Status light pulse must span a whole number of supervisor ticks"
#endif

bool StatusLights_Step(StatusLights *lights, MotorState state, bool fault, bool checking)
{
    if (fault)
    {
        state = MOTOR_STATE_FAULT;
    }
    checking = checking && state == MOTOR_STATE_IDLE;
    if (state != lights->state || checking != lights->checking)
    {
        lights->phase_ticks = 0U;
        lights->state = state;
        lights->checking = checking;
    }

    unsigned phase = lights->phase_ticks;
    unsigned period = 1U;
    bool red = state == MOTOR_STATE_FAULT;
    bool blue = false;
    switch (state)
    {
    case MOTOR_STATE_IDLE:
        period = checking ? 2U * PULSE_TICKS : 2U * SUPERVISOR_FREQ;
        blue = phase < PULSE_TICKS;
        break;
    case MOTOR_STATE_CALIBRATION:
        period = SUPERVISOR_FREQ;
        blue = phase < PULSE_TICKS || (phase >= 2U * PULSE_TICKS && phase < 3U * PULSE_TICKS);
        break;
    case MOTOR_STATE_RUNNING:
        blue = true;
        break;
    default:
        break;
    }
    lights->phase_ticks = phase + 1U < period ? phase + 1U : 0U;
    bool changed = red != lights->red || blue != lights->blue;
    lights->red = red;
    lights->blue = blue;
    return changed;
}
