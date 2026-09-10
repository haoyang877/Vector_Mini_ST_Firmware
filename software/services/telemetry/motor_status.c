#include "motor_status.h"
#include <stddef.h>

/* Single-core ISR/foreground mailbox. Every shared payload access and state
 * access is volatile and sequenced. Producer cannot overwrite READY data;
 * consumer cannot access REQUESTED data. No spin, interrupt lock or allocation. */
enum { EMPTY, REQUESTED, READY };
static volatile uint32_t state;
static volatile MotorStatus published;

void MotorStatus_Request(void)
{
    if (state == EMPTY) state = REQUESTED;
}
bool MotorStatus_IsRequested(void) { return state == REQUESTED; }
void MotorStatus_Publish(const MotorStatus *sample)
{
    if (sample == NULL || state != REQUESTED) return;
    published.fault = sample->fault;
    published.mode = sample->mode;
    published.position_target = sample->position_target;
    published.position_feedback = sample->position_feedback;
    published.speed_target = sample->speed_target;
    published.speed_feedback = sample->speed_feedback;
    published.current_reference = sample->current_reference;
    published.current_feedback = sample->current_feedback;
    published.position_planned = sample->position_planned;
    published.speed_planned = sample->speed_planned;
    state = READY;
}
bool MotorStatus_Take(MotorStatus *sample)
{
    if (sample == NULL || state != READY) return false;
    sample->fault = published.fault;
    sample->mode = published.mode;
    sample->position_target = published.position_target;
    sample->position_feedback = published.position_feedback;
    sample->speed_target = published.speed_target;
    sample->speed_feedback = published.speed_feedback;
    sample->current_reference = published.current_reference;
    sample->current_feedback = published.current_feedback;
    sample->position_planned = published.position_planned;
    sample->speed_planned = published.speed_planned;
    state = EMPTY;
    return true;
}
