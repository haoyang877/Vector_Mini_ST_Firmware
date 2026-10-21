/* 状态快照由调用方明确传入，协议投影不暴露内部可写对象。 */
#include "motor_status_mailbox.h"
#include <stddef.h>

/* Single-core ISR/foreground mailbox. Every shared payload access and s_state
 * access is volatile and sequenced. Producer cannot overwrite READY data;
 * consumer cannot access REQUESTED data. No spin, interrupt lock or allocation. */
enum
{
    EMPTY,
    REQUESTED,
    READY
};
static volatile uint32_t s_state;
static volatile MotorStatus s_published;

void MotorStatus_Request(void)
{
    if (s_state == EMPTY)
    {
        s_state = REQUESTED;
    }
}
bool MotorStatus_IsRequested(void)
{
    return s_state == REQUESTED;
}
void MotorStatus_Publish(const MotorStatus *sample)
{
    if (sample == NULL || s_state != REQUESTED)
    {
        return;
    }
    s_published.fault = sample->fault;
    s_published.mode = sample->mode;
    s_published.position_target = sample->position_target;
    s_published.position_feedback = sample->position_feedback;
    s_published.speed_target = sample->speed_target;
    s_published.speed_feedback_rad_s = sample->speed_feedback_rad_s;
    s_published.current_reference = sample->current_reference;
    s_published.current_feedback = sample->current_feedback;
    s_published.position_planned = sample->position_planned;
    s_published.speed_planned = sample->speed_planned;
    s_published.temperature = sample->temperature;
    s_published.bus_voltage = sample->bus_voltage;
    s_published.bus_current = sample->bus_current;
    s_state = READY;
}
bool MotorStatus_Take(MotorStatus *sample)
{
    if (sample == NULL || s_state != READY)
    {
        return false;
    }
    sample->fault = s_published.fault;
    sample->mode = s_published.mode;
    sample->position_target = s_published.position_target;
    sample->position_feedback = s_published.position_feedback;
    sample->speed_target = s_published.speed_target;
    sample->speed_feedback_rad_s = s_published.speed_feedback_rad_s;
    sample->current_reference = s_published.current_reference;
    sample->current_feedback = s_published.current_feedback;
    sample->position_planned = s_published.position_planned;
    sample->speed_planned = s_published.speed_planned;
    sample->temperature = s_published.temperature;
    sample->bus_voltage = s_published.bus_voltage;
    sample->bus_current = s_published.bus_current;
    s_state = EMPTY;
    return true;
}
