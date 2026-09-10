#ifndef MOTOR_STATUS_H
#define MOTOR_STATUS_H
#include <stdbool.h>
#include <stdint.h>

/** SI-unit observation captured by the motor owner at the end of a fast tick.
 * Targets are external commands; planned references are controller outputs.
 * Current is q-axis current, not bus current or phase RMS. */
typedef struct {
    uint16_t fault, mode;
    float position_target, position_feedback;
    float speed_target, speed_feedback;
    float current_reference, current_feedback;
    float position_planned, speed_planned;
} MotorStatus;

/** Single foreground consumer requests one sample; never blocks. */
void MotorStatus_Request(void);
/** Called only by the fast-loop producer. */
bool MotorStatus_IsRequested(void);
/** Fast-loop producer publishes only when requested, with no encoding/I/O. */
void MotorStatus_Publish(const MotorStatus *sample);
/** Foreground consumes a completed sample; false means try a later main tick. */
bool MotorStatus_Take(MotorStatus *sample);
#endif
