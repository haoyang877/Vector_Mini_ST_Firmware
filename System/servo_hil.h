#ifndef SERVO_HIL_H
#define SERVO_HIL_H
#include <stdbool.h>
#include <stdint.h>
#ifndef SERVO_HIL_ENABLE
#define SERVO_HIL_ENABLE 0
#endif

/* Bench envelope in motor radians; targets leave 5 deg to the travel trip. */
#define SERVO_HIL_TARGET_LIMIT_RAD 1.48352986f /* 85 deg */
#define SERVO_HIL_TRAVEL_LIMIT_RAD 1.57079633f /* 90 deg */
#define SERVO_HIL_MAX_CRUISE_RAD_S 0.78539816f /* 8 s/rev = 45 deg/s */

typedef enum {
    SERVO_HIL_NONE = 0, SERVO_HIL_STOP = 1, SERVO_HIL_ARM = 2,
    SERVO_HIL_POSITION = 3, SERVO_HIL_POSITION_KP = 4,
    SERVO_HIL_SPEED_KP = 5, SERVO_HIL_SPEED_KI = 6,
    SERVO_HIL_MAX_SPEED = 7, SERVO_HIL_POSITION_KD = 8
} ServoHilAction;
typedef struct { ServoHilAction action; float value; } ServoHilCommand;

/** Debug-only command service. Inputs are motor units (rad, rad/s, A).
 * No peripheral or controller internals are visible to this module.
 * Host writes opcode/value first, then commits sequence; ack completes it.
 * A heartbeat must change at least every 500 ms while armed.
 * elapsed_ticks counts fast ticks since the previous poll, at frequency_hz.
 */
ServoHilCommand ServoHil_Poll(float position, float speed, float iq,
    uint32_t mode, uint32_t error, uint32_t frequency_hz, uint32_t elapsed_ticks);
/** Adapter reports whether the validated command was accepted by the motor API. */
void ServoHil_Complete(bool accepted);
#endif
