#ifndef CAN_MOTOR_STATUS_H
#define CAN_MOTOR_STATUS_H
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "../../services/telemetry/motor_status.h"

#define CAN_MOTOR_STATUS_COMMAND 0x64U
#define CAN_MOTOR_STATUS_REPLY 0x65U
#define CAN_MOTOR_STATUS_ID_BASE 0x7F0U
#define CAN_MOTOR_STATUS_SIZE 48U

/** Initialization before reception: disabled, configured rate 20 Hz. */
void CanMotorStatus_Init(void);
/** ISR command owner: 0=stop, 1=start/resume, integer 10..200=set Hz + start.
 * Invalid input leaves configuration unchanged. RAM only. */
bool CanMotorStatus_Configure(float command);
/** Effective Hz, or zero while disabled. Valid from ISR or foreground. */
uint16_t CanMotorStatus_Rate(void);
/** Pure big-endian encoder. Fixed 48-byte payload, capacity checked. */
bool CanMotorStatus_Encode(const MotorStatus *sample, uint8_t *data, size_t capacity);
/** Foreground scheduler. Returns one fresh due frame; missed periods coalesce.
 * Caller submits once without waiting; congestion drops this frame. */
bool CanMotorStatus_Prepare(uint32_t now_ms, uint8_t node,
    uint16_t *identifier, uint8_t *data, size_t capacity);
#endif
