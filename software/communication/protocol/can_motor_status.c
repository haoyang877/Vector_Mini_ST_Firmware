#include "can_motor_status.h"
#include <limits.h>
#include <math.h>
#include <string.h>

/* One atomic aligned word: low 9 bits Hz, bit 9 enabled, rest command epoch.
 * Sole writer after initialization is the serialized receive-command context. */
static volatile uint32_t configuration = 20U;
static uint32_t observed_configuration, last_ms, phase;
static bool clock_valid, due;

void CanMotorStatus_Init(void)
{
    configuration = 20U;
    observed_configuration = 0U;
    last_ms = phase = 0U;
    clock_valid = due = false;
}
bool CanMotorStatus_Configure(float command)
{
    uint32_t old = configuration, rate = old & 511U, enabled;
    if (!isfinite(command)) return false;
    if (command == 0.0f) enabled = 0U;
    else if (command == 1.0f) enabled = 512U;
    else {
        if (command < 10.0f || command > 200.0f) return false;
        rate = (uint32_t)command;
        if ((float)rate != command) return false;
        enabled = 512U;
    }
    configuration = ((old + 1024U) & ~1023U) | enabled | rate;
    return true;
}
uint16_t CanMotorStatus_Rate(void)
{
    uint32_t value = configuration;
    return (value & 512U) ? (uint16_t)(value & 511U) : 0U;
}
static int32_t milli32(float value)
{
    float scaled;
    if (!isfinite(value)) return INT32_MIN;
    scaled = value * 1000.0f;
    /* 2^31 is exactly representable; INT32_MAX as float is not. */
    if (scaled >= 2147483648.0f) return INT32_MAX;
    if (scaled <= -2147483648.0f) return -INT32_MAX;
    return (int32_t)scaled;
}
static int32_t centi32(float value)
{
    float scaled;
    if (!isfinite(value)) return INT32_MIN;
    scaled = value * 100.0f;
    if (scaled >= 2147483648.0f) return INT32_MAX;
    if (scaled <= -2147483648.0f) return -INT32_MAX;
    return (int32_t)scaled;
}
static int16_t milli16(float value)
{
    float scaled;
    if (!isfinite(value)) return INT16_MIN;
    scaled = value * 1000.0f;
    if (scaled >= 32767.0f) return INT16_MAX;
    if (scaled <= -32767.0f) return -INT16_MAX;
    return (int16_t)scaled;
}
static int16_t centi16(float value)
{
    float scaled;
    if (!isfinite(value)) return INT16_MIN;
    scaled = value * 100.0f;
    if (scaled >= 32767.0f) return INT16_MAX;
    if (scaled <= -32767.0f) return -INT16_MAX;
    return (int16_t)scaled;
}
static void be16(uint8_t *p, uint16_t v) { p[0]=(uint8_t)(v>>8); p[1]=(uint8_t)v; }
static void be32(uint8_t *p, uint32_t v)
{
    p[0]=(uint8_t)(v>>24); p[1]=(uint8_t)(v>>16); p[2]=(uint8_t)(v>>8); p[3]=(uint8_t)v;
}
bool CanMotorStatus_Encode(const MotorStatus *s, uint8_t *data, size_t capacity)
{
    if (s == NULL || data == NULL || capacity < CAN_MOTOR_STATUS_SIZE) return false;
    memset(data, 0, CAN_MOTOR_STATUS_SIZE);
    be16(data, s->fault); be16(data+2, s->mode);
    be32(data+4, (uint32_t)milli32(s->position_target));
    be32(data+8, (uint32_t)milli32(s->position_feedback));
    be32(data+12, (uint32_t)centi32(s->speed_target));
    be32(data+16, (uint32_t)centi32(s->speed_feedback));
    be16(data+20, (uint16_t)milli16(s->current_reference));
    be16(data+22, (uint16_t)milli16(s->current_feedback));
    be32(data+24, (uint32_t)milli32(s->position_planned));
    be32(data+28, (uint32_t)centi32(s->speed_planned));
    be16(data+32, (uint16_t)centi16(s->temperature));
    be16(data+34, (uint16_t)centi16(s->bus_voltage));
    return true;
}
bool CanMotorStatus_Prepare(uint32_t now, uint8_t node,
    uint16_t *identifier, uint8_t *data, size_t capacity)
{
    uint32_t config = configuration, elapsed, rate;
    bool next_period = false;
    MotorStatus sample;
    if (node > 7U || identifier == NULL || data == NULL || capacity < CAN_MOTOR_STATUS_SIZE)
        return false;
    if (!clock_valid || config != observed_configuration) {
        observed_configuration = config;
        last_ms = now; phase = 0U; due = false; clock_valid = true;
        (void)MotorStatus_Take(&sample); /* Discard a pre-command completed observation. */
    }
    if ((config & 512U) == 0U) return false;
    elapsed = now - last_ms; last_ms = now; rate = config & 511U;
    if (elapsed >= 1000U) { phase = 0U; next_period = true; }
    else {
        phase += elapsed * rate;
        if (phase >= 1000U) { phase %= 1000U; next_period = true; }
    }
    if (next_period) {
        if (due) (void)MotorStatus_Take(&sample); /* Expired unconsumed sample. */
        due = true;
    }
    if (!due) return false;
    if (!MotorStatus_Take(&sample)) { MotorStatus_Request(); return false; }
    due = false;
    if (configuration != config) return false; /* Command arrived while building. */
    *identifier = (uint16_t)(CAN_MOTOR_STATUS_ID_BASE + node);
    return CanMotorStatus_Encode(&sample, data, capacity);
}
