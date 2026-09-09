#ifndef MOTOR_AXIS_PROFILE_H
#define MOTOR_AXIS_PROFILE_H
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#define MOTOR_AXIS_PROFILE_MAGIC 0x31535841U /* AXS1, little-endian Flash record */
#define MOTOR_AXIS_PROFILE_VERSION 1U

/* Optional, independently versioned tail of the existing parameter record.
 * Exactly 32 bytes on supported IEEE-754 binary32, little-endian targets.
 * An all-zero or all-erased record denotes an unconfigured legacy motor. */
typedef struct {
    uint32_t magic, version;
    char name[8];
    float minimum_position_rad, maximum_position_rad, maximum_speed_rad_s;
    uint32_t crc32;
} MotorAxisProfile;

/** Validate and load once at parameter restore; false means a corrupt record.
 * Caller owns both output and validity; corrupt records must block mode 3. */
bool MotorAxisProfile_Load(const MotorAxisProfile *stored, MotorAxisProfile *output);
/** Create a checked roll/pitch record for an explicitly configured mechanism. */
bool MotorAxisProfile_Create(MotorAxisProfile *output, const char *name,
    float minimum_position_rad, float maximum_position_rad, float maximum_speed_rad_s);
/** Fast bounds check after a successful load. No CRC work occurs in the ISR.
 * Actual position must remain strictly within limits; targets may equal them.
 * This is a software stop boundary, not a guarantee against physical overshoot. */
static inline bool MotorAxisProfile_AllowsPosition(const MotorAxisProfile *profile,
    bool valid, float actual, float target)
{
    return valid && isfinite(actual) && isfinite(target) &&
        (profile->magic == 0U ||
         (actual > profile->minimum_position_rad && actual < profile->maximum_position_rad &&
          target >= profile->minimum_position_rad && target <= profile->maximum_position_rad));
}
#endif
