#include "motor_axis_profile.h"
#include <stddef.h>
#include <string.h>

typedef char axis_record_size_check[(sizeof(MotorAxisProfile) == 32U) ? 1 : -1];
typedef char axis_crc_offset_check[(offsetof(MotorAxisProfile, crc32) == 28U) ? 1 : -1];

static uint32_t record_crc(const MotorAxisProfile *p)
{
    const uint8_t *bytes = (const uint8_t *)p;
    uint32_t crc = 0xFFFFFFFFU;
    unsigned i, bit;
    for (i = 0; i < offsetof(MotorAxisProfile, crc32); ++i) {
        crc ^= bytes[i];
        for (bit = 0; bit < 8; ++bit)
            crc = (crc >> 1) ^ (0xEDB88320U & (0U - (crc & 1U)));
    }
    return ~crc;
}

bool MotorAxisProfile_Load(const MotorAxisProfile *stored, MotorAxisProfile *output)
{
    const uint8_t *bytes = (const uint8_t *)stored;
    bool zero = true, erased = true;
    unsigned i;
    if (stored == NULL || output == NULL) return false;
    for (i = 0; i < sizeof(*stored); ++i) {
        zero = zero && bytes[i] == 0;
        erased = erased && bytes[i] == 0xFF;
    }
    if (zero || erased) { memset(output, 0, sizeof(*output)); return true; }
    *output = *stored;
    return stored->magic == MOTOR_AXIS_PROFILE_MAGIC && stored->version == MOTOR_AXIS_PROFILE_VERSION &&
        (memcmp(stored->name, "roll\0\0\0", 8) == 0 || memcmp(stored->name, "pitch\0\0", 8) == 0) &&
        isfinite(stored->minimum_position_rad) && isfinite(stored->maximum_position_rad) &&
        stored->minimum_position_rad < stored->maximum_position_rad &&
        isfinite(stored->maximum_speed_rad_s) && stored->maximum_speed_rad_s > 0 &&
        stored->crc32 == record_crc(stored);
}

bool MotorAxisProfile_Create(MotorAxisProfile *output, const char *name,
    float minimum_position_rad, float maximum_position_rad, float maximum_speed_rad_s)
{
    MotorAxisProfile candidate;
    if (output == NULL || name == NULL) return false;
    if (strcmp(name, "roll") != 0 && strcmp(name, "pitch") != 0) return false;
    memset(&candidate, 0, sizeof(candidate));
    candidate.magic = MOTOR_AXIS_PROFILE_MAGIC;
    candidate.version = MOTOR_AXIS_PROFILE_VERSION;
    memcpy(candidate.name, name, strlen(name));
    candidate.minimum_position_rad = minimum_position_rad;
    candidate.maximum_position_rad = maximum_position_rad;
    candidate.maximum_speed_rad_s = maximum_speed_rad_s;
    candidate.crc32 = record_crc(&candidate);
    return MotorAxisProfile_Load(&candidate, output);
}
