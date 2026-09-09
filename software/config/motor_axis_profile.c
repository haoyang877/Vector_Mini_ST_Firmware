#include "motor_axis_profile.h"
#include <stddef.h>
#include <string.h>

typedef char axis_record_size_check[(sizeof(MotorAxisProfile) == 32U) ? 1 : -1];
typedef char axis_crc_offset_check[(offsetof(MotorAxisProfile, crc32) == 28U) ? 1 : -1];
typedef char axis_identity_offset_check[(offsetof(MotorAxisProfile, identity) == 8U) ? 1 : -1];
typedef char axis_limits_offset_check[(offsetof(MotorAxisProfile, minimum_position_rad) == 16U) ? 1 : -1];

const char *MotorJointType_Name(uint32_t joint_type)
{
    switch (joint_type) {
    case MOTOR_JOINT_ROLL: return "roll";
    case MOTOR_JOINT_PITCH: return "pitch";
    case MOTOR_JOINT_YAW: return "yaw";
    case MOTOR_JOINT_WHEEL_RIGHT: return "wheel_right";
    case MOTOR_JOINT_WHEEL_LEFT: return "wheel_left";
    default: return "unknown";
    }
}

uint32_t MotorAxisProfile_JointType(const MotorAxisProfile *profile)
{
    if (profile == NULL) return MOTOR_JOINT_UNKNOWN;
    if (profile->magic == MOTOR_AXIS_PROFILE_NUMERIC_MAGIC &&
        profile->version == MOTOR_AXIS_PROFILE_NUMERIC_VERSION &&
        profile->identity.numeric.joint_type <= MOTOR_JOINT_WHEEL_LEFT)
        return profile->identity.numeric.joint_type;
    if (profile->magic == MOTOR_AXIS_PROFILE_MAGIC && profile->version == MOTOR_AXIS_PROFILE_VERSION) {
        if (memcmp(profile->identity.legacy_name, "roll\0\0\0", 8) == 0) return MOTOR_JOINT_ROLL;
        if (memcmp(profile->identity.legacy_name, "pitch\0\0", 8) == 0) return MOTOR_JOINT_PITCH;
    }
    return MOTOR_JOINT_UNKNOWN;
}

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
    if (stored->crc32 != record_crc(stored)) return false;
    if (stored->magic == MOTOR_AXIS_PROFILE_NUMERIC_MAGIC &&
        stored->version == MOTOR_AXIS_PROFILE_NUMERIC_VERSION) {
        if (stored->identity.numeric.joint_type > MOTOR_JOINT_WHEEL_LEFT ||
            stored->identity.numeric.config_revision > MOTOR_JOINT_CONFIG_REVISION) return false;
        if (stored->identity.numeric.config_revision == 0U)
            return stored->minimum_position_rad == 0.0f &&
                stored->maximum_position_rad == 0.0f && stored->maximum_speed_rad_s == 0.0f;
        if (stored->identity.numeric.joint_type != MOTOR_JOINT_ROLL &&
            stored->identity.numeric.joint_type != MOTOR_JOINT_PITCH) return false;
    } else if (stored->magic != MOTOR_AXIS_PROFILE_MAGIC || stored->version != MOTOR_AXIS_PROFILE_VERSION ||
               MotorAxisProfile_JointType(stored) == MOTOR_JOINT_UNKNOWN) return false;
    return
        isfinite(stored->minimum_position_rad) && isfinite(stored->maximum_position_rad) &&
        stored->minimum_position_rad < stored->maximum_position_rad &&
        isfinite(stored->maximum_speed_rad_s) && stored->maximum_speed_rad_s > 0;
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
    memcpy(candidate.identity.legacy_name, name, strlen(name));
    candidate.minimum_position_rad = minimum_position_rad;
    candidate.maximum_position_rad = maximum_position_rad;
    candidate.maximum_speed_rad_s = maximum_speed_rad_s;
    candidate.crc32 = record_crc(&candidate);
    return MotorAxisProfile_Load(&candidate, output);
}

bool MotorAxisProfile_CreateJoint(MotorAxisProfile *output, uint32_t joint_type,
    uint32_t config_revision, float minimum_position_rad,
    float maximum_position_rad, float maximum_speed_rad_s)
{
    MotorAxisProfile candidate;
    MotorJointControlConfig config;
    if (output == NULL) return false;
    memset(&candidate, 0, sizeof(candidate));
    candidate.magic = MOTOR_AXIS_PROFILE_NUMERIC_MAGIC;
    candidate.version = MOTOR_AXIS_PROFILE_NUMERIC_VERSION;
    candidate.identity.numeric.joint_type = joint_type;
    candidate.identity.numeric.config_revision = config_revision;
    candidate.minimum_position_rad = minimum_position_rad;
    candidate.maximum_position_rad = maximum_position_rad;
    candidate.maximum_speed_rad_s = maximum_speed_rad_s;
    candidate.crc32 = record_crc(&candidate);
    if (!MotorAxisProfile_Load(&candidate, &candidate)) return false;
    if (config_revision != 0U && !MotorAxisProfile_Resolve(&candidate, &config)) return false;
    *output = candidate;
    return true;
}

bool MotorAxisProfile_Resolve(const MotorAxisProfile *profile, MotorJointControlConfig *output)
{
    MotorAxisProfile checked;
    MotorJointControlConfig config = {8.0f, 2.0f, 0.5f, 1.0f, 0.785398163f, 0.785398163f, 6.0f};
    float minimum, maximum;
    if (output == NULL || !MotorAxisProfile_Load(profile, &checked) ||
        checked.magic != MOTOR_AXIS_PROFILE_NUMERIC_MAGIC ||
        checked.identity.numeric.config_revision != MOTOR_JOINT_CONFIG_REVISION) return false;
    switch (checked.identity.numeric.joint_type) {
    case MOTOR_JOINT_ROLL:
        minimum = -1.57079633f; maximum = 1.57079633f;
        break;
    case MOTOR_JOINT_PITCH:
        minimum = -0.3f; maximum = 0.9f; config.speed_ki = 2.0f;
        break;
    default: return false;
    }
    if (checked.minimum_position_rad < minimum || checked.maximum_position_rad > maximum ||
        checked.maximum_speed_rad_s > 0.785398163f) return false;
    *output = config;
    return true;
}
