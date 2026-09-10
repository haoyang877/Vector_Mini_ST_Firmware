#ifndef MOTOR_AXIS_PROFILE_H
#define MOTOR_AXIS_PROFILE_H
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <stddef.h>

#define MOTOR_AXIS_PROFILE_MAGIC 0x31535841U /* AXS1, little-endian Flash record */
#define MOTOR_AXIS_PROFILE_VERSION 1U
#define MOTOR_AXIS_PROFILE_NUMERIC_MAGIC 0x32535841U /* AXS2 */
#define MOTOR_AXIS_PROFILE_NUMERIC_VERSION 2U
#define MOTOR_JOINT_CONFIG_REVISION 1U

/* Stable protocol/storage IDs. Never renumber or reuse a published ID.
 * Store as uint32_t, not as a compiler-sized enum. */
typedef enum {
    MOTOR_JOINT_UNKNOWN = 0,
    MOTOR_JOINT_ROLL = 1,
    MOTOR_JOINT_PITCH = 2,
    MOTOR_JOINT_YAW = 3,
    MOTOR_JOINT_WHEEL_RIGHT = 4,
    MOTOR_JOINT_WHEEL_LEFT = 5
} MotorJointType;

/* Optional, independently versioned tail of the existing parameter record.
 * Exactly 32 bytes on supported IEEE-754 binary32, little-endian targets.
 * An all-zero or all-erased record denotes an unconfigured legacy motor. */
typedef struct {
    uint32_t magic, version;
    union {
        char legacy_name[8];
        struct { uint32_t joint_type, config_revision; } numeric;
    } identity;
    float minimum_position_rad, maximum_position_rad, maximum_speed_rad_s;
    uint32_t crc32;
} MotorAxisProfile;

/* Versioned mode-3 baseline; independent of board and individual calibration. */
typedef struct {
    float position_kp, position_kd, speed_kp, speed_ki;
    float acceleration_rad_s2, deceleration_rad_s2, maximum_current_a;
} MotorJointControlConfig;

/** Display mapping only; unknown IDs return "unknown". No hardware access. */
const char *MotorJointType_Name(uint32_t joint_type);
/** Decode identity from an already validated record, including AXS1. */
uint32_t MotorAxisProfile_JointType(const MotorAxisProfile *profile);
/** Create AXS2. Revision zero stores identity only and requires zero bounds;
 * it cannot enable a motor. Revision one currently supports roll/pitch only. */
bool MotorAxisProfile_CreateJoint(MotorAxisProfile *output, uint32_t joint_type,
    uint32_t config_revision, float minimum_position_rad,
    float maximum_position_rad, float maximum_speed_rad_s);
/** Validate and select an AXS2 baseline at startup, never in the fast loop.
 * Stored limits may narrow the baseline envelope, never widen it.
 * False leaves output untouched; AXS1 retains its separately saved gains. */
bool MotorAxisProfile_Resolve(const MotorAxisProfile *profile, MotorJointControlConfig *output);

/** Validate and load once at parameter restore; false means a corrupt record.
 * Caller owns both output and validity; corrupt records must block mode 3. */
bool MotorAxisProfile_Load(const MotorAxisProfile *stored, MotorAxisProfile *output);
/** Create a checked roll/pitch record for an explicitly configured mechanism. */
bool MotorAxisProfile_Create(MotorAxisProfile *output, const char *name,
    float minimum_position_rad, float maximum_position_rad, float maximum_speed_rad_s);
/** Select the startup single-turn branch relative to a calibrated zero.
 * Only validated roll/pitch envelopes wholly inside (-pi, pi) qualify.
 * Input/output units are 65536 counts/revolution. No clamping to travel limits:
 * out-of-range positions must still fail AllowsPosition. Unconfigured or
 * multi-turn envelopes retain the input; continuous tracking must not call this.
 * Caller must establish mechanical-zero calibration before using this helper. */
int32_t MotorAxisProfile_InitialEncoderOffsetQ15(const MotorAxisProfile *profile,
    bool valid, int32_t single_turn_difference);
/** Fast bounds check after a successful load. No CRC work occurs in the ISR.
 * Actual position must remain strictly within limits; targets may equal them.
 * This is a software stop boundary, not a guarantee against physical overshoot. */
static inline bool MotorAxisProfile_AllowsPosition(const MotorAxisProfile *profile,
    bool valid, float actual, float target)
{
    return profile != NULL && valid && isfinite(actual) && isfinite(target) &&
        (profile->magic != MOTOR_AXIS_PROFILE_NUMERIC_MAGIC ||
         (profile->identity.numeric.config_revision == MOTOR_JOINT_CONFIG_REVISION &&
          (profile->identity.numeric.joint_type == MOTOR_JOINT_ROLL ||
           profile->identity.numeric.joint_type == MOTOR_JOINT_PITCH))) &&
        (profile->magic == 0U ||
         (actual > profile->minimum_position_rad && actual < profile->maximum_position_rad &&
          target >= profile->minimum_position_rad && target <= profile->maximum_position_rad));
}
#endif
