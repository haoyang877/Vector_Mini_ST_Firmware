#include "motor_axis_profile.h"
#include <stddef.h>
#include <string.h>
#include "crc32.h"
#include "application_config.h"

/* 已发布AXS关节修订1的冻结解释；仅记录解码器持有，不随当前默认配置变化。 */
#define AXIS_JOINT_V1_POSITION_KP 8.0f
#define AXIS_JOINT_V1_POSITION_KD 2.0f
#define AXIS_JOINT_V1_SPEED_KP 0.5f
#define AXIS_JOINT_V1_ROLL_SPEED_KI 1.0f
#define AXIS_JOINT_V1_PITCH_SPEED_KI 2.0f
#define AXIS_JOINT_V1_ACCEL_RAD_S2 0.785398163f
#define AXIS_JOINT_V1_DECEL_RAD_S2 0.785398163f
#define AXIS_JOINT_V1_CURRENT_LIMIT_A 6.0f
#define AXIS_JOINT_V1_SPEED_LIMIT_RAD_S 0.785398163f
#define AXIS_JOINT_V1_ROLL_MIN_RAD (-1.57079633f)
#define AXIS_JOINT_V1_ROLL_MAX_RAD (1.57079633f)
#define AXIS_JOINT_V1_PITCH_MIN_RAD (-0.3f)
#define AXIS_JOINT_V1_PITCH_MAX_RAD (0.9f)

/* 五个产品节点共用两种电气模型。角速度和行程均以电机轴表示。
 * YQ 电流 PI 来自 AT32 台架；HT 按额定 R/L 与 1.2 kHz 带宽派生。 */
static const MotorElectricalPreset s_ht = {
    HT8115_POLE_PAIRS,
    HT8115_PHASE_RESISTANCE_OHM,
    HT8115_INDUCTANCE_H,
    HT8115_FLUX_WB,
    HT8115_INDUCTANCE_H *CURRENT_LOOP_BANDWIDTH_RAD_S,
    HT8115_PHASE_RESISTANCE_OHM *CURRENT_LOOP_BANDWIDTH_RAD_S,
    800000.0f,
    ENCODER_SPEED_PLL_HZ,
    1.0f};
static const MotorElectricalPreset s_yq = {YQ5708_POLE_PAIRS,
                                           YQ5708_PHASE_RESISTANCE_OHM,
                                           YQ5708_INDUCTANCE_H,
                                           YQ5708_FLUX_WB,
                                           AT32_YQ5708_CURRENT_KP,
                                           AT32_YQ5708_CURRENT_KI,
                                           800000.0f,
                                           ENCODER_SPEED_PLL_HZ,
                                           YQ5708_GEAR_RATIO};

/* ROLL/PITCH/YAW 各装 2 N·m 阻尼轴；标定起点按脱开负载的工装设置。 */
#define PRODUCT_RUN_CURRENT_LIMIT_A 7.0f
/* 输出轴最高 120°/s，恒加速 1 s 达到上限；控制与存储使用电机轴单位。 */
#define YQ_OUTPUT_SPEED_RAD_S (CONFIG_TWO_PI / 3.0f)
#define YQ_MOTOR_SPEED_RAD_S (YQ_OUTPUT_SPEED_RAD_S * YQ5708_GEAR_RATIO)
#define YQ_MOTOR_ACCEL_RAD_S2 (YQ_MOTOR_SPEED_RAD_S / 1.0f)

static const SensorlessStartupConfig s_wheel_start =
    CALIB_STARTUP_CONFIG(2.0f, 0.5f, HT8115_CALIB_SPEED_RAD_S *HT8115_POLE_PAIRS, 0.05f, 0.5f);
static const SensorlessStartupConfig s_roll_start =
    CALIB_STARTUP_CONFIG(2.0f, 0.5f, HT8115_CALIB_SPEED_RAD_S *HT8115_POLE_PAIRS, 0.5f, 1.0f);
/* PITCH 长扫圈的电机轴速度目标，保持在观测器最低锁定速度以上。 */
#define PITCH_CALIB_SPEED_RAD_S 24.0f
static const SensorlessStartupConfig s_pitch_start =
    CALIB_STARTUP_CONFIG(1.6f, 0.5f, PITCH_CALIB_SPEED_RAD_S *YQ5708_POLE_PAIRS, 0.012f, 0.5f);
static const SensorlessStartupConfig s_yq_start =
    CALIB_STARTUP_CONFIG(1.6f, 0.5f, YQ5708_CALIB_SPEED_RAD_S *YQ5708_POLE_PAIRS, 0.048f, 0.5f);
static const MotorAxisPreset s_presets[] = {
    {0x03,
     MOTOR_JOINT_WHEEL_LEFT,
     &s_ht,
     &s_wheel_start,
     HT8115_CALIB_SPEED_RAD_S,
     3.0f,
     20.0f,
     {2.0f, 0.0f, 0.05f, 0.5f, 20.0f, 20.0f, PRODUCT_RUN_CURRENT_LIMIT_A},
     0.0f,
     0.0f},
    {0x04,
     MOTOR_JOINT_WHEEL_RIGHT,
     &s_ht,
     &s_wheel_start,
     HT8115_CALIB_SPEED_RAD_S,
     3.0f,
     20.0f,
     {2.0f, 0.0f, 0.05f, 0.5f, 20.0f, 20.0f, PRODUCT_RUN_CURRENT_LIMIT_A},
     0.0f,
     0.0f},
    {0x05,
     MOTOR_JOINT_ROLL,
     &s_ht,
     &s_roll_start,
     HT8115_CALIB_SPEED_RAD_S,
     3.0f,
     3.14159265f,
     {8.0f, 2.0f, 0.5f, 1.0f, 3.14159265f, 3.14159265f, PRODUCT_RUN_CURRENT_LIMIT_A},
     2.0f,
     CONFIG_TWO_PI * (150.0f / 360.0f)},
    {0x06,
     MOTOR_JOINT_PITCH,
     &s_yq,
     &s_pitch_start,
     PITCH_CALIB_SPEED_RAD_S,
     1.6f,
     YQ_MOTOR_SPEED_RAD_S,
     {2.0f,
      0.0f,
      0.048f,
      0.5f,
      YQ_MOTOR_ACCEL_RAD_S2,
      YQ_MOTOR_ACCEL_RAD_S2,
      PRODUCT_RUN_CURRENT_LIMIT_A},
     2.0f,
     CONFIG_TWO_PI / 12.0f},
    {0x07,
     MOTOR_JOINT_YAW,
     &s_yq,
     &s_yq_start,
     YQ5708_CALIB_SPEED_RAD_S,
     1.6f,
     YQ_MOTOR_SPEED_RAD_S,
     {2.0f,
      0.0f,
      0.048f,
      0.5f,
      YQ_MOTOR_ACCEL_RAD_S2,
      YQ_MOTOR_ACCEL_RAD_S2,
      PRODUCT_RUN_CURRENT_LIMIT_A},
     2.0f,
     CONFIG_TWO_PI / 2.0f},
};

static const MotorAxisPreset *product_joint(uint32_t joint)
{
    for (unsigned i = 0; i < sizeof(s_presets) / sizeof(s_presets[0]); ++i)
    {
        if (s_presets[i].joint == joint)
        {
            return &s_presets[i];
        }
    }
    return NULL;
}

const MotorAxisPreset *MotorAxisProfile_ForNode(uint8_t node_id)
{
    for (unsigned i = 0; i < sizeof(s_presets) / sizeof(s_presets[0]); ++i)
    {
        if (s_presets[i].node_id == node_id)
        {
            return &s_presets[i];
        }
    }
    return NULL;
}

typedef char axis_record_size_check[(sizeof(MotorAxisProfile) == 32U) ? 1 : -1];
typedef char axis_crc_offset_check[(offsetof(MotorAxisProfile, crc32) == 28U) ? 1 : -1];
typedef char axis_identity_offset_check[(offsetof(MotorAxisProfile, identity) == 8U) ? 1 : -1];
typedef char
    axis_limits_offset_check[(offsetof(MotorAxisProfile, minimum_position_rad) == 16U) ? 1 : -1];

const char *MotorJointType_Name(uint32_t joint_type)
{
    switch (joint_type)
    {
    case MOTOR_JOINT_ROLL:
        return "roll";
    case MOTOR_JOINT_PITCH:
        return "pitch";
    case MOTOR_JOINT_YAW:
        return "yaw";
    case MOTOR_JOINT_WHEEL_RIGHT:
        return "wheel_right";
    case MOTOR_JOINT_WHEEL_LEFT:
        return "wheel_left";
    default:
        return "unknown";
    }
}

/** 记录 CRC 与齿槽表共用 common/crc32 的单一实现；只覆盖 crc32 字段之前的字节。 */
static uint32_t record_crc(const MotorAxisProfile *p)
{
    return Crc32_Compute(p, (uint32_t)offsetof(MotorAxisProfile, crc32), 0U);
}

static bool load_profile(const MotorAxisProfile *stored, MotorAxisProfile *output, bool restore)
{
    const uint8_t *bytes = (const uint8_t *)stored;
    bool zero = true, erased = true;
    unsigned i;
    if (stored == NULL || output == NULL)
    {
        return false;
    }
    for (i = 0; i < sizeof(*stored); ++i)
    {
        zero = zero && bytes[i] == 0;
        erased = erased && bytes[i] == 0xFF;
    }
    if (zero || erased)
    {
        memset(output, 0, sizeof(*output));
        return true;
    }
    *output = *stored;
    if (stored->crc32 != record_crc(stored))
    {
        return false;
    }
    if (stored->magic == MOTOR_AXIS_PROFILE_NUMERIC_MAGIC &&
        stored->version == MOTOR_AXIS_PROFILE_NUMERIC_VERSION)
    {
        if (stored->identity.numeric.config_revision == MOTOR_PRODUCT_AXIS_REVISION)
        {
            const uint32_t role = stored->identity.numeric.joint_type;
            const MotorAxisPreset *p = product_joint(role);
            if (p == NULL || !isfinite(stored->maximum_speed_rad_s) ||
                stored->maximum_speed_rad_s <= 0.0f ||
                stored->maximum_speed_rad_s > p->speed_ceiling_rad_s)
            {
                return false;
            }
            if (role == MOTOR_JOINT_WHEEL_LEFT || role == MOTOR_JOINT_WHEEL_RIGHT)
            {
                return stored->minimum_position_rad == 0.0f && stored->maximum_position_rad == 0.0f;
            }
            const float limit = p->output_travel_limit_rad * p->electrical->gear_ratio;
            if (!isfinite(stored->minimum_position_rad) || !isfinite(stored->maximum_position_rad))
            {
                return false;
            }
            if (stored->minimum_position_rad < -limit || stored->maximum_position_rad > limit)
            {
                if (!restore)
                {
                    return false;
                }
                /* 原记录CRC已验证；运行时只采用旧设备行程与当前机械范围的交集。 */
                output->minimum_position_rad = fmaxf(stored->minimum_position_rad, -limit);
                output->maximum_position_rad = fminf(stored->maximum_position_rad, limit);
                output->crc32 = record_crc(output);
            }
            return output->minimum_position_rad < output->maximum_position_rad;
        }
        if (stored->identity.numeric.joint_type > MOTOR_JOINT_WHEEL_LEFT ||
            stored->identity.numeric.config_revision > MOTOR_JOINT_CONFIG_REVISION)
        {
            return false;
        }
        if (stored->identity.numeric.config_revision == 0U)
        {
            return stored->minimum_position_rad == 0.0f && stored->maximum_position_rad == 0.0f &&
                   stored->maximum_speed_rad_s == 0.0f;
        }
        if (stored->identity.numeric.joint_type != MOTOR_JOINT_ROLL &&
            stored->identity.numeric.joint_type != MOTOR_JOINT_PITCH)
        {
            return false;
        }
    }
    else if (stored->magic != MOTOR_AXIS_PROFILE_MAGIC ||
             stored->version != MOTOR_AXIS_PROFILE_VERSION ||
             MotorAxisProfile_JointType(stored) == MOTOR_JOINT_UNKNOWN)
    {
        return false;
    }
    return isfinite(stored->minimum_position_rad) && isfinite(stored->maximum_position_rad) &&
           stored->minimum_position_rad < stored->maximum_position_rad &&
           isfinite(stored->maximum_speed_rad_s) && stored->maximum_speed_rad_s > 0;
}

bool MotorAxisProfile_Load(const MotorAxisProfile *stored, MotorAxisProfile *output)
{
    return load_profile(stored, output, false);
}

bool MotorAxisProfile_Restore(const MotorAxisProfile *stored, MotorAxisProfile *output)
{
    return load_profile(stored, output, true);
}

bool MotorAxisProfile_Create(MotorAxisProfile *output,
                             const char *name,
                             float minimum_position_rad,
                             float maximum_position_rad,
                             float maximum_speed_rad_s)
{
    MotorAxisProfile candidate;
    if (output == NULL || name == NULL)
    {
        return false;
    }
    if (strcmp(name, "roll") != 0 && strcmp(name, "pitch") != 0)
    {
        return false;
    }
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

bool MotorAxisProfile_CreateJoint(MotorAxisProfile *output,
                                  uint32_t joint_type,
                                  uint32_t config_revision,
                                  float minimum_position_rad,
                                  float maximum_position_rad,
                                  float maximum_speed_rad_s)
{
    MotorAxisProfile candidate;
    MotorJointControlConfig config;
    if (output == NULL)
    {
        return false;
    }
    memset(&candidate, 0, sizeof(candidate));
    candidate.magic = MOTOR_AXIS_PROFILE_NUMERIC_MAGIC;
    candidate.version = MOTOR_AXIS_PROFILE_NUMERIC_VERSION;
    candidate.identity.numeric.joint_type = joint_type;
    candidate.identity.numeric.config_revision = config_revision;
    candidate.minimum_position_rad = minimum_position_rad;
    candidate.maximum_position_rad = maximum_position_rad;
    candidate.maximum_speed_rad_s = maximum_speed_rad_s;
    candidate.crc32 = record_crc(&candidate);
    if (!MotorAxisProfile_Load(&candidate, &candidate))
    {
        return false;
    }
    if (config_revision != 0U && !MotorAxisProfile_Resolve(&candidate, &config))
    {
        return false;
    }
    *output = candidate;
    return true;
}

bool MotorAxisProfile_Resolve(const MotorAxisProfile *profile, MotorJointControlConfig *output)
{
    MotorAxisProfile checked;
    MotorJointControlConfig config = {AXIS_JOINT_V1_POSITION_KP,
                                      AXIS_JOINT_V1_POSITION_KD,
                                      AXIS_JOINT_V1_SPEED_KP,
                                      AXIS_JOINT_V1_ROLL_SPEED_KI,
                                      AXIS_JOINT_V1_ACCEL_RAD_S2,
                                      AXIS_JOINT_V1_DECEL_RAD_S2,
                                      AXIS_JOINT_V1_CURRENT_LIMIT_A};
    float minimum, maximum;
    if (output != NULL && MotorAxisProfile_Load(profile, &checked) &&
        checked.magic == MOTOR_AXIS_PROFILE_NUMERIC_MAGIC &&
        checked.identity.numeric.config_revision == MOTOR_PRODUCT_AXIS_REVISION)
    {
        const MotorAxisPreset *p = product_joint(checked.identity.numeric.joint_type);
        *output = p->control;
        return true;
    }
    if (output == NULL || !MotorAxisProfile_Load(profile, &checked) ||
        checked.magic != MOTOR_AXIS_PROFILE_NUMERIC_MAGIC ||
        checked.identity.numeric.config_revision != MOTOR_JOINT_CONFIG_REVISION)
    {
        return false;
    }
    switch (checked.identity.numeric.joint_type)
    {
    case MOTOR_JOINT_ROLL:
        minimum = AXIS_JOINT_V1_ROLL_MIN_RAD;
        maximum = AXIS_JOINT_V1_ROLL_MAX_RAD;
        break;
    case MOTOR_JOINT_PITCH:
        minimum = AXIS_JOINT_V1_PITCH_MIN_RAD;
        maximum = AXIS_JOINT_V1_PITCH_MAX_RAD;
        config.speed_ki = AXIS_JOINT_V1_PITCH_SPEED_KI;
        break;
    default:
        return false;
    }
    if (checked.minimum_position_rad < minimum || checked.maximum_position_rad > maximum ||
        checked.maximum_speed_rad_s > AXIS_JOINT_V1_SPEED_LIMIT_RAD_S)
    {
        return false;
    }
    *output = config;
    return true;
}
