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
/** Resolve the product CAN node address from a CRC-valid AXS1/AXS2 identity.
 * wheel_left=1, wheel_right=2, roll=3, pitch=4, yaw=5. These addresses are
 * independent of stable storage IDs. Unknown/invalid records retain fallback.
 * Startup/parameter restore only; does not change the record or enable motion. */
uint8_t MotorAxisProfile_CanNodeId(const MotorAxisProfile *profile, uint8_t fallback);
/** Decode identity from an already validated record, including AXS1. */
uint32_t MotorAxisProfile_JointType(const MotorAxisProfile *profile);
/**
 * @brief 创建 AXS2；版本 0 仅登记身份，版本 1 支持 roll/pitch/yaw。
 * @param output 调用方持有的输出记录。
 * @param joint_type 固定关节类型编号，独立于 CAN 节点地址。
 * @param config_revision 配置版本；0 要求三个运动边界均为零且不可使能。
 * @param minimum_position_rad 最小机械位置，rad。
 * @param maximum_position_rad 最大机械位置，rad。
 * @param maximum_speed_rad_s 最大轨迹速度，rad/s。
 * @return 记录和配置边界有效时返回 true，否则不修改输出。
 */
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
/**
 * @brief 启动时选择相对机械零位最近的单圈分支，不钳位实际位置。
 * @param profile 已加载的轴记录；roll/pitch 要求范围严格位于 (-pi, pi)，yaw 可达 ±pi。
 * @param valid 轴记录有效标志。
 * @param single_turn_difference 编码器单圈位置减机械零位，65536 count/rev。
 * @return 满足轴范围条件时返回最近分支，否则保留输入。
 * @note 仅在机械零位已标定的启动路径使用，连续跟踪不得调用。
 * 正负半圈端点保持输入符号，不把实际反馈钳位到目标范围。
 */
int32_t MotorAxisProfile_InitialEncoderOffsetQ15(const MotorAxisProfile *profile,
    bool valid, int32_t single_turn_difference);
/**
 * @brief 检查已加载轴配置及位置目标；实际位置越界不构成目标参数错误。
 * @param profile 调用方持有的轴配置。
 * @param valid 加载时缓存的配置有效标志。
 * @param actual_rad 实际位置，rad；仅检查有限性，允许处于目标范围外。
 * @param target_rad 控制目标，rad；配置范围包含上下限端点。
 * @return 配置及输入有效且目标合法时返回 true。
 * @note 可在快环调用，不计算 CRC；目标范围不保证实物不会越界。
 */
static inline bool MotorAxisProfile_AllowsPosition(const MotorAxisProfile *profile,
    bool valid, float actual_rad, float target_rad)
{
    return profile != NULL && valid && isfinite(actual_rad) && isfinite(target_rad) &&
        (profile->magic != MOTOR_AXIS_PROFILE_NUMERIC_MAGIC ||
         (profile->identity.numeric.config_revision == MOTOR_JOINT_CONFIG_REVISION &&
          (profile->identity.numeric.joint_type == MOTOR_JOINT_ROLL ||
           profile->identity.numeric.joint_type == MOTOR_JOINT_PITCH ||
           profile->identity.numeric.joint_type == MOTOR_JOINT_YAW))) &&
        (profile->magic == 0U ||
         (target_rad >= profile->minimum_position_rad &&
          target_rad <= profile->maximum_position_rad));
}
#endif
