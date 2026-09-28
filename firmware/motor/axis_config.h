/* 轴身份、控制基线与运行范围的值契约；不依赖参数服务或硬件。 */
#ifndef AXIS_CONFIG_H
#define AXIS_CONFIG_H
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

#define MOTOR_AXIS_PROFILE_MAGIC 0x31535841U /* AXS1，小端轴记录。 */
#define MOTOR_AXIS_PROFILE_VERSION 1U
#define MOTOR_AXIS_PROFILE_NUMERIC_MAGIC 0x32535841U /* AXS2 */
#define MOTOR_AXIS_PROFILE_NUMERIC_VERSION 2U
#define MOTOR_JOINT_CONFIG_REVISION 1U

/* 协议/存储稳定编号不得重排或复用；持久记录以 uint32_t 存储，不依赖枚举宽度。 */
typedef enum
{
    MOTOR_JOINT_UNKNOWN = 0,
    MOTOR_JOINT_ROLL = 1,
    MOTOR_JOINT_PITCH = 2,
    MOTOR_JOINT_YAW = 3,
    MOTOR_JOINT_WHEEL_RIGHT = 4,
    MOTOR_JOINT_WHEEL_LEFT = 5
} MotorJointType;

/* 参数尾部的独立版本轴记录；小端 IEEE-754 binary32 目标上为 32 字节。
 * 全零/全擦除表示未配置，布局与 CRC 偏移由服务和离线测试校验。 */
typedef struct
{
    uint32_t magic, version;
    union
    {
        char legacy_name[8];
        struct
        {
            uint32_t joint_type, config_revision;
        } numeric;
    } identity;
    float minimum_position_rad, maximum_position_rad, maximum_speed_rad_s;
    uint32_t crc32;
} MotorAxisProfile;

/* 带修订号的位置级联控制基线，独立于硬件板和单台电机标定。 */
typedef struct
{
    float position_kp, position_kd, speed_kp, speed_ki;
    float acceleration_rad_s2, deceleration_rad_s2, maximum_current_a;
} MotorJointControlConfig;

/**
 * @brief 检查已验证轴记录的运行范围，不执行 CRC。
 * @param profile 只读轴记录，NULL 返回 false。
 * @param valid 参数恢复已通过记录校验。
 * @param actual 有限实际位置，单位 rad，必须严格位于上下界内。
 * @param target 有限目标位置，单位 rad，允许等于边界。
 * @return 记录/修订与实际/目标范围允许时返回 true，否则返回 false。
 * @note 快环可调用；仅软件停机边界，不保证机械无过冲，未配置范围沿用既有语义。
 */
static inline bool MotorAxisProfile_AllowsPosition(const MotorAxisProfile *profile,
                                                   bool valid,
                                                   float actual,
                                                   float target)
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

/**
 * @brief 从已验证记录解析稳定轴类型。
 * @param profile 只读记录；支持 AXS1 和 AXS2，NULL 表示未知。
 * @return 稳定轴类型编号，未知为 MOTOR_JOINT_UNKNOWN。
 * @note 本函数不验证 CRC；先 Load 再用于运行决策。
 */
static inline uint32_t MotorAxisProfile_JointType(const MotorAxisProfile *profile)
{
    if (profile == NULL)
    {
        return MOTOR_JOINT_UNKNOWN;
    }
    if (profile->magic == MOTOR_AXIS_PROFILE_NUMERIC_MAGIC &&
        profile->version == MOTOR_AXIS_PROFILE_NUMERIC_VERSION &&
        profile->identity.numeric.joint_type <= MOTOR_JOINT_WHEEL_LEFT)
    {
        return profile->identity.numeric.joint_type;
    }
    if (profile->magic == MOTOR_AXIS_PROFILE_MAGIC &&
        profile->version == MOTOR_AXIS_PROFILE_VERSION)
    {
        if (memcmp(profile->identity.legacy_name, "roll\0\0\0", 8) == 0)
        {
            return MOTOR_JOINT_ROLL;
        }
        if (memcmp(profile->identity.legacy_name, "pitch\0\0", 8) == 0)
        {
            return MOTOR_JOINT_PITCH;
        }
    }
    return MOTOR_JOINT_UNKNOWN;
}
/**
 * @brief 选择机械零位附近的启动单圈分支。
 * @param profile 已加载且机械零位已标定的轴记录。
 * @param valid 该记录通过加载验证。
 * @param single_turn_difference 单圈零位差，65536 counts/转。
 * @return roll/pitch 且包络在 (-π,π) 内时返回就近偏移，其余保持输入。
 * @note 只在启动选分支，不钳位行程；运行连续追踪不可调用。
 */
static inline int32_t MotorAxisProfile_InitialEncoderOffsetQ15(const MotorAxisProfile *profile,
                                                               bool valid,
                                                               int32_t single_turn_difference)
{
    uint32_t joint_type;
    if (profile == NULL || !valid || single_turn_difference < -65535 ||
        single_turn_difference > 65535)
    {
        return single_turn_difference;
    }
    joint_type = MotorAxisProfile_JointType(profile);
    if ((joint_type != MOTOR_JOINT_ROLL && joint_type != MOTOR_JOINT_PITCH) ||
        !(profile->minimum_position_rad > -3.141592654f &&
          profile->maximum_position_rad < 3.141592654f &&
          profile->minimum_position_rad < profile->maximum_position_rad))
    {
        return single_turn_difference;
    }
    if (single_turn_difference > 32768)
    {
        return single_turn_difference - 65536;
    }
    if (single_turn_difference < -32768)
    {
        return single_turn_difference + 65536;
    }
    return single_turn_difference;
}
#endif
