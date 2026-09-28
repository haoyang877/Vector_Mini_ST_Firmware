#ifndef MOTOR_LOAD_PROFILE_H
#define MOTOR_LOAD_PROFILE_H

#include <stdbool.h>
#include <stdint.h>

/* 同一电机的装配配置；轴身份和电气标定独立保存。 */
#define MOTOR_LOAD_DAMPING_RING 1U
#define MOTOR_LOAD_FRICTION_FEEDFORWARD 2U
#define MOTOR_LOAD_RECORD_MAGIC 0x31504D44U
#define MOTOR_LOAD_RECORD_VERSION 1U

typedef struct
{
    float align_current_ramp_time_s;
    float align_hold_time_s;
    float align_current_a;
    float startup_iq_initial_a;
    float startup_iq_a;
    float startup_iq_ramp_time_s;
    float startup_id_a;
    float minimum_current_limit_a;
    float minimum_electrical_velocity_rad_s;
    float target_electrical_velocity_rad_s;
    float startup_ramp_time_s;
    float speed_lock_time_s;
    float speed_lock_filter_alpha;
    float observer_lock_ratio;
    float angle_handoff_time_s;
    float lock_timeout_s;
    float id_ramp_down_time_s;
    float observer_loss_time_s;
} SensorlessStartupConfig_TypeDef;

typedef struct
{
    SensorlessStartupConfig_TypeDef startup;
    float speed_mechanical_rad_s;
    float speed_stable_timeout_s;
    float startup_timeout_s;
    float stop_current_ramp_time_s;
    float electrical_zero_current_a;
    uint32_t scan_turns;
} MotorCalibrationProfile;

/* schema10 尾部独立扩展，CRC 覆盖前三个 little-endian uint32。 */
typedef struct
{
    uint32_t magic;
    uint32_t version;
    uint32_t flags;
    uint32_t crc32;
} MotorLoadRecord;

/** @brief 校验装配配置位，仅接受 0、1、3。
 * @param flags 阻尼及前馈位。
 * @return 配置合法时为 true。
 */
bool MotorLoadProfile_IsValid(uint32_t flags);

/** @brief 读取不可变的标定整定。
 * @param flags 已验证的配置位。
 * @return 配置指针；非法位返回空指针，调用方不得启动。
 */
const MotorCalibrationProfile *MotorLoadProfile_Calibration(uint32_t flags);

/** @brief 判断显式启用的阻尼摩擦前馈。
 * @param flags 已保存或生效的装配配置位。
 * @return 仅合法配置 3 返回 true。
 */
bool MotorLoadProfile_FeedforwardEnabled(uint32_t flags);

/** @brief 生成带版本及 CRC 的存储记录，不访问 Flash。
 * @param flags 要保存的合法配置位。
 * @param output 输出记录，调用方拥有。
 * @return 输入非法或输出为空时为 false，输出保持不变。
 */
bool MotorLoadRecord_Create(uint32_t flags, MotorLoadRecord *output);

/** @brief 读取存储记录；全零或全 FF 的旧尾部按配置 0 处理。
 * @param record 输入记录。
 * @param flags 输出配置位。
 * @return 校验成功为 true；失败时输出保持不变。
 */
bool MotorLoadRecord_Load(const MotorLoadRecord *record, uint32_t *flags);

#endif
