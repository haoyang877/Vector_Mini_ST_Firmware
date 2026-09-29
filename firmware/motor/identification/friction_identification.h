#ifndef FRICTION_IDENTIFICATION_H
#define FRICTION_IDENTIFICATION_H

#include <stdbool.h>
#include <stdint.h>

/* Platform-independent algorithm capacity. */
#define FRICTION_IDENTIFICATION_MAX_SPEED_POINTS 4U
#define FRICTION_IDENTIFICATION_MAX_SAMPLES (2U * FRICTION_IDENTIFICATION_MAX_SPEED_POINTS)

typedef enum
{
    FRICTION_IDENT_IDLE = 0,
    FRICTION_IDENT_TRACKING,
    FRICTION_IDENT_SAMPLING,
    FRICTION_IDENT_STOPPING,
    FRICTION_IDENT_COMPLETE,
    FRICTION_IDENT_FAILED
} FrictionIdentificationState;

typedef enum
{
    FRICTION_IDENT_REASON_NONE = 0,
    FRICTION_IDENT_REASON_CANCELLED,
    FRICTION_IDENT_REASON_INVALID_CONFIG,
    FRICTION_IDENT_REASON_TRACK_TIMEOUT,
    FRICTION_IDENT_REASON_SAMPLE_TIMEOUT,
    FRICTION_IDENT_REASON_STOP_TIMEOUT,
    FRICTION_IDENT_REASON_CURRENT_SATURATION,
    FRICTION_IDENT_REASON_INVALID_SAMPLE,
    FRICTION_IDENT_REASON_FIT_REJECTED,
    FRICTION_IDENT_REASON_SAFETY_FAULT
} FrictionIdentificationReason;

typedef struct
{
    float update_period_s;
    float speed_points_rad_s[FRICTION_IDENTIFICATION_MAX_SPEED_POINTS];
    uint32_t speed_point_count;
    float stable_time_s;
    float track_timeout_s;
    float sample_timeout_s;
    float stop_hold_time_s;
    float stop_timeout_s;
    float speed_tolerance_ratio;
    float minimum_speed_tolerance_rad_s;
    float stop_speed_rad_s;
    float sample_turns;
    float minimum_sample_time_s;
    float saturation_time_s;
    float rmse_floor_a;
    float rmse_ratio_max;
} FrictionIdentificationConfig;

typedef struct
{
    float measured_speed_rad_s;
    float ramped_speed_reference_rad_s;
    float iq_a;
    float mechanical_position_rad;
    bool current_saturated;
} FrictionIdentificationInput;

typedef struct
{
    float target_speed_rad_s;
    float mean_speed_rad_s;
    float mean_iq_a;
    uint32_t sample_count;
} FrictionIdentificationSample;

typedef struct
{
    float coulomb_pos_a;
    float coulomb_neg_a;
    float viscous_pos_a_per_rad_s;
    float viscous_neg_a_per_rad_s;
    float rmse_pos_a;
    float rmse_neg_a;
    bool valid;
} FrictionIdentificationResult;

typedef struct
{
    FrictionIdentificationConfig config;
    FrictionIdentificationState state;
    FrictionIdentificationReason reason;
    FrictionIdentificationResult result;
    FrictionIdentificationSample samples[FRICTION_IDENTIFICATION_MAX_SAMPLES];
    uint32_t point_index;
    uint32_t state_ticks;
    uint32_t stable_ticks;
    uint32_t saturation_ticks;
    uint32_t sample_count;
    float tracking_mean_speed;
    float sample_mean_speed;
    float sample_mean_iq;
    float sample_start_position;
    float target_speed_rad_s;
} FrictionIdentification;

/**
 * @brief 复制并验证辨识配置，复位当前会话。
 * @param identification 调用方拥有的辨识状态；修改与查询须串行，指针非空。
 * @param config 只读配置，速度 rad/s、时间 s、电流 A；不保留指针。
 * @return 配置有效返回 true，否则返回 false 并记录配置失败。
 * @note 纯计算，不访问硬件或分配内存。
 */
bool FrictionIdentification_Init(FrictionIdentification *identification,
                                 const FrictionIdentificationConfig *config);
/**
 * @brief 清除辨识工作状态和结果，保留已复制配置。
 * @param identification 调用方拥有的辨识状态；修改与查询须串行，指针非空。
 */
void FrictionIdentification_Reset(FrictionIdentification *identification);
/**
 * @brief 在配置有效时启动第一个正向速度点。
 * @param identification 调用方拥有的辨识状态；修改与查询须串行，指针非空。
 * @return 配置有效返回 true 并重新开始会话；无效配置返回 false。
 */
bool FrictionIdentification_Start(FrictionIdentification *identification);
/**
 * @brief 将活动辨识置为取消失败，并撤销速度目标。
 * @param identification 调用方拥有的辨识状态；修改与查询须串行，指针非空。
 * @note 仅计算状态；停止电机和关相由适配器负责。
 */
void FrictionIdentification_Abort(FrictionIdentification *identification);
/**
 * @brief 锁存失败原因并撤销目标及结果有效性。
 * @param identification 调用方拥有的辨识状态；修改与查询须串行，指针非空。
 * @param reason 调用方判定的失败原因。
 * @note 不负责实际功率输出关断。
 */
void FrictionIdentification_Fail(FrictionIdentification *identification,
                                 FrictionIdentificationReason reason);
/**
 * @brief 按固定配置周期推进稳定、采样、停转与拟合。
 * @param identification 调用方拥有的辨识状态；修改与查询须串行，指针非空。
 * @param input 本周期输入，速度 rad/s、位置 rad、电流 A；只在本次调用读取。
 * @note 可在有界控制周期调用；异常输入形成失败，硬件停机由外层执行。
 */
void FrictionIdentification_Update(FrictionIdentification *identification,
                                   const FrictionIdentificationInput *input);
/**
 * @brief 查询当前目标速度。
 * @param identification 调用方拥有的辨识状态；修改与查询须串行，指针非空。
 * @return 目标机械速度，单位 rad/s；终态为零。
 * @note 不转移所有权；借用指针不得跨状态修改或重启保存。
 */
float FrictionIdentification_GetTargetSpeed(const FrictionIdentification *identification);
/**
 * @brief 查询辨识进度。
 * @param identification 调用方拥有的辨识状态；修改与查询须串行，指针非空。
 * @return 进度百分比，范围 0..100。
 * @note 不转移所有权；借用指针不得跨状态修改或重启保存。
 */
float FrictionIdentification_GetProgressPercent(const FrictionIdentification *identification);
/**
 * @brief 查询配置中的双向速度点记录总数。
 * @param identification 调用方拥有的辨识状态；修改与查询须串行，指针非空。
 * @return 两倍配置速度点数；不是已完成数量或 ADC 样本数。
 * @note 不转移所有权；借用指针不得跨状态修改或重启保存。
 */
uint32_t FrictionIdentification_GetSampleCount(const FrictionIdentification *identification);
/**
 * @brief 查询辨识状态。
 * @param identification 调用方拥有的辨识状态；修改与查询须串行，指针非空。
 * @return 空闲、跟踪、采样、停转、完成或失败状态。
 * @note 不转移所有权；借用指针不得跨状态修改或重启保存。
 */
FrictionIdentificationState
FrictionIdentification_GetState(const FrictionIdentification *identification);
/**
 * @brief 查询最近失败原因。
 * @param identification 调用方拥有的辨识状态；修改与查询须串行，指针非空。
 * @return 当前原因枚举，无失败为 NONE。
 * @note 不转移所有权；借用指针不得跨状态修改或重启保存。
 */
FrictionIdentificationReason
FrictionIdentification_GetReason(const FrictionIdentification *identification);
/**
 * @brief 查询当前双向速度点索引。
 * @param identification 调用方拥有的辨识状态；修改与查询须串行，指针非空。
 * @return 从零开始的速度点索引。
 * @note 不转移所有权；借用指针不得跨状态修改或重启保存。
 */
uint32_t FrictionIdentification_GetPointIndex(const FrictionIdentification *identification);
/**
 * @brief 借用辨识拟合结果。
 * @param identification 调用方拥有的辨识状态；修改与查询须串行，指针非空。
 * @return 模块状态内的只读结果地址；必须检查 valid。
 * @note 不转移所有权；借用指针不得跨状态修改或重启保存。
 */
const FrictionIdentificationResult *
FrictionIdentification_GetResult(const FrictionIdentification *identification);
/**
 * @brief 借用已采集速度点记录数组。
 * @param identification 调用方拥有的辨识状态；修改与查询须串行，指针非空。
 * @return 模块状态内的只读数组地址；容量由 GetSampleCount 查询。
 * @note 不转移所有权；借用指针不得跨状态修改或重启保存。
 */
const FrictionIdentificationSample *
FrictionIdentification_GetSamples(const FrictionIdentification *identification);

#endif
