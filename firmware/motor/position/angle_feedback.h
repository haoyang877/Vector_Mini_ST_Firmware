#ifndef ANGLE_FEEDBACK_H
#define ANGLE_FEEDBACK_H

#include <stdbool.h>
#include <stdint.h>
#include "data_type.h"

/* 角度反馈（motor 层拥有）：传感器采样 -> 方向/线性化/电零位/多圈/速度。
 * 传感器型号由板级 hw_conf.h 的 ENCODER_SENSOR_TYPE 选择，驱动负责把各自位宽
 * 归一化为同一 Q15 语义；本层不访问寄存器、总线或板级宏。 */

/* 单圈角度表示：无符号 Q15，范围 [0, 65535]。 */
#define ENCODER_Q15_CPR 65536UL
#define ENCODER_Q15_HALF_TURN 32768
#define ENCODER_OFFSET_LUT_SIZE 1024U
#define ENCODER_OFFSET_LUT_BITS 10U
#define ENCODER_VELOCITY_WINDOW 16U
#define ENCODER_BAD_FRAME_OFFLINE_COUNT 100U

/** 编码器读取状态；当前实现只产生 OK 与 SPI_TIMEOUT，其余为协议层预留。 */
typedef enum
{
    ENCODER_READ_OK = 0,
    ENCODER_READ_SPI_TIMEOUT = 1,
    ENCODER_READ_CRC_MISMATCH = 2,
    ENCODER_READ_MAGNET_TOO_STRONG = 3,
    ENCODER_READ_MAGNET_TOO_WEAK = 4,
    ENCODER_READ_MAGNET_INVALID = 5,
    ENCODER_READ_OVERSPEED = 6,
    ENCODER_READ_TLE_RESET = 7,
    ENCODER_READ_TLE_SYSTEM_ERROR = 8,
    ENCODER_READ_TLE_INTERFACE_ERROR = 9,
    ENCODER_READ_TLE_INVALID_ANGLE = 10
} Encoder_ReadStatus;

/* 写入 Flash 的标定标志；机械零位属于可选反馈状态。 */
#define ENC_CALIB_LINEARIZED (1U << 0)
#define ENC_CALIB_ELECTRICAL_ZERO (1U << 1)
#define ENC_CALIB_MECHANICAL_ZERO (1U << 2)
#define ENC_CALIB_ZERO_POS ENC_CALIB_ELECTRICAL_ZERO
#define ENC_CALIB_ALL (ENC_CALIB_LINEARIZED | ENC_CALIB_ELECTRICAL_ZERO)

/** 编码器完整状态：持久化标定、单圈/多圈角度、速度估计与帧诊断。 */
typedef struct
{
    /* 持久化的方向配置与标定数据。 */
    uint16_t electrical_zero_q15;
    uint16_t mechanical_zero_q15;
    int16_t linearization_lut_q15[ENCODER_OFFSET_LUT_SIZE];
    uint8_t calib_flag;
    uint8_t reverse;

    /* 原始读数、方向校正值与 LUT 校正值。 */
    uint16_t raw_q15;
    uint16_t directed_q15;
    uint16_t linearized_q15;
    uint16_t previous_linearized_q15;
    int64_t shadow_q15;
    int64_t mechanical_zero_shadow_q15;
    int64_t velocity_shadow_q15;
    bool has_valid_sample;

    /* 对外提供的机械/电角度与角速度反馈。 */
    float theta_elec;
    float vel_elec;
    float theta_mech;
    float vel_mech;
    float vel_mech_continuous;

    /* 2 kHz 滑动平均机械速度估计。 */
    uint8_t velocity_divider;
    uint8_t velocity_history_index;
    uint8_t velocity_sample_count;
    bool velocity_ready;
    int32_t velocity_delta_history[ENCODER_VELOCITY_WINDOW];
    int32_t velocity_delta_sum;

    /* 传感器帧诊断与在线状态。 */
    Encoder_ReadStatus read_status;
    Encoder_ReadStatus read_status_latched;
    uint16_t frame_word;
    uint16_t safety_word;
    uint8_t crc_received;
    uint8_t crc_calculated;
    uint32_t crc_error_count;
    uint32_t read_error_count;
    uint16_t bad_frame_streak;
} Encoder_TypeDef;

/**
 * @brief 上电初始化编码器状态与 SPI 外设。
 * @param Encoder 编码器状态指针；持久化字段（方向/零位/LUT）须在调用前由参数服务填充。
 * @note 初始化一次；SPI 初始化失败走致命错误路径，不返回。
 */
void Encoder_ParamInit(Encoder_TypeDef *Encoder);

/**
 * @brief 同步读取一帧角度并更新角度与速度反馈。
 * @param MotorControl 电机控制状态指针，读取极对数与轴配置。
 * @param Encoder 编码器状态指针。
 * @note 20 kHz 快速环的兼容入口；新代码使用 BeginSample/CompleteSample 分离采样。
 */
void Encoder_Update(MotorControl_TypeDef *MotorControl, Encoder_TypeDef *Encoder);

/**
 * @brief 非阻塞地发起下一帧角度采样。
 * @return 已发起返回 true；总线未就绪返回 false，此时仍须由同一调用方完成本帧。
 * @note 仅 20 kHz 快速环调用；两次调用之间禁止访问传感器总线。
 */
bool Encoder_BeginSample(void);

/**
 * @brief 完成本周期的角度采样并更新同一套估计器。
 * @param MotorControl 电机控制状态指针，读取极对数与轴配置。
 * @param Encoder 编码器状态指针。
 * @param sample_started Encoder_BeginSample() 的返回值；false 时走同步兜底读取。
 * @note 与 BeginSample 由同一调用方配对执行，故障路径也必须调用。
 */
void Encoder_CompleteSample(MotorControl_TypeDef *MotorControl,
                            Encoder_TypeDef *Encoder,
                            bool sample_started);

/**
 * @brief 判断编码器是否在线。
 * @param Encoder 编码器状态指针。
 * @return 已有有效样本且连续坏帧少于 ENCODER_BAD_FRAME_OFFLINE_COUNT 时返回 true。
 */
bool Encoder_IsOnline(const Encoder_TypeDef *Encoder);

/**
 * @brief 以当前线性化角度作为电零位（mode 15 电零位标定的落点）。
 * @param Encoder 编码器状态指针。
 * @return 设置成功返回 true；编码器离线时返回 false 且不改写。
 */
bool Encoder_SetElectricalZero(Encoder_TypeDef *Encoder);

/**
 * @brief 写入指定的 Q15 电零位并置位电零位标定标志。
 * @param Encoder 编码器状态指针。
 * @param electrical_zero_q15 电零位，Q15（0..65535）。
 * @return 设置成功返回 true；编码器离线时返回 false 且不改写。
 */
bool Encoder_SetElectricalZeroQ15(Encoder_TypeDef *Encoder, uint16_t electrical_zero_q15);

/**
 * @brief 以当前线性化角度作为机械零位（位置原点）。
 * @param Encoder 编码器状态指针。
 * @return 设置成功返回 true；编码器离线时返回 false 且不改写。
 * @note 只影响位置原点，不等价于电零位标定。
 */
bool Encoder_SetMechanicalZero(Encoder_TypeDef *Encoder);

/**
 * @brief 设置编码器方向；改变方向会清空 LUT、电零位与机械零位标定。
 * @param Encoder 编码器状态指针。
 * @param reverse true 表示反向。
 * @note 在临界区内完成，避免 20 kHz 环读到中间状态。
 */
void Encoder_SetReverse(Encoder_TypeDef *Encoder, bool reverse);

/**
 * @brief 复位速度估计窗口（停机、模式切换或首个有效样本后调用）。
 * @param Encoder 编码器状态指针。
 */
void Encoder_ResetVelocity(Encoder_TypeDef *Encoder);

/**
 * @brief 读取电角速度。
 * @param Encoder 编码器状态指针。
 * @return 电角速度，单位 rad/s。
 */
float Encoder_GetEleVel(const Encoder_TypeDef *Encoder);

/**
 * @brief 读取机械角速度。
 * @param Encoder 编码器状态指针。
 * @return 机械角速度，单位 rad/s；窗口未满或低于零速死区时为 0。
 */
float Encoder_GetMecVel(const Encoder_TypeDef *Encoder);

/**
 * @brief 读取连续机械角速度，不含零速死区。
 * @param Encoder 编码器状态指针。
 * @return 机械角速度，单位 rad/s；速度窗口未满时为 0。
 */
float Encoder_GetMecVelContinuous(const Encoder_TypeDef *Encoder);

/**
 * @brief 判断最近一次成功采样是否同时刷新了分频后的速度估计。
 * @param Encoder 编码器状态指针。
 * @return 已刷新返回 true；坏帧或未到分频点时返回 false。
 * @note 供慢速遥测避免与快速环重复计算。
 */
bool Encoder_DidUpdateVelocity(const Encoder_TypeDef *Encoder);

/**
 * @brief 读取电角度。
 * @param Encoder 编码器状态指针。
 * @return 电角度，单位 rad。
 */
float Encoder_GetElePhase(const Encoder_TypeDef *Encoder);

/**
 * @brief 读取机械角度（相对机械零位的多圈位置）。
 * @param Encoder 编码器状态指针。
 * @return 机械角度，单位 rad。
 */
float Encoder_GetMecPos(const Encoder_TypeDef *Encoder);

/**
 * @brief 读取线性化后角度占整圈的比例。
 * @param Encoder 编码器状态指针。
 * @return [0,1) 的比例值，供调试与诊断使用。
 */
float Encoder_GetCountInCPR_Ratio(const Encoder_TypeDef *Encoder);

/**
 * @brief 读取标定标志位。
 * @param Encoder 编码器状态指针。
 * @return 标定位图，取值为 ENC_CALIB_* 的组合。
 */
uint8_t Encoder_GetCalibFlag(const Encoder_TypeDef *Encoder);

/**
 * @brief 读取连续坏帧计数。
 * @param Encoder 编码器状态指针。
 * @return 自最近一次有效帧以来的连续坏帧数。
 */
uint16_t Encoder_GetBadFrameStreak(const Encoder_TypeDef *Encoder);

/**
 * @brief 读取编码器方向配置。
 * @param Encoder 编码器状态指针。
 * @return 1 表示反向，0 表示正向。
 */
uint8_t Encoder_GetReverse(const Encoder_TypeDef *Encoder);

#endif
