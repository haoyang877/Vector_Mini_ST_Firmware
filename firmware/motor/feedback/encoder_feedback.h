#ifndef ENCODER_FEEDBACK_H
#define ENCODER_FEEDBACK_H

#include <stdbool.h>
#include <stdint.h>
#include "axis_config.h"
#include "encoder_config.h"

/* 角度反馈（motor 层拥有）：传感器采样 -> 方向/线性化/电零位/多圈/速度。
 * 两板消费统一的直接角度读数，数学处理使用65536计数域。
 * 本层不访问外设寄存器。 */

/* 单圈角度表示：无符号 Q15，范围 [0, 65535]。 */
#define ENCODER_Q15_CPR 65536UL
#define ENCODER_Q15_HALF_TURN 32768

/* 机械速度估计：20 kHz 快路径的二阶角度跟踪观测器（PLL）。
 * 带宽 ENCODER_PLL_OMEGA_N_HZ、阻尼比 ENCODER_PLL_ZETA；输出机械角速度 rad/s。
 * ENCODER_PLL_OMEGA_MAX_RAD_S 为输出限幅（防坏帧/极端输入发散）；
 * ENCODER_PLL_VEL_ZERO_THRESHOLD_RAD_S 为软零速死区（|vel| 低于它时 mechanical_velocity_rad_s 归零，
 * mechanical_velocity_continuous_rad_s 不受影响）。观测器只出速度，不参与换相、位置或标定路径。 */
/* 默认取低纹波点；PLL 常量位于反馈实现，不叠加滤波器，取舍见决策0002。 */

/** 编码器运行状态：20 kHz 采样/速度与 2 kHz 位置；不拥有持久化校准。 */
typedef struct
{
    /* 初始化时借用反馈配置；参数所有者原位更新，所有对象须持续有效。 */
    const EncoderCalibration *calibration;
    const int32_t *pole_pairs;
    const MotorAxisProfile *axis_profile;
    const bool *axis_profile_valid;

    /* 20 kHz 快路径写：原始读数、方向校正值、LUT 校正值与电角度。 */
    uint16_t raw_q15;
    uint16_t directed_q15;
    uint16_t linearized_q15;
    bool has_valid_sample;
    float electrical_angle_rad;
    /* 快路径每产生一帧有效样本递增；慢估计据此判断是否有新样本。 */
    volatile uint32_t sample_epoch;

    /* 2 kHz 慢估计为下列位置的唯一写者；快路径只置请求标志。 */
    uint16_t previous_linearized_q15;
    int64_t shadow_q15;
    int64_t mechanical_zero_shadow_q15;
    volatile bool rebase_requested;
    volatile bool zero_requested;
    volatile bool velocity_restart_requested;
    uint32_t slow_sample_epoch;

    /* 20 kHz 快路径写的角度跟踪观测器（PLL）状态；速度状态属快路径所有。 */
    float pll_theta_rad;
    float pll_omega_rad_s;
    bool velocity_ready;

    /* 对外提供的机械/电角度与角速度反馈。 */
    float mechanical_position_rad;
    float electrical_velocity_rad_s;
    float mechanical_velocity_rad_s;
    float mechanical_velocity_continuous_rad_s;
} EncoderState;

/**
 * @brief 上电初始化编码器运行状态并绑定快慢估计所需配置。
 * @param calibration 已装载的设备校准；方向归一化为 0/1，借用对象须持续有效。
 * @param pole_pairs 电机极对数的只读借用；原位更新下拍生效，小于等于 0 时按 1 处理。
 * @param axis_profile 只读轴记录，慢估计重定多圈基准时读取最新值。
 * @param axis_profile_valid 轴记录有效标志的只读借用，与轴记录一起由参数所有者维护。
 * @param encoder 编码器运行状态指针。
 * @note 使用前调用；所有配置指针须非空且持续有效，更换地址时须重新初始化。
 *       外设初始化由BSP启动流程完成。
 */
void Encoder_InitFeedback(EncoderCalibration *calibration,
                          const int32_t *pole_pairs,
                          const MotorAxisProfile *axis_profile,
                          const bool *axis_profile_valid,
                          EncoderState *encoder);

/**
 * @brief 读取本拍角度并更新电角度、方向校正和速度估计。
 * @param encoder 编码器状态指针。
 * @note 须先通过 Encoder_InitFeedback 绑定校准与极对数；每拍读取绑定对象的最新值。
 *       两板调用前均须先BspEncoder_Begin；固定选择16位计数，直接消费传输，不返回采集状态。
 */
void Encoder_Update(EncoderState *encoder);

/**
 * @brief 执行一次 2 kHz 编码器慢估计：多圈累积与机械角。
 * @param encoder 编码器状态指针。
 * @note 仅由 2 kHz 监督 tick 调用；本函数是慢位置状态的唯一写者，快路径只能通过
 *       rebase_requested/zero_requested 请求变更。角速度不在此计算，见快路径 PLL。
 *       须先通过 Encoder_InitFeedback 绑定配置；重定基准时读取最新轴记录与机械零位。
 */
void Encoder_UpdateSlowEstimate(EncoderState *encoder);

/**
 * @brief 判断编码器是否已有角度样本。
 * @param encoder 编码器状态指针。
 * @return 首次角度采样完成后返回 true。
 * @note 仅表示已取得样本，不诊断传感器连接或帧错误。
 */
bool Encoder_IsOnline(const EncoderState *encoder);

/**
 * @brief 以当前线性化角度作为电零位（mode 15 电零位标定的落点）。
 * @param calibration 设备校准可写视图，仅由参数/维护所有者提供。
 * @param encoder 编码器状态指针。
 * @return 设置成功返回 true；尚无角度样本时返回 false 且不改写。
 */
bool Encoder_SetElectricalZero(EncoderCalibration *calibration, EncoderState *encoder);

/**
 * @brief 写入指定的 Q15 电零位并置位电零位标定标志。
 * @param calibration 设备校准可写视图，仅由参数/维护所有者提供。
 * @param encoder 编码器状态指针。
 * @param electrical_zero_q15 电零位，Q15（0..65535）。
 * @return 设置成功返回 true；尚无角度样本时返回 false 且不改写。
 */
bool Encoder_SetElectricalZeroQ15(EncoderCalibration *calibration,
                                  EncoderState *encoder,
                                  uint16_t electrical_zero_q15);

/**
 * @brief 以当前线性化角度作为机械零位（位置原点）。
 * @param calibration 设备校准可写视图，仅由参数/维护所有者提供。
 * @param encoder 编码器状态指针。
 * @return 设置成功返回 true；尚无角度样本时返回 false 且不改写。
 * @note 只影响位置原点，不等价于电零位标定。
 */
bool Encoder_SetMechanicalZero(EncoderCalibration *calibration, EncoderState *encoder);

/**
 * @brief 设置编码器方向；改变方向会清空 LUT、电零位与机械零位标定。
 * @param calibration 设备校准可写视图，仅由参数/维护所有者提供。
 * @param encoder 编码器状态指针。
 * @param reverse true 表示反向。
 * @note 在临界区内完成，避免 20 kHz 环读到中间状态。
 */
void Encoder_SetReverse(EncoderCalibration *calibration, EncoderState *encoder, bool reverse);

/**
 * @brief 请求复位速度估计窗口（停机、模式切换或首个有效样本后调用）。
 * @param encoder 编码器状态指针。
 * @note 请求在下一 20 kHz 有效采样时生效；可从任意中断上下文调用。
 */
void Encoder_ResetVelocity(EncoderState *encoder);

/**
 * @brief 读取电角速度。
 * @param encoder 编码器状态指针。
 * @return 电角速度，单位 rad/s。
 */
float Encoder_GetElecVel(const EncoderState *encoder);

/**
 * @brief 读取机械角速度。
 * @param encoder 编码器状态指针。
 * @return 机械角速度，单位 rad/s；观测器未就绪或低于软零速死区时为 0。
 */
float Encoder_GetMechVel(const EncoderState *encoder);

/**
 * @brief 读取连续机械角速度，不含零速死区。
 * @param encoder 编码器状态指针。
 * @return 机械角速度，单位 rad/s；观测器未就绪时为 0。
 */
float Encoder_GetMechVelContinuous(const EncoderState *encoder);

/**
 * @brief 读取电角度。
 * @param encoder 编码器状态指针。
 * @return 电角度，单位 rad。
 */
float Encoder_GetElecPhase(const EncoderState *encoder);

/**
 * @brief 读取机械角度（相对机械零位的多圈位置）。
 * @param encoder 编码器状态指针。
 * @return 机械角度，单位 rad。
 */
float Encoder_GetMechPos(const EncoderState *encoder);

/**
 * @brief 读取标定标志位。
 * @param calibration 设备校准只读视图。
 * @return 标定位图，取值为 ENC_CALIB_* 的组合。
 */
uint8_t Encoder_GetCalibrationFlag(const EncoderCalibration *calibration);

#endif
