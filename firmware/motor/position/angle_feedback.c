#include "angle_feedback.h"

#include <limits.h>
#include <string.h>
#include "control_config.h"
#include "critical_hw.h"
#include "encoder_sensor.h"
#include "utils.h"

/* 角度反馈（motor 层拥有）：帧读取编排与电角度（20 kHz 快路径）、多圈/机械角/
 * 速度慢估计（2 kHz 独占写）。传感器通道见 platform/api/encoder_sensor.h，
 * SPI 传输见 platform/stm32g4/ports/motor/encoder_spi_stm32g4.c；本文件不访问寄存器。
 *
 * 数据流：传感器单圈角 raw_q15 -> 方向校正 directed_q15 -> LUT 线性化 linearized_q15；
 *         快路径：linearized_q15 -> 电角度 theta_elec（20 kHz 每有效帧）；
 *         慢路径：linearized_q15 -> 多圈 shadow_q15 -> theta_mech 与滑动平均速度（2 kHz）。
 *
 * 所有权约定：快路径只写即时量（raw/directed/linearized/theta_elec/sample_epoch）；
 * 慢状态（多圈/机械角/速度）由 Encoder_UpdateSlowEstimate() 独占写，快路径需要变更时
 * 只置 rebase_requested/zero_requested/velocity_restart_requested，由下一拍 2 kHz
 * 慢估计统一消费，避免跨中断读改写撕裂。 */

/* 速度零死区：16 样本窗口（2 kHz 下 8 ms）内增量和的绝对值不超过该值时 vel_mech 归零，
 * 约 0.096 rad/s，用于抑制静止时的量化抖动；vel_mech_continuous 不受该死区影响。 */
#define ENCODER_VELOCITY_ZERO_THRESHOLD_Q15 8

/* 登记一次读取结果；出现有效帧时清零连续坏帧计数。
 * 输入：encoder 状态归属；status 本次读取状态。
 * 失败时：锁存 read_status_latched、read_error_count 累加、bad_frame_streak 在
 * UINT16_MAX 处饱和（离线判定阈值见 Encoder_IsOnline）。无返回值。 */
static void Encoder_MarkReadStatus(Encoder_TypeDef *encoder, Encoder_ReadStatus status)
{
    encoder->read_status = status;
    if (status == ENCODER_READ_OK)
    {
        encoder->bad_frame_streak = 0U;
        return;
    }

    encoder->read_status_latched = status;
    encoder->read_error_count++;
    if (encoder->bad_frame_streak < UINT16_MAX)
    {
        encoder->bad_frame_streak++;
    }
}

/* 非阻塞发起一次传感器采样；总线异常时保留同步兜底路径。
 * 输入：无（单板传感器通道）。
 * 返回：已发起 true；总线未就绪 false，此时调用方仍须调用
 *       Encoder_CompleteSample(..., false) 走同步兜底完成本帧。
 * 仅 20 kHz 快环调用；两次调用之间禁止访问传感器总线。 */
bool Encoder_BeginSample(void)
{
    return encoder_sensor_begin();
}

/* 通道状态映射到既有对外读取状态；旧枚举保留以兼容遥测。
 * 输入：平台层 EncoderSensorStatus；返回：本层 Encoder_ReadStatus：
 * 帧错误->CRC_MISMATCH、传感器故障->TLE_SYSTEM_ERROR、总线超时/其他->SPI_TIMEOUT。 */
static Encoder_ReadStatus Encoder_MapSensorStatus(EncoderSensorStatus status)
{
    switch (status)
    {
    case ENCODER_SENSOR_FRAME_ERROR:
        return ENCODER_READ_CRC_MISMATCH;
    case ENCODER_SENSOR_SENSOR_FAULT:
        return ENCODER_READ_TLE_SYSTEM_ERROR;
    case ENCODER_SENSOR_BUS_TIMEOUT:
    default:
        return ENCODER_READ_SPI_TIMEOUT;
    }
}

/* 采样一帧并取出归一化 Q15 角度；失败时登记映射后的读取状态。
 * 输入：encoder 状态归属；raw_q15 输出参数，仅成功时写入单圈角 Q15；
 *       started 为 Encoder_BeginSample() 的返回值，false 时驱动自行发起同步读取。
 * 返回：成功 true（角度已写入）；失败 false（角度未写入）。
 * 成功时缓存 frame_word/safety_word 供诊断，遗留 CRC 字段清零。 */
static bool Encoder_ReadFrame(Encoder_TypeDef *encoder, uint16_t *raw_q15, bool started)
{
    EncoderSensorSample sample;

    if (encoder_sensor_complete(started, &sample) != ENCODER_SENSOR_OK)
    {
        Encoder_MarkReadStatus(encoder, Encoder_MapSensorStatus(sample.status));
        return false;
    }

    encoder->frame_word = sample.frame_word;
    encoder->safety_word = sample.safety_word;
    encoder->crc_received = 0U;
    encoder->crc_calculated = 0U;
    *raw_q15 = sample.angle_q15;
    Encoder_MarkReadStatus(encoder, ENCODER_READ_OK);
    return true;
}

/* 按方向配置取反单圈角度。
 * 输入：encoder（读 reverse）、raw_q15 原始单圈角。
 * 返回：方向校正后的 Q15 角；reverse == 0 原样返回，
 *       否则返回 0 - raw_q15 的模 65536 补码。 */
static uint16_t Encoder_ApplyDirectionQ15(const Encoder_TypeDef *encoder, uint16_t raw_q15)
{
    if (encoder->reverse == 0U)
    {
        return raw_q15;
    }

    return (uint16_t)(0U - raw_q15);
}

/* 用 1024 点 Q15 表做线性插值校正（每点覆盖 64 count，磁编码器偏心/非线性补偿）。
 * 输入：encoder（读 linearization_lut_q15）、已做方向校正的 raw_q15。
 * 返回：减去插值校正量后的 Q15 角（模 65536）；LUT 全零表示不校正。
 * 索引：index = raw_q15 >> 6，小数部分取低 6 位；末点回绕到表头做环形插值。 */
static uint16_t Encoder_ApplyLinearizationQ15(const Encoder_TypeDef *encoder, uint16_t raw_q15)
{
    uint16_t lut_index = raw_q15 >> 6;
    uint16_t fraction = raw_q15 & 0x003FU;
    int32_t correction_a = encoder->linearization_lut_q15[lut_index];
    int32_t correction_b =
        encoder->linearization_lut_q15[(lut_index + 1U) & (ENCODER_OFFSET_LUT_SIZE - 1U)];
    int32_t correction = correction_a + (((correction_b - correction_a) * fraction) >> 6);

    return (uint16_t)((int32_t)raw_q15 - correction);
}

/* 由线性化角与电零位得到电角度（20 kHz 快路径每拍更新）。
 * 输入：encoder、pole_pairs 极对数（调用方保证 >= 1）。
 * 返回：无返回值；写 theta_elec，单位 rad，范围 [0, 2π)。
 * 公式：(linearized_q15 - electrical_zero_q15) * pole_pairs 截断回 uint16，
 *       等价于对 65536 取模，天然完成电角度回绕，再乘 2π/65536。 */
static void Encoder_UpdateElecAngle(Encoder_TypeDef *encoder, uint32_t pole_pairs)
{
    uint16_t electrical_q15;

    electrical_q15 =
        (uint16_t)((uint32_t)(uint16_t)(encoder->linearized_q15 - encoder->electrical_zero_q15) *
                   pole_pairs);
    encoder->theta_elec = (float)electrical_q15 * (_2PI / (float)ENCODER_Q15_CPR);
}

/* 复位速度窗口：清历史、同步影子位置并清请求；仅慢估计上下文调用。
 * 输入：encoder；无返回值。
 * 输出/副作用：清增量历史/索引/样本数与增量和，velocity_ready = false，
 *              速度影子对齐 shadow_q15，三个速度输出归零，
 *              并清除 velocity_restart_requested（请求在此消费）。 */
static void Encoder_RestartVelocityWindow(Encoder_TypeDef *encoder)
{
    memset(encoder->velocity_delta_history, 0, sizeof(encoder->velocity_delta_history));
    encoder->velocity_history_index = 0U;
    encoder->velocity_sample_count = 0U;
    encoder->velocity_delta_sum = 0;
    encoder->velocity_ready = false;
    encoder->velocity_shadow_q15 = encoder->shadow_q15;
    encoder->vel_mech = 0.0f;
    encoder->vel_mech_continuous = 0.0f;
    encoder->vel_elec = 0.0f;
    encoder->velocity_restart_requested = false;
}

/* 每帧有效样本的多圈累积：半圈判定后累加最短路径增量。
 * 输入：encoder；无返回值。
 * 输出：previous_linearized_q15、shadow_q15（无界多圈位置，Q15 count）。
 * delta 超过 ±32768 时按 ±65536 折算，处理单圈回绕；shadow_q15 为 int64，
 * 可长期累积多圈而不溢出。 */
static void Encoder_UpdateShadow(Encoder_TypeDef *encoder)
{
    int32_t delta_q15 =
        (int32_t)encoder->linearized_q15 - (int32_t)encoder->previous_linearized_q15;

    if (delta_q15 > ENCODER_Q15_HALF_TURN)
    {
        delta_q15 -= (int32_t)ENCODER_Q15_CPR;
    }
    else if (delta_q15 < -ENCODER_Q15_HALF_TURN)
    {
        delta_q15 += (int32_t)ENCODER_Q15_CPR;
    }

    encoder->previous_linearized_q15 = encoder->linearized_q15;
    encoder->shadow_q15 += delta_q15;
}

/* 2 kHz 分频的 16 样本滑动平均速度估计；慢估计上下文每拍调用一次。
 * 输入：encoder、pole_pairs 极对数；无返回值。
 * 输出：vel_mech_continuous（连续量，无死区）、vel_mech（含零速死区）、vel_elec。
 * 窗口 = 16 x 0.5 ms = 8 ms；增量先钳位到 int32 再进环形缓冲；
 * 窗口满后 scale = 2π / (65536 x 16 x Supervisor_Ts)。 */
static void Encoder_UpdateVelocity(Encoder_TypeDef *encoder, uint32_t pole_pairs)
{
    int64_t delta64;
    int32_t delta_q15;
    int32_t sum_abs;
    float velocity_scale;

    delta64 = encoder->shadow_q15 - encoder->velocity_shadow_q15;
    encoder->velocity_shadow_q15 = encoder->shadow_q15;
    if (delta64 > INT32_MAX)
    {
        delta_q15 = INT32_MAX;
    }
    else if (delta64 < INT32_MIN)
    {
        delta_q15 = INT32_MIN;
    }
    else
    {
        delta_q15 = (int32_t)delta64;
    }

    encoder->velocity_delta_sum -= encoder->velocity_delta_history[encoder->velocity_history_index];
    encoder->velocity_delta_history[encoder->velocity_history_index] = delta_q15;
    encoder->velocity_delta_sum += delta_q15;
    encoder->velocity_history_index =
        (uint8_t)((encoder->velocity_history_index + 1U) % ENCODER_VELOCITY_WINDOW);
    if (encoder->velocity_sample_count < ENCODER_VELOCITY_WINDOW)
    {
        encoder->velocity_sample_count++;
    }
    encoder->velocity_ready = encoder->velocity_sample_count == ENCODER_VELOCITY_WINDOW;
    velocity_scale =
        _2PI / ((float)ENCODER_Q15_CPR * (float)ENCODER_VELOCITY_WINDOW * Supervisor_Ts);
    encoder->vel_mech_continuous =
        encoder->velocity_ready ? (float)encoder->velocity_delta_sum * velocity_scale : 0.0f;

    sum_abs = encoder->velocity_delta_sum;
    if (sum_abs < 0)
    {
        sum_abs = -sum_abs;
    }
    if (!encoder->velocity_ready || sum_abs <= ENCODER_VELOCITY_ZERO_THRESHOLD_Q15)
    {
        encoder->vel_mech = 0.0f;
    }
    else
    {
        encoder->vel_mech = encoder->vel_mech_continuous;
    }
    encoder->vel_elec = encoder->vel_mech * (float)pole_pairs;
}

/* 首样本/重定多圈基准：用当前线性化角、机械零位与轴策略确立多圈位置基准；仅慢估计调用。
 * 输入：MotorControl（轴配置与有效性）、encoder；无返回值。
 * 输出：previous_linearized_q15、shadow_q15、mechanical_zero_shadow_q15。
 * 未标定机械零位：影子零位取 0，shadow 以当前线性化角为基准；
 * 已标定：影子零位取 mechanical_zero_q15，并用
 * MotorAxisProfile_InitialEncoderOffsetQ15() 解析首样本圈数，使便携轴重启后
 * 多圈位置连续、行程检查仍然有效。 */
static void Encoder_RebaseMultiTurnPosition(const MotorControl_TypeDef *MotorControl,
                                            Encoder_TypeDef *encoder)
{
    encoder->previous_linearized_q15 = encoder->linearized_q15;
    encoder->shadow_q15 = encoder->linearized_q15;
    if ((encoder->calib_flag & ENC_CALIB_MECHANICAL_ZERO) == 0U)
    {
        encoder->mechanical_zero_shadow_q15 = 0;
    }
    else
    {
        encoder->mechanical_zero_shadow_q15 = (int64_t)encoder->mechanical_zero_q15;
        /* 只解析首样本的圈数；便携轴策略保留已标定零位并保持行程检查有效。 */
        encoder->shadow_q15 =
            encoder->mechanical_zero_shadow_q15 +
            MotorAxisProfile_InitialEncoderOffsetQ15(&MotorControl->axis_profile,
                                                     MotorControl->axis_profile_valid,
                                                     (int32_t)encoder->linearized_q15 -
                                                         (int32_t)encoder->mechanical_zero_q15);
    }
}

/* 请求复位速度估计窗口（停机、模式切换或首个有效样本后调用）。
 * 输入：encoder；无返回值。
 * 只置 velocity_restart_requested，由下一拍 Encoder_UpdateSlowEstimate() 生效；
 * 可从任意中断上下文调用，不会与慢估计写者撕裂。 */
void Encoder_ResetVelocity(Encoder_TypeDef *encoder)
{
    /* 慢状态由 2 kHz 慢估计独占写：此处只登记请求，避免跨中断撕裂。 */
    encoder->velocity_restart_requested = true;
}

/* 设置编码器方向；改变方向会清空 LUT、电零位、机械零位与全部角度状态。
 * 输入：encoder、reverse（true 表示反向）；无返回值。
 * 同值调用直接返回；变化时在临界区内完成写入与复位，避免 20 kHz 环读到中间状态，
 * 最后请求复位速度窗口。清标定后须重新标定，否则电角度不可用。 */
void Encoder_SetReverse(Encoder_TypeDef *encoder, bool reverse)
{
    uint8_t reverse_value = reverse ? 1U : 0U;
    uint32_t primask;

    if (encoder->reverse == reverse_value)
    {
        return;
    }

    primask = critical_hw_enter();
    encoder->reverse = reverse_value;
    encoder->electrical_zero_q15 = 0U;
    encoder->mechanical_zero_q15 = 0U;
    encoder->calib_flag = 0U;
    memset(encoder->linearization_lut_q15, 0, sizeof(encoder->linearization_lut_q15));
    encoder->raw_q15 = 0U;
    encoder->directed_q15 = 0U;
    encoder->linearized_q15 = 0U;
    encoder->has_valid_sample = false;
    encoder->theta_elec = 0.0f;
    encoder->shadow_q15 = 0;
    encoder->mechanical_zero_shadow_q15 = 0;
    encoder->previous_linearized_q15 = 0U;
    encoder->theta_mech = 0.0f;
    Encoder_ResetVelocity(encoder);
    critical_hw_exit(primask);
}

/* 上电初始化编码器状态与传感器通道。
 * 输入：encoder；持久化字段（方向/零位/LUT）须在调用前由参数服务填充；无返回值。
 * 方向值归一化为 0/1；清空快/慢/诊断状态；复位速度窗口；调用 encoder_sensor_init()。
 * SPI 初始化失败走致命错误路径，不返回。 */
void Encoder_ParamInit(Encoder_TypeDef *encoder)
{
    encoder->reverse = encoder->reverse != 0U ? 1U : 0U;
    encoder->raw_q15 = 0U;
    encoder->directed_q15 = 0U;
    encoder->linearized_q15 = 0U;
    encoder->previous_linearized_q15 = 0U;
    encoder->shadow_q15 = 0;
    encoder->mechanical_zero_shadow_q15 = (int64_t)encoder->mechanical_zero_q15;
    encoder->velocity_shadow_q15 = 0;
    encoder->has_valid_sample = false;
    encoder->theta_elec = 0.0f;
    encoder->theta_mech = 0.0f;
    encoder->sample_epoch = 0U;
    encoder->slow_sample_epoch = 0U;
    encoder->rebase_requested = false;
    encoder->zero_requested = false;
    encoder->read_status = ENCODER_READ_OK;
    encoder->read_status_latched = ENCODER_READ_OK;
    encoder->frame_word = 0U;
    encoder->safety_word = 0U;
    encoder->crc_received = 0U;
    encoder->crc_calculated = 0U;
    encoder->crc_error_count = 0U;
    encoder->read_error_count = 0U;
    encoder->bad_frame_streak = 0U;
    Encoder_RestartVelocityWindow(encoder);

    encoder_sensor_init();
}

/* 判断编码器是否在线。
 * 输入：encoder；返回：已有有效样本且连续坏帧 < ENCODER_BAD_FRAME_OFFLINE_COUNT
 * （20 kHz 下约 5 ms）时为 true。 */
bool Encoder_IsOnline(const Encoder_TypeDef *encoder)
{
    return encoder->has_valid_sample && encoder->bad_frame_streak < ENCODER_BAD_FRAME_OFFLINE_COUNT;
}

/* 写入指定的 Q15 电零位并置位电零位标定标志。
 * 输入：encoder、electrical_zero_q15（Q15，0..65535）。
 * 返回：设置成功 true；编码器离线 false 且不改写。 */
bool Encoder_SetElectricalZeroQ15(Encoder_TypeDef *encoder, uint16_t electrical_zero_q15)
{
    if (!Encoder_IsOnline(encoder))
    {
        return false;
    }

    encoder->electrical_zero_q15 = electrical_zero_q15;
    encoder->calib_flag |= ENC_CALIB_ELECTRICAL_ZERO;
    return true;
}

/* 以当前线性化角度作为电零位（mode 15 电零位标定的落点）。
 * 输入：encoder；返回：设置成功 true；编码器离线 false 且不改写。 */
bool Encoder_SetElectricalZero(Encoder_TypeDef *encoder)
{
    return Encoder_SetElectricalZeroQ15(encoder, encoder->linearized_q15);
}

/* 以当前线性化角度作为机械零位（位置原点）。
 * 输入：encoder；返回：设置成功 true；编码器离线 false 且不改写。
 * 零位在下一 2 kHz 慢估计时生效：机械零位影子取当时多圈值，theta_mech 归零；
 * 只影响位置原点，不等价于电零位标定。临界区内写入并置 zero_requested。 */
bool Encoder_SetMechanicalZero(Encoder_TypeDef *encoder)
{
    uint32_t primask;

    if (!Encoder_IsOnline(encoder))
    {
        return false;
    }

    primask = critical_hw_enter();
    encoder->mechanical_zero_q15 = encoder->linearized_q15;
    encoder->calib_flag |= ENC_CALIB_MECHANICAL_ZERO;
    /* 零位在下一 2 kHz 慢估计时生效：机械零位影子取当时多圈值，theta_mech 归零。 */
    encoder->zero_requested = true;
    critical_hw_exit(primask);
    return true;
}

/* 完成一帧读取：方向/LUT 校正与电角度更新（20 kHz 快路径）。
 * 输入：MotorControl（读 motor_pole_pairs，<= 0 按 1 处理）、encoder、
 *       sample_started 为 Encoder_BeginSample() 的返回值；无返回值。
 * 流程：取帧 -> 方向校正 -> LUT 线性化 -> 更新 raw/directed/linearized
 *       -> 首个有效帧置 has_valid_sample 并请求重定多圈基准（由慢估计执行）
 *       -> sample_epoch++ -> 更新 theta_elec。
 * 取帧失败时仅登记读取状态，theta_elec 保持旧值；本函数不写多圈/机械角/速度。 */
void Encoder_CompleteSample(MotorControl_TypeDef *MotorControl,
                            Encoder_TypeDef *encoder,
                            bool sample_started)
{
    uint16_t raw_q15;
    uint16_t directed_q15;
    uint16_t linearized_q15;
    uint32_t pole_pairs;

    if (!Encoder_ReadFrame(encoder, &raw_q15, sample_started))
    {
        return;
    }

    directed_q15 = Encoder_ApplyDirectionQ15(encoder, raw_q15);
    linearized_q15 = Encoder_ApplyLinearizationQ15(encoder, directed_q15);
    encoder->raw_q15 = raw_q15;
    encoder->directed_q15 = directed_q15;
    encoder->linearized_q15 = linearized_q15;
    pole_pairs = MotorControl->motor_pole_pairs > 0 ? (uint32_t)MotorControl->motor_pole_pairs : 1U;

    if (!encoder->has_valid_sample)
    {
        encoder->has_valid_sample = true;
        /* 首个有效帧：2 kHz 慢估计重建多圈基准并复位速度窗口。 */
        encoder->rebase_requested = true;
    }
    encoder->sample_epoch++;
    Encoder_UpdateElecAngle(encoder, pole_pairs);
}

/* 2 kHz 慢估计：多圈/机械角/速度为慢状态唯一写者；快路径只置请求标志。
 * 输入：MotorControl（极对数与轴配置）、encoder；无返回值。
 * 顺序：有新样本时按 rebase_requested 走重定多圈基准（并复位速度窗口），否则累积多圈并
 *       更新速度；随后消费 zero_requested（机械零位影子对齐当前多圈值）；
 *       再消费 velocity_restart_requested；最后每拍重算 theta_mech 与 vel_elec
 *       ——即使没有新样本也更新，保证零位请求下一拍立即生效。 */
void Encoder_UpdateSlowEstimate(MotorControl_TypeDef *MotorControl, Encoder_TypeDef *encoder)
{
    uint32_t pole_pairs =
        MotorControl->motor_pole_pairs > 0 ? (uint32_t)MotorControl->motor_pole_pairs : 1U;
    bool new_sample =
        encoder->has_valid_sample && encoder->sample_epoch != encoder->slow_sample_epoch;

    if (new_sample)
    {
        if (encoder->rebase_requested)
        {
            Encoder_RebaseMultiTurnPosition(MotorControl, encoder);
            encoder->rebase_requested = false;
            Encoder_RestartVelocityWindow(encoder);
        }
        else
        {
            Encoder_UpdateShadow(encoder);
            Encoder_UpdateVelocity(encoder, pole_pairs);
        }
        encoder->slow_sample_epoch = encoder->sample_epoch;
    }
    if (encoder->zero_requested)
    {
        encoder->mechanical_zero_shadow_q15 = encoder->shadow_q15;
        encoder->zero_requested = false;
    }
    if (encoder->velocity_restart_requested)
    {
        Encoder_RestartVelocityWindow(encoder);
    }
    encoder->theta_mech = (float)(encoder->shadow_q15 - encoder->mechanical_zero_shadow_q15) *
                          (_2PI / (float)ENCODER_Q15_CPR);
    encoder->vel_elec = encoder->vel_mech * (float)pole_pairs;
}

/* 同步读取一帧角度并更新电角度（20 kHz 快路径兼容入口）。
 * 输入：MotorControl、encoder；无返回值。
 * 等价于 Encoder_CompleteSample(..., false)；新代码使用 BeginSample/CompleteSample
 * 分离采样，以覆盖传感器 SPI 事务的整拍时延。 */
void Encoder_Update(MotorControl_TypeDef *MotorControl, Encoder_TypeDef *encoder)
{
    Encoder_CompleteSample(MotorControl, encoder, false);
}

/* 读取电角度。输入：encoder；返回：theta_elec，单位 rad，[0, 2π)，20 kHz 每帧更新。 */
float Encoder_GetElePhase(const Encoder_TypeDef *encoder)
{
    return encoder->theta_elec;
}

/* 读取机械角度（相对机械零位的多圈位置）。输入：encoder；返回：theta_mech，单位 rad。 */
float Encoder_GetMecPos(const Encoder_TypeDef *encoder)
{
    return encoder->theta_mech;
}

/* 读取电角速度。输入：encoder；返回：vel_elec，单位 rad/s，等于 vel_mech x 极对数。 */
float Encoder_GetEleVel(const Encoder_TypeDef *encoder)
{
    return encoder->vel_elec;
}

/* 读取机械角速度。输入：encoder；返回：vel_mech，单位 rad/s；
 * 速度窗口未满或窗口增量低于零速死区时为 0。 */
float Encoder_GetMecVel(const Encoder_TypeDef *encoder)
{
    return encoder->vel_mech;
}

/* 读取连续机械角速度，不含零速死区。输入：encoder；返回：vel_mech_continuous，
 * 单位 rad/s；速度窗口未满时为 0（供观测器等需要连续量纲的消费者）。 */
float Encoder_GetMecVelContinuous(const Encoder_TypeDef *encoder)
{
    return encoder->vel_mech_continuous;
}

/* 读取线性化后角度占整圈的比例。输入：encoder；返回：[0,1) 的比例，供调试与诊断。 */
float Encoder_GetCountInCPR_Ratio(const Encoder_TypeDef *encoder)
{
    return (float)encoder->linearized_q15 / (float)ENCODER_Q15_CPR;
}

/* 读取标定标志位。输入：encoder；返回：ENC_CALIB_* 位图（线性化/电零位/机械零位）。 */
uint8_t Encoder_GetCalibFlag(const Encoder_TypeDef *encoder)
{
    return encoder->calib_flag;
}

/* 读取连续坏帧计数。输入：encoder；返回：自最近一次有效帧以来的连续坏帧数，
 * 在 UINT16_MAX 处饱和（离线判定见 Encoder_IsOnline）。 */
uint16_t Encoder_GetBadFrameStreak(const Encoder_TypeDef *encoder)
{
    return encoder->bad_frame_streak;
}

/* 读取编码器方向配置。输入：encoder；返回：1 表示反向，0 表示正向。 */
uint8_t Encoder_GetReverse(const Encoder_TypeDef *encoder)
{
    return encoder->reverse;
}
