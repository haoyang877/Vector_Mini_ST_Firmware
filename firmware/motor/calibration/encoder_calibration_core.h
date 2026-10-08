#ifndef ENCODER_CALIBRATION_CORE_H
#define ENCODER_CALIBRATION_CORE_H

#include <stdbool.h>
#include <stdint.h>

/* 固定环形 Q15 表示；任务边界静态核对反馈层的同一布局，不引入电机运行对象。 */
#define ENCODER_CALIBRATION_Q15_CPR 65536UL
#define ENCODER_CALIBRATION_Q15_HALF_TURN 32768
#define ENCODER_CALIBRATION_LUT_SIZE 1024U
#define ENCODER_CALIBRATION_LUT_BITS 10U

/** 调用方独占的采样区；计数与候选按构表阶段复用，布局保持 6144 B。 */
typedef struct
{
    int32_t error_sum[ENCODER_CALIBRATION_LUT_SIZE];
    union
    {
        uint16_t samples[ENCODER_CALIBRATION_LUT_SIZE];
        int16_t candidate[ENCODER_CALIBRATION_LUT_SIZE];
    } bins;
} EncoderCalibrationWorkspace;

/** 单帧事实：任务已完成跟踪准入；stable 只控制本帧是否计入采样/验证。 */
typedef struct
{
    float observer_position_rad;
    float relative_theta_rad;
    float required_electrical_theta_rad;
    uint16_t directed_q15;
    bool stable;
} EncoderCalibrationSample;

/** 合格正向电气行程，rad；无效帧也更新前次位置，防止把间隙位移计入。 */
typedef struct
{
    float previous_observer_position_rad;
    float valid_electrical_travel_rad;
} EncoderCalibrationWindow;

/** 四遍计算游标；任务持有，只有 BuildLutStep 推进，开始时整体置零。 */
typedef struct
{
    uint16_t candidate_lut_index;
    uint8_t candidate_lut_stage;
    int32_t candidate_lut_previous;
    int32_t candidate_lut_shift;
    int64_t candidate_lut_sum;
} EncoderCalibrationBuildState;

/** 测量归约状态；采样/验证互斥复用窗口，开始验证时整体清零并重设previous。 */
typedef struct
{
    EncoderCalibrationWindow window;
    uint64_t residual_squared_sum;
    uint32_t residual_sample_count;
    uint32_t residual_peak_abs_q15;
} EncoderCalibrationVerifyState;

/** 只表示计算进度或数据不合格；任务决定故障、阶段转换及候选应用。 */
typedef enum
{
    ENCODER_CALIBRATION_BUILD_RUNNING = 0,
    ENCODER_CALIBRATION_BUILD_COMPLETE,
    ENCODER_CALIBRATION_BUILD_INVALID
} EncoderCalibrationBuildResult;

/**
 * @brief 求两个 Q15 环形角的有符号差，使用既有16位模运算语义。
 * @param target_q15 目标环形角，0..65535 counts。
 * @param source_q15 源环形角，0..65535 counts。
 * @return 环形差，int16 counts；半圈返回 -32768。
 * @note 纯计算，无状态和 I/O，可在快速环调用。
 */
int16_t EncoderCalibrationCore_Q15Difference(uint16_t target_q15, uint16_t source_q15);

/**
 * @brief 对候选 LUT 做既有线性插值并减去修正值。
 * @param candidate 只读的 1024 个 int16 Q15 修正值，不允许 NULL。
 * @param directed_q15 方向修正角，0..65535 counts。
 * @return 线性化环形角，0..65535 counts。
 * @note 所有权归调用方；仅访问当前及下一个 bin，不发布候选。
 */
uint16_t EncoderCalibrationCore_ApplyLut(const int16_t *candidate, uint16_t directed_q15);

/**
 * @brief 相对明确锚点累积环形角；原点与电零位窗口共用同一整数计算。
 * @param angle_q15 本帧方向修正角或线性化角，counts。
 * @param anchor_q15 窗口锚点，counts；首样本选择由任务完成，不改写。
 * @param sum_q15 调用方持有的 int64 解卷绕累加值，counts。
 * @param count 调用方持有的样本计数，开始前置零。
 * @note 指针有效且互不重叠，窗口/READ_OK 准入归任务；固定有界窗口内调用。
 */
void EncoderCalibrationCore_AddAngleSample(uint16_t angle_q15,
                                           uint16_t anchor_q15,
                                           int64_t *sum_q15,
                                           uint32_t *count);

/**
 * @brief 求解卷绕累加角的截断均值，不发布原点或电零位。
 * @param sum_q15 解卷绕累加值，counts。
 * @param count 样本数。
 * @param mean_q15 有效输出地址，成功时写入环形 Q15 均值。
 * @return 非空样本返回 true；count 为零返回 false 且不改输出。
 */
bool EncoderCalibrationCore_MeanQ15(int64_t sum_q15, uint32_t count, uint16_t *mean_q15);

/**
 * @brief 按合格帧累积正行程与一个 LUT bin 的环形误差，计数饱和时停止累加。
 * @param workspace 有效且已清零的独占采样区。
 * @param window 调用方持有的采样行程状态。
 * @param sample 有效帧事实；relative/required 为电气 rad，required 须大于零。
 * @note 保持原浮点转换与整数解卷绕顺序；最多访问一个 bin，不决定采样结束。
 */
void EncoderCalibrationCore_SampleLut(EncoderCalibrationWorkspace *workspace,
                                      EncoderCalibrationWindow *window,
                                      const EncoderCalibrationSample *sample);

/**
 * @brief 推进覆盖检查、均值解卷绕、整表平移、候选范围检查四遍计算中的一批。
 * @param workspace 独占采样区；只有第四遍写 union 中的候选表。
 * @param state 调用方持有的游标，开始前置零。
 * @param bins_per_cycle 本拍 bin 上界，1..1024；任务传入既有值 8。
 * @param minimum_samples 每 bin 最小覆盖样本，须大于零；任务传入既有值 16。
 * @return 未完成返回 RUNNING，第四遍结束返回 COMPLETE，覆盖/范围不合格返回 INVALID。
 * @note 不访问控制器、不决定运动、故障或发布；完成/失败也保留原游标位置。
 */
EncoderCalibrationBuildResult
EncoderCalibrationCore_BuildLutStep(EncoderCalibrationWorkspace *workspace,
                                    EncoderCalibrationBuildState *state,
                                    uint16_t bins_per_cycle,
                                    uint16_t minimum_samples);

/**
 * @brief 归约候选验证的合格正行程、残差平方和、峰值与样本数。
 * @param state 调用方持有的验证归约状态。
 * @param candidate 只读的 1024 个候选 Q15 修正值。
 * @param sample 已经跟踪准入的帧事实，单位与 SampleLut 相同。
 * @note 最多验证一帧；不做运动准入、不改活动 LUT。
 */
void EncoderCalibrationCore_VerifyLut(EncoderCalibrationVerifyState *state,
                                      const int16_t *candidate,
                                      const EncoderCalibrationSample *sample);

/**
 * @brief 按原整数平方比较验证覆盖、峰值与 RMS，不调用浮点开方。
 * @param state 只读验证归约状态。
 * @param maximum_rms_q15 RMS 上界，counts。
 * @param maximum_peak_q15 峰值上界，counts。
 * @return 非空且两项均未超过上界返回 true，否则 false。
 * @note 调用方先判定行程结束；本函数不发布、保存或使能输出。
 */
bool EncoderCalibrationCore_ValidateLut(const EncoderCalibrationVerifyState *state,
                                        uint32_t maximum_rms_q15,
                                        uint32_t maximum_peak_q15);

#endif
