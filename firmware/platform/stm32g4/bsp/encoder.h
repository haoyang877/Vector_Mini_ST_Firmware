#ifndef __ENCODER_H__
#define __ENCODER_H__

#include <stdbool.h>
#include <stdint.h>
#include "main.h"
#include "data_type.h"

/* TLE5012B single-turn angle representation: unsigned Q15, [0, 65535]. */
#define ENCODER_Q15_CPR                 65536UL
#define ENCODER_Q15_HALF_TURN           32768
#define ENCODER_OFFSET_LUT_SIZE          1024U
#define ENCODER_OFFSET_LUT_BITS          10U
#define ENCODER_BAD_FRAME_OFFLINE_COUNT  100U

/* 机械速度估计：20 kHz 快路径二阶角度跟踪观测器（PLL），取代原 2 kHz 16 点滑窗割线。
 * 原滑窗群延时约 4.0 ms；本观测器 600 Hz 约 0.38 ms、300 Hz 约 0.75 ms、90 Hz 约 2.5 ms。
 * 2026-09-24 台架实测（2 kHz 真值捕获法，Ki=10*Kp、6 A 限流、3 rad/s，判据=极限环）：
 *   ωn=90 Hz  → 增益上限 **0.06~0.07**，失稳频率 **~79 Hz**（≈ωn，PLL 自身成瓶颈），
 *               且同增益纹波反而更高（Kp=0.06 时 std 0.321 vs 600 Hz 的 0.246）→ 降 ωn 不省噪声。
 *   ωn=600 Hz → 增益上限 **0.11**，失稳频率 ~250 Hz。
 *   → 限制速度环带宽/增益上限的是**反馈路径相位滞后（PLL 二阶滞后为主），不是噪声**；
 *     降低 ωn 等于把整个速度环的带宽上限压到接近 ωn（90 Hz → 上限只有 ~80 Hz），故不可取。
 * 故按速调度：低速高 ωn 换增益余量，高速低 ωn 滤掉随速增长的与角度相关的周期误差
 * （10–20 rad/s 段纹波：600 Hz 固定 4–6%，300 Hz 固定 1.4–4.3%）。 */
#define ENCODER_PLL_OMEGA_N_LOW_SPEED_HZ     600.0f
#define ENCODER_PLL_OMEGA_N_HIGH_SPEED_HZ    300.0f
#define ENCODER_PLL_SCHED_LOW_RAD_S            5.0f
#define ENCODER_PLL_SCHED_HIGH_RAD_S          10.0f
#define ENCODER_PLL_ZETA                     0.707f
#define ENCODER_PLL_OMEGA_MAX_RAD_S          2000.0f
#define ENCODER_PLL_VEL_ZERO_THRESHOLD_RAD_S 0.05f

/* 测速来源选择（台架 A/B 对照用，两者同时编译，本宏只决定谁写速度输出）：
 *   0 = PLL 观测器（默认）
 *   1 = 4 抽头/2 ms 短窗（旧短窗方案 A2：由逐拍 0.5 ms 位移增量求和，
 *       群延时约 0.75 ms、无模型、对量化噪声抑制弱）
 * 说明：A2 与 PLL 共用同一 physical 角度链（linearized_q15），只换估计器。 */
#define ENCODER_VELOCITY_SOURCE_PLL          0U
#define ENCODER_VELOCITY_SOURCE_WINDOW4      1U
#ifndef ENCODER_VELOCITY_SOURCE
#define ENCODER_VELOCITY_SOURCE              ENCODER_VELOCITY_SOURCE_PLL
#endif
/* 窗口抽头数：40 × 20 kHz 快拍 = 2 ms，等价原"4 抽头 × 500 µs"短窗。
 * （4 抽头配合 Current_Ts 会退化成 200 µs 窗口，噪声大 10 倍，不是原方案。） */
#define ENCODER_VELOCITY_WINDOW4_TAPS             40U
/* 旧短窗口径的零点门限：8 计数（vel_mech）/ 1 计数（vel_mech_fast）。 */
#define ENCODER_VELOCITY_WINDOW4_ZERO_THRESHOLD_Q15        8
#define ENCODER_VELOCITY_WINDOW4_FAST_ZERO_THRESHOLD_Q15   1

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

/* Calibration flags stored in flash. Mechanical zero is optional feedback state. */
#define ENC_CALIB_LINEARIZED        (1U << 0)
#define ENC_CALIB_ELECTRICAL_ZERO   (1U << 1)
#define ENC_CALIB_MECHANICAL_ZERO   (1U << 2)
#define ENC_CALIB_ZERO_POS          ENC_CALIB_ELECTRICAL_ZERO
#define ENC_CALIB_ALL               (ENC_CALIB_LINEARIZED | ENC_CALIB_ELECTRICAL_ZERO)

typedef struct
{
	/* Persisted direction configuration and calibration data. */
	uint16_t electrical_zero_q15;
	uint16_t mechanical_zero_q15;
	int16_t linearization_lut_q15[ENCODER_OFFSET_LUT_SIZE];
	uint8_t calib_flag;
	uint8_t reverse;

	/* Raw TLE5012B reading, direction-corrected input, and LUT-corrected angle. */
	uint16_t raw_q15;
	uint16_t directed_q15;
	uint16_t linearized_q15;
	uint16_t previous_linearized_q15;
	int64_t shadow_q15;
	int64_t mechanical_zero_shadow_q15;
	bool has_valid_sample;

	/* Mechanical and electrical feedback exposed to the FOC/control interfaces. */
	float theta_elec;
	float vel_elec;
	float theta_mech;
	float vel_mech;
	float vel_mech_continuous;
	/* 速度环反馈速度：PLL 已是低延时源，与 vel_mech_continuous 同值（保留字段供既有消费者）。 */
	float vel_mech_fast;

	/* 20 kHz 快路径角度跟踪观测器（PLL）状态；速度状态由快路径独占写。 */
	float pll_theta_rad;
	float pll_omega_rad_s;
	bool velocity_ready;
	/* 快拍分频计数：Encoder_DidUpdateVelocity() 用它报告 2 kHz 慢拍边界
	 * （RTT 等调用方据此避免与慢拍同拍堆叠），与速度估计本身无关。 */
	uint8_t velocity_divider;

	/* A/B 用：4 抽头/2 ms 短窗测速（方案 A2）状态；仅当 ENCODER_VELOCITY_SOURCE
	 * 选中 WINDOW4 时参与速度输出，PLL 状态始终维护。 */
	uint8_t velocity_win4_index;
	uint8_t velocity_win4_sample_count;
	bool velocity_win4_ready;
	int32_t velocity_win4_delta_history[ENCODER_VELOCITY_WINDOW4_TAPS];
	int32_t velocity_win4_delta_sum;

	/* TLE5012B SSC frame diagnostics and online state. */
	Encoder_ReadStatus read_status;
	Encoder_ReadStatus read_status_latched;
	uint16_t tle5012_angle_word;
	uint16_t tle5012_safety_word;
	uint8_t tle5012_crc_received;
	uint8_t tle5012_crc_calculated;
	uint32_t tle5012_crc_error_count;
	uint32_t read_error_count;
	uint16_t bad_frame_streak;
} Encoder_TypeDef;

#define brd_enc_spi  hspi2
#define BRD_ENC_SPI  SPI2

#define BRD_ENC_CS_ENABLE  BRD_ENC_CS_GPIO_Port->BSRR = (uint32_t)BRD_ENC_CS_Pin << 16U
#define BRD_ENC_CS_DISABLE BRD_ENC_CS_GPIO_Port->BSRR = BRD_ENC_CS_Pin

void Encoder_ParamInit(Encoder_TypeDef *Encoder);
void Encoder_Update(MotorControl_TypeDef *MotorControl, Encoder_TypeDef *Encoder);
/** Nonblocking request for the next angle sample. The same fast-loop owner
 * must call CompleteSample exactly once with the returned token, even on a
 * protection fault. Do not access the sensor bus between these calls. */
bool Encoder_BeginSample(void);
/** Complete this cycle's request and update the same estimator as Update.
 * A false token uses the original synchronous read/recovery path. */
void Encoder_CompleteSample(MotorControl_TypeDef *MotorControl, Encoder_TypeDef *Encoder,
    bool sample_started);

bool Encoder_IsOnline(const Encoder_TypeDef *Encoder);
bool Encoder_SetElectricalZero(Encoder_TypeDef *Encoder);
bool Encoder_SetElectricalZeroQ15(Encoder_TypeDef *Encoder, uint16_t electrical_zero_q15);
bool Encoder_SetMechanicalZero(Encoder_TypeDef *Encoder);
void Encoder_SetReverse(Encoder_TypeDef *Encoder, bool reverse);
/** 重建速度观测器（停机、模式切换、方向变更或首帧后调用）。观测器在下一快拍以当前角
 *  自初始化，不产生速度冲击；可从任意中断上下文调用。 */
void Encoder_ResetVelocity(Encoder_TypeDef *Encoder);

/** 电角速度 rad/s，等于 vel_mech × 极对数（含软零速死区）。 */
float Encoder_GetEleVel(const Encoder_TypeDef *Encoder);
/** 机械角速度 rad/s；观测器未就绪或低于软零速死区时为 0。 */
float Encoder_GetMecVel(const Encoder_TypeDef *Encoder);
/** 机械角速度 rad/s，供速度环反馈；PLL 下与 Encoder_GetMecVelContinuous() 同值，
 *  保留接口以兼容既有短窗调用点。 */
float Encoder_GetMecVelFast(const Encoder_TypeDef *Encoder);
/** 机械角速度 rad/s，不含零速死区；观测器未就绪时为 0。 */
float Encoder_GetMecVelContinuous(const Encoder_TypeDef *Encoder);
/** 本快拍是否落在 2 kHz 慢拍边界（无坏帧且观测器已就绪）；可选遥测据此避免与慢拍堆叠。 */
bool Encoder_DidUpdateVelocity(const Encoder_TypeDef *Encoder);
float Encoder_GetElePhase(const Encoder_TypeDef *Encoder);
float Encoder_GetMecPos(const Encoder_TypeDef *Encoder);
float Encoder_GetCountInCPR_Ratio(const Encoder_TypeDef *Encoder);

#endif
