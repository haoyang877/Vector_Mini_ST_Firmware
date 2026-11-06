#ifndef SENSORLESS_STARTUP_H
#define SENSORLESS_STARTUP_H

#include "motor_config.h"
#include "foc.h"
#include "flux_observer.h"

typedef enum
{
    SENSORLESS_STARTUP_IDLE = 0,
    SENSORLESS_STARTUP_ALIGN,
    SENSORLESS_STARTUP_OPEN_LOOP,
    SENSORLESS_STARTUP_SPEED_LOCK,
    SENSORLESS_STARTUP_HANDOFF,
    SENSORLESS_STARTUP_CLOSED_LOOP
} SensorlessStartupPhase;

/* 调用方持有并串行访问；Reset 建立完整初始状态，不能用全零状态替代。
 * 本模块维护启动阶段、交接参考及计时；不拥有 FOC、PI 或观测器对象。 */
typedef struct
{
    SensorlessStartupPhase state;
    float open_loop_angle_rad;
    float open_loop_velocity_rad_s;
    float handoff_phase_delta_rad;
    float handoff_id_reference_a;
    float speed_feedback_rad_s;
    float lock_speed_feedback_rad_s;
    float direction;
    float speed_pi_output_max;
    uint32_t state_ticks;
    uint32_t speed_loop_ticks;
    uint32_t open_loop_ticks;
    uint32_t lock_ticks;
    uint32_t id_ramp_ticks;
    uint32_t loss_ticks;
} SensorlessStartupState;

/* 无感运行应用：启动序列与速度模式；观测器算法见 flux_observer.{c,h}。 */

/**
 * @brief  复位无感启动状态机。
 * @param  startup 无感启动状态指针。
 * @note startup 非空，可用于首次初始化或重复复位；回到 IDLE，方向与速度 PI 上限为 1。
 *       只改写 startup，不复位 FOC/PI/观测器，不改变电机目标或硬件输出。
 */
void SensorlessStartup_Reset(SensorlessStartupState *startup);

/**
 * @brief  执行无感启动状态机的单 tick 步骤。
 * @param configuration 活动配置只读视图；本次调用期间须保持稳定。
 * @param  foc foc 状态指针。
 * @param  motor_state 电机控制状态指针。
 * @param  controller 标定会话独占的速度 PI，用于交接、扫圈和减速。
 * @param  flux_observer 磁链观测器状态指针。
 * @param  startup 无感启动状态机指针。
 * @param  config 启动参数集，编码器标定配置。
 * @note 在 20kHz 快速环上下文调用；故障经 MotorFault_Report 锁存，由上层处理关相。
 *       检出故障即返回，不撤销本拍已执行的 FOC 输出。
 *       除 config 可为空并按参数故障处理外，指针均非空；各对象由调用方独占串行访问。
 *       startup 首次使用前须 Reset，观测器首次须零初始化；其他控制对象按各自契约准备。
 *       IDLE 启动时调用观测器 InitConfigured，并复位电流 PI 与速度 PI；
 *       正常状态推进可能调用 Foc_RunCurrent 施加输出，不阻塞。
 */
void SensorlessStartup_Run(const MotorConfiguration *configuration,
                           FocState *foc,
                           MotorControlState *motor_state,
                           PiController *controller,
                           FluxObserverState *flux_observer,
                           SensorlessStartupState *startup,
                           const SensorlessStartupConfig *config);

/** 编码器标定（Mode 13）观测器启动参数集；由 encoder_calibration 模块消费。 */
extern const SensorlessStartupConfig SensorlessStartup_EncoderCalibConfig;

#endif
