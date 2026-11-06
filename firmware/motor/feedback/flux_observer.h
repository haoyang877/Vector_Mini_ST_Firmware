#ifndef FLUX_OBSERVER_H
#define FLUX_OBSERVER_H

#include <stdint.h>
#include "motor_config.h"
#include "coordinate_types.h"

/* 调用方持有并串行访问；首次使用须零初始化，再调用 Init 或 InitConfigured。
 * x/y/eta 保留磁链观测公式命名；位置代际表示参考重建，不是采样节拍。 */
typedef struct
{
    AlphaBetaVector voltage_ab_v;
    AlphaBetaVector current_ab_a;

    float sin, cos;

    float gamma;

    float y1_last, y2_last;
    float etax1, etax2;
    float flux_error_wb2;
    float x1_last, x2_last;
    float x1, x2;
    float electrical_angle_rad, electrical_velocity_rad_s;
    float previous_electrical_angle_rad, previous_electrical_velocity_rad_s;
    float electrical_position_rad;
    int32_t electrical_turns; /* 多圈参考由整圈数和当前单圈角组成，避免长扫圈累计舍入。 */
    uint32_t position_epoch;
} FluxObserverState;

/**
 * @brief  初始化磁链观测器参数。
 * @param  flux_observer 观测器状态指针。
 * @note 指针非空；首次调用前对象须零初始化。可重复调用：清除积分及位置/速度历史，
 *       位置代际按 uint32_t 模加一，gamma 恢复默认值；保留最近采样的电流/电压向量。
 *       调用方保证与 Update/Get 串行，无硬件副作用。
 */
void FluxObserver_Init(FluxObserverState *flux_observer);

/**
 * @brief 上电及标定重启时复位状态并装入同一份型号整定。
 * @param observer 当前观测器。
 * @param configuration 启动装载的活动配置。
 * @note 沿用 Init 的首次/重复调用契约；configuration 非空。
 *       preset 为空时保留默认 gamma，否则从有效的 preset->electrical 装入 gamma。
 */
static inline void FluxObserver_InitConfigured(FluxObserverState *observer,
                                               const MotorConfiguration *configuration)
{
    FluxObserver_Init(observer);
    if (configuration->preset != NULL)
    {
        observer->gamma = configuration->preset->electrical->observer_gamma;
    }
}

/**
 * @brief  执行一次磁链观测器更新。
 * @param model 只读电机模型；使用相电阻、d/q电感和磁链，本次调用期间须保持稳定。
 * @param current_ab_a 本拍只读α/β电流向量，单位A，调用期间有效。
 * @param voltage_ab_v 本拍只读α/β实际电压向量，单位V，已由调用方结合母线电压换算。
 * @param flux_observer 调用方持有的观测器状态；仅修改此对象。
 * @note 在20kHz快速环调用；所有指针非空，状态已初始化并由调用方串行访问。
 *       不保留输入指针，输入不可与输出状态重叠，无硬件副作用。
 *       模型无效、积分/角度非有限或圈数溢出时重建跟踪状态并推进位置代际，保留 gamma；
 *       历史积分非有限时重建后继续本拍计算。最近采样向量以实际更新步骤为准。
 */
void FluxObserver_Update(const MotorModelConfig *model,
                         const AlphaBetaVector *current_ab_a,
                         const AlphaBetaVector *voltage_ab_v,
                         FluxObserverState *flux_observer);

/**
 * @brief  读取观测器电角度。
 * @param  flux_observer 观测器状态指针。
 * @return 电角度，单位 rad。
 */
float FluxObserver_GetElecPhase(FluxObserverState *flux_observer);

/**
 * @brief  读取观测器电角速度。
 * @param  flux_observer 观测器状态指针。
 * @return 电角速度，单位 rad/s。
 */
float FluxObserver_GetElecVel(FluxObserverState *flux_observer);

/**
 * @brief  读取观测器连续电角度（未归一化）。
 * @param  flux_observer 观测器状态指针。
 * @return 连续电角度，单位 rad。
 */
float FluxObserver_GetElecPos(FluxObserverState *flux_observer);

/**
 * @brief  读取观测器位置纪元计数。
 * @param  flux_observer 观测器状态指针。
 * @return 位置参考代际；变化表示参考已重建，不能用于判定是否产生新样本。
 */
uint32_t FluxObserver_GetPositionEpoch(FluxObserverState *flux_observer);

#endif
