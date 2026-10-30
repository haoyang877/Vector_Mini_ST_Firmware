#include "flux_observer.h"

#include <float.h>
#include <string.h>

#include "application_config.h"
#include "control_math.h"

#define FLUX_OBSERVER_DEFAULT_GAMMA 800000.0f
#define FLUX_OBSERVER_MAX_CORRECTION_STEP 0.2f
#define FLUX_OBSERVER_MIN_FLUX 1e-6f

#if FLT_RADIX != 2 || FLT_MANT_DIG != 24 || FLT_MAX_EXP != 128
#error "Flux observer requires IEEE 754 binary32 float"
#endif
typedef char FluxObserverFloatMustBe4Bytes[(sizeof(float) == 4U) ? 1 : -1];

static int fluxobserver_is_finite(float value)
{
    uint32_t bits;
    /* binary32指数全1仅表示NaN/无穷；固定4字节复制避免别名违规与浮点分支。 */
    memcpy(&bits, &value, sizeof(bits));
    return (bits & 0x7F800000U) != 0x7F800000U;
}

static void fluxobserver_reset_state(FluxObserverState *flux_observer)
{
    flux_observer->sin = 0.0f;
    flux_observer->cos = 1.0f;
    flux_observer->y1_last = 0.0f;
    flux_observer->y2_last = 0.0f;
    flux_observer->etax1 = 0.0f;
    flux_observer->etax2 = 0.0f;
    flux_observer->flux_error_wb2 = 0.0f;
    flux_observer->x1_last = 0.0f;
    flux_observer->x2_last = 0.0f;
    flux_observer->x1 = 0.0f;
    flux_observer->x2 = 0.0f;
    flux_observer->electrical_angle_rad = 0.0f;
    flux_observer->electrical_velocity_rad_s = 0.0f;
    flux_observer->previous_electrical_angle_rad = 0.0f;
    flux_observer->previous_electrical_velocity_rad_s = 0.0f;
    flux_observer->electrical_position_rad = 0.0f;
    flux_observer->position_epoch++;
}

/**
 * @brief  初始化磁链观测器参数。
 * @param  flux_observer 观测器状态指针。
 */
void FluxObserver_Init(FluxObserverState *flux_observer)
{
    fluxobserver_reset_state(flux_observer);
    flux_observer->gamma = FLUX_OBSERVER_DEFAULT_GAMMA;
}

/**
 * @brief  执行一次磁链观测器更新。
 * @param model 本次调用期间保持稳定的只读电机模型。
 * @param current_alpha_a 本拍α轴电流，单位A。
 * @param current_beta_a 本拍β轴电流，单位A。
 * @param voltage_alpha_v 本拍对应的α轴实际电压，单位V。
 * @param voltage_beta_v 本拍对应的β轴实际电压，单位V。
 * @param flux_observer 调用方持有的观测器状态。
 */
void FluxObserver_Update(const MotorModelConfig *model,
                         float current_alpha_a,
                         float current_beta_a,
                         float voltage_alpha_v,
                         float voltage_beta_v,
                         FluxObserverState *flux_observer)
{
    float Rs = model->motor_phase_resistance_ohm;
    float Ls = (model->motor_d_inductance_h + model->motor_q_inductance_h) * 0.5f;
    float flux = model->motor_flux_wb;
    float flux_sq;
    float gamma_limit;
    float gamma;
    float delta_theta = 0.0f;

    if (!fluxobserver_is_finite(Rs) || !fluxobserver_is_finite(Ls) ||
        !fluxobserver_is_finite(flux) || flux <= FLUX_OBSERVER_MIN_FLUX)
    {
        fluxobserver_reset_state(flux_observer);
        return;
    }

    if (!fluxobserver_is_finite(flux_observer->x1_last) ||
        !fluxobserver_is_finite(flux_observer->x2_last))
    {
        fluxobserver_reset_state(flux_observer);
    }

    flux_sq = flux * flux;
    gamma_limit = FLUX_OBSERVER_MAX_CORRECTION_STEP / (FOC_TS_S * flux_sq);
    gamma = flux_observer->gamma < gamma_limit ? flux_observer->gamma : gamma_limit;

    flux_observer->current_alpha_a = current_alpha_a;
    flux_observer->current_beta_a = current_beta_a;
    flux_observer->voltage_alpha_v = voltage_alpha_v;
    flux_observer->voltage_beta_v = voltage_beta_v;

    /*flux observer*/
    flux_observer->y1_last = -Rs * flux_observer->current_alpha_a + flux_observer->voltage_alpha_v;
    flux_observer->y2_last = -Rs * flux_observer->current_beta_a + flux_observer->voltage_beta_v;

    flux_observer->etax1 = flux_observer->x1_last - Ls * flux_observer->current_alpha_a;
    flux_observer->etax2 = flux_observer->x2_last - Ls * flux_observer->current_beta_a;

    flux_observer->flux_error_wb2 = flux_sq - (flux_observer->etax1 * flux_observer->etax1 +
                                               flux_observer->etax2 * flux_observer->etax2);

    flux_observer->x1 = FOC_TS_S * (flux_observer->y1_last +
                                    gamma * flux_observer->etax1 * flux_observer->flux_error_wb2) +
                        flux_observer->x1_last;
    flux_observer->x2 = FOC_TS_S * (flux_observer->y2_last +
                                    gamma * flux_observer->etax2 * flux_observer->flux_error_wb2) +
                        flux_observer->x2_last;

    if (!fluxobserver_is_finite(flux_observer->x1) || !fluxobserver_is_finite(flux_observer->x2))
    {
        fluxobserver_reset_state(flux_observer);
        return;
    }

    /*iteration*/
    flux_observer->x1_last = flux_observer->x1;
    flux_observer->x2_last = flux_observer->x2;

    flux_observer->cos = (flux_observer->x1 - Ls * flux_observer->current_alpha_a) / flux;
    flux_observer->sin = (flux_observer->x2 - Ls * flux_observer->current_beta_a) / flux;

    /*calculate angle with atan*/
    flux_observer->electrical_angle_rad = Math_Atan2(flux_observer->sin, flux_observer->cos);
    if (!fluxobserver_is_finite(flux_observer->electrical_angle_rad))
    {
        fluxobserver_reset_state(flux_observer);
        return;
    }

    /* atan2输出在单圈内，只补负半圈，避免通用多圈归一化。 */
    if (flux_observer->electrical_angle_rad < 0.0f)
    {
        flux_observer->electrical_angle_rad += _2PI;
    }

    delta_theta =
        flux_observer->electrical_angle_rad - flux_observer->previous_electrical_angle_rad;

    flux_observer->previous_electrical_angle_rad = flux_observer->electrical_angle_rad;

    if (delta_theta < -4.0f)
    {
        delta_theta += _2PI;
    }
    else if (delta_theta > 4.0f)
    {
        delta_theta -= _2PI;
    }

    flux_observer->electrical_position_rad += delta_theta;
    UTILS_LP_FAST(flux_observer->electrical_velocity_rad_s, delta_theta / FOC_TS_S, 0.1f);
}

/**
 * @brief  读取观测器电角度。
 * @param  flux_observer 观测器状态指针。
 * @return 电角度，单位 rad。
 */
float FluxObserver_GetElecPhase(FluxObserverState *flux_observer)
{
    return flux_observer->electrical_angle_rad;
}

/**
 * @brief  读取观测器电角速度。
 * @param  flux_observer 观测器状态指针。
 * @return 电角速度，单位 rad/s。
 */
float FluxObserver_GetElecVel(FluxObserverState *flux_observer)
{
    return flux_observer->electrical_velocity_rad_s;
}

float FluxObserver_GetElecPos(FluxObserverState *flux_observer)
{
    return flux_observer->electrical_position_rad;
}

uint32_t FluxObserver_GetPositionEpoch(FluxObserverState *flux_observer)
{
    return flux_observer->position_epoch;
}
