#include "foc_sensorless.h"

#include <float.h>
#include <math.h>

#include "control_config.h"
#include "utils.h"

#define FLUX_OBSERVER_DEFAULT_GAMMA 800000.0f
#define FLUX_OBSERVER_MAX_CORRECTION_STEP 0.2f
#define FLUX_OBSERVER_MIN_FLUX 1e-6f

static int fluxobserver_is_finite(float value)
{
    return value == value && value <= FLT_MAX && value >= -FLT_MAX;
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
void FluxObserver_ParamInit(FluxObserverState *flux_observer)
{
    fluxobserver_reset_state(flux_observer);
    flux_observer->gamma = FLUX_OBSERVER_DEFAULT_GAMMA;
}

/**
 * @brief  执行一次磁链观测器更新。
 * @param  foc foc 状态指针。
 * @param  motor_state 电机控制状态指针。
 * @param  flux_observer 观测器状态指针。
 */
void FluxObserver_Update(FocState *foc,
                         MotorControlState *motor_state,
                         FluxObserverState *flux_observer)
{
    float mod_to_V = foc->bus_voltage_filtered_v / 1.5f;
    float Rs = motor_state->motor_phase_resistance_ohm;
    float Ls = (motor_state->motor_d_inductance_h + motor_state->motor_q_inductance_h) * 0.5f;
    float flux = motor_state->motor_flux_wb;
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
        fluxobserver_reset_state(flux_observer);

    flux_sq = Math_Square(flux);
    gamma_limit = FLUX_OBSERVER_MAX_CORRECTION_STEP / (Current_Ts * flux_sq);
    gamma = Math_Min(flux_observer->gamma, gamma_limit);

    /*Use phase currents sampled in the current ADC interrupt.*/
    Clarke_Transform(foc->phase_a_current_a,
                     foc->phase_b_current_a,
                     foc->phase_c_current_a,
                     &foc->current_alpha_a,
                     &foc->current_beta_a);

    /*update input parameters*/
    flux_observer->current_alpha_a = foc->current_alpha_a;
    flux_observer->current_beta_a = foc->current_beta_a;
    flux_observer->voltage_alpha_v = foc->mod_alpha * mod_to_V;
    flux_observer->voltage_beta_v = foc->mod_beta * mod_to_V;

    /*flux observer*/
    flux_observer->y1_last = -Rs * flux_observer->current_alpha_a + flux_observer->voltage_alpha_v;
    flux_observer->y2_last = -Rs * flux_observer->current_beta_a + flux_observer->voltage_beta_v;

    flux_observer->etax1 = flux_observer->x1_last - Ls * flux_observer->current_alpha_a;
    flux_observer->etax2 = flux_observer->x2_last - Ls * flux_observer->current_beta_a;

    flux_observer->flux_error_wb2 =
        flux_sq - (Math_Square(flux_observer->etax1) + Math_Square(flux_observer->etax2));

    flux_observer->x1 = Current_Ts * (flux_observer->y1_last + gamma * flux_observer->etax1 *
                                                                   flux_observer->flux_error_wb2) +
                        flux_observer->x1_last;
    flux_observer->x2 = Current_Ts * (flux_observer->y2_last + gamma * flux_observer->etax2 *
                                                                   flux_observer->flux_error_wb2) +
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
    flux_observer->electrical_angle_rad =
        Math_NormalizeAngle(Math_Atan2(flux_observer->sin, flux_observer->cos));
    if (!fluxobserver_is_finite(flux_observer->electrical_angle_rad))
    {
        fluxobserver_reset_state(flux_observer);
        return;
    }

    delta_theta =
        flux_observer->electrical_angle_rad - flux_observer->previous_electrical_angle_rad;

    flux_observer->previous_electrical_angle_rad = flux_observer->electrical_angle_rad;

    if (delta_theta < -4.0f)
        delta_theta += _2PI;
    else if (delta_theta > 4.0f)
        delta_theta -= _2PI;

    flux_observer->electrical_position_rad += delta_theta;
    UTILS_LP_FAST(flux_observer->electrical_velocity_rad_s, delta_theta / Current_Ts, 0.1f);
}

/**
 * @brief  读取观测器电角度。
 * @param  flux_observer 观测器状态指针。
 * @return 电角度，单位 rad。
 */
float Observer_GetElePhase(FluxObserverState *flux_observer)
{
    return flux_observer->electrical_angle_rad;
}

/**
 * @brief  读取观测器电角速度。
 * @param  flux_observer 观测器状态指针。
 * @return 电角速度，单位 rad/s。
 */
float Observer_GetEleVel(FluxObserverState *flux_observer)
{
    return flux_observer->electrical_velocity_rad_s;
}

float Observer_GetElePosition(FluxObserverState *flux_observer)
{
    return flux_observer->electrical_position_rad;
}

uint32_t Observer_GetPositionEpoch(FluxObserverState *flux_observer)
{
    return flux_observer->position_epoch;
}
