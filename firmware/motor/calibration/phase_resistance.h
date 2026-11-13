#ifndef PHASE_RESISTANCE_H
#define PHASE_RESISTANCE_H
#include <stdbool.h>
#include <stdint.h>

/** @brief 相电阻终态快照；电阻均为Ω，相序ABC，线序AB/BC/CA。 */
typedef struct
{
    float nominal_ohm, phase_ohm[3], line_ohm[3];
    uint8_t result, open_phases, low_phases, high_phases, open_lines, failure, valid;
} PhaseResistanceReport;

/** @brief 调用方独占的双极性两档电流测量工作区。 */
typedef struct
{
    PhaseResistanceReport report;
    float points_v[2], points_a[2], positive_ohm;
    /* 留下各回路两档均值，便于停机后核对真实激励；工作区与LUT互斥。 */
    float measured_a[6][2], measured_v[6][2];
    float voltage_v, integral_v, target_a, sum_v, sum_a;
    float nominal_inductance_h, last_turns, window_position, window_min, window_max;
    float peak_current_a;
    uint32_t ticks, samples, stable_ticks, window_ticks, zero_ticks;
    uint8_t excitation, level;
    bool gap, done, window_current_stable;

} PhaseResistance;

/**
 * @brief 初始化一次关相后启动的相电阻检查。
 * @param state 调用方独占工作区。
 * @param nominal_ohm 有限正标称相电阻Ω。
 * @param inductance_h 有限正相电感H。
 * @param turns 本次原始机械角，单位圈。
 */
void PhaseResistance_Init(PhaseResistance *state,
                          float nominal_ohm,
                          float inductance_h,
                          float turns);
/**
 * @brief 推进一个有界快拍，输出下一拍线电压和相使能掩码。
 * @param state 已初始化工作区；终态结果只读。
 * @param current_a 三相实测电流A。
 * @param bus_v 母线电压V。
 * @param turns 当前机械角，单位圈。
 * @param period_s 快拍周期秒。
 * @param duty 输出ABC占空比。
 * @return AB/BC/CA相使能掩码；仅在零电流间隙改变。
 * @note 调用方持有功率许可，负责终态及STOP的实际关断。
 */
uint8_t PhaseResistance_Step(PhaseResistance *state,
                             const float current_a[3],
                             float bus_v,
                             float turns,
                             float period_s,
                             float duty[3]);
#endif
