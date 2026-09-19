#ifndef MCU_TEMPERATURE_H
#define MCU_TEMPERATURE_H

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

/* MCU 芯片结温传感契约（platform/api 层拥有，motor 层监督任务直接包含）。
 * 换算使用每颗芯片出厂的 TS_CAL1（30°C）、TS_CAL2（130°C，3.0 V 标定）与
 * VREFINT_CAL，并用实测 VREFINT 修正 VDDA；结果是 MCU die 温度，不代表绕组、
 * MOSFET 或环境温度。本仓库旧 LL 头文件把 TS_CAL2 标成 110°C，禁止使用其
 * 温度换算宏。90°C 跳闸与 100 ms 失效窗口是保守软件默认值，未经实物热箱
 * 验证；它们的版本化参数化归后续故障保护设计所有。 */

#define MCU_TEMPERATURE_TRIP_C 90.0f
#define MCU_TEMPERATURE_TIMEOUT_MS 100U

/**
 * @brief 将内部温度传感器与 VREFINT 的原始 ADC 计数换算为芯片结温与 VDDA。
 * @param raw_ts 内部温度传感器通道的 12 位原始计数；0 与 4095 视为无效。
 * @param raw_vref VREFINT 通道的 12 位原始计数；0 与 4095 视为无效。
 * @param cal30 出厂 TS_CAL1 计数，对应 30°C、VDDA=3.0 V。
 * @param cal130 出厂 TS_CAL2 计数，对应 130°C、VDDA=3.0 V。
 * @param vref_cal 出厂 VREFINT_CAL 计数，对应 VDDA=3.0 V。
 * @param celsius 输出芯片结温，单位 °C；仅当返回 true 时有效。
 * @param vdda_mv 输出估算 VDDA，单位 mV；仅当返回 true 时有效。
 * @return 换算成功返回 true；任一输入计数无效、VDDA 超出 1.62~3.6 V 或温度
 *         超出 -40~150°C 时返回 false。
 * @note 纯计算：不访问寄存器、不阻塞，可在监督任务上下文调用；返回 false 时
 *       输出参数不保证有效，调用方必须丢弃。
 */
static inline bool McuTemperature_Convert(uint32_t raw_ts,
                                          uint32_t raw_vref,
                                          uint16_t cal30,
                                          uint16_t cal130,
                                          uint16_t vref_cal,
                                          float *celsius,
                                          float *vdda_mv)
{
    float normalized;

    if (raw_ts == 0U || raw_ts >= 4095U || raw_vref == 0U || raw_vref >= 4095U || cal30 == 0U ||
        cal130 >= 4095U || cal130 <= cal30 || vref_cal == 0U || vref_cal >= 4095U)
    {
        return false;
    }
    *vdda_mv = 3000.0f * (float)vref_cal / (float)raw_vref;
    if (*vdda_mv < 1620.0f || *vdda_mv > 3600.0f)
    {
        return false;
    }
    normalized = (float)raw_ts * (float)vref_cal / (float)raw_vref;
    *celsius = 30.0f + (normalized - (float)cal30) * 100.0f / (float)(cal130 - cal30);
    return isfinite(*celsius) && *celsius >= -40.0f && *celsius <= 150.0f;
}

#endif
