#ifndef MCU_TEMPERATURE_H
#define MCU_TEMPERATURE_H

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

/* STM32G431 DS12589 table 5: 30/130 C at 3.0 V. The bundled LL header
 * incorrectly specifies 110 C; do not use its temperature conversion macro.
 * These are die temperatures, never motor winding or MOSFET measurements. */
#define MCU_TEMPERATURE_TRIP_C 90.0f
#define MCU_TEMPERATURE_TIMEOUT_MS 100U

static inline bool McuTemperature_Convert(uint32_t raw_ts, uint32_t raw_vref,
    uint16_t cal30, uint16_t cal130, uint16_t vref_cal, float *celsius, float *vdda_mv)
{
    float normalized;
    if (raw_ts == 0U || raw_ts >= 4095U || raw_vref == 0U || raw_vref >= 4095U ||
        cal30 == 0U || cal130 >= 4095U || cal130 <= cal30 ||
        vref_cal == 0U || vref_cal >= 4095U) return false;
    *vdda_mv = 3000.0f * (float)vref_cal / (float)raw_vref;
    if (*vdda_mv < 1620.0f || *vdda_mv > 3600.0f) return false;
    normalized = (float)raw_ts * (float)vref_cal / (float)raw_vref;
    *celsius = 30.0f + (normalized - (float)cal30) * 100.0f / (float)(cal130 - cal30);
    return isfinite(*celsius) && *celsius >= -40.0f && *celsius <= 150.0f;
}
#endif
