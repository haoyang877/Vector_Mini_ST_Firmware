#include "cogging_calibration.h"
#include <math.h>
#include <stddef.h>

#define SETTLE_TICKS COGGING_SETTLE_TICKS
#define SAMPLE_TICKS COGGING_SAMPLE_TICKS
#define POINT_TIMEOUT_TICKS 10000U
#define TOTAL_TIMEOUT_TICKS 3600000U /* 30 min, individual points have a 5 s deadline */
#define SATURATION_TICKS 400U
CoggingMapRecord CoggingMap;

uint32_t Cogging_Crc32(const void *data, uint32_t bytes, uint32_t seed)
{
    const uint8_t *p = (const uint8_t *)data;
    uint32_t crc = ~seed;
    while (bytes--) {
        crc ^= *p++;
        for (unsigned bit = 0; bit < 8; ++bit)
            crc = (crc >> 1) ^ (0xedb88320U & (0U - (crc & 1U)));
    }
    return ~crc;
}

uint32_t Cogging_EncoderSignature(uint8_t reverse, uint16_t electrical_zero,
    int32_t pole_pairs, const int16_t *encoder_lut)
{
    uint32_t identity[2] = {(uint32_t)electrical_zero | ((uint32_t)reverse << 16),
        (uint32_t)pole_pairs};
    uint32_t crc = Cogging_Crc32(identity, sizeof(identity), 0U);
    return Cogging_Crc32(encoder_lut, 1024U * sizeof(int16_t), crc);
}

static uint32_t RecordCrc(const CoggingMapRecord *r)
{
    uint32_t crc = Cogging_Crc32(r, 12U, 0U);
    return Cogging_Crc32(r->iq_q15, sizeof(r->iq_q15), crc);
}

bool CoggingMap_Valid(const CoggingMapRecord *r, uint32_t signature)
{
    return r != NULL && r->magic == COGGING_MAP_MAGIC &&
        r->encoder_signature == signature && isfinite(r->full_scale_a) && r->full_scale_a > 0.0f && r->crc32 == RecordCrc(r);
}

static bool Active(const CoggingCalibration *c)
{
    return c != NULL && (c->state == COGGING_SETTLING ||
        c->state == COGGING_SAMPLING || c->state == COGGING_TURNAROUND);
}

void CoggingCalibration_Abort(CoggingCalibration *c, CoggingReason reason)
{
    if (c != NULL && Active(c)) { c->state = COGGING_FAILED; c->reason = reason; }
}

bool CoggingCalibration_Start(CoggingCalibration *c, float position_rad, float full_scale_a)
{
    int32_t grid;
    if (c == NULL || !isfinite(position_rad) || position_rad < 0.0f ||
        position_rad >= 6.283186f || !isfinite(full_scale_a) || full_scale_a <= 0.0f) return false;
    grid = (int32_t)floorf(position_rad / COGGING_STEP_RAD) + 1;
    c->full_scale_a = full_scale_a;
    c->state = COGGING_SETTLING; c->reason = COGGING_OK;
    c->index = (uint32_t)grid & (COGGING_MAP_POINTS - 1U);
    c->target_grid = grid;
    c->target_rad = (float)grid * COGGING_STEP_RAD;
    c->direction_pass = c->points_done = c->point_ticks = c->stable_ticks = 0U;
    c->samples = c->saturation_ticks = c->total_ticks = 0U;
    c->mean_iq_a = 0.0f;
    c->filtered_velocity_rad_s = 0.0f;
    c->accepted_sample_ticks = c->sample_restarts = c->last_sample_accepted = 0U;
    c->rejected_ticks = c->window_rejected_ticks = c->consecutive_rejected_ticks = 0U;
    c->max_sample_error_rad = c->sample_error_sq_sum = 0.0f;
    /* Every entry is overwritten in the first pass before any result is valid. */
    return true;
}

void CoggingCalibration_Update(CoggingCalibration *c, float position_rad,
    float velocity_rad_s, float iq_a, bool saturated)
{
    bool stable;
    if (!Active(c)) return;
    c->last_sample_accepted = 0U;
    if (!isfinite(position_rad) || !isfinite(velocity_rad_s) || !isfinite(iq_a) ||
        fabsf(iq_a) > c->full_scale_a) {
        CoggingCalibration_Abort(c, COGGING_INVALID_INPUT); return;
    }
    /* First acquisition has no learned holding-current integral yet. */
    if (++c->point_ticks > (c->points_done == 0U ? 20000U : POINT_TIMEOUT_TICKS) ||
        ++c->total_ticks > TOTAL_TIMEOUT_TICKS) {
        CoggingCalibration_Abort(c, COGGING_POINT_TIMEOUT); return;
    }
    c->saturation_ticks = saturated ? c->saturation_ticks + 1U : 0U;
    if (c->saturation_ticks >= SATURATION_TICKS) {
        CoggingCalibration_Abort(c, COGGING_CURRENT_LIMIT); return;
    }
    /* Slow drift check (~5 Hz), separate from fast damping. Every accepted
     * current sample must satisfy the raw position and velocity gates. */
    c->filtered_velocity_rad_s += 0.015f * (velocity_rad_s - c->filtered_velocity_rad_s);
    stable = !saturated && fabsf(c->target_rad - position_rad) <= COGGING_POSITION_GATE_RAD &&
        fabsf(c->filtered_velocity_rad_s) <= 0.01f && fabsf(velocity_rad_s) <= 0.15f;
    if (!stable) {
        ++c->rejected_ticks;
        ++c->window_rejected_ticks;
        ++c->consecutive_rejected_ticks;
        /* Skip isolated encoder noise, never average its Iq. Require >=95%
         * coverage (200 good / at most 210 total ticks) in EACH window. A
         * 2 ms gap, larger excursion or saturation restarts qualification. */
        if (c->window_rejected_ticks > 10U || c->consecutive_rejected_ticks >= 4U ||
            saturated || fabsf(c->target_rad-position_rad) > 2.0f*COGGING_POSITION_TOL_RAD ||
            fabsf(velocity_rad_s) > 0.15f) {
            c->stable_ticks = c->samples = 0U; c->mean_iq_a = 0.0f;
            c->window_rejected_ticks = c->consecutive_rejected_ticks = 0U;
            if (c->state == COGGING_SAMPLING) { c->state = COGGING_SETTLING; ++c->sample_restarts; }
        }
        return;
    }
    c->consecutive_rejected_ticks = 0U;
    if (c->state != COGGING_SAMPLING) {
        if (++c->stable_ticks < SETTLE_TICKS) return;
        c->window_rejected_ticks = 0U;
        if (c->state == COGGING_TURNAROUND) {
            /* Approach the last forward point from the opposite side. */
            c->target_rad = (float)(--c->target_grid) * COGGING_STEP_RAD;
            c->point_ticks = c->stable_ticks = 0U;
            c->state = COGGING_SETTLING;
        } else c->state = COGGING_SAMPLING;
        return;
    }
    c->last_sample_accepted = 1U;
    ++c->accepted_sample_ticks;
    {
        float error = c->target_rad - position_rad;
        c->max_sample_error_rad = fmaxf(c->max_sample_error_rad, fabsf(error));
        c->sample_error_sq_sum += error * error;
    }
    c->mean_iq_a += (iq_a - c->mean_iq_a) / (float)(++c->samples);
    if (c->samples < SAMPLE_TICKS) return;
    {
        float counts = c->mean_iq_a * (32768.0f / c->full_scale_a);
        if (c->direction_pass != 0U) counts = (counts + (float)c->iq_q15[c->index]) * 0.5f;
        c->iq_q15[c->index] = (int16_t)fmaxf(-32768.0f, fminf(32767.0f, roundf(counts)));
    }
    ++c->points_done;
    c->point_ticks = c->stable_ticks = c->samples = 0U; c->mean_iq_a = 0.0f;
    c->window_rejected_ticks = c->consecutive_rejected_ticks = 0U;
    if (c->points_done == 2U * COGGING_MAP_POINTS) { c->state = COGGING_COMPLETE; return; }
    if (c->points_done == COGGING_MAP_POINTS) {
        c->direction_pass = 1U;
        c->target_rad = (float)(++c->target_grid) * COGGING_STEP_RAD;
        c->state = COGGING_TURNAROUND;
        return;
    }
    if (c->direction_pass == 0U) {
        c->index = (c->index + 1U) & (COGGING_MAP_POINTS - 1U);
        c->target_rad = (float)(++c->target_grid) * COGGING_STEP_RAD;
    } else {
        c->index = (c->index - 1U) & (COGGING_MAP_POINTS - 1U);
        c->target_rad = (float)(--c->target_grid) * COGGING_STEP_RAD;
    }
    c->state = COGGING_SETTLING;
}

bool CoggingCalibration_Finish(const CoggingCalibration *c, uint32_t signature, CoggingMapRecord *r)
{
    int32_t sum = 0;
    float mean;
    if (c == NULL || r == NULL || c->state != COGGING_COMPLETE ||
        c->points_done != 2U * COGGING_MAP_POINTS) return false;
    for (unsigned i = 0; i < COGGING_MAP_POINTS; ++i) sum += c->iq_q15[i];
    mean = (float)sum / (float)COGGING_MAP_POINTS;
    /* Validate every converted value before replacing an existing record. */
    for (unsigned i = 0; i < COGGING_MAP_POINTS; ++i)
        if (fabsf((float)c->iq_q15[i] - mean) > 32767.0f) return false;
    r->magic = 0U;
    r->encoder_signature = signature;
    r->full_scale_a = c->full_scale_a;
    for (unsigned i = 0; i < COGGING_MAP_POINTS; ++i)
        r->iq_q15[i] = (int16_t)roundf((float)c->iq_q15[i] - mean);
    r->magic = COGGING_MAP_MAGIC;
    r->crc32 = RecordCrc(r);
    return true;
}
