#ifndef COGGING_CALIBRATION_H
#define COGGING_CALIBRATION_H
#include <stdbool.h>
#include <stdint.h>

#define COGGING_MAP_POINTS 1024U
#define COGGING_MAP_MAGIC 0x31475143U /* CQG1: signed full-scale Q15, 1024 mechanical positions */
#define COGGING_STEP_RAD (6.283185307179586f / (float)COGGING_MAP_POINTS)
#define COGGING_POSITION_TOL_RAD (6.283185307179586f / 16384.0f) /* 0.0219727 deg */
/* Float subtraction across two turns can round a four-Q15-count error just
 * above its limit. This <0.011-count allowance is numerical, not a new bin. */
#define COGGING_POSITION_GATE_RAD (COGGING_POSITION_TOL_RAD + 0.000001f)
#define COGGING_SETTLE_TICKS 200U /* 100 ms of qualified holding at 2 kHz */
#define COGGING_SAMPLE_TICKS 200U /* 100 ms of accepted block-averaged Iq */

/* Index i is calibrated, directed single-turn encoder angle i * 2*pi/1024.
 * Independent of the user mechanical zero. Positive means positive holding Iq.
 * CRC covers magic, encoder_signature, full_scale_a, then iq_q15 (little-endian target). */
typedef struct {
    uint32_t magic, encoder_signature;
    float full_scale_a;
    uint32_t crc32;
    int16_t iq_q15[COGGING_MAP_POINTS];
} CoggingMapRecord;
extern CoggingMapRecord CoggingMap;
uint32_t Cogging_EncoderSignature(uint8_t reverse, uint16_t electrical_zero,
    int32_t pole_pairs, const int16_t *encoder_lut);

typedef enum {
    COGGING_IDLE = 0, COGGING_SETTLING, COGGING_SAMPLING,
    COGGING_TURNAROUND, COGGING_COMPLETE, COGGING_FAILED, COGGING_SAVING
} CoggingState;
typedef enum {
    COGGING_OK = 0, COGGING_CANCELLED, COGGING_INVALID_INPUT,
    COGGING_POINT_TIMEOUT, COGGING_CURRENT_LIMIT, COGGING_SAFETY_FAULT,
    COGGING_SAVE_FAILED
} CoggingReason;

typedef struct {
    CoggingState state;
    CoggingReason reason;
    uint32_t index, direction_pass, points_done;
    uint32_t point_ticks, stable_ticks, samples, saturation_ticks, total_ticks;
    int32_t target_grid;
    float target_rad, mean_iq_a, full_scale_a, filtered_velocity_rad_s;
    /* ISR-owned statistics include accepted ticks from subsequently retried
     * windows as well; therefore max error bounds every contributed sample. */
    uint32_t accepted_sample_ticks, sample_restarts, last_sample_accepted;
    uint32_t rejected_ticks, window_rejected_ticks, consecutive_rejected_ticks;
    float max_sample_error_rad, sample_error_sq_sum;
    int16_t iq_q15[COGGING_MAP_POINTS];
} CoggingCalibration;

/* Fixed 2 kHz owner. Start at the next absolute mechanical grid point. */
bool CoggingCalibration_Start(CoggingCalibration *c, float position_rad, float full_scale_a);
void CoggingCalibration_Update(CoggingCalibration *c, float position_rad,
    float velocity_rad_s, float iq_a, bool saturated);
void CoggingCalibration_Abort(CoggingCalibration *c, CoggingReason reason);
/* Foreground only, after outputs are off. Removes the constant load component. */
bool CoggingCalibration_Finish(const CoggingCalibration *c, uint32_t signature,
    CoggingMapRecord *record);
uint32_t Cogging_Crc32(const void *data, uint32_t bytes, uint32_t seed);
bool CoggingMap_Valid(const CoggingMapRecord *record, uint32_t signature);
#endif
