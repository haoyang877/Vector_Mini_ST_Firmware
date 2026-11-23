#ifndef HARDWARE_CHECK_STATE_H
#define HARDWARE_CHECK_STATE_H
#include <stdbool.h>
#include <stdint.h>
#include "linear_hall.h"

/* 维护区由轴拥有；检测期间独占，终态压缩缓存保留在复用区外。 */
#define HARDWARE_CHECK_QUEUE_SIZE 128U

typedef struct
{
    uint32_t cycles, frame, epoch;
    float voltage_v, frequency_hz;
    uint16_t current_q4[4], hall[2];
    bool powered, open_current, fault, stopping, stopped;
} HardwareCheckSample;

typedef struct
{
    int32_t sum;
    int64_t squares;
    float mean_counts, rms_counts;
} HardwareCheckNoiseWindow;

typedef struct
{
    uint32_t samples, first_cycles, last_cycles;
    float last_turns, travel_turns, min_turns, max_turns;
    float backward_turns, forward_turns;
    uint32_t last_progress;
    float progress_turns;
    bool moving;
    uint32_t forward_windows, backward_windows;
} HardwareCheckAngleTrack;

typedef struct
{
    HardwareCheckAngleTrack raw;
    float mean_turns;
    uint32_t first_cycles, samples;
} HardwareCheckAngleWindow;

typedef struct
{
    uint32_t resistance_epoch;
    uint32_t base_epoch, driven_epoch, stage_ms, last_cycles, revision;
    uint32_t static_samples, drive_samples, voltage_samples, temperature_samples;
    uint32_t voltage_outside_samples, voltage_invalid_samples;
    uint8_t ntc_window;
    uint8_t static_item;
    bool static_timing_fault;
    bool hall_installed, hall_calibrated, seen_frame, have_cycles;
    int32_t last_rotor, rotor_travel, rotor_min, rotor_max;
    HardwareCheckNoiseWindow noise[6];
    float min_v, max_v, temperature_c, minimum_temperature_c, maximum_temperature_c;
    float jitter_turns[2];
    uint16_t hall_min[2], hall_max[2];
    LinearHall hall;
    HardwareCheckAngleTrack hall_rest, legs[2][2], settle[2];
    HardwareCheckAngleWindow path[2];
    int first_sign;
    unsigned leg, settled_samples;
    uint32_t leg_samples[2];
    HardwareCheckAngleTrack warmup;
    float warmup_frequency_hz;
    unsigned stable_windows;
    bool rotation_ready;
    uint32_t warmup_begin_cycles;
} HardwareCheckState;

typedef struct
{
    HardwareCheckState state;
    HardwareCheckSample queue[HARDWARE_CHECK_QUEUE_SIZE];
} HardwareCheckWorkspace;
#endif
