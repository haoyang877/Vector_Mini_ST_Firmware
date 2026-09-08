#ifndef POSITION_SMOOTH_TRAJECTORY_H
#define POSITION_SMOOTH_TRAJECTORY_H

#include <stdbool.h>
#include <stdint.h>

/* Rest-to-rest, quintic-velocity profile. Units: rad and seconds.
 * All state belongs to the caller; no allocation or hardware dependencies. */
typedef struct {
    float start, target, distance, direction;
    float acceleration, deceleration, speed_limit, jerk_limit;
    float lower_speed, upper_speed, speed;
    float accel_time, cruise_time, decel_time;
    float inv_accel_time, inv_decel_time;
    float elapsed, duration;
    uint8_t iterations;
    bool ready, failed;
} PositionSmoothTrajectory;

typedef struct { float position, speed, acceleration, jerk; } PositionSmoothSample;

/** Begin a rest-to-rest plan; returns false for invalid/non-finite limits. */
bool PositionSmooth_Begin(PositionSmoothTrajectory *state, float start, float target,
    float speed_limit, float acceleration, float deceleration, float jerk_limit);
/** Perform at most four search iterations. No time advances until ready. */
bool PositionSmooth_Prepare(PositionSmoothTrajectory *state);
/** Evaluate a prepared profile at an absolute time, clamped to its endpoints. */
void PositionSmooth_Sample(const PositionSmoothTrajectory *state, float time_s,
    PositionSmoothSample *sample);
/** Advance by dt and return true once the exact zero-velocity endpoint is reached. */
bool PositionSmooth_Advance(PositionSmoothTrajectory *state, float dt,
    PositionSmoothSample *sample);

#endif
