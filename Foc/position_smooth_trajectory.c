#include "position_smooth_trajectory.h"
#include <math.h>
#include <string.h>

/* For velocity f(u)=10u^3-15u^4+6u^5, max f'=15/8 and
 * max |f''|=10/sqrt(3). Round the bounds upward. */
#define SMOOTH_ACCEL_PEAK 1.875f
#define SMOOTH_JERK_PEAK 5.773503f
#define SMOOTH_SEARCH_ITERATIONS 16U

static float maximum(float a, float b) { return a > b ? a : b; }

static float ramp_distance(const PositionSmoothTrajectory *s, float speed,
    float *ta, float *td)
{
    float jerk_time = sqrtf(SMOOTH_JERK_PEAK * speed / s->jerk_limit);
    *ta = maximum(SMOOTH_ACCEL_PEAK * speed / s->acceleration, jerk_time);
    *td = maximum(SMOOTH_ACCEL_PEAK * speed / s->deceleration, jerk_time);
    return 0.5f * speed * (*ta + *td);
}

bool PositionSmooth_Begin(PositionSmoothTrajectory *s, float start, float target,
    float speed, float acceleration, float deceleration, float jerk)
{
    if (s == 0 || !isfinite(start) || !isfinite(target) ||
        !isfinite(target-start) || !isfinite(speed) || speed <= 0 ||
        !isfinite(acceleration) || acceleration <= 0 ||
        !isfinite(deceleration) || deceleration <= 0 ||
        !isfinite(jerk) || jerk <= 0 ||
        !isfinite(SMOOTH_ACCEL_PEAK*speed/acceleration) ||
        !isfinite(SMOOTH_ACCEL_PEAK*speed/deceleration) ||
        !isfinite(SMOOTH_JERK_PEAK*speed/jerk)) return false;
    memset(s, 0, sizeof(*s));
    s->start = start; s->target = target;
    s->distance = fabsf(target-start); s->direction = target >= start ? 1.0f : -1.0f;
    s->speed_limit = speed; s->upper_speed = speed;
    s->acceleration = acceleration; s->deceleration = deceleration; s->jerk_limit = jerk;
    if (s->distance == 0) s->ready = true;
    return true;
}

bool PositionSmooth_Prepare(PositionSmoothTrajectory *s)
{
    unsigned i;
    float ta, td, distance, candidate;
    if (s->ready) return true;
    if (s->iterations == 0U) {
        distance = ramp_distance(s, s->speed_limit, &ta, &td);
        if (distance <= s->distance) {
            s->lower_speed = s->speed_limit;
            s->iterations = SMOOTH_SEARCH_ITERATIONS;
        }
    }
    for (i = 0; i < 4U && s->iterations < SMOOTH_SEARCH_ITERATIONS; ++i) {
        candidate = 0.5f * (s->lower_speed+s->upper_speed);
        if (candidate <= 0) { s->failed = true; return false; }
        distance = ramp_distance(s, candidate, &ta, &td);
        if (distance <= s->distance) s->lower_speed = candidate;
        else s->upper_speed = candidate;
        ++s->iterations;
    }
    if (s->iterations < SMOOTH_SEARCH_ITERATIONS) return false;
    s->speed = s->lower_speed;
    /* An extremely small move may be below search resolution. Continue with
     * a smaller bracket instead of fabricating a zero-speed moving profile. */
    if (s->speed <= 0) { s->iterations = 1U; return false; }
    distance = ramp_distance(s, s->speed, &s->accel_time, &s->decel_time);
    s->cruise_time = maximum((s->distance-distance)/s->speed, 0.0f);
    s->duration = s->accel_time+s->cruise_time+s->decel_time;
    s->inv_accel_time = 1.0f/s->accel_time;
    s->inv_decel_time = 1.0f/s->decel_time;
    s->ready = isfinite(s->duration) && isfinite(s->inv_accel_time) &&
        isfinite(s->inv_decel_time) && s->duration > 0;
    s->failed = !s->ready;
    return s->ready;
}

/* Stable near u=0. For deceleration use time remaining, avoiding subtraction
 * of two almost equal velocities/positions at the endpoint. */
static void ramp(float u, float *position, float *velocity, float *accel, float *jerk)
{
    float u2 = u*u, u3 = u2*u, one_minus_u = 1.0f-u;
    *position = u2*u2*(2.5f+u*(-3.0f+u));
    *velocity = u3*(10.0f+u*(-15.0f+6.0f*u));
    *accel = 30.0f*u2*one_minus_u*one_minus_u;
    *jerk = 60.0f*u*one_minus_u*(1.0f-2.0f*u);
}

void PositionSmooth_Sample(const PositionSmoothTrajectory *s, float time,
    PositionSmoothSample *o)
{
    float q, v, a, j, gain, u;
    o->speed = o->acceleration = o->jerk = 0;
    if (!s->ready || time <= 0) { o->position = s->start; return; }
    if (time >= s->duration) { o->position = s->target; return; }
    gain = s->direction*s->speed;
    if (time < s->accel_time) {
        u = time*s->inv_accel_time;
        ramp(u, &q, &v, &a, &j);
        o->position = s->start+gain*s->accel_time*q;
        o->acceleration = gain*s->inv_accel_time*a;
        o->jerk = gain*s->inv_accel_time*s->inv_accel_time*j;
    } else if (time < s->accel_time+s->cruise_time) {
        o->position = s->start+gain*(time-0.5f*s->accel_time);
        o->speed = gain;
        return;
    } else {
        u = (s->duration-time)*s->inv_decel_time;
        /* Round-off at the cruise/deceleration join must not extrapolate. */
        if (u > 1.0f) u = 1.0f;
        ramp(u, &q, &v, &a, &j);
        o->position = s->target-gain*s->decel_time*q;
        o->acceleration = -gain*s->inv_decel_time*a;
        o->jerk = gain*s->inv_decel_time*s->inv_decel_time*j;
    }
    o->speed = gain*v;
}

bool PositionSmooth_Advance(PositionSmoothTrajectory *s, float dt, PositionSmoothSample *o)
{
    s->elapsed += dt;
    if (s->elapsed > s->duration) s->elapsed = s->duration;
    PositionSmooth_Sample(s, s->elapsed, o);
    return s->ready && s->elapsed >= s->duration;
}
