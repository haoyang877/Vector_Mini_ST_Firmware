#ifndef DOMAIN_TRAPEZOIDAL_TRAJECTORY_H
#define DOMAIN_TRAPEZOIDAL_TRAJECTORY_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    float Y;
    float Yd;
    float Ydd;
    float start_position;
    float start_velocity;
    float end_position;
    float acc;
    float vel;
    float dec;
    float acc_distance;
    float t_acc;
    float t_vel;
    float t_dec;
    float t_total;
    uint32_t tick;
    bool profile_done;
} TrapezoidalTrajectoryContext;

void TrapezoidalTrajectory_Reset(TrapezoidalTrajectoryContext *context);
bool TrapezoidalTrajectory_Plan(TrapezoidalTrajectoryContext *context,
    float position, float start_position, float start_velocity,
    float maximum_velocity, float maximum_acceleration,
    float maximum_deceleration);
void TrapezoidalTrajectory_Evaluate(TrapezoidalTrajectoryContext *context,
    float sample_time);

#endif
