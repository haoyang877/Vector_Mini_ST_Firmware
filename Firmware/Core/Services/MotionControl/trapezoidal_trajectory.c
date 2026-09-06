#include "trapezoidal_trajectory.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

static float Trajectory_Square(float value)
{
    return value * value;
}

static float Trajectory_Max(float left, float right)
{
    return left > right ? left : right;
}

static float Trajectory_Sign(float value)
{
    return value >= 0.0f ? 1.0f : -1.0f;
}

static float Trajectory_CopySign(float magnitude, float sign)
{
    return sign >= 0.0f ? fabsf(magnitude) : -fabsf(magnitude);
}

void TrapezoidalTrajectory_Reset(TrapezoidalTrajectoryContext *context)
{
    if (context != NULL)
        memset(context, 0, sizeof(*context));
}

bool TrapezoidalTrajectory_Plan(TrapezoidalTrajectoryContext *trajectory,
    float position, float start_position, float start_velocity,
    float maximum_velocity, float maximum_acceleration,
    float maximum_deceleration)
{
    float distance;
    float stop_distance;
    float stopping_displacement;
    float direction;
    float minimum_distance;

    if (trajectory == NULL || !isfinite(position) || !isfinite(start_position) ||
        !isfinite(start_velocity) || !isfinite(maximum_velocity) ||
        maximum_velocity <= 0.0f || !isfinite(maximum_acceleration) ||
        maximum_acceleration <= 0.0f || !isfinite(maximum_deceleration) ||
        maximum_deceleration <= 0.0f)
        return false;

    distance = position - start_position;
    stop_distance = Trajectory_Square(start_velocity) /
        (2.0f * maximum_deceleration);
    stopping_displacement = Trajectory_CopySign(stop_distance, start_velocity);
    direction = Trajectory_Sign(distance - stopping_displacement);
    trajectory->acc = direction * maximum_acceleration;
    trajectory->dec = -direction * maximum_deceleration;
    trajectory->vel = direction * maximum_velocity;

    if ((direction * start_velocity) > (direction * trajectory->vel))
        trajectory->acc = -direction * maximum_acceleration;

    trajectory->t_acc = (trajectory->vel - start_velocity) / trajectory->acc;
    trajectory->t_dec = -trajectory->vel / trajectory->dec;
    minimum_distance = 0.5f * trajectory->t_acc *
        (trajectory->vel + start_velocity) +
        0.5f * trajectory->t_dec * trajectory->vel;

    if (direction * distance < direction * minimum_distance)
    {
        trajectory->vel = direction * sqrtf(Trajectory_Max(
            (trajectory->dec * Trajectory_Square(start_velocity) +
             2.0f * trajectory->acc * trajectory->dec * distance) /
            (trajectory->dec - trajectory->acc), 0.0f));
        trajectory->t_acc = Trajectory_Max(0.0f,
            (trajectory->vel - start_velocity) / trajectory->acc);
        trajectory->t_dec = Trajectory_Max(0.0f,
            -trajectory->vel / trajectory->dec);
        trajectory->t_vel = 0.0f;
    }
    else
    {
        trajectory->t_vel = (distance - minimum_distance) / trajectory->vel;
    }

    trajectory->t_total = trajectory->t_acc + trajectory->t_vel +
        trajectory->t_dec;
    trajectory->start_position = start_position;
    trajectory->start_velocity = start_velocity;
    trajectory->end_position = position;
    trajectory->acc_distance = start_position +
        start_velocity * trajectory->t_acc +
        0.5f * trajectory->acc * Trajectory_Square(trajectory->t_acc);
    trajectory->tick = 0U;
    trajectory->profile_done = false;
    return true;
}

void TrapezoidalTrajectory_Evaluate(TrapezoidalTrajectoryContext *trajectory,
    float sample_time)
{
    float time;

    if (trajectory == NULL || trajectory->profile_done ||
        !isfinite(sample_time) || sample_time <= 0.0f)
        return;

    trajectory->tick++;
    time = trajectory->tick * sample_time;
    if (time < trajectory->t_acc)
    {
        trajectory->Y = trajectory->start_position +
            trajectory->start_velocity * time +
            0.5f * trajectory->acc * Trajectory_Square(time);
        trajectory->Yd = trajectory->start_velocity + trajectory->acc * time;
        trajectory->Ydd = trajectory->acc;
    }
    else if (time < trajectory->t_acc + trajectory->t_vel)
    {
        trajectory->Y = trajectory->acc_distance + trajectory->vel *
            (time - trajectory->t_acc);
        trajectory->Yd = trajectory->vel;
        trajectory->Ydd = 0.0f;
    }
    else if (time < trajectory->t_total)
    {
        float deceleration_time = time - trajectory->t_total;
        trajectory->Y = trajectory->end_position +
            0.5f * trajectory->dec * Trajectory_Square(deceleration_time);
        trajectory->Yd = trajectory->dec * deceleration_time;
        trajectory->Ydd = trajectory->dec;
    }
    else
    {
        trajectory->Y = trajectory->end_position;
        trajectory->Yd = 0.0f;
        trajectory->Ydd = 0.0f;
        trajectory->profile_done = true;
    }
}
