#ifndef DOMAIN_POSITION_CASCADE_H
#define DOMAIN_POSITION_CASCADE_H

#include <stdbool.h>
#include <stdint.h>

#include "pi_controller.h"
#include "trapezoidal_trajectory.h"

typedef struct
{
    float target_position;
    float position_error_window;
    float acceleration;
    float deceleration;
    float maximum_speed;
    float position_kp;
    float position_kd;
    float speed_kp;
    float speed_ki;
    float current_limit_a;
    float position_kp_limit;
    float position_kd_limit;
    float position_sample_period_s;
    float speed_sample_period_s;
    uint16_t position_loop_divider;
    uint16_t speed_loop_divider;
} PositionCascadeConfig;

typedef struct
{
    float position_reference;
    float speed_reference;
    float speed_feedback;
    float iq_reference;
    bool target_reached;
} PositionCascadeOutput;

typedef struct
{
    PiController speed_controller;
    TrapezoidalTrajectoryContext trajectory;
    uint16_t position_loop_count;
    uint16_t speed_loop_count;
    float last_target;
    float last_acceleration;
    float last_deceleration;
    float last_maximum_speed;
    float last_position_error;
    float position_reference;
    float speed_reference;
    float iq_reference;
    bool target_reached;
    bool initialized;
} PositionCascadeContext;

void PositionCascade_Reset(PositionCascadeContext *context);
bool PositionCascade_Update(PositionCascadeContext *context,
    const PositionCascadeConfig *config,
    float measured_position, float measured_speed,
    PositionCascadeOutput *output);

#endif
