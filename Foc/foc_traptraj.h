#ifndef __FOC_TRAPTRAJ_H__
#define __FOC_TRAPTRAJ_H__

#include "main.h"
#include <stdbool.h>

typedef struct
{
    // Step
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
}Traj_TypeDef;

void TRAJ_plan(float position, float start_position, float start_velocity, float Vmax, float Amax, float Dmax);
void TRAJ_eval(void);
float TRAJ_Get_Y(void);
float TRAJ_Get_Yd(void);

#endif