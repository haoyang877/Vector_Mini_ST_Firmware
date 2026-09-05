#ifndef DOMAIN_PI_CONTROLLER_H
#define DOMAIN_PI_CONTROLLER_H

typedef struct
{
    float Kp;
    float Ki;
    float Ts;
    float Umin;
    float Umax;
    float Ref;
    float Fbk;
    float Err;
    float Up;
    float Ui;
    float Out;
} PiController;

void PI_Controller_Configure(PiController *controller, float proportional_gain, float integral_gain, float sample_period, float output_min, float output_max);
float PI_Controller_Run(PiController *controller, float reference, float feedback);
void PI_Controller_TrackOutput(PiController *controller, float applied_output);
void PI_Controller_Reset(PiController *controller);

#endif
