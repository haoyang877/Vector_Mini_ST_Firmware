#ifndef __FOC_PID_H__
#define __FOC_PID_H__

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
} PI_Controller_TypeDef;

void PI_Controller_Configure(PI_Controller_TypeDef *controller, float proportional_gain, float integral_gain, float sample_period, float output_min, float output_max);
float PI_Controller_Run(PI_Controller_TypeDef *controller, float reference, float feedback);
void PI_Controller_TrackOutput(PI_Controller_TypeDef *controller, float applied_output);
void PI_Controller_Reset(PI_Controller_TypeDef *controller);

#endif
