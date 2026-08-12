#include "foc_pid.h"

#include "utils.h"

void PI_Controller_Configure(PI_Controller_TypeDef *controller, float proportional_gain, float integral_gain, float sample_period, float output_min, float output_max)
{
    controller->Kp = proportional_gain;
    controller->Ki = integral_gain;
    controller->Ts = sample_period;
    controller->Umin = output_min;
    controller->Umax = output_max;
}

float PI_Controller_Run(PI_Controller_TypeDef *controller, float reference, float feedback)
{
    float integral_candidate;
    float output_candidate;

    controller->Ref = reference;
    controller->Fbk = feedback;
    controller->Err = reference - feedback;
    controller->Up = controller->Kp * controller->Err;

    integral_candidate = controller->Ui + controller->Ki * controller->Err * controller->Ts;
    output_candidate = controller->Up + integral_candidate;

    if ((output_candidate >= controller->Umin && output_candidate <= controller->Umax) ||
        (output_candidate > controller->Umax && controller->Err < 0.0f) ||
        (output_candidate < controller->Umin && controller->Err > 0.0f))
    {
        controller->Ui = integral_candidate;
    }

    controller->Out = constrain(controller->Up + controller->Ui, controller->Umin, controller->Umax);
    return controller->Out;
}

void PI_Controller_TrackOutput(PI_Controller_TypeDef *controller, float applied_output)
{
    controller->Ui += applied_output - controller->Out;
    controller->Ui = constrain(controller->Ui, controller->Umin, controller->Umax);
    controller->Out = applied_output;
}

void PI_Controller_Reset(PI_Controller_TypeDef *controller)
{
    controller->Ref = 0.0f;
    controller->Fbk = 0.0f;
    controller->Err = 0.0f;
    controller->Up = 0.0f;
    controller->Ui = 0.0f;
    controller->Out = 0.0f;
}
