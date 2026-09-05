#include "pi_controller.h"

static float PI_Controller_Clamp(float value, float minimum, float maximum)
{
    if (value < minimum)
        return minimum;
    if (value > maximum)
        return maximum;
    return value;
}

void PI_Controller_Configure(PiController *controller, float proportional_gain, float integral_gain, float sample_period, float output_min, float output_max)
{
    controller->Kp = proportional_gain;
    controller->Ki = integral_gain;
    controller->Ts = sample_period;
    controller->Umin = output_min;
    controller->Umax = output_max;
}

float PI_Controller_Run(PiController *controller, float reference, float feedback)
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

    controller->Out = PI_Controller_Clamp(controller->Up + controller->Ui,
        controller->Umin, controller->Umax);
    return controller->Out;
}

void PI_Controller_TrackOutput(PiController *controller, float applied_output)
{
    controller->Ui += applied_output - controller->Out;
    controller->Ui = PI_Controller_Clamp(controller->Ui,
        controller->Umin, controller->Umax);
    controller->Out = applied_output;
}

void PI_Controller_Reset(PiController *controller)
{
    controller->Ref = 0.0f;
    controller->Fbk = 0.0f;
    controller->Err = 0.0f;
    controller->Up = 0.0f;
    controller->Ui = 0.0f;
    controller->Out = 0.0f;
}
