/* PI 状态由调用方拥有；只计算条件积分与输出跟踪，不访问硬件。 */
#include "foc_pid.h"

/* Keep the reusable PI independent of board/vendor utility headers. */
static float pi_clamp(float value, float minimum, float maximum)
{
    if (value < minimum)
    {
        return minimum;
    }
    if (value > maximum)
    {
        return maximum;
    }
    return value;
}

void PiController_Configure(PiController *controller,
                            float proportional_gain,
                            float integral_gain,
                            float sample_period,
                            float output_min,
                            float output_max)
{
    controller->proportional_gain = proportional_gain;
    controller->integral_gain = integral_gain;
    controller->sample_period_s = sample_period;
    controller->output_min = output_min;
    controller->output_max = output_max;
}

float PiController_Run(PiController *controller, float reference, float feedback)
{
    float integral_candidate;
    float output_candidate;

    controller->reference = reference;
    controller->feedback = feedback;
    controller->error = reference - feedback;
    controller->proportional_output = controller->proportional_gain * controller->error;

    integral_candidate = controller->integral_output + controller->integral_gain *
                                                           controller->error *
                                                           controller->sample_period_s;
    output_candidate = controller->proportional_output + integral_candidate;

    if ((output_candidate >= controller->output_min &&
         output_candidate <= controller->output_max) ||
        (output_candidate > controller->output_max && controller->error < 0.0f) ||
        (output_candidate < controller->output_min && controller->error > 0.0f))
    {
        controller->integral_output = integral_candidate;
    }

    controller->output = pi_clamp(controller->proportional_output + controller->integral_output,
                                  controller->output_min,
                                  controller->output_max);
    return controller->output;
}

void PiController_TrackOutput(PiController *controller, float applied_output)
{
    controller->integral_output += applied_output - controller->output;
    controller->integral_output =
        pi_clamp(controller->integral_output, controller->output_min, controller->output_max);
    controller->output = applied_output;
}

void PiController_Reset(PiController *controller)
{
    controller->reference = 0.0f;
    controller->feedback = 0.0f;
    controller->error = 0.0f;
    controller->proportional_output = 0.0f;
    controller->integral_output = 0.0f;
    controller->output = 0.0f;
}
