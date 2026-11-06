/* PI 状态由调用方拥有；只计算条件积分与输出跟踪，不访问硬件。 */
#include "pi_controller.h"

/* 运行输出与跟踪积分共用限幅，不依赖带有板级类型的工具头。 */
static float clamp_output(float value, float minimum, float maximum)
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
                            float sample_period_s,
                            float output_min,
                            float output_max)
{
    controller->proportional_gain = proportional_gain;
    controller->integral_gain = integral_gain;
    controller->sample_period_s = sample_period_s;
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

    /* 饱和时仅接受使输出回到有效范围的积分方向，避免继续积累误差。 */
    if ((output_candidate >= controller->output_min &&
         output_candidate <= controller->output_max) ||
        (output_candidate > controller->output_max && controller->error < 0.0f) ||
        (output_candidate < controller->output_min && controller->error > 0.0f))
    {
        controller->integral_output = integral_candidate;
    }

    controller->output = clamp_output(controller->proportional_output + controller->integral_output,
                                      controller->output_min,
                                      controller->output_max);
    return controller->output;
}

void PiController_TrackOutput(PiController *controller, float applied_output)
{
    controller->integral_output += applied_output - controller->output;
    controller->integral_output =
        clamp_output(controller->integral_output, controller->output_min, controller->output_max);
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
