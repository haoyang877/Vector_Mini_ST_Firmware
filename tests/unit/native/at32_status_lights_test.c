#include "status_lights.h"
#include "bsp_system.h"
#include <assert.h>

/* BOARD_RESOURCES */
static struct
{
    uint32_t scr;
} s_port;
#define GPIOC (&s_port)
#define GPIO_PINS_13 (1U << 13)
#define GPIO_PINS_14 (1U << 14)

enum
{
    CRM_GPIOC_PERIPH_CLOCK = 1,
    TRUE,
    GPIO_MODE_OUTPUT,
    GPIO_OUTPUT_PUSH_PULL,
    GPIO_PULL_NONE,
    GPIO_DRIVE_STRENGTH_MODERATE
};
typedef struct
{
    unsigned gpio_pins, gpio_mode, gpio_out_type, gpio_pull, gpio_drive_strength;
} gpio_init_type;

static void crm_periph_clock_enable(unsigned clock, unsigned enabled)
{
    assert(clock == CRM_GPIOC_PERIPH_CLOCK && enabled == TRUE);
}

static void gpio_default_para_init(gpio_init_type *pins)
{
    *pins = (gpio_init_type){0};
}

static void gpio_init(const void *port, const gpio_init_type *pins)
{
    assert(port == GPIOC && pins->gpio_pins == 0x6000U && s_port.scr == 0x6000U);
    assert(pins->gpio_mode == GPIO_MODE_OUTPUT && pins->gpio_out_type == GPIO_OUTPUT_PUSH_PULL);
    assert(pins->gpio_pull == GPIO_PULL_NONE &&
           pins->gpio_drive_strength == GPIO_DRIVE_STRENGTH_MODERATE);
}

/* FUNCTIONS */

static void
tick(StatusLights *lights, MotorState state, bool fault, bool check, bool red, bool blue)
{
    bool changed = lights->red != red || lights->blue != blue;
    assert(StatusLights_Step(lights, state, fault, check) == changed);
    assert(lights->red == red && lights->blue == blue);
    if (changed)
    {
        BspSystem_WriteStatusLights(red, blue);
    }
    assert(s_port.scr == (red ? 0x20004000U : blue ? 0x40002000U : 0x6000U));
}

int main(void)
{
    StatusLights lights = {0};
    McuStatusLights_Init();
    assert(s_port.scr == 0x6000U);
    /* 按实际 2 kHz 调用，核对三个重复周期中每个亮灭边沿。 */
    for (unsigned n = 0; n < 12000U; ++n)
    {
        tick(&lights, MOTOR_STATE_IDLE, false, false, false, n % 4000U < 200U);
    }
    for (unsigned n = 0; n < 1200U; ++n)
    {
        tick(&lights, MOTOR_STATE_IDLE, false, true, false, n % 400U < 200U);
    }
    tick(&lights, MOTOR_STATE_RUNNING, false, true, false, true);
    tick(&lights, MOTOR_STATE_RUNNING, true, true, true, false);
    tick(&lights, MOTOR_STATE_FAULT, false, false, true, false);
    for (unsigned n = 0; n < 6000U; ++n)
    {
        unsigned phase = n % 2000U;
        tick(&lights,
             MOTOR_STATE_CALIBRATION,
             false,
             true,
             false,
             phase < 200U || (phase >= 400U && phase < 600U));
    }
    tick(&lights, MOTOR_STATE_SELF_TEST, false, false, false, false);
    tick(&lights, MOTOR_STATE_IDLE, false, false, false, true);
    return 0;
}
