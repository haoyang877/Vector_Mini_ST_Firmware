#include "app.h"
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

static AppContext g_app;
static bool g_hardware_check_capture;
static uint32_t s_now_ms;
static MotorStatus s_sample;
static bool s_valid = true;

uint32_t BspSystem_TimeMs(void)
{
    return s_now_ms;
}

bool MotorStateMachine_ReadStatus(const MotorStateMachine *machine, MotorStatus *status)
{
    assert(machine == &g_app.axis.state_machine);
    *status = s_sample;
    return s_valid;
}

/* FUNCTIONS */

int main(void)
{
    const uint32_t off = 0x6000U, red = 0x20004000U, blue = 0x40002000U;
    init_status_lights();
    assert(s_port.scr == off);
    /* 从回绕前 100 ms 开始，检查跨界时心跳宽度和刷新限频。 */
    uint32_t start_ms = UINT32_MAX - 99U;
    s_sample.state = MOTOR_STATE_IDLE;
    s_now_ms = start_ms;
    update_status_lights();
    assert(s_port.scr == blue);
    s_now_ms += 100U;
    update_status_lights();
    assert(s_port.scr == off);
    s_now_ms = start_ms + 2000U;
    update_status_lights();
    assert(s_port.scr == blue);

    g_hardware_check_capture = true;
    s_now_ms += 50U;
    update_status_lights();
    assert(s_port.scr == blue);
    s_now_ms += 100U;
    update_status_lights();
    assert(s_port.scr == off);
    s_now_ms += 100U;
    update_status_lights();
    assert(s_port.scr == blue);

    s_sample.state = MOTOR_STATE_RUNNING;
    s_now_ms += 100U;
    update_status_lights();
    assert(s_port.scr == blue);
    s_sample.fault = 1U;
    s_now_ms += 49U;
    update_status_lights();
    assert(s_port.scr == blue);
    ++s_now_ms;
    update_status_lights();
    assert(s_port.scr == red);
    s_sample.state = MOTOR_STATE_FAULT;
    s_sample.fault = 0U;
    s_now_ms += 50U;
    update_status_lights();
    assert(s_port.scr == red);
    s_sample.state = MOTOR_STATE_CALIBRATION;
    s_valid = false;
    s_now_ms += 50U;
    update_status_lights();
    assert(s_port.scr == red);
    s_valid = true;
    for (unsigned ms = 0U; ms <= 1000U; ms += 50U)
    {
        s_now_ms += 50U;
        update_status_lights();
        assert(s_port.scr ==
               ((ms < 100U || (ms >= 200U && ms < 300U) || ms == 1000U) ? blue : off));
    }
    s_sample.state = MOTOR_STATE_IDLE;
    g_hardware_check_capture = false;
    s_now_ms += 50U;
    update_status_lights();
    assert(s_port.scr == blue);
    return 0;
}
