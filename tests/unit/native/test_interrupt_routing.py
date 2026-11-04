"""验证双板真实main的中断映射、任务计数及AT32硬件确认顺序，不连接硬件。"""

from _bsp_test_support import arguments, production, run_c
from run_position_servo_tests import ROOT, function_source

RTT_FIXTURE = r"""
static unsigned rtt_writes;
void RTT_Sampling(void) {
 assert(g_app_fast_samples == rtt_writes + 1U);
 ++rtt_writes;
}
"""


def rtt_sampling_fixture() -> str:
    """编译真实APP RTT模块，核对格式、六路次序和满缓冲不重试。"""
    return r"""
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "app.h"
#include "motor_axis.h"
static FocState foc;
static EncoderState encoder;
const MotorAxisFeedback MotorAxis_Feedback = {&foc, &encoder};
static unsigned configured, writes;
float Encoder_GetElecPhase(const EncoderState *source) {
 assert(source == &encoder); return 4.0f;
}
float Encoder_GetMechVelContinuous(const EncoderState *source) {
 assert(source == &encoder); return 5.0f;
}
int SEGGER_RTT_ConfigUpBuffer(unsigned channel, const char *name, void *buffer,
                             unsigned bytes, unsigned flags) {
 assert(channel == 1U && buffer != NULL && bytes == 4096U && flags == 0U);
 assert(strcmp(name, "JScope_f4f4f4f4f4f4") == 0);
 ++configured;
 return 0;
}
unsigned SEGGER_RTT_WriteSkipNoLock(unsigned channel, const void *data, unsigned bytes) {
 assert(configured == 1U && channel == 1U && bytes == 24U);
 float channels[6]; memcpy(channels, data, sizeof(channels));
 assert(channels[0] == 1.0f && channels[1] == -2.0f && channels[2] == 3.0f);
 assert(channels[3] == 4.0f && channels[4] == 5.0f && channels[5] == 6.0f);
 return writes++ == 0U ? bytes : 0U;
}
int main(void) {
 foc.phase_current_a = (ABCVect){1.0f, -2.0f, 3.0f};
 foc.current_dq_a.q = 6.0f;
 RTT_Init(); RTT_Sampling(); RTT_Sampling();
 assert(configured == 1U && writes == 2U);
 puts("PASS APP RTT: six float channels, single write, full buffer does not retry");
 return 0;
}
"""


def routing_fixture(platform: str) -> str:
    """编译实际入口和应用节拍，BSP替身只产生已确认事件。"""
    entry = (ROOT / f"projects/{platform}/application/main.c").read_text(encoding="utf-8")
    app = (ROOT / "firmware/app/composition/app.c").read_text(encoding="utf-8")
    mapping = ROOT / f"firmware/platform/bsp/mcu/{platform}/interrupt_map.h"
    return (
        f'#include "{mapping.as_posix()}"\n'
        + r"""
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include "bsp.h"
#include "bsp_adc.h"
#include "bsp_system.h"
volatile uint32_t g_app_fast_samples;
static volatile uint32_t s_supervisor_ticks;
static unsigned order[8], count, ticks;
static bool event_ready;
static void step(unsigned event) { assert(count < 8U); order[count++] = event; }
static void MotorAxis_SampleFast(void) { assert(g_app_fast_samples == 0U); step(2U); }
static void MotorAxis_ControlFast(void) { assert(g_app_fast_samples == 0U); step(3U); }
static void MotorAxis_SupervisorTick(void) { assert(s_supervisor_ticks == 0U); step(5U); }
bool BspAdc_CompleteFast(void) { step(1U); return event_ready; }
void BspAdc_HandleInterrupt(void) {
 step(1U);
 if (event_ready) { BspApp_OnFastSample(); }
}
void BspSystem_HandleControlInterrupt(void) {
 step(4U);
 if (event_ready) { BspApp_OnControlTick(); }
}
void BspSystem_HandleTickInterrupt(void) { ++ticks; }
"""
        + RTT_FIXTURE
        + "\n".join(
            function_source(app, name) for name in ("BspApp_OnFastSample", "App_SupervisorTick")
        )
        + "\n".join(
            function_source(entry, name)
            for name in (
                "BspApp_OnControlTick",
                "MCU_ADC_IRQ_HANDLER",
                "MCU_CONTROL_IRQ_HANDLER",
                "MCU_SYSTICK_IRQ_HANDLER",
            )
        )
        + r"""
int main(void) {
 event_ready = false;
 MCU_ADC_IRQ_HANDLER(); MCU_CONTROL_IRQ_HANDLER();
 assert(count == 2U && order[0] == 1U && order[1] == 4U);
 assert(g_app_fast_samples == 0U && s_supervisor_ticks == 0U);
 count = 0U; event_ready = true;
 MCU_ADC_IRQ_HANDLER(); MCU_CONTROL_IRQ_HANDLER();
 assert(count == 5U && g_app_fast_samples == 1U && s_supervisor_ticks == 1U);
 for (unsigned i = 0U; i < count; ++i) { assert(order[i] == i + 1U); }
 MCU_SYSTICK_IRQ_HANDLER();
 assert(ticks == 1U && count == 5U && g_app_fast_samples == 1U && s_supervisor_ticks == 1U);
 puts("PASS main IRQ routing: hardware confirmation, one task/count per event, independent SysTick");
 return 0;
}
"""
    )


def at32_hardware_fixture() -> str:
    """执行真实AT32派发，覆盖无事件、ADC配对失败、清标志顺序和毫秒回绕。"""
    return (
        r"""
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include "bsp.h"
#include "bsp_adc.h"
#include "bsp_system.h"
#include "board_config.h"
typedef struct { unsigned flags; } FakeAdc;
static FakeAdc fast_adc, aux_adc;
#define MCU_ADC_FAST (&fast_adc)
#define MCU_ADC_AUX (&aux_adc)
#define ADC_PCCE_FLAG 4U
#define MCU_CONTROL_TIMER 6U
#define TMR_OVF_FLAG 1U
static volatile uint32_t s_milliseconds;
static unsigned trace[8], count, fast_calls, control_calls, timer_flag;
static void step(unsigned event) { assert(count < 8U); trace[count++] = event; }
static unsigned adc_flag_get(FakeAdc *adc, unsigned flag) { return adc->flags & flag; }
static void adc_flag_clear(FakeAdc *adc, unsigned flag) {
 step(adc == MCU_ADC_FAST ? 1U : 2U); adc->flags &= ~flag;
}
static unsigned tmr_flag_get(unsigned timer, unsigned flag) {
 assert(timer == 6U && flag == 1U); return timer_flag;
}
static void tmr_flag_clear(unsigned timer, unsigned flag) {
 assert(timer == 6U && flag == 1U); timer_flag = 0U; step(4U);
}
volatile uint32_t g_app_fast_samples;
void MotorAxis_SampleFast(void) {
 assert((fast_adc.flags & ADC_PCCE_FLAG) == 0U);
 assert((aux_adc.flags & ADC_PCCE_FLAG) == 0U);
 assert(g_app_fast_samples == fast_calls);
 step(3U);
}
void MotorAxis_ControlFast(void) {
 assert(count == 3U && trace[2] == 3U && g_app_fast_samples == fast_calls);
 ++fast_calls; step(4U);
}
void BspApp_OnControlTick(void) { assert(timer_flag == 0U); ++control_calls; step(5U); }
"""
        + RTT_FIXTURE
        + production(2, "adc", "BspAdc_CompleteFast")
        + function_source(
            (ROOT / "projects/at32f456/application/main.c").read_text(encoding="utf-8"),
            "MCU_ADC_IRQ_HANDLER",
        )
        + production(
            2, "system", "BspSystem_HandleControlInterrupt", "BspSystem_HandleTickInterrupt"
        )
        + r"""
int main(void) {
 MCU_ADC_IRQ_HANDLER(); assert(count == 0U && fast_calls == 0U && rtt_writes == 0U);
 fast_adc.flags = ADC_PCCE_FLAG;
 MCU_ADC_IRQ_HANDLER(); assert(count == 1U && trace[0] == 1U && fast_calls == 0U);
 count = 0U; fast_adc.flags = aux_adc.flags = ADC_PCCE_FLAG;
 MCU_ADC_IRQ_HANDLER(); assert(count == 4U && fast_calls == 1U && g_app_fast_samples == 1U);
 assert(trace[0] == 1U && trace[1] == 2U && trace[2] == 3U && trace[3] == 4U);
 assert(rtt_writes == 1U);
 count = 0U; aux_adc.flags = ADC_PCCE_FLAG;
 MCU_ADC_IRQ_HANDLER(); assert(count == 0U && fast_calls == 1U && rtt_writes == 1U);
 BspSystem_HandleControlInterrupt(); assert(count == 0U && control_calls == 0U);
 timer_flag = 1U; BspSystem_HandleControlInterrupt();
 assert(count == 2U && trace[0] == 4U && trace[1] == 5U && control_calls == 1U);
 s_milliseconds = UINT32_MAX; BspSystem_HandleTickInterrupt();
 assert(s_milliseconds == 0U && count == 2U);
 puts("PASS AT32 hardware IRQ: missing/paired events, clear-before-callback, SysTick wrap");
 return 0;
}
"""
    )


def stm32_adc_fixture() -> str:
    """执行真实STM32完成回调，拒绝错误句柄或未完成事件，并保留回调后清标志。"""
    return (
        r"""
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include "bsp.h"
typedef struct { uint32_t ISR; } Registers;
typedef struct { Registers *Instance; } ADC_HandleTypeDef;
static Registers fast_adc;
static ADC_HandleTypeDef fast_handle = { &fast_adc }, other_handle;
#define MCU_ADC_FAST (&fast_adc)
#define MCU_ADC_FAST_HANDLE fast_handle
#define ADC_ISR_JEOS 64U
#define ADC_ISR_JEOC 32U
static unsigned calls;
void BspApp_OnFastSample(void) {
 assert((fast_adc.ISR & ADC_ISR_JEOS) != 0U);
 ++calls;
}
"""
        + production(1, "adc", "HAL_ADCEx_InjectedConvCpltCallback")
        + r"""
int main(void) {
 HAL_ADCEx_InjectedConvCpltCallback(&fast_handle);
 assert(calls == 0U && fast_adc.ISR == 0U);
 fast_adc.ISR = ADC_ISR_JEOC;
 HAL_ADCEx_InjectedConvCpltCallback(&fast_handle);
 assert(calls == 0U && fast_adc.ISR == ADC_ISR_JEOC);
 fast_adc.ISR = ADC_ISR_JEOS | ADC_ISR_JEOC;
 HAL_ADCEx_InjectedConvCpltCallback(&other_handle);
 assert(calls == 0U && fast_adc.ISR == (ADC_ISR_JEOS | ADC_ISR_JEOC));
 HAL_ADCEx_InjectedConvCpltCallback(&fast_handle);
 assert(calls == 1U && fast_adc.ISR == (ADC_ISR_JEOS | ADC_ISR_JEOC));
 puts("PASS STM32 ADC: handle/completion filtering, callback before flag clear");
 return 0;
}
"""
    )


def stm32_timer_fixture() -> str:
    """执行生成区真实时基/定时器派发与真实HAL回调，检查温度触发先后顺序。"""
    generated = (
        ROOT / "firmware/platform/bsp/mcu/stm32g4/generated/Core/Src/stm32g4xx_it.c"
    ).read_text(encoding="utf-8")
    return (
        r"""
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include "bsp.h"
#include "bsp_system.h"
#include "board_config.h"
typedef struct { unsigned unused; } TIM_HandleTypeDef;
static TIM_HandleTypeDef htim7, other_timer;
static struct { unsigned ISR; } internal_adc;
#define MCU_CONTROL_TIMER_HANDLE htim7
#define MCU_ADC_INTERNAL (&internal_adc)
#define ADC_ISR_JEOS 64U
#define ADC_ISR_JEOC 32U
static unsigned tick, handled, calls, triggers;
static bool ongoing;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *timer);
static void HAL_IncTick(void) { ++tick; }
static void HAL_TIM_IRQHandler(TIM_HandleTypeDef *timer) {
 assert(timer == &htim7); ++handled; HAL_TIM_PeriodElapsedCallback(timer);
}
void BspApp_OnControlTick(void) { assert(triggers == 0U); ++calls; }
static bool LL_ADC_INJ_IsConversionOngoing(void *adc) {
 assert(adc == &internal_adc); return ongoing;
}
static void LL_ADC_INJ_StartConversion(void *adc) {
 assert(adc == &internal_adc && calls == 1U && internal_adc.ISR == 96U); ++triggers;
}
"""
        + production(1, "system", "HAL_TIM_PeriodElapsedCallback")
        + function_source(generated, "SysTick_Handler").replace(
            "SysTick_Handler", "BspSystem_HandleTickInterrupt"
        )
        + function_source(generated, "TIM7_IRQHandler").replace(
            "TIM7_IRQHandler", "BspSystem_HandleControlInterrupt"
        )
        + r"""
int main(void) {
 HAL_TIM_PeriodElapsedCallback(&other_timer); assert(calls == 0U && triggers == 0U);
 BspSystem_HandleTickInterrupt(); assert(tick == 1U);
 BspSystem_HandleControlInterrupt(); assert(handled == 1U && calls == 1U && triggers == 1U);
 calls = triggers = 0U; ongoing = true;
 BspSystem_HandleControlInterrupt(); assert(handled == 2U && calls == 1U && triggers == 0U);
 puts("PASS STM32 timers: HAL tick retained, handle filtering, callback before temperature trigger");
 return 0;
}
"""
    )


def main() -> None:
    """运行双板入口与硬件事件顺序夹具。"""
    args = arguments(__doc__)
    for board, platform in ((1, "stm32g4"), (2, "at32f456")):
        run_c(args, f"{platform}_main_irqs", routing_fixture(platform), board)
    run_c(args, "at32_hardware_irqs", at32_hardware_fixture(), 2)
    run_c(
        args,
        "at32_rtt_sampling",
        rtt_sampling_fixture(),
        2,
        [
            "-I",
            ROOT / "firmware/third_party/segger_rtt",
            ROOT / "firmware/app/composition/app_rtt.c",
        ],
    )
    run_c(args, "stm32_adc_irqs", stm32_adc_fixture(), 1)
    run_c(args, "stm32_timer_irqs", stm32_timer_fixture(), 1)


if __name__ == "__main__":
    main()
