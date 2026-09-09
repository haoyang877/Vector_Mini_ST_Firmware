"""Exercise the actual board fast dispatch with modeled W1C registers."""
import argparse
from pathlib import Path
import subprocess
from run_position_servo_tests import ROOT, function_source


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--cc', required=True)
    p.add_argument('--out', type=Path, required=True)
    args = p.parse_args()
    out = args.out.resolve(); out.mkdir(parents=True, exist_ok=True)
    source = function_source((ROOT/'Core/Src/stm32g4xx_it.c').read_text(),
                             'Board_ADC2DispatchInterrupt')
    fixture = r'''
#undef NDEBUG
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
typedef struct { uint32_t ISR, IER, JSQR, CFGR; } Registers;
typedef struct { Registers *Instance; uint32_t State; } ADC_HandleTypeDef;
static Registers reg;
static ADC_HandleTypeDef hadc2 = { &reg, 0 };
static struct { uint32_t CCR; } common;
#define ADC12_COMMON (&common)
#define ADC_FLAG_JEOC (1U << 5)
#define ADC_FLAG_JEOS (1U << 6)
#define ADC_JSQR_JEXTEN (3U << 6)
#define ADC_CFGR_JAUTO (1U << 25)
#define ADC_CFGR_JQM (1U << 21)
#define ADC_CCR_DUAL 31U
#define HAL_ADC_STATE_ERROR_INTERNAL 16U
#define HAL_ADC_STATE_INJ_EOC 512U
static unsigned callbacks, fallbacks, clears;
static void HAL_ADC_IRQHandler(ADC_HandleTypeDef *h) {
    assert(h == &hadc2); fallbacks++;
}
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *h) {
    assert(h == &hadc2 && (reg.ISR & ADC_FLAG_JEOS));
    assert((h->State & HAL_ADC_STATE_INJ_EOC) && clears == 0);
    callbacks++;
}
void clear_flags(ADC_HandleTypeDef *h, uint32_t flags) {
    assert(h == &hadc2 && callbacks == 1);
    assert(flags == (ADC_FLAG_JEOC | ADC_FLAG_JEOS));
    h->Instance->ISR &= ~flags; clears++;
}
#define __HAL_ADC_CLEAR_FLAG(h,f) clear_flags(h,f)
''' + source + r'''
int main(void) {
    unsigned pending, variant, cases = 0;
    for (pending = 0; pending < 2048; ++pending) {
        for (variant = 0; variant < 6; ++variant) {
            unsigned fast;
            callbacks = fallbacks = clears = 0;
            reg.ISR = pending | (1U << 15); /* unrelated disabled flag */
            reg.IER = 2047U;
            reg.JSQR = variant == 1 ? 0 : ADC_JSQR_JEXTEN;
            reg.CFGR = variant == 2 ? ADC_CFGR_JAUTO : variant == 3 ? ADC_CFGR_JQM : 0;
            common.CCR = variant == 4 ? 1 : 0;
            hadc2.State = variant == 5 ? HAL_ADC_STATE_ERROR_INTERNAL : 128;
            fast = !USE_HAL_ADC_REGISTER_CALLBACKS && pending == ADC_FLAG_JEOS && variant == 0;
            Board_ADC2DispatchInterrupt();
            assert(callbacks == fast && clears == fast);
            assert(fallbacks == (pending != 0 && !fast));
            assert(reg.ISR == ((pending | (1U << 15)) & (fast ? ~ADC_FLAG_JEOS : ~0U)));
            cases++;
        }
    }
    /* Disabled JEOC may coexist with the enabled JEOS; both must be cleared. */
    callbacks = fallbacks = clears = 0;
    reg.ISR = ADC_FLAG_JEOS | ADC_FLAG_JEOC; reg.IER = ADC_FLAG_JEOS;
    reg.JSQR = ADC_JSQR_JEXTEN; reg.CFGR = common.CCR = hadc2.State = 0;
    Board_ADC2DispatchInterrupt();
    assert(USE_HAL_ADC_REGISTER_CALLBACKS ? fallbacks == 1 : reg.ISR == 0);
    printf("PASS %u dispatch cases, callback registration=%d\n", cases+1, USE_HAL_ADC_REGISTER_CALLBACKS);
    return 0;
}
'''
    c = out/'dispatch.c'; c.write_text(fixture)
    compiler = [args.cc] + (['cc'] if Path(args.cc).stem == 'zig' else [])
    for callbacks in [0, 1]:
        exe = out/f'dispatch_{callbacks}.exe'
        subprocess.run(compiler + ['-std=c99', '-O2', '-Wall',
            f'-DUSE_HAL_ADC_REGISTER_CALLBACKS={callbacks}', str(c), '-o', str(exe)], check=True)
        subprocess.run([str(exe)], check=True)


if __name__ == '__main__':
    main()
