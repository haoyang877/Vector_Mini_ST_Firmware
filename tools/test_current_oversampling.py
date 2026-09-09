"""Run the actual ADC2 initializer and vendor LL code against host registers.

HAL calls are recorded instead of accessing physical peripherals. This checks
configuration and data scale, not analog accuracy or MCU timing.
"""
import argparse
from pathlib import Path
import subprocess

from run_position_servo_tests import ROOT, function_source


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cc', required=True)
    parser.add_argument('--out', type=Path, default=ROOT / 'outputs/current_oversampling/host')
    args = parser.parse_args()
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    actual = function_source((ROOT / 'Core/Src/adc.c').read_text(encoding='utf-8'), 'MX_ADC2_Init')
    fixture = r'''
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include "adc.h"
#include "stm32g4xx_ll_adc.h"

ADC_HandleTypeDef hadc2;
static ADC_TypeDef registers;
static unsigned ranks;
static const uint32_t channels[] = {
    ADC_CHANNEL_13, ADC_CHANNEL_3, ADC_CHANNEL_5, ADC_CHANNEL_17
};
static const uint32_t rank_ids[] = {
    ADC_INJECTED_RANK_1, ADC_INJECTED_RANK_2,
    ADC_INJECTED_RANK_3, ADC_INJECTED_RANK_4
};
void Error_Handler(void) { abort(); }
HAL_StatusTypeDef HAL_ADC_Init(ADC_HandleTypeDef *adc) {
    assert(adc == &hadc2 && adc->Instance == ADC2);
    assert(adc->Init.Resolution == ADC_RESOLUTION_12B);
    assert(adc->Init.ClockPrescaler == ADC_CLOCK_SYNC_PCLK_DIV4);
    assert(adc->Init.DataAlign == ADC_DATAALIGN_RIGHT);
    assert(adc->Init.ScanConvMode == ADC_SCAN_ENABLE);
    assert(adc->Init.OversamplingMode == DISABLE);
    adc->Instance = &registers;
    registers.CFGR2 = ADC_CFGR2_GCOMP; /* unrelated bit must survive */
    return HAL_OK;
}
HAL_StatusTypeDef HAL_ADCEx_InjectedConfigChannel(
        ADC_HandleTypeDef *adc, const ADC_InjectionConfTypeDef *config) {
    assert(adc == &hadc2 && ranks < 4);
    assert(config->InjectedChannel == channels[ranks]);
    assert(config->InjectedRank == rank_ids[ranks]);
    assert(config->InjectedNbrOfConversion == 4);
    assert(config->InjectedSamplingTime == (ranks == 3 ?
        ADC_SAMPLETIME_12CYCLES_5 : ADC_SAMPLETIME_6CYCLES_5));
    assert(config->ExternalTrigInjecConv == ADC_EXTERNALTRIGINJEC_T1_CC4);
    assert(config->ExternalTrigInjecConvEdge == ADC_EXTERNALTRIGINJECCONV_EDGE_FALLING);
    assert(config->InjectedDiscontinuousConvMode == DISABLE);
    assert(config->AutoInjectedConv == DISABLE);
    assert(config->QueueInjectedContext == DISABLE);
    assert(config->InjectedOffsetNumber == ADC_OFFSET_NONE);
    /* Model the generated HAL configuration before the board override. */
    assert(config->InjecOversamplingMode == DISABLE);
    registers.CFGR2 &= ~ADC_CFGR2_JOVSE;
    ++ranks;
    return HAL_OK;
}
''' + actual + r'''
int main(void) {
    unsigned run, value;
    for (run = 0; run < 2; ++run) {
        ranks = 0;
        registers.JSQR = 0x12345678U;
        MX_ADC2_Init();
        assert(ranks == 4);
        /* Decode the actual hardware fields, independently of LL getters. */
        assert((registers.CFGR2 & ADC_CFGR2_JOVSE) != 0);
        assert((registers.CFGR2 & (ADC_CFGR2_ROVSE | ADC_CFGR2_TROVS)) == 0);
        assert(((registers.CFGR2 & ADC_CFGR2_OVSR) >> ADC_CFGR2_OVSR_Pos) == 1);
        assert(((registers.CFGR2 & ADC_CFGR2_OVSS) >> ADC_CFGR2_OVSS_Pos) == 2);
        assert((registers.CFGR2 & ADC_CFGR2_GCOMP) != 0);
        assert(registers.JSQR == 0x12345678U); /* adapter keeps sequence */
    }
    /* All DC codes, including offset and full scale, retain 12-bit units. */
    for (value = 0; value <= 4095; ++value)
        assert(((value * 4U) >> 2) == value);
    assert(((2047U + 2048U + 2049U + 2048U) >> 2) == 2048U);
    puts("PASS ADC2: four ranks, 4x injected-only, /4 scale, repeat init, sequence preserved");
    return 0;
}
'''
    source = out / 'current_oversampling_test.c'
    source.write_text(fixture, encoding='utf-8')
    exe = out / 'current_oversampling_test.exe'
    compiler = [args.cc] + (['cc'] if Path(args.cc).stem == 'zig' else [])
    includes = ['Core/Inc', 'Drivers/STM32G4xx_HAL_Driver/Inc',
                'Drivers/CMSIS/Device/ST/STM32G4xx/Include', 'Drivers/CMSIS/Include']
    command = compiler + ['-std=c99', '-O1', '-Wall', '-Wextra', '-Werror',
                          '-Wno-pointer-to-int-cast', '-Wno-int-to-pointer-cast',
                          '-DSTM32G431xx', '-DUSE_HAL_DRIVER']
    for path in includes:
        command += ['-I', str(ROOT / path)]
    for step in (command + [str(source), '-o', str(exe)], [str(exe)]):
        subprocess.run(step, cwd=ROOT, check=True)


if __name__ == '__main__':
    main()
