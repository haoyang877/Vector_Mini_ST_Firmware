"""离线执行 ADC1 初始化、MCU 温度换算和过温路径，不访问硬件。"""

import argparse
from pathlib import Path
import subprocess
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'tools'))
from project_paths import ROOT, NATIVE_INCLUDE_FLAGS
from run_position_servo_tests import function_source


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cc', required=True)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    adc = (ROOT / 'firmware/platform/stm32g4/cubemx/Core/Src/adc.c').read_text(encoding='utf-8')
    sensing = (ROOT / 'firmware/motor/foc/foc_sensing.c').read_text(encoding='utf-8')
    fixture = r'''
#undef NDEBUG
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "adc.h"
#include "stm32g4xx_ll_adc.h"
static ADC_TypeDef registers;
ADC_HandleTypeDef hadc1;
#undef ADC1
#define ADC1 (&registers)
#undef __HAL_ADC_CLEAR_FLAG
#define __HAL_ADC_CLEAR_FLAG(h,f) ((h)->Instance->ISR &= ~(f))
static uint16_t cal30=1000, cal130=1400, cal_vref=1500;
#undef TEMPSENSOR_CAL1_ADDR
#undef TEMPSENSOR_CAL2_ADDR
#undef VREFINT_CAL_ADDR
#define TEMPSENSOR_CAL1_ADDR (&cal30)
#define TEMPSENSOR_CAL2_ADDR (&cal130)
#define VREFINT_CAL_ADDR (&cal_vref)
static unsigned injection_calls;
void Error_Handler(void) { abort(); }
HAL_StatusTypeDef HAL_ADC_Init(ADC_HandleTypeDef *h) {
    assert(h == &hadc1 && h->Instance == ADC1);
    assert(h->Init.ClockPrescaler == ADC_CLOCK_SYNC_PCLK_DIV4);
    assert(h->Init.Resolution == ADC_RESOLUTION_12B);
    assert(h->Init.OversamplingMode == DISABLE);
    return HAL_OK;
}
HAL_StatusTypeDef HAL_ADCEx_MultiModeConfigChannel(ADC_HandleTypeDef *h,
        const ADC_MultiModeTypeDef *c) {
    assert(h == &hadc1 && c->Mode == ADC_MODE_INDEPENDENT); return HAL_OK;
}
HAL_StatusTypeDef HAL_ADC_ConfigChannel(ADC_HandleTypeDef *h, const ADC_ChannelConfTypeDef *c) {
    assert(h == &hadc1 && c->Channel == ADC_CHANNEL_4); return HAL_OK;
}
HAL_StatusTypeDef HAL_ADCEx_InjectedConfigChannel(ADC_HandleTypeDef *h,
        const ADC_InjectionConfTypeDef *c) {
    assert(h == &hadc1 && injection_calls < 3);
    assert(c->ExternalTrigInjecConv == ADC_EXTERNALTRIGINJEC_T1_CC4);
    assert(c->ExternalTrigInjecConvEdge == ADC_EXTERNALTRIGINJECCONV_EDGE_FALLING);
    assert(c->InjectedSingleDiff == ADC_SINGLE_ENDED);
    assert(c->InjectedOffsetNumber == ADC_OFFSET_NONE);
    assert(c->QueueInjectedContext == DISABLE && c->AutoInjectedConv == DISABLE);
    assert(c->InjectedDiscontinuousConvMode == DISABLE);
    assert(c->InjecOversamplingMode == DISABLE);
    if (injection_calls == 0) {
        assert(c->InjectedChannel == ADC_CHANNEL_4 && c->InjectedNbrOfConversion == 1);
    } else {
        /* HAL 仅在 scan 缓存使能时才组装多通道注入序列。 */
        assert(h->Init.ScanConvMode == ADC_SCAN_ENABLE);
        assert(c->InjectedNbrOfConversion == 2);
        assert(c->InjectedSamplingTime == ADC_SAMPLETIME_640CYCLES_5);
        assert(c->InjectedChannel == (injection_calls == 1 ?
            ADC_CHANNEL_TEMPSENSOR_ADC1 : ADC_CHANNEL_VREFINT));
        assert(c->InjectedRank == (injection_calls == 1 ?
            ADC_INJECTED_RANK_1 : ADC_INJECTED_RANK_2));
    }
    injection_calls++; return HAL_OK;
}
typedef struct { float temp; } FOC_TypeDef;
enum { High_Temprature = 1 };
static unsigned errors;
void Set_ErrorNow(unsigned error) { assert(error == High_Temprature); errors++; }
''' + '\n'.join(function_source(adc, name) for name in (
        'MX_ADC1_Init', 'mcu_temperature_from_adc', 'motor_hw_mcu_temperature_c')) + '\n' + \
        function_source(sensing, 'Temperature_Update') + r'''
static void near(float actual, float expected) {
    assert(isfinite(actual) && fabsf(actual - expected) < 0.001f);
}
static void sample(uint32_t temperature, uint32_t vref) {
    registers.JDR1 = temperature; registers.JDR2 = vref;
    registers.ISR = ADC_FLAG_JEOS | ADC_FLAG_JEOC;
}
int main(void) {
    unsigned run, i;
    const uint32_t bad[] = {0,4095,65535};
    FOC_TypeDef f;
    for (run=0;run<2;run++) {
        injection_calls=0; MX_ADC1_Init(); assert(injection_calls == 3);
    }
    /* 双通道采样加转换在 PWM 周期内，且采样分别大于 5/4 us。 */
    assert(640.5/42.5 > 5 && 2*(640.5+12.5)/42.5 < 50);
    near(mcu_temperature_from_adc(1000,1500,1000,1400,1500),30);
    near(mcu_temperature_from_adc(1200,1500,1000,1400,1500),80);
    near(mcu_temperature_from_adc(1400,1500,1000,1400,1500),130);
    near(mcu_temperature_from_adc(1202,1500,1000,1400,1500),80.5);
    near(mcu_temperature_from_adc(1080,1350,1000,1400,1500),80);
    near(mcu_temperature_from_adc(900,1500,1000,1400,1500),5);
    for(i=0;i<3;i++) {
        assert(isnan(mcu_temperature_from_adc(bad[i],1500,1000,1400,1500)));
        assert(isnan(mcu_temperature_from_adc(1200,bad[i],1000,1400,1500)));
        assert(isnan(mcu_temperature_from_adc(1200,1500,bad[i],1400,1500)));
        assert(isnan(mcu_temperature_from_adc(1200,1500,1000,bad[i],1500)));
        assert(isnan(mcu_temperature_from_adc(1200,1500,1000,1400,bad[i])));
    }
    assert(isnan(mcu_temperature_from_adc(1200,1500,1000,1000,1500)));
    assert(isnan(mcu_temperature_from_adc(1200,500,1000,1400,1500)));
    assert(isnan(mcu_temperature_from_adc(1200,3000,1000,1400,1500)));
    registers.ISR=0; assert(isnan(motor_hw_mcu_temperature_c()));
    sample(1200,1500); near(motor_hw_mcu_temperature_c(),80);
    assert(registers.ISR == ADC_FLAG_JEOC);
    assert(isnan(motor_hw_mcu_temperature_c()));
    sample(1279,1500); Temperature_Update(&f); near(f.temp,99.75); assert(errors==0);
    sample(1280,1500); Temperature_Update(&f); near(f.temp,100); assert(errors==1);
    sample(1400,1500); Temperature_Update(&f); near(f.temp,130); assert(errors==2);
    sample(0,1500); Temperature_Update(&f); assert(isnan(f.temp) && errors==2);
    puts("PASS ADC1 sequence, MCU calibration 30/130 C, VDDA, invalid data, freshness, overtemperature");
    return 0;
}
'''
    source = out / 'mcu_temperature_test.c'
    source.write_text(fixture, encoding='utf-8')
    exe = out / 'mcu_temperature_test.exe'
    compiler = [args.cc] + (['cc'] if Path(args.cc).stem == 'zig' else []) + NATIVE_INCLUDE_FLAGS
    includes = ('Core/Inc', 'Drivers/STM32G4xx_HAL_Driver/Inc',
                'Drivers/CMSIS/Device/ST/STM32G4xx/Include', 'Drivers/CMSIS/Include')
    command = compiler + ['-std=c99', '-O1', '-Wall', '-Wextra', '-Werror',
                         '-Wno-pointer-to-int-cast', '-Wno-int-to-pointer-cast',
                         '-DSTM32G431xx', '-DUSE_HAL_DRIVER']
    for path in includes:
        command += ['-I', str(ROOT / 'firmware/platform/stm32g4/cubemx' / path)]
    subprocess.run(command + [str(source), '-o', str(exe)], cwd=ROOT, check=True)
    subprocess.run([str(exe)], cwd=ROOT, check=True)


if __name__ == '__main__':
    main()
