"""离线执行实际扇区选择、相电流重构及过流路径，不访问硬件。"""

import argparse
from pathlib import Path
import subprocess
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'tools'))
from project_paths import ROOT, NATIVE_INCLUDE_FLAGS
from run_position_servo_tests import function_source


PRELUDE = r'''
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "foc_sensing.h"
#include "foc_errhandle.h"
#include "motor_hw.h"
#include "hw_conf.h"
#include "utils.h"
static TIM_TypeDef timer;
static ADC_TypeDef adc;
static GPIO_TypeDef gpioa, gpiob;
#undef GPIOA
#undef GPIOB
#define GPIOA (&gpioa)
#define GPIOB (&gpiob)
#undef TIM1
#define TIM1 (&timer)
#undef ADC2
#define ADC2 (&adc)
#define fast_abs fabsf
static ErrorNow_TypeDef last_error;
void Set_ErrorNow(ErrorNow_TypeDef error) { last_error = error; }
'''

CASES = r'''
static const uint32_t enabled = TIM_CCER_CC1E | TIM_CCER_CC1NE |
    TIM_CCER_CC2E | TIM_CCER_CC2NE | TIM_CCER_CC3E | TIM_CCER_CC3NE;
static void pwm(unsigned a, unsigned b, unsigned c) {
    timer.CCER = enabled; timer.BDTR = TIM_BDTR_MOE; timer.ARR = 4250;
    timer.CCR1 = a; timer.CCR2 = b; timer.CCR3 = c;
}
static void near(float actual, float expected) {
    assert(fabsf(actual - expected) < 2e-5f);
}
static void set_raw(int a, int b, int c) {
    adc.JDR1 = (uint32_t)(2048 - a);
    adc.JDR2 = (uint32_t)(2048 - b);
    adc.JDR3 = (uint32_t)(2048 - c);
}
static void clear_current(FOC_TypeDef *f, MotorControl_TypeDef *m) {
    set_raw(0, 0, 0); Current_Cal(f, m); last_error = No_Error;
}
int main(void) {
    unsigned n, s, radius, direction, bit;
    unsigned vectors = 0;
    FOC_TypeDef f = {0};
    MotorControl_TypeDef m = {0};
    const unsigned duty[6][3] = {
        {400,2100,3800}, {2100,400,3800}, {3800,400,2100},
        {3800,2100,400}, {2100,3800,400}, {400,3800,2100}
    };
    const uint32_t enable_bits[] = {TIM_CCER_CC1E, TIM_CCER_CC1NE,
        TIM_CCER_CC2E, TIM_CCER_CC2NE, TIM_CCER_CC3E, TIM_CCER_CC3NE};
    /* 真实关相端口：每路 PWM/非高阻 GPIO 都必须拒绝切换。 */
    gpioa.MODER=(3U<<14)|(3U<<16)|(3U<<18)|(3U<<20); gpiob.MODER=15;
    timer.CCER=0x1000; timer.BDTR=TIM_BDTR_MOE;
    assert(motor_hw_phase_outputs_disabled());
    for(bit=0;bit<6;bit++) {
        timer.CCER=0x1000|enable_bits[bit]; assert(!motor_hw_phase_outputs_disabled());
    }
    timer.CCER=0x1000;
    for(bit=7;bit<=10;bit++) {
        gpioa.MODER^=1U<<(2*bit); assert(!motor_hw_phase_outputs_disabled());
        gpioa.MODER^=1U<<(2*bit);
    }
    for(bit=0;bit<2;bit++) {
        gpiob.MODER^=1U<<(2*bit); assert(!motor_hw_phase_outputs_disabled());
        gpiob.MODER^=1U<<(2*bit);
    }
    assert(motor_hw_phase_outputs_disabled());
    m.A_Offset = m.B_Offset = m.C_Offset = 2048;
    m.ModeNow = Speed_Mode;

    /* 六扇区分别污染被舍弃相：另外两相必须原样保留。 */
    for (s = 1; s <= 6; ++s) {
        pwm(duty[s-1][0], duty[s-1][1], duty[s-1][2]);
        assert(motor_hw_current_sample_sector() == s);
        clear_current(&f, &m);
        f.sector = (int32_t)(s % 6 + 1); /* 故意使用错误的软件扇区。 */
        set_raw(s == 3 || s == 4 ? 1000 : 100,
                s == 5 || s == 6 ? 1000 : -30,
                s == 1 || s == 2 ? 1000 : -70);
        Current_Cal(&f, &m);
        near(f.Ia, 100 * SENSING_CURR_FACTOR);
        near(f.Ib, -30 * SENSING_CURR_FACTOR);
        near(f.Ic, -70 * SENSING_CURR_FACTOR);
    }

    /* 实际 SVM 全角度扫描，含反方向；重构相必须是最大 CCR 相。 */
    for (direction = 0; direction < 2; ++direction) {
        for (radius = 1; radius <= 3; ++radius) {
            for (n = 0; n < 3600; ++n) {
                float angle = (direction ? -1.0f : 1.0f) * n * 6.283185307f / 3600;
                float amplitude = radius * (0.95f * SQRT_3_BY_2 / 3);
                float a, b, c;
                int32_t sector;
                TIM_TypeDef before;
                SVM_SectorJudge(amplitude*cosf(angle), amplitude*sinf(angle),
                                &a, &b, &c, &sector);
                pwm((unsigned)(a*4250), (unsigned)(b*4250), (unsigned)(c*4250));
                memcpy(&before, &timer, sizeof(timer));
                s = motor_hw_current_sample_sector();
                assert(memcmp(&before, &timer, sizeof(timer)) == 0);
                f.Ia = 1; f.Ib = 2; f.Ic = 4;
                Current_Reconstruct(&f, s);
                if (f.Ia != 1) {
                    assert(timer.CCR1 >= timer.CCR2 && timer.CCR1 >= timer.CCR3);
                    near(f.Ia, -6); near(f.Ib, 2); near(f.Ic, 4);
                } else if (f.Ib != 2) {
                    assert(timer.CCR2 >= timer.CCR1 && timer.CCR2 >= timer.CCR3);
                    near(f.Ia, 1); near(f.Ib, -5); near(f.Ic, 4);
                } else {
                    assert(timer.CCR3 >= timer.CCR1 && timer.CCR3 >= timer.CCR2);
                    near(f.Ia, 1); near(f.Ib, 2); near(f.Ic, -3);
                }
                near(f.Ia + f.Ib + f.Ic, 0);
                ++vectors;
            }
        }
    }

    /* 并列时舍弃较晚相；零矢量、关断、非法 CCR 不重构。 */
    pwm(3000,3000,500); assert(motor_hw_current_sample_sector() == 5);
    pwm(3000,500,3000); assert(motor_hw_current_sample_sector() == 2);
    pwm(500,3000,3000); assert(motor_hw_current_sample_sector() == 1);
    for (n = 0; n <= 4250; n += 2125) {
        pwm(n,n,n); assert(motor_hw_current_sample_sector() == 0);
    }
    for (bit = 0; bit < 6; ++bit) {
        pwm(400,2100,3800); timer.CCER &= ~enable_bits[bit];
        assert(motor_hw_current_sample_sector() == 0);
        set_raw(100,-30,20); Current_Cal(&f,&m);
        near(f.Ic, 20 * SENSING_CURR_FACTOR);
    }
    pwm(400,2100,3800); timer.BDTR = 0;
    assert(motor_hw_current_sample_sector() == 0);
    pwm(400,2100,3800); timer.ARR = 0;
    assert(motor_hw_current_sample_sector() == 0);
    pwm(4251,2100,3800); assert(motor_hw_current_sample_sector() == 0);
    pwm(400,4251,3800); assert(motor_hw_current_sample_sector() == 0);
    pwm(400,2100,4251); assert(motor_hw_current_sample_sector() == 0);
    f.Ia = 1; f.Ib = 2; f.Ic = 4;
    Current_Reconstruct(&f,0); Current_Reconstruct(&f,7);
    near(f.Ia,1); near(f.Ib,2); near(f.Ic,4);

    /* 零偏校准不重构，重新使能没有上一次扇区状态。 */
    pwm(400,2100,3800); clear_current(&f,&m);
    m.ModeNow = Calib_CurrentOffset; set_raw(100,-30,20); Current_Cal(&f,&m);
    near(f.Ic,20 * SENSING_CURR_FACTOR);
    m.ModeNow = Speed_Mode; Current_Cal(&f,&m);
    near(f.Ic,-70 * SENSING_CURR_FACTOR);
    clear_current(&f,&m); m.A_Offset = 1000;
    f.Ia = 1; f.Ib = 2; f.Ic = 4; Current_Cal(&f,&m);
    assert(last_error == CurrentOffset_Error);
    near(f.Ia,1); near(f.Ib,2); near(f.Ic,4);
    m.A_Offset = 2048;

    /* 原始异常相即使被重构成零，仍按原来的五拍确认过流。 */
    clear_current(&f,&m); set_raw(0,0,1500);
    for (n = 0; n < 4; ++n) { Current_Cal(&f,&m); assert(last_error == No_Error); }
    Current_Cal(&f,&m); assert(last_error == Over_Current); near(f.Ic,0);
    /* 两个测量值各自未超限，但重构值超限也必须触发。 */
    clear_current(&f,&m); set_raw(900,900,0);
    for (n = 0; n < 4; ++n) { Current_Cal(&f,&m); assert(last_error == No_Error); }
    Current_Cal(&f,&m); assert(last_error == Over_Current);
    assert(fabsf(f.Ic) > CURRENT_OVERCURRENT_TRIP_A);
    /* 正常一拍会清除连续计数。 */
    clear_current(&f,&m); set_raw(0,0,1500);
    for (n = 0; n < 4; ++n) { Current_Cal(&f,&m); }
    clear_current(&f,&m); set_raw(0,0,1500);
    Current_Cal(&f,&m); assert(last_error == No_Error);
    printf("PASS: %u SVM vectors, six sectors, ties, disable, calibration, raw/reconstructed overcurrent\n", vectors);
    return 0;
}
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cc', required=True)
    parser.add_argument('--out', type=Path, default=ROOT / 'outputs/current_reconstruction/host')
    args = parser.parse_args()
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    sensing = (ROOT / 'firmware/motor/foc/foc_sensing.c').read_text(encoding='utf-8')
    platform = (ROOT / 'firmware/platform/stm32g4/ports/motor/motor_current_stm32g4.c').read_text(encoding='utf-8')
    algorithm = (ROOT / 'firmware/motor/foc/foc_algorithm.c').read_text(encoding='utf-8')
    task = (ROOT / 'firmware/app/foc_task.c').read_text(encoding='utf-8')
    irq = function_source(task, 'FOC20kHzIRQHandler')
    if irq.index('Current_Cal(') > irq.index('Fluxobserver_Update('):
        raise RuntimeError('电流重构必须先于观测器和当拍 PWM 计算')
    fixture = PRELUDE + '\n'.join([
        function_source(platform, 'motor_hw_current_sample_sector'),
        function_source(platform, 'motor_hw_phase_outputs_disabled'),
        function_source(sensing, 'Current_Reconstruct'),
        next(line for line in sensing.splitlines()
             if line.startswith('#define OVERCURRENT_CONFIRM_CYCLES ')),
        function_source(sensing, 'Current_Cal'),
        function_source(algorithm, 'SVM_SectorJudge'),
    ]) + CASES
    source = out / 'current_reconstruction_test.c'
    source.write_text(fixture, encoding='utf-8')
    compiler = [args.cc] + (['cc'] if Path(args.cc).stem == 'zig' else [])
    command = compiler + NATIVE_INCLUDE_FLAGS + [
        '-std=c99', '-O1', '-Wall', '-Wextra', '-Werror',
        '-Wno-pointer-to-int-cast', '-Wno-int-to-pointer-cast',
        '-DSTM32G431xx', '-DUSE_HAL_DRIVER',
    ]
    for path in ('Core/Inc', 'Drivers/STM32G4xx_HAL_Driver/Inc',
                 'Drivers/CMSIS/Device/ST/STM32G4xx/Include', 'Drivers/CMSIS/Include'):
        command += ['-I', str(ROOT / 'firmware/platform/stm32g4/cubemx' / path)]
    for shunt in (2, 6):
        exe = out / f'current_reconstruction_{shunt}mohm.exe'
        subprocess.run(command + [f'-DCURRENT_SENSE_SHUNT_MILLIOHM={shunt}U',
                                 str(source), '-o', str(exe)], cwd=ROOT, check=True)
        subprocess.run([str(exe)], cwd=ROOT, check=True)


if __name__ == '__main__':
    main()
