"""Run the actual zero-offset accumulation, completion and cancellation code offline."""
import argparse
from pathlib import Path
import subprocess

from run_position_servo_tests import ROOT, NATIVE_INCLUDE_FLAGS, function_source


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cc', required=True)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    (args.out / 'main.h').write_text('#include <stdint.h>\n#include <stddef.h>\n', encoding='utf-8')
    source = (ROOT / 'firmware/motor/identification/foc_calibration.c').read_text(encoding='utf-8')
    fixture = r'''
#include <assert.h>
#include <stdio.h>
#include "foc_calibration.h"
#include "hw_conf.h"
static MotorControl_TypeDef MotorControl;
static FOC_TypeDef foc;
static struct { uint16_t a,b,c; } adc;
#undef CURRENT_ADC
#undef IA_ADC_CHANNEL
#undef IB_ADC_CHANNEL
#undef IC_ADC_CHANNEL
#define CURRENT_ADC (&adc)
#define IA_ADC_CHANNEL a
#define IB_ADC_CHANNEL b
#define IC_ADC_CHANNEL c
static uint32_t current_offset_sample_count;
static uint32_t current_offset_sum_a,current_offset_sum_b,current_offset_sum_c;
static void Set_ModeNow(ModeNow_TypeDef mode) { MotorControl.ModeNow=mode; }
static void Set_ErrorNow(ErrorNow_TypeDef error) { MotorControl.ErrorNow=error; }
''' + '\n'.join(function_source(source, name) for name in (
        'CurrentOffsetCalibration_Reset', 'Task_Calib_CurrentOffset')) + r'''
static void begin(uint16_t a,uint16_t b,uint16_t c) {
    CurrentOffsetCalibration_Reset();
    MotorControl.ModeNow=Calib_CurrentOffset; MotorControl.ErrorNow=No_Error;
    MotorControl.A_Offset=2048; MotorControl.B_Offset=2049; MotorControl.C_Offset=2050;
    adc.a=a; adc.b=b; adc.c=c;
}
static void sample(void) { Task_Calib_CurrentOffset(&foc,&MotorControl); }
int main(void) {
    assert(FOC_FREQ==20000U);
    begin(2001,2061,2020);
    for(unsigned i=0;i<FOC_FREQ-1U;++i) sample();
    assert(MotorControl.ModeNow==Calib_CurrentOffset && MotorControl.A_Offset==2048);
    assert(MotorControl.B_Offset==2049 && MotorControl.C_Offset==2050);
    sample();
    assert(MotorControl.ModeNow==Motor_Disable && MotorControl.ErrorNow==No_Error);
    assert(MotorControl.A_Offset==2001 && MotorControl.B_Offset==2061 && MotorControl.C_Offset==2020);
    assert(current_offset_sample_count==0U);
    begin(1990,1991,1992);
    for(unsigned i=0;i<FOC_FREQ/2U;++i) sample();
    CurrentOffsetCalibration_Reset(); adc.a=2101;adc.b=2102;adc.c=2103;
    for(unsigned i=0;i<FOC_FREQ;++i) sample();
    assert(MotorControl.A_Offset==2101 && MotorControl.B_Offset==2102 && MotorControl.C_Offset==2103);
    begin(0,2048,2048);
    for(unsigned i=0;i<FOC_FREQ;++i) sample();
    assert(MotorControl.ModeNow==Motor_Disable && MotorControl.ErrorNow==CurrentOffset_Error);
    assert(MotorControl.A_Offset==2048 && MotorControl.B_Offset==2049 && MotorControl.C_Offset==2050);
    begin(1948,2148,2048);
    for(unsigned i=0;i<FOC_FREQ;++i) sample();
    assert(MotorControl.ErrorNow==No_Error && MotorControl.A_Offset==1948 && MotorControl.B_Offset==2148);
    puts("PASS actual current-offset task: full 1s accumulation, RAM-only commit then STOP, abort/reset, invalid sample rollback");
    return 0;
}
'''
    c = args.out / 'current_offset_startup.c'
    c.write_text(fixture, encoding='utf-8')
    exe = args.out / 'current_offset_startup.exe'
    cc = [args.cc] + (['cc'] if Path(args.cc).stem == 'zig' else [])
    command = cc + ['-std=c99', '-O1', '-Wall', '-Wextra', '-Werror', '-I', str(args.out)]
    command += NATIVE_INCLUDE_FLAGS + [str(c), '-o', str(exe)]
    for step in (command, [str(exe)]):
        subprocess.run(step, cwd=ROOT, check=True)


if __name__ == '__main__':
    main()
