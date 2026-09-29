"""离线编译并运行生产 MCU 温度换算与监督任务；不访问硬件。"""

import sys as _sys
from pathlib import Path as _Path

_sys.path.insert(0, str(_Path(__file__).resolve().parents[3] / "tools"))
import argparse
import subprocess
from pathlib import Path

from project_paths import NATIVE_INCLUDE_FLAGS, ROOT
from run_position_servo_tests import function_source


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--cc", required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    (out / "main.h").write_text("#pragma once\n#include <stdint.h>\n")
    production = (ROOT / "firmware/motor/foc/foc_sensing.c").read_text(encoding="utf-8")
    src = (
        r"""
#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "foc_sensing.h"
#include "mcu_temperature.h"
#include "motor_sensing.h"
static struct {uint32_t JDR1,JDR2,flags;} adc;
#define ADC_FLAG_JEOS 1U
#define ADC_FLAG_JEOC 2U
static uint16_t cal30=1000,cal130=1400,vrefcal=1500;
static unsigned starts;
static bool busy;
static MotorControlState motor_state;
static FocState foc;
volatile McuTemperatureTelemetry s_mcu_temperature;
static void MotorFault_Report(MotorControlState *m, MotorFaultCode e) {m->fault_code=e;}
void motor_hw_temperature_poll(MotorHwTemperaturePoll *result) {
    result->sample_ready=false; result->conversion_ok=false;
    if(adc.flags & ADC_FLAG_JEOS) {
        result->raw_ts=adc.JDR1; result->raw_vref=adc.JDR2;
        adc.flags &= ~(ADC_FLAG_JEOS | ADC_FLAG_JEOC);
        result->sample_ready=true;
        result->conversion_ok=McuTemperature_Convert(result->raw_ts,result->raw_vref,cal30,cal130,
                                                     vrefcal,&result->celsius,&result->vdda_mv);
    }
    if(!busy) {busy=true;++starts;}
}
"""
        + function_source(production, "Temperature_Update")
        + r"""
static void fresh(uint32_t ts,uint32_t vr) {
 adc.JDR1=ts;adc.JDR2=vr;adc.flags=3;busy=false;Temperature_Update(&foc,&motor_state);
 assert(adc.flags==0 && busy);
}
static void reset(void) {
 memset((void*)&s_mcu_temperature,0,sizeof(s_mcu_temperature));memset(&foc,0,sizeof(foc));
 motor_state.fault_code=MOTOR_FAULT_NONE;adc.flags=0;busy=false;starts=0;
}
int main(void) {
 float c,v;
 assert(McuTemperature_Convert(1000,1500,1000,1400,1500,&c,&v) && c==30 && v==3000);
 assert(McuTemperature_Convert(1400,1500,1000,1400,1500,&c,&v) && c==130);
 assert(McuTemperature_Convert(1000,1500,1100,1540,1650,&c,&v) && c==30 && v==3300);
 assert(McuTemperature_Convert(1425,1875,1000,1400,1500,&c,&v) && c==65 && v==2400);
 assert(McuTemperature_Convert(760,1500,1000,1400,1500,&c,&v) && c==-30);
 assert(!McuTemperature_Convert(0,1500,1000,1400,1500,&c,&v));
 assert(!McuTemperature_Convert(1000,0,1000,1400,1500,&c,&v));
 assert(!McuTemperature_Convert(4095,1500,1000,1400,1500,&c,&v));
 assert(!McuTemperature_Convert(1000,1500,1000,1000,1500,&c,&v));
 assert(!McuTemperature_Convert(1000,1500,0,65535,1500,&c,&v));
 assert(!McuTemperature_Convert(1000,100,1000,1400,1500,&c,&v));
 reset();Temperature_Update(&foc,&motor_state);assert(isnan(foc.temperature_c) && starts==1 && !s_mcu_temperature.valid);
 fresh(1000,1500);assert(foc.temperature_c==30 && s_mcu_temperature.valid && s_mcu_temperature.sample_count==1);
 fresh(1040,1500);assert(fabsf(foc.temperature_c-30.1f)<.0001f && s_mcu_temperature.raw_celsius==40);
 /* A pending conversion is never restarted and stale samples expire exactly at 100 ms
    (200 supervision ticks at 2 kHz). */
 unsigned n=starts;
 for(unsigned i=0;i<199;i++) Temperature_Update(&foc,&motor_state);
 assert(s_mcu_temperature.valid && starts==n && motor_state.fault_code==MOTOR_FAULT_NONE);
 Temperature_Update(&foc,&motor_state);assert(!s_mcu_temperature.valid && isnan(foc.temperature_c) && motor_state.fault_code==MOTOR_FAULT_TEMPERATURE_SENSOR);
 fresh(1000,1500);assert(s_mcu_temperature.valid && foc.temperature_c==30 && motor_state.fault_code==MOTOR_FAULT_TEMPERATURE_SENSOR);
 reset();fresh(1239,1500);assert(motor_state.fault_code==MOTOR_FAULT_NONE);
 fresh(1240,1500);assert(motor_state.fault_code==MOTOR_FAULT_OVERTEMPERATURE && foc.temperature_c<90); /* unfiltered trip */
 motor_state.fault_code=MOTOR_FAULT_ENCODER;fresh(1400,1500);assert(motor_state.fault_code==MOTOR_FAULT_ENCODER);
 reset();fresh(1000,1500);fresh(0,1500);assert(!s_mcu_temperature.valid && isnan(foc.temperature_c));
 for(unsigned i=1;i<200;i++)fresh(0,1500);
 assert(motor_state.fault_code==MOTOR_FAULT_TEMPERATURE_SENSOR);
 reset();for(unsigned i=0;i<200;i++)Temperature_Update(&foc,&motor_state);
 assert(motor_state.fault_code==MOTOR_FAULT_TEMPERATURE_SENSOR); /* startup with no ADC */
 puts("PASS MCU 30/130C calibration, supply correction, invalid trim/ADC, filtering, raw hot trip, stale/startup timeout and fault preservation");
 return 0;
}
"""
    )
    fixture = out / "mcu_temperature.c"
    fixture.write_text(src, encoding="utf-8")
    exe = out / "mcu_temperature.exe"
    compiler = [args.cc] + (["cc"] if Path(args.cc).stem == "zig" else [])
    subprocess.run(
        compiler
        + ["-std=c99", "-O2", "-UNDEBUG", "-Wall", "-Wextra", "-Werror", "-I", str(out)]
        + NATIVE_INCLUDE_FLAGS
        + [str(fixture), "-lm", "-o", str(exe)],
        check=True,
    )
    subprocess.run([str(exe)], check=True)


if __name__ == "__main__":
    main()
