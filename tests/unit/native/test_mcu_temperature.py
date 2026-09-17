"""Run the production MCU temperature conversion/supervisor; no hardware access."""
import argparse,subprocess,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[3]/'tools'))
from project_paths import ROOT,NATIVE_INCLUDE_FLAGS
from run_position_servo_tests import function_source

def main():
    p=argparse.ArgumentParser();p.add_argument('--cc',required=True);p.add_argument('--out',type=Path,required=True);a=p.parse_args()
    out=a.out.resolve();out.mkdir(parents=True,exist_ok=True);(out/'main.h').write_text('#pragma once\n#include <stdint.h>\n')
    production=(ROOT/'firmware/motor/foc/foc_sensing.c').read_text()
    src=r'''
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "foc_sensing.h"
#include "mcu_temperature.h"
static struct {uint32_t JDR1,JDR2,flags;} adc;
#define ADC1 (&adc)
#define ADC_FLAG_JEOS 1U
#define ADC_FLAG_JEOC 2U
#define __HAL_ADC_GET_FLAG(h,f) (adc.flags & (f))
#define __HAL_ADC_CLEAR_FLAG(h,f) (adc.flags &= ~(f))
static uint16_t cal30=1000,cal130=1400,vrefcal=1500;
#define TEMPSENSOR_CAL1_ADDR (&cal30)
#define TEMPSENSOR_CAL2_ADDR (&cal130)
#define VREFINT_CAL_ADDR (&vrefcal)
static unsigned starts;
static bool busy;
#define LL_ADC_INJ_IsConversionOngoing(p) busy
#define LL_ADC_INJ_StartConversion(p) (busy=true,++starts)
static MotorControl_TypeDef MotorControl;
static FOC_TypeDef foc;
volatile McuTemperatureTelemetry McuTemperature;
static void Set_ErrorNow(ErrorNow_TypeDef e) {MotorControl.ErrorNow=e;}
'''+function_source(production,'Temperature_Update')+r'''
static void fresh(uint32_t ts,uint32_t vr) {
 adc.JDR1=ts;adc.JDR2=vr;adc.flags=3;busy=false;Temperature_Update(&foc);
 assert(adc.flags==0 && busy);
}
static void reset(void) {
 memset((void*)&McuTemperature,0,sizeof(McuTemperature));memset(&foc,0,sizeof(foc));
 MotorControl.ErrorNow=No_Error;adc.flags=0;busy=false;starts=0;
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
 reset();Temperature_Update(&foc);assert(isnan(foc.temp) && starts==1 && !McuTemperature.valid);
 fresh(1000,1500);assert(foc.temp==30 && McuTemperature.valid && McuTemperature.sample_count==1);
 fresh(1040,1500);assert(fabsf(foc.temp-30.2f)<.0001f && McuTemperature.raw_celsius==40);
 /* A pending conversion is never restarted and stale samples expire exactly at 100 ms. */
 unsigned n=starts;
 for(unsigned i=0;i<99;i++) Temperature_Update(&foc);
 assert(McuTemperature.valid && starts==n && MotorControl.ErrorNow==No_Error);
 Temperature_Update(&foc);assert(!McuTemperature.valid && isnan(foc.temp) && MotorControl.ErrorNow==TemperatureSensor_Error);
 fresh(1000,1500);assert(McuTemperature.valid && foc.temp==30 && MotorControl.ErrorNow==TemperatureSensor_Error);
 reset();fresh(1239,1500);assert(MotorControl.ErrorNow==No_Error);
 fresh(1240,1500);assert(MotorControl.ErrorNow==High_Temprature && foc.temp<90); /* unfiltered trip */
 MotorControl.ErrorNow=Encoder_Error;fresh(1400,1500);assert(MotorControl.ErrorNow==Encoder_Error);
 reset();fresh(1000,1500);fresh(0,1500);assert(!McuTemperature.valid && isnan(foc.temp));
 for(unsigned i=1;i<100;i++)fresh(0,1500);
 assert(MotorControl.ErrorNow==TemperatureSensor_Error);
 reset();for(unsigned i=0;i<100;i++)Temperature_Update(&foc);
 assert(MotorControl.ErrorNow==TemperatureSensor_Error); /* startup with no ADC */
 puts("PASS MCU 30/130C calibration, supply correction, invalid trim/ADC, filtering, raw hot trip, stale/startup timeout and fault preservation");
 return 0;
}
'''
    f=out/'mcu_temperature.c';f.write_text(src);exe=out/'mcu_temperature.exe'
    compiler=[a.cc]+(['cc'] if Path(a.cc).stem=='zig' else [])
    subprocess.run(compiler+['-std=c99','-O2','-UNDEBUG','-Wall','-Wextra','-Werror','-I',str(out)]+NATIVE_INCLUDE_FLAGS+[str(f),'-lm','-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)

if __name__=='__main__':main()
