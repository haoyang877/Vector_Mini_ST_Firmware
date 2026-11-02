"""用真实ADC换算与温度监督验证校准、滤波、原始过温和无效采样超时。"""

from _adc_fixture import adc_source
from _bsp_test_support import ROOT, arguments, run_c
from run_position_servo_tests import function_source


def main() -> None:
    """当前端口没有样本序号；超时测试只使用无效结果，不伪造帧新鲜度。"""
    args = arguments(__doc__)
    production = (ROOT / "firmware/motor/sensing/mcu_temperature_sensing.c").read_text(
        encoding="utf-8"
    )
    source = (
        adc_source(1)
        + r"""
#include <string.h>
#include "motor_sensing.h"
#include "application_config.h"
static volatile McuTemperatureTelemetry s_mcu_temperature;
static FocState foc;
static MotorControlState motor;
void MotorFault_Report(MotorControlState *state, MotorFaultCode fault) {
    if(state->fault_code==MOTOR_FAULT_NONE) state->fault_code=fault;
}
"""
        + function_source(production, "McuTemperature_Update")
        + function_source(production, "McuTemperature_IsFresh")
        + r"""
static void sample(unsigned ts,unsigned ref) {
    MCU_ADC_TEMPERATURE_RESULT=ts; MCU_ADC_REFERENCE_RESULT=ref;
    McuTemperature_Update(&foc,&motor);
}
static void reset(void) {
    memset((void*)&s_mcu_temperature,0,sizeof(s_mcu_temperature));
    memset(&foc,0,sizeof(foc)); memset(&motor,0,sizeof(motor));
}
int main(void) {
    reset();sample(1000,1500);assert(foc.temperature_c==30 && McuTemperature_IsFresh());
    assert(s_mcu_temperature.vdda_mv==3000 && s_mcu_temperature.sample_count==1);
    sample(1040,1500);assert(fabsf(foc.temperature_c-30.1f)<0.0001f);
    assert(s_mcu_temperature.raw_celsius==40);
    sample(1425,1875);assert(s_mcu_temperature.raw_celsius==65 && s_mcu_temperature.vdda_mv==2400);
    sample(1400,1500);assert(s_mcu_temperature.raw_celsius==130);
    assert(motor.fault_code==MOTOR_FAULT_OVERTEMPERATURE && foc.temperature_c<90);
    motor.fault_code=MOTOR_FAULT_ENCODER;sample(1400,1500);
    assert(motor.fault_code==MOTOR_FAULT_ENCODER);
    reset();sample(1000,0);assert(!McuTemperature_IsFresh() && isnan(foc.temperature_c));
    for(unsigned i=1;i<MCU_TEMPERATURE_TIMEOUT_TICKS-1;i++) sample(1000,0);
    assert(motor.fault_code==MOTOR_FAULT_NONE);
    sample(1000,0);assert(motor.fault_code==MOTOR_FAULT_TEMPERATURE_SENSOR);
    sample(1000,1500);assert(McuTemperature_IsFresh() && foc.temperature_c==30);
    assert(motor.fault_code==MOTOR_FAULT_TEMPERATURE_SENSOR);
    trim130=trim30;assert(isnan(BspAdc_ReadChannel(BSP_ADC_MCU_TEMPERATURE)));
    trim130=1400;reset();sample(760,1500);assert(foc.temperature_c==-30);
    puts("PASS temperature: factory trim, supply correction, filter, raw trip, invalid timeout, latched fault");
    return 0;
}
"""
    )
    run_c(args, "mcu_temperature", source, 1)


if __name__ == "__main__":
    main()
