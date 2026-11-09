"""核对两板实际ADC原码域、量纲、独立偏置与不支持通道。"""

from _adc_fixture import adc_source
from _bsp_test_support import arguments, run_c


def main() -> None:
    """穷举四次累加域，参考公式不复用生产换算宏。"""
    args = arguments(__doc__)
    for board in (1, 2):
        source = (
            adc_source(board)
            + r"""
int main(void) {
    assert(isnan(BspAdc_ReadChannel(BSP_ADC_PHASE_CURRENT_A)));
    const float offsets[3]={2047.25f,2048.5f,2049.75f};
    BspAdc_SetPhaseOffsets(offsets);
    BspAdc_SetBusCurrentOffset(2010.5f);
    for(unsigned sum=0;sum<=16380;sum++) {
        MCU_ADC_PHASE_A_RESULT=sum; MCU_ADC_PHASE_B_RESULT=sum; MCU_ADC_PHASE_C_RESULT=sum;
        MCU_ADC_BUS_VOLTAGE_RESULT=BSP_BOARD==1 ? sum : sum/4U;
        float counts=(float)sum/4.0f;
        for(unsigned phase=0;phase<3;phase++) {
            assert(BspAdc_ReadRawChannel((BspAdcChannel)phase)==counts);
            float expected=-(counts-offsets[phase])*3.3f/4095.0f/
                          (BSP_BOARD==1 ? 0.060f : 0.080f);
            assert(fabsf(BspAdc_ReadChannel((BspAdcChannel)phase)-expected)<0.00002f);
        }
        float bus_counts=BSP_BOARD==1 ? counts : (float)(sum/4U);
        assert(BspAdc_ReadRawChannel(BSP_ADC_BUS_VOLTAGE)==bus_counts);
        float volts=bus_counts*3.3f/4095.0f*(BSP_BOARD==1 ? 11.0f : 103.3f/3.3f);
        assert(fabsf(BspAdc_ReadChannel(BSP_ADC_BUS_VOLTAGE)-volts)<0.00003f);
    }
#if BSP_BOARD==2
    for(unsigned raw=0;raw<=4095;raw++) {
        MCU_ADC_BUS_CURRENT_RESULT=raw;
        assert(BspAdc_ReadRawChannel(BSP_ADC_BUS_CURRENT)==(float)raw);
        float expected=((float)raw-2010.5f)*3.3f/4095.0f/0.040f;
        assert(fabsf(BspAdc_ReadChannel(BSP_ADC_BUS_CURRENT)-expected)<0.00002f);
    }
#else
    assert(isnan(BspAdc_ReadRawChannel(BSP_ADC_BUS_CURRENT)));
    assert(isnan(BspAdc_ReadChannel(BSP_ADC_MOTOR_TEMPERATURE)));
#endif
    assert(isnan(BspAdc_ReadRawChannel((BspAdcChannel)99)));
    assert(isnan(BspAdc_ReadChannel((BspAdcChannel)99)));
    puts("PASS ADC: full sum domain, independent phase/bus offsets, units and unavailable channels");
    return 0;
}
"""
        )
        run_c(args, f"adc_contract_{board}", source, board)


if __name__ == "__main__":
    main()
