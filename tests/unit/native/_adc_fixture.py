"""ADC换算夹具：只模拟寄存器、DMA缓冲和工厂校准字，执行原函数。"""

from _bsp_test_support import production, resources


def adc_source(board: int) -> str:
    """返回指定板卡公共ADC读数与偏置接口及其最小存储。"""
    setup = r"""
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include "bsp_adc.h"
static struct { uint32_t JDR1,JDR2,JDR3,JDR4,pdt1,pdt2,pdt3,pdt4; } adc1,adc2;
#define ADC1 (&adc1)
#define ADC2 (&adc2)
static volatile float s_phase_offsets[3] = {NAN,NAN,NAN};
static volatile float s_bus_offset = NAN;
"""
    if board == 1:
        setup += r"""
static uint16_t trim30=1000,trim130=1400,vrefcal=1500;
#define TEMPSENSOR_CAL1_ADDR (&trim30)
#define TEMPSENSOR_CAL2_ADDR (&trim130)
#define VREFINT_CAL_ADDR (&vrefcal)
"""
    else:
        setup += r"""
static struct { uint32_t dtcnt; } dma_aux,dma_internal;
#define DMA1_CHANNEL2 (&dma_aux)
#define DMA1_CHANNEL3 (&dma_internal)
#define ADC_AUX_RANKS (BSP_ENABLED_MOTOR_TEMPERATURE + 2U*BSP_ENABLED_OUTPUT_SINCOS)
static volatile uint16_t s_aux_dma[6],s_temperature_dma[4];
"""
    return (
        setup
        + resources(board)
        + production(
            board,
            "adc",
            "BspAdc_ReadRawChannel",
            "BspAdc_ReadChannel",
            "BspAdc_SetPhaseOffsets",
            "BspAdc_SetBusCurrentOffset",
        )
    )
