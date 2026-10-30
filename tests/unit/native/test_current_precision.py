"""在真实单位下验证相电流换算与四分之一步长；不访问硬件。"""

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
    parser.add_argument("--out", type=Path, default=ROOT / "outputs/current_precision_tests")
    args = parser.parse_args()
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    (out / "main.h").write_text("#pragma once\n#include <stdint.h>\n")
    sensing = (ROOT / "firmware/motor/sensing/motor_current.c").read_text(encoding="utf-8")
    src = (
        r"""
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "motor_types.h"
#include "foc.h"
#include "board_config.h"
#include "application_config.h"
#include "bsp_adc.h"
#include "control_math.h"
static struct {uint32_t JDR1,JDR2,JDR3;} adc;

#define CURRENT_OFFSET_MIN_COUNTS 1948.0f
#define CURRENT_OFFSET_MAX_COUNTS 2148.0f
static MotorConfiguration configuration;
static MotorControlState motor_state;
static MotorMeasurement foc;
BspResult BspAdc_ReadFast(BspAdcFastSample *sample) {
    const uint32_t raw[3]={adc.JDR1,adc.JDR2,adc.JDR3};
    const float offsets[3]={configuration.device.phase_a_offset_counts,
        configuration.device.phase_b_offset_counts,configuration.device.phase_c_offset_counts};
    *sample=(BspAdcFastSample){.valid_mask=BSP_ADC_FAST_PHASE_CURRENT_VALID};
    for(unsigned i=0;i<3;i++) sample->phase_current_a[i]=-(raw[i]*.25f-offsets[i])*BSP_ADC_CURRENT_A_PER_COUNT;
    return BSP_OK;
}

"""
        + function_source(sensing, "MotorCurrentOffsets_IsValid")
        + function_source(sensing, "MotorCurrent_Update")
        + r"""
static void sample_current(void) {
    BspAdcFastSample sample;
    BspResult result=BspAdc_ReadFast(&sample);
    MotorCurrent_Update(&foc,result==BSP_OK?&sample:NULL);
}
int main(void) {
#ifdef TEST_EXPECT_DEFAULT_6_MILLIOHM
    assert(CURRENT_SENSE_SHUNT_MILLIOHM==6U);
    assert(CURRENT_SENSE_PROFILE_SHUNT_RESISTANCE_OHM==0.006f);
#endif
    assert(PARAM_HW_CURRENT_OFFSET_A_COUNTS==2048U);
    assert(PARAM_HW_CURRENT_OFFSET_B_COUNTS==2048U);
    assert(PARAM_HW_CURRENT_OFFSET_C_COUNTS==2048U);
    assert(MOTOR_SENSING_ADC_SAMPLE_COUNT*CURRENT_SENSE_PROFILE_ADC_MAX_COUNTS==16380U);
    /* 本项专测换算精度；上电零点采样由test_startup_current_offsets独立覆盖。 */
    configuration.device.phase_a_offset_counts=2032.625f; configuration.device.phase_b_offset_counts=2051.25f;
    configuration.device.phase_c_offset_counts=2036.5f;
    adc.JDR1=8131;adc.JDR2=8205;adc.JDR3=8146;
    sample_current();
    assert(motor_state.fault_code==MOTOR_FAULT_NONE);
    assert(foc.phase_b_current_a==0 && foc.phase_c_current_a==0);
    assert(fabsf(foc.phase_a_current_a+.125f*BSP_ADC_CURRENT_A_PER_COUNT)<1e-6f);
    adc.JDR1+=1;sample_current();
    assert(fabsf(foc.phase_a_current_a+.375f*BSP_ADC_CURRENT_A_PER_COUNT)<1e-6f);
    /* Actual 1 A input is not inadvertently rescaled by four. */
    adc.JDR2=(uint32_t)roundf(4*(configuration.device.phase_b_offset_counts-1/BSP_ADC_CURRENT_A_PER_COUNT));
    sample_current();assert(fabsf(foc.phase_b_current_a-1)<BSP_ADC_CURRENT_A_PER_COUNT*.13f);
    /* 缺帧只把三相置NAN，不再由采样模块报零偏故障。 */
    motor_state.fault_code=MOTOR_FAULT_NONE;
    MotorCurrent_Update(&foc,NULL);
    assert(motor_state.fault_code==MOTOR_FAULT_NONE);
    assert(isnan(foc.phase_a_current_a) && isnan(foc.phase_b_current_a) &&
           isnan(foc.phase_c_current_a));
    /* 零偏无效不再影响搬运：相电流按平台换算结果写入，且不报故障。 */
    configuration.device.phase_c_offset_counts=NAN;sample_current();
    assert(motor_state.fault_code==MOTOR_FAULT_NONE && isnan(foc.phase_c_current_a));
    puts("PASS quarter-code current resolution, actual current scale, NULL/NAN copy");
    return 0;
}
"""
    )
    fixture = out / "current_precision.c"
    fixture.write_text(src, encoding="utf-8")
    exe = out / "current_precision.exe"
    compiler = [args.cc] + (["cc"] if Path(args.cc).stem == "zig" else [])
    command = (
        compiler
        + [
            "-std=c99",
            "-O1",
            # zig cc 在优化构建下默认定义 NDEBUG；显式关闭，保证断言真实执行。
            "-UNDEBUG",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-Wno-unused-parameter",
            "-I",
            str(out),
        ]
        + NATIVE_INCLUDE_FLAGS
        + [str(fixture), str(ROOT / "firmware/motor/math/control_math.c"), "-lm", "-o", str(exe)]
    )
    for shunt in (None, 2, 6):
        profile_flags = (
            ["-DTEST_EXPECT_DEFAULT_6_MILLIOHM"]
            if shunt is None
            else [f"-DCURRENT_SENSE_SHUNT_MILLIOHM={shunt}"]
        )
        for step in (command + profile_flags, [str(exe)]):
            subprocess.run(step, check=True)


if __name__ == "__main__":
    main()
