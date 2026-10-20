"""在真实单位下验证相电流换算、四分之一步长与无效偏置保护；不访问硬件。"""

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
    sensing = (ROOT / "firmware/motor/foc/foc_sensing.c").read_text(encoding="utf-8")
    src = (
        r"""
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "motor_types.h"
#include "foc_algorithm.h"
#include "motor_sensing.h"
#include "control_math.h"
static struct {uint32_t JDR1,JDR2,JDR3;} adc;

#define OVERCURRENT_CONFIRM_CYCLES 5U
#define CURRENT_OFFSET_MIN_COUNTS 1948.0f
#define CURRENT_OFFSET_MAX_COUNTS 2148.0f
static MotorControlState motor_state;
static FocState foc;
static void MotorFault_Report(MotorControlState *m, MotorFaultCode x) {m->fault_code=x;}
uint16_t motor_hw_current_sample_raw(MotorHwCurrentPhase phase) {
    switch(phase) {
        case MOTOR_HW_CURRENT_PHASE_A: return (uint16_t)adc.JDR1;
        case MOTOR_HW_CURRENT_PHASE_B: return (uint16_t)adc.JDR2;
        case MOTOR_HW_CURRENT_PHASE_C: return (uint16_t)adc.JDR3;
        default: return 0U;
    }
}
"""
        + function_source(sensing, "MotorCurrentOffsets_Valid")
        + function_source(sensing, "Current_Cal")
        + r"""
int main(void) {
#ifdef TEST_EXPECT_DEFAULT_6_MILLIOHM
    assert(CURRENT_SENSE_SHUNT_MILLIOHM==6U);
    assert(CURRENT_SENSE_PROFILE_SHUNT_RESISTANCE_OHM==0.006f);
#endif
    /* 本项专测换算精度；上电零点采样由test_startup_current_offsets独立覆盖。 */
    motor_state.device.phase_a_offset_counts=2032.625f; motor_state.device.phase_b_offset_counts=2051.25f;
    motor_state.device.phase_c_offset_counts=2036.5f;
    adc.JDR1=8131;adc.JDR2=8205;adc.JDR3=8146;
    Current_Cal(&foc,&motor_state);
    assert(motor_state.fault_code==MOTOR_FAULT_NONE);
    assert(foc.phase_b_current_a==0 && foc.phase_c_current_a==0);
    assert(fabsf(foc.phase_a_current_a+.125f*MOTOR_SENSING_CURRENT_A_PER_COUNT)<1e-6f);
    adc.JDR1+=1;Current_Cal(&foc,&motor_state);
    assert(fabsf(foc.phase_a_current_a+.375f*MOTOR_SENSING_CURRENT_A_PER_COUNT)<1e-6f);
    /* Actual 1 A input is not inadvertently rescaled by four. */
    adc.JDR2=(uint32_t)roundf(4*(motor_state.device.phase_b_offset_counts-1/MOTOR_SENSING_CURRENT_A_PER_COUNT));
    Current_Cal(&foc,&motor_state);assert(fabsf(foc.phase_b_current_a-1)<MOTOR_SENSING_CURRENT_A_PER_COUNT*.13f);
    motor_state.device.phase_c_offset_counts=NAN;Current_Cal(&foc,&motor_state);
    assert(motor_state.fault_code==MOTOR_FAULT_CURRENT_OFFSET);
    puts("PASS quarter-code current resolution, actual current scale, invalid-offset protection");
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
