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
#include "data_type.h"
#include "foc_algorithm.h"
#include "motor_sensing.h"
#include "utils.h"
static struct {uint32_t JDR1,JDR2,JDR3;} adc;

#define OVERCURRENT_CONFIRM_CYCLES 5U
#define CURRENT_OFFSET_MIN_COUNTS 1948.0f
#define CURRENT_OFFSET_MAX_COUNTS 2148.0f
static MotorControl_TypeDef MotorControl;
static FOC_TypeDef foc;
static void Set_ErrorNow(ErrorNow_TypeDef x) {MotorControl.ErrorNow=x;}
uint16_t motor_hw_current_sample_raw(MotorHwCurrentPhase phase) {
    switch(phase) {
        case MOTOR_HW_CURRENT_PHASE_A: return (uint16_t)adc.JDR1;
        case MOTOR_HW_CURRENT_PHASE_B: return (uint16_t)adc.JDR2;
        case MOTOR_HW_CURRENT_PHASE_C: return (uint16_t)adc.JDR3;
        default: return 0U;
    }
}
"""
        + function_source(sensing, "Current_Cal")
        + r"""
int main(void) {
    /* 零点偏置标定已从固件移除，本测试直接使用已验证的标定值。 */
    MotorControl.A_Offset=2032.625f; MotorControl.B_Offset=2051.25f;
    MotorControl.C_Offset=2036.5f;
    adc.JDR1=8131;adc.JDR2=8205;adc.JDR3=8146;
    Current_Cal(&foc,&MotorControl);
    assert(MotorControl.ErrorNow==No_Error);
    assert(foc.Ib==0 && foc.Ic==0);
    assert(fabsf(foc.Ia+.125f*MOTOR_SENSING_CURRENT_A_PER_COUNT)<1e-6f);
    adc.JDR1+=1;Current_Cal(&foc,&MotorControl);
    assert(fabsf(foc.Ia+.375f*MOTOR_SENSING_CURRENT_A_PER_COUNT)<1e-6f);
    /* Actual 1 A input is not inadvertently rescaled by four. */
    adc.JDR2=(uint32_t)roundf(4*(MotorControl.B_Offset-1/MOTOR_SENSING_CURRENT_A_PER_COUNT));
    Current_Cal(&foc,&MotorControl);assert(fabsf(foc.Ib-1)<MOTOR_SENSING_CURRENT_A_PER_COUNT*.13f);
    MotorControl.C_Offset=NAN;Current_Cal(&foc,&MotorControl);
    assert(MotorControl.ErrorNow==CurrentOffset_Error);
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
        + [str(fixture), str(ROOT / "firmware/common/utils.c"), "-lm", "-o", str(exe)]
    )
    for shunt in (2, 6):
        for step in (command + [f"-DCURRENT_SENSE_SHUNT_MILLIOHM={shunt}"], [str(exe)]):
            subprocess.run(step, check=True)


if __name__ == "__main__":
    main()
