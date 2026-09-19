"""在真实单位下验证上电电流零偏标定与电流换算；不访问硬件。"""

import argparse
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tools"))
from project_paths import NATIVE_INCLUDE_FLAGS, ROOT
from run_position_servo_tests import function_source


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--cc", required=True)
    p.add_argument("--out", type=Path, default=ROOT / "outputs/current_precision_tests")
    a = p.parse_args()
    out = a.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    (out / "main.h").write_text("#pragma once\n#include <stdint.h>\n")
    sensing = (ROOT / "firmware/motor/foc/foc_sensing.c").read_text(encoding="utf-8")
    calibration = (ROOT / "firmware/motor/identification/foc_calibration.c").read_text(
        encoding="utf-8"
    )
    src = (
        r"""
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "data_type.h"
#include "foc_algorithm.h"
#include "motor_work.h"
#include "hw_conf.h"
#include "utils.h"
static struct {uint32_t JDR1,JDR2,JDR3,JDR4;} adc;

#define ADC2 (&adc)
#define OVERCURRENT_CONFIRM_CYCLES 5U
static MotorControl_TypeDef MotorControl;
static FOC_TypeDef foc;
static void Set_ErrorNow(ErrorNow_TypeDef x) {MotorControl.ErrorNow=x;}
"""
        + function_source(sensing, "Current_Cal")
        + function_source(calibration, "Task_Calib_CurrentOffset")
        + r"""
int main(void) {
    MotorWorkOutcome_TypeDef outcome = {MOTOR_WORK_RUNNING,Motor_Disable,No_Error,false};
    MotorControl.ModeNow=Calib_CurrentOffset; /* 模拟运行中的标定模式 */
    for(unsigned i=0;i<20000;i++) {
        adc.JDR1=8130+(i&1);adc.JDR2=8205;adc.JDR3=8146;
        outcome=Task_Calib_CurrentOffset(&foc,&MotorControl);
    }
    assert(fabsf(MotorControl.A_Offset-2032.625f)<.00025f);
    assert(MotorControl.B_Offset==2051.25f && MotorControl.C_Offset==2036.5f);
    /* 完成一次标定只返回停机请求，模式迁移由运行状态机执行。 */
    assert(outcome.result==MOTOR_WORK_STOP && MotorControl.ModeNow==Calib_CurrentOffset);
    /* A summed ADC LSB is 1/4 of the former output LSB. */
    adc.JDR1=8131;adc.JDR2=8205;adc.JDR3=8146;
    Current_Cal(&foc,&MotorControl);
    assert(MotorControl.ErrorNow==No_Error);
    assert(fabsf(foc.Ia+.125f*SENSING_CURR_FACTOR)<1e-6f);
    assert(foc.Ib==0 && foc.Ic==0);
    adc.JDR1+=1;Current_Cal(&foc,&MotorControl);
    assert(fabsf(foc.Ia+.375f*SENSING_CURR_FACTOR)<1e-6f);
    /* Actual 1 A input is not inadvertently rescaled by four. */
    adc.JDR2=(uint32_t)roundf(4*(MotorControl.B_Offset-1/SENSING_CURR_FACTOR));
    Current_Cal(&foc,&MotorControl);assert(fabsf(foc.Ib-1)<SENSING_CURR_FACTOR*.13f);
    MotorControl.C_Offset=NAN;Current_Cal(&foc,&MotorControl);assert(MotorControl.ErrorNow==CurrentOffset_Error);
    puts("PASS fractional startup offsets, quarter-code current resolution, actual current scale, invalid-offset protection");
    return 0;
}
"""
    )
    f = out / "current_precision.c"
    f.write_text(src, encoding="utf-8")
    exe = out / "current_precision.exe"
    compiler = [a.cc] + (["cc"] if Path(a.cc).stem == "zig" else [])
    cmd = (
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
        + [str(f), str(ROOT / "firmware/common/utils.c"), "-lm", "-o", str(exe)]
    )
    for shunt in (2, 6):
        for c in [cmd + [f"-DCURRENT_SENSE_SHUNT_MILLIOHM={shunt}"], [str(exe)]]:
            subprocess.run(c, check=True)


if __name__ == "__main__":
    main()
