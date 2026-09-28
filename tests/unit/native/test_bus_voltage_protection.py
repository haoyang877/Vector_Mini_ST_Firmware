"""使用模拟 ADC 输入执行生产母线采样与模式准入。

不打开探针，也不施加真实过压/欠压；仅覆盖离线判定逻辑。
"""

import sys as _sys
from pathlib import Path as _Path

_sys.path.insert(0, str(_Path(__file__).resolve().parents[3] / "tools"))
import argparse
import subprocess
from pathlib import Path

from project_paths import NATIVE_INCLUDE_FLAGS, ROOT
from run_position_servo_tests import function_source

PRELUDE = r"""
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "data_type.h"
#include "bus_voltage_profile.h"
#include "motor_sensing.h"
#define FOC_FREQ 20000U
#define SENSING_VBUS_FACTOR (3.3f / 4095.0f * 11.0f)
#define ADC2_SUM_TO_COUNTS .25f
#define UTILS_LP_FAST(y,x,a) ((y) -= (a) * ((y) - (x)))
#define ENC_CALIB_ALL 3U
#define ENC_CALIB_LINEARIZED 1U
typedef struct { float Vbus, Vbus_filt; } FOC_TypeDef;
typedef struct { unsigned calib_flag; } Encoder_TypeDef;
static struct { uint32_t JDR4; } adc;
uint16_t motor_hw_vbus_sample_raw(void) {
    return (uint16_t)adc.JDR4;
}
static MotorControl_TypeDef MotorControl;
static FOC_TypeDef FOC;
static Encoder_TypeDef OnBoard_Encoder;
static bool FocCogging_CanStart(const MotorControl_TypeDef *m,const Encoder_TypeDef *e) { (void)m;(void)e; return true; }
static void MotorFault_Report(MotorControl_TypeDef *m,ErrorNow_TypeDef e) { m->ErrorNow=e; }
static bool Encoder_IsOnline(const Encoder_TypeDef *e) { (void)e; return true; }
static uint8_t Encoder_GetCalibFlag(const Encoder_TypeDef *e) { return e->calib_flag; }
static float Encoder_GetMecPos(const Encoder_TypeDef *e) { (void)e; return 0; }
"""

CASES = r"""
static void input(float v) {
    adc.JDR4=(uint32_t)roundf(v/SENSING_VBUS_FACTOR/ADC2_SUM_TO_COUNTS);
    assert(adc.JDR4<=4*4095);
}
static void reset(float v) {
    memset(&MotorControl,0,sizeof(MotorControl));
    input(v); FOC.Vbus_filt=v;
    Vbus_Update(&FOC,&MotorControl); /* Disabled sample resets all counters. */
    MotorControl.axis_profile_valid=true;
    OnBoard_Encoder.calib_flag=3;
}
static void tick(unsigned n) { while(n--) Vbus_Update(&FOC,&MotorControl); }
static void running(float v) { reset(v);MotorControl.ModeNow=Speed_Mode; }
int main(void) {
    unsigned k;
    ModeNow_TypeDef modes[]={Current_Mode,Speed_Mode,Position_Mode,
        Calib_PhaseResistance,Vq_Mode,Calib_EncoderObserver,Calib_EleAngelOffset,
        Calib_Friction,Calib_Anticogging};
    /* Full 8S charge must run beyond both the old 10000-cycle window and
     * the new delays, without inheriting the old 30 V false threshold. */
    running(33.6f);tick(40000);assert(MotorControl.ErrorNow==No_Error);
    puts("PASS 33.6 V full-charge operation");
    running(34.2f);tick(39);assert(MotorControl.ErrorNow==No_Error);
    tick(1);assert(MotorControl.ErrorNow==Over_Voltage);
    input(32);tick(100);assert(MotorControl.ErrorNow==Over_Voltage);
    puts("PASS 2 ms sustained overvoltage and fault latch");
    running(23.9f);tick(1999);assert(MotorControl.ErrorNow==No_Error);
    tick(1);assert(MotorControl.ErrorNow==Under_Voltage);
    puts("PASS 100 ms sustained undervoltage");
    running(33);input(34.8f);tick(2);assert(MotorControl.ErrorNow==No_Error);
    tick(1);assert(MotorControl.ErrorNow==Over_Voltage);
    assert(FOC.Vbus_filt<34.0f);
    puts("PASS three-sample hard overvoltage bypasses filter lag");
    running(33);for(k=0;k<100;k++) {
        input(34.8f);tick(2);input(33);tick(20);
        assert(MotorControl.ErrorNow==No_Error);
    }
    running(23.9f);tick(1900);input(30);tick(1000);
    input(23.9f);tick(1900);assert(MotorControl.ErrorNow==No_Error);
    puts("PASS isolated spikes and interrupted dips do not accumulate");
    running(34.2f);tick(39);MotorControl.ModeNow=Motor_Disable;tick(1);
    MotorControl.ModeNow=Speed_Mode;tick(39);assert(MotorControl.ErrorNow==No_Error);
    tick(1);assert(MotorControl.ErrorNow==Over_Voltage);
    running(23.9f);tick(1999);MotorControl.ModeNow=Motor_Disable;tick(1);
    MotorControl.ModeNow=Speed_Mode;tick(1999);assert(MotorControl.ErrorNow==No_Error);
    tick(1);assert(MotorControl.ErrorNow==Under_Voltage);
    puts("PASS disable resets both confirmation counters");
    for(k=0;k<sizeof(modes)/sizeof(modes[0]);k++) {
        running(34.2f);MotorControl.ModeNow=modes[k];tick(40);
        assert(MotorControl.ErrorNow==Over_Voltage);
        reset(34.2f);assert(!FocMode_Admit(&MotorControl,&FOC,&OnBoard_Encoder,modes[k]));
        assert(MotorControl.ModeNow==Motor_Disable && MotorControl.ErrorNow==Over_Voltage);
        reset(24.5f);assert(!FocMode_Admit(&MotorControl,&FOC,&OnBoard_Encoder,modes[k]));
        assert(MotorControl.ModeNow==Motor_Disable && MotorControl.ErrorNow==Under_Voltage);
    }
    puts("PASS all torque modes protected, unsafe start rejected");
    /* 数值不能因删除枚举而重分配，内部入口也不能接受旧编号或未知值。 */
    assert(Current_Mode == 1 && Speed_Mode == 2 && Position_Mode == 3);
    assert(Calib_EncoderObserver == 13 && Vq_Mode == 14 && Calib_EleAngelOffset == 15);
    assert(Calib_PhaseResistance == 17 && Calib_Friction == 19);
    unsigned rejected[] = {4U, 5U, 11U, 12U, 16U, 18U, 20U, 255U};
    for(k=0;k<sizeof(rejected)/sizeof(rejected[0]);k++) {
        reset(33.0f);
        MotorControl.posRef=1.25f;
        assert(!FocMode_Admit(&MotorControl,&FOC,&OnBoard_Encoder,
                              (ModeNow_TypeDef)rejected[k]));
        assert(MotorControl.ModeNow==Motor_Disable && MotorControl.ErrorNow==No_Error);
        assert(MotorControl.posRef==1.25f);
    }
    puts("PASS retired and unknown mode IDs rejected without state changes");
    reset(33.6f);assert(FocMode_Admit(&MotorControl,&FOC,&OnBoard_Encoder,Speed_Mode));
    reset(25.7f);assert(FocMode_Admit(&MotorControl,&FOC,&OnBoard_Encoder,Speed_Mode));
    reset(33.9f);assert(!FocMode_Admit(&MotorControl,&FOC,&OnBoard_Encoder,Speed_Mode));
    reset(25.5f);assert(!FocMode_Admit(&MotorControl,&FOC,&OnBoard_Encoder,Speed_Mode));
    reset(24.5f);MotorControl.ModeNow=Speed_Mode;
    tick(3000);assert(MotorControl.ErrorNow==No_Error); /* recovery band is start-only */
    MotorControl.ModeNow=Motor_Disable;
    puts("PASS restart hysteresis does not interrupt legal running voltage");
    reset(33);FOC.Vbus_filt=NAN;assert(!FocMode_Admit(&MotorControl,&FOC,&OnBoard_Encoder,Speed_Mode));
    running(33);FOC.Vbus_filt=NAN;tick(1);assert(MotorControl.ErrorNow==Over_Voltage);
    running(34.2f);MotorControl.ErrorNow=Encoder_Error;tick(100);
    assert(MotorControl.ErrorNow==Encoder_Error);
    puts("PASS nonfinite sensing fails closed and first fault is retained");
    return 0;
}
"""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cc", required=True)
    ap.add_argument("--out", type=Path, default=ROOT / "outputs/bus_voltage_8s_20260917/host")
    a = ap.parse_args()
    out = a.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    (out / "main.h").write_text("#include <stdint.h>\n")
    fixture = out / "bus_voltage_test.c"
    fixture.write_text(
        PRELUDE
        + function_source(
            (ROOT / "firmware/motor/foc/foc_sensing.c").read_text(encoding="utf-8"), "Vbus_Update"
        )
        + "\n"
        + function_source(
            (ROOT / "firmware/app/foc_mode_dispatch.c").read_text(encoding="utf-8"),
            "FocMode_Admit",
        )
        + CASES
    )
    cc = [str(Path(a.cc).resolve())] if Path(a.cc).is_file() else [a.cc]
    if Path(a.cc).stem == "zig":
        cc += ["cc"]
    cc += NATIVE_INCLUDE_FLAGS
    exe = out / "bus_voltage_test.exe"
    command = cc + [
        "-std=c99",
        "-O1",
        "-UNDEBUG",
        "-Wall",
        "-Wextra",
        "-Werror",
        "-I",
        str(out),
        "-I",
        str(ROOT / "firmware/common"),
        "-I",
        str(ROOT / "firmware/motor/foc"),
        str(fixture),
        str(ROOT / "firmware/common/crc32.c"),
        str(ROOT / "firmware/services/parameters/motor_axis_profile.c"),
        "-o",
        str(exe),
    ]
    logs = []
    for cmd in [command, [str(exe)]]:
        r = subprocess.run(cmd, capture_output=True, text=True)
        logs.append(r.stdout + r.stderr)
        print(logs[-1], end="")
        (out / "test.log").write_text("\n".join(logs))
        r.check_returncode()


if __name__ == "__main__":
    main()
