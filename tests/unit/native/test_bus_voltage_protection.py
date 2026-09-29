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
#include "motor_types.h"
#include "bus_voltage_profile.h"
#include "motor_sensing.h"
#define FOC_FREQ 20000U
#define SENSING_VBUS_FACTOR (3.3f / 4095.0f * 11.0f)
#define ADC2_SUM_TO_COUNTS .25f
#define UTILS_LP_FAST(y,x,a) ((y) -= (a) * ((y) - (x)))
#define ENC_CALIB_ALL 3U
#define ENC_CALIB_LINEARIZED 1U
typedef struct { float bus_voltage_v, bus_voltage_filtered_v; } FocState;
typedef struct { unsigned calib_flag; } EncoderState;
static struct { uint32_t JDR4; } adc;
uint16_t motor_hw_vbus_sample_raw(void) {
    return (uint16_t)adc.JDR4;
}
static MotorControlState motor_state;
static FocState s_foc_state;
static EncoderState rotor_encoder;
static void MotorFault_Report(MotorControlState *m,MotorFaultCode e) { m->fault_code=e; }
"""

CASES = r"""
static void input(float v) {
    adc.JDR4=(uint32_t)roundf(v/SENSING_VBUS_FACTOR/ADC2_SUM_TO_COUNTS);
    assert(adc.JDR4<=4*4095);
}
static void reset(float v) {
    memset(&motor_state,0,sizeof(motor_state));
    motor_state.control_mode=MOTOR_CONTROL_NONE;
    input(v); s_foc_state.bus_voltage_filtered_v=v;
    Vbus_Update(&s_foc_state,&motor_state,motor_state.control_mode!=MOTOR_CONTROL_NONE); /* Disabled sample resets all counters. */
    motor_state.application.axis_profile_valid=true;
    rotor_encoder.calib_flag=3;
}
static void tick(unsigned n) { while(n--) Vbus_Update(&s_foc_state,&motor_state,motor_state.control_mode!=MOTOR_CONTROL_NONE); }
static void running(float v) { reset(v);motor_state.control_mode=MOTOR_CONTROL_SPEED; }
/* 正有限 float 的位模式有序；独立整数区间作参考，不复用待测浮点判定。 */
static void test_enable_window(void) {
    float lower = BUS_VOLTAGE_ENABLE_MIN_V, upper = BUS_VOLTAGE_ENABLE_MAX_V;
    uint32_t lower_bits, upper_bits, bits, random_bits = 0x1b497625U;
    unsigned i, exponent, sign, fraction, compared = 0;
    const uint32_t fractions[] = {0, 1, 0x3fffffU, 0x400000U, 0x7fffffU};
    memcpy(&lower_bits, &lower, sizeof(lower_bits));
    memcpy(&upper_bits, &upper, sizeof(upper_bits));
    for (i = 0; i < 2; ++i) {
        uint32_t center = i == 0 ? lower_bits : upper_bits;
        for (bits = center - 4096; bits <= center + 4096; ++bits) {
            memcpy(&s_foc_state.bus_voltage_filtered_v, &bits, sizeof(bits));
            assert(state_vbus_in_enable_window() == (bits >= lower_bits && bits <= upper_bits));
            ++compared;
        }
    }
    for (sign = 0; sign < 2; ++sign) {
        for (exponent = 0; exponent < 256; ++exponent) {
            for (fraction = 0; fraction < sizeof(fractions) / sizeof(fractions[0]); ++fraction) {
                bits = (sign << 31) | (exponent << 23) | fractions[fraction];
                memcpy(&s_foc_state.bus_voltage_filtered_v, &bits, sizeof(bits));
                assert(state_vbus_in_enable_window() == (bits >= lower_bits && bits <= upper_bits));
                ++compared;
            }
        }
    }
    for (i = 0; i < 1000000; ++i) {
        random_bits ^= random_bits << 13;
        random_bits ^= random_bits >> 17;
        random_bits ^= random_bits << 5;
        bits = random_bits;
        memcpy(&s_foc_state.bus_voltage_filtered_v, &bits, sizeof(bits));
        assert(state_vbus_in_enable_window() == (bits >= lower_bits && bits <= upper_bits));
        ++compared;
    }
    printf("PASS %u enable-window values, all exponent/sign classes and endpoint neighbors\n", compared);
}
int main(void) {
    unsigned k;
    test_enable_window();
    /* Full 8S charge must run beyond both the old 10000-cycle window and
     * the new delays, without inheriting the old 30 V false threshold. */
    running(33.6f);tick(40000);assert(motor_state.fault_code==MOTOR_FAULT_NONE);
    puts("PASS 33.6 V full-charge operation");
    running(34.2f);tick(39);assert(motor_state.fault_code==MOTOR_FAULT_NONE);
    tick(1);assert(motor_state.fault_code==MOTOR_FAULT_OVERVOLTAGE);
    input(32);tick(100);assert(motor_state.fault_code==MOTOR_FAULT_OVERVOLTAGE);
    puts("PASS 2 ms sustained overvoltage and fault latch");
    running(23.9f);tick(1999);assert(motor_state.fault_code==MOTOR_FAULT_NONE);
    tick(1);assert(motor_state.fault_code==MOTOR_FAULT_UNDERVOLTAGE);
    puts("PASS 100 ms sustained undervoltage");
    running(33);input(34.8f);tick(2);assert(motor_state.fault_code==MOTOR_FAULT_NONE);
    tick(1);assert(motor_state.fault_code==MOTOR_FAULT_OVERVOLTAGE);
    assert(s_foc_state.bus_voltage_filtered_v<34.0f);
    puts("PASS three-sample hard overvoltage bypasses filter lag");
    running(33);for(k=0;k<100;k++) {
        input(34.8f);tick(2);input(33);tick(20);
        assert(motor_state.fault_code==MOTOR_FAULT_NONE);
    }
    running(23.9f);tick(1900);input(30);tick(1000);
    input(23.9f);tick(1900);assert(motor_state.fault_code==MOTOR_FAULT_NONE);
    puts("PASS isolated spikes and interrupted dips do not accumulate");
    running(34.2f);tick(39);motor_state.control_mode=MOTOR_CONTROL_NONE;tick(1);
    motor_state.control_mode=MOTOR_CONTROL_SPEED;tick(39);assert(motor_state.fault_code==MOTOR_FAULT_NONE);
    tick(1);assert(motor_state.fault_code==MOTOR_FAULT_OVERVOLTAGE);
    running(23.9f);tick(1999);motor_state.control_mode=MOTOR_CONTROL_NONE;tick(1);
    motor_state.control_mode=MOTOR_CONTROL_SPEED;tick(1999);assert(motor_state.fault_code==MOTOR_FAULT_NONE);
    tick(1);assert(motor_state.fault_code==MOTOR_FAULT_UNDERVOLTAGE);
    puts("PASS disable resets both confirmation counters");
    /* 维护执行器不再借用控制模式值；同一显式保护许可覆盖转矩维护。 */
    reset(34.2f);
    for(k=0;k<40;k++) Vbus_Update(&s_foc_state,&motor_state,true);
    assert(motor_state.fault_code==MOTOR_FAULT_OVERVOLTAGE);
    puts("PASS explicit maintenance power expectation protected independently of control mode");
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
            (ROOT / "firmware/app/motor_state_machine/motor_state_machine.c").read_text(
                encoding="utf-8"
            ),
            "state_vbus_in_enable_window",
        ).replace("s_context.foc->", "s_foc_state.")
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
        str(ROOT / "firmware/motor/math"),
        "-I",
        str(ROOT / "firmware/motor/foc"),
        str(fixture),
        str(ROOT / "shared/checksum/crc32.c"),
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
