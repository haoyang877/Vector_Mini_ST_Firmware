"""离线验证母线电压搬运、低通收敛、缺帧置NAN与启动窗口位模式；不访问硬件。

不施加真实过压/欠压，也不验证采样模块的保护判定（该判定已删除）。
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
#include "bsp_adc.h"
#include "motor_types.h"
#include "motor_config.h"
#include "application_config.h"
#include "board_config.h"
#include "foc.h"
#define UTILS_LP_FAST(y,x,a) ((y) -= (a) * ((y) - (x)))
static FocState s_foc_state = {.bus_voltage_filter_alpha = 0.05f};
static float voltage_sample_v;
float BspAdc_ReadChannel(BspAdcChannel channel) {
    assert(channel == BSP_ADC_BUS_VOLTAGE);
    return voltage_sample_v;
}
"""

CASES = r"""
static void sample_voltage(float sample) {
    voltage_sample_v = sample;
    MotorVoltage_Update(&s_foc_state);
}
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
    unsigned i;
    float sample = 30.0f;

    test_enable_window();

    /* 原始值逐拍搬运；首拍低通从原状态向新输入收敛。 */
    s_foc_state.bus_voltage_filtered_v = 0.0f;
    sample_voltage(sample);
    assert(s_foc_state.bus_voltage_v == 30.0f);
    assert(fabsf(s_foc_state.bus_voltage_filtered_v - 1.5f) < 1e-6f);
    puts("PASS raw bus voltage copied and low-pass seeded");

    const float target_v = 32.5f;
    sample = target_v;
    for (i = 0; i < 1000; ++i) {
        sample_voltage(sample);
    }
    assert(s_foc_state.bus_voltage_v == target_v);
    assert(fabsf(s_foc_state.bus_voltage_filtered_v - target_v) < 0.01f);
    puts("PASS low-pass converges to input");

    /* 缺帧不再报故障；两个字段均为NAN。 */
    sample_voltage(NAN);
    assert(isnan(s_foc_state.bus_voltage_v) && isnan(s_foc_state.bus_voltage_filtered_v));
    puts("PASS NAN input yields NAN fields");
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
            (ROOT / "firmware/motor/sensing/motor_voltage.c").read_text(encoding="utf-8"),
            "MotorVoltage_Update",
        )
        + "\n"
        + function_source(
            (ROOT / "firmware/app/runtime/motor_lifecycle.c").read_text(encoding="utf-8"),
            "state_vbus_in_enable_window",
        )
        .replace(
            "state_vbus_in_enable_window(void)",
            "state_vbus_in_enable_window(void)",
        )
        .replace("s_context.foc->", "s_foc_state.")
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
        "-DBSP_BOARD=1",
        "-I",
        str(out),
        "-I",
        str(ROOT / "firmware/motor/math"),
        "-I",
        str(ROOT / "firmware/motor/control"),
        str(fixture),
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
