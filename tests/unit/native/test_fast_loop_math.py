"""在无 MCU 依赖下验证真实平方根及正余弦查表实现。

构建原生 C99 数值扫描；主机耗时不作为 MCU 速度估计。--cc 支持 Zig、GCC 或 Clang。
"""

import sys as _sys
from pathlib import Path as _Path

_sys.path.insert(0, str(_Path(__file__).resolve().parents[3] / "tools"))
import argparse
import subprocess
from pathlib import Path

from project_paths import NATIVE_INCLUDE_FLAGS, ROOT
from run_position_servo_tests import function_source


def sine_fixture() -> str:
    """用独立冻结的整表位模式核对生产查表，单独覆盖旧越界周期端点。"""
    golden = ROOT / "tests/fixtures/sine_table_f32_bits.txt"
    words = [
        word
        for line in golden.read_text(encoding="utf-8").splitlines()
        if not line.startswith("#")
        for word in line.split()
    ]
    if len(words) != 1024:
        raise ValueError("正弦黄金表必须保留全部 1024 个 float32 位模式")
    return (
        r"""
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "utils.h"
static const uint32_t golden[1024] = {
"""
        + ",\n".join("0x" + word + "U" for word in words)
        + r"""
};
static unsigned seam_repairs, samples;
static unsigned seen[1024];
static uint32_t bits(float value)
{
    uint32_t result;
    memcpy(&result, &value, sizeof(result));
    return result;
}
static uint32_t reference(float angle)
{
    /* 冻结基准归一化和缩放；不调用生产函数来计算期望值。 */
    int turns = (int)(angle * 0.1591549431f);
    float a = angle - turns * 6.2831853072f;
    a = a >= 0.0f ? a : a + 6.2831853072f;
    int index = (int)(162.97466172f * a);
    assert(index >= 0 && index <= 1024);
    if (index == 1024)
    {
        seam_repairs++;
        return 0U;
    }
    seen[index]++;
    return golden[index];
}
static void check(float angle)
{
    assert(bits(fast_sin(angle)) == reference(angle));
    assert(bits(fast_cos(angle)) == reference(1.5707963268f - angle));
    samples++;
}
int main(void)
{
    for (int i = 0; i < 1024; ++i)
    {
        float center = ((float)i + 0.25f) / 162.97466172f;
        check(center);
        float edge = (float)i / 162.97466172f;
        check(edge);
        check(nextafterf(edge, -INFINITY));
        check(nextafterf(edge, INFINITY));
    }
    for (int i = -100000; i <= 100000; ++i)
    {
        check((float)i * 0.0031f);
    }
    check(0.0f);
    check(-0.0f);
    check(-1.0e-8f);
    check(6.2831853072f);
    check(-6.2831853072f);
    for (unsigned i = 0; i < 1024; ++i)
    {
        assert(seen[i] != 0U);
    }
    assert(golden[0] == 0U && golden[1023] == 0U && seam_repairs != 0U);
    printf("PASS sine/cosine: all 1024 golden entries; %u angles; %u seam repairs; bit exact\n",
           samples, seam_repairs);
    return 0;
}
"""
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", required=True)
    parser.add_argument("--out", type=Path, default=ROOT / "outputs/fast_loop_math")
    args = parser.parse_args()
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    actual = function_source(
        (ROOT / "firmware/common/utils.c").read_text(encoding="utf-8"), "fast_sqrt"
    )
    fixture = (
        r"""
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
"""
        + actual
        + r"""
/* The old 32-bit target algorithm, with defined bit conversions for the host. */
static float legacy(float x) {
    uint32_t bits;
    float y;
    if (x < 1.19209290e-7f)
    {
        return 0.0f;
    }
    memcpy(&bits, &x, sizeof(bits));
    bits = 0x1FBA6EE6U + (bits >> 1);
    memcpy(&y, &bits, sizeof(y));
    y = (y + x / y) * 0.5f;
    return (y + x / y) * 0.5f;
}
int main(void) {
    uint32_t bits;
    unsigned samples = 0;
    float max_change = 0.0f;
    float cutoff = 1.19209290e-7f;
    assert(fast_sqrt(0.0f) == 0.0f);
    assert(fast_sqrt(-1.0f) == 0.0f);
    assert(fast_sqrt(nextafterf(cutoff, 0.0f)) == 0.0f);
    assert(fast_sqrt(cutoff) > 0.0f);
    assert(isnan(fast_sqrt(NAN)));
    assert(isinf(fast_sqrt(INFINITY)));
    /* Sweep positive normal floats across the entire exponent range. */
    for (bits = 0x00800000U; bits < 0x7f800000U; bits += 997U) {
        float x, result, before, expected, change;
        memcpy(&x, &bits, sizeof(x));
        result = fast_sqrt(x);
        before = legacy(x);
        expected = x < cutoff ? 0.0f : (float)sqrt((double)x);
        assert(result == expected);
        change = expected > 0.0f ? fabsf(before-result)/expected : 0.0f;
        if (change > max_change)
        {
            max_change = change;
        }
        assert(change < 0.000003f);
        samples++;
    }
    printf("PASS fast_sqrt: %u samples, max relative change %.9g; cutoff and nonfinite inputs checked\n",
           samples, (double)max_change);
    return 0;
}
"""
    )
    source = out / "fast_loop_math_test.c"
    source.write_text(fixture, encoding="utf-8")
    exe = out / "fast_loop_math_test.exe"
    compiler = [args.cc] + (["cc"] if Path(args.cc).stem == "zig" else []) + NATIVE_INCLUDE_FLAGS
    commands = [
        compiler
        + [
            "-std=c99",
            "-O2",
            "-UNDEBUG",
            "-Wall",
            "-Wextra",
            "-Werror",
            str(source),
            "-o",
            str(exe),
        ],
        [str(exe)],
    ]
    logs = []
    for command in commands:
        result = subprocess.run(command, cwd=ROOT, capture_output=True, text=True)
        logs.append(result.stdout + result.stderr)
        (out / "test.log").write_text("\n".join(logs), encoding="utf-8")
        print(logs[-1], end="")
        result.check_returncode()

    sine_source = out / "sine_table_test.c"
    sine_source.write_text(sine_fixture(), encoding="utf-8")
    sine_exe = out / "sine_table_test.exe"
    sine_commands = [
        compiler
        + [
            "-std=c99",
            "-O2",
            "-UNDEBUG",
            "-Wall",
            "-Wextra",
            "-Werror",
            str(sine_source),
            str(ROOT / "firmware/common/utils.c"),
            "-o",
            str(sine_exe),
        ],
        [str(sine_exe)],
    ]
    for command in sine_commands:
        result = subprocess.run(command, cwd=ROOT, capture_output=True, text=True)
        logs.append(result.stdout + result.stderr)
        (out / "test.log").write_text("\n".join(logs), encoding="utf-8")
        print(logs[-1], end="")
        result.check_returncode()


if __name__ == "__main__":
    main()
