"""Validate the actual fast_sqrt implementation without MCU dependencies.

Builds a native C99 numerical sweep. Host timing is deliberately not used as
an MCU speed estimate. --cc accepts Zig, GCC or Clang.
"""
import argparse
from pathlib import Path
import subprocess

from run_position_servo_tests import ROOT, function_source


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cc', required=True)
    parser.add_argument('--out', type=Path, default=ROOT / 'outputs/fast_loop_math')
    args = parser.parse_args()
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    actual = function_source((ROOT / 'System/utils.c').read_text(encoding='utf-8'), 'fast_sqrt')
    fixture = r'''
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
''' + actual + r'''
/* The old 32-bit target algorithm, with defined bit conversions for the host. */
static float legacy(float x) {
    uint32_t bits;
    float y;
    if (x < 1.19209290e-7f) return 0.0f;
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
        if (change > max_change) max_change = change;
        assert(change < 0.000003f);
        samples++;
    }
    printf("PASS fast_sqrt: %u samples, max relative change %.9g; cutoff and nonfinite inputs checked\n",
           samples, (double)max_change);
    return 0;
}
'''
    source = out / 'fast_loop_math_test.c'
    source.write_text(fixture, encoding='utf-8')
    exe = out / 'fast_loop_math_test.exe'
    compiler = [args.cc] + (['cc'] if Path(args.cc).stem == 'zig' else [])
    commands = [compiler + ['-std=c99', '-O2', '-Wall', '-Wextra', '-Werror',
                            str(source), '-o', str(exe)], [str(exe)]]
    logs = []
    for command in commands:
        result = subprocess.run(command, cwd=ROOT, capture_output=True, text=True)
        logs.append(result.stdout + result.stderr)
        (out / 'test.log').write_text('\n'.join(logs), encoding='utf-8')
        print(logs[-1], end='')
        result.check_returncode()


if __name__ == '__main__':
    main()
