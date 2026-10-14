"""离线验证半波查表与原 1024 点表逐位一致，覆盖周期舍入边界。"""

import argparse
import hashlib
import json
from pathlib import Path
import re
import struct
import subprocess
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'tools'))
from project_paths import ROOT
from run_position_servo_tests import function_source


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cc', required=True)
    parser.add_argument('--out', type=Path, default=ROOT / 'outputs/fast_trig')
    args = parser.parse_args()
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    source = (ROOT / 'firmware/common/utils.c').read_text(encoding='utf-8')
    table = re.search(r'static const float sin_half_tab\[512\] = \{(.*?)\n\};', source, re.S)
    half = [float(value.strip().removesuffix('f')) for value in table[1].split(',') if value.strip()]
    original = half + [0.0 - value for value in reversed(half)]
    vectors = json.loads((ROOT / 'tests/vectors/app_size_v1.json').read_text(encoding='utf-8'))
    digest = hashlib.sha256(b''.join(struct.pack('<f', value) for value in original)).hexdigest()
    if len(half) != vectors['stored_points'] or digest != vectors['sine_table_ieee754_le_sha256']:
        raise ValueError('半波镜像未逐位还原 v0.5.2 原始查表')
    reference = 'static const float original[1024] = {\n' + ',\n'.join(
        format(value, '.9g') + 'f' if '.' in format(value, '.9g') else format(value, '.9g') + '.0f'
        for value in original) + '\n};\n'
    actual = table[0] + '\n' + '\n'.join(function_source(source, name) for name in (
        'normalizeAngle', 'fast_sin', 'fast_cos'))
    fixture = r'''
#undef NDEBUG
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "utils.h"
''' + reference + actual + r'''
static unsigned checked, wrapped;
static float max_error;
static uint32_t bits(float value) {
    uint32_t result;
    memcpy(&result, &value, sizeof(result));
    return result;
}
static void check(float theta) {
    int index = (int)(LUT_MULT * normalizeAngle(theta));
    float value = fast_sin(theta);
    float error = fabsf(value - (float)sin((double)theta));
    assert(index >= 0 && index <= 1024);
    if (index < 1024) {
        assert(bits(value) == bits(original[index]));
    } else {
        assert(bits(value) == bits(original[0]));
        wrapped++;
    }
    assert(error < 0.0063f);
    if (error > max_error) max_error = error;
    checked++;
}
int main(void) {
    unsigned i;
    for (i = 0; i < 1024; i++) {
        float theta = ((float)i + 0.25f) / LUT_MULT;
        assert((int)(LUT_MULT * normalizeAngle(theta)) == (int)i);
        assert(bits(fast_sin(theta)) == bits(original[i]));
        printf("LUT %08x\n", (unsigned)bits(fast_sin(theta)));
        theta = (float)i / LUT_MULT;
        check(theta);
        check(nextafterf(theta, INFINITY));
        check(nextafterf(theta, -INFINITY));
        check(theta - _2PI);
    }
    for (i = 0; i <= 1000000; i++) {
        float theta = ((float)i / 1000000.0f - 0.5f) * (128.0f * _PI);
        float shifted = _PI_2 - theta;
        check(theta);
        assert(bits(fast_cos(theta)) == bits(fast_sin(shifted)));
    }
    check(-0.000000001f);
    check(0.0f); check(-0.0f); check(_2PI); check(-_2PI);
    assert(wrapped > 0);
    printf("PASS fast trig: %u samples, %u rounded boundaries, max absolute error %.9g\n",
        checked, wrapped, (double)max_error);
    return 0;
}
'''
    (out / 'main.h').write_text('#include <stdint.h>\n', encoding='utf-8')
    test = out / 'fast_trig_test.c'
    test.write_text(fixture, encoding='utf-8')
    exe = out / 'fast_trig_test.exe'
    compiler = [args.cc] + (['cc'] if Path(args.cc).stem == 'zig' else [])
    command = compiler + ['-std=c99', '-O2', '-Wall', '-Wextra', '-Werror',
        '-I', str(out), '-I', str(ROOT / 'firmware/common'), str(test), '-o', str(exe)]
    result = subprocess.run(command, cwd=ROOT, capture_output=True, text=True)
    (out / 'compile.log').write_text(result.stdout + result.stderr, encoding='utf-8')
    result.check_returncode()
    result = subprocess.run([str(exe)], cwd=ROOT, capture_output=True, text=True)
    (out / 'test.log').write_text(result.stdout + result.stderr, encoding='utf-8')
    result.check_returncode()
    actual_bits = [int(line[4:], 16) for line in result.stdout.splitlines() if line.startswith('LUT ')]
    actual_digest = hashlib.sha256(b''.join(struct.pack('<I', value) for value in actual_bits)).hexdigest()
    if actual_digest != vectors['sine_table_ieee754_le_sha256']:
        raise ValueError('实际 C 实现查表结果与原始 golden 哈希不同')
    print(result.stdout.splitlines()[-1])
    print('PASS all 1024 original table values: ' + actual_digest)


if __name__ == '__main__':
    main()
