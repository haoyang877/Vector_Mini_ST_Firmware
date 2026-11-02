"""离线验证坐标结构体、六扇区和PWM重构，可与修改前源码逐位比较。"""

import argparse
import subprocess
from pathlib import Path

from run_position_servo_tests import NATIVE_INCLUDE_FLAGS, ROOT, function_source

FUNCTIONS = (
    "Clarke_Transform",
    "Clarke_Inverse",
    "Park_Transform",
    "Park_Inverse",
    "Svm_SectorJudge",
)

CASES = r"""
static unsigned checked;
static void near(float a, float b, float tolerance) {
    assert(fabsf(a - b) <= tolerance);
}
static void check(float alpha, float beta, float theta) {
    AlphaBetaVector ab = {alpha, beta}, recovered;
    ABCVect abc, duty;
    DqVector dq;
    float magnitude = hypotf(alpha, beta);
    float tolerance = 8e-7f * (1.0f + magnitude);
    Clarke_Inverse(&ab, &abc);
    near(abc.a + abc.b + abc.c, 0, tolerance);
    Clarke_Transform(&abc, &recovered);
    near(recovered.alpha, alpha, tolerance);
    near(recovered.beta, beta, tolerance);
    Park_Transform(&ab, theta, &dq);
    near(dq.d, alpha*cosf(theta) + beta*sinf(theta), 1e-5f*(1+magnitude));
    near(dq.q, -alpha*sinf(theta) + beta*cosf(theta), 1e-5f*(1+magnitude));
    Park_Inverse(&dq, theta, &recovered);
    near(recovered.alpha, alpha, 1e-5f*(1+magnitude));
    near(recovered.beta, beta, 1e-5f*(1+magnitude));

    int32_t sector = Svm_SectorJudge(&ab, &duty);
    assert(sector >= 1 && sector <= 6);
    /* Recover the requested vector from average line-to-line PWM voltages.
     * This also checks the original active-low PWM polarity and normalization. */
    near(.5f*(duty.b + duty.c - 2*duty.a), alpha, tolerance);
    near(1.5f*ONE_BY_SQRT_3*(duty.c - duty.b), beta, tolerance);
    float smallest = fminf(duty.a, fminf(duty.b, duty.c));
    float largest = fmaxf(duty.a, fmaxf(duty.b, duty.c));
    near(smallest + largest, 1, tolerance);
    if (magnitude <= .95f*SQRT_3_BY_2) {
        assert(smallest >= 0 && largest <= 1);
    }
#ifdef HAS_REFERENCE
    float a, b, c, d, q, x, y;
    int32_t original_sector;
    Reference_Clarke_Inverse(alpha, beta, &a, &b, &c);
    assert(memcmp(&abc.a, &a, sizeof(a)) == 0);
    assert(memcmp(&abc.b, &b, sizeof(b)) == 0);
    assert(memcmp(&abc.c, &c, sizeof(c)) == 0);
    Reference_Clarke_Transform(a, b, c, &x, &y);
    Clarke_Transform(&abc, &recovered);
    assert(memcmp(&recovered.alpha, &x, sizeof(x)) == 0);
    assert(memcmp(&recovered.beta, &y, sizeof(y)) == 0);
    Reference_Park_Transform(alpha, beta, theta, &d, &q);
    assert(memcmp(&dq.d, &d, sizeof(d)) == 0);
    assert(memcmp(&dq.q, &q, sizeof(q)) == 0);
    Reference_Park_Inverse(d, q, theta, &x, &y);
    Park_Inverse(&dq, theta, &recovered);
    assert(memcmp(&recovered.alpha, &x, sizeof(x)) == 0);
    assert(memcmp(&recovered.beta, &y, sizeof(y)) == 0);
    Reference_Svm_SectorJudge(alpha, beta, &a, &b, &c, &original_sector);
    assert(sector == original_sector);
    assert(memcmp(&duty.a, &a, sizeof(a)) == 0);
    assert(memcmp(&duty.b, &b, sizeof(b)) == 0);
    assert(memcmp(&duty.c, &c, sizeof(c)) == 0);
#endif
    ++checked;
}
static void boundaries(void) {
    const float beta[] = {0, -0.0f, .3f, -.3f};
    for (unsigned i=0; i<sizeof(beta)/sizeof(beta[0]); ++i) {
        float edge = ONE_BY_SQRT_3*beta[i];
        const float alpha[] = {0, -0.0f, edge, -edge,
            nextafterf(edge, -INFINITY), nextafterf(edge, INFINITY),
            nextafterf(-edge, -INFINITY), nextafterf(-edge, INFINITY)};
        for (unsigned j=0; j<sizeof(alpha)/sizeof(alpha[0]); ++j)
            check(alpha[j], beta[i], 0);
    }
    const AlphaBetaVector vectors[] = {
        {0,0}, {.5f,0}, {0,.5f}, {-.5f,0}, {0,-.5f},
        {ONE_BY_SQRT_3*.3f,.3f}, {-ONE_BY_SQRT_3*.3f,.3f},
        {-ONE_BY_SQRT_3*.3f,-.3f}, {ONE_BY_SQRT_3*.3f,-.3f}
    };
    const int expected[] = {1,1,2,3,5,1,2,5,6};
    for (unsigned i=0; i<sizeof(vectors)/sizeof(vectors[0]); ++i) {
        ABCVect duty;
        assert(Svm_SectorJudge(&vectors[i], &duty) == expected[i]);
    }
    ABCVect duty;
    AlphaBetaVector zero = {0,0}, overmodulated = {1.2f,0};
    Svm_SectorJudge(&zero, &duty);
    assert(duty.a == .5f && duty.b == .5f && duty.c == .5f);
    Svm_SectorJudge(&overmodulated, &duty);
    assert(duty.a < 0 && duty.b > 1 && duty.c > 1);
}
int main(void) {
    boundaries();
    const float radii[] = {0, .001f, .2f, .5f, .95f*SQRT_3_BY_2, SQRT_3_BY_2, 1.2f};
    for (unsigned i=0; i<36000; ++i) {
        float angle = ((float)i + .5f)*_2PI/36000;
        float theta = 7*angle - 3*_2PI;
        for (unsigned j=0; j<sizeof(radii)/sizeof(radii[0]); ++j) {
            check(radii[j]*cosf(angle), radii[j]*sinf(angle), theta);
            if (radii[j] > 0) {
                AlphaBetaVector ab = {radii[j]*cosf(angle), radii[j]*sinf(angle)};
                ABCVect duty;
                assert(Svm_SectorJudge(&ab, &duty) == (int)(i/6000)+1);
            }
        }
    }
    printf("PASS FOC transforms: %u vectors, six sectors, boundaries, PWM reconstruction and round trips\n", checked);
    return 0;
}
"""


def main() -> None:
    """提取真实函数，在主机上验证单位变换和原六扇区输出。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", required=True)
    parser.add_argument("--out", type=Path, default=ROOT / "outputs/foc_axis_structs")
    parser.add_argument("--reference-source", type=Path)
    args = parser.parse_args()
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    source = (ROOT / "firmware/motor/control/foc.c").read_text(encoding="utf-8")
    fixture = """
#undef NDEBUG
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "foc.h"
#include "control_math.h"
"""
    fixture += "\n".join(function_source(source, name) for name in FUNCTIONS)
    if args.reference_source:
        reference = args.reference_source.read_text(encoding="utf-8")
        fixture += "\n#define HAS_REFERENCE 1\n"
        fixture += "\n".join(
            function_source(reference, name).replace(name, "Reference_" + name)
            for name in FUNCTIONS
        )
    fixture += CASES
    test = out / "foc_axis_structs_test.c"
    test.write_text(fixture, encoding="utf-8")
    exe = out / "foc_axis_structs_test.exe"
    compiler = [args.cc] + (["cc"] if Path(args.cc).stem == "zig" else [])
    for command in (
        compiler
        + ["-std=c99", "-O2", "-Wall", "-Wextra", "-Werror"]
        + NATIVE_INCLUDE_FLAGS
        + [str(test), str(ROOT / "firmware/motor/math/control_math.c"), "-lm", "-o", str(exe)],
        [str(exe)],
    ):
        result = subprocess.run(command, cwd=ROOT, capture_output=True, text=True)
        with (out / "test.log").open("a", encoding="utf-8") as log:
            log.write(result.stdout + result.stderr)
        print(result.stdout + result.stderr, end="")
        result.check_returncode()


if __name__ == "__main__":
    main()
