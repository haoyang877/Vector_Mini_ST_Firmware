"""Validate the actual fast_sqrt implementation without MCU dependencies.

Builds a native C99 numerical sweep. Host timing is deliberately not used as
an MCU speed estimate. --cc accepts Zig, GCC or Clang.
"""

import sys as _sys
from pathlib import Path as _Path
_sys.path.insert(0, str(_Path(__file__).resolve().parents[3] / "tools"))
from project_paths import ROOT, NATIVE_INCLUDE_FLAGS

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
    utils = (ROOT / 'firmware/common/utils.c').read_text(encoding='utf-8')
    actual = function_source(utils, 'fast_sqrt')
    observer = (ROOT / 'firmware/motor/foc/foc_sensorless.c').read_text(encoding='utf-8')
    reference_observer = function_source(observer, 'Fluxobserver_Update').replace(
        'Fluxobserver_Update', 'Reference_Fluxobserver_Update').replace(
        'Fluxobserver_IsFinite', 'Legacy_Fluxobserver_IsFinite')
    transforms = (ROOT / 'firmware/motor/foc/foc_algorithm.c').read_text(encoding='utf-8')
    observer += '\n'.join(function_source(utils, name) for name in (
        'fast_abs', 'fast_sq', 'fast_min', 'fast_atan2', 'normalizeAngle'))
    observer += function_source(transforms, 'Clarke_Transform')
    (out / 'main.h').write_text('#include <stdint.h>\n#include <stddef.h>\n')
    fixture = r'''
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
''' + actual + '\n' + observer + r'''
/* 修改前的有限性判定；其余观测器计算从当前生产函数提取，逐位比较状态轨迹。 */
static int Legacy_Fluxobserver_IsFinite(float value) {
    return value == value && value <= FLT_MAX && value >= -FLT_MAX;
}
''' + reference_observer + r'''
static float from_bits(uint32_t bits) {
    float value; memcpy(&value,&bits,sizeof(value)); return value;
}
static void check_finite_pattern(uint32_t bits) {
    float value=from_bits(bits);
    int expected=(bits & 0x7f800000U)!=0x7f800000U;
    assert(Fluxobserver_IsFinite(value)==expected);
    assert(Fluxobserver_IsFinite(value)==Legacy_Fluxobserver_IsFinite(value));
}
static void check_finite_bit_patterns(void) {
    unsigned checked=0;
    const uint32_t edges[]={0U,0x80000000U,1U,0x80000001U,0x007fffffU,0x807fffffU,
        0x00800000U,0x80800000U,0x7f7fffffU,0xff7fffffU,0x7f800000U,0xff800000U,
        0x7f800001U,0xff800001U,0x7fbfffffU,0xffbfffffU,0x7fc00000U,0xffc00000U,
        0x7fffffffU,0xffffffffU};
    for(unsigned i=0;i<sizeof(edges)/sizeof(edges[0]);++i) {
        check_finite_pattern(edges[i]); ++checked;
    }
    /* 所有指数、符号和尾数边界，以及全部 NaN payload（包括 sNaN/qNaN）。 */
    for(uint32_t exponent=0;exponent<256;++exponent) {
        for(uint32_t fraction=0;fraction<0x800000U;fraction+=8191U) {
            check_finite_pattern((exponent<<23)|fraction);
            check_finite_pattern(0x80000000U|(exponent<<23)|fraction); checked+=2;
        }
    }
    for(uint32_t fraction=0;fraction<0x800000U;++fraction) {
        check_finite_pattern(0x7f800000U|fraction);
        check_finite_pattern(0xff800000U|fraction); checked+=2;
    }
    uint32_t bits=0x12345678U;
    for(unsigned i=0;i<1000000U;++i) {
        bits=1664525U*bits+1013904223U; check_finite_pattern(bits); ++checked;
    }
    printf("PASS observer finite predicate: %u patterns, all NaN payloads, infinities, signed zero, subnormal and FLT_MAX\n",checked);
}
static void check_observer_trajectory_equivalence(void) {
    unsigned checked=0;
    for(unsigned scenario=0;scenario<8;++scenario) {
        FOC_TypeDef foc={0}, reference_foc={0};
        MotorControl_TypeDef motor={0};
        Fluxobserver_TypeDef observer={0}, reference={0};
        motor.motor_phase_resistance=2.144f;
        motor.motor_d_inductance=motor.motor_q_inductance=.0009f;
        motor.motor_flux=.0175025f;
        Fluxobserver_ParamInit(&observer);
        observer.x1_last=motor.motor_flux; reference=observer;
        for(unsigned tick=0;tick<20000;++tick) {
            float phase=(float)tick*.01575f*(scenario&1U ? -1.0f:1.0f);
            float amplitude=(scenario&2U) ? 3.0f:.1f;
            foc.Ia=amplitude*cosf(phase); foc.Ib=amplitude*cosf(phase-_2PI/3.0f);
            foc.Ic=-foc.Ia-foc.Ib; foc.Vbus_filt=33.0f;
            foc.mod_alpha=.4f*cosf(phase+.5f); foc.mod_beta=.4f*sinf(phase+.5f);
            if(tick==1000) motor.motor_phase_resistance=2.2f;
            if(tick==4000) observer.x1_last=reference.x1_last=NAN;
            if(tick==5000) foc.Ia=INFINITY;
            if(tick==6000) motor.motor_phase_resistance=-INFINITY;
            if(tick==6001) motor.motor_phase_resistance=2.2f;
            if(tick==7000) foc.mod_alpha=from_bits(0x7f800001U);
            if(tick==8000) motor.motor_d_inductance=motor.motor_q_inductance=FLT_MAX;
            if(tick==8001) motor.motor_d_inductance=motor.motor_q_inductance=.001f;
            if(tick==9000) foc.Vbus_filt=NAN;
            if(tick==10000) motor.motor_flux=NAN;
            if(tick==10001) motor.motor_flux=.018f;
            if(tick==11000) observer.gamma=reference.gamma=NAN;
            if(tick==12000) observer.x1_last=reference.x1_last=FLT_MAX;
            if(tick==13000) observer.x2_last=reference.x2_last=from_bits(0xff800001U);
            if(tick==14000) motor.motor_flux=from_bits(1U);
            if(tick==14001) motor.motor_flux=.018f;
            if(tick==15000) motor.motor_flux=from_bits(0x80000000U);
            if(tick==15001) motor.motor_flux=.018f;
            if((scenario&4U) && tick==16000) observer.gamma=reference.gamma=INFINITY;
            reference_foc=foc;
            Reference_Fluxobserver_Update(&reference_foc,&motor,&reference);
            Fluxobserver_Update(&foc,&motor,&observer);
            assert(memcmp(&reference,&observer,sizeof(observer))==0);
            assert(memcmp(&reference_foc,&foc,sizeof(foc))==0);
            ++checked;
        }
        assert(observer.position_epoch>10U);
    }
    printf("PASS observer full-state bitwise equivalence: %u dynamic ticks, live parameters, nonfinite inputs/state and resets\n",checked);
}
static void check_observer_math(void) {
    FOC_TypeDef foc={0};
    MotorControl_TypeDef motor={0};
    Fluxobserver_TypeDef observer={0};
    motor.motor_phase_resistance=2.144f;
    motor.motor_d_inductance=motor.motor_q_inductance=.0009f;
    motor.motor_flux=.0175025f;
    /* gamma=0、零输入使状态固定，隔离单圈角度归一化且覆盖正反向跨零。 */
    for(unsigned i=0;i<=200000;++i) {
        float phase=(float)i*(4.0f*_PI/200000.0f)-2.0f*_PI;
        observer.x1_last=motor.motor_flux*cosf(phase);
        observer.x2_last=motor.motor_flux*sinf(phase);
        float raw=fast_atan2(observer.x2_last/motor.motor_flux,
            observer.x1_last/motor.motor_flux);
        float expected=normalizeAngle(raw);
        Fluxobserver_Update(&foc,&motor,&observer);
        assert(observer.theta_e==expected);
        assert(observer.theta_e>=0 && observer.theta_e<=_2PI);
    }
    /* 参数非法/状态非有限仍重置 epoch；gamma 无效仍沿用旧 fast_min 限幅。 */
    float *fields[]={&motor.motor_phase_resistance,&motor.motor_d_inductance,
        &motor.motor_q_inductance,&motor.motor_flux};
    for(unsigned i=0;i<sizeof(fields)/sizeof(fields[0]);++i) {
        float saved=*fields[i]; unsigned epoch=observer.position_epoch;
        *fields[i]=NAN; Fluxobserver_Update(&foc,&motor,&observer);
        assert(observer.position_epoch==epoch+1 && observer.theta_e==0);
        *fields[i]=saved;
    }
    unsigned epoch=observer.position_epoch;
    motor.motor_flux=0; Fluxobserver_Update(&foc,&motor,&observer);
    assert(observer.position_epoch==epoch+1);
    motor.motor_flux=.0175025f; observer.x1_last=NAN;
    epoch=observer.position_epoch; Fluxobserver_Update(&foc,&motor,&observer);
    assert(observer.position_epoch==epoch+1 && isfinite(observer.theta_e));
    Fluxobserver_TypeDef reference={0};
    reference.x1_last=.01f; reference.x2_last=.01f;
    reference.gamma=.2f/(Current_Ts*(motor.motor_flux*motor.motor_flux));
    observer=reference; observer.gamma=NAN;
    Fluxobserver_Update(&foc,&motor,&reference);
    Fluxobserver_Update(&foc,&motor,&observer);
    assert(observer.x1==reference.x1 && observer.x2==reference.x2);
    puts("PASS observer: 200001 angle vectors, live model/state guards, NaN gamma limit");
}
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
    check_finite_bit_patterns();
    check_observer_trajectory_equivalence();
    check_observer_math();
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
    compiler = [args.cc] + (['cc'] if Path(args.cc).stem == 'zig' else []) + ['-I', str(out)] + NATIVE_INCLUDE_FLAGS
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
