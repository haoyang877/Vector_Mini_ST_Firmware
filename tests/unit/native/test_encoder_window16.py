"""离线运行实际均值测速C代码，检查时间尺度、回绕和控制入口；不接触硬件。"""
import argparse
from pathlib import Path
import subprocess
from run_position_servo_tests import ROOT, function_source


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cc', required=True)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    source = (ROOT/'firmware/platform/stm32g4/bsp/encoder.c').read_text(encoding='utf-8')
    header = (ROOT/'firmware/platform/stm32g4/bsp/encoder.h').read_text(encoding='utf-8')
    definitions = header[header.index('/* TLE5012B'):header.index('#define brd_enc_spi')]
    fixture = r'''
#undef NDEBUG
#include <assert.h>
#include <limits.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#define _2PI 6.2831853072f
#define SPEED_LOOP_DIVIDER 10U
#define Speed_Ts 0.0005f
''' + definitions + r'''
#if ENCODER_VELOCITY_SOURCE != ENCODER_VELOCITY_SOURCE_WINDOW16
#error "Expected restored default"
#endif
typedef struct { int motor_pole_pairs, axis_profile; bool axis_profile_valid; } MotorControl_TypeDef;
static uint16_t sample;
static bool sample_ok = true;
static bool Encoder_ReadTle5012BFrame(Encoder_TypeDef *e,uint16_t *v,bool started)
{ (void)e; (void)started; *v=sample; return sample_ok; }
static uint16_t Encoder_ApplyDirectionQ15(Encoder_TypeDef *e,uint16_t v)
{ (void)e; return v; }
static uint16_t Encoder_ApplyLinearizationQ15(Encoder_TypeDef *e,uint16_t v)
{ (void)e; return v; }
static int32_t MotorAxisProfile_InitialEncoderOffsetQ15(const int *p,bool valid,int32_t q)
{ (void)p; (void)valid; return q; }
''' + '\n'.join(function_source(source, name) for name in (
        'Encoder_ResetVelocity', 'Encoder_UpdateVelocity2kHz', 'Encoder_UpdateAngles',
        'Encoder_CompleteSample', 'Encoder_GetMecVelContinuous', 'Encoder_GetMecVelFast',
        'Encoder_GetMecVel', 'Encoder_DidUpdateVelocity')) + r'''
int main(void)
{
    MotorControl_TypeDef m={0};
    Encoder_TypeDef e={0};
    m.motor_pole_pairs=21;
    /* 正反跨圈保持倍率正确；无PLL函数可链接，真实采样入口必须走均值分支。 */
    for(int sign=-1;sign<=1;sign+=2) {
        memset(&e,0,sizeof(e)); sample=65520;
        Encoder_CompleteSample(&m,&e,false);
        for(unsigned k=1;k<=1000;k++) {
            sample=(uint16_t)(sample+sign*2);
            Encoder_CompleteSample(&m,&e,false);
            assert(Encoder_DidUpdateVelocity(&e)==(k>=160 && k%10==0));
            if(k<160) { assert(Encoder_GetMecVelContinuous(&e)==0); }
            else {
                float expected=sign*2*_2PI/(65536.0f*0.00005f);
                assert(fabsf(Encoder_GetMecVelContinuous(&e)-expected)<0.0001f);
                assert(Encoder_GetMecVelFast(&e)==Encoder_GetMecVelContinuous(&e));
                assert(fabsf(e.vel_elec-expected*21)<0.001f);
            }
            assert(e.pll_omega_rad_s==0 && e.pll_theta_rad==0);
        }
        assert(e.shadow_q15==65520+sign*2000);
        /* 不再位移时，16个慢拍后完全卸掉旧速度；不是200 us或2 ms窗口。 */
        for(unsigned k=1;k<=160;k++) {
            Encoder_CompleteSample(&m,&e,false);
            if(k%10==0) {
                float expected=sign*2*_2PI/(65536.0f*0.00005f)*(160-k)/160.0f;
                assert(fabsf(e.vel_mech_continuous-expected)<0.0001f);
            }
        }
        assert(e.vel_mech_continuous==0);
    }
    /* 8计数门限只用于常规通道，位置环仍能看到低速。 */
    memset(&e,0,sizeof(e)); sample=1000; Encoder_CompleteSample(&m,&e,false);
    for(unsigned k=1;k<=160;k++) { if(k==10) { sample+=8; } Encoder_CompleteSample(&m,&e,false); }
    assert(e.vel_mech==0 && e.vel_mech_continuous>0);
    assert(fabsf(e.vel_mech_continuous-8*_2PI/(65536.0f*.008f))<1e-6f);
    Encoder_TypeDef before=e; sample_ok=false;
    Encoder_CompleteSample(&m,&e,false);
    assert(memcmp(&e,&before,sizeof(e))==0);
    sample_ok=true;
    e.bad_frame_streak=1; assert(!Encoder_DidUpdateVelocity(&e)); e.bad_frame_streak=0;
    Encoder_ResetVelocity(&e);
    assert(!e.velocity_ready && !e.velocity_win4_ready && e.vel_mech_continuous==0);
    assert(e.velocity_shadow_q15==e.shadow_q15 && e.velocity_divider==0);
    for(unsigned k=0;k<160;k++) { Encoder_CompleteSample(&m,&e,false); }
    assert(e.velocity_ready && e.vel_mech_continuous==0);
    puts("PASS actual WINDOW16: 2 kHz / 8 ms, both directions, wrap, no PLL, warmup, stop, deadband, reset, invalid frame");
    return 0;
}
'''
    c = out/'encoder_window16.c'
    c.write_text(fixture, encoding='utf-8')
    exe = out/'encoder_window16.exe'
    compiler = [args.cc] + (['cc'] if Path(args.cc).stem=='zig' else [])
    subprocess.run(compiler+['-std=c99','-O2','-Wall','-Wextra','-Werror',str(c),'-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)


if __name__=='__main__':
    main()
