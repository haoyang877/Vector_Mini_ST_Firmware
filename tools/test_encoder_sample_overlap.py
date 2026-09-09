"""Check command/receive ownership and fallback using actual board functions."""
import argparse
from pathlib import Path
import subprocess
from run_position_servo_tests import ROOT, function_source


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--cc',required=True);p.add_argument('--out',type=Path,required=True)
    a=p.parse_args();out=a.out.resolve();out.mkdir(parents=True,exist_ok=True)
    source=(ROOT/'Bsp/encoder.c').read_text()
    fixture=r'''
#undef NDEBUG
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <stdio.h>
#define __IO volatile
#define SPI_CR1_SPE 64U
#define SPI_FLAG_TXE 2U
#define SPI_FLAG_RXNE 1U
#define SPI_FLAG_BSY 128U
#define SPI_FLAG_OVR 64U
#define ENC_SPI_XFER_SPIN_MAX 340U
typedef struct { uint32_t CR1, SR, DR; } SPI_TypeDef;
static SPI_TypeDef registers;
static struct { SPI_TypeDef *Instance; } brd_enc_spi = { &registers };
typedef struct {
    uint16_t tle5012_angle_word, tle5012_safety_word;
    uint8_t tle5012_crc_received, tle5012_crc_calculated;
    unsigned status;
} Encoder_TypeDef;
enum { ENCODER_READ_OK, ENCODER_READ_SPI_TIMEOUT };
static unsigned cs, hiz, begins, ends, commands, receives, completes, fail;
static uint16_t response;
static void select_sensor(void) { assert(!cs); cs=1; begins++; }
static void release_sensor(void) { assert(cs && !hiz); cs=0; ends++; }
#define BRD_ENC_CS_ENABLE select_sensor()
#define BRD_ENC_CS_DISABLE release_sensor()
#define __NOP() ((void)0)
static void SPI2_MOSI_HiZ(void) { assert(cs && !hiz); hiz=1; }
static void SPI2_MOSI_RestoreAF(void) { assert(cs && hiz); hiz=0; }
static bool SPI_WaitBSYClear(SPI_TypeDef *s, uint32_t limit) {
    assert(s==&registers && limit==340U); return true;
}
static void Encoder_MarkReadStatus(Encoder_TypeDef *e,unsigned status) { e->status=status; }
static uint16_t SPI_Reg_ReadRx16(SPI_TypeDef *s,bool *ok) {
    assert(s==&registers && cs && !hiz && s->DR==0x8021);
    completes++; *ok=fail!=1; return 0;
}
static uint16_t SPI_Reg_TxRx16(SPI_TypeDef *s,uint16_t data,bool *ok) {
    assert(s==&registers && cs);
    if (data==0x8021) { assert(!hiz); commands++; *ok=fail!=1; return 0; }
    assert(data==0 && hiz); receives++; *ok=fail!=2; return response;
}
''' + function_source(source,'Encoder_BeginSample') + '\n' + function_source(source,'Encoder_ReadTle5012BFrame') + r'''
int main(void) {
    unsigned sr, enabled, pipeline, failure, word, cases=0;
    for (enabled=0;enabled<2;enabled++) for(sr=0;sr<256;sr++) {
        bool started;
        registers.CR1=enabled?SPI_CR1_SPE:0;registers.SR=sr;registers.DR=0x55;
        cs=hiz=begins=ends=0;
        started=Encoder_BeginSample();
        assert(started == (enabled && (sr & (SPI_FLAG_TXE|SPI_FLAG_RXNE|SPI_FLAG_OVR|SPI_FLAG_BSY))==SPI_FLAG_TXE));
        assert(begins==(unsigned)started && cs==(unsigned)started);
        assert(registers.DR==(started?0x8021:0x55));
        cases++;
    }
    for(pipeline=0;pipeline<2;pipeline++) for(failure=0;failure<3;failure++) {
        for(word=0;word<65536;word+=257) {
            Encoder_TypeDef e={0};uint16_t raw=0x1357;bool started,ok;
            cs=hiz=begins=ends=commands=receives=completes=0;
            fail=failure;response=(uint16_t)word;
            registers.CR1=SPI_CR1_SPE;registers.SR=pipeline?SPI_FLAG_TXE:SPI_FLAG_BSY;
            started=Encoder_BeginSample();assert(started==(bool)pipeline);
            /* Sensing/protection may execute here but must not touch this bus. */
            ok=Encoder_ReadTle5012BFrame(&e,&raw,started);
            assert(ok==(failure==0) && !cs && !hiz && begins==1 && ends==1);
            assert(commands==(pipeline?0U:1U) && completes==pipeline);
            assert(receives==(failure==1?0U:1U));
            assert(e.status==(failure?ENCODER_READ_SPI_TIMEOUT:ENCODER_READ_OK));
            assert(raw==(failure?0x1357:(uint16_t)((word & 0x7fffU)<<1)));
            if(!failure) assert(e.tle5012_angle_word==word && e.tle5012_safety_word==0);
            cases++;
        }
    }
    printf("PASS %u encoder cases: nonblocking request, same frame, fallback, both transfer failures, CS/HiZ cleanup\n",cases);
    return 0;
}
'''
    c=out/'encoder_overlap.c';c.write_text(fixture);exe=out/'encoder_overlap.exe'
    cc=[a.cc]+(['cc'] if Path(a.cc).stem=='zig' else [])
    subprocess.run(cc+['-std=c99','-O2','-Wall','-Wextra','-Werror',str(c),'-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)


if __name__=='__main__':main()
