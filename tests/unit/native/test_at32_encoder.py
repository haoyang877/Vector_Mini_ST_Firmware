"""核对两板真实Begin/ReadAngle：片选、MOSI释放、DMA与角度位宽。"""

from _bsp_test_support import arguments, production, resources, run_c
from run_position_servo_tests import ROOT


def main() -> None:
    """覆盖16/21位输出及DMA字段映射；不增加CRC、超时或软件状态接口。"""
    args = arguments(__doc__)
    for board in (1, 2):
        source = r"""
#include <assert.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include "bsp_encoder.h"
static unsigned nops;
#define __NOP() (++nops)
#define __IO volatile
"""
        if board == 1:
            source += (
                r"""
static struct {uint32_t BSRR,MODER;} gpio;
static struct {uint32_t SR,DR;} spi;
#define GPIOB (&gpio)
#define SPI2 (&spi)
#define SPI_SR_TXE 2U
#define SPI_SR_RXNE 1U
#define SPI_SR_BSY 128U
"""
                + resources(board)
                + production(board, "encoder_spi", "BspEncoder_Begin", "BspEncoder_ReadAngle")
                + r"""
int main(void) {
    const BspEncoderResolution resolutions[]={BSP_ENCODER_16BIT,BSP_ENCODER_21BIT};
    for(unsigned format=0;format<2;format++) {
    for(unsigned word=0;word<65536;word++) {
        gpio.MODER=0xAAAAAAAA; spi.SR=3; nops=0;
        BspEncoder_Begin();
        assert(gpio.BSRR==(1U<<28) && gpio.MODER==0x2AAAAAAA && spi.DR==0 && nops==2);
        spi.DR=word;
        uint32_t expected=(uint32_t)(word&0x7FFFU)<<(format==0 ? 1U : 6U);
        assert(BspEncoder_ReadAngle(resolutions[format])==expected);
        assert(gpio.BSRR==(1U<<12) && gpio.MODER==0xAAAAAAAA);
    }
    }
    puts("PASS STM32 encoder: 16/21-bit count scaling, full angle domain, CS and MOSI restore");
    return 0;
}
"""
            )
        else:
            encoder_source = (ROOT / "firmware/platform/bsp/mcu/at32f456/encoder_spi.c").read_text(
                encoding="utf-8"
            )
            frames = encoder_source[
                encoder_source.index("#define MT6835_FRAME_WORD_COUNT") : encoder_source.index(
                    "void McuEncoder_Init(void)"
                )
            ]
            source += (
                r"""
static struct {uint32_t clr,scr;} gpio;
static struct {uint32_t ctrl2,sts;} spi;
static struct {uint32_t clr;} dma;
static struct {uint32_t dtcnt,ctrl;} rx,tx;
#define GPIOB (&gpio)
#define SPI2 (&spi)
#define DMA2 (&dma)
#define DMA2_CHANNEL1 (&rx)
#define DMA2_CHANNEL2 (&tx)
#define SPI_I2S_BF_FLAG 128U
#define __DMB() ((void)0)
"""
                + frames
                + resources(board)
                + production(
                    board,
                    "encoder_spi",
                    "chip_select_delay",
                    "BspEncoder_Begin",
                    "BspEncoder_ReadAngle",
                    "BspEncoder_ReadCompletedFrame",
                )
                + r"""
int main(void) {
    assert(sizeof(Mt6835TxFrame)==6 && sizeof(Mt6835RxFrame)==6);
    assert(offsetof(Mt6835TxFrame,dummy)==2);
    assert(offsetof(Mt6835RxFrame,ignored)==0);
    assert(offsetof(Mt6835RxFrame,angle_high)==2);
    for(unsigned word=0;word<65536;word++) {
        rx.ctrl=tx.ctrl=spi.ctrl2=0x100;spi.sts=0;nops=0;
        BspEncoder_Begin();
        uint16_t transmitted[3];
        memcpy(transmitted,(const void *)&s_encoder_tx,sizeof(transmitted));
        assert(transmitted[0]==0xA003 && transmitted[1]==0 && transmitted[2]==0);
        assert(s_encoder_tx.command==0xA && s_encoder_tx.address==0x003);
        assert(gpio.clr==(1U<<12) && dma.clr==0xFF && rx.dtcnt==3 && tx.dtcnt==3);
        assert(rx.ctrl==0x101 && tx.ctrl==0x101 && spi.ctrl2==0x103 && nops==36);
        uint16_t tail=(uint16_t)(word^0xA55AU);
        uint16_t received[3]={0x1234,(uint16_t)word,tail};
        memcpy((void *)&s_encoder_rx,received,sizeof(received));
        rx.dtcnt=0;
        assert(s_encoder_rx.ignored==0x1234 && s_encoder_rx.angle_high==word);
        assert(s_encoder_rx.crc==(tail&0xFFU));
        assert(s_encoder_rx.status==((tail>>8)&0x07U));
        assert(s_encoder_rx.angle_low==(tail>>11));
        assert(BspEncoder_ReadAngle(BSP_ENCODER_16BIT)==word);
        assert(BspEncoder_ReadCompletedFrame()==((uint32_t)word<<16U|tail));
        assert(rx.ctrl==0x100 && tx.ctrl==0x100 && spi.ctrl2==0x100);
        assert(gpio.scr==(1U<<12) && nops==72);
    }
    for(uint32_t angle=0;angle<2097152U;angle++) {
        nops=0;
        BspEncoder_Begin();
        uint16_t tail=(uint16_t)(((angle&31U)<<11)|((angle>>5)&0x7FFU));
        uint16_t received[3]={0x1234,(uint16_t)(angle>>5),tail};
        memcpy((void *)&s_encoder_rx,received,sizeof(received));
        rx.dtcnt=0;
        assert(BspEncoder_ReadAngle(BSP_ENCODER_21BIT)==angle);
        assert(BspEncoder_ReadCompletedFrame()==((angle>>5U)<<16U|tail));
        assert(rx.ctrl==0x100 && tx.ctrl==0x100 && spi.ctrl2==0x100);
        assert(gpio.scr==(1U<<12) && nops==72);
    }
    puts("PASS AT32 encoder: all 2097152 angles, 16-bit compatibility, DMA fields, CS and release");
    return 0;
}
"""
            )
        run_c(args, f"encoder_{board}", source, board)


if __name__ == "__main__":
    main()
