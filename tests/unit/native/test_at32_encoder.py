"""核对两板真实Begin/ReadAngle：片选、MOSI释放、DMA与角度位宽。"""

from _bsp_test_support import arguments, production, resources, run_c


def main() -> None:
    """覆盖所有16位响应；不保留已删除的CRC、超时或软件状态接口。"""
    args = arguments(__doc__)
    for board in (1, 2):
        source = r"""
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
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
    for(unsigned word=0;word<65536;word++) {
        gpio.MODER=0xAAAAAAAA; spi.SR=3; nops=0;
        BspEncoder_Begin();
        assert(gpio.BSRR==(1U<<28) && gpio.MODER==0x2AAAAAAA && spi.DR==0 && nops==2);
        spi.DR=word;
        assert(BspEncoder_ReadAngle()==(uint16_t)((word&0x7FFFU)<<1));
        assert(gpio.BSRR==(1U<<12) && gpio.MODER==0xAAAAAAAA);
    }
    puts("PASS STM32 encoder: full angle domain, CS, MOSI release/restore and response start");
    return 0;
}
"""
            )
        else:
            source += (
                r"""
static struct {uint32_t clr,scr;} gpio;
static struct {uint32_t ctrl2,sts;} spi;
static struct {uint32_t clr;} dma;
static struct {uint32_t dtcnt,ctrl;} rx,tx;
static volatile uint16_t s_encoder_tx[3],s_encoder_rx[3];
#define GPIOB (&gpio)
#define SPI2 (&spi)
#define DMA2 (&dma)
#define DMA2_CHANNEL1 (&rx)
#define DMA2_CHANNEL2 (&tx)
#define SPI_I2S_BF_FLAG 128U
#define __DMB() ((void)0)
"""
                + resources(board)
                + production(
                    board,
                    "encoder_spi",
                    "chip_select_delay",
                    "BspEncoder_Begin",
                    "BspEncoder_ReadAngle",
                )
                + r"""
int main(void) {
    for(unsigned word=0;word<65536;word++) {
        rx.ctrl=tx.ctrl=spi.ctrl2=0x100;spi.sts=0;nops=0;
        BspEncoder_Begin();
        assert(s_encoder_tx[0]==0xA003 && s_encoder_tx[1]==0 && s_encoder_tx[2]==0);
        assert(gpio.clr==(1U<<12) && dma.clr==0xFF && rx.dtcnt==3 && tx.dtcnt==3);
        assert(rx.ctrl==0x101 && tx.ctrl==0x101 && spi.ctrl2==0x103 && nops==36);
        rx.dtcnt=0;s_encoder_rx[0]=0x1234;s_encoder_rx[1]=word;s_encoder_rx[2]=0xABCD;
        assert(BspEncoder_ReadAngle()==word);
        assert(rx.ctrl==0x100 && tx.ctrl==0x100 && spi.ctrl2==0x100);
        assert(gpio.scr==(1U<<12) && nops==72);
    }
    puts("PASS AT32 encoder: full angle domain, command, 3-word DMA, CS delay and release");
    return 0;
}
"""
            )
        run_c(args, f"encoder_{board}", source, board)


if __name__ == "__main__":
    main()
