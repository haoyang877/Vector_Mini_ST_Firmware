"""离线验证实际编码器 SPI 分阶段读取、故障收尾与同步读一致；不访问硬件。"""

import argparse
from pathlib import Path
import subprocess

from run_position_servo_tests import NATIVE_INCLUDE_FLAGS, ROOT, function_source


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    source = (ROOT / "firmware/platform/stm32g4/bsp/encoder.c").read_text(encoding="utf-8")
    header = (ROOT / "firmware/platform/stm32g4/bsp/encoder.h").read_text(encoding="utf-8")
    stages = "\n".join(line for line in header.splitlines() if line.startswith("#define ENCODER_SAMPLE_"))
    fixture = r'''
#undef NDEBUG
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <limits.h>
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
typedef enum { ENCODER_READ_OK, ENCODER_READ_SPI_TIMEOUT } Encoder_ReadStatus;
typedef struct {
    uint16_t tle5012_angle_word, tle5012_safety_word, bad_frame_streak;
    uint8_t tle5012_crc_received, tle5012_crc_calculated;
    Encoder_ReadStatus read_status, read_status_latched;
    uint32_t read_error_count;
} Encoder_TypeDef;
enum {
    NO_FAILURE, COMMAND_TXE, COMMAND_RXNE, COMMAND_BSY,
    RESPONSE_TXE, RESPONSE_RXNE, RESPONSE_BSY, CLEANUP_BSY
};
static unsigned cs, hiz, begins, ends, command_txe, response_txe;
static unsigned command_rx, response_rx, bsy_pending, cleanups, fail;
static uint16_t response;
static void select_sensor(void) { assert(!cs); cs=1; begins++; }
static void release_sensor(void) { assert(cs && !hiz); cs=0; ends++; }
#define BRD_ENC_CS_ENABLE select_sensor()
#define BRD_ENC_CS_DISABLE release_sensor()
#define __NOP() ((void)0)
static void SPI2_MOSI_HiZ(void) { assert(cs && !hiz); hiz=1; }
static void SPI2_MOSI_RestoreAF(void) { assert(cs && hiz); hiz=0; }

/* 模拟寄存器随线上传输推进；实际起发/读取函数仍从源码提取。 */
static bool SPI_WaitFlag(SPI_TypeDef *s, uint32_t flag, uint32_t limit)
{
    assert(s == &registers && limit == ENC_SPI_XFER_SPIN_MAX && cs);
    if (flag == SPI_FLAG_TXE)
    {
        if (hiz) { response_txe++; return fail != RESPONSE_TXE; }
        command_txe++; return fail != COMMAND_TXE;
    }
    assert(flag == SPI_FLAG_RXNE && !bsy_pending);
    if (hiz)
    {
        assert(s->DR == 0U); response_rx++;
        if (fail == RESPONSE_RXNE) { return false; }
        s->DR = response;
    }
    else
    {
        assert(s->DR == 0x8021U); command_rx++;
        if (fail == COMMAND_RXNE) { return false; }
        s->DR = 0x1234U;
    }
    bsy_pending = 1U;
    return true;
}
static bool SPI_WaitBSYClear(SPI_TypeDef *s, uint32_t limit)
{
    assert(s == &registers && limit == ENC_SPI_XFER_SPIN_MAX && cs);
    if (bsy_pending)
    {
        bsy_pending = 0U;
        s->SR = SPI_FLAG_TXE;
        return fail != (hiz ? RESPONSE_BSY : COMMAND_BSY);
    }
    cleanups++;
    return fail != CLEANUP_BSY;
}
''' + stages + "\n" + function_source(source, "SPI_WaitFlag").replace(
        "SPI_WaitFlag", "real_wait_flag"
    ) + "\n" + function_source(source, "SPI_WaitBSYClear").replace(
        "SPI_WaitBSYClear", "real_wait_bsy_clear"
    ) + "\n" + "\n".join(function_source(source, name) for name in (
        "Encoder_MarkReadStatus", "SPI_Reg_ReadRx16", "SPI_Reg_StartTx16",
        "SPI_Reg_TxRx16", "Encoder_BeginSample", "Encoder_BeginReadback",
        "Encoder_ReadTle5012BFrame",
    )) + r'''
static void reset_bus(void)
{
    cs=hiz=begins=ends=command_txe=response_txe=command_rx=response_rx=bsy_pending=cleanups=0U;
    registers.CR1=SPI_CR1_SPE; registers.SR=SPI_FLAG_TXE; registers.DR=0x55U;
}
int main(void)
{
    unsigned sr, enabled, path, failure, word, cases=0U;
    /* 实际轮询函数遇到永久忙/无接收也必须有界返回，正常标志不消耗等待。 */
    for (sr=0; sr<256; sr++)
    {
        registers.SR=sr;
        assert(real_wait_flag(&registers,SPI_FLAG_TXE,340U) == ((sr & SPI_FLAG_TXE) != 0U));
        assert(real_wait_flag(&registers,SPI_FLAG_RXNE,0U) == ((sr & SPI_FLAG_RXNE) != 0U));
        assert(real_wait_bsy_clear(&registers,340U) == ((sr & SPI_FLAG_BSY) == 0U));
    }
    for (enabled=0; enabled<2; enabled++) for (sr=0; sr<256; sr++)
    {
        bool started;
        reset_bus(); registers.CR1=enabled?SPI_CR1_SPE:0U; registers.SR=sr;
        started=Encoder_BeginSample();
        assert(started == (enabled && (sr & (SPI_FLAG_TXE|SPI_FLAG_RXNE|SPI_FLAG_OVR|SPI_FLAG_BSY)) == SPI_FLAG_TXE));
        assert(begins == (unsigned)started && cs == (unsigned)started);
        assert(registers.DR == (started ? 0x8021U : 0x55U));
        if (!started)
        {
            assert(Encoder_BeginReadback(false) == ENCODER_SAMPLE_SYNC);
            assert(!cs && !hiz && !command_rx && !response_txe && !cleanups);
        }
        cases++;
    }
    /* 三条读路径逐一覆盖每个 16 位角度字，及所有起发/RX/BSY 失败点。 */
    for (path=0; path<3; path++) for (failure=0; failure<=CLEANUP_BSY; failure++)
    {
        if (path && failure == COMMAND_TXE) { continue; }
        for (word=0; word<65536U; word += failure ? 257U : 1U)
        {
            Encoder_TypeDef e={0}; uint16_t raw=0x1357U; uint8_t stage; bool ok;
            bool expected_ok = failure == NO_FAILURE || failure == CLEANUP_BSY;
            bool command_ok = failure != COMMAND_TXE && failure != COMMAND_RXNE && failure != COMMAND_BSY;
            reset_bus(); fail=failure; response=(uint16_t)word;
            e.read_error_count=11U; e.bad_frame_streak=2U;
            e.tle5012_angle_word=0x4567U;
            registers.CR1=path ? SPI_CR1_SPE : 0U;
            stage=(uint8_t)Encoder_BeginSample(); assert(stage == (path ? 1U : 0U));
            if (path == 2U)
            {
                stage=Encoder_BeginReadback(true);
                assert(stage == (command_ok && failure != RESPONSE_TXE ?
                    ENCODER_SAMPLE_RESPONSE_PENDING : ENCODER_SAMPLE_FAILED));
                assert(cs && begins == 1U && !ends && !cleanups && !response_rx);
                assert(hiz == (stage == ENCODER_SAMPLE_RESPONSE_PENDING));
                assert(e.read_error_count == 11U && raw == 0x1357U);
                /* 计算重叠期间，驱动不能重复起发或提前发布角度。 */
                registers.SR=SPI_FLAG_BSY;
                assert(!Encoder_BeginSample() && begins == 1U);
            }
            ok=Encoder_ReadTle5012BFrame(&e,&raw,stage);
            assert(ok == expected_ok && !cs && !hiz && begins == 1U && ends == 1U);
            assert(cleanups == 1U && !bsy_pending);
            assert(command_txe == (path ? 0U : 1U));
            assert(command_rx == (failure == COMMAND_TXE ? 0U : 1U));
            assert(response_txe == (command_ok ? 1U : 0U));
            assert(response_rx == (command_ok && failure != RESPONSE_TXE ? 1U : 0U));
            assert(e.read_status == (expected_ok ? ENCODER_READ_OK : ENCODER_READ_SPI_TIMEOUT));
            assert(e.read_error_count == (expected_ok ? 11U : 12U));
            assert(e.bad_frame_streak == (expected_ok ? 0U : 3U));
            assert(raw == (expected_ok ? (uint16_t)((word & 0x7fffU)<<1) : 0x1357U));
            assert(e.tle5012_angle_word == (expected_ok ? word : 0x4567U));
            assert(!e.tle5012_safety_word && !e.tle5012_crc_received && !e.tle5012_crc_calculated);
            cases++;
        }
    }
    printf("PASS %u encoder cases: sync/command/response parity, bounded failures, one status/CS/angle per transaction\n",cases);
    return 0;
}
'''
    c = out / "encoder_overlap.c"
    c.write_text(fixture, encoding="utf-8")
    exe = out / "encoder_overlap.exe"
    cc = [args.cc] + (["cc"] if Path(args.cc).stem == "zig" else []) + NATIVE_INCLUDE_FLAGS
    subprocess.run(cc + ["-std=c99", "-O2", "-Wall", "-Wextra", "-Werror", str(c), "-o", str(exe)], check=True)
    subprocess.run([str(exe)], check=True)


if __name__ == "__main__":
    main()
