"""按层验证编码器采样：SPI 传输端口与帧协议编排；不访问硬件。

夹具分两个可执行文件：
1. 传输端口（寄存器桩）：总线就绪判定、CS/MOSI 顺序、两类传输失败与资源释放；
2. 协议编排（传输 seam 桩）：请求/读帧取值、异步与回退路径、角度解码与状态映射。
"""

import sys as _sys
from pathlib import Path as _Path

_sys.path.insert(0, str(_Path(__file__).resolve().parents[3] / "tools"))
import argparse
import subprocess
from pathlib import Path

from project_paths import NATIVE_INCLUDE_FLAGS, ROOT
from run_position_servo_tests import function_source


def _transport_source() -> str:
    source = (ROOT / "firmware/platform/stm32g4/ports/motor/encoder_spi_stm32g4.c").read_text(
        encoding="utf-8"
    )
    return (
        r"""
#undef NDEBUG
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#define __IO volatile
#define __NOP() ((void)0)
#define SPI_CR1_SPE 64U
#define SPI_FLAG_TXE 2U
#define SPI_FLAG_RXNE 1U
#define SPI_FLAG_BSY 128U
#define SPI_FLAG_OVR 64U
#define ENC_SPI_XFER_SPIN_MAX 340U
typedef struct { uint32_t CR1, SR, DR; } SPI_TypeDef;
static SPI_TypeDef registers;
typedef struct { SPI_TypeDef *Instance; } SPI_HandleTypeDef;
static SPI_HandleTypeDef hspi2 = { &registers };
#define ENCODER_SPI_HANDLE hspi2
static unsigned cs, hiz, begins, ends, commands, receives, completes, fail;
static uint16_t response;
static void select_sensor(void) { assert(!cs); cs = 1; begins++; }
static void release_sensor(void) { assert(cs && !hiz); cs = 0; ends++; }
#define ENCODER_SPI_CS_ENABLE select_sensor()
#define ENCODER_SPI_CS_DISABLE release_sensor()
static void SPI2_MOSI_HiZ(void) { assert(cs && !hiz); hiz = 1; }
static void SPI2_MOSI_RestoreAF(void) { assert(cs && hiz); hiz = 0; }
static bool SPI_WaitBSYClear(SPI_TypeDef *s, uint32_t limit) {
    assert(s == &registers && limit == 340U); return true;
}
static uint16_t SPI_Reg_ReadRx16(SPI_TypeDef *s, bool *ok) {
    assert(s == &registers && cs && !hiz && s->DR == 0x8021U);
    completes++; *ok = fail != 1; return 0;
}
static uint16_t SPI_Reg_TxRx16(SPI_TypeDef *s, uint16_t data, bool *ok) {
    assert(s == &registers && cs);
    if (data == 0x8021U) { assert(!hiz); commands++; *ok = fail != 1; return 0; }
    assert(data == 0U && hiz); receives++; *ok = fail != 2; return response;
}
"""
        + function_source(source, "encoder_spi_read_begin")
        + "\n"
        + function_source(source, "encoder_spi_read_complete")
        + r"""
int main(void) {
    unsigned sr, enabled, pipeline, failure, word, cases = 0;
    for (enabled = 0; enabled < 2; enabled++) for (sr = 0; sr < 256; sr++) {
        bool started;
        registers.CR1 = enabled ? SPI_CR1_SPE : 0; registers.SR = sr; registers.DR = 0x55;
        cs = hiz = begins = ends = 0;
        started = encoder_spi_read_begin(0x8021U);
        assert(started == (enabled && (sr & (SPI_FLAG_TXE|SPI_FLAG_RXNE|SPI_FLAG_OVR|SPI_FLAG_BSY)) == SPI_FLAG_TXE));
        assert(begins == (unsigned)started && cs == (unsigned)started);
        assert(registers.DR == (started ? 0x8021U : 0x55U));
        cases++;
    }
    for (pipeline = 0; pipeline < 2; pipeline++) for (failure = 0; failure < 3; failure++) {
        for (word = 0; word < 65536U; word += 257U) {
            uint16_t out = 0x1357U;
            bool started, ok;
            cs = hiz = begins = ends = commands = receives = completes = 0;
            fail = failure; response = (uint16_t)word;
            registers.CR1 = SPI_CR1_SPE; registers.SR = pipeline ? SPI_FLAG_TXE : SPI_FLAG_BSY;
            started = encoder_spi_read_begin(0x8021U);
            assert(started == (bool)pipeline);
            /* 采样/保护可在此期间执行，但不得访问同一总线。 */
            ok = encoder_spi_read_complete(started, 0x8021U, 0U, &out);
            assert(ok == (failure == 0) && !cs && !hiz && begins == 1 && ends == 1);
            assert(commands == (pipeline ? 0U : 1U) && completes == pipeline);
            assert(receives == (failure == 1 ? 0U : 1U));
            assert(out == (failure ? 0x1357U : (uint16_t)word));
            cases++;
        }
    }
    printf("PASS %u encoder transport cases: bus-ready check, CS/HiZ order, fallback, both failures, cleanup\n", cases);
    return 0;
}
"""
    )


def _protocol_source() -> str:
    source = (ROOT / "firmware/platform/stm32g4/bsp/encoder.c").read_text(encoding="utf-8")
    return (
        r"""
#undef NDEBUG
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#define ENCODER_REQUEST_READ_ANGLE 0x8021U
#define ENCODER_READ_FRAME 0x0000U
typedef struct {
    uint16_t tle5012_angle_word, tle5012_safety_word;
    uint8_t tle5012_crc_received, tle5012_crc_calculated;
    unsigned status;
} Encoder_TypeDef;
enum { ENCODER_READ_OK, ENCODER_READ_SPI_TIMEOUT };
static void Encoder_MarkReadStatus(Encoder_TypeDef *e, unsigned status) { e->status = status; }
static unsigned begin_calls, complete_calls;
static unsigned last_begin_frame, last_request_frame, last_read_frame;
static bool begin_result, complete_result, seen_begin_ok;
static uint16_t complete_response;
bool encoder_spi_read_begin(uint16_t request_frame) {
    begin_calls++; last_begin_frame = request_frame; return begin_result;
}
bool encoder_spi_read_complete(bool begin_ok, uint16_t request_frame, uint16_t read_frame,
                               uint16_t *word) {
    complete_calls++; seen_begin_ok = begin_ok;
    last_request_frame = request_frame; last_read_frame = read_frame;
    if (!complete_result) return false;
    *word = complete_response; return true;
}
"""
        + function_source(source, "Encoder_BeginSample")
        + "\n"
        + function_source(source, "Encoder_ReadTle5012BFrame")
        + r"""
static void reset(unsigned pipeline, unsigned fail) {
    begin_calls = complete_calls = 0;
    last_begin_frame = last_request_frame = last_read_frame = 0;
    begin_result = pipeline != 0; complete_result = fail == 0; seen_begin_ok = false;
}
int main(void) {
    Encoder_TypeDef e;
    uint16_t raw;
    unsigned word, cases = 0;
    for (unsigned pipeline = 0; pipeline < 2; pipeline++) {
        for (unsigned fail = 0; fail < 2; fail++) {
            for (word = 0; word < 65536U; word += 257U) {
                memset(&e, 0, sizeof(e)); raw = 0x1357U;
                reset(pipeline, fail); complete_response = (uint16_t)word;
                assert(Encoder_BeginSample() == (bool)pipeline);
                assert(begin_calls == 1 && last_begin_frame == 0x8021U);
                assert(Encoder_ReadTle5012BFrame(&e, &raw, pipeline != 0) == (fail == 0));
                assert(complete_calls == 1 && seen_begin_ok == (pipeline != 0));
                assert(last_request_frame == 0x8021U && last_read_frame == 0U);
                if (fail == 0) {
                    assert(raw == (uint16_t)((word & 0x7FFFU) << 1U));
                    assert(e.tle5012_angle_word == (uint16_t)word);
                    assert(e.tle5012_safety_word == 0U && e.tle5012_crc_received == 0U);
                    assert(e.status == ENCODER_READ_OK);
                } else {
                    assert(raw == 0x1357U && e.status == ENCODER_READ_SPI_TIMEOUT);
                }
                cases++;
            }
        }
    }
    printf("PASS %u encoder protocol cases: request/read frame, async and fallback paths, decode, timeout status\n", cases);
    return 0;
}
"""
    )


def _build(out: Path, name: str, source: str, cc: str) -> Path:
    fixture = out / f"{name}.c"
    fixture.write_text(source, encoding="utf-8")
    exe = out / f"{name}.exe"
    compiler = [cc] + (["cc"] if Path(cc).stem == "zig" else [])
    subprocess.run(
        compiler
        + ["-std=c99", "-O2", "-UNDEBUG", "-Wall", "-Wextra", "-Werror", "-I", str(out)]
        + NATIVE_INCLUDE_FLAGS
        + [str(fixture), "-o", str(exe)],
        check=True,
    )
    return exe


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    (out / "main.h").write_text("#pragma once\n#include <stdint.h>\n")
    for name, source in (
        ("encoder_transport", _transport_source()),
        ("encoder_protocol", _protocol_source()),
    ):
        exe = _build(out, name, source, args.cc)
        subprocess.run([str(exe)], check=True)


if __name__ == "__main__":
    main()
