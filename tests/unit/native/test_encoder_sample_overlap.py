"""按层验证编码器采样：SPI 传输端口、传感器通道 driver 与角度层状态映射；不访问硬件。

夹具分三个可执行文件：
1. 传输端口（寄存器桩）：总线就绪判定、CS/MOSI 顺序、两类传输失败与资源释放；
2. 传感器通道（总线 seam 桩）：请求/读帧取值、异步与回退路径、角度解码与状态归类；
3. 角度层映射（通道 seam 桩）：通道状态到读取状态的映射、诊断字与角度传递、失败不覆盖。
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
    source = (ROOT / "firmware/platform/bsp/mcu/stm32g4/encoder_spi.c").read_text(encoding="utf-8")
    return (
        r"""
#undef NDEBUG
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "encoder_spi.h"
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
static unsigned cs, hiz, begins, ends, commands, receives, completes, read_starts, fail;
static uint16_t response;
static const EncoderSpiRequest request = {0x8021U, 2U};
static void select_sensor(void) { assert(!cs); cs = 1; begins++; }
static void release_sensor(void) { assert(cs && !hiz); cs = 0; ends++; }
#define ENCODER_SPI_CS_ENABLE select_sensor()
#define ENCODER_SPI_CS_DISABLE release_sensor()
static void spi_mosi_release(void) { assert(cs && !hiz); hiz = 1; }
static void spi_mosi_restore(void) { assert(cs && hiz); hiz = 0; }
static bool spi_wait_idle(SPI_TypeDef *s, uint32_t limit) {
    assert(s == &registers && limit == 340U); return true;
}
static uint16_t spi_read_word(SPI_TypeDef *s, bool *ok) {
    assert(s == &registers && cs);
    if (hiz) {
        receives++; *ok = fail != 2; return response;
    }
    assert(s->DR == 0x8021U);
    completes++; *ok = fail != 1; return 0;
}
static bool spi_start_word(SPI_TypeDef *s, uint16_t data) {
    assert(s == &registers && cs && hiz && data == 0U);
    read_starts++; return fail != 3;
}
static uint16_t spi_transfer_word(SPI_TypeDef *s, uint16_t data, bool *ok) {
    assert(s == &registers && cs);
    if (data == 0x8021U) { assert(!hiz); commands++; *ok = fail != 1; return 0; }
    assert(data == 0U && hiz); receives++; *ok = fail != 2; return response;
}
"""
        + function_source(source, "EncoderSpi_Begin")
        + "\n"
        + function_source(source, "EncoderSpi_BeginReadback")
        + "\n"
        + function_source(source, "EncoderSpi_Complete")
        + r"""
int main(void) {
    unsigned sr, enabled, pipeline, failure, word, cases = 0;
    for (enabled = 0; enabled < 2; enabled++) for (sr = 0; sr < 256; sr++) {
        bool started;
        registers.CR1 = enabled ? SPI_CR1_SPE : 0; registers.SR = sr; registers.DR = 0x55;
        cs = hiz = begins = ends = 0;
        started = EncoderSpi_Begin(&request);
        assert(started == (enabled && (sr & (SPI_FLAG_TXE|SPI_FLAG_RXNE|SPI_FLAG_OVR|SPI_FLAG_BSY)) == SPI_FLAG_TXE));
        assert(begins == (unsigned)started && cs == (unsigned)started);
        assert(registers.DR == (started ? 0x8021U : 0x55U));
        cases++;
    }
    for (pipeline = 0; pipeline < 3; pipeline++) for (failure = 0; failure < 4; failure++) {
        if (failure == 3 && pipeline != 2) continue;
        for (word = 0; word < 65536U; word += 257U) {
            uint8_t bytes[2] = {0x13U, 0x57U};
            uint16_t out;
            bool started, ok;
            uint8_t stage;
            cs = hiz = begins = ends = commands = receives = completes = read_starts = 0;
            fail = failure; response = (uint16_t)word;
            registers.CR1 = SPI_CR1_SPE; registers.SR = pipeline ? SPI_FLAG_TXE : SPI_FLAG_BSY;
            stage = started = EncoderSpi_Begin(&request);
            assert(started == (pipeline != 0));
            if (pipeline == 2) {
                stage = EncoderSpi_BeginReadback(started);
                assert(stage == (failure == 1 || failure == 3 ? ENCODER_SPI_READ_FAILED
                                                            : ENCODER_SPI_READ_RESPONSE_PENDING));
                assert(cs && (hiz == (failure != 1 && failure != 3)));
                assert(receives == 0); /* 此处仅发起响应，不能提前等读。 */
            } else if (!pipeline) {
                assert(EncoderSpi_BeginReadback(false) == ENCODER_SPI_READ_SYNC);
                assert(!cs && !hiz && completes == 0 && read_starts == 0);
            }
            /* 保护/观测器计算可在此重叠，但不得再次访问总线。 */
            ok = EncoderSpi_Complete(stage, &request, bytes);
            out = (uint16_t)(((uint16_t)bytes[0] << 8) | bytes[1]);
            assert(ok == (failure == 0) && !cs && !hiz && begins == 1 && ends == 1);
            assert(commands == (pipeline ? 0U : 1U) && completes == (unsigned)(pipeline != 0));
            assert(receives == (failure == 1 || failure == 3 ? 0U : 1U));
            assert(read_starts == (unsigned)(pipeline == 2 && failure != 1));
            assert(out == (failure ? 0x1357U : (uint16_t)word));
            cases++;
        }
    }
    printf("PASS %u encoder transport cases: sync/command/response, both RX failures, start failure, cleanup\n", cases);
    return 0;
}
"""
    )


def _mapping_source() -> str:
    source = (ROOT / "firmware/motor/feedback/angle_feedback.c").read_text(encoding="utf-8")
    return (
        r"""
#undef NDEBUG
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "bsp_encoder.h"
#include "bsp_internal.h"
typedef unsigned EncoderReadStatus;
#define ENCODER_READ_OK 0U
#define ENCODER_READ_SPI_TIMEOUT 1U
#define ENCODER_READ_CRC_MISMATCH 2U
#define ENCODER_READ_TLE_SYSTEM_ERROR 8U
typedef struct {
    uint16_t frame_word, safety_word;
    uint8_t crc_received, crc_calculated;
    EncoderReadStatus status;
} EncoderState;
static void encoder_mark_read_status(EncoderState *e, EncoderReadStatus status) {
    e->status = status;
}
static unsigned begin_calls;
static BspResult complete_status;
static BspEncoderSample complete_sample;
BspResult BspEncoder_Begin(void) { begin_calls++; return BSP_OK; }
BspResult BspEncoder_BeginReadback(void) { return BSP_OK; }
BspResult BspEncoder_Complete(BspEncoderSample *out) {
    if(complete_status==BSP_OK) { *out=complete_sample; }
    return complete_status;
}
BspResult BspEncoder_ReadDiagnostics(BspEncoderDiagnostics *out) {
    *out=(BspEncoderDiagnostics){.type=BSP_ENCODER_TLE5012B,.sample_result=complete_status,
        .sample_sequence=complete_sample.sequence,.valid_mask=BSP_ENCODER_DIAG_FRAME_VALID,
        .frame_word=0xBEEFU}; return BSP_OK;
}

"""
        + function_source(source, "Encoder_BeginSample")
        + "\n"
        + function_source(source, "Encoder_BeginReadback")
        + "\n"
        + function_source(source, "encoder_map_sensor_status")
        + "\n"
        + function_source(source, "encoder_read_frame")
        + r"""
static void expect(BspResult status, EncoderReadStatus expected, bool ok) {
    EncoderState e;
    uint16_t raw = 0x2468U;
    complete_status = status;
    complete_sample=(BspEncoderSample){.angle_counts=0x1234U,.sequence=1U};
    e.frame_word = 0U; e.safety_word = 0U; e.crc_received = 1U; e.crc_calculated = 1U;
    assert(encoder_read_frame(&e, &raw, 1U) == ok);
    assert(e.status == expected);
    if (ok) {
        assert(raw == 0x1234U && e.frame_word == 0xBEEFU && e.safety_word == 0U);
        assert(e.crc_received == 0U && e.crc_calculated == 0U);
    } else {
        assert(raw == 0x2468U); /* 失败不覆盖调用方缓存。 */
        assert(e.frame_word == 0U && e.safety_word == 0U);
    }
}
int main(void) {
    begin_calls = 0;
    assert(Encoder_BeginSample() && begin_calls == 1);
    assert(Encoder_BeginReadback(true) == 1U);
    expect(BSP_OK, ENCODER_READ_OK, true);
    expect(BSP_TIMEOUT, ENCODER_READ_SPI_TIMEOUT, false);
    expect(BSP_CRC_ERROR, ENCODER_READ_CRC_MISMATCH, false);
    expect(BSP_SENSOR_FAULT, ENCODER_READ_TLE_SYSTEM_ERROR, false);
    puts("PASS encoder mapping cases: begin passthrough, status mapping, diagnostics and raw angle transfer");
    return 0;
}
"""
    )


def _build(out: Path, name: str, source: str, cc: str, extra: list[str] | None = None) -> Path:
    fixture = out / f"{name}.c"
    source = source.replace(
        "../../../firmware/platform/bsp/mcu/stm32g4/encoder_spi.c",
        (ROOT / "firmware/platform/bsp/mcu/stm32g4/encoder_spi.c").as_posix(),
    )
    fixture.write_text(source, encoding="utf-8")
    exe = out / f"{name}.exe"
    compiler = [cc] + (["cc"] if Path(cc).stem == "zig" else [])
    subprocess.run(
        compiler
        + ["-std=c99", "-O2", "-UNDEBUG", "-Wall", "-Wextra", "-Werror", "-I", str(out)]
        + NATIVE_INCLUDE_FLAGS
        + [str(ROOT / "tests/unit/bsp_preemption_stubs.c")]
        + ["-I", str(ROOT / "firmware/platform/bsp/encoder")]
        + (extra or [])
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
    for name, source in (
        ("encoder_transport", _transport_source()),
        ("encoder_mapping", _mapping_source()),
    ):
        exe = _build(out, name, source, args.cc)
        subprocess.run([str(exe)], check=True)

    executable = _build(
        out,
        "encoder_tle_protocol",
        (ROOT / "tests/unit/native/tle5012b_protocol_test.c").read_text(encoding="utf-8"),
        args.cc,
        [
            str(ROOT / "firmware/platform/devices/encoder_tle5012b.c"),
            str(ROOT / "firmware/platform/bsp/bsp_encoder.c"),
        ],
    )
    subprocess.run([str(executable)], check=True)

    executable = _build(
        out,
        "encoder_stm32_registers",
        (ROOT / "tests/unit/native/stm32_encoder_test.c").read_text(encoding="utf-8"),
        args.cc,
        [
            "-I",
            str(ROOT / "tests/unit/native/stm32_encoder_fake"),
            str(ROOT / "firmware/platform/devices/encoder_tle5012b.c"),
            str(ROOT / "firmware/platform/bsp/bsp_encoder.c"),
        ],
    )
    subprocess.run([str(executable)], check=True)


if __name__ == "__main__":
    main()
