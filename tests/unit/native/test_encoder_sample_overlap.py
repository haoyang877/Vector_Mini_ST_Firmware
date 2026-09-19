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


def _channel_source() -> str:
    source = (ROOT / "firmware/platform/stm32g4/ports/motor/encoder_tle5012b.c").read_text(
        encoding="utf-8"
    )
    return (
        r"""
#undef NDEBUG
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "encoder_sensor.h"
#define TLE5012B_REQUEST_READ_ANGLE 0x8021U
#define TLE5012B_READ_FRAME 0x0000U
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
static void encoder_spi_init(void) {}
"""
        + function_source(source, "encoder_sensor_init")
        + "\n"
        + function_source(source, "encoder_sensor_begin")
        + "\n"
        + function_source(source, "encoder_sensor_complete")
        + "\n"
        + function_source(source, "encoder_sensor_type")
        + r"""
static void reset(unsigned pipeline, unsigned fail) {
    begin_calls = complete_calls = 0;
    last_begin_frame = last_request_frame = last_read_frame = 0;
    begin_result = pipeline != 0; complete_result = fail == 0; seen_begin_ok = false;
}
int main(void) {
    EncoderSensorSample s;
    unsigned word, cases = 0;
    assert(encoder_sensor_type() == ENCODER_SENSOR_TYPE_TLE5012B);
    for (unsigned pipeline = 0; pipeline < 2; pipeline++) {
        for (unsigned fail = 0; fail < 2; fail++) {
            for (word = 0; word < 65536U; word += 257U) {
                s.angle_q15 = 0x1357U; s.status = ENCODER_SENSOR_SENSOR_FAULT;
                reset(pipeline, fail); complete_response = (uint16_t)word;
                assert(encoder_sensor_begin() == (bool)pipeline);
                assert(begin_calls == 1 && last_begin_frame == 0x8021U);
                assert(encoder_sensor_complete(pipeline != 0, &s)
                       == (fail == 0 ? ENCODER_SENSOR_OK : ENCODER_SENSOR_BUS_TIMEOUT));
                assert(complete_calls == 1 && seen_begin_ok == (pipeline != 0));
                assert(last_request_frame == 0x8021U && last_read_frame == 0U);
                if (fail == 0) {
                    assert(s.angle_q15 == (uint16_t)((word & 0x7FFFU) << 1U));
                    assert(s.frame_word == (uint16_t)word);
                    assert(s.safety_word == 0U && s.crc_ok);
                } else {
                    assert(s.angle_q15 == 0x1357U);
                }
                cases++;
            }
        }
    }
    printf("PASS %u encoder channel cases: TLE request/read frame, async and fallback, decode, bus-timeout status\n", cases);
    return 0;
}
"""
    )


def _mapping_source() -> str:
    source = (ROOT / "firmware/platform/stm32g4/bsp/encoder.c").read_text(encoding="utf-8")
    return (
        r"""
#undef NDEBUG
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "encoder_sensor.h"
typedef unsigned Encoder_ReadStatus;
#define ENCODER_READ_OK 0U
#define ENCODER_READ_SPI_TIMEOUT 1U
#define ENCODER_READ_CRC_MISMATCH 2U
#define ENCODER_READ_TLE_SYSTEM_ERROR 8U
typedef struct {
    uint16_t frame_word, safety_word;
    uint8_t crc_received, crc_calculated;
    Encoder_ReadStatus status;
} Encoder_TypeDef;
static void Encoder_MarkReadStatus(Encoder_TypeDef *e, Encoder_ReadStatus status) {
    e->status = status;
}
static unsigned begin_calls;
static EncoderSensorStatus complete_status;
static EncoderSensorSample complete_sample;
bool encoder_sensor_begin(void) { begin_calls++; return true; }
EncoderSensorStatus encoder_sensor_complete(bool started, EncoderSensorSample *out) {
    (void)started;
    *out = complete_sample;
    return complete_status;
}
"""
        + function_source(source, "Encoder_BeginSample")
        + "\n"
        + function_source(source, "Encoder_MapSensorStatus")
        + "\n"
        + function_source(source, "Encoder_ReadFrame")
        + r"""
static void expect(EncoderSensorStatus status, Encoder_ReadStatus expected, bool ok) {
    Encoder_TypeDef e;
    uint16_t raw = 0x2468U;
    complete_status = status;
    complete_sample.status = status;
    complete_sample.angle_q15 = 0x1234U;
    complete_sample.frame_word = 0xBEEFU;
    complete_sample.safety_word = 0x0001U;
    complete_sample.crc_ok = true;
    e.frame_word = 0U; e.safety_word = 0U; e.crc_received = 1U; e.crc_calculated = 1U;
    assert(Encoder_ReadFrame(&e, &raw, false) == ok);
    assert(e.status == expected);
    if (ok) {
        assert(raw == 0x1234U && e.frame_word == 0xBEEFU && e.safety_word == 0x0001U);
        assert(e.crc_received == 0U && e.crc_calculated == 0U);
    } else {
        assert(raw == 0x2468U); /* 失败不覆盖调用方缓存。 */
        assert(e.frame_word == 0U && e.safety_word == 0U);
    }
}
int main(void) {
    begin_calls = 0;
    assert(Encoder_BeginSample() && begin_calls == 1);
    expect(ENCODER_SENSOR_OK, ENCODER_READ_OK, true);
    expect(ENCODER_SENSOR_BUS_TIMEOUT, ENCODER_READ_SPI_TIMEOUT, false);
    expect(ENCODER_SENSOR_FRAME_ERROR, ENCODER_READ_CRC_MISMATCH, false);
    expect(ENCODER_SENSOR_SENSOR_FAULT, ENCODER_READ_TLE_SYSTEM_ERROR, false);
    puts("PASS encoder mapping cases: begin passthrough, status mapping, diagnostics and raw angle transfer");
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
        ("encoder_channel", _channel_source()),
        ("encoder_mapping", _mapping_source()),
    ):
        exe = _build(out, name, source, args.cc)
        subprocess.run([str(exe)], check=True)


if __name__ == "__main__":
    main()
