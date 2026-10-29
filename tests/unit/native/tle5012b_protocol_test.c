#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "bsp_encoder.h"
#include "bsp_internal.h"
#include "encoder_spi.h"
#include "encoder_protocol.h"
uint32_t BspSystem_EnterCritical(void)
{
    return 0U;
}
void BspSystem_ExitCritical(uint32_t key)
{
    (void)key;
}
bool BspPwm_ArePhasesDisabled(void)
{
    return true;
}

/* 使用真实TLE协议模块，SPI桩仅返回线上字节；覆盖完整角度域和失败输出保持。 */
static unsigned s_begin_calls, s_complete_calls, s_readback_calls, s_init_calls, s_reads;
static uint16_t s_response;
static uint8_t s_stage;
static bool s_started, s_success, s_init_success;
static uint8_t s_config_failed_mask;
static uint16_t s_config_base;
static const uint16_t s_commands[6] = {0x8001U, 0x8061U, 0x8081U, 0x8091U, 0x8011U, 0x8001U};

bool EncoderSpi_Init(void)
{
    ++s_init_calls;
    return s_init_success;
}

bool EncoderSpi_Begin(const EncoderSpiRequest *request)
{
    assert(request->command == 0x8021U && request->response_bytes == 2U);
    ++s_begin_calls;
    return s_started;
}

uint8_t EncoderSpi_BeginReadback(bool started)
{
    assert(started);
    ++s_readback_calls;
    return 2U;
}

bool EncoderSpi_Complete(uint8_t stage, const EncoderSpiRequest *request, uint8_t *data)
{
    assert(request->command == 0x8021U && request->response_bytes == 2U && stage == s_stage);
    ++s_complete_calls;
    if (s_success)
    {
        data[0] = (uint8_t)(s_response >> 8);
        data[1] = (uint8_t)s_response;
    }
    return s_success;
}

EncoderSpiReadStatus EncoderSpi_Read(const EncoderSpiRequest *request, uint8_t *data)
{
    unsigned index = s_reads++;
    assert(index < 6U && request->command == s_commands[index] && request->response_bytes == 2U);
    if (s_config_failed_mask & (1U << index))
    {
        return ENCODER_SPI_READ_ERROR;
    }
    uint16_t value = (uint16_t)(s_config_base + index);
    data[0] = (uint8_t)(value >> 8);
    data[1] = (uint8_t)value;
    return ENCODER_SPI_READ_OK;
}

int main(void)
{
    assert(BspEncoder_GetType() == BSP_ENCODER_TLE5012B);
    assert(BspEncoder_Begin() == BSP_NOT_READY && s_begin_calls == 0U);
    assert(BspEncoder_CaptureConfig() == BSP_NOT_READY && s_reads == 0U);
    assert(!Encoder_Init() && s_init_calls == 1U && s_reads == 0U);
    assert(BspEncoder_Begin() == BSP_NOT_READY && s_begin_calls == 0U);
    s_init_success = true;
    s_config_base = 0x1234U;
    assert(Encoder_Init());
    assert(s_init_calls == 2U && s_reads == 6U);
    assert(!Encoder_Init() && s_init_calls == 2U && s_reads == 6U);
    for (unsigned mask = 0; mask < 64; mask++)
    {
        s_reads = 0U;
        s_config_failed_mask = 0U;
        s_config_base = 0x1234U;
        assert(BspEncoder_CaptureConfig() == BSP_OK);
        s_reads = 0U;
        s_config_failed_mask = (uint8_t)mask;
        s_config_base = 0xAB00U;
        assert(BspEncoder_CaptureConfig() == (mask ? BSP_IO_ERROR : BSP_OK));
        assert(s_reads == 6U);
        for (unsigned i = 0; i < 6; i++)
        {
            uint16_t word = 0x1357U;
            BspResult result = BspEncoder_ReadConfigWord((uint8_t)i, &word);
            assert(mask & (1U << i) ? result == BSP_NOT_READY && word == 0x1357U
                                    : result == BSP_OK && word == 0xAB00U + i);
        }
    }
    uint16_t untouched = 0x1357U;
    assert(BspEncoder_ReadConfigWord(6U, &untouched) == BSP_INVALID_ARGUMENT &&
           untouched == 0x1357U);
    for (unsigned pipeline = 0; pipeline < 3; pipeline++)
    {
        s_started = pipeline != 0;
        s_stage = (uint8_t)pipeline;
        s_success = true;
        assert(BspEncoder_Begin() == BSP_OK);
        if (pipeline == 2)
        {
            assert(BspEncoder_BeginReadback() == BSP_OK);
        }
        assert(BspEncoder_Complete(NULL) == BSP_INVALID_ARGUMENT);
        assert(BspEncoder_Complete(NULL) == BSP_NOT_READY);
        for (unsigned failure = 0; failure < 2; failure++)
            for (unsigned word = 0; word < 65536; word++)
            {
                BspEncoderSample sample = {0x1357U, 0x2468U}, before = sample;
                s_started = pipeline != 0;
                s_success = failure == 0;
                s_response = (uint16_t)word;
                s_begin_calls = s_readback_calls = s_complete_calls = 0U;
                assert(BspEncoder_Begin() == BSP_OK);
                assert(BspEncoder_Begin() == BSP_BUSY);
                assert(BspEncoder_CaptureConfig() == BSP_BUSY);
                if (pipeline == 2)
                {
                    assert(BspEncoder_BeginReadback() == BSP_OK);
                }
                BspResult result = s_success ? BSP_OK : BSP_TIMEOUT;
                assert(BspEncoder_Complete(&sample) == result);
                assert(s_begin_calls == 1 && s_complete_calls == 1 &&
                       s_readback_calls == (pipeline == 2));
                BspEncoderDiagnostics d;
                assert(BspEncoder_ReadDiagnostics(&d) == BSP_OK && d.sample_result == result);
                assert(!d.crc_checked);
                if (s_success)
                {
                    assert(sample.angle_counts == (uint16_t)((word & 0x7FFFU) << 1));
                    assert(d.frame_word == word && d.valid_mask == BSP_ENCODER_DIAG_FRAME_VALID);
                    assert(d.sample_sequence == sample.sequence);
                }
                else
                {
                    assert(memcmp(&sample, &before, sizeof(sample)) == 0 && d.valid_mask == 0U);
                }
            }
    }
    puts("PASS TLE5012B: 393216 samples, complete angle domain, 64 config masks, ownership, "
         "diagnostic validity and output retention");
    return 0;
}
