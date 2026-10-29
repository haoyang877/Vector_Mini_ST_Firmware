#include "bsp_encoder.h"
#include "bsp_system.h"
#include "bsp_pwm.h"
#include "bsp_config.h"
#include "bsp_internal.h"
#include "encoder_protocol.h"
#include <stddef.h>

typedef enum
{
    ENCODER_OWNER_NONE,
    ENCODER_OWNER_SAMPLE,
    ENCODER_OWNER_CONFIG
} EncoderOwner;
static EncoderOwner s_owner;
static bool s_initialized, s_async, s_advanced, s_diagnostics_seen;
static uint8_t s_stage, s_config_valid;
static BspResult s_stage_result;
static uint32_t s_sample_sequence, s_attempt_sequence;
static uint16_t s_config_words[BSP_ENCODER_CONFIG_WORDS];
static BspEncoderDiagnostics s_diagnostics;

bool Encoder_Init(void)
{
    if (!BSP_ENABLED_ROTOR_ENCODER)
    {
        return true;
    }
    if (s_initialized || !EncoderSpi_Init())
    {
        return false;
    }
    s_initialized = true;
    (void)BspEncoder_CaptureConfig();
    return true;
}

BspResult BspEncoder_Begin(void)
{
    if (!BSP_ENABLED_ROTOR_ENCODER)
    {
        return BSP_NOT_SUPPORTED;
    }
    uint32_t key = BspSystem_EnterCritical();
    BspResult result = !s_initialized                  ? BSP_NOT_READY
                       : s_owner != ENCODER_OWNER_NONE ? BSP_BUSY
                                                       : BSP_OK;
    if (result == BSP_OK)
    {
        s_owner = ENCODER_OWNER_SAMPLE;
        s_advanced = false;
        s_stage_result = BSP_OK;
        /* 同步回退也持有本拍事务，Begin失败不等于未取得所有权。 */
        s_async = EncoderSpi_Begin(&g_encoder_protocol.sample);
        s_stage = s_async ? ENCODER_SPI_READ_COMMAND_PENDING : ENCODER_SPI_READ_SYNC;
    }
    BspSystem_ExitCritical(key);
    return result;
}

BspResult BspEncoder_BeginReadback(void)
{
    if (!BSP_ENABLED_ROTOR_ENCODER)
    {
        return BSP_NOT_SUPPORTED;
    }
    if (s_owner != ENCODER_OWNER_SAMPLE)
    {
        return BSP_NOT_READY;
    }
    if (s_advanced)
    {
        s_stage_result = BSP_FRAME_ERROR;
        return BSP_FRAME_ERROR;
    }
    s_advanced = true;
    s_stage = EncoderSpi_BeginReadback(s_async);
    s_stage_result = s_stage == ENCODER_SPI_READ_FAILED ? BSP_TIMEOUT : BSP_OK;
    return s_stage_result;
}

BspResult BspEncoder_Complete(BspEncoderSample *sample)
{
    if (!BSP_ENABLED_ROTOR_ENCODER)
    {
        return BSP_NOT_SUPPORTED;
    }
    if (s_owner != ENCODER_OWNER_SAMPLE)
    {
        return BSP_NOT_READY;
    }
    uint8_t data[4] = {0};
    bool received = EncoderSpi_Complete(s_stage, &g_encoder_protocol.sample, data);
    EncoderDecodedSample decoded = {0};
    BspResult result = received ? EncoderProtocol_Decode(data, &decoded) : BSP_TIMEOUT;
    if (s_stage_result != BSP_OK)
    {
        result = s_stage_result;
    }
    if (sample == NULL)
    {
        result = BSP_INVALID_ARGUMENT;
    }
    uint32_t key = BspSystem_EnterCritical();
    if (result == BSP_OK)
    {
        *sample = (BspEncoderSample){decoded.angle_counts, ++s_sample_sequence};
    }
    s_diagnostics = (BspEncoderDiagnostics){g_encoder_protocol.type,
                                            result,
                                            ++s_attempt_sequence,
                                            result == BSP_OK ? s_sample_sequence : 0U,
                                            decoded.valid_mask,
                                            decoded.frame_word,
                                            decoded.safety_word,
                                            decoded.crc_checked};
    s_diagnostics_seen = true;
    s_owner = ENCODER_OWNER_NONE;
    BspSystem_ExitCritical(key);
    return result;
}

BspEncoderType BspEncoder_GetType(void)
{
    return BSP_ENABLED_ROTOR_ENCODER ? g_encoder_protocol.type : BSP_ENCODER_NONE;
}

BspResult BspEncoder_ReadDiagnostics(BspEncoderDiagnostics *diagnostics)
{
    if (diagnostics == NULL)
    {
        return BSP_INVALID_ARGUMENT;
    }
    if (!BSP_ENABLED_ROTOR_ENCODER)
    {
        return BSP_NOT_SUPPORTED;
    }
    uint32_t key = BspSystem_EnterCritical();
    BspResult result = s_diagnostics_seen ? BSP_OK : BSP_NOT_READY;
    if (result == BSP_OK)
    {
        *diagnostics = s_diagnostics;
    }
    BspSystem_ExitCritical(key);
    return result;
}

BspResult BspEncoder_CaptureConfig(void)
{
    if (!BSP_ENABLED_ROTOR_ENCODER)
    {
        return BSP_NOT_SUPPORTED;
    }
    uint32_t key = BspSystem_EnterCritical();
    BspResult result = !s_initialized                  ? BSP_NOT_READY
                       : !BspPwm_ArePhasesDisabled()   ? BSP_REJECTED
                       : s_owner != ENCODER_OWNER_NONE ? BSP_BUSY
                                                       : BSP_OK;
    if (result == BSP_OK)
    {
        s_owner = ENCODER_OWNER_CONFIG;
        s_config_valid = 0U;
    }
    BspSystem_ExitCritical(key);
    if (result != BSP_OK)
    {
        return result;
    }
    for (uint8_t index = 0U; index < BSP_ENCODER_CONFIG_WORDS; ++index)
    {
        uint8_t data[2];
        EncoderSpiReadStatus status = EncoderSpi_Read(&g_encoder_protocol.config[index], data);
        if (status == ENCODER_SPI_READ_OK)
        {
            s_config_words[index] = g_encoder_protocol.config_bytes == 1U
                                        ? data[0]
                                        : (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
            s_config_valid |= (uint8_t)(1U << index);
        }
        else
        {
            result = BSP_IO_ERROR;
        }
    }
    key = BspSystem_EnterCritical();
    s_owner = ENCODER_OWNER_NONE;
    BspSystem_ExitCritical(key);
    return result;
}

BspResult BspEncoder_ReadConfigWord(uint8_t index, uint16_t *word)
{
    if (word == NULL || index >= BSP_ENCODER_CONFIG_WORDS)
    {
        return BSP_INVALID_ARGUMENT;
    }
    if (!BSP_ENABLED_ROTOR_ENCODER)
    {
        return BSP_NOT_SUPPORTED;
    }
    if ((s_config_valid & (1U << index)) == 0U)
    {
        return BSP_NOT_READY;
    }
    *word = s_config_words[index];
    return BSP_OK;
}
