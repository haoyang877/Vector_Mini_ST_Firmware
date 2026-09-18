#include "cogging_map.h"
#include <math.h>
#include <stddef.h>
#include "crc32.h"

CoggingMapRecord CoggingMap;

uint32_t Cogging_EncoderSignature(uint8_t reverse,
                                  uint16_t electrical_zero,
                                  int32_t pole_pairs,
                                  const int16_t *encoder_lut)
{
    uint32_t identity[2] = {(uint32_t)electrical_zero | ((uint32_t)reverse << 16),
                            (uint32_t)pole_pairs};
    uint32_t crc = Crc32_Compute(identity, sizeof(identity), 0U);
    return Crc32_Compute(encoder_lut, COGGING_MAP_POINTS * sizeof(int16_t), crc);
}

static uint32_t RecordCrc(const CoggingMapRecord *r)
{
    /* CRC 覆盖 magic、encoder_signature、full_scale_a，再覆盖整张表。 */
    uint32_t crc = Crc32_Compute(r, (uint32_t)offsetof(CoggingMapRecord, crc32), 0U);
    return Crc32_Compute(r->iq_q15, sizeof(r->iq_q15), crc);
}

float CoggingMap_LookupA(const CoggingMapRecord *record, uint16_t position_q15)
{
    const uint32_t index = (uint32_t)position_q15 >> COGGING_MAP_INDEX_SHIFT;
    const float fraction =
        (float)(position_q15 & COGGING_MAP_INDEX_MASK) / (float)COGGING_MAP_SUBSTEPS;
    const float first = (float)record->iq_q15[index];
    const float second = (float)record->iq_q15[(index + 1U) & (COGGING_MAP_POINTS - 1U)];
    /* 表值按表内保存的满量程从 Q15 还原为 A；最后一点跨圈插值到第 0 点。 */
    return (first + (second - first) * fraction) * (record->full_scale_a / 32768.0f);
}

bool CoggingMap_Valid(const CoggingMapRecord *r, uint32_t signature)
{
    return r != NULL && r->magic == COGGING_MAP_MAGIC && r->encoder_signature == signature &&
           isfinite(r->full_scale_a) && r->full_scale_a > 0.0f && r->crc32 == RecordCrc(r);
}

bool CoggingMap_Build(const int16_t *fresh_q15,
                      float full_scale_a,
                      uint32_t signature,
                      CoggingMapRecord *record)
{
    int32_t sum = 0;
    float mean;
    if (fresh_q15 == NULL || record == NULL || !isfinite(full_scale_a) || full_scale_a <= 0.0f)
    {
        return false;
    }
    for (unsigned i = 0; i < COGGING_MAP_POINTS; ++i)
    {
        sum += fresh_q15[i];
    }
    mean = (float)sum / (float)COGGING_MAP_POINTS;
    /* 替换旧记录之前先校验每个转换值都在 int16 范围内。 */
    for (unsigned i = 0; i < COGGING_MAP_POINTS; ++i)
    {
        if (fabsf((float)fresh_q15[i] - mean) > 32767.0f)
        {
            return false;
        }
    }
    /* 写入期间 magic 保持 0：ISR 不会被半成品记录误认为有效表。 */
    record->magic = 0U;
    record->encoder_signature = signature;
    record->full_scale_a = full_scale_a;
    for (unsigned i = 0; i < COGGING_MAP_POINTS; ++i)
    {
        record->iq_q15[i] = (int16_t)roundf((float)fresh_q15[i] - mean);
    }
    record->magic = COGGING_MAP_MAGIC;
    record->crc32 = RecordCrc(record);
    return true;
}
