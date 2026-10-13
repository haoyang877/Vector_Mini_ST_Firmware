/* APP/Loader共同的纯字节契约；不依赖HAL、结构填充或硬件地址解引用。 */
#include "boot_contract.h"
#include <string.h>

static uint32_t read_le32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void write_le32(uint8_t *p, uint32_t value)
{
    unsigned i;
    for (i = 0; i < 4; ++i)
    {
        p[i] = (uint8_t)(value >> (8 * i));
    }
}

uint32_t Boot_Crc32(const uint8_t *data, size_t size)
{
    uint32_t crc = UINT32_MAX;
    size_t i;
    unsigned bit;
    for (i = 0; i < size; ++i)
    {
        crc ^= data[i];
        for (bit = 0; bit < 8; ++bit)
        {
            crc = (crc >> 1) ^ ((0U - (crc & 1U)) & 0xEDB88320UL);
        }
    }
    return crc ^ UINT32_MAX;
}

void Boot_RecordEncode(const BootImage *image, uint8_t out[BOOT_RECORD_BYTES])
{
    memset(out, 0, BOOT_RECORD_BYTES);
    write_le32(out, 0x3252444CUL);
    out[4] = 1;
    out[6] = BOOT_RECORD_BYTES;
    write_le32(out + 8, BOOT_LAYOUT_ID);
    write_le32(out + 12, image->size);
    write_le32(out + 16, image->crc);
    write_le32(out + 20, image->version);
    write_le32(out + 24, BOOT_HARDWARE_ID);
    write_le32(out + 28, BOOT_PARAM_ABI);
    write_le32(out + 36, Boot_Crc32(out, 36));
    memcpy(out + 40, "BCOMMIT1", 8);
}

bool Boot_RecordDecode(const uint8_t raw[BOOT_RECORD_BYTES], BootImage *image)
{
    uint32_t size = read_le32(raw + 12);
    if (read_le32(raw) != 0x3252444CUL || raw[4] != 1 || raw[5] != 0 ||
        raw[6] != BOOT_RECORD_BYTES || raw[7] != 0 || read_le32(raw + 8) != BOOT_LAYOUT_ID ||
        read_le32(raw + 24) != BOOT_HARDWARE_ID || read_le32(raw + 28) != BOOT_PARAM_ABI ||
        read_le32(raw + 32) != 0 || read_le32(raw + 36) != Boot_Crc32(raw, 36) ||
        memcmp(raw + 40, "BCOMMIT1", 8) != 0 || size < 8 || size > BOOT_APP_CAPACITY ||
        (size & 7U) != 0)
    {
        return false;
    }
    image->size = size;
    image->crc = read_le32(raw + 16);
    image->version = read_le32(raw + 20);
    return true;
}

bool Boot_VectorsValid(const uint8_t vectors[8], uint32_t size)
{
    uint32_t sp = read_le32(vectors);
    uint32_t reset = read_le32(vectors + 4);
    uint32_t address = reset & ~1UL;
    return size >= 8 && size <= BOOT_APP_CAPACITY && (size & 7U) == 0 && sp >= 0x20000028UL &&
           sp <= 0x20008000UL && (sp & 7U) == 0 && (reset & 1U) != 0 && address >= BOOT_APP_BASE &&
           address - BOOT_APP_BASE < size;
}

void Boot_MailboxEncode(uint32_t node, uint32_t token, uint8_t out[BOOT_MAILBOX_BYTES])
{
    memset(out, 0, BOOT_MAILBOX_BYTES);
    write_le32(out, 0x314C4D42UL);
    out[4] = 1;
    out[6] = BOOT_MAILBOX_BYTES;
    write_le32(out + 8, 1);
    write_le32(out + 12, node);
    write_le32(out + 16, token);
    write_le32(out + 24, Boot_Crc32(out, 24));
    write_le32(out + 28, 0x54494D43UL);
}

bool Boot_MailboxDecode(const uint8_t raw[BOOT_MAILBOX_BYTES], uint32_t *node)
{
    uint32_t value = read_le32(raw + 12);
    if (read_le32(raw) != 0x314C4D42UL || raw[4] != 1 || raw[5] != 0 ||
        raw[6] != BOOT_MAILBOX_BYTES || raw[7] != 0 || read_le32(raw + 8) != 1 || value > 15 ||
        read_le32(raw + 20) != 0 || read_le32(raw + 24) != Boot_Crc32(raw, 24) ||
        read_le32(raw + 28) != 0x54494D43UL)
    {
        return false;
    }
    *node = value;
    return true;
}
