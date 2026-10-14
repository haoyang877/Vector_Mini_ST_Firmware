/* 停止等待升级；前台串行所有者，错误请求不能擦写Flash。 */
#include "loader_core.h"
#include "loader_port.h"
#include "loader_proto.h"
#include <string.h>

enum
{
    IDLE,
    PREPARED,
    RECEIVING,
    VERIFIED,
    ERROR_STATE
};
enum
{
    OK,
    BAD_FRAME,
    BAD_CRC,
    BAD_STATE,
    BAD_OFFSET,
    BAD_SIZE,
    FLASH_ERROR,
    VERIFY_FAIL,
    BAD_OPCODE,
    BAD_SESSION,
    BUSY,
    CONFLICT,
    BAD_IDENTITY,
    BAD_COMPATIBILITY
};

static bool image_valid(const BootImage *image)
{
    const uint8_t *app = LoaderPort_App();
    return Boot_VectorsValid(app, image->size) && Boot_Crc32(app, image->size) == image->crc;
}

bool LoaderCore_BootValid(void)
{
    uint8_t raw[BOOT_RECORD_BYTES];
    BootImage image;
    LoaderPort_ReadRecord(raw);
    return Boot_RecordDecode(raw, &image) && image_valid(&image);
}

void LoaderCore_Init(LoaderCore *core, uint8_t node)
{
    memset(core, 0, sizeof(*core));
    core->node = node;
}

void LoaderCore_Tick(LoaderCore *core, uint32_t now_ms)
{
    if (core->session != 0 && !core->activate_pending &&
        (uint32_t)(now_ms - core->last_activity_ms) > 30000UL)
    {
        uint8_t node = core->node;
        LoaderCore_Init(core, node);
    }
}

static uint16_t process(LoaderCore *c, const ldr_request_t *r, uint8_t *d, uint16_t *n)
{
    uint8_t uid[12];
    uint32_t offset, length;
    uint8_t record[BOOT_RECORD_BYTES];
    *n = 0;
    if (r->opcode <= LDR_OP_UID && r->opcode >= LDR_OP_INFO)
    {
        if (r->plen != 4 || r->session != 0)
        {
            return BAD_FRAME;
        }
        memcpy(d, r->payload, 4);
        *n = 4;
        if (r->opcode == LDR_OP_INFO)
        {
            ldr_put_u32(d + 4, BOOT_LOADER_VERSION);
            ldr_put_u32(d + 8, BOOT_APP_BASE);
            ldr_put_u32(d + 12, BOOT_APP_CAPACITY);
            ldr_put_u32(d + 16, BOOT_LAYOUT_ID);
            ldr_put_u32(d + 20, BOOT_HARDWARE_ID);
            ldr_put_u32(d + 24, BOOT_PARAM_ABI);
            d[28] = c->node;
            memset(d + 29, 0, 3);
            ldr_put_u32(d + 32, LoaderPort_ParamCrc());
            *n = 36;
        }
        else if (r->opcode == LDR_OP_STATUS)
        {
            d[4] = c->state;
            d[5] = c->last_error;
            d[6] = d[7] = 0;
            ldr_put_u32(d + 8, c->offset);
            ldr_put_u32(d + 12, c->image.size);
            ldr_put_u32(d + 16, c->image.crc);
            ldr_put_u32(d + 20, c->session);
            *n = 24;
        }
        else
        {
            LoaderPort_Uid(d + 4);
            *n = 16;
        }
        return OK;
    }
    if (r->opcode == LDR_OP_BEGIN)
    {
        if (r->plen != 40 || r->session == 0)
        {
            return BAD_FRAME;
        }
        if (c->session != 0)
        {
            return BUSY;
        }
        LoaderPort_Uid(uid);
        if (memcmp(uid, r->payload + 28, 12) != 0)
        {
            return BAD_IDENTITY;
        }
        if (ldr_get_u16(r->payload + 12) != 1 || ldr_get_u16(r->payload + 14) != 0 ||
            ldr_get_u32(r->payload + 16) != BOOT_LAYOUT_ID ||
            ldr_get_u32(r->payload + 20) != BOOT_HARDWARE_ID ||
            ldr_get_u32(r->payload + 24) != BOOT_PARAM_ABI)
        {
            return BAD_COMPATIBILITY;
        }
        length = ldr_get_u32(r->payload);
        if (length < 8 || length > BOOT_APP_CAPACITY || (length & 7U) != 0)
        {
            return BAD_SIZE;
        }
        c->image.size = length;
        c->image.crc = ldr_get_u32(r->payload + 4);
        c->image.version = ldr_get_u32(r->payload + 8);
        c->session = r->session;
        c->state = PREPARED;
        c->offset = 0;
        c->activate_pending = false;
        ldr_put_u32(d, BOOT_APP_BASE);
        *n = 4;
        return OK;
    }
    if (r->opcode < LDR_OP_ERASE || r->opcode > LDR_OP_ABORT)
    {
        return BAD_OPCODE;
    }
    if (r->session == 0 || r->session != c->session)
    {
        return BAD_SESSION;
    }
    if (r->seq <= c->last_sequence)
    {
        return CONFLICT;
    }
    if (r->opcode != LDR_OP_PROGRAM && r->plen != 0)
    {
        return BAD_FRAME;
    }
    switch (r->opcode)
    {
    case LDR_OP_ERASE:
        if (c->state != PREPARED)
        {
            return BAD_STATE;
        }
        if (!LoaderPort_EraseRecord() || !LoaderPort_EraseApp())
        {
            c->state = ERROR_STATE;
            return FLASH_ERROR;
        }
        c->state = RECEIVING;
        return OK;
    case LDR_OP_PROGRAM:
        if (c->state != RECEIVING)
        {
            return BAD_STATE;
        }
        if (r->plen < 12 || r->plen > 44)
        {
            return BAD_SIZE;
        }
        offset = ldr_get_u32(r->payload);
        length = r->plen - 4U;
        if ((length & 7U) != 0 || length > LDR_MAX_CHUNK || length > c->image.size ||
            offset > c->image.size - length)
        {
            return BAD_SIZE;
        }
        if ((offset & 7U) != 0 || length > BOOT_PAGE_BYTES - offset % BOOT_PAGE_BYTES ||
            offset > c->offset)
        {
            return BAD_OFFSET;
        }
        if (offset < c->offset)
        {
            if (length > c->offset - offset ||
                memcmp(LoaderPort_App() + offset, r->payload + 4, length) != 0)
            {
                return CONFLICT;
            }
        }
        else
        {
            if (!LoaderPort_Program(offset, r->payload + 4, length))
            {
                c->state = ERROR_STATE;
                return FLASH_ERROR;
            }
            c->offset += length;
        }
        ldr_put_u32(d, c->offset);
        *n = 4;
        return OK;
    case LDR_OP_VERIFY:
        if (c->state != RECEIVING || c->offset != c->image.size)
        {
            return BAD_STATE;
        }
        if (!image_valid(&c->image))
        {
            c->state = ERROR_STATE;
            return VERIFY_FAIL;
        }
        c->state = VERIFIED;
        ldr_put_u32(d, c->image.crc);
        *n = 4;
        return OK;
    case LDR_OP_ACTIVATE:
        if (c->state != VERIFIED)
        {
            return BAD_STATE;
        }
        if (!image_valid(&c->image))
        {
            c->state = ERROR_STATE;
            return VERIFY_FAIL;
        }
        Boot_RecordEncode(&c->image, record);
        if (!LoaderPort_Commit(record))
        {
            c->state = ERROR_STATE;
            return FLASH_ERROR;
        }
        c->activate_pending = true;
        return OK;
    case LDR_OP_ABORT:
        c->state = IDLE;
        c->session = 0;
        c->offset = 0;
        c->activate_pending = false;
        return OK;
    default:
        return BAD_OPCODE;
    }
}

uint8_t LoaderCore_Request(
    LoaderCore *c, const uint8_t *raw, uint8_t size, uint32_t now_ms, uint8_t reply[64])
{
    ldr_request_t r;
    uint8_t data[46], payload[48];
    uint16_t result, n;
    uint8_t reply_length;
    bool mutation;
    if (!ldr_parse_request(raw, size, &r))
    {
        return 0;
    }
    mutation = r.opcode >= LDR_OP_BEGIN && r.opcode <= LDR_OP_ABORT;
    if (mutation && c->cache_valid && r.seq == c->last_sequence &&
        r.session == ldr_get_u32(c->cache_request + 8))
    {
        if (size == c->cache_length && memcmp(raw, c->cache_request, size) == 0)
        {
            memcpy(reply, c->cache_reply, c->reply_length);
            c->last_activity_ms = now_ms;
            return c->reply_length;
        }
        ldr_put_u16(payload, CONFLICT);
        return ldr_build_reply(r.seq, r.session, r.opcode, payload, 2, reply);
    }
    result = process(c, &r, data, &n);
    if (result != OK)
    {
        c->last_error = (uint8_t)result;
        n = 0;
    }
    ldr_put_u16(payload, result);
    memcpy(payload + 2, data, n);
    reply_length = ldr_build_reply(r.seq, r.session, r.opcode, payload, n + 2U, reply);
    if (mutation && result == OK)
    {
        c->last_error = 0;
        c->last_activity_ms = now_ms;
        c->last_sequence = r.seq;
        memcpy(c->cache_request, raw, size);
        memcpy(c->cache_reply, reply, reply_length);
        c->cache_length = size;
        c->reply_length = reply_length;
        c->cache_valid = true;
    }
    return reply_length;
}
