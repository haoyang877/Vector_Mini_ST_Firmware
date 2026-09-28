/* 纯内存模型：不加载设备DLL、不访问CAN/J-Link；验证升级事务与跳前校验。 */
#include "upgrade.h"
#include "image_mailbox.h"
#include <assert.h>
#include <string.h>
#define MODEL_BASE 0x1000U
#define MODEL_PAGE 2048U
static uint8_t flash_bytes[MODEL_PAGE * 2], programmed[512], image[640], manifest[96];
static uint32_t now_ms, programs, resets;
static bool physical_safe = true, cross_deadline;
static LoaderImageConfig config;
static LoaderUpgrade upgrade;
static void be32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v >> 24);
    p[1] = (uint8_t)(v >> 16);
    p[2] = (uint8_t)(v >> 8);
    p[3] = (uint8_t)v;
}
static uint32_t crc_more(uint32_t crc, const uint8_t *p, unsigned n)
{
    unsigned i, j;
    for (i = 0; i < n; ++i)
    {
        crc ^= p[i];
        for (j = 0; j < 8; ++j)
        {
            crc = (crc >> 1) ^ ((0U - (crc & 1U)) & 0xEDB88320U);
        }
    }
    return crc;
}
static uint32_t crc32(const uint8_t *p, unsigned n)
{
    return ~crc_more(~0U, p, n);
}
static bool read_bytes(void *context, uint32_t a, void *out, uint32_t n)
{
    (void)context;
    if (a < MODEL_BASE || n > sizeof(flash_bytes) || a - MODEL_BASE > sizeof(flash_bytes) - n)
    {
        return false;
    }
    memcpy(out, flash_bytes + (a - MODEL_BASE), n);
    return true;
}
static bool erase_bytes(void *context, uint32_t a, uint32_t n)
{
    (void)context;
    if (n != MODEL_PAGE || a % MODEL_PAGE || a < MODEL_BASE || a > MODEL_BASE + MODEL_PAGE)
    {
        return false;
    }
    memset(flash_bytes + (a - MODEL_BASE), 255, n);
    memset(programmed + (a - MODEL_BASE) / 8, 0, n / 8);
    return true;
}
static bool program_bytes(void *context, uint32_t a, const void *p, uint32_t n)
{
    uint32_t unit;
    (void)context;
    if (n != 8 || a % 8 || a < MODEL_BASE || a > MODEL_BASE + sizeof(flash_bytes) - 8)
    {
        return false;
    }
    unit = (a - MODEL_BASE) / 8;
    if (programmed[unit])
    {
        return false;
    }
    programmed[unit] = 1;
    memcpy(flash_bytes + (a - MODEL_BASE), p, 8);
    ++programs;
    if (cross_deadline)
    {
        now_ms = LOADER_UPGRADE_LEASE_MS;
    }
    return true;
}
static bool safe(void *context)
{
    (void)context;
    return physical_safe;
}
static uint32_t clock_ms(void *context)
{
    (void)context;
    return now_ms;
}
static void feed(void *context)
{
    (void)context;
}
static uint32_t lock(void *context)
{
    (void)context;
    return 0;
}
static void unlock(void *context, uint32_t key)
{
    (void)context;
    (void)key;
}
static bool boot_now(void *context)
{
    (void)context;
    ++resets;
    return true;
}
static void build_image(void)
{
    uint32_t sp = 0x20000800U, entry = MODEL_BASE + MODEL_PAGE + 0x241U, crc;
    memset(image, 0, sizeof(image));
    memset(manifest, 0, sizeof(manifest));
    memcpy(image, &sp, 4);
    memcpy(image + 4, &entry, 4);
    be32(image + 512, YGI_DESCRIPTOR_MAGIC);
    be32(image + 516, YGI_DESCRIPTOR_VERSION);
    be32(image + 520, YGI_PROFILE);
    be32(image + 524, 3);
    be32(image + 528, 1);
    be32(image + 532, 2);
    be32(image + 536, 1);
    be32(image + 540, 0x10000);
    be32(image + 544, YGI_ROLE_NORMAL);
    be32(image + 548, 11);
    image[553] = 11;
    image[555] = 11;
    be32(image + 556, sizeof(image));
    be32(image + 560, 0);
    be32(manifest, YGI_MANIFEST_MAGIC);
    be32(manifest + 4, YGI_PROFILE);
    be32(manifest + 8, 3);
    be32(manifest + 12, 1);
    be32(manifest + 16, 2);
    be32(manifest + 20, 1);
    be32(manifest + 24, 0x10000);
    be32(manifest + 28, 7);
    be32(manifest + 32, sizeof(image));
    be32(manifest + 36, YGI_ROLE_NORMAL);
    be32(manifest + 40, 11);
    manifest[45] = 11;
    manifest[47] = 11;
    crc = crc32(image, sizeof(image));
    be32(image + 560, crc);
    be32(manifest + 48, crc);
}
static void initialize(void)
{
    LoaderUpgradePorts ports = {0};
    memset(flash_bytes, 255, sizeof(flash_bytes));
    memset(programmed, 0, sizeof(programmed));
    memset(&config, 0, sizeof(config));
    now_ms = programs = resets = 0;
    physical_safe = true;
    cross_deadline = false;
    build_image();
    config.io.base = MODEL_BASE;
    config.io.bytes = sizeof(flash_bytes);
    config.io.erase_bytes = MODEL_PAGE;
    config.io.program_bytes = 8;
    config.io.read = read_bytes;
    config.io.erase = erase_bytes;
    config.io.program = program_bytes;
    config.app.base = MODEL_BASE + MODEL_PAGE;
    config.app.bytes = MODEL_PAGE;
    config.layout = 3;
    config.mcu = 1;
    config.product = 2;
    config.hardware = 1;
    config.param_abi = 11;
    config.schema = 11;
    config.ram_start = 0x20000000U;
    config.ram_end = 0x20000800U;
    config.descriptor_offset = 512;
    config.allowed = safe;
    config.watchdog = feed;
    ports.now_ms = clock_ms;
    ports.outputs_off_and_link_ready = safe;
    ports.watchdog = feed;
    ports.lock = lock;
    ports.unlock = unlock;
    ports.boot = boot_now;
    assert(LoaderUpgrade_Init(&upgrade, &config, &ports));
}
static LoaderUpgradeBinding begin(void)
{
    LoaderUpgradeStatus status;
    assert(LoaderUpgrade_Begin(&upgrade, 2, 9, manifest, &status) == YI_OK);
    LoaderUpgrade_Poll(&upgrade);
    LoaderUpgrade_Poll(&upgrade);
    assert(upgrade.image.phase == YI_RECEIVING);
    return status.binding;
}
static void complete_and_boot(void)
{
    LoaderUpgradeBinding binding;
    uint64_t ticket;
    uint8_t corrupt[128], saved;
    uint32_t before, i;
    initialize();
    binding = begin();
    for (i = 0; i < sizeof(image); i += 128)
    {
        assert(LoaderUpgrade_Write(&upgrade, &binding, i, image + i, 128) == YI_OK);
    }
    before = programs;
    assert(LoaderUpgrade_Write(&upgrade, &binding, 0, image, 128) == YI_OK);
    assert(programs == before);
    memcpy(corrupt, image, 128);
    corrupt[0] ^= 1;
    assert(LoaderUpgrade_Write(&upgrade, &binding, 0, corrupt, 128) == YI_CONFLICT);
    assert(programs == before);
    /* 跳前校验读的就是已写入 Flash，而不是 RAM 副本。 */
    assert(LoaderImage_CheckImage(&config));
    assert(LoaderUpgrade_Finish(&upgrade, &binding) == YI_OK);
    for (i = 0; i < 10; ++i)
    {
        LoaderUpgrade_Poll(&upgrade);
    }
    assert(upgrade.image.phase == YI_VERIFIED);
    assert(LoaderUpgrade_Activate(&upgrade, &binding, &ticket) == YI_OK);
    assert(!LoaderUpgrade_TxComplete(&upgrade, ticket + 1, true));
    assert(!resets);
    assert(LoaderUpgrade_TxComplete(&upgrade, ticket, true));
    assert(resets == 1);
    assert(!LoaderUpgrade_TxComplete(&upgrade, ticket, true));
    assert(resets == 1);
    /* 保存的镜像整包 CRC 不匹配时必须留在 Loader。 */
    saved = flash_bytes[(config.app.base - MODEL_BASE) + 100U];
    flash_bytes[(config.app.base - MODEL_BASE) + 100U] ^= 1U;
    assert(!LoaderImage_CheckImage(&config));
    flash_bytes[(config.app.base - MODEL_BASE) + 100U] = saved;
    assert(LoaderImage_CheckImage(&config));
}
static void expiry_and_old_session(void)
{
    LoaderUpgradeBinding old, binding;
    LoaderUpgradeStatus status;
    initialize();
    old = begin();
    cross_deadline = true;
    assert(LoaderUpgrade_Write(&upgrade, &old, 0, image, 8) == YI_DENIED);
    assert(LoaderUpgrade_Status(&upgrade, NULL, &status) == YI_OK && status.next_offset == 8 &&
           status.phase == YI_ERROR);
    cross_deadline = false;
    binding = begin();
    assert(binding.session_id != old.session_id);
    assert(LoaderUpgrade_Write(&upgrade, &old, 0, image, 8) == YI_DENIED);
    assert(LoaderUpgrade_Write(&upgrade, &binding, 0, image, 8) == YI_OK);
    physical_safe = false;
    LoaderUpgrade_Poll(&upgrade);
    physical_safe = true;
    assert(LoaderUpgrade_Write(&upgrade, &binding, 8, image + 8, 8) == YI_DENIED);
}
static void mailbox_golden(void)
{
    static const uint8_t base[32] = {0x59, 0x47, 0x49, 0x33, 0x04, 0x00, 0x00, 0x00,
                                     0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
                                     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t expected[32], actual[32];
    BootMailboxMessage message = {YG_BOOT_ENTER, 1}, decoded;
    uint32_t crc;
    memcpy(expected, base, sizeof(expected));
    crc = crc32(expected, 28);
    expected[28] = (uint8_t)crc;
    expected[29] = (uint8_t)(crc >> 8);
    expected[30] = (uint8_t)(crc >> 16);
    expected[31] = (uint8_t)(crc >> 24);
    assert(BootMailbox_Encode(&message, actual));
    assert(!memcmp(actual, expected, 32));
    assert(BootMailbox_Decode(expected, &decoded));
    assert(decoded.command == YG_BOOT_ENTER && decoded.node == 1);
    actual[9] ^= 1;
    assert(!BootMailbox_Decode(actual, &decoded));
    message.node = 255;
    assert(!BootMailbox_Encode(&message, actual));
}
int main(void)
{
    mailbox_golden();
    complete_and_boot();
    expiry_and_old_session();
    return 0;
}
