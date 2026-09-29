/* 纯内存模型：不加载设备DLL、不访问CAN/J-Link；验证升级事务与跳前校验。 */
#include "upgrade.h"
#include "image_mailbox.h"
#include "device_identity.h"
#include "yg_protocol_upgrade_wire.h"
#include "yg_protocol_byte_order.h"
#include <assert.h>
#include <string.h>
#define MODEL_BASE 0x1000U
#define MODEL_PAGE 2048U
static uint8_t flash_bytes[MODEL_PAGE * 2], programmed[512], image[640], manifest[96];
static uint32_t now_ms, programs, resets;
static uint32_t tear_address, tear_bytes;
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
    memcpy(flash_bytes + (a - MODEL_BASE), p, a == tear_address ? tear_bytes : 8U);
    ++programs;
    if (a == tear_address)
    {
        return false;
    }
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
    uint32_t sp = 0x20000800U, entry = MODEL_BASE + MODEL_PAGE + 0x261U, crc;
    memset(image, 0, sizeof(image));
    memset(manifest, 0, sizeof(manifest));
    memcpy(image, &sp, 4);
    memcpy(image + 4, &entry, 4);
    be32(image + 512, YGI_DESCRIPTOR_MAGIC);
    be32(image + 516, YGI_DESCRIPTOR_VERSION);
    be32(image + 520, YGI_PROFILE);
    be32(image + 524, 4);
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
    be32(manifest + 8, 4);
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
    memset(image + 512U + YGI_ACTIVATION_OFFSET, 255, 24U);
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
    config.layout = 4;
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
    assert(!LoaderImage_CheckImage(&config));
    assert(LoaderImage_Check(&config, &upgrade.image.image));
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

static void begin_identity_and_image_compatibility(void)
{
    LoaderUpgradeStatus status;
    LoaderImageIdentity parsed;
    uint8_t old[96];
    initialize();
    (void)begin();
    uint32_t erased = upgrade.image.erased;
    assert(LoaderUpgrade_Begin(&upgrade, 2, 9, manifest, &status) == YI_OK);
    assert(status.binding.session_id == 1U && upgrade.image.erased == erased);
    assert(LoaderUpgrade_Begin(&upgrade, 2, 10, manifest, &status) == YI_BUSY);
    assert(LoaderUpgrade_Begin(&upgrade, 3, 9, manifest, &status) == YI_BUSY);
    memcpy(old, manifest, sizeof(old));
    old[31] ^= 1U;
    assert(LoaderUpgrade_Begin(&upgrade, 2, 9, old, &status) == YI_BUSY);
    memcpy(old, manifest, sizeof(old));
    be32(old + 4, 0x53593349U);
    assert(!LoaderImage_DecodeManifest(&config, old, &parsed));
    memcpy(old, manifest, sizeof(old));
    be32(old + 8, 3U);
    assert(!LoaderImage_DecodeManifest(&config, old, &parsed));
    memcpy(old, manifest, sizeof(old));
    be32(old + 36, YGI_ROLE_MAINTENANCE);
    assert(!LoaderImage_DecodeManifest(&config, old, &parsed));
}

static void descriptor_versions_and_role_are_rejected(void)
{
    static const uint32_t offsets[] = {516U, 520U, 524U, 544U};
    static const uint32_t values[] = {1U, 0x53593349U, 3U, YGI_ROLE_MAINTENANCE};
    initialize();
    memcpy(flash_bytes + MODEL_PAGE, image, sizeof(image));
    assert(LoaderImage_Activate(&config, &upgrade.image.image) == false);
    LoaderImageIdentity parsed;
    assert(LoaderImage_DecodeManifest(&config, manifest, &parsed));
    assert(LoaderImage_Check(&config, &parsed));
    for (unsigned i = 0U; i < sizeof(offsets) / sizeof(offsets[0]); ++i)
    {
        memcpy(flash_bytes + MODEL_PAGE, image, sizeof(image));
        uint8_t *candidate = flash_bytes + MODEL_PAGE;
        be32(candidate + offsets[i], values[i]);
        be32(candidate + 560U, 0U);
        be32(candidate + 560U, crc32(candidate, sizeof(image)));
        /* CRC仍正确，必须由版本/角色判据拒绝，不能只靠破坏摘要。 */
        assert(!LoaderImage_Check(&config, &parsed));
    }
}

static void activation_rechecks_flash(void)
{
    uint32_t i;
    uint64_t ticket;
    initialize();
    LoaderUpgradeBinding binding = begin();
    for (i = 0; i < sizeof(image); i += 128U)
    {
        assert(LoaderUpgrade_Write(&upgrade, &binding, i, image + i, 128U) == YI_OK);
    }
    assert(LoaderUpgrade_Finish(&upgrade, &binding) == YI_OK);
    for (i = 0; i < 10U; ++i)
    {
        LoaderUpgrade_Poll(&upgrade);
    }
    assert(LoaderUpgrade_Activate(&upgrade, &binding, &ticket) == YI_OK);
    flash_bytes[MODEL_PAGE + 100U] ^= 1U;
    assert(!LoaderUpgrade_TxComplete(&upgrade, ticket, true));
    assert(resets == 0U && upgrade.image.phase == YI_ERROR && upgrade.image.reason == YI_DIGEST);
}

static void host_nonce_status_golden(void)
{
    static const uint8_t expected[40] = {0U, 0U, 0U, 0U, 0U,   0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
                                         0U, 0U, 0U, 6U, 128U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
                                         0U, 0U, 0U, 0U, 0U,   0U, 0U, 0U, 0U, 0U, 0U, 0U};
    YgProtocolUpgradeReply reply = {.flags = YG_PROTOCOL_UPGRADE_FLAG_INTEGRITY_ONLY |
                                             YG_PROTOCOL_UPGRADE_FLAG_HOST_NONCE,
                                    .max_block_bytes = 128U};
    uint8_t actual[40];
    assert(YgProtocolUpgrade_EncodeReply(&reply, actual, sizeof(actual), NULL) == YG_PROTOCOL_OK);
    assert(!memcmp(actual, expected, sizeof(actual)));
    reply.flags |= 8U;
    assert(YgProtocolUpgrade_EncodeReply(&reply, actual, sizeof(actual), NULL) ==
           YG_PROTOCOL_INVALID_FIELD);
}
static void persistent_gates_and_power_loss(void)
{
    LoaderImageIdentity identity;
    uint32_t marker = MODEL_BASE + MODEL_PAGE + 512U;
    uint8_t confirmation[8];
    for (uint32_t cut = 0U; cut <= sizeof(image); cut += 8U)
    {
        initialize();
        tear_address = 0U;
        LoaderUpgradeBinding binding = begin();
        for (uint32_t offset = 0U; offset < cut; offset += 8U)
        {
            assert(LoaderUpgrade_Write(&upgrade, &binding, offset, image + offset, 8U) == YI_OK);
        }
        /* 即使整包完整，冷启动也不能越过未激活门槛。 */
        assert(!LoaderImage_CheckImage(&config));
        assert(!LoaderImage_PrepareBoot(&config));
    }
    for (uint32_t cut = 0U; cut < 8U; ++cut)
    {
        initialize();
        memcpy(flash_bytes + MODEL_PAGE, image, sizeof(image));
        assert(LoaderImage_DecodeManifest(&config, manifest, &identity));
        tear_address = marker + YGI_ACTIVATION_OFFSET;
        tear_bytes = cut;
        assert(!LoaderImage_Activate(&config, &identity));
        assert(!LoaderImage_CheckImage(&config));
        assert(!LoaderImage_PrepareBoot(&config));
        tear_address = 0U;
    }
    for (uint32_t cut = 0U; cut < 8U; ++cut)
    {
        initialize();
        memcpy(flash_bytes + MODEL_PAGE, image, sizeof(image));
        assert(LoaderImage_DecodeManifest(&config, manifest, &identity));
        assert(LoaderImage_Activate(&config, &identity));
        tear_address = marker + YGI_ATTEMPT_OFFSET;
        tear_bytes = cut;
        assert(!LoaderImage_PrepareBoot(&config));
        /* 返回失败的编程操作不能被当作完整首次试运行；半写单元不得补写。 */
        if (cut != 0U)
        {
            tear_address = 0U;
            assert(!LoaderImage_PrepareBoot(&config));
        }
        tear_address = 0U;
    }
    initialize();
    memcpy(flash_bytes + MODEL_PAGE, image, sizeof(image));
    assert(LoaderImage_DecodeManifest(&config, manifest, &identity));
    assert(LoaderImage_Activate(&config, &identity));
    assert(LoaderImage_CheckImage(&config));
    assert(LoaderImage_PrepareBoot(&config));
    assert(!LoaderImage_PrepareBoot(&config));
    /* 试运行开始后的掉电不可再试；只有APP确认完整才允许后续冷启动。 */
    be32(confirmation, YGI_CONFIRM_MAGIC);
    be32(confirmation + 4U, ~YGI_CONFIRM_MAGIC);
    for (uint32_t cut = 0U; cut < 8U; ++cut)
    {
        memset(flash_bytes + MODEL_PAGE + 512U + YGI_CONFIRM_OFFSET, 255, 8U);
        memcpy(flash_bytes + MODEL_PAGE + 512U + YGI_CONFIRM_OFFSET, confirmation, cut);
        assert(!LoaderImage_PrepareBoot(&config));
    }
    memcpy(flash_bytes + MODEL_PAGE + 512U + YGI_CONFIRM_OFFSET, confirmation, 8U);
    assert(LoaderImage_PrepareBoot(&config));
    assert(LoaderImage_PrepareBoot(&config));
    assert(LoaderImage_CheckImage(&config));
}

static void production_identity_contract(void)
{
    uint8_t record[64] = {0}, payload[32];
    DeviceIdentity identity;
    YgProtocol_WriteU32Le(record, DEVICE_IDENTITY_MAGIC);
    YgProtocol_WriteU32Le(record + 4U, 1U);
    memcpy(record + 8U, "YG2026-ESC-001", 14U);
    YgProtocol_WriteU32Le(record + 24U, 7U);
    YgProtocol_WriteU32Le(record + 56U, crc32(record, 56U));
    assert(YgProtocol_ReadU32Le(record + 56U) == 0x5B451A70U);
    YgProtocol_WriteU32Le(record + 60U, ~DEVICE_IDENTITY_MAGIC);
    assert(DeviceIdentity_Decode(record, &identity));
    DeviceIdentity_EncodeInfo(&identity, 3U, 2U, 3U, 0x1234U, payload);
    assert(memcmp(payload, record + 8U, 16U) == 0);
    assert(YgProtocol_ReadU32Le(payload + 16U) == 7U);
    assert(payload[20] == 1U && payload[21] == 3U && payload[22] == 2U);
    for (uint32_t i = 0U; i < sizeof(record); ++i)
    {
        record[i] ^= 1U;
        assert(!DeviceIdentity_Decode(record, &identity));
        assert(!identity.valid);
        record[i] ^= 1U;
    }
    for (uint32_t cut = 0U; cut < sizeof(record); ++cut)
    {
        uint8_t partial[64];
        memset(partial, 255, 64U);
        memcpy(partial, record, cut);
        assert(!DeviceIdentity_Decode(partial, &identity));
    }
}
int main(void)
{
    mailbox_golden();
    complete_and_boot();
    expiry_and_old_session();
    begin_identity_and_image_compatibility();
    descriptor_versions_and_role_are_rejected();
    activation_rechecks_flash();
    host_nonce_status_golden();
    persistent_gates_and_power_loss();
    production_identity_contract();
    return 0;
}
