/* APP侧窄启动端口：只写镜像记录页，固件下载及boot epoch分配始终由Loader拥有。 */
#include "boot_hw.h"
#include "image_mailbox.h"
#include "image_transfer.h"
#include "vector_mini_st_g431.h"
#include "stm32g4xx_hal.h"
#include <string.h>
extern const uint32_t __Vectors;
static BootHwInfo info;
static LoaderImageConfig config;
static LoaderImageIdentity identity;
static bool initialized, write_authority;
static bool outputs_off(void)
{
    const uint32_t mask = TIM_CCER_CC1E | TIM_CCER_CC1NE | TIM_CCER_CC2E | TIM_CCER_CC2NE |
                          TIM_CCER_CC3E | TIM_CCER_CC3NE;
    return !(TIM1->CCER & mask) && !(GPIOA->IDR & 0x780U) && !(GPIOB->IDR & 3U);
}
static bool read_image(void *context, uint32_t address, void *out, uint32_t bytes)
{
    (void)context;
    if (!out || !bytes || bytes > TARGET_ERASE_BYTES + TARGET_APP_BYTES ||
        address < TARGET_RECORD_BASE ||
        address - TARGET_RECORD_BASE > TARGET_ERASE_BYTES + TARGET_APP_BYTES - bytes ||
        !outputs_off() || (FLASH->ECCR & (FLASH_ECCR_ECCC | FLASH_ECCR_ECCD)))
    {
        return false;
    }
    /* APP不恢复未知ECC异常；读取前先确认关相，NMI默认停留并由继承IWDG退回Loader。 */
    memcpy(out, (const void *)address, bytes);
    __DSB();
    return !(FLASH->ECCR & (FLASH_ECCR_ECCC | FLASH_ECCR_ECCD));
}
static bool allowed(void *context)
{
    (void)context;
    return write_authority && info.available && outputs_off();
}
static void watchdog(void *context)
{
    (void)context;
    /* 完整镜像扫描不喂狗：坏APP不能靠反复确认延长试启动期限。 */
}
static bool erase_record(void *context, uint32_t address, uint32_t bytes)
{
    FLASH_EraseInitTypeDef erase = {0};
    uint32_t error, acr;
    bool ok;
    if (!allowed(context) || address != TARGET_RECORD_BASE || bytes != TARGET_ERASE_BYTES ||
        HAL_FLASH_Unlock() != HAL_OK)
    {
        return false;
    }
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);
    erase.Banks = FLASH_BANK_1;
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.Page = (address - TARGET_FLASH_BASE) / TARGET_ERASE_BYTES;
    erase.NbPages = 1;
    ok = HAL_FLASHEx_Erase(&erase, &error) == HAL_OK;
    (void)HAL_FLASH_Lock();
    acr = FLASH->ACR;
    FLASH->ACR = acr & ~FLASH_ACR_DCEN;
    __DSB();
    FLASH->ACR = (acr & ~FLASH_ACR_DCEN) | FLASH_ACR_DCRST;
    __DSB();
    FLASH->ACR = acr & ~FLASH_ACR_DCEN;
    __DSB();
    FLASH->ACR = acr;
    __DSB();
    __ISB();
    return ok;
}
static bool program_record(void *context, uint32_t address, const void *data, uint32_t bytes)
{
    uint64_t value, check;
    bool ok;
    if (!data || bytes != 8 || address % 8 || address < TARGET_RECORD_BASE ||
        address - TARGET_RECORD_BASE > TARGET_ERASE_BYTES - 8 || !allowed(context) ||
        HAL_FLASH_Unlock() != HAL_OK)
    {
        return false;
    }
    memcpy(&value, data, 8);
    ok = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, address, value) == HAL_OK;
    (void)HAL_FLASH_Lock();
    return ok && read_image(context, address, &check, 8) && check == value;
}
static void configure(void)
{
    config.io.base = TARGET_RECORD_BASE;
    config.io.bytes = TARGET_ERASE_BYTES + TARGET_APP_BYTES;
    config.io.erase_bytes = TARGET_ERASE_BYTES;
    config.io.program_bytes = 8;
    config.io.read = read_image;
    config.io.erase = erase_record;
    config.io.program = program_record;
    config.app.base = TARGET_APP_BASE;
    config.app.bytes = TARGET_APP_BYTES;
    config.record.base = TARGET_RECORD_BASE;
    config.record.bytes = TARGET_ERASE_BYTES;
    config.layout = TARGET_LAYOUT;
    config.mcu = TARGET_MCU;
    config.product = TARGET_PRODUCT;
    config.hardware = TARGET_HARDWARE;
    config.param_abi = TARGET_PARAM_ABI;
    config.ram_start = TARGET_RAM_BASE;
    config.ram_end = TARGET_RAM_END;
    config.descriptor_offset = TARGET_DESCRIPTOR_OFFSET;
    config.schema = TARGET_SCHEMA;
    config.allowed = allowed;
    config.watchdog = watchdog;
}
bool BootHw_Init(void)
{
    uint8_t raw[YGIM_BYTES], manifest[96];
    volatile uint8_t *mailbox = (volatile uint8_t *)TARGET_MAILBOX_ADDRESS;
    BootMailboxMessage message;
    uint32_t state, key, i;
    if (initialized)
    {
        return info.available;
    }
    initialized = true;
    if ((uint32_t)&__Vectors != TARGET_APP_BASE || !outputs_off())
    {
        return false;
    }
    key = __get_PRIMASK();
    __disable_irq();
    SCB->VTOR = TARGET_APP_BASE;
    __DSB();
    __ISB();
    for (i = 0; i < sizeof(raw); ++i)
    {
        raw[i] = mailbox[i];
        mailbox[i] = 0;
    }
    __DMB();
    configure();
    if (BootMailbox_Decode(raw, &message) &&
        (message.command == YG_BOOT_TRIAL || message.command == YG_BOOT_NORMAL) &&
        LoaderImage_ReadRecord(&config, manifest, &state) &&
        ((state == 2 && message.command == YG_BOOT_TRIAL) ||
         (state == 3 && message.command == YG_BOOT_NORMAL)) &&
        LoaderImage_DecodeManifest(&config, manifest, &identity) &&
        identity.token == message.token && LoaderImage_Check(&config, &identity))
    {
        info.boot_epoch = message.epoch;
        info.image_token = message.token;
        info.image_role = identity.role;
        info.node = (uint8_t)message.node;
        info.trial = state == 2;
        info.confirmed = state == 3;
        info.available = true;
    }
    __set_PRIMASK(key);
    return info.available;
}
void BootHw_GetInfo(BootHwInfo *out)
{
    if (out != NULL)
    {
        *out = info;
    }
}
static LoaderParameterState parameter_shape(void)
{
    uint32_t offset;
    if (FLASH->ECCR & (FLASH_ECCR_ECCC | FLASH_ECCR_ECCD))
    {
        return YI_PARAMS_UNSAFE;
    }
    if (*(volatile const uint32_t *)(TARGET_PARAM_BASE + TARGET_RAW_SCHEMA_OFFSET) ==
            TARGET_SCHEMA &&
        *(volatile const uint32_t *)(TARGET_PARAM_BASE + TARGET_RAW_MAGIC_OFFSET) ==
            TARGET_RAW_MAGIC)
    {
        return (FLASH->ECCR & (FLASH_ECCR_ECCC | FLASH_ECCR_ECCD)) ? YI_PARAMS_UNSAFE
                                                                   : YI_PARAMS_RAW11;
    }
    if (identity.role != YGI_ROLE_MAINTENANCE)
    {
        return YI_PARAMS_UNSAFE;
    }
    for (offset = 0; offset < TARGET_PARAM_BYTES; offset += 4)
    {
        if (*(volatile const uint32_t *)(TARGET_PARAM_BASE + offset) != UINT32_MAX)
        {
            return YI_PARAMS_UNSAFE;
        }
    }
    return (FLASH->ECCR & (FLASH_ECCR_ECCC | FLASH_ECCR_ECCD)) ? YI_PARAMS_UNSAFE : YI_PARAMS_BLANK;
}
bool BootHw_Confirm(uint64_t epoch, uint32_t token)
{
    uint32_t key;
    bool ok;
    LoaderParameterState parameters;
    if (!info.available || epoch != info.boot_epoch || token != info.image_token || !outputs_off())
    {
        return false;
    }
    key = __get_PRIMASK();
    __disable_irq();
    if (!outputs_off())
    {
        __set_PRIMASK(key);
        return false;
    }
    parameters = parameter_shape();
    if (parameters == YI_PARAMS_UNSAFE)
    {
        __set_PRIMASK(key);
        return false;
    }
    write_authority = true;
    ok = LoaderImage_Confirm(&config, parameters, &identity) == YI_OK;
    write_authority = false;
    if (ok)
    {
        info.confirmed = true;
        info.trial = false;
    }
    __set_PRIMASK(key);
    return ok;
}
bool BootHw_EnterLoader(uint64_t epoch, uint8_t node)
{
    BootMailboxMessage message = {YG_BOOT_ENTER, 0, 0, 0};
    uint8_t raw[YGIM_BYTES];
    volatile uint8_t *mailbox = (volatile uint8_t *)TARGET_MAILBOX_ADDRESS;
    uint32_t key, i;
    if (!info.available || epoch != info.boot_epoch || node == 255)
    {
        return false;
    }
    message.node = node;
    message.token = info.image_token;
    message.epoch = epoch;
    if (!BootMailbox_Encode(&message, raw))
    {
        return false;
    }
    key = __get_PRIMASK();
    __disable_irq();
    if (!outputs_off())
    {
        __set_PRIMASK(key);
        return false;
    }
    *(volatile uint32_t *)mailbox = 0;
    __DMB();
    for (i = 4; i < sizeof(raw); ++i)
    {
        mailbox[i] = raw[i];
    }
    __DMB();
    *(volatile uint32_t *)mailbox = YGIM_MAGIC;
    __DSB();
    for (i = 0; i < sizeof(raw); ++i)
    {
        if (mailbox[i] != raw[i])
        {
            __set_PRIMASK(key);
            return false;
        }
    }
    NVIC_SystemReset();
    __set_PRIMASK(key);
    return false;
}
void BootHw_Watchdog(bool healthy)
{
    if (healthy && info.available && info.confirmed)
    {
        IWDG->KR = 0xAAAAU;
    }
}
