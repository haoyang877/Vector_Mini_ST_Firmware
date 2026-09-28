#include "flash_guard.h"
#include "stm32g4xx.h"
#define ERROR_BITS (FLASH_ECCR_ECCC | FLASH_ECCR_ECCD)
#define FLASH_BYTES 131072U
static volatile uint32_t bad_pages[2], active_unit;
static volatile bool busy, active, failed, unknown_error;
#if !defined(YG_FLASH_GUARD_MODEL)
#if defined(__CC_ARM)
/* 该LDRB的精确后继PC是ECC归属证据，禁止合并汇编行。 */
// clang-format off
__asm uint32_t yg_flash_guard_load(uint32_t address)
{
    EXPORT yg_flash_guard_resume
    LDRB r0,[r0]
yg_flash_guard_resume
    BX lr
}
// clang-format on
#else
#error Guarded load requires reviewed ARMCC code or the explicit native model
#endif
extern const uint8_t yg_flash_guard_resume;
#define LOAD_BYTE(a) yg_flash_guard_load(a)
#define RESUME_PC ((uint32_t)&yg_flash_guard_resume)
#define CLEAR_ECC(v) (FLASH->ECCR = ((v) & FLASH_ECCR_ECCIE) | ((v) & ERROR_BITS))
#else
#define LOAD_BYTE(a) yg_flash_model_load(a)
#define RESUME_PC YG_FLASH_MODEL_RESUME_PC
#define CLEAR_ECC(v) yg_flash_model_clear(v)
#endif
static bool inside(uint32_t a, uint32_t n)
{
    return a >= FLASH_BASE && n && n <= FLASH_BYTES && a - FLASH_BASE <= FLASH_BYTES - n;
}
static void mark(uint32_t ecc)
{
    uint32_t offset = ecc & FLASH_ECCR_ADDR_ECC;
    if ((ecc & FLASH_ECCR_SYSF_ECC) || offset >= FLASH_BYTES)
    {
        unknown_error = true;
    }
    else
    {
        bad_pages[(offset / 2048U) / 32U] |= 1U << ((offset / 2048U) % 32U);
    }
}
static bool bad(uint32_t a)
{
    uint32_t page = (a - FLASH_BASE) / 2048U;
    return (bad_pages[page / 32U] & (1U << (page % 32U))) != 0;
}
bool LoaderFlash_HandleNmi(const uint32_t frame[8], uint32_t exc_return)
{
    uint32_t ecc = FLASH->ECCR;
    if (!frame ||
        (exc_return != 0xFFFFFFF9U && exc_return != 0xFFFFFFFDU && exc_return != 0xFFFFFFE9U &&
         exc_return != 0xFFFFFFEDU) ||
        !active || (RCC->CIFR & (RCC_CIFR_CSSF | RCC_CIFR_LSECSSF)) || !(frame[7] & (1U << 24)) ||
        (frame[7] & 0x1FFU) || !(ecc & FLASH_ECCR_ECCD) || (ecc & FLASH_ECCR_SYSF_ECC) ||
        (ecc & FLASH_ECCR_ADDR_ECC) != (active_unit - FLASH_BASE) || frame[6] != (RESUME_PC & ~1U))
    {
        return false;
    }
    failed = true;
    mark(ecc);
    CLEAR_ECC(ecc);
    __DSB();
    return true;
}
static void cache_reset(void)
{
    uint32_t acr = FLASH->ACR;
    FLASH->ACR = acr & ~FLASH_ACR_DCEN;
    __DSB();
    FLASH->ACR = (acr & ~FLASH_ACR_DCEN) | FLASH_ACR_DCRST;
    __DSB();
    FLASH->ACR = acr & ~FLASH_ACR_DCEN;
    __DSB();
    FLASH->ACR = acr;
    __DSB();
    __ISB();
}
bool LoaderFlash_EraseCompleted(uint32_t a)
{
    uint32_t key, page;
    bool ok = false;
    if (a % 2048U || !inside(a, 2048))
    {
        return false;
    }
    key = __get_PRIMASK();
    __disable_irq();
    if (!busy && !active && !unknown_error && !(FLASH->ECCR & ERROR_BITS))
    {
        cache_reset();
        page = (a - FLASH_BASE) / 2048U;
        bad_pages[page / 32U] &= ~(1U << (page % 32U));
        ok = true;
    }
    __set_PRIMASK(key);
    return ok;
}
bool LoaderFlash_Read(uint32_t a, void *destination, uint32_t bytes)
{
    uint8_t *p = destination;
    uint32_t key, ecc, n, i;
    bool ok = true;
    if (!p || !inside(a, bytes))
    {
        return false;
    }
    key = __get_PRIMASK();
    __disable_irq();
    if (busy || unknown_error)
    {
        __set_PRIMASK(key);
        return false;
    }
    busy = true;
    __set_PRIMASK(key);
    while (bytes)
    {
        key = __get_PRIMASK();
        __disable_irq();
        ecc = FLASH->ECCR;
        /* 既存错误不归当前读取所有；保留给异常所有者，不能清掉未知ECCD。 */
        if (ecc & ERROR_BITS)
        {
            mark(ecc);
            unknown_error = true;
            ok = false;
        }
        if (!ok || bad(a))
        {
            ok = false;
            __set_PRIMASK(key);
            break;
        }
        n = 8U - (a & 7U);
        if (n > bytes)
        {
            n = bytes;
        }
        active_unit = a & ~7U;
        failed = false;
        active = true;
        __DMB();
        for (i = 0; i < n; i++)
        {
            p[i] = (uint8_t)LOAD_BYTE(a + i);
            __DSB();
            ecc = FLASH->ECCR;
            if (ecc & ERROR_BITS)
            {
                failed = true;
                mark(ecc);
                if (ecc & FLASH_ECCR_ECCD)
                {
                    unknown_error = true;
                }
                else
                {
                    CLEAR_ECC(ecc);
                }
            }
            if (failed)
            {
                break;
            }
        }
        active = false;
        __DMB();
        ok = !failed;
        __set_PRIMASK(key);
        if (!ok)
        {
            break;
        }
        a += n;
        p += n;
        bytes -= n;
    }
    key = __get_PRIMASK();
    __disable_irq();
    busy = false;
    __set_PRIMASK(key);
    return ok;
}
