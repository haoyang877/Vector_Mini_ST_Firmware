/* 注入生产Flash等待函数与vendor位定义，模型仅模拟状态寄存器和独立时钟。 */
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

static struct
{
    uint32_t SR;
} model_flash;
#define FLASH (&model_flash)
static uint32_t model_now, wait_started, release_after;
static bool release_busy;
static bool flash_ready(void);

uint32_t LoaderPlatform_NowMs(void)
{
    if (release_busy && (uint32_t)(model_now - wait_started) >= release_after)
    {
        model_flash.SR &= ~FLASH_SR_BSY;
    }
    return model_now++;
}
static void setup(uint32_t flags, uint32_t start, bool release, uint32_t after)
{
    model_flash.SR = flags;
    model_now = wait_started = start;
    release_busy = release;
    release_after = after;
}
int main(void)
{
    const uint32_t errors[] = {FLASH_SR_OPERR,
                               FLASH_SR_PROGERR,
                               FLASH_SR_WRPERR,
                               FLASH_SR_PGAERR,
                               FLASH_SR_SIZERR,
                               FLASH_SR_PGSERR,
                               FLASH_SR_MISERR,
                               FLASH_SR_FASTERR,
                               FLASH_SR_RDERR,
                               FLASH_SR_OPTVERR};
    for (unsigned i = 0; i < sizeof(errors) / sizeof(errors[0]); ++i)
    {
        setup(errors[i], 0U, false, 0U);
        assert(!flash_ready());
    }
    setup(FLASH_SR_EOP, 0U, false, 0U);
    assert(flash_ready());
    setup(FLASH_SR_BSY, 0U, false, 0U);
    assert(!flash_ready() && model_now <= FLASH_TIMEOUT_MS + 2U);
    setup(FLASH_SR_BSY, UINT32_MAX - 10U, true, 20U);
    assert(flash_ready());
    puts("PASS Flash complete error mask, bounded BSY wait and timer wrap");
    return 0;
}
