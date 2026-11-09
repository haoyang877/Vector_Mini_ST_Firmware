/* 复用复位邮箱的真实写入函数；用会跳变的时钟模拟交接时被中断抢占。 */
#include "image_mailbox.h"
#include <assert.h>
#include <setjmp.h>
#include <stdio.h>
#include <string.h>

static uint32_t mailbox_words[YGIM_BYTES / sizeof(uint32_t)];
#define TARGET_MAILBOX_ADDRESS ((uintptr_t)mailbox_words)
#define __DMB() ((void)0)
#define __DSB() ((void)0)
static bool available = true;
static bool reset_allowed;
static unsigned read_count, late_read, off_count, lose_stop;
static jmp_buf reset;
static uint32_t HAL_GetTick(void)
{
    return ++read_count >= late_read ? 20U : 19U;
}
static bool outputs_off(void)
{
    return ++off_count != lose_stop;
}
static void NVIC_SystemReset(void)
{
    assert(reset_allowed);
    longjmp(reset, 1);
}

/* REAL_ENTRY */

int main(void)
{
    for (unsigned at = 1; at <= 3; ++at)
    {
        late_read = at;
        read_count = off_count = 0;
        assert(!BootHw_EnterLoader(7U, 10U, 10U) && mailbox_words[0] == 0U);
    }
    late_read = 4;
    for (unsigned at = 1; at <= 3; ++at)
    {
        lose_stop = at;
        read_count = off_count = 0;
        assert(!BootHw_EnterLoader(7U, 10U, 10U) && mailbox_words[0] == 0U);
    }
    lose_stop = read_count = off_count = 0;
    reset_allowed = true;
    if (setjmp(reset) == 0)
    {
        (void)BootHw_EnterLoader(7U, 10U, 10U);
        assert(false);
    }
    BootMailboxMessage message;
    assert(BootMailbox_Decode((const uint8_t *)mailbox_words, &message) && message.node == 7U);
    puts("PASS unmasked mailbox: timeout, lost shutdown, valid commit before reset");
    return 0;
}
