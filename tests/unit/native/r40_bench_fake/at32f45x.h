#ifndef R40_BENCH_FAKE_H
#define R40_BENCH_FAKE_H
#include <stdbool.h>
#include <stdint.h>
/* 验证镜像测试的窄寄存器替身，不模拟控制器或Flash算法。 */
typedef struct
{
    uint32_t cctrl, brk, pr, c2dt, c3dt, c4dt;
    bool running;
} tmr_type;
typedef struct
{
    uint32_t idt;
} gpio_type;
typedef struct
{
    uint32_t ctrlstat;
} can_type;
extern tmr_type fake_tmr1, fake_tmr6;
extern gpio_type fake_gpioa, fake_gpiob;
extern can_type fake_can;
extern uint32_t system_core_clock;
#define TMR1 (&fake_tmr1)
#define TMR6 (&fake_tmr6)
#define GPIOA (&fake_gpioa)
#define GPIOB (&fake_gpiob)
#define CAN1 (&fake_can)
#define DMA1_CHANNEL2 2U
#define DMA1_CHANNEL3 3U
#define FALSE false
/** @brief 记录测试中的中断屏蔽。 */
void __disable_irq(void);
/** @brief 记录测试定时器启停。
 * @param timer 测试寄存器。
 * @param enabled 启停选择。
 */
void tmr_counter_enable(tmr_type *timer, bool enabled);
/** @brief 记录DMA启停。
 * @param channel 模拟通道。
 * @param enabled 启停选择。
 */
void dma_channel_enable(unsigned channel, bool enabled);
#endif
