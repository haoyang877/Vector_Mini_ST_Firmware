"""验证两板真实PWM寄存器行为：六路门控、单相写入及计数器所有权。"""

from _bsp_test_support import arguments, production, resources, run_c


def main() -> None:
    """枚举初始门控位与占空比；不假造已删除的HAL重试/提交窗口。"""
    args = arguments(__doc__)
    for board in (1, 2):
        setup = r"""
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include "bsp_pwm.h"
#include "bsp_internal.h"
static struct {uint32_t ARR,CCR1,CCR2,CCR3,CCER,BDTR,CR1,pr,c2dt,c3dt,c4dt,cctrl,brk,ctrl1;} timer;
#define TIM1 (&timer)
#define TMR1 (&timer)
#define TIM_CCER_CC1E 1U
#define TIM_CCER_CC1NE 4U
#define TIM_CCER_CC2E 16U
#define TIM_CCER_CC2NE 64U
#define TIM_CCER_CC3E 256U
#define TIM_CCER_CC3NE 1024U
#define TIM_BDTR_MOE 32768U
""" + resources(board)
        fields = (
            ("ARR", "CCER", "BDTR", "CR1", "CCR1", "CCR2", "CCR3")
            if board == 1
            else ("pr", "cctrl", "brk", "ctrl1", "c2dt", "c3dt", "c4dt")
        )
        for macro, field in zip(
            ("PERIOD", "CHANNELS", "MASTER", "CONTROL", "A", "B", "C"), fields, strict=True
        ):
            setup += f"\n#define {macro} timer.{field}\n"
        setup += f"#define EXPECTED_MASK {0x555 if board == 1 else 0x5550}U\n"
        source = (
            setup
            + production(
                board,
                "pwm",
                "BspPwm_SetDuty",
                "BspPwm_SetPhaseDuty",
                "BspPwm_Enable",
                "BspPwm_Disable",
                "BspPwm_ArePhasesDisabled",
            )
            + r"""
int main(void) {
    PERIOD=4500; CONTROL=0x11;
    for(unsigned initial=0;initial<65536;initial++) {
        CHANNELS=initial; MASTER=0x12345678;
        assert(BspPwm_ArePhasesDisabled()==((initial&EXPECTED_MASK)==0));
        BspPwm_Enable();
        assert(CHANNELS==(initial|EXPECTED_MASK)); assert(MASTER==(0x12345678U|32768U));
        assert(CONTROL==0x11 && PERIOD==4500 && !BspPwm_ArePhasesDisabled());
        BspPwm_Disable();
        assert(CHANNELS==(initial&~EXPECTED_MASK)); assert(MASTER==(0x12345678U&~32768U));
        assert(CONTROL==0x11 && BspPwm_ArePhasesDisabled());
    }
    for(unsigned i=0;i<=4500;i++) {
        float duty=(float)i/4500.0f;
        BspPwm_SetDuty(duty,0.25f,0.75f);
        assert(A==(uint16_t)(duty*4500.0f) && B==1125 && C==3375);
        BspPwm_SetPhaseDuty(BSP_PWM_PHASE_B,0.5f);
        assert(A==(uint16_t)(duty*4500.0f) && B==2250 && C==3375);
        BspPwm_SetPhaseDuty((BspPwmPhase)99,0.0f);
        assert(B==2250 && CONTROL==0x11 && PERIOD==4500);
    }
    puts("PASS PWM: all 65536 channel masks, unrelated bits, counter preserved, full duty range");
    return 0;
}
"""
        )
        run_c(args, f"pwm_{board}", source, board)


if __name__ == "__main__":
    main()
