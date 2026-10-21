"""基础位置串级与板级初始化离线检查；不编译退役 RTT、轨迹或摩擦套件。"""

import sys as _sys
from pathlib import Path as _Path

_sys.path.insert(0, str(_Path(__file__).resolve().parents[3] / "tools"))
import argparse
import os
import re
import shutil
import subprocess
from pathlib import Path

from project_paths import NATIVE_INCLUDE_FLAGS, ROOT, resolve_source_path


def function_source(source: str, name: str) -> str:
    """返回真实 C 函数定义，跳过同名调用和声明。"""
    # A call may precede the definition (e.g. the board fast ADC adapter).
    match = re.search(
        r"^[ \t]*(?:[A-Za-z_]\w*\s+)+\**" + re.escape(name) + r"\s*\([^;{}]*\)\s*\{",
        source,
        re.M,
    )
    if match is None:
        raise ValueError("Function definition not found: " + name)
    start, brace = match.start(), match.end() - 1
    depth = 1
    end = brace + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end]


def board_startup_fixture() -> str:
    """用主机寄存器桩验证真实采样启动和三相使能条件。"""
    source = (ROOT / "firmware/platform/stm32g4/ports/board/board_startup_stm32g4.c").read_text(
        encoding="utf-8"
    )
    return (
        r"""
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
enum {ADC_SINGLE_ENDED=0, TIM_CHANNEL_1=1, TIM_CHANNEL_2=2, TIM_CHANNEL_3=3,
      TIM_CHANNEL_4=4, ADC_IT_JEOC=1, ADC_IT_JEOS=2};
static int hadc1, hadc2, htim1, htim7;
static uint32_t SystemCoreClock = 170000000U;
static unsigned delay_clock;
static unsigned phases, sampling, adc_started;
static unsigned adc_irq_mask = ADC_IT_JEOC;
static void delay_init(uint16_t clock) { delay_clock = clock; }
static void HAL_ADCEx_Calibration_Start(int *adc, int mode) { (void)adc; (void)mode; }
static void HAL_TIM_PWM_Start(int *timer, int channel) {
    (void)timer; if (channel == TIM_CHANNEL_4) sampling++; else phases++;
}
static void HAL_TIMEx_OCN_Start(int *timer, int channel) { (void)timer; (void)channel; phases++; }
static void HAL_ADCEx_InjectedStart(int *adc) { (void)adc; adc_started++; }
static void __HAL_ADC_ENABLE_IT(int *adc, int mask) { assert(adc == &hadc2); adc_irq_mask |= mask; }
static void __HAL_ADC_DISABLE_IT(int *adc, int mask) { assert(adc == &hadc2); adc_irq_mask &= ~mask; }
static void HAL_TIM_Base_Start_IT(int *timer) { (void)timer; }
"""
        + function_source(source, "board_hw_start")
        + r"""
int main(void) {
    board_hw_start(false);
    assert(phases == 0 && sampling == 1 && adc_started == 2);
    assert(adc_irq_mask == ADC_IT_JEOS && delay_clock == 170U);
    board_hw_start(true);
    assert(phases == 6 && sampling == 2 && adc_started == 4);
    assert(adc_irq_mask == ADC_IT_JEOS);
    puts("PASS actual board startup: unconfigured phase outputs off, sampling retained");
    return 0;
}
"""
    )


def board_orchestration_fixture(source: str | None = None) -> str:
    """验证实际初始化入口；冻结源码按自身App_Init或旧Board_Init名称提取。"""
    if source is None:
        source = resolve_source_path(
            ROOT,
            "firmware/app/composition/app_startup.c",
            "firmware/app/composition/board_config.c",
        ).read_text(encoding="utf-8")
    entry = "App_Init" if "void App_Init(" in source else "Board_Init"
    return (
        r"""
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
static int order[8];
static int order_len;
static bool board_start_argument;
static void log_step(int step) { order[order_len++] = step; }
/* 参数装载归初始化所有者；板级组合只调用一次所有者入口。 */
static void MotorAxis_Init(void) { log_step(3); }
static unsigned MotorAxis_NodeId(void) { return 1U; }
static void board_hw_start(bool motor_phases_enabled) {
    log_step(4);
    board_start_argument = motor_phases_enabled;
}
static bool protocol_ready = true;
static bool s_communication_ready;
static bool YgProtocolRuntime_Init(unsigned node) { assert(node == 1U); log_step(5); return protocol_ready; }
static bool YgProtocolStatusAdapter_Init(void) { return protocol_ready; }
static bool bind_motor_stop(void) { log_step(7); return protocol_ready; }
static bool YgProtocolOwner_BindServices(void) { log_step(8); return protocol_ready; }
static bool bind_device_description(void) { log_step(9); return protocol_ready; }
static bool YgProtocolRuntime_Start(void) { log_step(10); return protocol_ready; }
static void power_stage_hw_stop(void) { log_step(6); }
"""
        + function_source(source, entry)
        + r"""
int main(void) {
    static const int expected[] = {3, 4, 5, 7, 8, 9, 10};
    int index;
    INIT_ENTRY();
    assert(order_len == 7 && !board_start_argument && s_communication_ready);
    for (index = 0; index < 7; ++index) assert(order[index] == expected[index]);
    protocol_ready = false;
    order_len = 0;
    INIT_ENTRY();
    assert(order_len == 4 && order[3] == 6 && !s_communication_ready);
    puts("PASS board startup: order, configuration and communication failure disables power");
    return 0;
}
""".replace("INIT_ENTRY", entry)
    )


def main() -> None:
    """只构建基础位置和真实板级初始化的主机夹具，无硬件访问。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", default=os.environ.get("SERVO_CC"))
    parser.add_argument("--out", type=Path, default=ROOT / "outputs/position_basic_tests")
    args = parser.parse_args()
    cc = args.cc or shutil.which("zig") or shutil.which("gcc") or shutil.which("clang")
    if not cc:
        parser.error("Provide --cc with a native C99 compiler or Zig executable")
    cc = str(Path(cc).resolve()) if Path(cc).is_file() else cc
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    compiler = [cc] + (["cc"] if Path(cc).stem == "zig" else []) + NATIVE_INCLUDE_FLAGS
    logs = []

    def run(command: list[str]) -> None:
        result = subprocess.run(command, cwd=ROOT, capture_output=True, text=True)
        logs.append(result.stdout + result.stderr)
        print(logs[-1], end="")
        (out / "host_tests.log").write_text("\n".join(logs), encoding="utf-8")
        result.check_returncode()

    def build(name: str, files: list[str]) -> None:
        executable = out / (name + (".exe" if os.name == "nt" else ""))
        run(
            compiler
            + ["-std=c99", "-O1", "-UNDEBUG", "-Wall", "-Wextra", "-Werror"]
            + files
            + ([] if os.name == "nt" else ["-lm"])
            + ["-o", str(executable)]
        )
        run([str(executable)])

    build(
        "position_servo_test",
        [
            "tests/unit/position_servo_test.c",
            "firmware/motor/position/position_cascade.c",
            "firmware/motor/foc/foc_pid.c",
        ],
    )
    for name, fixture in (
        ("board_startup_test", board_startup_fixture()),
        ("board_orchestration_test", board_orchestration_fixture()),
    ):
        path = out / (name + ".c")
        path.write_text(fixture, encoding="utf-8")
        build(name, [str(path)])


if __name__ == "__main__":
    main()
