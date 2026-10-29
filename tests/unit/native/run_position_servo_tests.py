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

from project_paths import NATIVE_INCLUDE_FLAGS, ROOT


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
    """执行真实Bsp_Start，采样成功才能开放监督，失败关相且禁止重复启动。"""
    source = (ROOT / "firmware/platform/bsp/mcu/stm32g4/system.c").read_text(encoding="utf-8")
    return (
        r"""
#include <assert.h>
#include <stdio.h>
#include "bsp.h"
#define HAL_OK 0
static unsigned s_startup_state,step,off;
static int htim7;
static bool adc_ok=true,timer_ok=true;
static bool Adc_Start(void) { assert(++step==1U); return adc_ok; }
static int HAL_TIM_Base_Start_IT(int *h) { assert(h==&htim7 && ++step==2U); return timer_ok ? HAL_OK : 1; }
static void BspPwm_Disable(void) { ++off; }
"""
        + function_source(source, "Bsp_Start")
        + r"""
int main(void) {
 assert(Bsp_Start()==BSP_NOT_READY && step==0U);
 s_startup_state=2U; assert(Bsp_Start()==BSP_OK && step==2U && off==0U);
 assert(Bsp_Start()==BSP_NOT_READY && step==2U);
 s_startup_state=2U;step=0U;adc_ok=false;
 assert(Bsp_Start()==BSP_IO_ERROR && step==1U && off==1U && s_startup_state==4U);
 s_startup_state=2U;step=0U;adc_ok=true;timer_ok=false;
 assert(Bsp_Start()==BSP_IO_ERROR && step==2U && off==2U && s_startup_state==4U);
 puts("PASS sampling/supervisor order, repeated Start, failure closes output"); return 0;
}
"""
    )


def board_orchestration_fixture(source: str | None = None) -> str:
    """执行真实应用Init/Start，硬件准备和应用对象生命周期分离。"""
    source = source or (ROOT / "firmware/app/composition/app_startup.c").read_text(encoding="utf-8")
    return (
        r"""
#include <assert.h>
#include <stdio.h>
#include <stdbool.h>
static unsigned order[16],count;
static bool s_communication_ready,ready=true;
static void log_step(unsigned n) { order[count++]=n; }
static void AppTasks_Init(void) { log_step(1); }
static void MotorAxis_Init(void) { log_step(2); }
static unsigned MotorAxis_NodeId(void) { return 7; }
static bool YgProtocolRuntime_Init(unsigned n) { assert(n==7);log_step(3);return ready; }
static bool YgProtocolStatusAdapter_Init(void) { log_step(4);return ready; }
static bool bind_motor_stop(void) { log_step(5);return ready; }
static bool YgProtocolOwner_BindServices(void) { log_step(6);return ready; }
static bool bind_device_description(void) { log_step(7);return ready; }
static bool YgProtocolRuntime_Start(void) { log_step(8);return ready; }
static void AppTasks_Start(void) { log_step(9); }
static void BspPwm_Disable(void) { log_step(10); }
"""
        + function_source(source, "App_Init")
        + function_source(source, "App_Start")
        + r"""
int main(void) {
 App_Init();assert(count==7 && s_communication_ready);
 App_Start();assert(count==9);
 for(unsigned i=0;i<9;i++) assert(order[i]==i+1);
 count=0;ready=false;App_Init();assert(!s_communication_ready && count==4 && order[3]==10);
 App_Start();assert(count==6 && order[4]==10 && order[5]==9);
 puts("PASS object binding before BSP Start, communication failure closes output");return 0;
}
"""
    )


def application_entry_fixture(platform: str) -> str:
    """两板真实main必须依次Init BSP、Init APP、Start BSP、Start APP并服务前台。"""
    source = (ROOT / f"projects/{platform}/application/main.c").read_text(encoding="utf-8")
    return (
        r"""
#include <assert.h>
#include <setjmp.h>
#include <stdio.h>
#include "bsp.h"
static jmp_buf exit_entry;
static unsigned trace[8],count,failure,iterations;
static void step(unsigned n) { trace[count++]=n; }
static BspResult Bsp_Init(void) { step(1);return failure==1 ? BSP_IO_ERROR : BSP_OK; }
static void App_Init(void) { step(2); }
static BspResult Bsp_Start(void) { step(3);return failure==2 ? BSP_IO_ERROR : BSP_OK; }
static void App_Start(void) { step(4); }
static void App_Process(void) { step(5);if(++iterations==2)longjmp(exit_entry,1); }
static void startup_failed(void) { longjmp(exit_entry,1); }
""".replace("static BspResult Bsp_", "BspResult Bsp_")
        + function_source(source, "main").replace(
            "int main(void)", "static int application_entry(void)"
        )
        + r"""
int main(void) {
 const unsigned expected[]={1,2,3,4,5,5};
 for(failure=0;failure<3;failure++) {
  count=iterations=0;if(setjmp(exit_entry)==0)application_entry();
  assert(count==(failure==1 ? 1U : failure==2 ? 3U : 6U));
  for(unsigned i=0;i<count;i++)assert(trace[i]==expected[i]);
 }
 puts("PASS dual-board main: BSP/APP lifecycle and failure ordering");return 0;
}
"""
    )


def platform_init_fixture() -> str:
    """核对真实STM32初始化原外设相对顺序及新BSP唯一所有者。"""
    source = (ROOT / "firmware/platform/bsp/mcu/stm32g4/system.c").read_text(encoding="utf-8")
    calls = (
        "BootHw_Init",
        "HAL_Init",
        "system_clock_init",
        "MX_GPIO_Init",
        "MX_DMA_Init",
        "MX_ADC2_Init",
        "Can_Init",
        "Pwm_Init",
        "MX_TIM2_Init",
        "MX_TIM7_Init",
        "Led_Init",
        "MX_ADC1_Init",
        "MX_SPI1_Init",
        "MX_SPI2_Init",
        "__HAL_FLASH_PREFETCH_BUFFER_ENABLE",
        "Adc_Init",
        "system_time_init",
        "Flash_Init",
        "Encoder_Init",
    )
    return (
        '#include <assert.h>\n#include <stdio.h>\n#include "bsp.h"\n#include "bsp_config.h"\n'
        "static unsigned s_startup_state,count,off;\n"
        + "\n".join(
            f"static bool {name}(void) {{ assert(++count=={index});return true; }}"
            for index, name in enumerate(calls, 1)
        )
        + "\nstatic void BspPwm_Disable(void) { ++off; }\n"
        + function_source(source, "Bsp_Init")
        + "\nint main(void) { assert(Bsp_Init()==BSP_OK && count==19 && off==0);"
        "assert(Bsp_Init()==BSP_NOT_READY && count==19);"
        'puts("PASS STM32 19-step initialization, original peripheral order, exactly once");return 0;}\n'
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
        ("application_entry_stm32_test", application_entry_fixture("stm32g4")),
        ("application_entry_at32_test", application_entry_fixture("at32f456")),
        ("platform_init_test", platform_init_fixture()),
        ("board_startup_test", board_startup_fixture()),
        ("board_orchestration_test", board_orchestration_fixture()),
    ):
        path = out / (name + ".c")
        path.write_text(fixture, encoding="utf-8")
        build(name, [str(path)])


if __name__ == "__main__":
    main()
