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
    """执行真实启动入口，验证采样/计数器/监督顺序及失败关相。"""
    source = (ROOT / "firmware/platform/bsp/mcu/stm32g4/system.c").read_text(encoding="utf-8")
    return (
        r"""
#include <assert.h>
#include <stdio.h>
#include "bsp.h"
static unsigned s_startup_state,step,off;
static struct {unsigned CR1;} timer;
static int handle;
#define MCU_PWM_TIMER (&timer)
#define MCU_CONTROL_TIMER_HANDLE handle
#define TIM_CR1_CEN 1U
#define HAL_OK 0
static bool timer_ok=true;
static void Adc_Start(void) { assert(++step==1U && !(timer.CR1&1U)); }
static int HAL_TIM_Base_Start_IT(int *h) {
 assert(h==&handle && ++step==2U && (timer.CR1&1U)); return timer_ok ? HAL_OK : 1;
}
static void BspPwm_Disable(void) { ++off; }
"""
        + function_source(source, "Bsp_Start")
        + r"""
int main(void) {
 assert(Bsp_Start()==BSP_NOT_READY && step==0U);
 s_startup_state=2U; assert(Bsp_Start()==BSP_OK && step==2U && off==0U);
 assert(Bsp_Start()==BSP_NOT_READY && step==2U);
 s_startup_state=2U;step=0U;timer.CR1=0;timer_ok=false;
 assert(Bsp_Start()==BSP_IO_ERROR && step==2U && off==1U && s_startup_state==4U);
 assert(Bsp_Start()==BSP_NOT_READY && step==2U);
 puts("PASS sampling, PWM counter, supervisor order and failure closes output"); return 0;
}
"""
    )


def board_orchestration_fixture(source: str | None = None) -> str:
    """执行真实应用Init/Start，核对协议、采样和健康监督先后关系。"""
    source = source or (ROOT / "firmware/app/composition/app.c").read_text(encoding="utf-8")
    return (
        r"""
#include <assert.h>
#include <stdio.h>
#include <stdbool.h>
#include "bsp_system.h"
static unsigned order[24],count;
static bool s_communication_ready,ready=true,s_watchdog_enabled;
volatile uint32_t g_app_fast_samples;
static volatile uint32_t s_supervisor_ticks;
static uint32_t s_watchdog_last_ms,s_watchdog_last_fast,s_watchdog_last_slow;
static void log_step(unsigned n) { order[count++]=n; }
static void MotorAxis_Init(void) { log_step(1); }
static unsigned MotorAxis_NodeId(void) { return 7; }
static bool Protocol_Init(unsigned n) { assert(n==7);log_step(2);return ready; }
static bool bind_device_description(void) { log_step(4);return ready; }
static bool Protocol_Start(void) { log_step(5);return ready; }
BspResult BspWatchdog_Start(void) { log_step(6);return BSP_OK; }
uint32_t BspSystem_TimeMs(void) { log_step(7);return 123; }
static void BspPwm_Disable(void) { log_step(8); }
static void halt_disabled(void) { assert(false); }
"""
        + function_source(source, "start_watchdog")
        + function_source(source, "App_Init")
        + function_source(source, "App_Start")
        + r"""
int main(void) {
 g_app_fast_samples=s_supervisor_ticks=9;
 App_Init();assert(count==3 && s_communication_ready && g_app_fast_samples==0 && s_supervisor_ticks==0);
 App_Start();assert(count==6 && s_watchdog_enabled && s_watchdog_last_ms==123);
 const unsigned expected[]={1,2,4,5,6,7};
 for(unsigned i=0;i<6;i++) assert(order[i]==expected[i]);
 count=0;ready=false;App_Init();assert(!s_communication_ready && count==3 && order[2]==8);
 App_Start();assert(count==6 && order[3]==8 && order[4]==6 && order[5]==7);
 puts("PASS app binding order, watchdog startup and failed communication shuts output");return 0;
}
"""
    )


def application_entry_fixture(platform: str) -> str:
    """两板真实main必须依次Init BSP、Init APP、Start BSP、Start APP并服务前台。"""
    source = (ROOT / f"projects/{platform}/application/main.c").read_text(encoding="utf-8")
    rtt = ""
    if platform == "at32f456":
        rtt = r"""
void RTT_Init(void) { assert(count==2U); }
"""
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
        + rtt
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
    """核对真实STM32初始化顺序、唯一所有者及Flash失败后不可重复启动。"""
    source = (ROOT / "firmware/platform/bsp/mcu/stm32g4/system.c").read_text(encoding="utf-8")
    calls = (
        "BootHw_Init",
        "HAL_Init",
        "system_clock_init",
        "MX_GPIO_Init",
        "McuCan_Init",
        "McuPwm_Init",
        "MCU_CONTROL_TIMER_INIT",
        "__HAL_FLASH_PREFETCH_BUFFER_ENABLE",
        "McuAdc_Init",
        "Flash_Init",
        "McuEncoder_Init",
    )
    stubs = []
    for index, name in enumerate(calls, 1):
        result = "bool" if name in ("BootHw_Init", "Flash_Init") else "void"
        returned = (
            "return flash_ok;"
            if name == "Flash_Init"
            else "return true;"
            if result == "bool"
            else ""
        )
        stubs.append(f"static {result} {name}(void) {{ assert(++count=={index});{returned} }}")
    return (
        '#include <assert.h>\n#include <stdio.h>\n#include "bsp.h"\n#include "board_config.h"\n'
        "static unsigned s_startup_state,count,off; static bool flash_ok=true;\n"
        + "\n".join(stubs)
        + "\nstatic void BspPwm_Disable(void) { ++off; }\n"
        + function_source(source, "Bsp_Init")
        + r"""
int main(void) {
 assert(Bsp_Init()==BSP_OK && count==11 && off==0 && s_startup_state==2);
 assert(Bsp_Init()==BSP_NOT_READY && count==11);
 s_startup_state=count=0;flash_ok=false;
 assert(Bsp_Init()==BSP_IO_ERROR && count==10 && off==1 && s_startup_state==4);
 assert(Bsp_Init()==BSP_NOT_READY && count==10);
 puts("PASS STM32 initialization order, exactly once, Flash failure closes output");return 0;
}
"""
    )


def at32_lifecycle_fixture() -> str:
    """验证AT32无启动状态缓存的初始化顺序及独立看门狗重复启动保护。"""
    source = (ROOT / "firmware/platform/bsp/mcu/at32f456/system.c").read_text(encoding="utf-8")
    return (
        r"""
#include <assert.h>
#include <stdio.h>
#include "bsp.h"
#include "board_config.h"
static unsigned trace[8],count,watchdog_calls;
static bool s_watchdog_started,watchdog_ok=true;
static void step(unsigned value) { assert(count<8U);trace[count++]=value; }
static bool BootHw_Init(void) { step(1);return false; }
static void McuSystem_Init(void) { step(2); }
static void McuCan_Init(void) { step(3); }
static void McuAdc_Init(void) { step(4); }
static bool Flash_Init(void) { step(5);return true; }
static void McuEncoder_Init(void) { step(6); }
static void Adc_Start(void) { step(7); }
static void McuSystem_Start(void) { step(8); }
static bool watchdog_init(void) { ++watchdog_calls;return watchdog_ok; }
"""
        + function_source(source, "Bsp_Init")
        + function_source(source, "Bsp_Start")
        + function_source(source, "BspWatchdog_Start")
        + r"""
static void expect_trace(const unsigned *expected,unsigned length) {
 assert(count==length);
 for(unsigned i=0U;i<length;++i) { assert(trace[i]==expected[i]); }
}
int main(void) {
 const unsigned ok[]={1,2,3,4,5,6,7,8};
 assert(Bsp_Init()==BSP_OK && count==6U);
 assert(Bsp_Start()==BSP_OK);
 expect_trace(ok,8U);
 watchdog_ok=false;
 assert(BspWatchdog_Start()==BSP_IO_ERROR && !s_watchdog_started && watchdog_calls==1U);
 watchdog_ok=true;
 assert(BspWatchdog_Start()==BSP_OK && s_watchdog_started && watchdog_calls==2U);
 assert(BspWatchdog_Start()==BSP_NOT_READY && watchdog_calls==2U);
 puts("PASS AT32 lifecycle: no startup state, ordered initialization, independent watchdog guard");
 return 0;
}
"""
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
    compiler = (
        [cc]
        + (["cc"] if Path(cc).stem == "zig" else [])
        + NATIVE_INCLUDE_FLAGS
        + ["-DBSP_BOARD=1", "-DFOC_ACTIVE_MOTOR_PROFILE=1"]
    )
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
            "firmware/motor/control/position_control.c",
            "firmware/motor/control/pi_controller.c",
        ],
    )
    for name, fixture in (
        ("application_entry_stm32_test", application_entry_fixture("stm32g4")),
        ("application_entry_at32_test", application_entry_fixture("at32f456")),
        ("platform_init_test", platform_init_fixture()),
        ("board_startup_test", board_startup_fixture()),
        ("board_orchestration_test", board_orchestration_fixture()),
        ("at32_lifecycle_test", at32_lifecycle_fixture()),
    ):
        path = out / (name + ".c")
        path.write_text(fixture, encoding="utf-8")
        build(name, [str(path)])


if __name__ == "__main__":
    main()
