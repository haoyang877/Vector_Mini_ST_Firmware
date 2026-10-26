"""验证真实 APP 部署出口及静默 Loader；HAL/邮箱均为内存，不接触硬件。"""

import argparse
import hashlib
import json
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tools"))
from project_paths import NATIVE_INCLUDE_FLAGS, ROOT
from run_position_servo_tests import function_source


def main():
    """生成最小 HAL 夹具，运行两种 APP 策略和经典 Loader 生命周期。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    # 与生产声明一致的 HAL 边界；字段只用于内存断言。
    header = """#ifndef FAKE_FDCAN_H
#define FAKE_FDCAN_H
#include <stdint.h>
#include <stddef.h>
#define HAL_OK 0
#define FDCAN_FRAME_CLASSIC 0
#define FDCAN_FRAME_FD_BRS 2
#define FDCAN_STANDARD_ID 0
#define FDCAN_DATA_FRAME 0
#define FDCAN_FD_CAN 1
#define FDCAN_CLASSIC_CAN 0
#define FDCAN_BRS_ON 1
#define FDCAN_BRS_OFF 0
#define FDCAN_NO_TX_EVENTS 0
#define FDCAN_RX_FIFO0 0
#define FDCAN_TX_QUEUE_OPERATION 1
#define FDCAN_TX_BUFFER0 1
#define FDCAN_TX_BUFFER1 2
#define FDCAN_TX_BUFFER2 4
#define FDCAN_DLC_BYTES_48 14
#define FDCAN_DLC_BYTES_8 8
#define FDCAN_DLC_BYTES_4 4
#define FDCAN_DLC_BYTES_2 2
#define FDCAN_DLC_BYTES_16 10
#define FDCAN_FILTER_RANGE 0
#define FDCAN_FILTER_TO_RXFIFO0 1
#define FDCAN_REJECT 2
#define FDCAN_FILTER_REMOTE 1
#define FDCAN_IT_RX_FIFO0_NEW_MESSAGE 8
typedef struct {unsigned Identifier,IdType,TxFrameType,DataLength,ErrorStateIndicator,BitRateSwitch,FDFormat,TxEventFifoControl,MessageMarker;} FDCAN_TxHeaderTypeDef;
typedef struct {unsigned Identifier,IdType,RxFrameType,DataLength,FDFormat,BitRateSwitch;} FDCAN_RxHeaderTypeDef;
typedef struct {unsigned IdType,FilterIndex,FilterType,FilterConfig,FilterID1,FilterID2;} FDCAN_FilterTypeDef;
typedef struct {struct {unsigned TxFifoQueueMode,DataPrescaler,NominalPrescaler,FrameFormat;} Init;struct {unsigned IR,TXBTO;} *Instance;unsigned LatestTxFifoQRequest;} Handle;
extern Handle hfdcan1;
unsigned HAL_FDCAN_IsTxBufferMessagePending(Handle *,unsigned);
unsigned HAL_FDCAN_AddMessageToTxFifoQ(Handle *,const FDCAN_TxHeaderTypeDef *,const uint8_t *);
unsigned HAL_FDCAN_GetRxMessage(Handle *,unsigned,FDCAN_RxHeaderTypeDef *,uint8_t *);
unsigned HAL_FDCAN_ConfigFilter(Handle *,const FDCAN_FilterTypeDef *);
unsigned HAL_FDCAN_ConfigGlobalFilter(Handle *,unsigned,unsigned,unsigned,unsigned);
unsigned HAL_FDCAN_Start(Handle *);
unsigned HAL_FDCAN_Stop(Handle *);
unsigned HAL_FDCAN_Init(Handle *);
unsigned HAL_FDCAN_ActivateNotification(Handle *,unsigned,unsigned);
void Error_Handler(void);
#endif
"""
    (out / "fdcan.h").write_text(header, encoding="utf-8")
    (out / "stm32g4xx_hal.h").write_text(
        "#include <stddef.h>\n#include <stdint.h>\ntypedef struct {unsigned unused;} FDCAN_HandleTypeDef;\n",
        encoding="utf-8",
    )
    (out / "loader_silent.c").write_text(
        """#include <assert.h>
#include <stdio.h>
#include "loader_link.h"
int main(void) {
    ldr_rx_frame_t frame={0};uint8_t payload[64]={0};
    for(unsigned node=0;node<8;++node) {
        assert(loader_link_init(node));
        for(unsigned tick=0;tick<3000;++tick) {
            assert(!loader_link_rx_pop(&frame));
            assert(!loader_link_send(payload,sizeof(payload)));
            assert(!loader_link_sent());
        }
        loader_link_shutdown();
    }
    puts("PASS real classic Loader: no HAL linkage, startup/reinit and repeated idle RX/TX remain silent");
    return 0;
}
""",
        encoding="utf-8",
    )
    # 真实硬件端口函数使用内存邮箱；经典路径不得写邮箱或触发复位。
    source = (ROOT / "firmware/platform/stm32g4/ports/boot/boot_stm32g4.c").read_text(
        encoding="utf-8"
    )
    guard = """#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "can_deployment.h"
#define BOOT_MAILBOX_BYTES 32
static uint32_t mailbox[8];
#define BOOT_MAILBOX_BASE ((uintptr_t)mailbox)
static unsigned resets,encodes;
static void Boot_MailboxEncode(uint32_t node,uint32_t token,uint8_t *raw)
{ (void)node;(void)token;memset(raw,0x5a,32);++encodes; }
#define __DSB() ((void)0)
#define NVIC_SystemReset() (++resets)
"""
    # FD 才使用这项替身，避免经典构建产生未使用函数警告。
    guard = guard.replace(
        "static void Boot_MailboxEncode",
        "#if !VECTOR_CAN_CLASSIC_ONLY\nstatic void Boot_MailboxEncode",
    )
    guard = guard.replace("#define __DSB()", "#endif\n#define __DSB()")
    guard += (
        function_source(source, "boot_hw_reset_to_loader")
        + """
int main(void) {
    boot_hw_reset_to_loader(3,10);
    assert(resets==(VECTOR_CAN_CLASSIC_ONLY?0U:1U));
    assert(encodes==resets);
    assert(mailbox[7]==(VECTOR_CAN_CLASSIC_ONLY?0U:0x54494d43U));
    puts("PASS real APP mailbox/reset deployment guard");return 0;
}
"""
    )
    (out / "boot_guard.c").write_text(guard, encoding="utf-8")
    handoff = """#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "can_deployment.h"
#include "boot_handoff.h"
enum {Motor_Disable};
static struct {unsigned ModeNow;} MotorControl;
static unsigned ModeLast,locks,resets,sends;
static bool BootHandoff_Pending(void) { return true; }
static void MotorControl_LockMaintenance(void) { ++locks; }
static void motor_hw_outer_barrier(void) { }
static unsigned time_hw_now_ms(void) { return 10; }
static bool motor_hw_phase_outputs_disabled(void) { return true; }
static bool comm_hw_can_reply_sent(void) { return true; }
static bool ModeSwitch_Handle(unsigned mode) { MotorControl.ModeNow=mode;return true; }
static bool comm_hw_can_try_send_reply(uint16_t id,const uint8_t *data,unsigned size)
{ (void)id;(void)data;(void)size;++sends;return true; }
static uint8_t BootHandoff_Node(void) { return 3; }
static void BootHandoff_ReplyQueued(uint32_t now) { (void)now; }
static BootAction BootHandoff_Poll(uint32_t now,bool stopped,bool off,bool sent)
{ (void)now;(void)stopped;(void)off;(void)sent;return BOOT_RESET; }
static void boot_hw_reset_to_loader(unsigned node,unsigned time)
{ (void)node;(void)time;++resets; }
""".replace(
        '#include "boot_handoff.h"',
        "typedef enum {BOOT_WAIT,BOOT_STOP,BOOT_REPLY_OK,BOOT_REPLY_FAILED,BOOT_RESET} BootAction;",
    )
    app_source = (ROOT / "firmware/app/boot_handoff_app.c").read_text(encoding="utf-8")
    handoff += (
        function_source(app_source, "BootHandoff_AppPoll")
        + """
int main(void) {
    BootHandoff_AppPoll();
    assert(locks==(VECTOR_CAN_CLASSIC_ONLY?0U:1U));
    assert(resets==locks && sends==0);
    puts("PASS actual APP handoff: stale pending state cannot bypass classic deployment");
    return 0;
}
"""
    )
    (out / "handoff_guard.c").write_text(handoff, encoding="utf-8")
    compiler = [args.cc] + (["cc"] if Path(args.cc).stem == "zig" else [])
    common = (
        compiler
        + NATIVE_INCLUDE_FLAGS
        + [
            "-std=c99",
            "-O2",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-UNDEBUG",
            "-I",
            str(out),
            "-I",
            str(ROOT / "firmware/services/boot_handoff"),
            "-I",
            str(ROOT / "loader/firmware"),
        ]
    )
    logs = []

    def run(command):
        result = subprocess.run(command, cwd=ROOT, capture_output=True, text=True)
        logs.append(result.stdout + result.stderr)
        (out / "tests.log").write_text("".join(logs), encoding="utf-8")
        print(logs[-1], end="", flush=True)
        result.check_returncode()

    for policy in (0, 1):
        for name, sources in (
            (
                "app",
                [
                    ROOT / "tests/unit/can_deployment_lifecycle_test.c",
                    ROOT / "firmware/platform/stm32g4/ports/comm/comm_status_stm32g4.c",
                    ROOT / "firmware/services/boot_handoff/boot_handoff.c",
                ],
            ),
            ("boot", [out / "boot_guard.c"]),
            ("handoff", [out / "handoff_guard.c"]),
        ):
            exe = out / f"{name}_{policy}.exe"
            run(
                common
                + [f"-DVECTOR_CAN_CLASSIC_ONLY={policy}"]
                + list(map(str, sources))
                + ["-o", str(exe)]
            )
            run([str(exe)])
    exe = out / "loader_silent.exe"
    run(
        common
        + [
            "-DVECTOR_CAN_CLASSIC_ONLY=1",
            str(out / "loader_silent.c"),
            str(ROOT / "loader/firmware/loader_can.c"),
            "-o",
            str(exe),
        ]
    )
    run([str(exe)])
    sources = [
        "shared/can_deployment.h",
        "loader/firmware/loader_can.c",
        "firmware/platform/stm32g4/ports/comm/comm_status_stm32g4.c",
        "firmware/services/boot_handoff/boot_handoff.c",
        "firmware/platform/stm32g4/ports/boot/boot_stm32g4.c",
    ]
    (out / "summary.json").write_text(
        json.dumps(
            {
                "status": "PASS",
                "hardware_accessed": False,
                "policies": [0, 1],
                "source_sha256": {
                    p: hashlib.sha256((ROOT / p).read_bytes()).hexdigest() for p in sources
                },
            },
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )


if __name__ == "__main__":
    main()
