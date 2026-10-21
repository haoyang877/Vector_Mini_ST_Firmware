"""编译真实Loader发送函数，验证缺ACK取消和实际发送完成判据。"""

import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tests/unit/native"))
from run_position_servo_tests import function_source  # noqa: E402

PRELUDE = r"""
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#define FDCAN_TX_BUFFER0 1U
#define FDCAN_TX_BUFFER1 2U
#define FDCAN_TX_BUFFER2 4U
#define FDCAN_STANDARD_ID 0U
#define FDCAN_DATA_FRAME 0U
#define FDCAN_FD_CAN 1U
#define FDCAN_BRS_ON 1U
#define FDCAN_NO_TX_EVENTS 0U
#define HAL_OK 0U
#define LDR_RSP_BASE 0x7E0U
typedef struct { uint32_t Identifier, IdType, TxFrameType, FDFormat, BitRateSwitch,
    DataLength, TxEventFifoControl; } FDCAN_TxHeaderTypeDef;
static struct { uint32_t TXBCR, TXBTO; } registers;
static struct { uint32_t LatestTxFifoQRequest;
    typeof(registers) *Instance; } hldr_can = {0,&registers};
static uint8_t s_node=4;
static uint32_t s_tx_started_ms, now_ms, pending, adds;
static uint32_t HAL_GetTick(void) { return now_ms; }
static uint32_t len_to_dlc(uint8_t len) { return len; }
static unsigned HAL_FDCAN_IsTxBufferMessagePending(void *handle,uint32_t mask) {
    assert(handle==&hldr_can); return pending&mask;
}
static unsigned HAL_FDCAN_AddMessageToTxFifoQ(void *handle,FDCAN_TxHeaderTypeDef *tx,uint8_t *data) {
    assert(handle==&hldr_can && data!=NULL && tx->Identifier==0x7E4);
    assert(tx->FDFormat==FDCAN_FD_CAN && tx->BitRateSwitch==FDCAN_BRS_ON);
    ++adds; hldr_can.LatestTxFifoQRequest=1; registers.TXBTO=0; return HAL_OK;
}
"""

CASES = r"""
int main(void) {
    uint8_t data[20]={0};
    now_ms=1000;
    assert(loader_link_send(data,sizeof(data)) && adds==1);
    pending=1; now_ms=1099;
    assert(!loader_link_send(data,sizeof(data)) && registers.TXBCR==0 && adds==1);
    now_ms=1100;
    assert(!loader_link_send(data,sizeof(data)) && registers.TXBCR==7 && adds==1);
    pending=0; registers.TXBCR=0;
    assert(!loader_link_sent());
    assert(loader_link_send(data,sizeof(data)) && adds==2 && !loader_link_sent());
    registers.TXBTO=1; assert(loader_link_sent());
    pending=1; assert(!loader_link_sent());
    pending=0; now_ms=0xFFFFFFF0U;
    assert(loader_link_send(data,sizeof(data)));
    pending=1; now_ms=0x53U;
    assert(!loader_link_send(data,sizeof(data)) && registers.TXBCR==0);
    now_ms=0x54U;
    assert(!loader_link_send(data,sizeof(data)) && registers.TXBCR==7);
    puts("PASS actual Loader CAN: bounded pending TX, abort is not sent, retry and timer wrap");
    return 0;
}
"""


def main():
    """使用本地寄存器替身，禁止加载探针或CAN DLL。"""
    compiler, out = sys.argv[1], Path(sys.argv[2])
    out.mkdir(parents=True, exist_ok=True)
    source = (ROOT / "loader/firmware/loader_can.c").read_text(encoding="utf-8")
    source = source.split("/* LOADER_FD_IMPLEMENTATION */", 1)[1]
    fixture = out / "tx_backpressure.c"
    fixture.write_text(
        PRELUDE
        + function_source(source, "loader_link_send")
        + function_source(source, "loader_link_sent")
        + CASES,
        encoding="utf-8",
    )
    command = [compiler] + (["cc"] if Path(compiler).stem == "zig" else [])
    exe = out / "tx_backpressure.exe"
    subprocess.run(
        command
        + [
            "-std=gnu99",
            "-DVECTOR_CAN_CLASSIC_ONLY=0",
            "-O1",
            "-UNDEBUG",
            "-Wall",
            "-Wextra",
            "-Werror",
            str(fixture),
            "-o",
            str(exe),
        ],
        check=True,
    )
    subprocess.run([str(exe)], check=True)


if __name__ == "__main__":
    main()
