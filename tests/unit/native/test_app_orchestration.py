"""用完整APP源码验证健康监督、Loader交接和身份发布，可与重构前源码逐事件对照。"""

import argparse
import subprocess
from pathlib import Path

from _bsp_test_support import run_c
from run_position_servo_tests import ROOT


def fixture(source: str, loader: bool) -> str:
    """仅替换APP调用的外部边界，保留真实文件的静态状态和条件编译。"""
    # 旧基线只归一化已删除入口与计数的名称，不改动函数体或执行顺序。
    source = source.replace("s_fast_samples", "g_app_fast_samples").replace(
        "static volatile uint32_t g_app_fast_samples;", "volatile uint32_t g_app_fast_samples;"
    )
    if "void BspApp_OnFastSample(" not in source:
        source = source.replace("void App_FastTick(", "void BspApp_OnFastSample(")
    return (
        f"#define YG_APP_LOADER_IMAGE {int(loader)}\n"
        "#if BSP_BOARD == 2\n#define APP_TARGET_AT32_F456 1\n#endif\n"
        + (
            '#include "yg_protocol.h"\nbool AppProtocol_Init(void);\n'
            "bool Protocol_BindBoot(const BootEntryService *service);\n"
            if "AppProtocol_Init(" in source
            else ""
        )
        + "void MotorAxis_FastTick(void);\n"
        + source
        + (
            "\n#define TEST_STATIC_BOOT 1\n"
            if "const BootEntryService Protocol_Boot" in source
            else "\n#define TEST_STATIC_BOOT 0\n"
        )
        + r"""
#include <assert.h>
#include <setjmp.h>
#include <stdio.h>
#include <string.h>

static uint32_t trace = 2166136261U, now_ms, failure_stage;
static unsigned feeds, disables, enters, confirms, depth;
static bool expect_halt, stopped, lock_allowed = true, healthy = true, link_ready = true;
static uint32_t progress = 1U;
static BspResult watchdog_result = BSP_OK;
static jmp_buf halt_return;
#if YG_APP_LOADER_IMAGE
#if TEST_STATIC_BOOT
#define boot_service Protocol_Boot
#else
static BootEntryService boot_service;
#endif
#endif
static void event(uint32_t kind, uint32_t value) {
 trace = (trace ^ kind) * 16777619U;
 trace = (trace ^ value) * 16777619U;
}
static void bytes(uint32_t kind, const uint8_t *data, unsigned count) {
 for (unsigned i = 0; i < count; ++i) event(kind, data[i]);
}
void MotorAxis_Init(void) { event(1, 0); }
uint8_t MotorAxis_NodeId(void) { event(2, 7); return 7; }
bool Protocol_Init(uint8_t node) { event(3, node); return failure_stage != 1; }
bool AppProtocol_Init(void) { return true; }
bool Protocol_Start(void) { event(5, 0); return failure_stage != 5; }
void BspPwm_Disable(void) {
 event(6, ++disables);
 if (expect_halt) { assert(depth == 1); longjmp(halt_return, 1); }
}
BspResult BspWatchdog_Start(void) { event(7, watchdog_result); return watchdog_result; }
void BspWatchdog_Feed(void) { event(8, ++feeds); }
uint32_t BspSystem_TimeMs(void) { event(9, now_ms); return now_ms; }
uint32_t BspSystem_EnterCritical(void) { event(10, depth); return depth++; }
void BspSystem_ExitCritical(uint32_t saved) { assert(depth == saved + 1); depth = saved; event(11, saved); }
void BspSystem_MemoryBarrier(void) { event(12, 0U); }
BspResult BspSystem_ReadIdentity(uint8_t *record) { event(13, 0); memset(record, 0, 64); return BSP_OK; }
BspResult BspSystem_ReadUid(uint8_t *uid) { event(14, 0); memset(uid, 0x35, 12); return failure_stage == 3 ? BSP_IO_ERROR : BSP_OK; }
bool Protocol_SetInfo(const ProtocolInfo *info,
 const ProtocolCaps *caps) {
 event(15, info->product_id); event(16, info->hardware_revision);
 event(17, info->application_version); event(18, info->boot_version); event(19, info->boot_api);
 bytes(20, info->uid, 12); event(21, caps->features); event(22, caps->supported_modes);
 event(23, caps->max_full_frame); event(24, caps->max_block_data);
 assert(caps->features == 0x1d00U && caps->supported_modes == 0x64eU);
 return failure_stage != 4;
}
bool Protocol_SetInfoPage(uint16_t page, const uint8_t *payload) {
 bytes(25 + page, payload, 32); return true;
}
uint32_t BootHw_ImageCrc32(void) { event(28, 0); return 0x12345678U; }
bool BootHw_IsConfirmed(void) { event(29, 0); return false; }
bool BootHw_IsAvailable(void) { event(30, 0); return true; }
bool Protocol_BindBoot(const BootEntryService *service) {
#if YG_APP_LOADER_IMAGE && !TEST_STATIC_BOOT
 boot_service = *service;
#else
 (void)service;
#endif
 return true;
}
bool MotorLifecycle_ReadBootHealth(uint32_t *value) { *value = progress; event(32, progress); return healthy; }
bool Protocol_IsReady(void) { event(33, link_ready); return link_ready; }
bool BspPwm_ArePhasesDisabled(void) { event(34, stopped); return stopped; }
bool MotorLifecycle_IsStopped(void) { event(35, stopped); return stopped; }
bool MotorLifecycle_LockStop(void) { event(36, lock_allowed); return lock_allowed; }
bool BootHw_EnterLoader(uint8_t node) { assert(depth == 1 && stopped); event(37, node); ++enters; return false; }
bool BootHw_Confirm(uint32_t version, uint32_t crc) { event(38, version); event(39, crc); ++confirms; return true; }
void BootHw_Watchdog(bool valid) { event(40, valid); }
void MotorAxis_ProcessPositionPlan(void) {}
void MotorAxis_ProcessCalibrationPublication(OutputEncoderCalibrationProgress progress) { (void)progress; event(41, 0); }
void AppProtocol_Refresh(void) { event(42, 0); }
void Protocol_Process(uint32_t time) { event(43, time); ++now_ms; }
void MotorAxis_FastTick(void) { event(44, 0); }
void MotorAxis_SampleFast(void) {}
void MotorAxis_ControlFast(void) { event(44, 0); }
void MotorAxis_SupervisorTick(void) { event(45, 0); }
void Protocol_RxInterrupt(void) { event(46, 0); }
bool MotorLifecycle_RequestRemoteStop(void) { return true; }
MotorControlResult MotorLifecycle_RequestCalibration(MotorCalibrationType type) { (void)type; return MOTOR_CONTROL_ACCEPTED; }
MotorControlResult MotorLifecycle_RequestEnable(MotorControlMode mode) { (void)mode; return MOTOR_CONTROL_ACCEPTED; }
MotorControlResult MotorLifecycle_SetTarget(const MotorTarget *target) { (void)target; return MOTOR_CONTROL_ACCEPTED; }

static void check_startup(void) {
 for (failure_stage = 0; failure_stage <= 5; ++failure_stage) {
  unsigned before = disables;
  App_Init();
  bool init_ok = failure_stage == 0 || failure_stage == 2 || failure_stage == 5 || (!YG_APP_LOADER_IMAGE && failure_stage == 6);
  assert(s_communication_ready == init_ok);
  assert(disables == before + !init_ok);
  App_Start();
  assert(s_communication_ready == (init_ok && failure_stage != 5));
  assert(s_watchdog_enabled);
 }
 failure_stage = 0;
 watchdog_result = BSP_NOT_SUPPORTED;
 App_Init(); App_Start();
 assert(!s_watchdog_enabled);
 unsigned before = feeds;
 App_Process(); assert(feeds == before);
 watchdog_result = BSP_IO_ERROR;
 expect_halt = true;
 if (setjmp(halt_return) == 0) { App_Start(); assert(false); }
 expect_halt = false; depth = 0;
 watchdog_result = BSP_OK;
}
static void process_at(uint32_t time) { now_ms = time; App_Process(); }
static void check_watchdog(void) {
 now_ms = 0;
 App_Init(); App_Start();
 unsigned before = feeds;
 BspApp_OnFastSample(); App_SupervisorTick();
 process_at(8); assert(feeds == before);
 process_at(9); assert(feeds == before + 1);
 /* 协议服务推进时钟；看门狗必须使用服务后的时刻。 */
 assert(s_watchdog_last_ms == 10);
 for (unsigned stalled = 0; stalled < 3; ++stalled) {
  if (stalled == 0) BspApp_OnFastSample();
  if (stalled == 1) App_SupervisorTick();
  unsigned old_feeds = feeds, old_disables = disables;
  expect_halt = true;
  if (setjmp(halt_return) == 0) { process_at(s_watchdog_last_ms + 9U); assert(false); }
  expect_halt = false; depth = 0;
  assert(feeds == old_feeds && disables == old_disables + 1);
  now_ms = 20; App_Start();
 }
 now_ms = UINT32_MAX - 5U; App_Start();
 BspApp_OnFastSample(); App_SupervisorTick();
 before = feeds;
 process_at(3); assert(feeds == before + 1 && s_watchdog_last_ms == 4);
 s_watchdog_enabled = false;
 BspApp_OnCanReceive();
}
#if YG_APP_LOADER_IMAGE
static void request(const BootEntryRequest *value, BootEntryResult expected, uint64_t ticket) {
 BootEntryReply reply = {0};
 boot_service.request(NULL, value, &reply);
 assert(reply.result == expected && reply.ack_ticket == ticket);
 event(47, reply.result); event(48, (uint32_t)reply.ack_ticket);
}
static void reset_entry(void) {
 s_entry_requested = false; stopped = false; lock_allowed = true; depth = 0; now_ms = 1000;
}
static void check_loader(void) {
 const BootEntryRequest valid = {0x123456789abcdefULL, 7, 8};
 BootEntryRequest invalid = valid;
 reset_entry();
 boot_service.request(NULL, &valid, NULL);
 request(NULL, BOOT_ENTRY_UNAVAILABLE, 0);
 invalid.boot_epoch = 0; request(&invalid, BOOT_ENTRY_UNAVAILABLE, 0);
 invalid = valid; invalid.source_node = UINT8_MAX; request(&invalid, BOOT_ENTRY_UNAVAILABLE, 0);
 s_boot_supported = false; request(&valid, BOOT_ENTRY_UNAVAILABLE, 0); s_boot_supported = true;
 request(&valid, BOOT_ENTRY_PENDING, 0);
 for (unsigned field = 0; field < 3; ++field) {
  invalid = valid;
  if (field == 0) ++invalid.boot_epoch;
  if (field == 1) ++invalid.sequence;
  if (field == 2) ++invalid.source_node;
  request(&invalid, BOOT_ENTRY_DENIED, 0);
 }
 stopped = true; request(&valid, BOOT_ENTRY_READY, valid.boot_epoch);
 boot_service.tx_complete(NULL, valid.boot_epoch + 1, true); assert(enters == 0);
 boot_service.tx_complete(NULL, valid.boot_epoch, false); assert(enters == 0);
 boot_service.tx_complete(NULL, valid.boot_epoch, true); assert(enters == 0);
 request(&valid, BOOT_ENTRY_FAILED, 0);
 reset_entry(); request(&valid, BOOT_ENTRY_PENDING, 0);
 stopped = true; request(&valid, BOOT_ENTRY_READY, valid.boot_epoch);
 boot_service.tx_complete(NULL, valid.boot_epoch, true); assert(enters == 1 && depth == 0);
 boot_service.tx_complete(NULL, valid.boot_epoch, true); assert(enters == 1);
 reset_entry(); lock_allowed = false; request(&valid, BOOT_ENTRY_DENIED, 0);
 stopped = true; request(&valid, BOOT_ENTRY_DENIED, 0);
 reset_entry(); now_ms = UINT32_MAX - 499U; request(&valid, BOOT_ENTRY_PENDING, 0);
 stopped = true; now_ms = 499; request(&valid, BOOT_ENTRY_READY, valid.boot_epoch);
 now_ms = 500; boot_service.tx_complete(NULL, valid.boot_epoch, true); assert(enters == 1);
 request(&valid, BOOT_ENTRY_FAILED, 0);
 reset_entry(); request(&valid, BOOT_ENTRY_PENDING, 0);
 now_ms = 2000; request(&valid, BOOT_ENTRY_FAILED, 0);
 reset_entry(); request(&valid, BOOT_ENTRY_PENDING, 0);
 stopped = true; request(&valid, BOOT_ENTRY_READY, valid.boot_epoch);
 stopped = false; boot_service.tx_complete(NULL, valid.boot_epoch, true); assert(enters == 1);
 s_entry_requested = false; stopped = true; s_healthy_intervals = 0; s_health_checked_ms = 0;
 for (unsigned i = 1; i <= 12; ++i) {
  ++progress; process_at(i * 100);
  assert(boot_service.confirm(NULL, 1, 2) == (i >= 11));
 }
 assert(s_healthy_intervals == 11 && confirms == 2);
 healthy = false; process_at(1300);
 assert(s_healthy_intervals == 0 && !boot_service.confirm(NULL, 1, 2));
}
#endif
int main(void) {
 check_startup(); check_watchdog();
#if YG_APP_LOADER_IMAGE
 check_loader();
#endif
 printf("PASS APP trace=%08x feeds=%u disables=%u enters=%u confirms=%u\n", trace, feeds, disables, enters, confirms);
 return 0;
}
"""
    )


def main() -> None:
    """按两板和有无Loader编译，选配同输入下重构前后的逐事件摘要对比。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--reference-source", type=Path)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    current = (ROOT / "firmware/app/composition/app.c").read_text(encoding="utf-8")
    extra = [ROOT / "shared/boot/device_identity.c", ROOT / "shared/checksum/crc32.c"]
    for board in (1, 2):
        for loader in (False, True):
            name = f"app_{board}_{int(loader)}"
            run_c(args, name, fixture(current, loader), board, extra)
            if args.reference_source:
                reference = args.reference_source.read_text(encoding="utf-8")
                run_c(args, name + "_before", fixture(reference, loader), board, extra)
                outputs = [
                    subprocess.check_output([str(args.out / (item + ".exe"))])
                    for item in (name, name + "_before")
                ]
                if outputs[0] != outputs[1]:
                    raise RuntimeError(f"{name}重构前后的APP事件轨迹不一致")


if __name__ == "__main__":
    main()
