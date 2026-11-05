"""验证停机保存准入、真实schema14往返及Flash失败传播，全部I/O由内存替代。"""

from _bsp_test_support import arguments, run_c
from run_position_servo_tests import ROOT, function_source


def fixture() -> str:
    """使用实际保存、零偏校验和记录编解码，仅替换生命周期与Flash边界。"""
    axis = (ROOT / "firmware/app/runtime/motor_axis.c").read_text(encoding="utf-8")
    current = (ROOT / "firmware/motor/sensing/motor_current.c").read_text(encoding="utf-8")
    return (
        r"""
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "motor_axis.h"
#include "motor_lifecycle.h"
#include "motor_sensing.h"
#include "encoder_calibration.h"
#include "parameter_record.h"
#include "parameter_values.h"
#include "bsp_flash.h"

static MotorConfiguration s_motor_configuration;
static MotorControlState s_motor_state;
static CurrentOffsetCalibration s_current_offset_calibration;
static InterfaceParam s_parameter_record, stored_record;
static MotorState state;
static bool stopped, released, inject_fault, inject_start;
static unsigned writes, uploads;
static BspResult write_result;
MotorState MotorLifecycle_GetState(void) { return state; }
bool MotorLifecycle_IsStopped(void) { return stopped; }
bool EncoderCalibration_IsIdle(void) { return released; }
BspResult BspFlash_Write(const void *data, size_t length) {
 assert(state == MOTOR_STATE_IDLE && stopped && released);
 assert(s_motor_state.fault_code == MOTOR_FAULT_NONE);
 assert(data == &s_parameter_record && length == 2288U);
 assert(s_parameter_record.record_crc32 == Param_RecordCrc(&s_parameter_record));
 ++writes;
 if (write_result == BSP_OK) { memcpy(&stored_record, data, length); }
 return write_result;
}
static void observed_upload(InterfaceParam *record, const MotorConfiguration *configuration) {
 ++uploads;
 Param_Upload(record, configuration);
 if (inject_fault) { s_motor_state.fault_code = MOTOR_FAULT_ENCODER; }
 if (inject_start) { stopped = false; }
}
"""
        + function_source(current, "MotorCurrentOffsets_IsValid")
        + "\n#define Param_Upload observed_upload\n"
        + function_source(axis, "MotorAxis_SaveCalibration")
        + r"""
#undef Param_Upload
static void reset(void) {
#if BSP_BOARD == BSP_BOARD_AT32F456
 assert(Param_CreateAxis(3, 0, 0, 2, NULL, &s_motor_configuration));
#else
 Param_Return_Default(&s_motor_configuration);
#endif
 s_motor_configuration.model.thermal = (MotorWindingThermalConfig){3.0f,7.0f,65.98486785f,.8f,.7f,1.0f,.5f};
 s_motor_configuration.device.phase_a_offset_counts = 2048.0f;
 s_motor_configuration.device.phase_b_offset_counts = 2048.0f;
 s_motor_configuration.device.phase_c_offset_counts = 2048.0f;
 s_motor_configuration.configuration_confirmed = true;
 s_motor_configuration.device.encoder.calib_flag = 7U;
 s_motor_configuration.device.encoder.reverse = 1U;
 s_motor_configuration.device.encoder.electrical_zero_q15 = 12345U;
 s_motor_configuration.device.encoder.mechanical_zero_q15 = 54321U;
 for (unsigned i = 0; i < ENCODER_OFFSET_LUT_SIZE; ++i) {
  s_motor_configuration.device.encoder.linearization_lut_q15[i] = (int16_t)(i % 65U) - 32;
 }
 memset(&s_motor_state, 0, sizeof(s_motor_state));
 s_current_offset_calibration.complete = true;
 s_current_offset_calibration.failed = false;
 state = MOTOR_STATE_IDLE; stopped = released = true;
 inject_fault = inject_start = false;
 write_result = BSP_OK;
 writes = uploads = 0U;
 assert(ParameterValue_ConfigurationValid(&s_motor_configuration));
}
static void rejected(MotorControlResult expected) {
 unsigned before = writes, encoded = uploads;
 assert(MotorAxis_SaveCalibration() == expected);
 assert(writes == before && uploads == encoded);
}
int main(void) {
 reset();
 for (unsigned flags = 1U; flags <= 7U; ++flags) {
  s_motor_configuration.device.encoder.calib_flag = flags;
  assert(MotorAxis_SaveCalibration() == MOTOR_CONTROL_ACCEPTED);
  MotorConfiguration loaded;
  Param_Return_Default(&loaded);
  Param_Download(&stored_record, sizeof(stored_record), &loaded);
  assert(loaded.configuration_source == MOTOR_CONFIGURATION_STORED && loaded.configuration_confirmed);
  assert(loaded.device.encoder.calib_flag == flags && loaded.device.encoder.reverse == 1U);
  assert(loaded.device.encoder.electrical_zero_q15 == 12345U);
  assert(loaded.device.encoder.mechanical_zero_q15 == 54321U);
  assert(memcmp(loaded.device.encoder.linearization_lut_q15,
                s_motor_configuration.device.encoder.linearization_lut_q15,
                sizeof(loaded.device.encoder.linearization_lut_q15)) == 0);
 }
 assert(writes == 7U && uploads == 7U);
 /* 未定义位仍拒绝，即使整条CRC正确。 */
 stored_record.encoder_calib_flag = 8U;
 stored_record.record_crc32 = Param_RecordCrc(&stored_record);
 MotorConfiguration loaded;
 Param_Download(&stored_record, sizeof(stored_record), &loaded);
 assert(loaded.configuration_source == MOTOR_CONFIGURATION_CORRUPT);

 reset(); state = MOTOR_STATE_RUNNING; rejected(MOTOR_CONTROL_BUSY);
 state = MOTOR_STATE_CALIBRATION; rejected(MOTOR_CONTROL_BUSY);
 state = MOTOR_STATE_SELF_TEST; rejected(MOTOR_CONTROL_BUSY);
 state = MOTOR_STATE_IDLE; stopped = false; rejected(MOTOR_CONTROL_BUSY);
 stopped = true; released = false; rejected(MOTOR_CONTROL_BUSY);
 released = true; s_motor_state.fault_code = MOTOR_FAULT_ENCODER; rejected(MOTOR_CONTROL_FAULT);
 s_motor_state.fault_code = MOTOR_FAULT_NONE;
 s_current_offset_calibration.complete = false; rejected(MOTOR_CONTROL_DENIED);
 s_current_offset_calibration.complete = true;
 s_current_offset_calibration.failed = true; rejected(MOTOR_CONTROL_DENIED);
 s_current_offset_calibration.failed = false;
 s_motor_configuration.device.phase_a_offset_counts = NAN; rejected(MOTOR_CONTROL_DENIED);
 s_motor_configuration.device.phase_a_offset_counts = 2048.0f;
 s_motor_configuration.application.axis_profile_valid = false; rejected(MOTOR_CONTROL_DENIED);
 s_motor_configuration.application.axis_profile_valid = true;
 s_motor_configuration.device.encoder.calib_flag = 0U; rejected(MOTOR_CONTROL_DENIED);
 s_motor_configuration.device.encoder.calib_flag = 8U; rejected(MOTOR_CONTROL_DENIED);
 s_motor_configuration.device.encoder.calib_flag = 7U;
 s_motor_configuration.device.encoder.reverse = 2U; rejected(MOTOR_CONTROL_DENIED);

 reset(); inject_fault = true;
 assert(MotorAxis_SaveCalibration() == MOTOR_CONTROL_FAULT && writes == 0U && uploads == 1U);
 reset(); inject_start = true;
 assert(MotorAxis_SaveCalibration() == MOTOR_CONTROL_BUSY && writes == 0U && uploads == 1U);

 const BspResult failures[] = {BSP_NOT_SUPPORTED, BSP_NOT_READY, BSP_INVALID_ARGUMENT, BSP_IO_ERROR};
 for (unsigned i = 0; i < sizeof(failures)/sizeof(failures[0]); ++i) {
  reset(); write_result = failures[i];
  assert(MotorAxis_SaveCalibration() == MOTOR_CONTROL_PERSIST_FAILED && writes == 1U);
 }
 reset(); write_result = BSP_REJECTED;
 assert(MotorAxis_SaveCalibration() == MOTOR_CONTROL_DENIED && writes == 1U);
 puts("PASS calibration save: idle admission, recheck, schema14 roundtrip, all calibration flags, Flash errors");
 return 0;
}
"""
    )


def watchdog_fixture() -> str:
    """验证AT32仅停机擦写时扩展WDT窗口，完成或失败后恢复，不在等待中喂狗。"""
    source = (ROOT / "firmware/platform/bsp/mcu/at32f456/system.c").read_text(encoding="utf-8")
    return (
        r"""
#include <assert.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
static struct { uint32_t cmd, rld, div; volatile uint32_t sts; } watchdog;
#define WDT (&watchdog)
static bool s_watchdog_started, s_flash_watchdog_active;
static uint32_t s_flash_watchdog_reload;
static bool off = true;
static uint32_t masked = 1U, exception;
bool BspPwm_ArePhasesDisabled(void) { return off; }
uint32_t __get_PRIMASK(void) { return masked; }
uint32_t __get_IPSR(void) { return exception; }
"""
        + function_source(source, "set_watchdog_reload")
        + function_source(source, "McuSystem_SetFlashWatchdog")
        + r"""
int main(void) {
 watchdog.rld = 511U; watchdog.div = 0U;
 assert(McuSystem_SetFlashWatchdog(true) && McuSystem_SetFlashWatchdog(false));
 assert(watchdog.rld == 511U && watchdog.cmd == 0U && !s_flash_watchdog_active);
 s_watchdog_started = true;
 off = false; assert(!McuSystem_SetFlashWatchdog(true)); off = true;
 masked = 0U; assert(!McuSystem_SetFlashWatchdog(true)); masked = 1U;
 exception = 17U; assert(!McuSystem_SetFlashWatchdog(true)); exception = 0U;
 assert(watchdog.rld == 511U && watchdog.cmd == 0U);
 for (uint32_t reload = 511U; reload <= 4095U; reload = reload * 2U + 1U) {
  watchdog.rld = reload;
  assert(McuSystem_SetFlashWatchdog(true));
  assert(s_flash_watchdog_active && watchdog.rld == 4095U && watchdog.cmd == 0xAAAAU);
  assert(watchdog.div == 0U && s_flash_watchdog_reload == reload);
  assert(!McuSystem_SetFlashWatchdog(true));
  assert(McuSystem_SetFlashWatchdog(false));
  assert(!s_flash_watchdog_active && watchdog.rld == reload && watchdog.cmd == 0xAAAAU);
 }
 watchdog.sts = 1U;
 assert(!McuSystem_SetFlashWatchdog(true) && !s_flash_watchdog_active);
 watchdog.sts = 0U;
 assert(McuSystem_SetFlashWatchdog(true));
 watchdog.sts = 1U;
 assert(!McuSystem_SetFlashWatchdog(false));
 assert(s_flash_watchdog_active && watchdog.cmd == 0x5555U);
 watchdog.sts = 0U;
 assert(McuSystem_SetFlashWatchdog(false) && !s_flash_watchdog_active);
 puts("PASS AT32 Flash watchdog: off-only, masked foreground, bounded window, restore, no busy-loop feeds");
 return 0;
}
"""
    )


def main() -> None:
    """分别按两板配置执行真实记录保存与重新装载。"""
    args = arguments(__doc__)
    extra = [
        ROOT / path
        for path in (
            "firmware/parameters/parameter_record.c",
            "firmware/parameters/parameter_values.c",
            "firmware/parameters/motor_axis_profile.c",
            "firmware/motor/protection/motor_thermal.c",
            "firmware/motor/protection/motor_thermal_config.c",
            "firmware/motor/control/speed_feedforward.c",
            "firmware/motor/math/control_math.c",
            "shared/checksum/crc32.c",
        )
    ]
    for board in (1, 2):
        run_c(args, f"calibration_save_{board}", fixture(), board, extra)
    run_c(args, "flash_watchdog", watchdog_fixture(), 2)


if __name__ == "__main__":
    main()
