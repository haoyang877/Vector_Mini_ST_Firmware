"""验证停机保存准入、真实schema14往返及Flash失败传播，全部I/O由内存替代。"""

import json

from _bsp_test_support import arguments, run_c
from run_position_servo_tests import ROOT, function_source


def fixture() -> str:
    """使用实际保存、零偏校验和记录编解码，仅替换生命周期与Flash边界。"""
    axis = (ROOT / "firmware/app/runtime/motor_axis.c").read_text(encoding="utf-8")
    current = (ROOT / "firmware/motor/sensing/motor_current.c").read_text(encoding="utf-8")
    golden = json.loads((ROOT / "tests/fixtures/protocol/hall_calibration_v1.json").read_text())
    vectors = "\n".join(
        f"static const uint8_t {name}[]={{" + ",".join(map(str, bytes.fromhex(golden[name]))) + "};"
        for name in ("successful", "no_samples")
    )
    return (
        r"""
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "motor_axis.h"
#include "linear_hall.h"
#include "motor_lifecycle.h"
#include "motor_sensing.h"
#include "encoder_calibration.h"
#include "parameter_record.h"
#include "parameter_values.h"
#include "bsp_flash.h"
#include "bsp_system.h"

static MotorConfiguration s_motor_configuration;
static MotorControlState s_motor_state;
static CurrentOffsetCalibration s_current_offset_calibration;
static InterfaceParam s_parameter_record, stored_record;
static MotorState state;
static bool stopped, released, inject_fault, inject_start;
static bool publication, cancel_during_upload;
static uint32_t critical;
#if BSP_ENABLED_LINEAR_HALL
"""
        + f"\n#define HALL_OFFSET {golden['offset']}U\n{vectors}\n"
        + r"""
static LinearHall s_linear_hall;
static LinearHallCalibration s_hall_sweep;
#endif
static MotorCalibrationType kind;
static bool encoder_publish_ok;
static unsigned completions;
static unsigned writes, uploads;
static BspResult write_result;
MotorState MotorLifecycle_GetState(void) { return state; }
bool MotorLifecycle_IsStopped(void) { return stopped; }
bool EncoderCalibration_IsIdle(void) { return released; }
uint32_t BspSystem_EnterCritical(void) { return critical++; }
void BspSystem_ExitCritical(uint32_t previous) { assert(critical==previous+1); critical=previous; }
bool MotorLifecycle_ReadPendingCalibrationPublication(uint32_t *epoch) {
 if (!publication || s_motor_state.fault_code!=MOTOR_FAULT_NONE) return false;
 *epoch=33U; return true;
}
MotorCalibrationType MotorLifecycle_GetCalibrationType(void) { return kind; }
static EncoderState s_rotor_encoder;
static uint32_t now_ms;
uint32_t BspSystem_TimeMs(void) { return now_ms; }
float Encoder_GetMechVelContinuous(const EncoderState *encoder) { return encoder->mechanical_velocity_continuous_rad_s; }
bool MotorLifecycle_ContinueElectricalZero(uint32_t epoch) {
 assert(epoch==33 && released && critical && publication);
 kind=MOTOR_CALIBRATION_JOINT_ELECTRICAL_ZERO; publication=false; return true;
}
bool EncoderCalibration_PublishLut(EncoderCalibration *cal, EncoderState *encoder) {
 assert(critical && publication && encoder==&s_rotor_encoder);
 if(encoder_publish_ok) { cal->calib_flag=ENC_CALIB_LINEARIZED; released=true; }
 return encoder_publish_ok;
}
void MotorLifecycle_CompleteCalibrationPublication(uint32_t epoch, bool ok) {
 assert(epoch==33U && critical && publication);
 publication=false; if(ok) ++completions;
}
void MotorFault_Report(MotorControlState *motor,MotorFaultCode fault) { motor->fault_code=fault; }
BspResult BspFlash_Write(const void *data, size_t length) {
 assert(critical && released && ((state == MOTOR_STATE_IDLE && stopped) || publication));
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
 if (cancel_during_upload) { publication=false; }
 if (inject_fault) { s_motor_state.fault_code = MOTOR_FAULT_ENCODER; }
 if (inject_start) { stopped = false; }
}
"""
        + function_source(current, "MotorCurrentOffsets_IsValid")
        + "\n#define Param_Upload observed_upload\n"
        + function_source(axis, "can_save_calibration")
        + function_source(axis, "save_calibration")
        + function_source(axis, "MotorAxis_SaveCalibration")
        + function_source(axis, "MotorAxis_ProcessCalibrationPublication")
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
 publication=false; cancel_during_upload=false; encoder_publish_ok=true; completions=0; kind=MOTOR_CALIBRATION_NONE;
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

#if BSP_ENABLED_LINEAR_HALL
 LinearHall hall;
 LinearHall_Init(&hall);
 assert(!hall.calibration_success && hall.calibration_faults==LINEAR_HALL_CAL_FAULT_NONE);
 LinearHall_Update(&hall,101,3693);
 assert(hall.val.sin_cal==-1 && hall.val.cos_cal==1);
 LinearHall_Update(&hall,3760,119);
 assert(hall.val.sin_cal==1 && hall.val.cos_cal==-1);
 LinearHall_Update(&hall,1930.5f,1906.0f);
 assert(hall.val.sin_cal==0 && hall.val.cos_cal==0);
 assert(hall.sin_off_counts==1930.5f && hall.cos_off_counts==1906 && hall.sin_amp==1829.5f && hall.cos_amp==1787);
 for(unsigned i=0; i<720; ++i) {
  float theta = 6.2831853f*i/720.0f;
  float s=1930.5f+1829.5f*sinf(theta), c=1906+1787*cosf(theta);
  LinearHall_Update(&hall,s,c);
  assert(hall.val.sin_counts==s && hall.val.cos_counts==c);
  assert(fabsf(hall.val.sin_cal-sinf(theta)) < 1e-6f && fabsf(hall.val.cos_cal-cosf(theta)) < 1e-6f);
  assert(fabsf(remainderf(hall.val.angle_rad-theta,6.2831853f)) < 1e-6f);
  assert(hall.val.angle_rad>=0 && hall.val.angle_rad<6.2831853f);
 }
 LinearHallCalibration sweep;
 LinearHall_CalibrationBegin(&sweep);
 LinearHall_CalibrationSample(&sweep,NAN,999);
 assert(!LinearHall_CalibrationFinish(&sweep,&hall));
 assert(!hall.calibration_success && hall.calibration_faults==LINEAR_HALL_CAL_FAULT_NO_SAMPLES);
 assert(!LinearHall_CalibrationFinish(&sweep,&hall) && hall.calibration_faults==LINEAR_HALL_CAL_FAULT_NO_SAMPLES);
 LinearHall_CalibrationBegin(&sweep);
 for (unsigned i=0;i<=360;++i) {
  float theta=-6.2831855f*(float)i/360.0f;
  LinearHall_CalibrationSample(&sweep,1700+800*sinf(theta),2400+1300*cosf(theta));
 }
 assert(LinearHall_CalibrationFinish(&sweep,&hall));
 assert(hall.calibration_success && hall.calibration_faults==LINEAR_HALL_CAL_FAULT_NONE);
 assert(!LinearHall_CalibrationFinish(&sweep,&hall) && hall.calibration_success);
 assert(hall.sin_off_counts==1700 && hall.sin_amp==800);
 assert(hall.cos_off_counts==2400 && hall.cos_amp==1300);
 LinearHall_Update(&hall,2100,1750);
 assert(hall.val.sin_cal==0.5f && hall.val.cos_cal==-0.5f);
 assert(fabsf(hall.val.angle_rad-2.3561945f)<1e-6f);
 /* 缺样不能改变任一通道的极值。 */
 LinearHall_CalibrationBegin(&sweep);
 LinearHall_CalibrationSample(&sweep,10,2486);
 LinearHall_CalibrationSample(&sweep,1610,4086);
 LinearHall_CalibrationSample(&sweep,NAN,4095);
 assert(LinearHall_CalibrationFinish(&sweep,&hall));
 assert(hall.sin_off_counts==810 && hall.cos_off_counts==3286 && hall.sin_amp==800 && hall.cos_amp==800);
 /* 任一通道超限，或其幅值仅为799.5 count，都不更新系数。 */
 const float rejected_extrema[][4] = {
  {9,100,2000,2000}, {100,9,2000,2000}, {100,100,4087,2000}, {100,100,2000,4087},
  {100,100,1699,2000}, {100,100,2000,1699}, {0,3000,4095,4087}
 };
 const uint8_t expected_faults[] = {
  LINEAR_HALL_CAL_FAULT_SIN_LOW, LINEAR_HALL_CAL_FAULT_COS_LOW,
  LINEAR_HALL_CAL_FAULT_SIN_HIGH, LINEAR_HALL_CAL_FAULT_COS_HIGH,
  LINEAR_HALL_CAL_FAULT_SIN_AMP_LOW, LINEAR_HALL_CAL_FAULT_COS_AMP_LOW,
  LINEAR_HALL_CAL_FAULT_SIN_LOW | LINEAR_HALL_CAL_FAULT_SIN_HIGH |
  LINEAR_HALL_CAL_FAULT_COS_HIGH | LINEAR_HALL_CAL_FAULT_COS_AMP_LOW
 };
 for(unsigned i=0;i<sizeof(rejected_extrema)/sizeof(rejected_extrema[0]);++i) {
  const float *e=rejected_extrema[i];
  LinearHall_CalibrationBegin(&sweep);
  LinearHall_CalibrationSample(&sweep,e[0],e[1]);
  LinearHall_CalibrationSample(&sweep,e[2],e[3]);
  LinearHall_CalibrationSample(&sweep,(e[0]+e[2])*0.5f,(e[1]+e[3])*0.5f);
  assert(!LinearHall_CalibrationFinish(&sweep,&hall));
  assert(!hall.calibration_success && hall.calibration_faults==expected_faults[i]);
  assert(hall.sin_off_counts==810 && hall.cos_off_counts==3286 && hall.sin_amp==800 && hall.cos_amp==800);
 }
 /* 同一条 Flash 记录同时保存编码器和四个 Hall 系数；模拟重启及后续零位标定。 */
 reset();
 MotorWindingThermalConfig heat=s_motor_configuration.model.thermal;
 assert(Param_CreateAxis(6,-1,1,0.5f,&heat,&s_motor_configuration));
 s_motor_configuration.configuration_confirmed=true;
 LinearHall_Init(&s_linear_hall);
 LinearHall_CalibrationBegin(&s_hall_sweep);
 LinearHall_CalibrationSample(&s_hall_sweep,200,600);
 LinearHall_CalibrationSample(&s_hall_sweep,3400,3600);
 kind=MOTOR_CALIBRATION_ENCODER_LINEARIZATION;
 state=MOTOR_STATE_CALIBRATION; stopped=false; publication=true; released=false;
 MotorAxis_ProcessCalibrationPublication(); assert(!released && !writes);
 now_ms=201; MotorAxis_ProcessCalibrationPublication();
 assert(released && kind==MOTOR_CALIBRATION_JOINT_ELECTRICAL_ZERO && !publication && !writes);
 assert(s_hall_sweep.active && s_hall_sweep.sin_min==200 && s_hall_sweep.cos_max==3600);
 s_motor_configuration.device.encoder.calib_flag=ENC_CALIB_ALL;
 publication=true;
 MotorAxis_ProcessCalibrationPublication();
 assert(completions==1 && writes==1 && s_motor_state.fault_code==MOTOR_FAULT_NONE);
 assert(stored_record.hall_tag_status==0x48410180U && stored_record.encoder_calib_flag==ENC_CALIB_ALL);
 assert(!memcmp((uint8_t *)&stored_record+HALL_OFFSET,successful,sizeof successful));
 Param_Download(&stored_record,sizeof stored_record,&loaded);
 assert(loaded.configuration_source==MOTOR_CONFIGURATION_STORED && loaded.device.node_id==6);
 LinearHall_Init(&hall); Param_RestoreLinearHall(&stored_record,&hall);
 assert(hall.calibration_success && hall.calibration_faults==0 && hall.sin_amp==1600 && hall.cos_amp==1500);
 assert(hall.sin_off_counts==1800 && hall.cos_off_counts==2100);
 kind=MOTOR_CALIBRATION_ELECTRICAL_ZERO; publication=true;
 s_motor_configuration.device.encoder.calib_flag=ENC_CALIB_ALL;
 MotorAxis_ProcessCalibrationPublication();
 assert(writes==2 && stored_record.encoder_calib_flag==ENC_CALIB_ALL && stored_record.hall_tag_status==0x48410180U);
 kind=MOTOR_CALIBRATION_JOINT_ELECTRICAL_ZERO; publication=true;
 LinearHall_CalibrationBegin(&s_hall_sweep);
 MotorAxis_ProcessCalibrationPublication();
 assert(writes==3 && completions==2 && s_motor_state.fault_code==MOTOR_FAULT_LINEAR_HALL_CALIBRATION);
 assert(!memcmp((uint8_t *)&stored_record+HALL_OFFSET,no_samples,sizeof no_samples));
 LinearHall_Init(&hall); Param_RestoreLinearHall(&stored_record,&hall);
 assert(!hall.calibration_success && hall.calibration_faults==LINEAR_HALL_CAL_FAULT_NO_SAMPLES);
 assert(hall.sin_amp==1600 && hall.cos_off_counts==2100);
 s_motor_state.fault_code=MOTOR_FAULT_NONE; publication=true; kind=MOTOR_CALIBRATION_MECHANICAL_ZERO;
 write_result=BSP_IO_ERROR; MotorAxis_ProcessCalibrationPublication();
 assert(writes==4 && completions==2 && s_motor_state.fault_code==MOTOR_FAULT_CALIBRATION_STORAGE);
 s_motor_state.fault_code=MOTOR_FAULT_NONE; write_result=BSP_OK;
 publication=true; cancel_during_upload=true; MotorAxis_ProcessCalibrationPublication();
 assert(!publication && writes==4 && completions==2 && s_motor_state.fault_code==MOTOR_FAULT_NONE);
 stored_record.hall_tag_status=1U; LinearHall_Init(&hall); Param_RestoreLinearHall(&stored_record,&hall);
 assert(!hall.calibration_success && !hall.calibration_faults && hall.sin_amp==1829.5f);
 stored_record.hall_tag_status=0x48410180U; stored_record.hall_cos_amp=NAN;
 Param_RestoreLinearHall(&stored_record,&hall); assert(!hall.calibration_success && hall.sin_amp==1829.5f);
#endif
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
            "firmware/motor/feedback/linear_hall.c",
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
