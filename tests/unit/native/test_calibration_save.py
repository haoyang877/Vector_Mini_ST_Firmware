"""验证联合标定保存、取消/故障隔离、真实 schema14 往返及 Flash 失败传播。"""

import json

from _bsp_test_support import arguments, run_c
from run_position_servo_tests import ROOT, function_source


def fixture() -> str:
    """使用真实发布、记录编解码和 Hall 算法，仅替换生命周期与 Flash 边界。"""
    axis = (ROOT / "firmware/app/motor_calibration.c").read_text(encoding="utf-8")
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
#include "motor_state_machine.h"
#include "motor_sensing.h"
#include "encoder_calibration.h"
#include "parameter_record.h"
#include "parameter_values.h"
#include "bsp_flash.h"
#include "bsp_system.h"

static MotorConfiguration s_motor_configuration;
static MotorControlState s_motor_state;
static MotorAxisRecord s_axis_record;
static InterfaceParam stored_record;
static MotorAxisRecord stored_axis;
static OutputEncoder s_output_encoder;
static MotorCalibration calibration;
static MotorStateMachine machine;
static MotorState state;
static bool stopped, released, inject_fault, inject_start;
static bool cancel_during_upload, replace_during_upload, change_kind_during_upload;
#define session_epoch (machine.epoch)
static uint32_t critical, control_depth;
static bool in_adc;
#if BSP_ENABLED_LINEAR_HALL
"""
        + f"\n#define HALL_OFFSET {golden['offset']}U\n{vectors}\n"
        + r"""
static LinearHall s_linear_hall;


#endif
static MotorCalibrationType kind;
static bool encoder_publish_ok;
static unsigned saved_publications, publication_failures, lut_installs;
static unsigned writes, uploads;
static BspResult write_result;
bool MotorStateMachine_IsStopped(const MotorStateMachine *instance) {
    (void)instance; return stopped; }
bool EncoderCalibration_IsIdle(const EncoderCalibrationSession *instance) {
    (void)instance; return released; }
uint32_t BspSystem_EnterCritical(void) { return critical++; }
void BspSystem_ExitCritical(uint32_t previous) { assert(critical==previous+1); critical=previous; }
uint32_t BspSystem_EnterControlCritical(void) { return control_depth++; }
void BspSystem_ExitControlCritical(uint32_t previous) { assert(control_depth==previous+1); control_depth=previous; }
bool MotorStateMachine_ReadPendingCalibrationPublication(const MotorStateMachine *instance, uint32_t *epoch) {
    (void)instance;
 if (machine.calibration_handoff!=MOTOR_CALIBRATION_AWAITING_PUBLICATION || state!=MOTOR_STATE_CALIBRATION || !stopped || s_motor_state.fault_code!=MOTOR_FAULT_NONE) return false;
 *epoch=session_epoch; return true;
}
MotorCalibrationType MotorStateMachine_GetCalibrationType(const MotorStateMachine *instance) {
    (void)instance; return kind; }
static EncoderState s_rotor_encoder;
static uint32_t now_ms;
uint32_t BspSystem_TimeMs(void) { return now_ms; }
float Encoder_GetMechVelContinuous(const EncoderState *encoder) { return encoder->mechanical_velocity_continuous_rad_s; }
void MotorStateMachine_ContinueElectricalZero(MotorStateMachine *instance) {
    (void)instance;
 assert(released && in_adc && !critical && machine.calibration_handoff==MOTOR_CALIBRATION_AWAITING_PUBLICATION);
 kind=MOTOR_CALIBRATION_COMBINED_ELECTRICAL_ZERO; machine.calibration_handoff=MOTOR_CALIBRATION_NO_HANDOFF;
}
bool EncoderCalibration_PublishLut(EncoderCalibrationSession *instance, EncoderCalibration *cal, EncoderState *encoder) {
    (void)instance;
 assert(in_adc && !critical && machine.calibration_handoff==MOTOR_CALIBRATION_AWAITING_PUBLICATION && encoder==&s_rotor_encoder);
 ++lut_installs;
 if(encoder_publish_ok) { cal->calib_flag=ENC_CALIB_LINEARIZED; released=true; }
 return encoder_publish_ok;
}
void MotorStateMachine_CompleteCalibrationPublication(MotorStateMachine *instance, uint32_t epoch, bool ok) {
    (void)instance;
 assert(epoch==session_epoch && (critical || control_depth || in_adc) && machine.calibration_handoff==MOTOR_CALIBRATION_AWAITING_PUBLICATION);
 machine.calibration_handoff=MOTOR_CALIBRATION_NO_HANDOFF; if(ok) { ++saved_publications; instance->calibration_saved_epoch=epoch; } else ++publication_failures;
}
void MotorFault_Report(MotorControlState *motor,MotorFaultCode fault) { motor->fault_code=fault; }
BspResult BspFlash_Write(const void *data, size_t length) {
 assert(critical && released && ((state == MOTOR_STATE_IDLE && stopped) || machine.calibration_handoff==MOTOR_CALIBRATION_AWAITING_PUBLICATION));
 assert(s_motor_state.fault_code == MOTOR_FAULT_NONE);
 assert(data == &s_axis_record && length == sizeof(MotorAxisRecord));
 assert(s_axis_record.motor.record_crc32 == Param_RecordCrc(&s_axis_record.motor));
 ++writes;
 if (write_result == BSP_OK) {
  memcpy(&stored_record, data, sizeof stored_record);
  memcpy(&stored_axis, data, sizeof stored_axis);
 }
 return write_result;
}
void observed_upload(InterfaceParam *record, const MotorConfiguration *configuration) {
 ++uploads;
 Param_Upload(record, configuration);
 if (cancel_during_upload) { machine.calibration_handoff=MOTOR_CALIBRATION_NO_HANDOFF; }
 if (inject_fault) { s_motor_state.fault_code = MOTOR_FAULT_ENCODER; }
 if (inject_start) { stopped = false; }
 if (replace_during_upload) { ++session_epoch; }
 if (change_kind_during_upload) { kind=MOTOR_CALIBRATION_ELECTRICAL_ZERO; }
}
"""
        + "\n#define Param_Upload observed_upload\n"
        + "\n#if BSP_ENABLED_LINEAR_HALL\n"
        + function_source(axis, "request_lut_install")
        + "\n#endif\n"
        + function_source(axis, "MotorCalibration_PublishLutFast")
        + "\n"
        + function_source(axis, "MotorCalibration_ProcessPublication")
        + r"""
#undef Param_Upload
static void reset(void) {
 calibration = (MotorCalibration){.write_record=BspFlash_Write,.state_machine=&machine, .drive={.configuration=&s_motor_configuration,.encoder=&s_rotor_encoder,.motor=&s_motor_state}, .record=&s_axis_record, .output=&s_output_encoder};
#if BSP_ENABLED_LINEAR_HALL
 calibration.hall=&s_linear_hall;
#endif

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
 OutputEncoder_Init(&s_output_encoder, 1.0f);
 state = MOTOR_STATE_CALIBRATION; stopped = released = true;
 inject_fault = inject_start = false;
 write_result = BSP_OK;
 writes = uploads = 0U;
 machine.calibration_handoff=MOTOR_CALIBRATION_NO_HANDOFF;
 cancel_during_upload=replace_during_upload=change_kind_during_upload=false;
 session_epoch=33; encoder_publish_ok=true; saved_publications=publication_failures=lut_installs=0;
 kind=MOTOR_CALIBRATION_NONE; now_ms=0; s_rotor_encoder=(EncoderState){0};
 assert(ParameterValue_ConfigurationValid(&s_motor_configuration));
}
int main(void) {
 reset();
 for (unsigned flags = 1U; flags <= 7U; ++flags) {
  s_motor_configuration.device.encoder.calib_flag = flags;
  Param_Upload(&stored_record, &s_motor_configuration);
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
 /* 未定义位仍拒绝，即使整条CRC正确。 */
 stored_record.encoder_calib_flag = 8U;
 stored_record.record_crc32 = Param_RecordCrc(&stored_record);
 MotorConfiguration loaded;
 Param_Download(&stored_record, sizeof(stored_record), &loaded);
 assert(loaded.configuration_source == MOTOR_CONFIGURATION_CORRUPT);

 /* 两板普通轴均须等 ADC 安装 LUT，重复邮箱不能再次安装或提前保存。 */
 reset();
 kind=MOTOR_CALIBRATION_ENCODER_LINEARIZATION;
 machine.calibration_handoff=MOTOR_CALIBRATION_AWAITING_PUBLICATION; released=false;
 MotorCalibration_ProcessPublication(&calibration);
 assert(calibration.lut_request_epoch==session_epoch && !writes && !lut_installs);
 in_adc=true; MotorCalibration_PublishLutFast(&calibration); in_adc=false;
 assert(lut_installs==1 && !writes && calibration.lut_published_epoch==session_epoch);
 calibration.lut_request_epoch=session_epoch;
 in_adc=true; MotorCalibration_PublishLutFast(&calibration); in_adc=false;
 assert(lut_installs==1 && !calibration.lut_request_epoch && !publication_failures);
 MotorCalibration_ProcessPublication(&calibration);
 assert(writes==1 && saved_publications==1);
 kind=MOTOR_CALIBRATION_MECHANICAL_ZERO;
 machine.calibration_handoff=MOTOR_CALIBRATION_AWAITING_PUBLICATION;
 s_rotor_encoder.linearized_q15=1000;
 OutputEncoder_Locate(&s_output_encoder,1000,0);
 MotorCalibration_ProcessPublication(&calibration);
 assert(writes==2 && saved_publications==2 && s_output_encoder.position_rad==0);
 assert(s_output_encoder.params.status & OUTPUT_ENCODER_ZERO_SET);
 kind=MOTOR_CALIBRATION_ELECTRICAL_ZERO;
 machine.calibration_handoff=MOTOR_CALIBRATION_AWAITING_PUBLICATION;
 MotorCalibration_ProcessPublication(&calibration);
 assert(writes==3 && saved_publications==3);
 reset();

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
  assert(fabsf(remainderf(hall.val.angle_rad+theta,6.2831853f)) < 1e-6f);
  assert(hall.val.angle_rad>=0 && hall.val.angle_rad<6.2831853f);
 }
 LinearHallCalibration sweep;
 LinearHall_CalibrationBegin(&sweep);
 assert(!LinearHall_CalibrationFinish(&sweep,&hall));
 assert(!hall.calibration_success && hall.calibration_faults==LINEAR_HALL_CAL_FAULT_NO_SAMPLES);
 LinearHall_CalibrationBegin(&sweep);
 for (unsigned i=0;i<=360;++i) {
  float theta=-6.2831855f*(float)i/360.0f;
  LinearHall_CalibrationSample(&sweep,1700+800*sinf(theta),2400+1300*cosf(theta));
 }
 assert(LinearHall_CalibrationFinish(&sweep,&hall));
 assert(hall.calibration_success && hall.calibration_faults==LINEAR_HALL_CAL_FAULT_NONE);
 assert(hall.sin_off_counts==1700 && hall.sin_amp==800);
 assert(hall.cos_off_counts==2400 && hall.cos_amp==1300);
 LinearHall_Update(&hall,2100,1750);
 assert(hall.val.sin_cal==0.5f && hall.val.cos_cal==-0.5f);
 assert(fabsf(hall.val.angle_rad-3.9269908f)<1e-6f);
 /* 阈值边界恰好合格。 */
 LinearHall_CalibrationBegin(&sweep);
 LinearHall_CalibrationSample(&sweep,10,2486);
 LinearHall_CalibrationSample(&sweep,1610,4086);
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
 /* 正反边界和非有限速度都不能判停，6 s 超时不写入。 */
 MotorWindingThermalConfig heat=s_motor_configuration.model.thermal;
 const float moving_speeds[] = {1.0f,-1.0f,NAN,INFINITY};
 for(unsigned i=0;i<sizeof(moving_speeds)/sizeof(moving_speeds[0]);++i) {
  reset();
  assert(Param_CreateAxis(6,-1,1,0.5f,&heat,&s_motor_configuration));
  kind=MOTOR_CALIBRATION_ENCODER_LINEARIZATION;
  machine.calibration_handoff=MOTOR_CALIBRATION_AWAITING_PUBLICATION; released=false;
  s_rotor_encoder.mechanical_velocity_continuous_rad_s=moving_speeds[i];
  MotorCalibration_ProcessPublication(&calibration);
  now_ms=5999; MotorCalibration_ProcessPublication(&calibration);
  assert(machine.calibration_handoff==MOTOR_CALIBRATION_AWAITING_PUBLICATION);
  assert(!calibration.lut_request_epoch && !writes);
  now_ms=6000; MotorCalibration_ProcessPublication(&calibration);
  assert(machine.calibration_handoff==MOTOR_CALIBRATION_NO_HANDOFF);
  assert(publication_failures==1 && !writes);
 }
 /* 任一超速拍重新计时；200 ms 恰好允许请求。 */
 reset(); assert(Param_CreateAxis(6,-1,1,0.5f,&heat,&s_motor_configuration));
 kind=MOTOR_CALIBRATION_ENCODER_LINEARIZATION;
 machine.calibration_handoff=MOTOR_CALIBRATION_AWAITING_PUBLICATION; released=false;
 MotorCalibration_ProcessPublication(&calibration);
 now_ms=199; s_rotor_encoder.mechanical_velocity_continuous_rad_s=1.0f;
 MotorCalibration_ProcessPublication(&calibration);
 s_rotor_encoder.mechanical_velocity_continuous_rad_s=0;
 now_ms=398; MotorCalibration_ProcessPublication(&calibration);
 assert(!calibration.lut_request_epoch);
 now_ms=399; MotorCalibration_ProcessPublication(&calibration);
 assert(calibration.lut_request_epoch==session_epoch && !writes);
 /* 无符号差值跨毫秒计数回绕仍维持同一稳定窗口。 */
 reset(); assert(Param_CreateAxis(6,-1,1,0.5f,&heat,&s_motor_configuration));
 kind=MOTOR_CALIBRATION_ENCODER_LINEARIZATION;
 machine.calibration_handoff=MOTOR_CALIBRATION_AWAITING_PUBLICATION; released=false;
 now_ms=UINT32_MAX-100U; MotorCalibration_ProcessPublication(&calibration);
 now_ms+=199U; MotorCalibration_ProcessPublication(&calibration);
 assert(!calibration.lut_request_epoch);
 now_ms+=1U; MotorCalibration_ProcessPublication(&calibration);
 assert(calibration.lut_request_epoch==session_epoch && !writes && !publication_failures);

 /* 同一条 Flash 记录同时保存编码器和四个 Hall 系数；模拟重启及后续零位标定。 */
 reset();
 assert(Param_CreateAxis(6,-1,1,0.5f,&heat,&s_motor_configuration));
 s_motor_configuration.configuration_confirmed=true;
 LinearHall_Init(&s_linear_hall);
 OutputEncoder_Init(&s_output_encoder, 8.0f);
 LinearHall_CalibrationBegin(&calibration.hall_sweep);
 LinearHall_CalibrationSample(&calibration.hall_sweep,200,600);
 LinearHall_CalibrationSample(&calibration.hall_sweep,3400,3600);
 kind=MOTOR_CALIBRATION_ENCODER_LINEARIZATION;
 state=MOTOR_STATE_CALIBRATION; stopped=true; machine.calibration_handoff=MOTOR_CALIBRATION_AWAITING_PUBLICATION; released=false;
 MotorCalibration_ProcessPublication(&calibration); assert(!released && !writes);
 now_ms=201; MotorCalibration_ProcessPublication(&calibration);
 assert(!released && !writes && calibration.lut_request_epoch == session_epoch);
 ++session_epoch;
 in_adc=true; MotorCalibration_PublishLutFast(&calibration); in_adc=false;
 assert(!released && !writes && !calibration.lut_request_epoch);
 MotorCalibration_ProcessPublication(&calibration);
 now_ms+=201; MotorCalibration_ProcessPublication(&calibration);
 assert(calibration.lut_request_epoch == session_epoch);
 in_adc=true; MotorCalibration_PublishLutFast(&calibration); in_adc=false;
 assert(released && kind==MOTOR_CALIBRATION_COMBINED_ELECTRICAL_ZERO && machine.calibration_handoff!=MOTOR_CALIBRATION_AWAITING_PUBLICATION && !writes);
 assert(calibration.hall_sweep.sin_min==200 && calibration.hall_sweep.cos_max==3600);
 s_motor_configuration.device.encoder.calib_flag=ENC_CALIB_ALL;
 memset(s_motor_configuration.device.encoder.linearization_lut_q15,0,sizeof s_motor_configuration.device.encoder.linearization_lut_q15);
 calibration.output_calibration.count=OUTPUT_ENCODER_CAL_SAMPLES;
 for(unsigned i=0;i<OUTPUT_ENCODER_CAL_SAMPLES;++i) {
  int32_t counts=(int32_t)(i*8.0f*65536.0f*1.01f/255.0f);
  float h=(float)counts*(6.283185307f/(65536.0f*8.0f));
  calibration.output_calibration.samples[i]=(OutputEncoderCalibSample){counts,
   (uint16_t)(1800-1600*sinf(h)),(uint16_t)(2100+1500*cosf(h))};
 }
 machine.calibration_handoff=MOTOR_CALIBRATION_AWAITING_PUBLICATION;
 MotorCalibration_ProcessPublication(&calibration);
 assert(saved_publications==1 && writes==1 && s_motor_state.fault_code==MOTOR_FAULT_NONE);
 assert(stored_record.hall_tag_status==0x48410180U && stored_record.encoder_calib_flag==ENC_CALIB_ALL);
 assert(stored_axis.output.hall_lut_status==LINEAR_HALL_LUT_OK);
 assert(!memcmp((uint8_t *)&stored_record+HALL_OFFSET,successful,sizeof successful));
 Param_Download(&stored_record,sizeof stored_record,&loaded);
 assert(loaded.configuration_source==MOTOR_CONFIGURATION_STORED && loaded.device.node_id==6);
 LinearHall_Init(&hall); Param_RestoreLinearHall(&stored_record,&hall);
 assert(hall.calibration_success && hall.calibration_faults==0 && hall.sin_amp==1600 && hall.cos_amp==1500);
 assert(hall.sin_off_counts==1800 && hall.cos_off_counts==2100);
 kind=MOTOR_CALIBRATION_ELECTRICAL_ZERO; machine.calibration_handoff=MOTOR_CALIBRATION_AWAITING_PUBLICATION;
 s_motor_configuration.device.encoder.calib_flag=ENC_CALIB_ALL;
 MotorCalibration_ProcessPublication(&calibration);
 assert(writes==2 && stored_record.encoder_calib_flag==ENC_CALIB_ALL && stored_record.hall_tag_status==0x48410180U);
 kind=MOTOR_CALIBRATION_COMBINED_ELECTRICAL_ZERO; machine.calibration_handoff=MOTOR_CALIBRATION_AWAITING_PUBLICATION;
 LinearHall_CalibrationBegin(&calibration.hall_sweep);
 MotorCalibration_ProcessPublication(&calibration);
 assert(writes==3 && saved_publications==3 && machine.calibration_saved_epoch==session_epoch && s_motor_state.fault_code==MOTOR_FAULT_LINEAR_HALL_CALIBRATION);
 assert(!memcmp((uint8_t *)&stored_record+HALL_OFFSET,no_samples,sizeof no_samples));
 LinearHall_Init(&hall); Param_RestoreLinearHall(&stored_record,&hall);
 assert(!hall.calibration_success && hall.calibration_faults==LINEAR_HALL_CAL_FAULT_NO_SAMPLES);
 assert(hall.sin_amp==1600 && hall.cos_off_counts==2100);
 s_motor_state.fault_code=MOTOR_FAULT_NONE; machine.calibration_handoff=MOTOR_CALIBRATION_AWAITING_PUBLICATION; kind=MOTOR_CALIBRATION_MECHANICAL_ZERO;
 write_result=BSP_IO_ERROR; MotorCalibration_ProcessPublication(&calibration);
 assert(writes==4 && saved_publications==3 && s_motor_state.fault_code==MOTOR_FAULT_CALIBRATION_STORAGE);
 s_motor_state.fault_code=MOTOR_FAULT_NONE; write_result=BSP_OK;
 machine.calibration_handoff=MOTOR_CALIBRATION_AWAITING_PUBLICATION; cancel_during_upload=true; MotorCalibration_ProcessPublication(&calibration);
 assert(machine.calibration_handoff!=MOTOR_CALIBRATION_AWAITING_PUBLICATION && writes==4 && saved_publications==3 && s_motor_state.fault_code==MOTOR_FAULT_NONE);
 stored_record.hall_tag_status=1U; LinearHall_Init(&hall); Param_RestoreLinearHall(&stored_record,&hall);
 assert(!hall.calibration_success && !hall.calibration_faults && hall.sin_amp==1829.5f);
 stored_record.hall_tag_status=0x48410180U; stored_record.hall_cos_amp=NAN;
 Param_RestoreLinearHall(&stored_record,&hall); assert(!hall.calibration_success && hall.sin_amp==1829.5f);

 /* 编码期间允许中断；取消、新会话、故障和启相均不得写入旧结果。 */
 for(unsigned mode=0;mode<5;++mode) {
  machine.calibration_handoff=MOTOR_CALIBRATION_AWAITING_PUBLICATION; session_epoch=33; stopped=true; s_motor_state.fault_code=MOTOR_FAULT_NONE;
  cancel_during_upload=mode==0; replace_during_upload=mode==1;
  kind=MOTOR_CALIBRATION_MECHANICAL_ZERO; change_kind_during_upload=mode==4;
  inject_fault=mode==2; inject_start=mode==3;
  unsigned before=writes, completed=saved_publications;
  MotorCalibration_ProcessPublication(&calibration);
  assert(writes==before && saved_publications==completed);
 }
 cancel_during_upload=replace_during_upload=change_kind_during_upload=inject_fault=inject_start=false;
 const BspResult failures[] = {BSP_NOT_SUPPORTED, BSP_NOT_READY, BSP_INVALID_ARGUMENT, BSP_IO_ERROR, BSP_REJECTED};
 for(unsigned i=0;i<sizeof(failures)/sizeof(failures[0]);++i) {
  machine.calibration_handoff=MOTOR_CALIBRATION_AWAITING_PUBLICATION; stopped=true; s_motor_state.fault_code=MOTOR_FAULT_NONE; write_result=failures[i];
  unsigned before=writes, completed=saved_publications;
  MotorCalibration_ProcessPublication(&calibration);
  assert(writes==before+1 && saved_publications==completed);
  assert(s_motor_state.fault_code==MOTOR_FAULT_CALIBRATION_STORAGE);
 }
#endif
 puts("PASS calibration publication: cancellation, stale session, schema14, Hall thresholds and Flash errors");
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
            "firmware/motor/feedback/output_encoder.c",
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


if __name__ == "__main__":
    main()
