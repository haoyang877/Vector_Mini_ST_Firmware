#ifndef FOC_COGGING_CALIBRATION_H
#define FOC_COGGING_CALIBRATION_H
#include "data_type.h"
#include "encoder.h"
#include "foc_algorithm.h"
#include "foc_pid.h"
#include "cogging_calibration.h"
extern CoggingCalibration CoggingCalib;
typedef struct {
    float velocity_rad_s, iq_a, iq_reference_a, position_rad, target_rad;
    float vbus_v, temperature_c;
    uint32_t encoder_status, previous_fault;
} CoggingFaultSnapshot;
extern CoggingFaultSnapshot CoggingFault;
/* One frozen, coherent 2 kHz frame. Host writes 1 to request, ISR publishes 2,
 * host reads the frame then writes 0. Grid/state refer to BEFORE this tick's
 * state-machine update; accepted indicates whether this exact tick sampled. */
typedef struct {
    uint32_t tick, state, index, direction_pass, points_done, accepted;
    float target_rad, reference_rad, position_rad, error_rad;
    float velocity_rad_s, iq_average_a, iq_reference_a, vbus_v;
    float holding_velocity_rad_s;
} CoggingTelemetryFrame;
extern volatile uint32_t CoggingTelemetryState;
extern volatile CoggingTelemetryFrame CoggingTelemetry;
bool FocCogging_CanStart(const MotorControl_TypeDef *motor, const Encoder_TypeDef *encoder);
void FocCogging_Task(FOC_TypeDef *foc, MotorControl_TypeDef *motor,
    PI_Controller_TypeDef *pi, Encoder_TypeDef *encoder);
void FocCogging_Abort(void);
/* Foreground: finalize a completed table and request normal parameter save. */
void FocCogging_Service(void);
bool FocCogging_TableValid(void);
CoggingState FocCogging_GetState(void);
void FocCogging_SaveResult(bool success);
/* Runtime compensation is opt-in, RAM-only, encoder torque mode only. */
typedef struct {
    uint32_t request, enabled, rejected;
    float blend, table_a, applied_a, total_a;
} CoggingCompensationControl;
extern volatile CoggingCompensationControl CoggingCompensation;
bool FocCogging_SetCompensation(bool enabled);
float FocCogging_Apply(const MotorControl_TypeDef *motor, const Encoder_TypeDef *encoder);
/* Optional bounded torque bench guard. Lease is renewed by the host; expiry
 * stops PWM even if the host/probe disconnects. Never persisted. */
typedef struct {
    uint32_t enabled, lease_ticks, trip;
    float speed_limit_rad_s, current_limit_a;
} CoggingTorqueGuard;
extern volatile CoggingTorqueGuard TorqueGuard;
bool FocCogging_TorqueGuard(MotorControl_TypeDef *motor, const Encoder_TypeDef *encoder);
typedef struct {
    uint32_t tick;
    float position_rad, velocity_rad_s, command_a, compensation_a, total_a;
    float feedback_a, vbus_v, blend;
} CoggingTorqueFrame;
extern volatile uint32_t TorqueTelemetryState;
extern volatile CoggingTorqueFrame TorqueTelemetry;
void FocCogging_TorqueObserve(const FOC_TypeDef *foc, const MotorControl_TypeDef *motor,
    const Encoder_TypeDef *encoder);
#endif
