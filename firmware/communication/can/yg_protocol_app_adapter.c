#include "yg_protocol_app_adapter.h"

#include <math.h>

#include "angle_feedback.h"
#include "foc_algorithm.h"
#include "foc_run_state.h"
#include "foc_sensing.h"
#include "motor_state.h"
#include "motor_stop_service.h"
#include "yg_protocol_link.h"
#include "yg_protocol_motor_adapter.h"

extern MotorControl_TypeDef MotorControl;
extern FOC_TypeDef FOC;
extern Encoder_TypeDef OnBoard_Encoder;

static motor_stop_service_t stop_service;
static yg_protocol_motor_adapter_t motor_adapter;
static yg_protocol_motor_status_source_t status_source;
static uint32_t sample_counter;

static motor_stop_owner_result_t request_stop(void *context)
{
    (void)context;
    FocRunState_RequestProtocolStop();
    return MOTOR_STOP_OWNER_ACCEPTED;
}

static bool power_disabled(void *context)
{
    (void)context;
    return FocRunState_IsPowerDisabled();
}

static const motor_stop_owner_port_t stop_port = {
    request_stop,
    power_disabled,
};

static void sample_status(MotorStatus *sample)
{
    sample->fault = (uint16_t)MotorControl.ErrorNow;
    sample->mode = (uint16_t)MotorControl.ModeNow;
    sample->position_target = MotorControl.posRef;
    sample->position_feedback = Encoder_GetMecPos(&OnBoard_Encoder);
    sample->speed_target = MotorControl.speedRef;
    sample->speed_feedback =
        (MotorControl.ModeNow == Position_Mode || MotorControl.ModeNow == Position_Impedance_Mode)
            ? MotorControl.pos_vel_filtered
            : Encoder_GetMecVel(&OnBoard_Encoder);
    sample->current_reference = MotorControl.iqRef;
    sample->current_feedback = FOC.Iq;
    sample->temperature = FOC.temp;
    sample->bus_voltage = FOC.Vbus_filt;
    sample->bus_current = FOC.Ibus_filt;
    sample->position_planned = NAN;
    sample->speed_planned = NAN;
    if (MotorControl.ModeNow == Position_Mode)
    {
        sample->position_planned = MotorControl.posShadow;
        sample->speed_planned = MotorControl.pos_trajectory_speed_rad_s;
    }
    else if (MotorControl.ModeNow == Position_Impedance_Mode)
    {
        sample->position_planned = MotorControl.posShadow;
        sample->speed_planned = MotorControl.speedShadow;
    }
    else if (MotorControl.ModeNow == Speed_Mode)
    {
        sample->speed_planned = MotorControl.speedShadow;
    }
}

bool YgProtocolAppAdapter_Init(void)
{
    if (!MotorStopService_Init(&stop_service, &stop_port, NULL))
    {
        return false;
    }
    motor_adapter.stop_service = &stop_service;
    status_source = (yg_protocol_motor_status_source_t){0};
    status_source.boot_id = 0U;
    status_source.sample_available = false;
    if (!YgProtocolLink_BindMotorService(&(yg_protocol_motor_service_t){
            &motor_adapter,
            yg_protocol_motor_adapter_handle,
        }))
    {
        return false;
    }
    return YgProtocolLink_BindMotorStatusSource(&status_source);
}

void YgProtocolAppAdapter_RefreshStatus(void)
{
    sample_status(&status_source.sample);
    status_source.sample_counter = ++sample_counter;
    status_source.faults = (uint32_t)MotorControl.ErrorNow;
    status_source.state = MotorControl.ErrorNow != No_Error
                              ? 5U
                              : (MotorControl.ModeNow == Motor_Disable ? 1U : 3U);
    status_source.mode = (uint8_t)(MotorControl.ModeNow <= 8U ? MotorControl.ModeNow : 0U);
    status_source.last_applied_sequence = 0U;
    status_source.measurement_valid_bits = YG_PROTOCOL_MOTOR_MEASUREMENT_MASK;
    status_source.target_applied = true;
    status_source.sample_available = true;
}
