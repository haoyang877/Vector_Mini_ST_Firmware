#include "can_binding_commands.h"

#include <math.h>
#include "utils.h"
#include "angle_feedback.h"
#include "current_sense_profile.h"
#include "foc_algorithm.h"
#include "foc_errhandle.h"
#include "foc_param.h"
#include "foc_param_profile.h"
#include "foc_cogging_calibration.h"
#include "foc_friction_identification.h"
#include "position_cascade.h"
#include "can_transport.h"

/* 写路径实现：只做状态更新与限幅，保持与拆分前逐 token 一致。
 * 每个命令的接受条件、限幅与副作用顺序都不得改动。 */

extern MotorControl_TypeDef MotorControl;
extern Encoder_TypeDef OnBoard_Encoder;

/**
 * @brief 设置编码器状态的遗留槽位：当前为空实现，仅保留线槽位。
 * @param data 线路传入的编码器状态值，本实现忽略。
 * @note 保留 0x0C 槽位以避免与既有主机兼容性说明冲突；删除需单独评审。
 */
static void CAN_SetEncoderState(int data)
{
    (void)data;
}

void CanBinding_ApplyCommand(CAN_PARAM_ID param_id, float data, int data_int)
{
    switch (param_id)
    {
    /*setting parameters*/
    case CAN_SET_MODE:
        if (data_int >= 0 && data_int < (int)MODE_NUM)
            ModeSwitch_Handle((ModeNow_TypeDef)data_int);
        break;

    case CAN_SET_CURRENT:
        ModeSwitch_Handle(Current_Mode);
        if (fast_abs(data) <= MotorControl.current_limit)
            MotorControl.iqRef = data;
        break;

    case CAN_SET_SPEED:
        if (MotorControl.ModeNow != Sensorless_Speed_Mode)
            ModeSwitch_Handle(Speed_Mode);
        if (fast_abs(data) <= MotorControl.speed_limit)
            MotorControl.speedRef = data;
        break;

    case CAN_SET_POS:
    {
        float position_ref = data;
        if (isfinite(position_ref) &&
            (MotorControl.ModeNow == Position_Mode ||
             MotorControl.ModeNow == Position_Impedance_Mode || ModeSwitch_Handle(Position_Mode)))
        {
            MotorControl.posRef = position_ref;
        }
    }
    break;

    /*user parameters*/
    case CAN_SET_NODE_ID:
        if (data_int >= 0 && data_int <= 7)
        {
            CANMsg.node_id = data;
            Param_SetSpeedLimit(
                fminf(MotorControl.speed_limit, Param_SpeedLimitRadS(CANMsg.node_id)));
        }
        break;

    case CAN_SET_POLEPARIS:
        if (data_int >= 2 && data_int <= 30)
            MotorControl.motor_pole_pairs = data;
        break;

    case CAN_SET_ENCODER_STATE:
        if (MotorControl.ModeNow != Current_Mode && MotorControl.ModeNow != Speed_Mode &&
            MotorControl.ModeNow != Speed_Mode)
        {
            CAN_SetEncoderState(data_int);
        }
        break;

    case CAN_SET_ENCODER_REVERSE:
        if (MotorControl.ModeNow == Motor_Disable && (data_int == 0 || data_int == 1))
        {
            if (Encoder_GetReverse(&OnBoard_Encoder) != (uint8_t)data_int)
                MotorControl.friction_model_valid = false;
            Encoder_SetReverse(&OnBoard_Encoder, data_int != 0);
        }
        break;

    case CAN_SET_CURRENT_CAL:
        if (data >= 0.0f && data <= CURRENT_SENSE_PROFILE_CALIB_LIMIT_MAX_A)
            MotorControl.calib_current = data;
        break;

    case CAN_SET_CURRENT_LIMIT:
        if (data >= 0.0f && data <= CURRENT_SENSE_PROFILE_COMMAND_LIMIT_MAX_A)
        {
            MotorControl.current_limit = data;
            MotorControl.iqRef = constrain(MotorControl.iqRef, -data, data);
        }
        break;

    case CAN_SET_SPEED_LIMIT:
        Param_SetSpeedLimit(data);
        break;

    case CAN_SET_SPEED_ACC:
        if (data >= 0.0f && data <= 1000.0f * _2PI)
            MotorControl.speedAcc = data;
        break;

    case CAN_SET_SPEED_DEC:
        if (data >= 0.0f && data <= 1000.0f * _2PI)
            MotorControl.speedDec = data;
        break;

    case CAN_SET_SPEED_KP:
        if (data >= 0.01f && data <= 2.0f)
            MotorControl.speed_Kp = data;
        break;

    case CAN_SET_SPEED_KI:
        if (data >= 0.0f && data <= 2.0f)
            MotorControl.speed_Ki = data;
        break;

    case CAN_SET_POS_ACC:
        if (data > 0.0f && data <= 200.0f * _2PI)
            MotorControl.posAcc = data;
        break;

    case CAN_SET_POS_DEC:
        if (data > 0.0f && data <= 200.0f * _2PI)
            MotorControl.posDec = data;
        break;

    case CAN_SET_POS_MAXSPEED:
        if (data > 0.0f && data <= POSITION_IMPEDANCE_MAX_SPEED_RPS * _2PI &&
            data <= MotorControl.speed_limit)
        {
            float position_maxspeed = data;
            if (MotorControl.pos_maxspeed != position_maxspeed)
                MotorControl.pos_maxspeed = position_maxspeed;
        }
        break;

    case CAN_SET_POS_KP:
        if (data >= 0.0f && data <= POSITION_IMPEDANCE_KP_MAX_A_PER_RAD)
            MotorControl.pos_Kp = data;
        break;

    case CAN_SET_POS_KD:
        if (data >= 0.0f && data <= POSITION_IMPEDANCE_KD_MAX_A_PER_RAD_S)
            MotorControl.pos_Kd = data;
        break;

    case CAN_SET_POS_KI:
        if (data >= 0.0f && data <= POSITION_IMPEDANCE_KI_MAX_A_PER_RAD_S)
            MotorControl.pos_Ki = data;
        break;

    case CAN_SET_POS_INTEGRAL_LIMIT:
        if (data >= 0.0f && data <= CURRENT_SENSE_PROFILE_COMMAND_LIMIT_MAX_A)
            MotorControl.pos_integral_limit = data;
        break;

    case CAN_SET_CASCADE_POS_KP:
        if (data >= 0.0f && data <= CASCADE_POSITION_KP_MAX_PER_S)
            MotorControl.cascade_pos_Kp = data;
        break;

    case CAN_SET_CASCADE_POS_KD:
        if (data >= 0.0f && data <= CASCADE_POSITION_KD_MAX)
            MotorControl.cascade_pos_Kd = data;
        break;

    case CAN_APPLY_FRICTION_MODEL:
        if (data_int == 1)
            (void)FocFrictionIdentification_ApplyCandidate(&MotorControl);
        break;

    case CAN_SET_COGGING:
        if (data == 0.0f || data == 1.0f)
            CoggingCompensation.request = (uint32_t)data; /* CRC validation in foreground */
        break;

    case CAN_SET_CAN_BR:
        if (data_int == 100 || data_int == 125 || data_int == 200 || data_int == 250 ||
            data_int == 500 || data_int == 1000 || data_int == 2000 || data_int == 2500 ||
            data_int == 5000)
            CanTransport_SetBaudrate((uint32_t)data_int);
        break;

    case CAN_SET_CAN_HB:
        if ((data_int >= 500 && data <= 1000) || data_int == 0)
            CANMsg.can_hb_set = data_int;
        break;

    default:
        break;
    }
}
