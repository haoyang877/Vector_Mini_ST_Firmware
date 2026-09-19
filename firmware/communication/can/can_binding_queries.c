#include "can_binding_queries.h"

#include <math.h>
#include "angle_feedback.h"
#include "foc_algorithm.h"
#include "foc_sensing.h"
#include "foc_cogging_calibration.h"
#include "foc_friction_identification.h"
#include "can_transport.h"

/* 读路径实现：只读状态并暂存应答，保持与拆分前逐 token 一致。
 * 每个查询的取值来源、条件与回复参数 ID 都不得改动。 */

extern MotorControl_TypeDef MotorControl;
extern FOC_TypeDef FOC;
extern Encoder_TypeDef OnBoard_Encoder;

/**
 * @brief 返回编码器在线状态。
 * @return 在线返回 1，离线返回 0。
 * @note 只读编码器状态，不修改任何状态。
 */
static int CAN_GetEncoderState(void)
{
    return Encoder_IsOnline(&OnBoard_Encoder) ? 1 : 0;
}

void CanBinding_ApplyQuery(CAN_PARAM_ID param_id, float data, int data_int)
{
    switch (param_id)
    {
    /*setting parameters*/
    case CAN_GET_MODE:
        CAN_SendMessage_Update(CAN_GET_MODE, (float)MotorControl.ModeNow);
        break;

    case CAN_GET_CURRENT_SET:
        CAN_SendMessage_Update(CAN_GET_CURRENT_SET, MotorControl.iqRef);
        break;

    case CAN_GET_SPEED_SET:
        CAN_SendMessage_Update(CAN_GET_SPEED_SET, MotorControl.speedRef);
        break;

    case CAN_GET_POS_SET:
        CAN_SendMessage_Update(CAN_GET_POS_SET, MotorControl.posRef);
        break;

    /*user parameters*/
    case CAN_GET_NODE_ID:
        CAN_SendMessage_Update(CAN_GET_NODE_ID, (float)CANMsg.node_id);
        break;

    case CAN_GET_POLEPARIS:
        CAN_SendMessage_Update(CAN_GET_POLEPARIS, (float)MotorControl.motor_pole_pairs);
        break;

    case CAN_GET_ENCODER_STATE:
        CAN_SendMessage_Update(CAN_GET_ENCODER_STATE, (float)CAN_GetEncoderState());
        break;

    case CAN_GET_ENCODER_REVERSE:
        CAN_SendMessage_Update(CAN_GET_ENCODER_REVERSE,
                               (float)Encoder_GetReverse(&OnBoard_Encoder));
        break;

    case CAN_GET_CURRENT_CAL:
        CAN_SendMessage_Update(CAN_GET_CURRENT_CAL, MotorControl.calib_current);
        break;

    case CAN_GET_CURRENT_LIMIT:
        CAN_SendMessage_Update(CAN_GET_CURRENT_LIMIT, MotorControl.current_limit);
        break;

    case CAN_GET_SPEED_LIMIT:
        CAN_SendMessage_Update(CAN_GET_SPEED_LIMIT, MotorControl.speed_limit);
        break;

    case CAN_GET_SPEED_ACC:
        CAN_SendMessage_Update(CAN_GET_SPEED_ACC, MotorControl.speedAcc);
        break;

    case CAN_GET_SPEED_DEC:
        CAN_SendMessage_Update(CAN_GET_SPEED_DEC, MotorControl.speedDec);
        break;

    case CAN_GET_SPEED_KP:
        CAN_SendMessage_Update(CAN_GET_SPEED_KP, MotorControl.speed_Kp);
        break;

    case CAN_GET_SPEED_KI:
        CAN_SendMessage_Update(CAN_GET_SPEED_KI, MotorControl.speed_Ki);
        break;

    case CAN_GET_POS_ACC:
        CAN_SendMessage_Update(CAN_GET_POS_ACC, MotorControl.posAcc);
        break;

    case CAN_GET_POS_DEC:
        CAN_SendMessage_Update(CAN_GET_POS_DEC, MotorControl.posDec);
        break;

    case CAN_GET_POS_MAXSPEED:
        CAN_SendMessage_Update(CAN_GET_POS_MAXSPEED, MotorControl.pos_maxspeed);
        break;

    case CAN_GET_POS_KP:
        CAN_SendMessage_Update(CAN_GET_POS_KP, MotorControl.pos_Kp);
        break;

    case CAN_GET_POS_KD:
        CAN_SendMessage_Update(CAN_GET_POS_KD, MotorControl.pos_Kd);
        break;

    case CAN_GET_POS_KI:
        CAN_SendMessage_Update(CAN_GET_POS_KI, MotorControl.pos_Ki);
        break;

    case CAN_GET_POS_INTEGRAL_LIMIT:
        CAN_SendMessage_Update(CAN_GET_POS_INTEGRAL_LIMIT, MotorControl.pos_integral_limit);
        break;

    case CAN_GET_CASCADE_POS_KP:
        CAN_SendMessage_Update(CAN_GET_CASCADE_POS_KP, MotorControl.cascade_pos_Kp);
        break;

    case CAN_GET_CASCADE_POS_KD:
        CAN_SendMessage_Update(CAN_GET_CASCADE_POS_KD, MotorControl.cascade_pos_Kd);
        break;

    case CAN_GET_FRICTION_STATE:
        CAN_SendMessage_Update(CAN_GET_FRICTION_STATE, (float)FocFrictionIdentification_GetState());
        break;
    case CAN_GET_FRICTION_REASON:
        CAN_SendMessage_Update(CAN_GET_FRICTION_REASON,
                               (float)FocFrictionIdentification_GetReason());
        break;
    case CAN_GET_FRICTION_COULOMB_POS:
        CAN_SendMessage_Update(CAN_GET_FRICTION_COULOMB_POS,
                               FocFrictionIdentification_GetResult()->coulomb_pos_a);
        break;
    case CAN_GET_FRICTION_COULOMB_NEG:
        CAN_SendMessage_Update(CAN_GET_FRICTION_COULOMB_NEG,
                               FocFrictionIdentification_GetResult()->coulomb_neg_a);
        break;
    case CAN_GET_FRICTION_VISCOUS_POS:
        CAN_SendMessage_Update(CAN_GET_FRICTION_VISCOUS_POS,
                               FocFrictionIdentification_GetResult()->viscous_pos_a_per_rad_s);
        break;
    case CAN_GET_FRICTION_VISCOUS_NEG:
        CAN_SendMessage_Update(CAN_GET_FRICTION_VISCOUS_NEG,
                               FocFrictionIdentification_GetResult()->viscous_neg_a_per_rad_s);
        break;
    case CAN_GET_FRICTION_RMSE_POS:
        CAN_SendMessage_Update(CAN_GET_FRICTION_RMSE_POS,
                               FocFrictionIdentification_GetResult()->rmse_pos_a);
        break;
    case CAN_GET_FRICTION_RMSE_NEG:
        CAN_SendMessage_Update(CAN_GET_FRICTION_RMSE_NEG,
                               FocFrictionIdentification_GetResult()->rmse_neg_a);
        break;
    case CAN_GET_FRICTION_CANDIDATE_VALID:
        CAN_SendMessage_Update(CAN_GET_FRICTION_CANDIDATE_VALID,
                               FocFrictionIdentification_GetResult()->valid ? 1.0f : 0.0f);
        break;
    case CAN_GET_FRICTION_MODEL_VALID:
        CAN_SendMessage_Update(CAN_GET_FRICTION_MODEL_VALID,
                               MotorControl.friction_model_valid ? 1.0f : 0.0f);
        break;

    case CAN_GET_COGGING_STATE:
        CAN_SendMessage_Update(param_id, (float)FocCogging_GetState());
        break;
    case CAN_GET_COGGING_REASON:
        CAN_SendMessage_Update(param_id, (float)CoggingCalib.reason);
        break;
    case CAN_GET_COGGING_PROGRESS:
        CAN_SendMessage_Update(param_id,
                               100.0f * (float)CoggingCalib.points_done /
                                   (2.0f * (float)COGGING_MAP_POINTS));
        break;
    case CAN_GET_COGGING_POINT:
        if (isfinite(data) && data >= 0.0f && data < (float)COGGING_MAP_POINTS &&
            data == (float)data_int)
            CAN_SendMessage_Update(
                param_id, FocCogging_TableValid() ? (float)CoggingMap.iq_q15[data_int] : NAN);
        break;
    case CAN_GET_COGGING_FULL_SCALE:
        CAN_SendMessage_Update(param_id, FocCogging_TableValid() ? CoggingMap.full_scale_a : NAN);
        break;
    case CAN_GET_COGGING_VALID:
        CAN_SendMessage_Update(param_id, FocCogging_TableValid() ? 1.0f : 0.0f);
        break;
    case CAN_GET_COGGING:
        CAN_SendMessage_Update(CAN_GET_COGGING, (float)CoggingCompensation.enabled);
        break;
    case CAN_GET_TEMPERATURE_SOURCE:
        CAN_SendMessage_Update(param_id, 1.0f);
        break;
    case CAN_GET_TEMPERATURE_VALID:
        CAN_SendMessage_Update(param_id, (float)McuTemperature.valid);
        break;

    case CAN_GET_CAN_BR:
        CAN_SendMessage_Update(CAN_GET_CAN_BR, (float)CanTransport_Baudrate());
        break;

    case CAN_GET_CAN_HB:
        CAN_SendMessage_Update(CAN_GET_CAN_HB, (float)CANMsg.can_hb_set);
        break;

    /*state parameters*/
    case CAN_GET_VBUS:
        CAN_SendMessage_Update(CAN_GET_VBUS, FOC.Vbus_filt);
        break;

    case CAN_GET_IBUS:
        CAN_SendMessage_Update(CAN_GET_IBUS, FOC.Ibus_filt);
        break;

    case CAN_GET_IA:
        CAN_SendMessage_Update(CAN_GET_IA, FOC.Ia);
        break;

    case CAN_GET_IB:
        CAN_SendMessage_Update(CAN_GET_IB, FOC.Ib);
        break;

    case CAN_GET_IC:
        CAN_SendMessage_Update(CAN_GET_IC, FOC.Ic);
        break;

    case CAN_GET_ID:
        CAN_SendMessage_Update(CAN_GET_ID, FOC.Id);
        break;

    case CAN_GET_IQ:
        CAN_SendMessage_Update(CAN_GET_IQ, FOC.Iq);
        break;

    case CAN_GET_SPEED2_FILT:
        CAN_SendMessage_Update(CAN_GET_SPEED2_FILT, Encoder_GetMecVel(&OnBoard_Encoder));
        break;

    case CAN_GET_POS2_FILT:
        CAN_SendMessage_Update(CAN_GET_POS2_FILT, Encoder_GetMecPos(&OnBoard_Encoder));
        break;

    case CAN_GET_TEMP:
        CAN_SendMessage_Update(CAN_GET_TEMP, FOC.temp);
        break;

    case CAN_GET_RS:
        CAN_SendMessage_Update(CAN_GET_RS, MotorControl.motor_phase_resistance);
        break;

    case CAN_GET_LD:
        CAN_SendMessage_Update(CAN_GET_LD, MotorControl.motor_d_inductance);
        break;

    case CAN_GET_LQ:
        CAN_SendMessage_Update(CAN_GET_LQ, MotorControl.motor_q_inductance);
        break;

    case CAN_GET_FLUX:
        CAN_SendMessage_Update(CAN_GET_FLUX, MotorControl.motor_flux);
        break;

    case CAN_GET_ERROR:
        CAN_SendMessage_Update(CAN_GET_ERROR, (float)MotorControl.ErrorNow);
        break;

    default:
        break;
    }
}
