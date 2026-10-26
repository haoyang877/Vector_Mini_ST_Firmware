#include "boot_handoff.h"
#include "motor_hw.h"
#include <string.h>
#include "boot_hw.h"
#include "time_hw.h"
#include "motor_load_control.h"
#include "interface_can.h"
#include "can_deployment.h"

#include <limits.h>
#include <math.h>
#include "utils.h"
#include "foc_algorithm.h"
#include "foc_param.h"
#include "foc_param_profile.h"
#include "foc_errhandle.h"
#include "encoder.h"
#include "hw_conf.h"
#include "comm_hw.h"
#include "can_motor_status.h"
#include "can_parameter_format.h"
#include "foc_friction_identification.h"
#include "firmware_version.h"
#include "foc_task.h"
#include "foc_run.h"
#include "motor_thermal_control.h"
#include "position_cascade_config.h"

CANMsg_TypeDef CANMsg;
volatile CAN_Diagnostics_TypeDef CAN_Diagnostics;

/* CAN_REPLY_RUNTIME_BEGIN */
/** @brief 单槽回复的发布状态，由 RX 发布、主循环处理并释放。 */
typedef enum {
    CAN_REPLY_FREE,
    CAN_REPLY_UID,
    CAN_REPLY_CRC,
    CAN_REPLY_READY
} CanReplyState;

/** @brief 单槽回复；占用期间 RX 不修改载荷、ID 或帧格式。
 * @note RX 填写全部字段后发布状态，主循环成功入队后才释放。
 * UID/CRC 计算及 HAL 重试期间不跟随新查询改变帧格式。
 */
static volatile struct {
    CanReplyState state; /**< FREE 时其余字段无效。 */
    uint16_t identifier; /**< 请求时冻结的标准 ID，不随节点改号而变化。 */
    uint8_t length;      /**< 载荷字节数，范围 2～16。 */
    bool fd;            /**< 请求时冻结的经典/FD 格式。 */
    bool brs;           /**< 请求时冻结的 FD 速率切换位，经典帧为 false。 */
    uint8_t data[16];    /**< 本槽拥有的载荷副本。 */
} can_reply;

/** @brief 不重入 RX 分派中的请求格式；分派结束恢复编译部署的默认格式。 */
static bool can_request_fd = !VECTOR_CAN_CLASSIC_ONLY;
/** @brief 不重入 RX 分派中的速率切换位；仅 FD 请求有效。 */
static bool can_request_brs = !VECTOR_CAN_CLASSIC_ONLY;

/**
 * @brief 对当前上下文独占的诊断项饱和计数，避免长期运行回绕。
 * @param[in,out] counter 非 NULL 的诊断项；调用者保证单写者。
 */
static void CAN_Count(volatile uint32_t *counter)
{
    if (*counter != UINT32_MAX) {
        ++*counter;
    }
}

/**
 * @brief 将载荷、标识符及当前请求格式发布到空回复槽。
 * @param[in] identifier 请求时确定的标准回复 ID。
 * @param[in] data 至少 length 字节的载荷；函数复制后不保留该指针。
 * @param[in] length 初始载荷长度，不超过 16；经典帧不超过 8。
 * @param[in] state 发布状态，不能为 FREE；UID/CRC 由主循环补齐载荷。
 * @return 发布成功返回 true；槽忙或载荷超出当前格式能力返回 false。
 * @note 仅不重入 RX 调用；屏障保证先发布数据再发布状态。失败不覆盖旧回复。
 */
static bool CAN_QueueReply(uint16_t identifier, const uint8_t *data,
    uint8_t length, CanReplyState state)
{
    unsigned i;
    if (can_reply.state != CAN_REPLY_FREE) {
        CAN_Count(&CAN_Diagnostics.reply_busy);
        return false;
    }
    if (length > sizeof(can_reply.data) || (!can_request_fd && length > 8U) ||
        (VECTOR_CAN_CLASSIC_ONLY && (can_request_fd || can_request_brs || length > 8U))) {
        CAN_Count(&CAN_Diagnostics.rx_rejected);
        return false;
    }
    can_reply.identifier = identifier;
    can_reply.length = length;
    can_reply.fd = can_request_fd;
    can_reply.brs = can_request_fd && can_request_brs;
    for (i = 0U; i < length; ++i) {
        can_reply.data[i] = data[i];
    }
    CANMsg.can_tx_en = true;
    motor_hw_outer_barrier();
    can_reply.state = state;
    return true;
}
/* CAN_REPLY_RUNTIME_END */

extern MotorControl_TypeDef MotorControl;
extern ModeNow_TypeDef ModeLast;
extern FOC_TypeDef FOC;
extern Encoder_TypeDef OnBoard_Encoder;

/** @brief CAN RX 内停机改号；先提交过滤器，成功后才发布软件节点。 */
static bool CAN_SetNode(float data)
{
    uint8_t node;
    if (!isfinite(data) || data < 0.0f || data > 7.0f ||
        data != (float)(uint8_t)data || MotorControl.ModeNow != Motor_Disable ||
        ModeLast != Motor_Disable || !motor_hw_phase_outputs_disabled()) {
        CAN_Count(&CAN_Diagnostics.node_rejected);
        return false;
    }
    node = (uint8_t)data;
    if (node == CANMsg.node_id) {
        return true;
    }
    if (!comm_hw_can_set_node_filter(node)) {
        CAN_Count(&CAN_Diagnostics.node_filter_failed);
        return false;
    }
    /* 同一 CAN RX 不重入；ADC 保持可抢占，新接收帧在本次 ISR 退出后处理。 */
    CANMsg.node_id = node;
    (void)Param_SetSpeedLimit(fminf(MotorControl.speed_limit, Param_SpeedLimitRadS(node)));
    return true;
}

/** @brief CAN 参数在线路上的数值编码。 */
typedef enum {
	CAN_VALUE_FLOAT32,
	CAN_VALUE_MILLI_I32,
	CAN_VALUE_CENTI_I32,
	CAN_VALUE_MILLI_I16,
} CanValueEncoding;

/**
 * @brief 返回写命令参数的线路编码。
 * @param param_id 参数 ID。
 * @return 位置/速度/电流参数的定点编码，其他参数返回遗留 float32。
 */
static CanValueEncoding CAN_CommandEncoding(CAN_PARAM_ID param_id)
{
	switch (param_id) {
	case CAN_SET_CURRENT:
	case CAN_SET_CURRENT_CAL:
	case CAN_SET_CURRENT_LIMIT:
		return CAN_VALUE_MILLI_I16;
	case CAN_SET_POS:
		return CAN_VALUE_MILLI_I32;
	case CAN_SET_SPEED:
	case CAN_SET_SPEED_LIMIT:
	case CAN_SET_SPEED_ACC:
	case CAN_SET_SPEED_DEC:
	case CAN_SET_POS_ACC:
	case CAN_SET_POS_DEC:
	case CAN_SET_POS_MAXSPEED:
		return CAN_VALUE_CENTI_I32;
	default:
		return CAN_VALUE_FLOAT32;
	}
}

/**
 * @brief 返回回复参数的线路编码。
 * @param param_id 回复参数 ID。
 * @return 位置/速度/电流参数的定点编码，其他参数返回遗留 float32。
 */
static CanValueEncoding CAN_ReplyEncoding(CAN_PARAM_ID param_id)
{
	switch (param_id) {
	case CAN_GET_CURRENT_SET:
	case CAN_GET_CURRENT_CAL:
	case CAN_GET_CURRENT_LIMIT:
	case CAN_GET_IBUS:
	case CAN_GET_IA:
	case CAN_GET_IB:
	case CAN_GET_IC:
	case CAN_GET_ID:
	case CAN_GET_IQ:
	case CAN_GET_FRICTION_COULOMB_POS:
	case CAN_GET_FRICTION_COULOMB_NEG:
	case CAN_GET_FRICTION_RMSE_POS:
	case CAN_GET_FRICTION_RMSE_NEG:
		return CAN_VALUE_MILLI_I16;
	case CAN_GET_POS_SET:
	case CAN_GET_POS2_FILT:
		return CAN_VALUE_MILLI_I32;
	case CAN_GET_SPEED_SET:
	case CAN_GET_SPEED_LIMIT:
	case CAN_GET_SPEED_ACC:
	case CAN_GET_SPEED_DEC:
	case CAN_GET_POS_ACC:
	case CAN_GET_POS_DEC:
	case CAN_GET_POS_MAXSPEED:
	case CAN_GET_SPEED2_FILT:
		return CAN_VALUE_CENTI_I32;
	default:
		return CAN_VALUE_FLOAT32;
	}
}

/**
 * @brief 将浮点 SI 值转换为保留最小值作为无效哨兵的 int32 毫单位。
 * @param value 有限 SI 值。
 * @return 截断并饱和后的线路值；非有限值返回 INT32_MIN。
 */
static int32_t CAN_Milli32(float value)
{
	float scaled;
	if (!isfinite(value)) return INT32_MIN;
	scaled = value * 1000.0f;
	if (scaled >= 2147483648.0f) return INT32_MAX;
	if (scaled <= -2147483648.0f) return -INT32_MAX;
	return (int32_t)scaled;
}

/**
 * @brief 将浮点速度或加速度转换为保留最小值哨兵的 int32 百分一单位。
 * @param value 有限 SI 值，单位 rad/s 或 rad/s²。
 * @return 截断并饱和后的线路值；非有限值返回 INT32_MIN。
 */
static int32_t CAN_Centi32(float value)
{
	float scaled;
	if (!isfinite(value)) return INT32_MIN;
	scaled = value * 100.0f;
	if (scaled >= 2147483648.0f) return INT32_MAX;
	if (scaled <= -2147483648.0f) return -INT32_MAX;
	return (int32_t)scaled;
}

/**
 * @brief 将浮点安培值转换为保留最小值作为无效哨兵的 int16 毫安。
 * @param value 有限安培值。
 * @return 截断并饱和后的线路值；非有限值返回 INT16_MIN。
 */
static int16_t CAN_Milli16(float value)
{
	float scaled;
	if (!isfinite(value)) return INT16_MIN;
	scaled = value * 1000.0f;
	if (scaled >= 32767.0f) return INT16_MAX;
	if (scaled <= -32767.0f) return -INT16_MAX;
	return (int16_t)scaled;
}

/**
	* @brief  FDCAN1 Filter Init  
			  Stdandard ID, Range Mode 
 **/
void FDCAN1_Param_Init(void)
{
	CanMotorStatus_Init();
	comm_hw_can_start(CANMsg.node_id);
	CANMsg.baudrate = 1000;
}

/**
	* @brief  Handle CAN heartbeat disconnect protection
 **/
void CAN_DisConnect_Handle(void)
{
	/* Recompute on every supervisor tick: a previous RUN must not keep the
	 * watchdog armed after STOP. Keep any existing fault latched here. */
	CANMsg.can_hb_en = CANMsg.can_hb_set != 0U &&
		(MotorControl.ModeNow == Current_Mode ||
		 MotorControl.ModeNow == Speed_Mode ||
		 MotorControl.ModeNow == Position_Mode ||
		 MotorControl.ModeNow == Position_Impedance_Mode);
	if (!CANMsg.can_hb_en) {
		CANMsg.can_hb_count = 0U;
		return;
	}
	if (CANMsg.can_rx_en) {
		/* Saturate so a sustained disconnect cannot wrap the counter. */
		if (CANMsg.can_hb_count < CANMsg.can_hb_set) ++CANMsg.can_hb_count;
		if (CANMsg.can_hb_count >= CANMsg.can_hb_set &&
			MotorControl.ErrorNow == No_Error)
			Set_ErrorNow(CAN_DisConnect);
	}
}

/**
	* @brief  Set encoder state from CAN parameter value
	* @param  data: encoded encoder state value
 **/
void CAN_SetEncoderState(int data)
{
	(void)data;
}

/**
	* @brief  Switch CAN baudrate when baudrate setting changes
 **/
void CAN_BaudRateSwitching(void)
{
	static uint32_t baudrate_last = 1000;
	
	if(baudrate_last != CANMsg.baudrate)
    {
		comm_hw_can_set_bitrate(CANMsg.baudrate);
	}
	
	baudrate_last = CANMsg.baudrate;
}

/**
	* @brief  Get encoded encoder state
	* @retval encoded encoder state value
 **/
int CAN_GetEncoderState(void)
{
	return Encoder_IsOnline(&OnBoard_Encoder) ? 1 : 0;
}

/** @brief 返回明确实现的命令类别；0 未实现，1 查询，2 写入，3 BOOT。 */
static unsigned CAN_RequestKind(CAN_PARAM_ID param_id)
{
    switch (param_id) {
    case CAN_GET_MODE:
    case CAN_GET_CURRENT_SET:
    case CAN_GET_SPEED_SET:
    case CAN_GET_POS_SET:
    case CAN_GET_NODE_ID:
    case CAN_GET_POLEPARIS:
    case CAN_GET_ENCODER_STATE:
    case CAN_GET_ENCODER_REVERSE:
    case CAN_GET_CURRENT_CAL:
    case CAN_GET_CURRENT_LIMIT:
    case CAN_GET_SPEED_LIMIT:
    case CAN_GET_SPEED_ACC:
    case CAN_GET_SPEED_DEC:
    case CAN_GET_SPEED_KP:
    case CAN_GET_SPEED_KI:
    case CAN_GET_POS_ACC:
    case CAN_GET_POS_DEC:
    case CAN_GET_POS_MAXSPEED:
    case CAN_GET_POS_KP:
    case CAN_GET_POS_KD:
    case CAN_GET_POS_KI:
    case CAN_GET_POS_INTEGRAL_LIMIT:
    case CAN_GET_CASCADE_POS_KP:
    case CAN_GET_CASCADE_POS_KD:
    case CAN_GET_POSITION_SPEED_KP:
    case CAN_GET_POSITION_SPEED_KI:
    case CAN_GET_FRICTION_STATE:
    case CAN_GET_FRICTION_REASON:
    case CAN_GET_FRICTION_COULOMB_POS:
    case CAN_GET_FRICTION_COULOMB_NEG:
    case CAN_GET_FRICTION_VISCOUS_POS:
    case CAN_GET_FRICTION_VISCOUS_NEG:
    case CAN_GET_FRICTION_RMSE_POS:
    case CAN_GET_FRICTION_RMSE_NEG:
    case CAN_GET_FRICTION_CANDIDATE_VALID:
    case CAN_GET_FRICTION_MODEL_VALID:
    case CAN_GET_CAN_BR:
    case CAN_GET_CAN_HB:
    case CAN_GET_VBUS:
    case CAN_GET_IBUS:
    case CAN_GET_IA:
    case CAN_GET_IB:
    case CAN_GET_IC:
    case CAN_GET_ID:
    case CAN_GET_IQ:
    case CAN_GET_SPEED2_FILT:
    case CAN_GET_POS2_FILT:
    case CAN_GET_TEMP:
    case CAN_GET_RS:
    case CAN_GET_LD:
    case CAN_GET_LQ:
    case CAN_GET_FLUX:
    case CAN_GET_ERROR:
    case CAN_GET_SOFTWARE_VERSION:
    case CAN_GET_PROTOCOL_REVISION:
    case CAN_GET_STATUS_STREAM:
    case CAN_GET_LOAD_PROFILE:
    case CAN_GET_LOAD_PROFILE_REVISION:
    case CAN_GET_THERMAL_REVISION:
    case CAN_GET_POSITION_PLANNED:
    case CAN_GET_VELOCITY_PLANNED:
    case CAN_GET_VELOCITY_REFERENCE:
    case CAN_GET_TEMPERATURE_PROTECTION:
    case CAN_GET_SPEED_FF_CURRENT:
    case CAN_GET_SPEED_FF_ACCELERATION_PLANNED:
    case CAN_GET_SPEED_FF_ENABLE:
    case CAN_GET_SPEED_FF_GAIN:
    case CAN_GET_SPEED_FF_ACCELERATION:
    case CAN_GET_SPEED_FF_JERK:
    case CAN_GET_SPEED_FF_CURRENT_LIMIT:
    case CAN_GET_THERMAL_CONTINUOUS:
    case CAN_GET_THERMAL_PEAK:
    case CAN_GET_THERMAL_HEATING_TAU:
    case CAN_GET_THERMAL_DERATING_ENTER:
    case CAN_GET_THERMAL_DERATING_EXIT:
    case CAN_GET_THERMAL_TRIP:
    case CAN_GET_THERMAL_CLEAR:
    case CAN_GET_DEVICE_UID:
    case CAN_GET_PARAMETER_CRC:
        return 1U;
    case CAN_SET_CURRENT_CAL:
    case CAN_SET_SPEED_ACC:
    case CAN_SET_SPEED_DEC:
    case CAN_SET_SPEED_KP:
    case CAN_SET_SPEED_KI:
    case CAN_SET_POS_ACC:
    case CAN_SET_POS_DEC:
    case CAN_SET_POS_MAXSPEED:
    case CAN_SET_POS_KP:
    case CAN_SET_POS_KD:
    case CAN_SET_POS_KI:
    case CAN_SET_POS_INTEGRAL_LIMIT:
    case CAN_SET_CASCADE_POS_KP:
    case CAN_SET_CASCADE_POS_KD:
    case CAN_SET_POSITION_SPEED_KP:
    case CAN_SET_POSITION_SPEED_KI:
    case CAN_SET_MODE:
    case CAN_SET_CURRENT:
    case CAN_SET_SPEED:
    case CAN_SET_POS:
    case CAN_SET_NODE_ID:
    case CAN_SET_POLEPARIS:
    case CAN_SET_ENCODER_REVERSE:
    case CAN_SET_CURRENT_LIMIT:
    case CAN_SET_SPEED_LIMIT:
    case CAN_SET_CAN_BR:
    case CAN_SET_CAN_HB:
    case CAN_SET_STATUS_STREAM:
    case CAN_SET_LOAD_PROFILE:
    case CAN_SET_TEMPERATURE_PROTECTION:
    case CAN_SET_SPEED_FF_ENABLE:
    case CAN_SET_SPEED_FF_GAIN:
    case CAN_SET_SPEED_FF_ACCELERATION:
    case CAN_SET_SPEED_FF_JERK:
    case CAN_SET_SPEED_FF_CURRENT_LIMIT:
    case CAN_APPLY_FRICTION_MODEL:
    case CAN_SET_THERMAL_CONTINUOUS:
    case CAN_SET_THERMAL_PEAK:
    case CAN_SET_THERMAL_HEATING_TAU:
    case CAN_SET_THERMAL_DERATING_ENTER:
    case CAN_SET_THERMAL_DERATING_EXIT:
    case CAN_SET_THERMAL_TRIP:
    case CAN_SET_THERMAL_CLEAR:
        return 2U;
    case CAN_ENTER_BOOT: return 3U;
    default: return 0U;
    }
}

/**
 * @brief 按当前控制模式选择0x3F反馈源，仅读取既有编码器样本，不产生新采样。
 * 外部位置参考构建在mode3使用控制器同源的连续机械速度，避免旧8计数门限
 * 将低速运动报告为零；普通构建及其他模式保留原有带门限速度语义。
 * @return 当前机械速度rad/s；不改变模式、控制目标、编码器状态或通信看门狗。
 */
static float CAN_ReportedMechanicalVelocity(void)
{
#if POSITION_EXTERNAL_REFERENCE_ENABLE
    if (MotorControl.ModeNow == Position_Mode) {
        /* 只切换数据来源；后续0.01rad/s定点编码及非有限哨兵保持原协议。 */
        return Encoder_GetMecVelContinuous(&OnBoard_Encoder);
    }
#endif
    return OnBoard_Encoder.vel_mech;
}

/* CAN_MOTION_RUNTIME_BEGIN */
/** @brief CAN RX独占的锁存快照，首查询更新，后两查询使用同一设备发布序号。 */
static struct {
    float values[3]; /**< 驱动内部规划位置rad、规划速度和速度环参考rad/s。 */
    uint16_t sequence; /**< 设备快环发布序号低16位，不随查询次数改变。 */
    uint32_t latched_ms; /**< 首查询锁存时刻，超过20ms则后包失效。 */
    bool valid; /**< 锁存时模式为3、无故障且发布快照有效。 */
} can_motion_latch;

/**
 * @brief 按0x93/95/97返回同一快环快照的八字节只读分包。
 * @param[in] param_id 已校验的规划查询号；调用者为不重入CAN RX。
 * @return true表示格式合法且已入回复槽；槽满返回false并沿用忙计数。
 * @note 首包仅在槽空时更新锁存，不更改模式、目标、Flash或伺服参数。
 */
static bool CAN_ReadMotion(CAN_PARAM_ID param_id)
{
    uint8_t payload[8] = {1U, 0U, 0U, 0U, 0U, 0U, 0U, 0U};
    unsigned part = ((unsigned)param_id - CAN_GET_POSITION_PLANNED) / 2U;
    uint32_t raw;
    if (part >= 3U) { return false; }
    if (can_reply.state != CAN_REPLY_FREE) { CAN_Count(&CAN_Diagnostics.reply_busy); return false; }
    if (part == 0U) {
        can_motion_latch.valid = MotorControl.ModeNow == Position_Mode &&
            MotorControl.ErrorNow == No_Error &&
            MotorOuterLoop_GetMotionTelemetry(can_motion_latch.values, &can_motion_latch.sequence);
        can_motion_latch.latched_ms = time_hw_now_ms();
    }
    payload[1] = can_motion_latch.valid && MotorControl.ModeNow == Position_Mode &&
        MotorControl.ErrorNow == No_Error &&
        (uint32_t)(time_hw_now_ms() - can_motion_latch.latched_ms) < 20U ? 1U : 0U;
    payload[2] = (uint8_t)(can_motion_latch.sequence >> 8);
    payload[3] = (uint8_t)can_motion_latch.sequence;
    raw = FloatToIntBit(payload[1] ? can_motion_latch.values[part] : NAN);
    payload[4] = (uint8_t)(raw >> 24); payload[5] = (uint8_t)(raw >> 16);
    payload[6] = (uint8_t)(raw >> 8); payload[7] = (uint8_t)raw;
    return CAN_QueueReply((uint16_t)((CANMsg.node_id << 8) | param_id), payload, 8U, CAN_REPLY_READY);
}

/* CAN_MOTION_RUNTIME_END */

/**
 * @brief 查询现有参数或新增规划只读快照，不改变原有参数编码。
 * @param[in] param_id 已通过白名单的读取ID，由CAN RX独占调用。
 * @return true表示查询被接受，false表示未知参数；回复槽忙沿用原有规则。
 */
static bool CAN_ReadParameter(CAN_PARAM_ID param_id)
{
    float value;
    if (param_id == CAN_GET_POSITION_PLANNED || param_id == CAN_GET_VELOCITY_PLANNED ||
        param_id == CAN_GET_VELOCITY_REFERENCE) { return CAN_ReadMotion(param_id); }
    switch (param_id) {
    case CAN_GET_MODE: value = (float)MotorControl.ModeNow; break;
    case CAN_GET_CURRENT_SET: value = MotorControl.iqRef; break;
    case CAN_GET_SPEED_SET: value = MotorControl.speedRef; break;
    case CAN_GET_POS_SET: value = MotorControl.posRef; break;
    case CAN_GET_NODE_ID: value = (float)CANMsg.node_id; break;
    case CAN_GET_POLEPARIS: value = (float)MotorControl.motor_pole_pairs; break;
    case CAN_GET_ENCODER_STATE: value = (float)CAN_GetEncoderState(); break;
    case CAN_GET_ENCODER_REVERSE: value = (float)OnBoard_Encoder.reverse; break;
    case CAN_GET_CURRENT_CAL: value = MotorControl.calib_current; break;
    case CAN_GET_CURRENT_LIMIT: value = MotorControl.current_limit; break;
    case CAN_GET_SPEED_LIMIT: value = MotorControl.speed_limit; break;
    case CAN_GET_SPEED_ACC: value = MotorControl.speedAcc; break;
    case CAN_GET_SPEED_DEC: value = MotorControl.speedDec; break;
    case CAN_GET_SPEED_KP: value = MotorControl.speed_Kp; break;
    case CAN_GET_SPEED_KI: value = MotorControl.speed_Ki; break;
    case CAN_GET_POS_ACC: value = MotorControl.posAcc; break;
    case CAN_GET_POS_DEC: value = MotorControl.posDec; break;
    case CAN_GET_POS_MAXSPEED: value = MotorControl.pos_maxspeed; break;
    case CAN_GET_POS_KP: value = MotorControl.pos_Kp; break;
    case CAN_GET_POS_KD: value = MotorControl.pos_Kd; break;
    case CAN_GET_POS_KI: value = MotorControl.pos_Ki; break;
    case CAN_GET_POS_INTEGRAL_LIMIT: value = MotorControl.pos_integral_limit; break;
    case CAN_GET_CASCADE_POS_KP: value = MotorControl.cascade_pos_Kp; break;
    case CAN_GET_CASCADE_POS_KD: value = MotorControl.cascade_pos_Kd; break;
    case CAN_GET_POSITION_SPEED_KP: value = MotorControl.position_speed_Kp; break;
    case CAN_GET_POSITION_SPEED_KI: value = MotorControl.position_speed_Ki; break;
    case CAN_GET_FRICTION_STATE: value = (float)FocFrictionIdentification_GetState(); break;
    case CAN_GET_FRICTION_REASON: value = (float)FocFrictionIdentification_GetReason(); break;
    case CAN_GET_FRICTION_COULOMB_POS: value = FocFrictionIdentification_GetResult()->coulomb_pos_a; break;
    case CAN_GET_FRICTION_COULOMB_NEG: value = FocFrictionIdentification_GetResult()->coulomb_neg_a; break;
    case CAN_GET_FRICTION_VISCOUS_POS: value = FocFrictionIdentification_GetResult()->viscous_pos_a_per_rad_s; break;
    case CAN_GET_FRICTION_VISCOUS_NEG: value = FocFrictionIdentification_GetResult()->viscous_neg_a_per_rad_s; break;
    case CAN_GET_FRICTION_RMSE_POS: value = FocFrictionIdentification_GetResult()->rmse_pos_a; break;
    case CAN_GET_FRICTION_RMSE_NEG: value = FocFrictionIdentification_GetResult()->rmse_neg_a; break;
    case CAN_GET_FRICTION_CANDIDATE_VALID: value = FocFrictionIdentification_GetResult()->valid ? 1.0f : 0.0f; break;
    case CAN_GET_FRICTION_MODEL_VALID: value = MotorControl.friction_model_valid ? 1.0f : 0.0f; break;
    case CAN_GET_CAN_BR: value = (float)CANMsg.baudrate; break;
    case CAN_GET_CAN_HB: value = (float)CANMsg.can_hb_set; break;
    case CAN_GET_VBUS: value = FOC.Vbus_filt; break;
    case CAN_GET_IBUS: value = FOC.Ibus_filt; break;
    case CAN_GET_IA: value = FOC.Ia; break;
    case CAN_GET_IB: value = FOC.Ib; break;
    case CAN_GET_IC: value = FOC.Ic; break;
    case CAN_GET_ID: value = FOC.Id; break;
    case CAN_GET_IQ: value = FOC.Iq; break;
    case CAN_GET_SPEED2_FILT: value = CAN_ReportedMechanicalVelocity(); break;
    case CAN_GET_POS2_FILT: value = OnBoard_Encoder.theta_mech; break;
    case CAN_GET_TEMP: value = FOC.temp; break;
    case CAN_GET_RS: value = MotorControl.motor_phase_resistance; break;
    case CAN_GET_LD: value = MotorControl.motor_d_inductance; break;
    case CAN_GET_LQ: value = MotorControl.motor_q_inductance; break;
    case CAN_GET_FLUX: value = MotorControl.motor_flux; break;
    case CAN_GET_ERROR: value = (float)MotorControl.ErrorNow; break;
    case CAN_GET_SOFTWARE_VERSION: value = 0.0f; break;
    case CAN_GET_PROTOCOL_REVISION: value = (float)CAN_PARAMETER_FORMAT_REVISION; break;
    case CAN_GET_STATUS_STREAM: value = (float)CanMotorStatus_Rate(); break;
    case CAN_GET_LOAD_PROFILE: value = (float)MotorControl_GetLoadProfile(); break;
    case CAN_GET_LOAD_PROFILE_REVISION: value = 1.0f; break;
    case CAN_GET_THERMAL_REVISION: value = 3.0f; break;
    case CAN_GET_TEMPERATURE_PROTECTION: value = MotorThermalControl_ProtectionEnabled() ? 1.0f : 0.0f; break;
    case CAN_GET_SPEED_FF_CURRENT: value = MotorControl.speed_ff_state.current_a; break;
    case CAN_GET_SPEED_FF_ACCELERATION_PLANNED: value = MotorControl.speed_ff_state.acceleration_rad_s2; break;
    case CAN_GET_SPEED_FF_ENABLE: value = MotorControl.speed_ff_config.enabled ? 1.0f : 0.0f; break;
    case CAN_GET_SPEED_FF_GAIN: value = MotorControl.speed_ff_config.gain_a_per_rad_s2; break;
    case CAN_GET_SPEED_FF_ACCELERATION: value = MotorControl.speed_ff_config.acceleration_limit_rad_s2; break;
    case CAN_GET_SPEED_FF_JERK: value = MotorControl.speed_ff_config.jerk_limit_rad_s3; break;
    case CAN_GET_SPEED_FF_CURRENT_LIMIT: value = MotorControl.speed_ff_config.current_limit_a; break;
    case CAN_GET_THERMAL_CONTINUOUS:
    case CAN_GET_THERMAL_PEAK:
    case CAN_GET_THERMAL_HEATING_TAU:
    case CAN_GET_THERMAL_DERATING_ENTER:
    case CAN_GET_THERMAL_DERATING_EXIT:
    case CAN_GET_THERMAL_TRIP:
    case CAN_GET_THERMAL_CLEAR:
    {
        MotorThermalParameter parameter = (MotorThermalParameter)
            (((unsigned)param_id - CAN_GET_THERMAL_CONTINUOUS) / 2U);
        if (!MotorThermalControl_GetParameter(parameter, &value)) { return false; }
        if (parameter >= MOTOR_THERMAL_DERATING_ENTER) { value *= 100.0f; }
        break;
    }
    default: return false;
    }
    CAN_SendMessage_Update(param_id, value);
    return true;
}

/**
 * @brief 校验并执行参数请求；只有业务接受才返回 true。
 * @param[in] param_id 明确实现的查询或写入编号。
 * @param[in] data 已按线路单位解码的数值；查询忽略该占位值。
 * @return 查询有效或写入成功返回 true；拒绝时不改变模式或目标。
 * @note 不负责喂狗；模式准入仍委托控制所有者，可保留其安全故障诊断。
 */
bool CAN_ReceiveMessage_Update(CAN_PARAM_ID param_id, float data)
{
    unsigned kind = CAN_RequestKind(param_id);
    if (kind == 1U) { return CAN_ReadParameter(param_id); }
    if (kind != 2U) { return false; }
    /* 扩展写命令自行校验并保留历史 NAK；普通写入先拒绝非有限数值。 */
    if (!isfinite(data) && param_id != CAN_SET_STATUS_STREAM && param_id != CAN_SET_LOAD_PROFILE &&
        param_id != CAN_SET_TEMPERATURE_PROTECTION &&
        !(param_id >= CAN_SET_THERMAL_CONTINUOUS && param_id <= CAN_SET_THERMAL_CLEAR) &&
        !(param_id >= CAN_SET_SPEED_FF_ENABLE && param_id <= CAN_SET_SPEED_FF_CURRENT_LIMIT)) {
        return false;
    }
    switch (param_id) {
    case CAN_SET_CURRENT_CAL:
        if (!(data >= 0.0f && data <= CURRENT_CALIB_LIMIT_MAX_A)) { return false; }
        MotorControl.calib_current = data;
        return true;
    case CAN_SET_SPEED_ACC:
        if (!(data >= 0.0f && data <= 1000.0f * _2PI)) { return false; }
        MotorControl.speedAcc = data;
        return true;
    case CAN_SET_SPEED_DEC:
        if (!(data >= 0.0f && data <= 1000.0f * _2PI)) { return false; }
        MotorControl.speedDec = data;
        return true;
    case CAN_SET_SPEED_KP:
        if (!(data >= 0.01f && data <= 2.0f)) { return false; }
        MotorControl.speed_Kp = data;
        return true;
    case CAN_SET_SPEED_KI:
        if (!(data >= 0.0f && data <= 2.0f)) { return false; }
        MotorControl.speed_Ki = data;
        return true;
    case CAN_SET_POS_ACC:
        if (!(data > 0.0f && data <= 200.0f * _2PI)) { return false; }
        MotorControl.posAcc = data;
        return true;
    case CAN_SET_POS_DEC:
        if (!(data > 0.0f && data <= 200.0f * _2PI)) { return false; }
        MotorControl.posDec = data;
        return true;
    case CAN_SET_POS_MAXSPEED:
        if (!(data > 0.0f && data <= POSITION_IMPEDANCE_MAX_SPEED_RPS * _2PI && data <= MotorControl.speed_limit)) { return false; }
        MotorControl.pos_maxspeed = data;
        return true;
    case CAN_SET_POS_KP:
        if (!(data >= 0.0f && data <= POSITION_IMPEDANCE_KP_MAX_A_PER_RAD)) { return false; }
        MotorControl.pos_Kp = data;
        return true;
    case CAN_SET_POS_KD:
        if (!(data >= 0.0f && data <= POSITION_IMPEDANCE_KD_MAX_A_PER_RAD_S)) { return false; }
        MotorControl.pos_Kd = data;
        return true;
    case CAN_SET_POS_KI:
        if (!(data >= 0.0f && data <= POSITION_IMPEDANCE_KI_MAX_A_PER_RAD_S)) { return false; }
        MotorControl.pos_Ki = data;
        return true;
    case CAN_SET_POS_INTEGRAL_LIMIT:
        if (!(data >= 0.0f && data <= CURRENT_COMMAND_LIMIT_MAX_A)) { return false; }
        MotorControl.pos_integral_limit = data;
        return true;
    case CAN_SET_CASCADE_POS_KP:
        if (!(data >= 0.0f && data <= CASCADE_POSITION_KP_MAX_PER_S)) { return false; }
        MotorControl.cascade_pos_Kp = data;
        return true;
    case CAN_SET_CASCADE_POS_KD:
        if (!(data >= 0.0f && data <= CASCADE_POSITION_KD_MAX)) { return false; }
        MotorControl.cascade_pos_Kd = data;
        return true;
    case CAN_SET_POSITION_SPEED_KP:
        if (!(data >= 0.0f)) { return false; }
        MotorControl.position_speed_Kp = data;
        return true;
    case CAN_SET_POSITION_SPEED_KI:
        if (!(data >= 0.0f)) { return false; }
        MotorControl.position_speed_Ki = data;
        return true;
    case CAN_SET_MODE:
        if (data < 0.0f || data >= (float)MODE_NUM || data != (float)(unsigned)data) { return false; }
        return ModeSwitch_Handle((ModeNow_TypeDef)(unsigned)data);
    case CAN_SET_CURRENT:
        if (fabsf(data) > MotorControl.current_limit || MotorControl.ErrorNow != No_Error) { return false; }
        if (MotorControl.ModeNow != Current_Mode && !ModeSwitch_Handle(Current_Mode)) { return false; }
        MotorControl.iqRef = data;
        return true;
    case CAN_SET_SPEED:
        if (fabsf(data) > MotorControl.speed_limit || MotorControl.ErrorNow != No_Error) { return false; }
        if (MotorControl.ModeNow != Speed_Mode && MotorControl.ModeNow != Sensorless_Speed_Mode &&
            !ModeSwitch_Handle(Speed_Mode)) { return false; }
        MotorControl.speedRef = data;
        return true;
    case CAN_SET_POS:
        if (MotorControl.ErrorNow != No_Error ||
            !MotorAxisProfile_AllowsPosition(&MotorControl.axis_profile, MotorControl.axis_profile_valid,
                OnBoard_Encoder.theta_mech, data)) { return false; }
        if (MotorControl.ModeNow != Position_Mode && !ModeSwitch_Handle(Position_Mode)) { return false; }
        MotorControl.posRef = data;
        MotorOuterLoop_PublishPositionTarget(data, time_hw_now_ms());
        return true;
    case CAN_SET_NODE_ID: return CAN_SetNode(data);
    case CAN_SET_POLEPARIS:
        if (data < 2.0f || data > 30.0f || data != (float)(unsigned)data) { return false; }
        MotorControl.motor_pole_pairs = (int)data;
        return true;
    case CAN_SET_ENCODER_REVERSE:
        if (MotorControl.ModeNow != Motor_Disable || (data != 0.0f && data != 1.0f)) { return false; }
        if (OnBoard_Encoder.reverse != (uint8_t)data) { MotorControl.friction_model_valid = false; }
        Encoder_SetReverse(&OnBoard_Encoder, data != 0.0f);
        return true;
    case CAN_SET_CURRENT_LIMIT:
        if (data < 0.0f || data > CURRENT_COMMAND_LIMIT_MAX_A) { return false; }
        MotorControl.current_limit = data;
        MotorControl.iqRef = constrain(MotorControl.iqRef, -data, data);
        return true;
    case CAN_SET_SPEED_LIMIT: return Param_SetSpeedLimit(data);
    case CAN_SET_CAN_BR:
        if (data != 100.0f && data != 125.0f && data != 200.0f && data != 250.0f &&
            data != 500.0f && data != 1000.0f &&
            (VECTOR_CAN_CLASSIC_ONLY || (data != 2000.0f && data != 2500.0f && data != 5000.0f))) { return false; }
        CANMsg.baudrate = (uint32_t)data;
        return true;
    case CAN_SET_CAN_HB:
        if (data != 0.0f && (data < 500.0f || data > 1000.0f || data != (float)(unsigned)data)) { return false; }
        CANMsg.can_hb_set = (uint32_t)data;
        return true;
    case CAN_APPLY_FRICTION_MODEL:
        return data == 1.0f && FocFrictionIdentification_ApplyCandidate(&MotorControl);
    case CAN_SET_STATUS_STREAM:
    {
        bool accepted = CanMotorStatus_Configure(data);
        CAN_SendMessage_Update(CAN_GET_STATUS_STREAM, accepted ? (float)CanMotorStatus_Rate() : -1.0f);
        return accepted;
    }
    case CAN_SET_LOAD_PROFILE:
    {
        bool accepted = (data == 0.0f || data == 1.0f || data == 3.0f) &&
            MotorControl_RequestLoadProfile((uint32_t)data);
        if (!accepted) { CAN_SendMessage_Update(CAN_GET_LOAD_PROFILE, -1.0f); }
        return accepted;
    }
    case CAN_SET_TEMPERATURE_PROTECTION:
    {
        bool off = motor_hw_phase_outputs_disabled();
        bool accepted = (data == 0.0f || data == 1.0f) && MotorControl.ModeNow == Motor_Disable &&
            ModeLast == Motor_Disable && off && MotorThermalControl_RequestProtection(data == 1.0f,
                (uint32_t)MotorControl.ModeNow, off);
        CAN_SendMessage_Update(CAN_GET_TEMPERATURE_PROTECTION, accepted ? data : -1.0f);
        return accepted;
    }
    case CAN_SET_THERMAL_CONTINUOUS:
    case CAN_SET_THERMAL_PEAK:
    case CAN_SET_THERMAL_HEATING_TAU:
    case CAN_SET_THERMAL_DERATING_ENTER:
    case CAN_SET_THERMAL_DERATING_EXIT:
    case CAN_SET_THERMAL_TRIP:
    case CAN_SET_THERMAL_CLEAR:
    {
        unsigned index = ((unsigned)param_id - CAN_SET_THERMAL_CONTINUOUS) / 2U;
        MotorThermalParameter parameter = (MotorThermalParameter)index;
        float value = parameter >= MOTOR_THERMAL_DERATING_ENTER ? data * 0.01f : data;
        bool accepted = MotorThermalControl_RequestParameter(parameter, value,
            (uint32_t)MotorControl.ModeNow, motor_hw_phase_outputs_disabled());
        CAN_SendMessage_Update((CAN_PARAM_ID)(CAN_GET_THERMAL_CONTINUOUS + index * 2U), accepted ? data : -1.0f);
        return accepted;
    }
    case CAN_SET_SPEED_FF_ENABLE:
    case CAN_SET_SPEED_FF_GAIN:
    case CAN_SET_SPEED_FF_ACCELERATION:
    case CAN_SET_SPEED_FF_JERK:
    case CAN_SET_SPEED_FF_CURRENT_LIMIT:
    {
        SpeedFeedforwardConfig config = MotorControl.speed_ff_config;
        bool accepted;
        switch (param_id) {
        case CAN_SET_SPEED_FF_ENABLE:
            if (data != 0.0f && data != 1.0f) {
                CAN_SendMessage_Update(CAN_GET_SPEED_FF_ENABLE, -1.0f);
                return false;
            }
            config.enabled = data == 1.0f; break;
        case CAN_SET_SPEED_FF_GAIN: config.gain_a_per_rad_s2 = data; break;
        case CAN_SET_SPEED_FF_ACCELERATION: config.acceleration_limit_rad_s2 = data; break;
        case CAN_SET_SPEED_FF_JERK: config.jerk_limit_rad_s3 = data; break;
        default: config.current_limit_a = data; break;
        }
        accepted = MotorControl_SetSpeedFeedforward(&config);
        if (accepted) { (void)CAN_ReadParameter((CAN_PARAM_ID)((unsigned)param_id + 1U)); }
        else { CAN_SendMessage_Update((CAN_PARAM_ID)((unsigned)param_id + 1U), -1.0f); }
        return accepted;
    }
    default: return false;
    }
}

/**
 * @brief 按原参数编码生成回复，并将当前请求的经典/FD/BRS 格式冻结到回复槽。
 * @param[in] param_id 回复参数号，决定 float32、定点或软件版本编码。
 * @param[in] data 回复的 SI 数值；软件版本号使用只读编译常量。
 * @note 仅不重入 RX 分派调用；槽忙时不覆盖旧帧或调试镜像，不等待硬件。
 */
void CAN_SendMessage_Update(CAN_PARAM_ID param_id, float data)
{
	uint8_t payload[4] = {0};
    uint8_t length;
	CanValueEncoding encoding = CAN_ReplyEncoding(param_id);
	if (encoding == CAN_VALUE_MILLI_I16) {
		uint16_t value = (uint16_t)CAN_Milli16(data);
		payload[0] = (uint8_t)(value >> 8);
		payload[1] = (uint8_t)value;
		length = 2U;
	} else {
		uint32_t value;
		if (param_id == CAN_GET_SOFTWARE_VERSION)
		{
			value = g_firmware_software_version;
		}
		else if (encoding == CAN_VALUE_MILLI_I32)
			value = (uint32_t)CAN_Milli32(data);
		else if (encoding == CAN_VALUE_CENTI_I32)
			value = (uint32_t)CAN_Centi32(data);
		else
			value = FloatToIntBit(data);
		payload[0] = (uint8_t)(value >> 24);
		payload[1] = (uint8_t)(value >> 16);
		payload[2] = (uint8_t)(value >> 8);
		payload[3] = (uint8_t)value;
		length = 4U;
	}
    if (CAN_QueueReply((uint16_t)((CANMsg.node_id << 8) | param_id), payload,
        length, CAN_REPLY_READY)) {
        CANMsg.tx_param_id = param_id;
        CANMsg.tx_data = data;
        memcpy(CANMsg.tx_data_u8, payload, sizeof(payload));
        CANMsg.tx_data_len = length;
    }
}

/**
 * @brief 分派本节点参数帧，并为回复记录请求的经典/FD/BRS 格式。
 * @note CAN RX 不重入；参数编码不变，仅业务接受的请求能刷新看门狗。
 * 经典 UID、BOOT 和启用 FD 状态流的请求被拒绝，不产生隐式 FD 发送。
 * 已接受的普通请求可能更新原有控制参数、刷新心跳或发布回复；不等待发送。
 */
void CANRxIRQHandler(void)
{
    CommHwCanFrame frame;
    CAN_PARAM_ID param_id;
    unsigned kind;
    uint32_t raw;
    float value;
    bool accepted = false;
    CanValueEncoding encoding;
    if (!comm_hw_can_receive(&frame) || frame.extended || frame.remote ||
        frame.identifier > 0x7FFU || (frame.identifier >> 8) != CANMsg.node_id ||
        (!frame.fd && frame.brs) || (VECTOR_CAN_CLASSIC_ONLY && (frame.fd || frame.brs))) {
        CAN_Count(&CAN_Diagnostics.rx_rejected);
        return;
    }
    param_id = (CAN_PARAM_ID)(frame.identifier & 0xFFU);
    kind = CAN_RequestKind(param_id);
    encoding = CAN_CommandEncoding(param_id);
    if (kind == 0U || BootHandoff_Pending() ||
        frame.length != (encoding == CAN_VALUE_MILLI_I16 ? 2U : 4U) ||
        ((!frame.fd || VECTOR_CAN_CLASSIC_ONLY) &&
            (param_id == CAN_GET_DEVICE_UID || param_id == CAN_ENTER_BOOT))) {
        CAN_Count(&CAN_Diagnostics.rx_rejected);
        return;
    }
    can_request_fd = frame.fd;
    can_request_brs = frame.brs;
    if (param_id == CAN_GET_DEVICE_UID || param_id == CAN_GET_PARAMETER_CRC) {
        accepted = CAN_QueueReply((uint16_t)frame.identifier, frame.data, 4U,
            param_id == CAN_GET_DEVICE_UID ? CAN_REPLY_UID : CAN_REPLY_CRC);
        value = 0.0f;
    } else if (param_id == CAN_ENTER_BOOT) {
        value = 0.0f;
        if (memcmp(frame.data, "BOOT", 4U) == 0) {
            accepted = BootHandoff_Request(CANMsg.node_id, time_hw_now_ms());
            if (accepted) { MotorControl_LockMaintenance(); motor_hw_outer_barrier(); }
        }
    } else {
        raw = (uint32_t)frame.data[0] << 8 | frame.data[1];
        if (encoding == CAN_VALUE_MILLI_I16) {
            value = raw == 0x8000U ? NAN : (float)(int16_t)raw / 1000.0f;
        } else {
            raw = (raw << 16) | (uint32_t)frame.data[2] << 8 | frame.data[3];
            if (encoding == CAN_VALUE_MILLI_I32 || encoding == CAN_VALUE_CENTI_I32) {
                value = raw == 0x80000000U ? NAN : (float)(int32_t)raw /
                    (encoding == CAN_VALUE_MILLI_I32 ? 1000.0f : 100.0f);
            } else { value = IntBitToFloat(raw); }
        }
        if (!frame.fd && param_id == CAN_SET_STATUS_STREAM && value != 0.0f) {
            accepted = false;
        } else { accepted = CAN_ReceiveMessage_Update(param_id, value); }
    }
    can_request_fd = !VECTOR_CAN_CLASSIC_ONLY;
    can_request_brs = !VECTOR_CAN_CLASSIC_ONLY;
    if (!accepted) {
        CAN_Count(&CAN_Diagnostics.rx_rejected);
        return;
    }
    /* 业务接受后才发布调试镜像和恢复通信；非法流量不能延长存活时间。 */
    CANMsg.rx_param_id = param_id;
    CANMsg.rx_data = value;
    memcpy(CANMsg.rx_data_u8, frame.data, frame.length);
    CANMsg.can_rx_en = true;
    CANMsg.can_hb_count = 0U;
    if (MotorControl.ErrorNow == CAN_DisConnect) { Set_ErrorNow(No_Error); }
}

/**
 * @brief 前台按回复槽冻结的格式发送，HAL 忙时保持整帧待下次重试。
 * @note 仅主循环调用；UID/CRC 只计算一次。成功入队后释放槽，不等同已送达。
 * FD 兼容部署下，空槽继续处理显式状态流；不修改模式、目标或参数。
 */
void CAN_SendMessage(void)
{
    uint8_t payload[16];
    unsigned i;
    uint8_t length;
    uint16_t identifier;
    if (BootHandoff_Pending()) { return; }
    /* 防止调试器保留的旧 FD 回复堵住经典队列，更不能降级截断后发送。 */
    if (VECTOR_CAN_CLASSIC_ONLY && can_reply.state != CAN_REPLY_FREE &&
        (can_reply.fd || can_reply.brs || can_reply.length > 8U || can_reply.state == CAN_REPLY_UID)) {
        CANMsg.can_tx_en = false;
        can_reply.state = CAN_REPLY_FREE;
        return;
    }
    if (can_reply.state == CAN_REPLY_FREE) {
        uint8_t status[CAN_MOTOR_STATUS_SIZE];
        if (CanMotorStatus_Prepare(time_hw_now_ms(), CANMsg.node_id,
            &identifier, status, sizeof(status)) && can_reply.state == CAN_REPLY_FREE &&
            !BootHandoff_Pending()) {
            (void)comm_hw_can_try_send_status(identifier, status, sizeof(status));
        }
        return;
    }
    motor_hw_outer_barrier();
    if (can_reply.state == CAN_REPLY_CRC || can_reply.state == CAN_REPLY_UID) {
        if (can_reply.state == CAN_REPLY_CRC) {
            uint32_t crc = boot_hw_parameters_crc();
            payload[4] = (uint8_t)(crc >> 24);
            payload[5] = (uint8_t)(crc >> 16);
            payload[6] = (uint8_t)(crc >> 8);
            payload[7] = (uint8_t)crc;
            length = 8U;
        } else {
            boot_hw_uid(payload + 4);
            length = 16U;
        }
        for (i = 4U; i < length; ++i) {
            can_reply.data[i] = payload[i];
        }
        can_reply.length = length;
        can_reply.state = CAN_REPLY_READY;
    }
    /* 本次回复的 ID、nonce 和数据均来自入槽时的请求，改号不能重定向旧回复。 */
    identifier = can_reply.identifier;
    length = can_reply.length;
    for (i = 0U; i < length; ++i) {
        payload[i] = can_reply.data[i];
    }
    if (BootHandoff_Pending()) { return; }
    if (comm_hw_can_try_send_reply_format(identifier, payload, length, can_reply.fd, can_reply.brs)) {
        CANMsg.can_tx_en = false;
        motor_hw_outer_barrier();
        can_reply.state = CAN_REPLY_FREE;
    } else {
        CAN_Count(&CAN_Diagnostics.tx_busy);
    }
}
