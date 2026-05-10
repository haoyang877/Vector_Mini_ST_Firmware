#ifndef __INTERFACE_CAN_H__
#define __INTERFACE_CAN_H__

#include "main.h"
#include "data_type.h"

typedef enum
{
	CAN_SET_MODE			= 0x00,
	CAN_GET_MODE			= 0x01,
	CAN_SET_CURRENT			= 0x02,
	CAN_GET_CURRENT_SET		= 0x03,
	CAN_SET_SPEED			= 0x04,
	CAN_GET_SPEED_SET		= 0x05,
	CAN_SET_POS				= 0x06,
	CAN_GET_POS_SET			= 0x07,
/***********************************/
	CAN_SET_NODE_ID			= 0x08,
	CAN_GET_NODE_ID			= 0x09,
	CAN_SET_POLEPARIS		= 0x0A,
	CAN_GET_POLEPARIS		= 0x0B,
	CAN_SET_ENCODER_STATE 	= 0x0C,
	CAN_GET_ENCODER_STATE	= 0x0D,
	CAN_SET_CURRENT_CAL		= 0x0E,
	CAN_GET_CURRENT_CAL		= 0x0F,
	CAN_SET_CURRENT_LIMIT	= 0x10,
	CAN_GET_CURRENT_LIMIT	= 0x11,
	CAN_SET_SPEED_LIMIT		= 0x12,
	CAN_GET_SPEED_LIMIT		= 0x13,
	CAN_SET_SPEED_ACC 		= 0x14,
	CAN_GET_SPEED_ACC		= 0x15,
	CAN_SET_SPEED_DEC		= 0x16,
	CAN_GET_SPEED_DEC		= 0x17,
	CAN_SET_SPEED_KP		= 0x18,
	CAN_GET_SPEED_KP		= 0x19,
	CAN_SET_SPEED_KI 		= 0x1A,
	CAN_GET_SPEED_KI		= 0x1B,
	CAN_SET_POS_ACC			= 0x1C,
	CAN_GET_POS_ACC			= 0x1D,
	CAN_SET_POS_DEC			= 0x1E,
	CAN_GET_POS_DEC			= 0x1F,
	CAN_SET_POS_MAXSPEED	= 0x20,
	CAN_GET_POS_MAXSPEED	= 0x21,
	CAN_SET_POS_KP			= 0x22,
	CAN_GET_POS_KP			= 0x23,
	CAN_SET_POS_KD			= 0x24,
	CAN_GET_POS_KD			= 0x25,
	CAN_SET_COGGING			= 0x26,
	CAN_GET_COGGING			= 0x27,
	CAN_SET_CAN_BR			= 0x28,
	CAN_GET_CAN_BR			= 0x29,
	CAN_SET_CAN_HB			= 0x2A,
	CAN_GET_CAN_HB			= 0x2B,
/***********************************/
	CAN_SET_VBUS			= 0x2C,
	CAN_GET_VBUS			= 0x2D,
	CAN_SET_IBUS			= 0x2E,
	CAN_GET_IBUS			= 0x2F,
	CAN_SET_IA				= 0x30,
	CAN_GET_IA				= 0x31,
	CAN_SET_IB				= 0x32,
	CAN_GET_IB				= 0x33,	
	CAN_SET_IC				= 0x34,
	CAN_GET_IC				= 0x35,	
	CAN_SET_ID				= 0x36,
	CAN_GET_ID				= 0x37,	
	CAN_SET_IQ				= 0x38,
	CAN_GET_IQ				= 0x39,	
	CAN_SET_SPEED1_FILT		= 0x3A,
	CAN_GET_SPEED1_FILT		= 0x3B,	
	CAN_SET_POS1_FILT		= 0x3C,
	CAN_GET_POS1_FILT		= 0x3D,
	CAN_SET_SPEED2_FILT		= 0x3E,
	CAN_GET_SPEED2_FILT		= 0x3F,	
	CAN_SET_POS2_FILT		= 0x40,
	CAN_GET_POS2_FILT		= 0x41,
	CAN_SET_TEMP			= 0x42,
	CAN_GET_TEMP			= 0x43,
	CAN_SET_RS				= 0x44,
	CAN_GET_RS				= 0x45,
	CAN_SET_LD				= 0x46,
	CAN_GET_LD				= 0x47,
	CAN_SET_LQ				= 0x48,
	CAN_GET_LQ				= 0x49,
	CAN_SET_FLUX			= 0x4A,
	CAN_GET_FLUX			= 0x4B,
	CAN_SET_ERROR			= 0x4C,
	CAN_GET_ERROR			= 0x4D,
}CAN_PARAM_ID;

typedef struct
{
	/*node ID*/
	uint8_t node_id;
	/*CAN baudrate (kbps)*/
	uint32_t baudrate;
	/*parameter ID*/
	CAN_PARAM_ID rx_param_id,tx_param_id;
	float rx_data,tx_data;
	uint8_t rx_data_u8[4],tx_data_u8[4];
	bool can_rx_en;
	bool can_tx_en;
	bool can_hb_en;
	/*heart beat timeout setting (ms)*/
	uint32_t can_hb_set;
	uint32_t can_hb_count;
}CANMsg_TypeDef;

void FDCAN1_Param_Init(void);
void CAN_DisConnect_Handle(void);
void CAN_BaudRateSwitching(void);
void CAN_ReceiveMessage_Update(CAN_PARAM_ID param_id, float data);
void CAN_SendMessage_Update(CAN_PARAM_ID param_id, float data);
void CANRxIRQHandler(void);
void CAN_SendMessage(void);
#endif