#ifndef __INTERFACE_USB_H__
#define __INTERFACE_USB_H__

#include "main.h"
#include <stdbool.h>

#define COMMAND_MIN_LENGTH	8

/*USB analyze step*/
typedef enum
{
	USB_RX_NULL,
	USB_RX_HEAD,
	USB_RX_WRP,
	USB_RX_UNDERLINE,
	USB_RX_PARAM,
	USB_RX_EQUAL,
	USB_RX_SIGN,
	USB_RX_INT_DATA,
	USB_RX_POINT,
	USB_RX_FLOAT_DATA,
	USB_RX_TAIL
}USBRxStep_TypeDef;

typedef enum
{
	USB_NO_ERROR,
	USB_INCOMPLETE_DATA,
	USB_SYNTAX_ERROR,
	USB_WRITE_INVALID,
	USB_UNKNOWNED_PARAM,
	USB_DATA_INVALID,
	USB_DATA_OUT_OF_RANGE,
	USB_CYCLIC_OVERFLOW
}USBRXError_TypeDef;

/*USB param id is composed by three characters in ASCII format*/
/*for instance, "USB_MODE" is composed by m(0x6D) o(0x6F) d(0x64) */
typedef enum
{
	USB_MODE			= 0x6D6F64,
	USB_CURRENT_SET		= 0x695F73,
	USB_SPEED_SET		= 0x735F73,
	USB_POS_SET			= 0x705F73,
/***********************************/
	USB_NODE_ID			= 0x636964,
	USB_POLEPARIS		= 0x706F6C,
	USB_ENCODER_STATE 	= 0x655F73,
	USB_CURRENT_CAL		= 0x696361,
	USB_CURRENT_LIMIT	= 0x696C6D,
	USB_SPEED_LIMIT		= 0x736C6D,
	USB_SPEED_ACC		= 0x736163,
	USB_SPEED_DEC		= 0x736465,
	USB_SPEED_KP		= 0x735F70,
	USB_SPEED_KI 		= 0x735F69,
	USB_POS_ACC			= 0x706163,
	USB_POS_DEC			= 0x706465,
	USB_POS_MAXSPEED	= 0x706D73,
	USB_POS_KP			= 0x705F70,
	USB_POS_KD			= 0x705F64,
	USB_COGGING			= 0x636F67,
	USB_CAN_BR			= 0x636272,
	USB_CAN_HB			= 0x636862,
/***********************************/
	USB_VBUS			= 0x766273,
	USB_IBUS			= 0x696273,
	USB_IA				= 0x695F61,
	USB_IB				= 0x695F62,
	USB_IC				= 0x695F63,
	USB_ID				= 0x695F64,
	USB_IQ				= 0x695F71,
	USB_SPEED1_FILT		= 0x733166,
	USB_POS1_FILT		= 0x703166,
	USB_SPEED2_FILT		= 0x733266,
	USB_POS2_FILT		= 0x703266,
	USB_TEMP			= 0x746D70,
	USB_RS				= 0x6D7273,
	USB_LD				= 0x6D6C64,
	USB_LQ				= 0x6D6C71,
	USB_FLUX			= 0x6D6678,
	USB_ERROR			= 0x657272
}USB_PARAM_ID;

typedef union
{
	uint8_t str[4];
    uint32_t combined;
}USB_Param;

typedef enum 
{
	TYPE_FLOAT,
	TYPE_UINT8,
    TYPE_UINT16,
	TYPE_UINT32,
    TYPE_INT32
}VarType;

typedef struct 
{
    void *addr;
    VarType type;
    float scale;
}VarInfo;

typedef struct
{
	/*write, read, print flag 0:write 1:read 2:print*/
	uint8_t rx_write_read_print;
	/*param id*/
	USB_Param rx_param_id;
	uint8_t int_or_float;
	int rx_int_length,rx_float_length;
	/*sign of data +1 or -1*/
	float rx_sign;
	/*integer part, decimal part*/
	uint8_t rx_int_data[10],rx_dec_data[10];
	/*combined data*/
	float rx_data;
	
	uint8_t tx_en;
	char tx_str[80];

	uint8_t print_en;
	uint8_t en_channel_num;
	VarInfo p_var[5];
	uint32_t print_array[6];
}USBMsg_TypeDef;

USBRXError_TypeDef USB_CyclicAnalyze(void);
USBRXError_TypeDef USB_ReceiveMessage_Update(uint8_t w_r_p, USB_PARAM_ID param_id, float data, uint8_t int_or_float);
void USB_RxIRQHandler(uint8_t *data, uint16_t length);
void USB_SendMessage(void);
void USB_PrintProfile(void);

#endif