#ifndef __ENCODER_H__
#define __ENCODER_H__

#include "main.h"
#include "data_type.h"

/*Encoder type*/
typedef enum 
{	
	SENSORLESS = -1,
	TLE5012B = 0,
	MT6816 = 1,
	MT6701 = 2
}Encoder_Type;

typedef enum
{
	ON_BOARD = 0,
	EXTERNAL = 1
}Encoder_Source;

/*Encoder type*/
typedef enum 
{	
	ENCODER_DISABLE = 0,
	ENCODER_ENABLE = 1
}Encoder_Enable;

typedef struct
{
	Encoder_Type type;
	Encoder_Enable enable;
	Encoder_Source source;
	
	int resolution;
	
	/*range of one loop*/
	/*cpr = 2 ^ (resolution)*/
	int cpr;
	float one_by_cpr;
	
	int dir;
	
	/*raw data*/
    int raw;
    int count_in_cpr;
    int count_in_cpr_prev;

    int64_t shadow_count;

    /*pll use*/
    float pos_cpr_counts;
    float vel_estimate_counts;

	float turns;
	/*mechanical angle (r)*/
    float pos;
	/*mechanical anglular velocity (r/s)*/
    float vel;

	/*electrical angle (rad)*/
    float theta_elec;
	/*electrical angular velocity (rad/s)*/
    float vel_elec;

	/*mechanical angle (rad)*/
	float theta_mech;
	/*mechanical anglular velocity (rad/s)*/
	float vel_mech;
	
    float pll_kp;
    float pll_ki;
    float interpolation;
    float snap_threshold;
	
	int offset;
	int offset_lut[128];
	int zero_count;
	
	int disconnect_count;
}Encoder_TypeDef;


#define brd_enc_spi	 hspi2
#define BRD_ENC_SPI  SPI2

#define ext_enc_spi  hspi1
#define EXT_ENC_SPI	 SPI1

#define BRD_ENC_CS_ENABLE  BRD_ENC_CS_GPIO_Port->BSRR = (uint32_t)BRD_ENC_CS_Pin << 16U
#define BRD_ENC_CS_DISABLE BRD_ENC_CS_GPIO_Port->BSRR = BRD_ENC_CS_Pin   

#define EXT_ENC_CS_ENABLE  EXT_ENC_CS_GPIO_Port->BSRR = (uint32_t)EXT_ENC_CS_Pin << 16U
#define EXT_ENC_CS_DISABLE EXT_ENC_CS_GPIO_Port->BSRR = EXT_ENC_CS_Pin        		

void Encoder_ParamInit(Encoder_TypeDef *Encoder);

uint16_t ReadTLE5012B_Raw(Encoder_Source source);
uint16_t ReadMT6816_Raw(Encoder_Source source);
uint16_t ReadMT6701_Raw(Encoder_Source source);
uint16_t ReadSPIEncoder_Raw(Encoder_TypeDef *Encoder);

void Encoder_Update(MotorControl_TypeDef *MotorControl, Encoder_TypeDef *Encoder);
float Encoder_GetEleVel(Encoder_TypeDef *Encoder);
float Encoder_GetMecVel(Encoder_TypeDef *Encoder);
float Encoder_GetElePhase(Encoder_TypeDef *Encoder);
float Encoder_GetMecPos(Encoder_TypeDef *Encoder);
float Encoder_GetCountInCPR_Ratio(Encoder_TypeDef *Encoder);

void Encoder_ChangeDetect(Encoder_TypeDef *Encoder1, Encoder_TypeDef *Encoder2);

void Task_Set_ZeroPosition(MotorControl_TypeDef *MotorControl, Encoder_TypeDef *Encoder);


#endif