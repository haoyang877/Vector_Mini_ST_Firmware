#ifndef DOMAIN_FAST_MATH_H
#define DOMAIN_FAST_MATH_H

#include <stdint.h>

#define MATH_PI 			3.1415926536f
#define MATH_PI_BY_TWO 			1.5707963268f
#define MATH_PI_BY_THREE 			1.0471975512f
#define MATH_TWO_PI 			6.2831853072f
#define MATH_THREE_PI_BY_TWO 			4.7123889804f
#define MATH_ONE_BY_TWO_PI		0.1591549431f

#define MATH_SQRT_3			1.732050807f
#define MATH_ONE_BY_SQRT_3 	0.577350269f
#define MATH_TWO_BY_SQRT_3	1.154700538f
#define MATH_SQRT_3_BY_2		0.866025403f

#define FAST_MATH_LUT_SCALE	    162.97466172f

#define FAST_MATH_LOW_PASS(value, sample, filter_constant) \
	((value) -= (filter_constant) * ((value) - (sample)))

#define FAST_MATH_MOVING_AVERAGE(value, sample, count) \
	FAST_MATH_LOW_PASS(value, sample, 2.0f / ((count) + 1.0f))

typedef struct
{
	float state_n,state_n_1,state_n_2;
	float a0,a1,a2;
	float b0,b1,b2;
	float gain0,gain1;
}ButterworthFilter;

float FastMath_Clamp(float amt, float low, float high);
float FastMath_NormalizeAngle(float angle);
uint32_t FloatBits_Encode(float x);
float FloatBits_Decode(uint32_t x);
float FastMath_UpdateButterworth(float input, ButterworthFilter *filter);
float FastMath_Abs(float x);
float FastMath_Square(float x);
float FastMath_Max(float x,float y);
float FastMath_Min(float x,float y);
float FastMath_Sin(float theta);
float FastMath_Cos(float theta);
float FastMath_Atan2(float y, float x);
float FastMath_Sqrt(float x);
float FastMath_Pow(float x, int y);
float FastMath_Sign(float x);
float FastMath_CopySign(float x, float y);

#endif
