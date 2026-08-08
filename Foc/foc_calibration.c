#include "foc_calibration.h"

//#include <math.h>
#include <stdbool.h>
#include "utils.h"
#include "foc_errhandle.h"
#include "hw_conf.h"
#include "heap.h"
#include "foc_sensorless.h"

//int p_error_arr[MAX_MOTOR_POLE_PAIRS * SAMPLES_PER_PPAIR];
int *p_error_arr = NULL;

CalibStep_TyepeDef CalibStep = CS_NULL;

/*observer-based encoder linearization calibration parameters*/
#define OBS_CALIB_ALIGN_TIME    1.5f                    /*s, rotor align + observer converge*/
#define OBS_CALIB_RAMP_TIME     3.0f                    /*s, open-loop speed ramp duration*/
#define OBS_CALIB_SPEED         (2.0f * _PI * 20.0f)    /*electrical rad/s (~20 Hz)*/
#define OBS_CALIB_STEP_ANGLE    (_2PI / (float)SAMPLES_PER_PPAIR)

static void Encoder_Calib_Finalize(Encoder_TypeDef *Encoder, MotorControl_TypeDef *MotorControl);
static void Encoder_Calib_Abort(void);

/**
	* @brief  Finalize encoder linearization calibration
	*         compute average offset, build 128-point LUT with FIR filter,
	*         mark linearization as calibrated and save to flash
	* @param  *Encoder: encoder struct pointer
	* @param  *MotorControl: MotorControl struct pointer
 **/
static void Encoder_Calib_Finalize(Encoder_TypeDef *Encoder, MotorControl_TypeDef *MotorControl)
{
	/*calculate average offset*/
	int64_t moving_avg = 0;
	int total = MotorControl->motor_pole_pairs * SAMPLES_PER_PPAIR;
	for(int i = 0; i < total; i++)
	{
		moving_avg += p_error_arr[i];
	}
	Encoder->offset = moving_avg / total;
	
	/*FIR and map measurements to lut*/
	int window     = SAMPLES_PER_PPAIR;
	int lut_offset = p_error_arr[0] * OFFSET_LUT_NUM / Encoder->cpr;
	for(int i = 0; i < OFFSET_LUT_NUM; i++)
	{
		moving_avg = 0;
		for(int j = (-window) / 2; j < (window) / 2; j++)
		{
			int index = i * MotorControl->motor_pole_pairs * SAMPLES_PER_PPAIR / OFFSET_LUT_NUM + j;
			if(index < 0)
			{
				index += total;
			}
			else if(index > (total - 1))
			{
				index -= total;
			}
			moving_avg += p_error_arr[index];
		}
		moving_avg    = moving_avg / window;
		int lut_index = lut_offset + i;
		
		if(lut_index > (OFFSET_LUT_NUM - 1))
		{
			lut_index -= OFFSET_LUT_NUM;
		}
		Encoder->offset_lut[lut_index] = moving_avg - Encoder->offset;
	}
	
	if(p_error_arr != NULL)
	{
		HEAP_free(p_error_arr);
		p_error_arr = NULL;
	}
	
	/*angle sensor linearization calibrated*/
	Encoder->calib_flag |= ENC_CALIB_LINEARIZED;
	
	CalibStep = CS_NULL;
	PWM_TurnOnHighSides();
	Set_ModeNow(Save_Param);
}

/**
	* @brief  Abort encoder linearization calibration and release resources
 **/
static void Encoder_Calib_Abort(void)
{
	if(p_error_arr != NULL)
	{
		HEAP_free(p_error_arr);
		p_error_arr = NULL;
	}
	CalibStep = CS_NULL;
	PWM_TurnOnHighSides();
}

/**
	* @brief  Calibrate Rs Ld Lq Flux
    * @param  *FOC: FOC struct pointer
    * @param  *MotorControl: MotorControl struct pointer
 **/
void Task_Calib_R_L_Flux(FOC_TypeDef *FOC, MotorControl_TypeDef *MotorControl)
{
	static int loop_count;
	
    /*R*/
	static uint32_t R_count;
	static float duty;
	static float R_voltage[3];
	static float R_current[3];
	
    /*L*/
	static uint32_t L_count;
	static float voltage_d,voltage_q;
    static float L_voltage_pu[20];
	static float Is;
	static float Is_testd,Is_testq;
    static const float we_L = _2PI * 1000.0f;
	
	/*Flux*/
	static uint32_t Flux_count;
	static const float MAX_DUTY = 0.3f;
	static float theta_e_flux;
	static float we_flux;
	static float voltage_norm,current_norm;
	
	float time = (float) loop_count * Current_Ts;
	
	static float A_offset_sum,B_offset_sum,C_offset_sum;
	
	
	switch(CalibStep)
	{
		case CS_NULL:
			CalibStep = CS_ADC_OFFSET_START;
		break;
		
		case CS_ADC_OFFSET_START:
		{
			loop_count = 0;
			time = 0;
			CalibStep = CS_ADC_OFFSET_LOOP;
		}
		break;
		
		case CS_ADC_OFFSET_LOOP:
		{
			A_offset_sum += (float)CURRENT_ADC->IA_ADC_CHANNEL;
			B_offset_sum += (float)CURRENT_ADC->IB_ADC_CHANNEL;
			C_offset_sum += (float)CURRENT_ADC->IC_ADC_CHANNEL;
			if(time >= 1.0f)
			{
				CalibStep = CS_ADC_OFFSET_END;
			}
		}
		break;
		
		case CS_ADC_OFFSET_END:
		{
			MotorControl->A_Offset = (uint16_t)(A_offset_sum / 20000.0f);
			MotorControl->B_Offset = (uint16_t)(B_offset_sum / 20000.0f);
			MotorControl->C_Offset = (uint16_t)(C_offset_sum / 20000.0f);
			
			A_offset_sum = 0;
			B_offset_sum = 0;
			C_offset_sum = 0;
		
			if(MotorControl->A_Offset < 2018 || MotorControl->A_Offset > 2078 ||
			   MotorControl->B_Offset < 2018 || MotorControl->B_Offset > 2078 ||
			   MotorControl->C_Offset < 2018 || MotorControl->C_Offset > 2078	)
			{
				Set_ErrorNow(CurrentOffset_Error);
				CalibStep = CS_NULL;
			}
			else
			{
				CalibStep = CS_MOTOR_R_START;
			}
		}
		break;
		
		case CS_MOTOR_R_START:
		{
			loop_count  = 0;
			duty = 0.0f;
			R_voltage[0] = 0.0f;
			R_voltage[1] = 0.0f;
			R_voltage[2] = 0.0f;
			R_current[0] = 0.0f;
			R_current[1] = 0.0f;
		    R_current[2] = 0.0f;
			CalibStep  = CS_MOTOR_RA_LOOP;
		}
		break;
		
		case CS_MOTOR_RA_LOOP:
		{
			if(fast_abs(FOC->Ia) < MotorControl->calib_current)
			{
				/*add duty cycle graudually*/
				if(time >= 0.01f)
				{
					duty += 0.0002f;
					Set_A_Duty(duty);
					Set_B_Duty(0.0f);
					Set_C_Duty(0.0f);
					
					loop_count = 0;
				}
				
				if(duty >= 1)
				{
					Set_ErrorNow(Large_Phase_Resistance);
					CalibStep = CS_NULL;
					loop_count = 0;
				}
			}
			/*reach the maximum calib current*/
			else
			{
				if(++R_count <= 20000)
				{
					R_voltage[0] += FOC->Vbus_filt * (duty - 2.0f * (float)(MOS_DEADTIME / PWM_PERIOD));
					R_current[0] += fast_abs(FOC->Ia);
				}

				else
				{
					R_voltage[0] = R_voltage[0] / 20000.0f;
					R_current[0] = R_current[0] / 20000.0f;
					R_count = 0;
					duty = 0.0f;
					PWM_TurnOnHighSides();
					CalibStep = CS_MOTOR_RB_LOOP;
					loop_count = 0;
				}
			}
		}
		break;
		
		case CS_MOTOR_RB_LOOP:
		{
			if(fast_abs(FOC->Ib) < MotorControl->calib_current)
			{
				if(time >= 0.01f)
				{
					duty += 0.0002f;
					Set_B_Duty(duty);
					Set_A_Duty(0.0f);
					Set_C_Duty(0.0f);
					
					loop_count = 0;
				}
				
				if(duty >= 1)
				{
					Set_ErrorNow(Large_Phase_Resistance);
					CalibStep = CS_NULL;
					loop_count = 0;
				}
			}
			else
			{
				if(++R_count <= 20000)
				{
					R_voltage[1] += FOC->Vbus_filt * (duty - 2.0f * (float)(MOS_DEADTIME / PWM_PERIOD));
					R_current[1] += fast_abs(FOC->Ib);
				}

				else
				{
					R_voltage[1] = R_voltage[1] / 20000.0f;
					R_current[1] = R_current[1] / 20000.0f;
					R_count = 0;
					duty = 0.0f;
					PWM_TurnOnHighSides();
					CalibStep = CS_MOTOR_RC_LOOP;
					loop_count = 0;
				}
			}
		}
		break;
			
		case CS_MOTOR_RC_LOOP:
		{
			if(fast_abs(FOC->Ic) < MotorControl->calib_current)
			{
				if(time >= 0.01f)
				{
					duty += 0.0002f;
					Set_C_Duty(duty);
					Set_A_Duty(0.0f);
					Set_B_Duty(0.0f);
					
					loop_count = 0;
				}
				
				if(duty >= 1)
				{
					Set_ErrorNow(Large_Phase_Resistance);
					CalibStep = CS_NULL;
					loop_count = 0;
				}
				
			}
			else
			{
				if(++R_count <= 20000)
				{
					R_voltage[2] += FOC->Vbus_filt * (duty - 2.0f * (float)(MOS_DEADTIME / PWM_PERIOD));
					R_current[2] += fast_abs(FOC->Ic);
				}

				else
				{
					R_voltage[2] = R_voltage[2] / 20000.0f;
					R_current[2] = R_current[2] / 20000.0f;
					R_count = 0;
					duty = 0.0f;
					PWM_TurnOnHighSides();
					CalibStep = CS_MOTOR_R_END;
					loop_count = 0;
				}
			}
		}
		break;
			
		case CS_MOTOR_R_END:
		{
			/*4mohm is consideration of Rdson of MOSFET + shunt resistor + line resistance*/
			MotorControl->motor_phase_resistance = 
			(R_voltage[0] + R_voltage[1] + R_voltage[2]) / (R_current[0] + R_current[1] + R_current[2]) * 
			2.0f / 3.0f - 0.004f;
			
			R_voltage[0] = 0.0f;
			R_voltage[1] = 0.0f;
			R_voltage[2] = 0.0f;
			R_current[0] = 0.0f;
			R_current[1] = 0.0f;
		    R_current[2] = 0.0f;
		
			if(MotorControl->motor_phase_resistance > 0.2f)
			{
				Set_ErrorNow(Large_Phase_Resistance);
				CalibStep = CS_NULL;
			}
			else
			{
				MotorControl->id_Ki = MotorControl->motor_phase_resistance * 200.0f;
				MotorControl->iq_Ki = MotorControl->motor_phase_resistance * 200.0f;
				
				CalibStep = CS_MOTOR_L_START;
			}
		}				
		break;
		
		case CS_MOTOR_L_START:
		{
			loop_count  = 0;
			L_count = 0;
			
			float angle = 0;
			for(int i = 0; i < 20; i++)
			{
				angle = (float)i / 20.0f;
				L_voltage_pu[i] = fast_sin(angle);
			}
			
			CalibStep = CS_MOTOR_LD_LOOP;
		}
		break;
		
		case CS_MOTOR_LD_LOOP:
		{	
			int i = loop_count % 20;
			
			Is = fast_sqrt(FOC->Ialpha * FOC->Ialpha + FOC->Ibeta * FOC->Ibeta);
			
			if(Is < MotorControl->calib_current * 0.5f)
			{
				voltage_d += 0.0001f;
			}
			else
			{
				if(++L_count <= 10000)
				{	
					UTILS_LP_FAST(FOC->Ialpha_filt, FOC->Ialpha, 0.2f);
					UTILS_LP_FAST(FOC->Ibeta_filt, FOC->Ibeta, 0.2f);
					
					Is_testd += fast_sqrt(FOC->Ialpha_filt * FOC->Ialpha_filt + FOC->Ibeta_filt * FOC->Ibeta_filt);
				}
				else
				{
					loop_count = 0;
					L_count = 0;
					Is_testd = Is_testd / 10000.0f;
					PWM_TurnOnHighSides();
					CalibStep = CS_MOTOR_LQ_LOOP;
				}
			}
			
			/*inject sine wave along d axis*/
			FOC_Voltage(FOC, voltage_d * L_voltage_pu[i], 0, 0);
			
		}
		break;
		
		case CS_MOTOR_LQ_LOOP:
		{	
			int i = loop_count % 20;
			
			Is = fast_sqrt(FOC->Ialpha * FOC->Ialpha + FOC->Ibeta * FOC->Ibeta);
			
			if(Is < MotorControl->calib_current * 0.5f)
			{
				voltage_q += 0.0001f;
			}
			else
			{
				if(++L_count <= 10000)
				{	
					UTILS_LP_FAST(FOC->Ialpha_filt, FOC->Ialpha, 0.2f);
					UTILS_LP_FAST(FOC->Ibeta_filt, FOC->Ibeta, 0.2f);
					
					Is_testq += fast_sqrt(FOC->Ialpha_filt * FOC->Ialpha_filt + FOC->Ibeta_filt * FOC->Ibeta_filt);
				}
				else
				{
					loop_count = 0;
					L_count = 0;	
					Is_testq = Is_testq / 10000.0f;
					PWM_TurnOnHighSides();
					CalibStep = CS_MOTOR_L_END;
				}
			}
			
			/*inject sine wave along q axis*/
			FOC_Voltage(FOC, 0, voltage_q * L_voltage_pu[i], 0);
		}
		break;
		
		case CS_MOTOR_L_END:
		{
			float Ld = (voltage_d - MotorControl->motor_phase_resistance * Is_testd) / (we_L * Is_testd);
			float Lq = (voltage_q - MotorControl->motor_phase_resistance * Is_testq) / (we_L * Is_testq);
				
			MotorControl->motor_d_inductance = Ld * 2.25f;
			MotorControl->motor_q_inductance = Lq * 2.25f;
			
			loop_count = 0;
			
			if(MotorControl->motor_d_inductance > 0.0005f || MotorControl->motor_q_inductance > 0.0005f)
			{
				Set_ErrorNow(Large_Phase_Inductance);
				CalibStep = CS_NULL;
			}
			else
			{
				MotorControl->id_Kp = MotorControl->motor_d_inductance * 200.0f;
				MotorControl->iq_Kp = MotorControl->motor_q_inductance * 200.0f;
				CalibStep = CS_MOTOR_FLUX_START;
				PWM_TurnOnHighSides();
			}
		}
		break;
		
		case CS_MOTOR_FLUX_START:
		{
			loop_count  = 0;
			Flux_count = 0;
			theta_e_flux = 0.0f;
			we_flux = 0.0f;
			voltage_norm = 0.0f;
			current_norm = 0.0f;
			
			MotorControl->iqRef = MotorControl->calib_current * 0.5f;
			MotorControl->idRef = 0.0f;
			
			CalibStep = CS_MOTOR_FLUX_LOOP;
		}
		break;
		
		case CS_MOTOR_FLUX_LOOP:
		{
			theta_e_flux += we_flux * Current_Ts;
			theta_e_flux = normalizeAngle(theta_e_flux);
			
			if(fast_abs(FOC->duty) < MAX_DUTY)
			{
				we_flux += 0.01f;
			}
			else
			{
				if(++Flux_count <= 20000)
				{
					float vd = FOC->mod_d * FOC->Vbus_filt / 1.5f;
					float vq = FOC->mod_q * FOC->Vbus_filt / 1.5f;
					float id = FOC->Id;
					float iq = FOC->Iq;
					
					voltage_norm += fast_sqrt(vd * vd + vq * vq);
					current_norm += fast_sqrt(id * id + iq * iq);
				}
				else
				{
					loop_count = 0;
					Flux_count = 0;
					voltage_norm /= 20000.0f;
					current_norm /= 20000.0f;
					MotorControl->iqRef = 0.0f;
					Stop_PWM_Generate();
					CalibStep = CS_MOTOR_FLUX_END;
				}
			}
			
			FOC_Current(FOC, MotorControl, theta_e_flux, we_flux);
		}
		break;
		
		case CS_MOTOR_FLUX_END:
		{
			float L = (MotorControl->motor_d_inductance + MotorControl->motor_q_inductance) * 0.5f;
			
			MotorControl->motor_flux = (voltage_norm - MotorControl->motor_phase_resistance * current_norm) / we_flux - L * current_norm;
				
			loop_count = 0;
			CalibStep = CS_NULL;
			
			Set_ModeNow(Save_Param);
		}
		break;
		
		default:break;
	}
	loop_count ++;
}

/**
	* @brief  Calibrate encoder offset and linearization
    * @param  *FOC: FOC struct pointer
    * @param  *MotorControl: MotorControl struct pointer
    * @param  *Encoder: Encoder struct pointer 
 **/
void Task_Calib_Encoder(FOC_TypeDef *FOC, MotorControl_TypeDef *MotorControl, Encoder_TypeDef *Encoder)
{
	static int loop_count;
	
	static const float calib_phase_vel = _PI;
	
	static float phase_set;
	static float start_count;
	
    static int sample_count;
    static float   next_sample_time;
	
	float time = (float) loop_count * Current_Ts;
	
    const float voltage = MotorControl->calib_current * MotorControl->motor_phase_resistance * 3.0f / 2.0f;
	
	switch(CalibStep)
	{
		case CS_NULL:
			CalibStep = CS_DIR_START;
		break;
		
		case CS_DIR_START:
		{	
			Encoder->dir = +1;
			phase_set = 0.0f;
			FOC_Voltage(FOC, voltage, 0.0f, phase_set);
			if(time >= 2.0f)
			{
				start_count = (float) Encoder->shadow_count;
				CalibStep = CS_DIR_LOOP;
				break;
			}
		}
		break;
		
		case CS_DIR_LOOP:
		{
			phase_set += calib_phase_vel * Current_Ts;
			FOC_Voltage(FOC, voltage, 0.0f, phase_set);
			if(phase_set >= 4.0f * _2PI)
				CalibStep = CS_DIR_END;
		}
		break;
		
		case CS_DIR_END:
		{
			int32_t diff = Encoder->shadow_count - start_count;
			
			/*judge the direction of encoder*/
			/*to ensure that positive Q current produces torque*/
			/*in the positive direction wrt the position sensor*/
			if(diff > 0)
				Encoder->dir = +1;
			else
				Encoder->dir = -1;
			
			CalibStep = CS_ENCODER_START;
		}
		break;
		
		case CS_ENCODER_START:
		{
			if(p_error_arr == NULL) 
				p_error_arr = HEAP_malloc(SAMPLES_PER_PPAIR * MotorControl->motor_pole_pairs * sizeof(int));
			
			phase_set        = 0;
			loop_count       = 0;
			sample_count     = 0;
			next_sample_time = 0;
			CalibStep        = CS_ENCODER_CW_LOOP;
		}
		break;
		
		case CS_ENCODER_CW_LOOP:
		{
			if (sample_count < (MotorControl->motor_pole_pairs * SAMPLES_PER_PPAIR)) 
			{
				if (time > next_sample_time) 
				{
					next_sample_time += _2PI / ((float) SAMPLES_PER_PPAIR * calib_phase_vel);

					int count_ref = (phase_set * (float)Encoder->cpr) / (_2PI * (float) MotorControl->motor_pole_pairs);
					int error     = Encoder->raw - count_ref;
					error += Encoder->cpr * (error < 0);
					p_error_arr[sample_count] = error;

					sample_count++;
				}

				phase_set += calib_phase_vel * Current_Ts;
			} 
			else 
			{
				phase_set -= calib_phase_vel * Current_Ts;
				loop_count = 0;
				sample_count--;
				next_sample_time = 0;
				CalibStep = CS_ENCODER_CCW_LOOP;
				break;
			}
			FOC_Voltage(FOC, voltage, 0.0f, phase_set);
		}
		break;
		
		case CS_ENCODER_CCW_LOOP:
		{
			if (sample_count >= 0) 
			{
				if (time > next_sample_time) 
				{
					next_sample_time += _2PI / ((float) SAMPLES_PER_PPAIR * calib_phase_vel);

					int count_ref = (phase_set * (float)Encoder->cpr) / (_2PI * (float) MotorControl->motor_pole_pairs);
					int error     = Encoder->raw - count_ref;
					error += Encoder->cpr * (error < 0);
					p_error_arr[sample_count] = (p_error_arr[sample_count] + error) / 2;

					sample_count--;
				}

				phase_set -= calib_phase_vel * Current_Ts;
			} 
			else 
			{
				PWM_TurnOnLowSides();
				CalibStep = CS_ENCODER_END;
				break;
			}
			FOC_Voltage(FOC, voltage, 0.0f, phase_set);
		}
		break;
		
		case CS_ENCODER_END:
		{
			loop_count            = 0;
			sample_count          = 0;
			next_sample_time      = 0;
			Encoder_Calib_Finalize(Encoder, MotorControl);
		}
		break;
		
		default:break;
	}
	
	loop_count++;
}

/**
	* @brief  Calibrate encoder offset and linearization using flux observer
	*         drive the motor open-loop at a moderate electrical speed and use
	*         the observer electrical angle (back-EMF based) as the position
	*         reference instead of the injected phase, so that the rotor
	*         slip / torque-angle lag is not included in the LUT error
    * @param  *FOC: FOC struct pointer
    * @param  *MotorControl: MotorControl struct pointer
    * @param  *Encoder: Encoder struct pointer
    * @param  *Fluxobserver: flux observer struct pointer
 **/
void Task_Calib_EncoderObserver(FOC_TypeDef *FOC, MotorControl_TypeDef *MotorControl, Encoder_TypeDef *Encoder, Fluxobserver_TypeDef *Fluxobserver)
{
	static int loop_count;
	
	/*direction detection*/
	static float phase_set;
	static float start_count;
	
	/*observer drive and reference*/
	static float obs_phase;
	static float obs_omega;
	static float theta_cum;
	static float theta_last;
	static float theta_ref_start;
	static bool  sampling_started;
	static int   last_idx;
	static int   total_samples;
	static uint32_t settle_count;
	
	float time = (float)loop_count * Current_Ts;
	
	const float voltage = MotorControl->calib_current * MotorControl->motor_phase_resistance * 3.0f / 2.0f;
	
	/*flux observer must be updated every cycle in this mode*/
	Fluxobserver_Update(FOC, MotorControl, Fluxobserver);
	
	switch(CalibStep)
	{
		case CS_NULL:
			loop_count = 0;
			CalibStep = CS_DIR_START;
		break;
		
		/*----------------- direction detection -----------------*/
		case CS_DIR_START:
		{	
			Encoder->dir = +1;
			phase_set = 0.0f;
			FOC_Voltage(FOC, voltage, 0.0f, phase_set);
			if(time >= 2.0f)
			{
				start_count = (float) Encoder->shadow_count;
				CalibStep = CS_DIR_LOOP;
				break;
			}
		}
		break;
		
		case CS_DIR_LOOP:
		{
			phase_set += _PI * Current_Ts;
			FOC_Voltage(FOC, voltage, 0.0f, phase_set);
			if(phase_set >= 4.0f * _2PI)
				CalibStep = CS_DIR_END;
		}
		break;
		
		case CS_DIR_END:
		{
			int32_t diff = Encoder->shadow_count - start_count;
			
			/*positive Q current must produce torque in positive direction*/
			if(diff > 0)
				Encoder->dir = +1;
			else
				Encoder->dir = -1;
			
			CalibStep = CS_OBS_ALIGN;
		}
		break;
		
		/*----------------- align rotor and let observer converge -----------------*/
		case CS_OBS_ALIGN:
		{
			/*motor parameters are filled externally, they are required here*/
			if(MotorControl->motor_pole_pairs <= 0 || MotorControl->motor_pole_pairs > MAX_MOTOR_POLE_PAIRS)
			{
				Set_ErrorNow(PolePairs_Error);
				CalibStep = CS_NULL;
				break;
			}
			if(MotorControl->motor_phase_resistance <= 0.0f || MotorControl->motor_flux <= 0.0f)
			{
				Set_ErrorNow(MotorParam_Error);
				CalibStep = CS_NULL;
				break;
			}
			
			if(p_error_arr == NULL)
			{
				p_error_arr = HEAP_malloc(SAMPLES_PER_PPAIR * MotorControl->motor_pole_pairs * sizeof(int));
				if(p_error_arr == NULL)
				{
					Set_ErrorNow(Encoder_Error);
					CalibStep = CS_NULL;
					break;
				}
			}
			
			loop_count       = 0;
			obs_phase        = 0.0f;
			obs_omega        = 0.0f;
			theta_cum        = 0.0f;
			theta_last       = Observer_GetElePhase(Fluxobserver);
			theta_ref_start  = 0.0f;
			sampling_started = false;
			last_idx         = -1;
			total_samples    = 0;
			settle_count     = 0;
			
			/*align rotor on d-axis, observer converges to a steady angle*/
			FOC_Voltage(FOC, voltage, 0.0f, 0.0f);
			
			CalibStep = CS_OBS_ALIGN_LOOP;
		}
		break;
		
		case CS_OBS_ALIGN_LOOP:
		{
			/*hold d-axis voltage, rotor aligned and observer converges*/
			FOC_Voltage(FOC, voltage, 0.0f, 0.0f);
			
			if(time >= OBS_CALIB_ALIGN_TIME)
			{
				theta_last = Observer_GetElePhase(Fluxobserver);
				CalibStep  = CS_OBS_RAMP_CW;
			}
		}
		break;
		
		/*----------------- ramp open-loop speed to target -----------------*/
		case CS_OBS_RAMP_CW:
		{
			/*unwrap observer angle into continuous cumulative electrical angle*/
			float theta_e = Observer_GetElePhase(Fluxobserver);
			float delta   = theta_e - theta_last;
			theta_last    = theta_e;
			if(delta > _PI) delta -= _2PI;
			else if(delta < -_PI) delta += _2PI;
			theta_cum    += delta;
			
			obs_omega += (OBS_CALIB_SPEED / OBS_CALIB_RAMP_TIME) * Current_Ts;
			if(obs_omega >= OBS_CALIB_SPEED)
			{
				obs_omega        = OBS_CALIB_SPEED;
				settle_count     = 0;
				sampling_started = false;
				total_samples    = SAMPLES_PER_PPAIR * MotorControl->motor_pole_pairs;
				CalibStep        = CS_OBS_SAMPLE_CW;
			}
			
			obs_phase += obs_omega * Current_Ts;
			/*boost voltage with back-EMF estimate to keep the rotor in sync*/
			FOC_Voltage(FOC, voltage + MotorControl->motor_flux * obs_omega, 0.0f, obs_phase);
		}
		break;
		
		/*----------------- sample one mechanical turn (CW) -----------------*/
		case CS_OBS_SAMPLE_CW:
		{
			float theta_e = Observer_GetElePhase(Fluxobserver);
			float delta   = theta_e - theta_last;
			theta_last    = theta_e;
			if(delta > _PI) delta -= _2PI;
			else if(delta < -_PI) delta += _2PI;
			theta_cum    += delta;
			
			obs_phase += OBS_CALIB_SPEED * Current_Ts;
			FOC_Voltage(FOC, voltage + MotorControl->motor_flux * OBS_CALIB_SPEED, 0.0f, obs_phase);
			
			/*wait for observer to lock onto the target speed*/
			if(fast_abs(Fluxobserver->omega_e) < OBS_CALIB_SPEED * 0.5f)
			{
				if(++settle_count >= (uint32_t)(1.0f / Current_Ts))
				{
					Set_ErrorNow(Encoder_Error);
					Encoder_Calib_Abort();
				}
				break;
			}
			settle_count = 0;
			
			/*start the sampling reference at the current observer angle*/
			if(sampling_started == false)
			{
				sampling_started = true;
				theta_ref_start  = theta_cum;
				last_idx         = -1;
			}
			
			float theta_rel = theta_cum - theta_ref_start;
			int   idx       = (int)(theta_rel / OBS_CALIB_STEP_ANGLE);
			
			if(idx >= 0 && idx < total_samples && idx > last_idx)
			{
				float count_ref = (theta_rel * (float)Encoder->cpr) / (_2PI * (float)MotorControl->motor_pole_pairs);
				int   error     = Encoder->raw - (int)count_ref;
				error += Encoder->cpr * (error < 0);
				p_error_arr[idx] = error;
				last_idx         = idx;
			}
			
			/*one full mechanical turn sampled, ramp down and reverse*/
			if(idx >= total_samples)
				CalibStep = CS_OBS_RAMP_CCW;
		}
		break;
		
		/*----------------- ramp speed down and reverse -----------------*/
		case CS_OBS_RAMP_CCW:
		{
			float theta_e = Observer_GetElePhase(Fluxobserver);
			float delta   = theta_e - theta_last;
			theta_last    = theta_e;
			if(delta > _PI) delta -= _2PI;
			else if(delta < -_PI) delta += _2PI;
			theta_cum    += delta;
			
			obs_omega -= 2.0f * OBS_CALIB_SPEED / OBS_CALIB_RAMP_TIME * Current_Ts;
			if(obs_omega <= -OBS_CALIB_SPEED)
			{
				obs_omega    = -OBS_CALIB_SPEED;
				settle_count = 0;
				last_idx     = total_samples;
				CalibStep    = CS_OBS_SAMPLE_CCW;
			}
			
			obs_phase += obs_omega * Current_Ts;
			FOC_Voltage(FOC, voltage + MotorControl->motor_flux * fast_abs(obs_omega), 0.0f, obs_phase);
		}
		break;
		
		/*----------------- sample one mechanical turn (CCW), average into LUT -----------------*/
		case CS_OBS_SAMPLE_CCW:
		{
			float theta_e = Observer_GetElePhase(Fluxobserver);
			float delta   = theta_e - theta_last;
			theta_last    = theta_e;
			if(delta > _PI) delta -= _2PI;
			else if(delta < -_PI) delta += _2PI;
			theta_cum    += delta;
			
			obs_phase -= OBS_CALIB_SPEED * Current_Ts;
			FOC_Voltage(FOC, voltage + MotorControl->motor_flux * OBS_CALIB_SPEED, 0.0f, obs_phase);
			
			/*wait for observer to lock onto the reverse speed*/
			if(fast_abs(Fluxobserver->omega_e) < OBS_CALIB_SPEED * 0.5f)
			{
				if(++settle_count >= (uint32_t)(1.0f / Current_Ts))
				{
					Set_ErrorNow(Encoder_Error);
					Encoder_Calib_Abort();
				}
				break;
			}
			settle_count = 0;
			
			float theta_rel = theta_cum - theta_ref_start;
			int   idx       = (int)(theta_rel / OBS_CALIB_STEP_ANGLE);
			
			if(theta_rel >= 0.0f && idx >= 0 && idx < total_samples && idx < last_idx)
			{
				float count_ref = (theta_rel * (float)Encoder->cpr) / (_2PI * (float)MotorControl->motor_pole_pairs);
				int   error     = Encoder->raw - (int)count_ref;
				error += Encoder->cpr * (error < 0);
				p_error_arr[idx] = (p_error_arr[idx] + error) / 2;
				last_idx         = idx;
			}
			
			/*reverse pass finished*/
			if(theta_rel < 0.0f || idx < 0)
				CalibStep = CS_OBS_END;
		}
		break;
		
		case CS_OBS_END:
		{
			Encoder_Calib_Finalize(Encoder, MotorControl);
		}
		break;
		
		default:break;
	}
	
	loop_count++;
}

/**
	* @brief  Calibrate current sensor ADC zero offset
	*         automatically executed at power-up
	* @param  *FOC: FOC struct pointer
	* @param  *MotorControl: MotorControl struct pointer
 **/
void Task_Calib_CurrentOffset(FOC_TypeDef *FOC, MotorControl_TypeDef *MotorControl)
{
	static uint32_t offset_count;
	static uint32_t A_offset_sum,B_offset_sum,C_offset_sum;
	
	A_offset_sum += CURRENT_ADC->IA_ADC_CHANNEL;
	B_offset_sum += CURRENT_ADC->IB_ADC_CHANNEL;
	C_offset_sum += CURRENT_ADC->IC_ADC_CHANNEL;
	offset_count++;
	
	/*accumulate 1s (20000 cycles @ 20kHz)*/
	if(offset_count >= 20000)
	{
		MotorControl->A_Offset = (uint16_t)(A_offset_sum / offset_count);
		MotorControl->B_Offset = (uint16_t)(B_offset_sum / offset_count);
		MotorControl->C_Offset = (uint16_t)(C_offset_sum / offset_count);
		
		offset_count = 0;
		A_offset_sum = B_offset_sum = C_offset_sum = 0;
		
		/*check offset is around half scale (2048)*/
		// if(MotorControl->A_Offset < 2018 || MotorControl->A_Offset > 2078 ||
		//    MotorControl->B_Offset < 2018 || MotorControl->B_Offset > 2078 ||
		//    MotorControl->C_Offset < 2018 || MotorControl->C_Offset > 2078)
		// {
		// 	Set_ErrorNow(CurrentOffset_Error);
		// }
		
		Set_ModeNow(Motor_Disable);
	}
}

