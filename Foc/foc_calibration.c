#include "foc_calibration.h"

//#include <math.h>
#include <stdbool.h>
#include <limits.h>
#include <stdint.h>
#include <string.h>
#include "utils.h"
#include "foc_errhandle.h"
#include "hw_conf.h"
#include "heap.h"
#include "foc_sensorless.h"

static int32_t *p_error_sum = NULL;
static uint16_t *calibration_samples = NULL;

CalibStep_TyepeDef CalibStep = CS_NULL;

#define OBS_CALIB_ALIGN_TIME        1.5f
#define ENC_ZERO_ALIGN_TIME         1.5f
#define ENC_ZERO_SAMPLE_TIME        1.0f
#define OBS_CALIB_RAMP_TIME         3.0f
#define OBS_CALIB_SPEED             (2.0f * _PI * 20.0f)
#define OBS_CALIB_TIMEOUT_FACTOR    1.5f
#define OBS_CALIB_UNLOCK_TIMEOUT_TICKS FOC_FREQ

static void Encoder_Calib_ReleaseSamples(void)
{
	if (p_error_sum != NULL)
	{
		HEAP_free(p_error_sum);
		p_error_sum = NULL;
	}
	if (calibration_samples != NULL)
	{
		HEAP_free(calibration_samples);
		calibration_samples = NULL;
	}
}

static void Encoder_Calib_Abort(void)
{
	Encoder_Calib_ReleaseSamples();
	CalibStep = CS_NULL;
	PWM_TurnOnHighSides();
}

static bool Encoder_Calib_Finalize(Encoder_TypeDef *Encoder)
{
	int32_t correction_previous;
	int32_t correction_shift = 0;
	int64_t average_correction;
	uint16_t lut_index;

	if (p_error_sum == NULL || calibration_samples == NULL)
		return false;

	for (lut_index = 0U; lut_index < ENCODER_OFFSET_LUT_SIZE; ++lut_index)
	{
		if (calibration_samples[lut_index] == 0U)
			return false;
	}

	correction_previous = (int16_t)(p_error_sum[0] / (int32_t)calibration_samples[0]);
	p_error_sum[0] = correction_previous;
	for (lut_index = 1U; lut_index < ENCODER_OFFSET_LUT_SIZE; ++lut_index)
	{
		int32_t correction = (int16_t)(p_error_sum[lut_index] / (int32_t)calibration_samples[lut_index]);
		while (correction - correction_previous > ENCODER_Q15_HALF_TURN)
			correction -= (int32_t)ENCODER_Q15_CPR;
		while (correction - correction_previous < -ENCODER_Q15_HALF_TURN)
			correction += (int32_t)ENCODER_Q15_CPR;
		p_error_sum[lut_index] = correction;
		correction_previous = correction;
	}

	average_correction = 0;
	for (lut_index = 0U; lut_index < ENCODER_OFFSET_LUT_SIZE; ++lut_index)
		average_correction += p_error_sum[lut_index];
	average_correction /= (int64_t)ENCODER_OFFSET_LUT_SIZE;
	while (average_correction > INT16_MAX)
	{
		average_correction -= (int32_t)ENCODER_Q15_CPR;
		correction_shift += (int32_t)ENCODER_Q15_CPR;
	}
	while (average_correction < INT16_MIN)
	{
		average_correction += (int32_t)ENCODER_Q15_CPR;
		correction_shift -= (int32_t)ENCODER_Q15_CPR;
	}

	for (lut_index = 0U; lut_index < ENCODER_OFFSET_LUT_SIZE; ++lut_index)
	{
		int32_t correction = p_error_sum[lut_index] - correction_shift;
		if (correction < INT16_MIN || correction > INT16_MAX)
			return false;
		Encoder->linearization_lut_q15[lut_index] = (int16_t)correction;
	}

	Encoder->electrical_zero_q15 = 0U;
	Encoder->mechanical_zero_q15 = 0U;
	Encoder->calib_flag &= (uint8_t)~(ENC_CALIB_ELECTRICAL_ZERO | ENC_CALIB_MECHANICAL_ZERO);
	Encoder->calib_flag |= ENC_CALIB_LINEARIZED;
	Encoder_Calib_ReleaseSamples();
	Encoder_ResetVelocity(Encoder);
	CalibStep = CS_NULL;
	PWM_TurnOnHighSides();
	Set_ModeNow(Save_Param);
	return true;
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
 * @brief Build the 1024-point linearization LUT with observer alignment and one forward sweep.
 *        The observer defines the mechanical angle reference; no encoder direction detection or reverse averaging is used.
 */
void Task_Calib_EncoderObserver(FOC_TypeDef *FOC, MotorControl_TypeDef *MotorControl,
	Encoder_TypeDef *Encoder, Fluxobserver_TypeDef *Fluxobserver)
{
	static uint32_t loop_count;
	static float drive_phase;
	static float drive_omega;
	static float observer_theta_last;
	static float observer_theta_unwrapped;
	static float sample_theta_start;
	static bool sampling_started;
	static uint32_t observer_unlock_ticks;
	float time = (float)loop_count * Current_Ts;
	float voltage;
	float observer_theta;
	float observer_delta;
	float theta_relative;
	float required_theta;

	if (!Encoder_IsOnline(Encoder))
	{
		Set_ErrorNow(Encoder_Error);
		Encoder_Calib_Abort();
		return;
	}
	if (MotorControl->motor_pole_pairs <= 0)
	{
		Set_ErrorNow(PolePairs_Error);
		Encoder_Calib_Abort();
		return;
	}
	if (MotorControl->motor_phase_resistance <= 0.0f || MotorControl->motor_flux <= 0.0f ||
		MotorControl->calib_current <= 0.0f)
	{
		Set_ErrorNow(MotorParam_Error);
		Encoder_Calib_Abort();
		return;
	}

	voltage = MotorControl->calib_current * MotorControl->motor_phase_resistance * 1.5f;
	required_theta = _2PI * (float)MotorControl->motor_pole_pairs;

	switch (CalibStep)
	{
		case CS_NULL:
			loop_count = 0U;
			CalibStep = CS_OBS_ALIGN;
			break;

		case CS_OBS_ALIGN:
			if (p_error_sum == NULL)
			{
				p_error_sum = HEAP_malloc(ENCODER_OFFSET_LUT_SIZE * sizeof(*p_error_sum));
				if (p_error_sum == NULL)
				{
					Set_ErrorNow(Encoder_Error);
					Encoder_Calib_Abort();
					return;
				}
			}
			if (calibration_samples == NULL)
			{
				calibration_samples = HEAP_malloc(ENCODER_OFFSET_LUT_SIZE * sizeof(*calibration_samples));
				if (calibration_samples == NULL)
				{
					Set_ErrorNow(Encoder_Error);
					Encoder_Calib_Abort();
					return;
				}
			}
			memset(p_error_sum, 0, ENCODER_OFFSET_LUT_SIZE * sizeof(*p_error_sum));
			memset(calibration_samples, 0, ENCODER_OFFSET_LUT_SIZE * sizeof(*calibration_samples));
			loop_count = 0U;
			drive_phase = 0.0f;
			drive_omega = 0.0f;
			observer_theta_last = Observer_GetElePhase(Fluxobserver);
			observer_theta_unwrapped = 0.0f;
			sample_theta_start = 0.0f;
			sampling_started = false;
			observer_unlock_ticks = 0U;
			FOC_Voltage(FOC, voltage, 0.0f, 0.0f);
			CalibStep = CS_OBS_ALIGN_LOOP;
			break;

		case CS_OBS_ALIGN_LOOP:
			FOC_Voltage(FOC, voltage, 0.0f, 0.0f);
			if (time >= OBS_CALIB_ALIGN_TIME)
			{
				loop_count = 0U;
				observer_theta_last = Observer_GetElePhase(Fluxobserver);
				CalibStep = CS_OBS_RAMP_CW;
			}
			break;

		case CS_OBS_RAMP_CW:
		case CS_OBS_SAMPLE_CW:
			observer_theta = Observer_GetElePhase(Fluxobserver);
			observer_delta = observer_theta - observer_theta_last;
			observer_theta_last = observer_theta;
			if (observer_delta > _PI)
				observer_delta -= _2PI;
			else if (observer_delta < -_PI)
				observer_delta += _2PI;
			observer_theta_unwrapped += observer_delta;

			if (CalibStep == CS_OBS_RAMP_CW)
			{
				drive_omega += (OBS_CALIB_SPEED / OBS_CALIB_RAMP_TIME) * Current_Ts;
				if (drive_omega >= OBS_CALIB_SPEED)
				{
					drive_omega = OBS_CALIB_SPEED;
					observer_unlock_ticks = 0U;
					CalibStep = CS_OBS_SAMPLE_CW;
				}
			}

			drive_phase += drive_omega * Current_Ts;
			FOC_Voltage(FOC, voltage + MotorControl->motor_flux * drive_omega, 0.0f, drive_phase);

			if (CalibStep != CS_OBS_SAMPLE_CW)
				break;

			if (Fluxobserver->omega_e < OBS_CALIB_SPEED * 0.5f)
			{
				if (++observer_unlock_ticks >= OBS_CALIB_UNLOCK_TIMEOUT_TICKS)
				{
					Set_ErrorNow(Encoder_Error);
					Encoder_Calib_Abort();
				}
				break;
			}
			observer_unlock_ticks = 0U;

			if (!sampling_started)
			{
				sampling_started = true;
				sample_theta_start = observer_theta_unwrapped;
			}

			theta_relative = observer_theta_unwrapped - sample_theta_start;
			if (Encoder->read_status == ENCODER_READ_OK &&
				theta_relative >= 0.0f && theta_relative < required_theta)
			{
				uint16_t lut_index = Encoder->directed_q15 >> 6;
				uint16_t reference_q15 = (uint16_t)(theta_relative * ((float)ENCODER_Q15_CPR / required_theta));
				int16_t correction_q15 = (int16_t)(uint16_t)(Encoder->directed_q15 - reference_q15);
				if (calibration_samples[lut_index] < UINT16_MAX)
				{
					p_error_sum[lut_index] += correction_q15;
					calibration_samples[lut_index]++;
				}
			}
			else if (theta_relative >= required_theta)
			{
				CalibStep = CS_OBS_END;
			}
			else if (time >= OBS_CALIB_TIMEOUT_FACTOR * required_theta / OBS_CALIB_SPEED + OBS_CALIB_RAMP_TIME + OBS_CALIB_ALIGN_TIME)
			{
				Set_ErrorNow(Encoder_Error);
				Encoder_Calib_Abort();
			}
			break;

		case CS_OBS_END:
			if (!Encoder_Calib_Finalize(Encoder))
			{
				Set_ErrorNow(Encoder_Error);
				Encoder_Calib_Abort();
			}
			break;

		default:
			Encoder_Calib_Abort();
			break;
	}

	loop_count++;
}

/**
 * @brief Calibrate electrical zero using the averaged linearized Q15 angle.
 */
void Task_Calib_EleAngelOffset(FOC_TypeDef *FOC, MotorControl_TypeDef *MotorControl, Encoder_TypeDef *Encoder)
{
	static uint32_t loop_count;
	static uint32_t sample_count;
	static uint16_t sample_anchor;
	static int64_t unwrapped_sum;
	float time = (float)loop_count * Current_Ts;
	float align_current = MotorControl->calib_current;

	if (!Encoder_IsOnline(Encoder))
	{
		Set_ErrorNow(Encoder_Error);
		loop_count = 0U;
		sample_count = 0U;
		unwrapped_sum = 0;
		PWM_TurnOnHighSides();
		return;
	}
	if ((Encoder->calib_flag & ENC_CALIB_LINEARIZED) == 0U)
	{
		Set_ErrorNow(Encoder_NotCalibrated);
		return;
	}
	if (align_current <= 0.0f)
	{
		Set_ErrorNow(MotorParam_Error);
		return;
	}
	if (MotorControl->current_limit > 0.0f && align_current > MotorControl->current_limit)
		align_current = MotorControl->current_limit;

	if (loop_count == 0U)
	{
		sample_count = 0U;
		sample_anchor = Encoder->linearized_q15;
		unwrapped_sum = 0;
		Encoder->calib_flag &= (uint8_t)~ENC_CALIB_ELECTRICAL_ZERO;
		FOC_CurrentController_Reset(FOC);
	}

	MotorControl->idRef = align_current;
	MotorControl->iqRef = 0.0f;
	FOC_Current(FOC, MotorControl, 0.0f, 0.0f);

	if (time >= (ENC_ZERO_ALIGN_TIME - ENC_ZERO_SAMPLE_TIME) &&
		Encoder->read_status == ENCODER_READ_OK)
	{
		int32_t unwrapped_q15 = (int32_t)sample_anchor + (int16_t)(uint16_t)(Encoder->linearized_q15 - sample_anchor);
		unwrapped_sum += unwrapped_q15;
		sample_count++;
	}

	if (time >= ENC_ZERO_ALIGN_TIME)
	{
		bool calibrated = false;
		if (sample_count > 0U)
		{
			uint16_t electrical_zero_q15 = (uint16_t)(unwrapped_sum / (int64_t)sample_count);
			calibrated = Encoder_SetElectricalZeroQ15(Encoder, electrical_zero_q15);
		}

		MotorControl->idRef = 0.0f;
		MotorControl->iqRef = 0.0f;
		FOC_CurrentController_Reset(FOC);
		loop_count = 0U;
		sample_count = 0U;
		unwrapped_sum = 0;
		PWM_TurnOnHighSides();

		if (!calibrated)
		{
			Set_ErrorNow(Encoder_Error);
			return;
		}
		Set_ModeNow(Save_Param);
		return;
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

