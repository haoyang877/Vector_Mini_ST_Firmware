#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* Compile the production adapter against a minimal register fake. Generated
 * STM32 headers are deliberately bypassed; this file is not in the host runner. */
#define __ADC_H__
#define __TIM_H__

typedef struct
{
	volatile uint32_t CCER;
	volatile uint32_t BDTR;
	volatile uint32_t CCMR1;
	volatile uint32_t CCMR2;
	volatile uint32_t ARR;
	volatile uint32_t CCR1;
	volatile uint32_t CCR2;
	volatile uint32_t CCR3;
} TIM_TypeDef;

typedef struct
{
	volatile uint32_t ISR;
	volatile uint32_t JSQR;
	volatile uint32_t JDR1;
	volatile uint32_t JDR2;
	volatile uint32_t JDR3;
	volatile uint32_t JDR4;
} ADC_TypeDef;

typedef struct { TIM_TypeDef *Instance; } TIM_HandleTypeDef;
typedef struct { ADC_TypeDef *Instance; } ADC_HandleTypeDef;

static TIM_TypeDef FakeTim1;
static ADC_TypeDef FakeAdc2;
TIM_HandleTypeDef htim1;
ADC_HandleTypeDef hadc2;

#define TIM1 (&FakeTim1)
#define ADC2 (&FakeAdc2)
#define TIM_CCER_CC1E  (UINT32_C(1) << 0)
#define TIM_CCER_CC1NE (UINT32_C(1) << 2)
#define TIM_CCER_CC2E  (UINT32_C(1) << 4)
#define TIM_CCER_CC2NE (UINT32_C(1) << 6)
#define TIM_CCER_CC3E  (UINT32_C(1) << 8)
#define TIM_CCER_CC3NE (UINT32_C(1) << 10)
#define TIM_CCER_CC4E  (UINT32_C(1) << 12)
#define TIM_BDTR_MOE   (UINT32_C(1) << 15)
#define TIM_CCMR1_OC1PE (UINT32_C(1) << 3)
#define TIM_CCMR1_OC2PE (UINT32_C(1) << 11)
#define TIM_CCMR2_OC3PE (UINT32_C(1) << 3)
#define ADC_ISR_OVR    (UINT32_C(1) << 4)
#define ADC_ISR_JEOS   (UINT32_C(1) << 6)
#define ADC_ISR_JQOVF  (UINT32_C(1) << 10)
#define ADC_JSQR_JL_Msk UINT32_C(0x3)
#define ADC_JSQR_JEXTSEL_Msk (UINT32_C(0x1F) << 2)
#define ADC_JSQR_JEXTEN_Msk (UINT32_C(0x3) << 7)
#define ADC_JSQR_JSQ1_Pos 9U
#define ADC_JSQR_JSQ1_Msk (UINT32_C(0x1F) << ADC_JSQR_JSQ1_Pos)
#define ADC_JSQR_JSQ2_Pos 15U
#define ADC_JSQR_JSQ2_Msk (UINT32_C(0x1F) << ADC_JSQR_JSQ2_Pos)
#define ADC_JSQR_JSQ3_Pos 21U
#define ADC_JSQR_JSQ3_Msk (UINT32_C(0x1F) << ADC_JSQR_JSQ3_Pos)
#define ADC_JSQR_JSQ4_Pos 27U
#define ADC_JSQR_JSQ4_Msk (UINT32_C(0x1F) << ADC_JSQR_JSQ4_Pos)
#define ADC_EXTERNALTRIGINJEC_T1_CC4 \
	((UINT32_C(1) << 2) | (UINT32_C(1) << 7))
#define ADC_EXTERNALTRIGINJECCONV_EDGE_FALLING (UINT32_C(2) << 7)
#define CLEAR_BIT(register_, bits_) ((register_) &= ~(bits_))
#define SET_BIT(register_, bits_) ((register_) |= (bits_))

#include "../../Firmware/Platform/Stm32G431/motor_drive_tim1_adc2_stm32g431.c"

#define CHECK(condition_) do { \
	if (!(condition_)) { \
		(void)fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #condition_); \
		return 1; \
	} \
} while (0)

static BspMotorDriveEndpointCapabilities MakeCapabilities(void)
{
	BspMotorDriveEndpointCapabilities capabilities;

	(void)memset(&capabilities, 0, sizeof(capabilities));
	capabilities.endpoint_id = 1U;
	capabilities.availability = BSP_ENDPOINT_AVAILABLE;
	capabilities.supported_current_sense_topologies =
		BSP_CURRENT_SENSE_TOPOLOGY_BIT(
			BSP_CURRENT_SENSE_LOW_SIDE_THREE_SHUNT);
	capabilities.current_sensor_capacity = BSP_MOTOR_PHASE_COUNT;
	capabilities.supports_synchronized_sampling = true;
	capabilities.supported_sampling_modes =
		BSP_CURRENT_SAMPLING_MODE_BIT(BSP_CURRENT_SAMPLING_MODE_FIXED);
	return capabilities;
}

int main(void)
{
	BspMotorDriveEndpointCapabilities capabilities = MakeCapabilities();
	BspMotorDriveEndpointCapabilities invalid_capabilities = capabilities;
	BspMotorDriveConfiguration configuration;
	BspMotorDriveCycleCommand command;
	BspMotorDriveSample sample;
	BspMotorDrivePort port;
	uint32_t preserved_ccer;

	(void)memset(&configuration, 0, sizeof(configuration));
	(void)memset(&command, 0, sizeof(command));
	(void)memset(&FakeTim1, 0, sizeof(FakeTim1));
	(void)memset(&FakeAdc2, 0, sizeof(FakeAdc2));
	htim1.Instance = TIM1;
	hadc2.Instance = ADC2;
	FakeTim1.ARR = 4250U;
	FakeAdc2.JSQR = ADC_JSQR_JL_Msk |
		(ADC_EXTERNALTRIGINJEC_T1_CC4 & ADC_JSQR_JEXTSEL_Msk) |
		ADC_EXTERNALTRIGINJECCONV_EDGE_FALLING |
		(UINT32_C(13) << ADC_JSQR_JSQ1_Pos) |
		(UINT32_C(3) << ADC_JSQR_JSQ2_Pos) |
		(UINT32_C(5) << ADC_JSQR_JSQ3_Pos) |
		(UINT32_C(17) << ADC_JSQR_JSQ4_Pos);
	FakeTim1.BDTR = TIM_BDTR_MOE;
	invalid_capabilities.supported_sampling_modes = 0U;
	CHECK(!MotorDriveTim1Adc2Stm32g431_CreatePort(&invalid_capabilities,
		&port));
	CHECK((FakeTim1.BDTR & TIM_BDTR_MOE) == 0U);

	FakeTim1.BDTR = TIM_BDTR_MOE;
	invalid_capabilities.supported_sampling_modes =
		BSP_CURRENT_SAMPLING_MODE_BIT(BSP_CURRENT_SAMPLING_MODE_DYNAMIC);
	CHECK(!MotorDriveTim1Adc2Stm32g431_CreatePort(&invalid_capabilities,
		&port));
	CHECK((FakeTim1.BDTR & TIM_BDTR_MOE) == 0U);

	CHECK(MotorDriveTim1Adc2Stm32g431_CreatePort(&capabilities, &port));
	configuration.current_sense_topology =
		BSP_CURRENT_SENSE_LOW_SIDE_THREE_SHUNT;
	configuration.pwm_frequency_hz = 20000U;
	configuration.sampling_mode = BSP_CURRENT_SAMPLING_MODE_DYNAMIC;
	FakeTim1.CCER = TIM_CCER_CC4E | MOTOR_OUTPUT_ENABLE_MASK;
	FakeTim1.BDTR = TIM_BDTR_MOE;
	CHECK(port.initialize_safe(port.context, &configuration) ==
		BSP_RESULT_NOT_SUPPORTED);
	CHECK((FakeTim1.BDTR & TIM_BDTR_MOE) == 0U);
	CHECK(FakeTim1.CCER == TIM_CCER_CC4E);

	configuration.sampling_mode = BSP_CURRENT_SAMPLING_MODE_UNSPECIFIED;
	FakeTim1.CCER = TIM_CCER_CC4E | MOTOR_OUTPUT_ENABLE_MASK;
	FakeTim1.BDTR = TIM_BDTR_MOE;
	CHECK(port.initialize_safe(port.context, &configuration) ==
		BSP_RESULT_NOT_SUPPORTED);
	CHECK((FakeTim1.BDTR & TIM_BDTR_MOE) == 0U);
	CHECK(FakeTim1.CCER == TIM_CCER_CC4E);

	configuration.sampling_mode = BSP_CURRENT_SAMPLING_MODE_FIXED;
	configuration.fixed_sample_count = BSP_MOTOR_PHASE_COUNT;
	configuration.fixed_direct_phase_currents = BSP_MOTOR_PHASE_ALL;
	CHECK(port.initialize_safe(port.context, &configuration) == BSP_RESULT_OK);
	CHECK((FakeTim1.CCMR1 & (TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE)) ==
		(TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE));
	CHECK((FakeTim1.CCMR2 & TIM_CCMR2_OC3PE) != 0U);
	preserved_ccer = FakeTim1.CCER;
	CHECK(port.arm(port.context) == BSP_RESULT_NOT_READY);
	CHECK((FakeTim1.BDTR & TIM_BDTR_MOE) == 0U);

	command.sampling.mode = BSP_CURRENT_SAMPLING_MODE_FIXED;
	command.sampling.cycle_valid_phase_currents = BSP_MOTOR_PHASE_ALL;
	command.sampling.sequence = 42U;
	command.pwm.phase_duty[0] = 0.25f;
	command.pwm.phase_duty[1] = 0.5f;
	command.pwm.phase_duty[2] = 1.0f;
	CHECK(port.commit_cycle(port.context, &command) == BSP_RESULT_OK);
	CHECK(FakeTim1.CCR1 == 1062U && FakeTim1.CCR2 == 2125U &&
		FakeTim1.CCR3 == 4250U);
	CHECK(port.read_sample(port.context, &sample) == BSP_RESULT_NOT_READY);
	FakeAdc2.JDR1 = 101U;
	FakeAdc2.JDR2 = 202U;
	FakeAdc2.JDR3 = 303U;
	FakeAdc2.JDR4 = 2800U;
	FakeAdc2.ISR = ADC_ISR_JEOS;
	MotorDriveTim1Adc2Stm32g431_OnInjectedSequenceComplete();
	CHECK(port.read_sample(port.context, &sample) == BSP_RESULT_OK);
	CHECK(sample.sequence == 42U && sample.current_sample_count == 3U &&
		sample.valid_phase_currents == BSP_MOTOR_PHASE_ALL);
	CHECK(port.read_sample(port.context, &sample) == BSP_RESULT_NOT_READY);
	CHECK(port.arm(port.context) == BSP_RESULT_OK);
	CHECK((FakeTim1.CCER & MOTOR_OUTPUT_ENABLE_MASK) ==
		MOTOR_OUTPUT_ENABLE_MASK);
	CHECK((FakeTim1.CCER & TIM_CCER_CC4E) == preserved_ccer);
	CHECK((FakeTim1.BDTR & TIM_BDTR_MOE) != 0U);

	command.pwm.phase_duty[1] = NAN;
	CHECK(port.commit_cycle(port.context, &command) ==
		BSP_RESULT_INVALID_ARGUMENT);
	CHECK(FakeTim1.CCR1 == 1062U && FakeTim1.CCR2 == 2125U &&
		FakeTim1.CCR3 == 4250U);
	command.pwm.phase_duty[1] = -0.1f;
	CHECK(port.commit_cycle(port.context, &command) ==
		BSP_RESULT_INVALID_ARGUMENT);
	command.pwm.phase_duty[1] = 1.1f;
	CHECK(port.commit_cycle(port.context, &command) ==
		BSP_RESULT_INVALID_ARGUMENT);
	command.pwm.phase_duty[1] = 0.5f;
	command.sampling.mode = BSP_CURRENT_SAMPLING_MODE_UNSPECIFIED;
	CHECK(port.commit_cycle(port.context, &command) ==
		BSP_RESULT_NOT_SUPPORTED);
	command.sampling.mode = BSP_CURRENT_SAMPLING_MODE_DYNAMIC;
	CHECK(port.commit_cycle(port.context, &command) ==
		BSP_RESULT_NOT_SUPPORTED);
	command.sampling.mode = BSP_CURRENT_SAMPLING_MODE_FIXED;
	command.sampling.sampling_point_count = 1U;
	CHECK(port.commit_cycle(port.context, &command) ==
		BSP_RESULT_NOT_SUPPORTED);

	command.sampling.sampling_point_count = 0U;
	command.sampling.sequence = 43U;
	CHECK(port.commit_cycle(port.context, &command) == BSP_RESULT_OK);
	FakeAdc2.ISR = ADC_ISR_JEOS | ADC_ISR_OVR | ADC_ISR_JQOVF;
	MotorDriveTim1Adc2Stm32g431_OnInjectedSequenceComplete();
	CHECK(port.read_sample(port.context, &sample) == BSP_RESULT_OK);
	CHECK(sample.current_sample_count == 3U &&
		sample.current_raw[0] == 101U &&
		sample.current_raw[1] == 202U &&
		sample.current_raw[2] == 303U &&
		sample.bus_voltage_raw == 2800U);
	CHECK(sample.sequence == 43U);
	CHECK(sample.status == (BSP_MOTOR_DRIVE_SAMPLE_VALID |
		BSP_MOTOR_DRIVE_SAMPLE_ADC_OVERRUN));
	CHECK((port.read_faults(port.context) &
		BSP_MOTOR_DRIVE_FAULT_SAMPLING) != 0U);

	port.disable_immediate(port.context);
	port.disable_immediate(port.context);
	CHECK((FakeTim1.BDTR & TIM_BDTR_MOE) == 0U);
	CHECK(port.disarm(port.context) == BSP_RESULT_OK);
	CHECK((FakeTim1.CCER & MOTOR_OUTPUT_ENABLE_MASK) == 0U);
	CHECK((FakeTim1.CCER & TIM_CCER_CC4E) != 0U);
	(void)puts("PASS motor_drive_tim1_adc2_stm32g431_fake");
	return 0;
}
